#!/usr/bin/env python3
import serial
import time
import smbus
import statistics
import math
from enum import Enum

DEBUG = True  # set to False to disable debug prints
DEBUG_INTERVAL = 1.0  # seconds between prints
_last_debug_print = 0

# -------------------------------
# Machine State Enum
# -------------------------------
class MachineState(Enum):
    IDLE = 0
    HEATING = 1
    MAINTAINING = 2
    COOLING = 3
    SHUTDOWN = 4
    FAULT = 5

# -------------------------------
# Config Loader
# -------------------------------
def load_config(path="/etc/supersmoker.conf"):
    cfg = {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, val = line.split("=", 1)
            cfg[key.strip()] = val.strip()
    return cfg

# -------------------------------
# MAD helpers
# -------------------------------
def median_absolute_deviation(data, median_val=None):
    if not data:
        return 0.0
    if median_val is None:
        median_val = statistics.median(data)
    return statistics.median([abs(x - median_val) for x in data])

def trimmed_mean(data, trim_fraction):
    if not data:
        raise ValueError("trimmed_mean: empty data")
    n = len(data)
    k = int(math.floor(trim_fraction * n))
    if 2 * k >= n:
        return statistics.mean(data)
    s = sorted(data)
    return statistics.mean(s[k:n - k])

# -------------------------------
# Thermocouple read + MAD + EMA
# -------------------------------
ema_temp = None

def read_raw_temp_c(bus, i2c_addr, slope, offset):
    """Read raw counts from I2C and apply linear calibration."""
    data = bus.read_i2c_block_data(i2c_addr, 1, 2)
    raw = (data[0] << 8) | data[1]
    corrected_c = slope * raw + offset
    return corrected_c, raw

def read_thermocouple_celsius(bus, i2c_addr, sample_rate, trim_fraction, mad_threshold,
                              min_samples, slope, offset, window_seconds=1.0, ema_alpha=0.25):
    global ema_temp
    samples = []
    start = time.perf_counter()
    sleep_interval = 1.0 / sample_rate

    while time.perf_counter() - start < window_seconds:
        try:
            temp_c, _ = read_raw_temp_c(bus, i2c_addr, slope, offset)
            samples.append(temp_c)
        except Exception:
            pass
        time.sleep(sleep_interval)

    if len(samples) < min_samples:
        return None  # Not enough valid samples

    median_val = statistics.median(samples)
    mad = median_absolute_deviation(samples, median_val)

    if mad > 0:
        survivors = [x for x in samples if abs(x - median_val) <= mad_threshold * mad]
    else:
        survivors = samples

    if len(survivors) < min_samples:
        survivors = samples

    clean_avg = trimmed_mean(survivors, trim_fraction)

    # EMA smoothing
    if ema_temp is None:
        ema_temp = clean_avg
    else:
        ema_temp = ema_alpha * clean_avg + (1 - ema_alpha) * ema_temp

    return ema_temp

# -------------------------------
# Machine State Logic
# -------------------------------
def determine_machine_state(temp, setpoint, cfg):
    try:
        heat_band = float(cfg["heat_band"])
        maintain_band = float(cfg["maintain_band"])
        cool_band = float(cfg["cool_band"])
    except KeyError:
        heat_band, maintain_band, cool_band = 10, 5, 15

    if temp is None:
        return MachineState.FAULT
    if setpoint <= 0:
        return MachineState.IDLE
    diff = temp - setpoint

    if diff < -heat_band:
        return MachineState.HEATING
    elif abs(diff) <= maintain_band:
        return MachineState.MAINTAINING
    elif diff > cool_band:
        return MachineState.COOLING
    return MachineState.MAINTAINING

# -------------------------------
# Serial response
# -------------------------------
def send_data(ser, temp, setpoint, state, cfg):
    kp = cfg["kp"]
    ki = cfg["ki"]
    kd = cfg["kd"]
    outmin = cfg["outmin"]
    outmax = cfg["outmax"]

    lines = [
        f"TEMP={temp:.2f}",
        f"SETPOINT={setpoint:.2f}",
        f"STATE={state.value}",
        f"KP={kp}",
        f"KI={ki}",
        f"KD={kd}",
        f"OUTMIN={outmin}",
        f"OUTMAX={outmax}",
        "END\n",
    ]
    ser.write("\n".join(lines).encode("ascii"))

# -------------------------------
# Main loop
# -------------------------------
def main():
    cfg = load_config()

    # Require serial port
    if "serial_port" not in cfg:
        raise RuntimeError("serial_port must be defined in supersmoker.conf")
    port = cfg["serial_port"]

    # Require I2C/calibration keys
    required_keys = [
        "i2c_addr", "i2c_bus", "sample_rate", "trim_fraction",
        "mad_threshold", "min_samples",
        "raw_0c", "raw_100c", "actual_0c", "actual_100c"
    ]
    for key in required_keys:
        if key not in cfg:
            raise RuntimeError(f"{key} must be defined in supersmoker.conf")

    I2C_ADDR = int(cfg["i2c_addr"], 0)
    I2C_BUS = int(cfg["i2c_bus"])
    SAMPLE_RATE = float(cfg["sample_rate"])
    TRIM_FRACTION = float(cfg["trim_fraction"])
    MAD_THRESHOLD = float(cfg["mad_threshold"])
    MIN_SAMPLES = int(cfg["min_samples"])

    raw_0C = float(cfg["raw_0c"])
    raw_100C = float(cfg["raw_100c"])
    actual_0C = float(cfg["actual_0c"])
    actual_100C = float(cfg["actual_100c"])
    slope = (actual_100C - actual_0C) / (raw_100C - raw_0C)
    offset = actual_0C - slope * raw_0C

    bus = smbus.SMBus(I2C_BUS)
    ser = serial.Serial(port, 115200, timeout=0.1)

    print("Supersmoker Supervisor started (PLC-poll model)")

    global _last_debug_print

    while True:
        try:
            line = ser.readline().decode("ascii", errors="ignore").strip()
            if line == "GET":
                cfg = load_config()  # reload config each poll
                setpoint = float(cfg["setpoint"])
                temp = read_thermocouple_celsius(
                    bus, I2C_ADDR, SAMPLE_RATE, TRIM_FRACTION, MAD_THRESHOLD,
                    MIN_SAMPLES, slope, offset, window_seconds=0.5
                )
                state = determine_machine_state(temp, setpoint, cfg)
                send_data(ser, temp if temp is not None else 0.0, setpoint, state, cfg)

                if DEBUG:
                    now = time.time()
                    if now - _last_debug_print >= DEBUG_INTERVAL:
                        print(
                            f"[DEBUG] Temp={temp:.2f} °C | "
                            f"Setpoint={setpoint:.2f} °C | "
                            f"State={state.name} ({state.value}) | "
                            f"KP={cfg['kp']} KI={cfg['ki']} KD={cfg['kd']} | "
                            f"OUTMIN={cfg['outmin']} OUTMAX={cfg['outmax']}"
                        )
                        _last_debug_print = now
        except Exception as e:
            print("Error:", e)
        time.sleep(0.05)

if __name__ == "__main__":
    main()
