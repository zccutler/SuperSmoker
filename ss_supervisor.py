#!/usr/bin/env python3
import serial
import time
import smbus
import statistics
import math
import sys
from enum import Enum

DEBUG = True
DEBUG_INTERVAL = 1.0
_last_debug_print = 0

I2C_ADDR = 0x4F
I2C_BUS = 1
SAMPLE_RATE = 400
TRIM_FRACTION = 0.20
MAD_THRESHOLD = 3.0
MIN_SAMPLES = 10

raw_0C = 16.8
raw_100C = 467
actual_0C = 0.0
actual_100C = 100.0
slope = (actual_100C - actual_0C) / (raw_100C - raw_0C)
offset = actual_0C - slope * raw_0C

bus = smbus.SMBus(I2C_BUS)
ema_temp = None


class MachineState(Enum):
    IDLE = 0
    HEATING = 1
    MAINTAINING = 2
    COOLING = 3
    SHUTDOWN = 4
    FAULT = 5


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


def read_raw_temp_c():
    data = bus.read_i2c_block_data(I2C_ADDR, 1, 2)
    raw = (data[0] << 8) | data[1]
    corrected_c = slope * raw + offset
    return corrected_c, raw


def read_thermocouple_celsius(window_seconds=1.0, ema_alpha=0.25):
    global ema_temp
    samples = []
    start = time.perf_counter()
    sleep_interval = 1.0 / SAMPLE_RATE

    while time.perf_counter() - start < window_seconds:
        try:
            temp_c, _ = read_raw_temp_c()
            samples.append(temp_c)
        except Exception:
            pass
        time.sleep(sleep_interval)

    if len(samples) < MIN_SAMPLES:
        return None

    median_val = statistics.median(samples)
    mad = median_absolute_deviation(samples, median_val)

    if mad > 0:
        survivors = [x for x in samples if abs(x - median_val) <= MAD_THRESHOLD * mad]
    else:
        survivors = samples

    if len(survivors) < MIN_SAMPLES:
        survivors = samples

    clean_avg = trimmed_mean(survivors, TRIM_FRACTION)

    if ema_temp is None:
        ema_temp = clean_avg
    else:
        ema_temp = ema_alpha * clean_avg + (1 - ema_alpha) * ema_temp

    return ema_temp


def determine_machine_state(temp, setpoint, cfg):
    try:
        heat_band = float(cfg.get("heat_band", "10"))
        maintain_band = float(cfg.get("maintain_band", "5"))
        cool_band = float(cfg.get("cool_band", "15"))
    except:
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


def send_data(ser, temp, setpoint, state, cfg):
    kp = cfg.get("kp", "2.0")
    ki = cfg.get("ki", "0.5")
    kd = cfg.get("kd", "0.0")
    outmin = cfg.get("outmin", "0")
    outmax = cfg.get("outmax", "255")

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


def main():
    cfg = load_config()
    port = cfg.get("serial_port", "/dev/ttyUSB0")
    ser = serial.Serial(port, 115200, timeout=0.1)

    print("Supersmoker Supervisor started", file=sys.stderr)

    global _last_debug_print

    while True:
        try:
            line = ser.readline().decode("ascii", errors="ignore").strip()

            if line == "GET":
                cfg = load_config()
                setpoint = float(cfg.get("setpoint", "0"))
                temp = read_thermocouple_celsius(window_seconds=0.5)
                state = determine_machine_state(temp, setpoint, cfg)
                send_data(ser, temp if temp is not None else 0.0, setpoint, state, cfg)

                if DEBUG:
                    now = time.time()
                    if now - _last_debug_print >= DEBUG_INTERVAL:
                        print(
                            f"[DEBUG] Temp={temp:.2f} | Setpoint={setpoint:.2f} | "
                            f"State={state.name} | KP={cfg.get('kp')} KI={cfg.get('ki')} KD={cfg.get('kd')}",
                            file=sys.stderr
                        )
                        _last_debug_print = now

        except Exception as e:
            print(f"Error: {e}", file=sys.stderr)

        time.sleep(0.05)


if __name__ == "__main__":
    main()
