#!/usr/bin/env python3
import serial
import time
import smbus
import statistics
import math
from enum import Enum
from threading import Thread, Lock

# -------------------------------
# Configuration Loader
# -------------------------------
def load_config(path="/etc/supersmoker.conf"):
    cfg = {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, val = line.split("=", 1)
            key = key.strip()
            val = val.strip()

            if val.lower().startswith("0x"):
                cfg[key] = int(val, 16)
            else:
                try:
                    cfg[key] = float(val)
                except ValueError:
                    cfg[key] = val
    return cfg

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
# MAD / trimmed mean helpers
# -------------------------------
def median_absolute_deviation(data, median_val=None):
    if not data:
        return 0.0
    if median_val is None:
        median_val = statistics.median(data)
    return statistics.median([abs(x - median_val) for x in data])

def trimmed_mean(data, trim_fraction):
    if not data:
        return 0.0
    n = len(data)
    k = int(math.floor(trim_fraction * n))
    if 2 * k >= n:
        return statistics.mean(data)
    s = sorted(data)
    return statistics.mean(s[k:n - k])

# -------------------------------
# Thermocouple Reader Thread
# -------------------------------
class ThermocoupleReader(Thread):
    def __init__(self, bus, i2c_addr, sample_rate, trim_fraction, mad_threshold, min_samples, slope, offset, ema_alpha=0.25):
        super().__init__(daemon=True)
        self.bus = bus
        self.addr = i2c_addr
        self.sample_rate = sample_rate
        self.trim_fraction = trim_fraction
        self.mad_threshold = mad_threshold
        self.min_samples = min_samples
        self.slope = slope
        self.offset = offset
        self.ema_alpha = ema_alpha

        self.lock = Lock()
        self.temperature = None
        self.running = True

    def read_raw_temp(self):
        try:
            data = self.bus.read_i2c_block_data(self.addr, 1, 2)
            raw = (data[0] << 8) | data[1]
            temp_c = self.slope * raw + self.offset
            return temp_c
        except Exception:
            return None

    def run(self):
        sleep_interval = 1.0 / self.sample_rate
        while self.running:
            samples = []
            start = time.perf_counter()
            while time.perf_counter() - start < 1.0:  # 1-second window
                temp = self.read_raw_temp()
                if temp is not None:
                    samples.append(temp)
                time.sleep(sleep_interval)
            if len(samples) < self.min_samples:
                continue
            median_val = statistics.median(samples)
            mad = median_absolute_deviation(samples, median_val)
            survivors = [x for x in samples if abs(x - median_val) <= self.mad_threshold * mad] if mad > 0 else samples
            if len(survivors) < self.min_samples:
                survivors = samples
            clean_avg = trimmed_mean(survivors, self.trim_fraction)
            with self.lock:
                if self.temperature is None:
                    self.temperature = clean_avg
                else:
                    self.temperature = self.ema_alpha * clean_avg + (1 - self.ema_alpha) * self.temperature

    def get_temperature(self):
        with self.lock:
            return self.temperature

# -------------------------------
# Machine State Logic
# -------------------------------
def determine_machine_state(temp, setpoint, cfg):
    heat_band = cfg.get("heat_band", 10.0)
    maintain_band = cfg.get("maintain_band", 5.0)
    cool_band = cfg.get("cool_band", 15.0)

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
# Serial Response
# -------------------------------
def send_data(ser, temp, setpoint, state, cfg):
    lines = [
        f"TEMP={temp:.2f}",
        f"SETPOINT={setpoint:.2f}",
        f"STATE={state.value}",
        f"KP={cfg.get('kp',2.0)}",
        f"KI={cfg.get('ki',0.5)}",
        f"KD={cfg.get('kd',0.0)}",
        f"OUTMIN={cfg.get('outmin',0)}",
        f"OUTMAX={cfg.get('outmax',255)}",
        "END\n"
    ]
    ser.write("\n".join(lines).encode("ascii"))

# -------------------------------
# Main Loop
# -------------------------------
def main():
    cfg = load_config()
    serial_port = cfg.get("serial_port")
    if not serial_port:
        raise RuntimeError("serial_port must be defined in supersmoker.conf")

    # I2C calibration
    slope = (cfg["actual_100C"] - cfg["actual_0C"]) / (cfg["raw_100C"] - cfg["raw_0C"])
    offset = cfg["actual_0C"] - slope * cfg["raw_0C"]

    bus = smbus.SMBus(int(cfg["i2c_bus"]))
    therm_reader = ThermocoupleReader(
        bus=bus,
        i2c_addr=cfg["i2c_addr"],
        sample_rate=cfg["sample_rate"],
        trim_fraction=cfg["trim_fraction"],
        mad_threshold=cfg["mad_threshold"],
        min_samples=int(cfg["min_samples"]),
        slope=slope,
        offset=offset,
    )
    therm_reader.start()

    ser = serial.Serial(serial_port, 115200, timeout=0.1)
    print("Supersmoker Supervisor started")

    while True:
        try:
            line = ser.readline().decode("ascii", errors="ignore").strip()
            if line == "GET":
                cfg = load_config()
                setpoint = float(cfg.get("setpoint", 0.0))
                temp = therm_reader.get_temperature() or 0.0
                state = determine_machine_state(temp, setpoint, cfg)
                send_data(ser, temp, setpoint, state, cfg)
        except Exception as e:
            print("Error:", e)
        time.sleep(0.05)

if __name__ == "__main__":
    main()
