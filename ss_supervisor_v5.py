#!/usr/bin/env python3
import serial
import time
import smbus
import statistics
import math
from enum import Enum
from threading import Thread, Event
import atexit

from ss_debug_module import ssdebug  # separate module

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

# ==============================
# Load Config
# ==============================
def load_config(path="/etc/supersmoker.conf"):
    cfg = {}
    with open(path) as f:
        for line in f:
            line = line.split("#")[0].strip()  # remove inline comments
            if not line or "=" not in line:
                continue
            key, val = line.split("=", 1)
            cfg[key.strip()] = val.strip()
    return cfg

cfg = load_config()

serial_port = cfg["serial_port"]
ser = serial.Serial(serial_port, 115200, timeout=0.1)

# I2C Config
I2C_ADDR = int(cfg["i2c_addr"], 16)
I2C_BUS = int(cfg["i2c_bus"])
SAMPLE_RATE = float(cfg["sample_rate"])
TRIM_FRACTION = float(cfg["trim_fraction"])
MAD_THRESHOLD = float(cfg["mad_threshold"])
MIN_SAMPLES = int(cfg["min_samples"])
raw_0C = float(cfg["raw_0C"])
raw_100C = float(cfg["raw_100C"])
actual_0C = float(cfg["actual_0C"])
actual_100C = float(cfg["actual_100C"])
slope = (actual_100C - actual_0C) / (raw_100C - raw_0C)
offset = actual_0C - slope * raw_0C

bus = smbus.SMBus(I2C_BUS)
ema_temp = None

# PID State
setpoint = float(cfg["setpoint"])
kp = float(cfg["kp"])
ki = float(cfg["ki"])
kd = float(cfg["kd"])
outmin = int(cfg["outmin"])
outmax = int(cfg["outmax"])

# Machine state
class MachineState(Enum):
    IDLE = 0
    ACTIVE = 1

machine_state = MachineState.IDLE

# Debug Logger
debug_enabled = cfg.get("debug", "False") == "True"
if debug_enabled:
    logger = ssdebug(cfg)

# ==============================
# Dynamic Config Reload (inotify)
# ==============================
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import threading

class ConfigChangeHandler(FileSystemEventHandler):
    def __init__(self, path, reload_callback):
        self.path = path
        self.reload_callback = reload_callback
        super().__init__()

    def on_modified(self, event):
        if event.src_path.endswith(self.path):
            try:
                self.reload_callback()
                print(f"[INFO] Config reloaded from {self.path}")
            except Exception as e:
                print(f"[ERROR] Failed to reload config: {e}")

def reload_config():
    global cfg, setpoint, kp, ki, kd, outmin, outmax, I2C_ADDR, I2C_BUS, SAMPLE_RATE
    cfg = load_config("/etc/supersmoker.conf")  # adjust path if needed

    # Reload runtime parameters
    setpoint = float(cfg["setpoint"])
    kp = float(cfg["kp"])
    ki = float(cfg["ki"])
    kd = float(cfg["kd"])
    outmin = int(cfg["outmin"])
    outmax = int(cfg["outmax"])

    # Reload I2C config if needed
    I2C_ADDR = int(cfg["i2c_addr"], 16)
    I2C_BUS = int(cfg["i2c_bus"])
    SAMPLE_RATE = float(cfg["sample_rate"])

# Initialize watchdog observer
config_path = "supersmoker.conf"  # relative filename
observer = Observer()
event_handler = ConfigChangeHandler(config_path, reload_config)
observer.schedule(event_handler, path="/etc", recursive=False)  # adjust to actual folder
observer_thread = threading.Thread(target=observer.start, daemon=True)
observer_thread.start()

# Ensure observer stops on exit
import atexit
atexit.register(observer.stop)
atexit.register(observer.join)

# ==============================
# Thermocouple Thread
# ==============================
actualTemp = 0.0
stop_event = Event()

def thermocouple_loop():
    global ema_temp, actualTemp
    window_seconds = 0.5
    sleep_interval = 1.0 / SAMPLE_RATE

    while not stop_event.is_set():
        samples = []
        start = time.perf_counter()
        while time.perf_counter() - start < window_seconds:
            try:
                data = bus.read_i2c_block_data(I2C_ADDR, 1, 2)
                raw = (data[0] << 8) | data[1]
                samples.append(slope * raw + offset)
            except Exception:
                pass
            time.sleep(sleep_interval)

        if len(samples) >= MIN_SAMPLES:
            median_val = statistics.median(samples)
            mad = median_absolute_deviation(samples, median_val)
            if mad > 0:
                survivors = [x for x in samples if abs(x - median_val) <= MAD_THRESHOLD * mad]
            else:
                survivors = samples
            clean_avg = trimmed_mean(survivors, TRIM_FRACTION)

            # EMA smoothing
            if ema_temp is None:
                ema_temp = clean_avg
            else:
                ema_temp = 0.25 * clean_avg + 0.75 * ema_temp
            actualTemp = ema_temp

        time.sleep(sleep_interval)

therm_thread = Thread(target=thermocouple_loop, daemon=True)
therm_thread.start()
atexit.register(lambda: stop_event.set())

# ==============================
# Machine State Logic
# ==============================

def update_machine_state():
    """
    Sets the machine_state based on core logic:
    - ACTIVE if system is below setpoint or currently running
    - IDLE only if explicitly configured or stopped
    """
    global machine_state

    # Sanity check
    if actualTemp is None or setpoint is None:
        # If temperature reading is missing, fallback to IDLE
        machine_state = MachineState.IDLE
        return

    # Core active/idle decision:
    # ACTIVE if temperature below setpoint, otherwise IDLE
    if actualTemp < setpoint:
        machine_state = MachineState.ACTIVE
    else:
        machine_state = MachineState.IDLE

# ==============================
# Serial GET Handler
# ==============================
def send_data():
    lines = [
        f"TEMP={actualTemp:.2f}",
        f"SETPOINT={setpoint:.2f}",
        f"STATE={machine_state.value}",
        f"KP={kp}",
        f"KI={ki}",
        f"KD={kd}",
        f"OUTMIN={outmin}",
        f"OUTMAX={outmax}",
        "END\n"
    ]
    ser.write("\n".join(lines).encode("ascii"))

# ==============================
# Main Loop
# ==============================
while True:
    try:
        line = ser.readline().decode("ascii", errors="ignore").strip()
        if line == "GET":
            cfg = load_config()  # hot reload
            update_machine_state()
            send_data()
            if debug_enabled:
                logger.log(actualTemp, setpoint, machine_state.value, kp, ki, kd, outmin, outmax)
    except Exception as e:
        print("Supervisor Error:", e)
    time.sleep(0.05)
