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
    logger = DebugLogger(cfg)

# ==============================
# Thermocouple Thread
# ==============================
actualTemp = 0.0
stop_event = Event()

def thermocouple_loop():
    global ema_temp, actualTemp
    sleep_interval = 1.0 / SAMPLE_RATE
    while not stop_event.is_set():
        try:
            data = bus.read_i2c_block_data(I2C_ADDR, 1, 2)
            raw = (data[0] << 8) | data[1]
            temp = slope * raw + offset

            # EMA
            if ema_temp is None:
                ema_temp = temp
            else:
                ema_temp = 0.25 * temp + 0.75 * ema_temp
            actualTemp = ema_temp

        except Exception:
            pass

        time.sleep(sleep_interval)

therm_thread = Thread(target=thermocouple_loop, daemon=True)
therm_thread.start()
atexit.register(lambda: stop_event.set())

# ==============================
# Lid open/close detection
# ==============================
LID_DROP_THRESHOLD = 15.0  # degC drop to consider lid open
lid_open = False
lid_temp_reference = None

def update_machine_state():
    global lid_open, machine_state, lid_temp_reference
    if lid_open:
        # check if temperature is rising back → lid closed
        if actualTemp is not None and lid_temp_reference is not None:
            if actualTemp > lid_temp_reference + 2.0:  # sustained rise
                lid_open = False
                machine_state = MachineState.ACTIVE
    else:
        # detect lid open
        if actualTemp is not None and setpoint is not None:
            if actualTemp < setpoint - 30.0:  # rapid drop triggers lid open
                lid_open = True
                lid_temp_reference = actualTemp
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
            debug_logger = ssdebug(cfg)
            update_machine_state()
            send_data()
            if debug_enabled:
                logger.log(actualTemp, setpoint, machine_state.value, kp, ki, kd, outmin, outmax)
    except Exception as e:
        print("Supervisor Error:", e)
    time.sleep(0.05)
