#!/usr/bin/env python3
"""
ss_supervisor_v5.py
Simplified supervisor (ACTIVE/IDLE PLC model).
- Uno (PLC) polls with "GET"
- Supervisor responds with STATE=ACTIVE + parameters OR STATE=IDLE
- Lid-open detection forces STATE=IDLE until lid-closed detected (sustained rise)
- Async thermocouple reader (MAD + EMA)
- Debugging via ss_debug.get_debug_logger(cfg) (optional)
"""

import time
import smbus
import statistics
import math
import threading
from collections import deque
from enum import Enum

# optional debug module
try:
    from ss_debug import get_debug_logger
except Exception:
    class _NoDebug:
        def debug(self, *a, **k): pass
        def info(self, *a, **k): pass
        def warning(self, *a, **k): pass
        def error(self, *a, **k): pass
        def kpi(self, *a, **k): pass
    def get_debug_logger(cfg): return _NoDebug()

# -------------------------------
# Machine "states" (supervisor-level)
# PLC only sees ACTIVE or IDLE
# -------------------------------
class SupervisorMode(Enum):
    IDLE = 0
    ACTIVE = 1
    LID_OPEN = 2   # supervisor-only (PLC will be told IDLE)

# -------------------------------
# Config loader (no-inline-comments; hex support)
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
            if isinstance(val, str) and val.lower().startswith("0x"):
                try:
                    cfg[key] = int(val, 16)
                    continue
                except ValueError:
                    pass
            try:
                cfg[key] = float(val)
            except ValueError:
                cfg[key] = val
    return cfg

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
# Thermocouple reader thread
# -------------------------------
class ThermocoupleReader(threading.Thread):
    def __init__(self, bus, i2c_addr, sample_rate, trim_fraction, mad_threshold, min_samples,
                 raw_0C, raw_100C, actual_0C, actual_100C, ema_alpha=0.25, history_seconds=600):
        super().__init__(daemon=True)
        self.bus = bus
        self.addr = int(i2c_addr)
        self.sample_rate = float(sample_rate)
        self.trim_fraction = float(trim_fraction)
        self.mad_threshold = float(mad_threshold)
        self.min_samples = int(min_samples)
        self.slope = (float(actual_100C) - float(actual_0C)) / (float(raw_100C) - float(raw_0C))
        self.offset = float(actual_0C) - self.slope * float(raw_0C)
        self.ema_alpha = float(ema_alpha)
        self.temperature = None
        self.temperature_lock = threading.Lock()
        self.history = deque()   # (ts, temp)
        self.history_lock = threading.Lock()
        self.history_seconds = int(history_seconds)
        self._stopped = threading.Event()

    def read_raw(self):
        try:
            data = self.bus.read_i2c_block_data(self.addr, 1, 2)
            raw = (data[0] << 8) | data[1]
            temp_c = self.slope * raw + self.offset
            return temp_c
        except Exception:
            return None

    def append_history(self, ts, temp):
        with self.history_lock:
            self.history.append((ts, temp))
            cutoff = ts - self.history_seconds
            while self.history and self.history[0][0] < cutoff:
                self.history.popleft()

    def get_history_snapshot(self):
        with self.history_lock:
            return list(self.history)

    def get_temperature(self):
        with self.temperature_lock:
            return self.temperature

    def stop(self):
        self._stopped.set()

    def run(self):
        sleep_interval = 1.0 / max(1, int(self.sample_rate))
        while not self._stopped.is_set():
            samples = []
            window_start = time.perf_counter()
            duration = 1.0
            while time.perf_counter() - window_start < duration:
                t = self.read_raw()
                if t is not None:
                    samples.append(t)
                time.sleep(sleep_interval)
            if len(samples) < self.min_samples:
                if samples:
                    avg = statistics.mean(samples)
                    ts = time.time()
                    self.append_history(ts, avg)
                continue
            median_val = statistics.median(samples)
            mad = median_absolute_deviation(samples, median_val)
            if mad > 0:
                survivors = [x for x in samples if abs(x - median_val) <= self.mad_threshold * mad]
            else:
                survivors = samples
            if len(survivors) < self.min_samples:
                survivors = samples
            clean_avg = trimmed_mean(survivors, self.trim_fraction)
            with self.temperature_lock:
                if self.temperature is None:
                    self.temperature = clean_avg
                else:
                    self.temperature = self.ema_alpha * clean_avg + (1 - self.ema_alpha) * self.temperature
            ts = time.time()
            self.append_history(ts, self.temperature)

# -------------------------------
# Lid event detection helpers
# -------------------------------
def detect_lid_open(history, drop_threshold, drop_window):
    if not history:
        return False
    now_ts, now_temp = history[-1]
    cutoff = now_ts - drop_window
    older = None
    for ts, t in history:
        if ts >= cutoff:
            older = (ts, t)
            break
    if older is None:
        older = history[0]
    old_temp = older[1]
    delta = old_temp - now_temp
    return delta >= drop_threshold

def detect_lid_closed(history, rise_threshold, rise_window):
    if not history:
        return False
    now_ts, now_temp = history[-1]
    cutoff = now_ts - rise_window
    older = None
    for ts, t in history:
        if ts >= cutoff:
            older = (ts, t)
            break
    if older is None:
        return False
    old_temp = older[1]
    delta = now_temp - old_temp
    return delta >= rise_threshold

# -------------------------------
# Build reply
# -------------------------------
def build_reply_payload(mode, temp, setpoint, cfg):
    """
    mode: SupervisorMode.IDLE or SupervisorMode.ACTIVE
    If ACTIVE, include SETPOINT and PID params. If IDLE, only STATE=IDLE
    """
    if mode == SupervisorMode.ACTIVE:
        lines = [
            "STATE=ACTIVE",
            f"SETPOINT={setpoint:.2f}",
            f"KP={cfg['kp']}",
            f"KI={cfg['ki']}",
            f"KD={cfg['kd']}",
            f"OUTMIN={int(cfg['outmin'])}",
            f"OUTMAX={int(cfg['outmax'])}",
            "END",
        ]
    else:
        lines = [
            "STATE=IDLE",
            "END",
        ]
    return ("\n".join(lines) + "\n").encode("ascii")

# -------------------------------
# Main
# -------------------------------
def main():
    cfg = load_config()

    # required keys
    required = [
        "serial_port", "setpoint",
        "i2c_addr", "i2c_bus",
        "sample_rate", "trim_fraction", "mad_threshold", "min_samples",
        "raw_0C", "raw_100C", "actual_0C", "actual_100C",
        "kp", "ki", "kd", "outmin", "outmax",
        "maintain_band_celsius",
        "lid_open_drop_threshold", "lid_open_drop_window",
        "lid_closed_rise_threshold", "lid_closed_window",
        # debug config optional
    ]
    missing = [k for k in required if k not in cfg]
    if missing:
        raise RuntimeError("Missing required config keys: " + ", ".join(missing))

    # config numeric conversions
    serial_port = cfg["serial_port"]
    sample_rate = float(cfg["sample_rate"])
    trim_fraction = float(cfg["trim_fraction"])
    mad_threshold = float(cfg["mad_threshold"])
    min_samples = int(cfg["min_samples"])
    raw_0C = float(cfg["raw_0C"])
    raw_100C = float(cfg["raw_100C"])
    actual_0C = float(cfg["actual_0C"])
    actual_100C = float(cfg["actual_100C"])

    maintain_band = float(cfg["maintain_band_celsius"])
    lid_open_drop_threshold = float(cfg["lid_open_drop_threshold"])
    lid_open_drop_window = float(cfg["lid_open_drop_window"])
    lid_closed_rise_threshold = float(cfg["lid_closed_rise_threshold"])
    lid_closed_window = float(cfg["lid_closed_window"])

    debug = get_debug_logger(cfg)

    # set up I2C reader
    bus = smbus.SMBus(int(cfg["i2c_bus"]))
    therm_reader = ThermocoupleReader(
        bus=bus,
        i2c_addr=cfg["i2c_addr"],
        sample_rate=sample_rate,
        trim_fraction=trim_fraction,
        mad_threshold=mad_threshold,
        min_samples=min_samples,
        raw_0C=raw_0C,
        raw_100C=raw_100C,
        actual_0C=actual_0C,
        actual_100C=actual_100C,
        ema_alpha=0.25,
        history_seconds=600
    )
    therm_reader.start()

    import serial
    ser = serial.Serial(serial_port, 115200, timeout=0.1)

    print("Supersmoker Supervisor v5 (simplified ACTIVE/IDLE) started")

    mode = SupervisorMode.IDLE
    try:
        while True:
            try:
                line = ser.readline().decode("ascii", errors="ignore").strip()
            except Exception as e:
                debug.error("Serial read error: %s", e)
                time.sleep(0.05)
                continue

            if line == "GET":
                cfg = load_config()
                setpoint = float(cfg["setpoint"])

                temp = therm_reader.get_temperature()
                history = therm_reader.get_history_snapshot()

                # If lid-open currently active, check lid-closed (sustained rise) first
                if mode == SupervisorMode.LID_OPEN:
                    if detect_lid_closed(history, lid_closed_rise_threshold, lid_closed_window):
                        mode = SupervisorMode.ACTIVE
                        debug.info("LID_CLOSED detected -> resuming ACTIVE")
                    else:
                        # remain in LID_OPEN
                        debug.info("Still LID_OPEN (waiting for sustained rise)")
                else:
                    # mode not LID_OPEN; check for rapid drop (lid open)
                    # Only consider lid-open if we currently have an active target (setpoint>0)
                    if setpoint > 0 and temp is not None:
                        if detect_lid_open(history, lid_open_drop_threshold, lid_open_drop_window):
                            mode = SupervisorMode.LID_OPEN
                            debug.info("LID_OPEN detected -> forcing PLC IDLE")
                        else:
                            # normal determination: ACTIVE if setpoint>0, IDLE if setpoint <= 0
                            if setpoint > 0:
                                mode = SupervisorMode.ACTIVE
                            else:
                                mode = SupervisorMode.IDLE
                    else:
                        mode = SupervisorMode.IDLE

                # Build reply: if mode==ACTIVE include params; if IDLE or LID_OPEN send STATE=IDLE
                reply_mode = SupervisorMode.ACTIVE if mode == SupervisorMode.ACTIVE else SupervisorMode.IDLE
                payload = build_reply_payload(reply_mode, temp if temp is not None else 0.0, setpoint, cfg)
                try:
                    ser.write(payload)
                except Exception as e:
                    debug.error("Serial write error: %s", e)

                # Debug & KPI
                debug.kpi(temp=temp if temp is not None else 0.0,
                          setpoint=setpoint,
                          state=(1 if mode==SupervisorMode.ACTIVE else 0))
                debug.info("Handled GET: mode=%s temp=%s setpoint=%.2f",
                           mode.name,
                           ("{:.2f}".format(temp) if temp is not None else "N/A"),
                           setpoint)
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("Supervisor exiting (KeyboardInterrupt)")
    finally:
        try:
            therm_reader.stop()
        except Exception:
            pass
        try:
            ser.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
