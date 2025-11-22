# ss_debug_module.py
import csv
import os
import time
from threading import Thread, Event

class ssdebug:
    def __init__(self, cfg):
        self.enabled = cfg.get("debug", "False") == "True"
        if not self.enabled:
            return

        prefix = cfg.get("debug_log_prefix", "ss_debug")
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.filename = f"/tmp/{prefix}_{timestamp}.csv"

        self.flush_interval = int(cfg.get("debug_flush_interval", "180"))  # seconds
        self.buffer = []
        self.stop_event = Event()

        # Start flush thread
        self.thread = Thread(target=self._flush_loop, daemon=True)
        self.thread.start()

        # Ensure flush on exit
        import atexit
        atexit.register(self.stop)

        # Write CSV header
        self.buffer.append(["timestamp","actualTemp","setpoint","machine_state","kp","ki","kd","outmin","outmax"])

    def log(self, actualTemp, setpoint, machine_state, kp, ki, kd, outmin, outmax):
        if not self.enabled:
            return
        ts = time.time()
        self.buffer.append([ts, actualTemp, setpoint, machine_state, kp, ki, kd, outmin, outmax])
        print(f"[DEBUG] t={ts:.1f} | Temp={actualTemp:.2f} | SP={setpoint:.2f} | State={machine_state} | KP={kp} KI={ki} KD={kd} | OUTMIN={outmin} OUTMAX={outmax}")

    def _flush_loop(self):
        while not self.stop_event.wait(self.flush_interval):
            self.flush_to_disk()

    def flush_to_disk(self):
        if not self.buffer:
            return
        # Ensure /var/log exists
        os.makedirs("/var/log/ss_debug_log", exist_ok=True)
        dest_file = f"var/log/ss_debug_log/{os.path.basename(self.filename)}"
        with open(dest_file, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerows(self.buffer)
        self.buffer.clear()

    def stop(self):
        self.stop_event.set()
        self.flush_to_disk()
