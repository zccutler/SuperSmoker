import logging
import csv
import time
import os
import shutil
import threading
import atexit
from datetime import datetime

class BufferedFileWriter:
    """
    Writes to a RAM file in /tmp and periodically flushes it to /var/log.
    """
    def __init__(self, tmp_path, final_path, flush_interval):
        self.tmp_path = tmp_path
        self.final_path = final_path
        self.flush_interval = flush_interval
        self._stop_flag = False

        # ensure tmp exists and create empty file
        with open(self.tmp_path, "w") as f:
            pass

        # background flush thread
        self.thread = threading.Thread(target=self._flusher, daemon=True)
        self.thread.start()

        # ensure final flush on exit
        atexit.register(self.flush)

    def write(self, data: str):
        with open(self.tmp_path, "a") as f:
            f.write(data)

    def flush(self):
        """
        Copies current /tmp log to /var/log safely without truncating.
        """
        try:
            os.makedirs(os.path.dirname(self.final_path), exist_ok=True)
            shutil.copyfile(self.tmp_path, self.final_path)
        except Exception as e:
            print(f"[DEBUG] Failed to flush logs: {e}")

    def _flusher(self):
        while not self._stop_flag:
            time.sleep(self.flush_interval)
            self.flush()

    def stop(self):
        self._stop_flag = True
        self.flush()


class BufferedHandler(logging.Handler):
    """
    A logging.Handler that writes formatted log messages
    into the BufferedFileWriter buffer.
    """
    def __init__(self, buffer_writer):
        super().__init__()
        self.buf = buffer_writer

    def emit(self, record):
        msg = self.format(record) + "\n"
        self.buf.write(msg)


class KPIHandler:
    """
    CSV writer stored in RAM first, flushed periodically.
    """
    def __init__(self, tmp_path, final_path, flush_interval):
        self.buf = BufferedFileWriter(tmp_path, final_path, flush_interval)
        # create CSV header
        with open(tmp_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["timestamp", "temp", "setpoint", "state"])

    def write_kpi(self, *, temp, setpoint, state):
        ts = time.time()
        with open(self.buf.tmp_path, "a", newline="") as f:
            w = csv.writer(f)
            w.writerow([ts, temp, setpoint, state])


class DebugWrapper:
    def __init__(self, logger, kpi_handler):
        self.logger = logger
        self.kpi_handler = kpi_handler

    def debug(self, *args, **kwargs):
        self.logger.debug(*args, **kwargs)

    def info(self, *args, **kwargs):
        self.logger.info(*args, **kwargs)

    def warning(self, *args, **kwargs):
        self.logger.warning(*args, **kwargs)

    def error(self, *args, **kwargs):
        self.logger.error(*args, **kwargs)

    def kpi(self, *, temp, setpoint, state):
        if self.kpi_handler is not None:
            self.kpi_handler.write_kpi(
                temp=temp, setpoint=setpoint, state=state
            )


def get_debug_logger(cfg):
    """
    Creates the fully buffered debug logging system.
    """

    enabled = int(cfg.get("debug", 0)) == 1
    if not enabled:
        # Debug disabled = return no-op logger
        logger = logging.getLogger("supersmoker")
        logger.addHandler(logging.NullHandler())
        return DebugWrapper(logger, None)

    # convert timestamp placeholders
    timestamp = datetime.now().strftime("%Y-%m-%d_%H%M%S")

    log_path = cfg["debug_log"].format(timestamp=timestamp)
    csv_path = cfg["debug_csv"].format(timestamp=timestamp)

    flush_interval = int(cfg.get("debug_flush_interval", 180))

    # TMP RAM filenames
    tmp_log = f"/tmp/{os.path.basename(log_path)}"
    tmp_csv = f"/tmp/{os.path.basename(csv_path)}"

    # base logger
    logger = logging.getLogger("supersmoker")
    logger.setLevel(logging.DEBUG)

    # console output
    ch = logging.StreamHandler()
    ch.setFormatter(logging.Formatter("%H:%M:%S [%(levelname)s] %(message)s"))
    logger.addHandler(ch)

    # buffered text log file
    log_buffer = BufferedFileWriter(tmp_log, log_path, flush_interval)
    fh = BufferedHandler(log_buffer)
    fh.setFormatter(logging.Formatter(
        "%Y-%m-%d %H:%M:%S [%(levelname)s] %(message)s"
    ))
    logger.addHandler(fh)

    # buffered KPI CSV
    kpi_handler = KPIHandler(tmp_csv, csv_path, flush_interval)

    return DebugWrapper(logger, kpi_handler)
