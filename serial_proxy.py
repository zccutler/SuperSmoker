#!/usr/bin/env python3
"""
Supersmoker serial proxy / logger.

- Creates a PTY that acts as the PLC serial port.
- Forwards PLC->Supervisor and Supervisor->PLC.
- Logs all traffic with timestamps.
- Avoids collisions: only the proxy writes to the real serial port.
"""

import os
import pty
import serial
import select
import sys
import time

REAL_SERIAL = "/dev/ttyACM0"
BAUD = 115200
LOG_FILE = "/tmp/ss_serial_proxy.log"

def timestamp():
    return time.strftime("%Y-%m-%dT%H:%M:%S.%fZ", time.gmtime())

# Open the real serial port
ser = serial.Serial(REAL_SERIAL, BAUD, timeout=0)

# Create a pseudo-terminal for PLC to connect to
master_fd, slave_fd = pty.openpty()
slave_name = os.ttyname(slave_fd)
print(f"[INFO] PLC should connect to: {slave_name}")

# Open file object for the master PTY
master = os.fdopen(master_fd, "rb+", buffering=0)

# Open log file
log = open(LOG_FILE, "a", buffering=1)

def log_line(direction, data):
    line = f"{timestamp()} {direction} {len(data)} bytes: {data!r}\n"
    log.write(line)
    sys.stdout.write(line)
    sys.stdout.flush()

def main_loop():
    while True:
        # Use select to wait for either master PTY (PLC) or serial (Supervisor)
        rlist, _, _ = select.select([master, ser], [], [], 0.05)

        for r in rlist:
            if r is master:
                # Data from PLC → forward to Supervisor
                data = os.read(master_fd, 1024)
                if data:
                    log_line("PLC->SUP", data)
                    ser.write(data)
            elif r is ser:
                # Data from Supervisor → forward to PLC
                data = ser.read(1024)
                if data:
                    log_line("SUP->PLC", data)
                    os.write(master_fd, data)

try:
    main_loop()
except KeyboardInterrupt:
    print("\n[INFO] Proxy terminated by user")
finally:
    ser.close()
    master.close()
    log.close()
