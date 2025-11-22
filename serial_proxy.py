#!/usr/bin/env python3
"""
serial_proxy.py

Creates a pseudo-tty for supervisor to connect to, proxies traffic to real serial
device (PLC side), and logs everything with timestamps.

Usage:
    sudo python3 serial_proxy.py --real /dev/ttyUSB0 --baud 115200 --log /var/log/ss_serial_proxy.log

Output:
    Prints the pseudo-tty path (e.g. /dev/pts/3). Point supervisor config serial_port to that path.
"""

import os
import pty
import argparse
import select
import sys
import time
import tty
import termios
from datetime import datetime

def ts():
    return datetime.utcnow().isoformat(timespec='milliseconds') + 'Z'

def log_line(fp, direction, data):
    # direction: 'SUP->PLC' or 'PLC->SUP'
    line = f"{ts()} {direction} {len(data)} bytes: {data!r}\n"
    fp.write(line)
    fp.flush()
    # also print to stdout for quick debugging
    sys.stdout.write(line)
    sys.stdout.flush()

def open_real_serial(path, baud):
    import serial
    # open non-blocking
    ser = serial.Serial(path, baud, timeout=0)
    return ser

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--real", required=True, help="Real serial device (PLC side), e.g. /dev/ttyUSB0")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--log", default="./serial_proxy.log")
    args = p.parse_args()

    # create pty pair
    master_fd, slave_fd = pty.openpty()
    slave_name = os.ttyname(slave_fd)
    print(f"[{ts()}] Created pty: {slave_name}")
    print(f"[{ts()}] Point supervisor serial_port in config to: {slave_name}")
    sys.stdout.flush()

    # configure pts to raw mode (optional)
    attrs = termios.tcgetattr(master_fd)
    tty.setraw(master_fd)

    # open real serial device using pyserial
    try:
        ser = open_real_serial(args.real, args.baud)
    except Exception as e:
        print(f"[{ts()}] Failed to open real serial device {args.real}: {e}", file=sys.stderr)
        sys.exit(1)

    # open log file
    log_fp = open(args.log, "a", buffering=1)

    # We will select on master_fd and ser.fileno()
    master_fileno = master_fd
    ser_fileno = ser.fileno()

    print(f"[{ts()}] Proxy running. Logging to {args.log}")
    try:
        while True:
            rlist, _, _ = select.select([master_fileno, ser_fileno], [], [], 0.2)
            # Data from supervisor (connected to the pty slave) -> real serial -> PLC
            if master_fileno in rlist:
                try:
                    data = os.read(master_fileno, 4096)
                except OSError:
                    data = b''
                if data:
                    # write to log
                    log_line(log_fp, "SUP->PLC", data)
                    # forward to real serial
                    ser.write(data)

            # Data from real serial (PLC) -> pty master -> supervisor
            if ser_fileno in rlist:
                try:
                    data = ser.read(4096)
                except Exception:
                    data = b''
                if data:
                    log_line(log_fp, "PLC->SUP", data)
                    os.write(master_fileno, data)
    except KeyboardInterrupt:
        print(f"[{ts()}] Stopping proxy (KeyboardInterrupt)")
    finally:
        try:
            ser.close()
        except Exception:
            pass
        log_fp.close()

if __name__ == "__main__":
    main()