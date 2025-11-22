#!/usr/bin/env python3
import serial
import time

PORT = "/dev/ttyUSB0"
BAUD = 115200

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    print("Pi test script started. Press Ctrl-C to exit.")

    counter = 0
    try:
        while True:
            counter += 1
            cmd = f"GET {counter}\n"
            ser.write(cmd.encode("ascii"))

            time.sleep(0.5)  # small delay

            while ser.in_waiting:
                line = ser.readline().decode("ascii", errors="ignore").strip()
                if line:
                    print(f"Uno: {line}")

            time.sleep(0.5)

    except KeyboardInterrupt:
        print("Exiting...")
        ser.close()

if __name__ == "__main__":
    main()
