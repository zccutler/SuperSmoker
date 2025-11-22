#!/usr/bin/python3
import time
import smbus
import statistics
import math

# ----- I2C Settings -----
I2C_ADDR = 0x4F
I2C_BUS = 1
SAMPLE_INTERVAL = 2  # seconds between reported outputs
SAMPLE_RATE = 400      # raw samples per second inside each window
TRIM_FRACTION = 0.20   # drop 10% lowest & highest samples
MAD_THRESHOLD = 3.0    # drop samples > 4×MAD from median
MIN_SAMPLES = 10        # minimum to compute valid average

bus = smbus.SMBus(I2C_BUS)

# ----- Calibration Data -----
raw_0C = 16.8    # raw counts in ice bath
raw_100C = 467 # raw counts in boiling water
actual_0C = 0.0
actual_100C = 100.0

# Compute linear correction
slope = (actual_100C - actual_0C) / (raw_100C - raw_0C)
offset = actual_0C - slope * raw_0C

# -------------------------------
# Helper functions
# -------------------------------

def median_absolute_deviation(data, median=None):
    if not data:
        return 0.0
    if median is None:
        median = statistics.median(data)
    return statistics.median([abs(x - median) for x in data])

def trimmed_mean(data, trim_fraction):
    if not data:
        raise ValueError("trimmed_mean: empty data")
    n = len(data)
    k = int(math.floor(trim_fraction * n))
    if 2 * k >= n:
        return statistics.mean(data)
    s = sorted(data)
    return statistics.mean(s[k:n - k])

def read_temp_c():
    """Read raw counts from sensor and apply linear correction to return Celsius."""
    data = bus.read_i2c_block_data(I2C_ADDR, 1, 2)
    raw = (data[0] << 8) | data[1]
    corrected_c = slope * raw + offset
    return corrected_c, raw

def robust_window_average(window_seconds=1.0):
    """
    Rapidly sample temperature readings for a short window,
    discard outliers, and return a cleaned average temperature.
    """
    samples = []
    start = time.perf_counter()
    deadline = start + window_seconds
    sleep_interval = 1.0 / SAMPLE_RATE

    while time.perf_counter() < deadline:
        try:
            corrected_c, _ = read_temp_c()
            samples.append(corrected_c)
        except Exception as e:
            print(f"Read error: {e}")
        time.sleep(sleep_interval)

    if len(samples) < MIN_SAMPLES:
        return None, {}

    median = statistics.median(samples)
    mad = median_absolute_deviation(samples, median)

    # MAD-based rejection
    if mad > 0:
        survivors = [x for x in samples if abs(x - median) <= MAD_THRESHOLD * mad]
    else:
        survivors = list(samples)

    if len(survivors) < MIN_SAMPLES:
        survivors = list(samples)

    clean_avg = trimmed_mean(survivors, TRIM_FRACTION)

    diagnostics = {
        "raw_count": len(samples),
        "kept_count": len(survivors),
        "removed_count": len(samples) - len(survivors),
        "mad": mad,
        "median": median,
        "min": min(samples),
        "max": max(samples),
        "stddev": statistics.pstdev(samples) if len(samples) > 1 else 0.0,
    }

    return clean_avg, diagnostics

# -------------------------------
# Main Loop
# -------------------------------

def main():
    try:
        ema_alpha = 0.25
        ema_value = None

        while True:
            avg_temp, info = robust_window_average(window_seconds=SAMPLE_INTERVAL)

            if avg_temp is None:
                print("No valid samples collected.")
                continue

            # Exponential moving average smoothing
            if ema_value is None:
                ema_value = avg_temp
            else:
                ema_value = ema_alpha * avg_temp + (1 - ema_alpha) * ema_value

            print(
                f"Cleaned Avg: {avg_temp:6.2f} °C | Smoothed: {ema_value:6.2f} °C "
                f"| Samples: {info.get('raw_count',0)} "
                f"Kept: {info.get('kept_count',0)} "
                f"Removed: {info.get('removed_count',0)} "
                f"MAD: {info.get('mad',0):.4f}"
            )

    except KeyboardInterrupt:
        print("Exiting...")

if __name__ == "__main__":
    main()
