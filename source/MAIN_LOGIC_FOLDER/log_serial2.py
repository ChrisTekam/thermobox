"""
log_serial.py

Reads CSV lines printed by the Arduino (Logger.h: "millis,setpoint,
temperature,duty,pwm") over serial and saves them to a timestamped
.csv file, while also echoing them to the terminal so you can watch
the run live.

The time column is converted from milliseconds to seconds, and only
one row per SAMPLE_PERIOD_S seconds is logged (the Arduino can keep
sending at a faster rate).

Usage:
    python log_serial.py                  # auto-detects the port
    python log_serial.py COM5             # Windows: force a port
    python log_serial.py /dev/ttyACM0     # Linux/Mac: force a port

Stop logging with Ctrl+C - the file is flushed after every row, so
nothing is lost even if you kill it abruptly.

Requires: pip install pyserial
"""

import sys
import csv
import glob
import time
import serial
from datetime import datetime

BAUD_RATE = 9600          # must match Logger(baud) in Logger.h
OUTPUT_DIR = "."          # where to save the .csv files
SAMPLE_PERIOD_S = 1       # log one row per this many seconds


def find_port():
    """Best-effort auto-detect for an Arduino-like serial port."""
    candidates = (
        glob.glob("/dev/ttyACM*")
        + glob.glob("/dev/ttyUSB*")
        + glob.glob("/dev/cu.usbmodem*")
        + glob.glob("/dev/cu.usbserial*")
        + glob.glob("COM*")
    )
    if not candidates:
        return None
    return candidates[0]


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    if port is None:
        print("No serial port found. Plug in the Arduino, or pass the "
              "port explicitly, e.g.: python log_serial.py COM5")
        sys.exit(1)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{OUTPUT_DIR}/run_{timestamp}.csv"

    print(f"Opening {port} at {BAUD_RATE} baud...")
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
    except serial.SerialException as e:
        print(f"Could not open {port}: {e}")
        sys.exit(1)

    # Give the Uno a moment - opening the port resets it.
    time.sleep(2)
    ser.reset_input_buffer()

    print(f"Logging to {filename}  (Ctrl+C to stop)\n")

    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time_s", "setpoint", "temperature", "duty", "pwm"])
        row_count = 0
        last_slot = None

        try:
            while True:
                raw = ser.readline().decode("utf-8", errors="replace").strip()
                if not raw:
                    continue  # timeout with no data - just try again

                parts = raw.split(",")
                try:
                    t = float(parts[0]) / 1000.0   # ms -> s
                except ValueError:
                    continue  # skip boot messages / garbled lines

                slot = int(t // SAMPLE_PERIOD_S)
                if slot == last_slot:
                    continue  # already logged a row this second
                last_slot = slot

                parts[0] = f"{t:.1f}"
                print(",".join(parts))
                writer.writerow(parts)
                f.flush()
                row_count += 1

        except KeyboardInterrupt:
            print(f"\nStopped. {row_count} rows saved to {filename}")
        finally:
            ser.close()


if __name__ == "__main__":
    main()
