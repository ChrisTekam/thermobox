"""
log_serial2.py

Reads the CSV lines printed by the Arduino sketch (millis,setpoint,
temperature,duty,pwm) over serial and saves them to a timestamped
run_YYYYMMDD_HHMMSS.csv with the columns:

    time_s,setpoint,temperature,duty,pwm

time_s is the Arduino's millis() converted to seconds: the time since the
Arduino was reset. Opening the port resets an Uno, so this is also the time
since the controller started. One row is saved per SAMPLE_PERIOD_S seconds
and every saved row is echoed to the terminal.

Usage:
    python log_serial2.py                   # auto-detects the Arduino port
    python log_serial2.py COM5              # force a port (Windows)
    python log_serial2.py /dev/ttyACM0      # force a port (Linux/Mac)

Close the Arduino Serial Monitor first (only one program can use the port).
Stop with Ctrl+C. The file is flushed after every row.

Requires: pip install pyserial
"""

import csv
import os
import sys
import time
from datetime import datetime

import serial
import serial.tools.list_ports

BAUD_RATE = 9600            # must match Logger(9600) in the sketch
OUTPUT_DIR = "."            # where to save the .csv files
SAMPLE_PERIOD_S = 1.0       # log one row per this many seconds
TIME_TOLERANCE_S = 0.05     # allowance for millis() jitter between ticks
NO_DATA_WARNING_S = 10      # warn if no valid row arrives within this time

ARDUINO_HEADER = "millis,setpoint,temperature,duty,pwm"
HEADER = ["time_s", "setpoint", "temperature", "duty", "pwm"]

# USB vendor IDs: Arduino, Arduino (older), CH340/CH341, FTDI, CP210x
ARDUINO_VIDS = {0x2341, 0x2A03, 0x1A86, 0x0403, 0x10C4}
ARDUINO_KEYWORDS = ("arduino", "ch340", "ch341", "usb serial", "usb-serial",
                    "ftdi", "cp210")


def looks_like_arduino(port):
    text = " ".join([port.description or "", port.manufacturer or "",
                     port.product or ""]).lower()
    return port.vid in ARDUINO_VIDS or any(k in text for k in ARDUINO_KEYWORDS)


def find_port():
    """Return the device name of the Arduino port, or None if unclear."""
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return None

    candidates = [p for p in ports if looks_like_arduino(p)]
    if not candidates and len(ports) == 1:
        candidates = ports
    if not candidates:
        print("Could not tell which port is the Arduino. Available ports:")
        for p in ports:
            print(f"  {p.device}  ({p.description})")
        return None

    # On macOS every port shows up as both /dev/cu.* and /dev/tty.*; keep cu.*.
    cu_names = {p.device[len("/dev/cu."):] for p in candidates
                if p.device.startswith("/dev/cu.")}
    candidates = [p for p in candidates
                  if not (p.device.startswith("/dev/tty.")
                          and p.device[len("/dev/tty."):] in cu_names)]
    if len(candidates) > 1:
        print("Several possible ports found; using the first:")
        for p in candidates:
            print(f"  {p.device}  ({p.description})")
    return candidates[0].device


def parse_row(raw):
    """Return the 5 fields as strings if raw is a valid data row, else None."""
    parts = [p.strip() for p in raw.split(",")]
    if len(parts) != len(HEADER):
        return None
    try:
        int(parts[0])                  # millis is an integer
        for p in parts[1:]:
            float(p)                   # also accepts 'nan'
    except ValueError:
        return None
    return parts


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    if port is None:
        print("No Arduino serial port found. Plug it in, or pass the port "
              "explicitly, e.g.: python log_serial2.py COM5")
        sys.exit(1)

    filename = os.path.join(OUTPUT_DIR,
                            f"run_{datetime.now():%Y%m%d_%H%M%S}.csv")

    print(f"Opening {port} at {BAUD_RATE} baud...")
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
    except serial.SerialException as e:
        print(f"Could not open {port}: {e}")
        sys.exit(1)

    print(f"Logging to {filename}  (Ctrl+C to stop)\n")

    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(HEADER)
        f.flush()

        row_count = 0
        next_log_t = None
        last_t = None
        warned = False
        start = time.time()

        try:
            while True:
                raw = ser.readline().decode("utf-8", errors="replace").strip()

                if not raw:
                    if row_count == 0 and not warned \
                            and time.time() - start > NO_DATA_WARNING_S:
                        print("No data received yet. Check the port, the baud "
                              "rate, and that the Serial Monitor is closed.")
                        warned = True
                    continue

                if raw == ARDUINO_HEADER:
                    continue

                row = parse_row(raw)
                if row is None:
                    print(f"[skipped] {raw}")   # FATAL/WARNING lines show here
                    continue

                t = int(row[0]) / 1000.0        # ms -> s

                if last_t is not None and t < last_t - 1.0:
                    print("[note] Arduino restarted: time_s starts again from 0")
                    next_log_t = None
                last_t = t

                if next_log_t is not None and t < next_log_t - TIME_TOLERANCE_S:
                    continue
                if next_log_t is None or t - next_log_t > SAMPLE_PERIOD_S:
                    next_log_t = t              # first row, or resync after a gap
                next_log_t += SAMPLE_PERIOD_S

                row[0] = f"{t:.1f}"
                print(",".join(row))
                writer.writerow(row)
                f.flush()
                row_count += 1

        except KeyboardInterrupt:
            print(f"\nStopped. {row_count} rows saved to {filename}")
        except serial.SerialException as e:
            print(f"\nSerial connection lost: {e}\n"
                  f"{row_count} rows saved to {filename}")
        finally:
            ser.close()


if __name__ == "__main__":
    main()
