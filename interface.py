#!/usr/bin/env python3
"""
Cyclic Loading Test Rig — Serial Command + CSV Logger

Arduino command parser (taskSerialRecieve):
  'R' -> Running
  'I' -> Idle
  'F' -> Fault

Arduino print format (printSystemState):
  maxForce, targetForce, forceError, nonZeroCount, currentMeasure, StateString

This script:
  - Connects to serial using constants configured below
  - Starts a background reader that parses lines and logs to CSV
  - Lets you send R/I/F from the keyboard to change state
  - Adds cycle_count column that persists across runs
"""

import csv
import os
import time
import threading
from datetime import datetime

import serial  # pip install pyserial


# =========================
# USER CONFIG (EDIT THESE)
# =========================
SERIAL_PORT = "COM12"          # e.g. "COM6" (Windows) or "/dev/ttyACM0" (Linux) or "/dev/tty.usbmodemXXXX" (macOS)
BAUD_RATE = 115200             # must match Serial.begin(...) in your Arduino code

# Output CSV file path, change as needed. If file exists, continues appending.
CSV_OUTPUT_PATH = "Test_Log_CX_XXXX.csv"

# Optional behavior
ARDUINO_RESET_WAIT_S = 2.0     # many Arduinos reset when serial opens; wait before reading/sending
FLUSH_EVERY_N_ROWS = 20        # flush CSV periodically to reduce data loss risk
PRINT_LIVE_RX = True           # print parsed RX lines to terminal


CSV_HEADER = [
    "timestamp_iso",
    "cycle_count",     # <-- NEW COLUMN
    "maxForce",
    "targetForce",
    "forceError",
    "nonZeroCount",
    "currentMeasure",
    "systemState",
    "raw_line",
]


def now_iso():
    """Local timestamp with timezone, millisecond precision."""
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def parse_state_line(line: str):
    """
    Parse Arduino output from printSystemState().

    Expected:
      maxForce, targetForce, forceError, nonZeroCount, currentMeasure, State

    Arduino prints: Serial.print(", "); so split on commas and strip whitespace.
    Returns dict or None if parsing fails.
    """
    raw = line.strip()
    if not raw:
        return None

    parts = [p.strip() for p in raw.split(",")]
    if len(parts) < 6:
        return None

    maxForce_s = parts[0]
    targetForce_s = parts[1]
    forceError_s = parts[2]
    nonZeroCount_s = parts[3]
    currentMeasure_s = parts[4]
    state_s = ",".join(parts[5:]).strip()  # robust if extra commas occur later

    try:
        maxForce = float(maxForce_s)
        targetForce = float(targetForce_s)
        forceError = float(forceError_s)
        nonZeroCount = int(float(nonZeroCount_s))  # tolerate "12.0"
        currentMeasure = float(currentMeasure_s)
    except ValueError:
        return None

    return {
        "maxForce": maxForce,
        "targetForce": targetForce,
        "forceError": forceError,
        "nonZeroCount": nonZeroCount,
        "currentMeasure": currentMeasure,
        "systemState": state_s,
        "raw_line": raw,
    }


class SerialRigClient:
    def __init__(self, port: str, baud: int, csv_path: str, timeout_s: float = 0.2):
        self.port = port
        self.baud = baud
        self.csv_path = csv_path
        self.timeout_s = timeout_s

        self.ser = None
        self.stop_event = threading.Event()
        self.reader_thread = None

        self._csv_file = None
        self._csv_writer = None
        self._rows_since_flush = 0

        # Persisted counter
        self._cycle_lock = threading.Lock()
        self.cycle_count = 1  # will be updated on open if file exists

    def connect(self):
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baud,
            timeout=self.timeout_s,
            write_timeout=1.0,
        )

        # Arduino boards often reset on port open.
        time.sleep(ARDUINO_RESET_WAIT_S)

        self._open_csv()

        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()

        print(f"[OK] Connected to {self.port} @ {self.baud} baud")
        print(f"[OK] Logging to: {os.path.abspath(self.csv_path)}")
        print(f"[OK] Starting cycle_count at: {self.cycle_count}")
        print("\nCommands: R=Running, I=Idle, F=Fault, Q=Quit\n")

    # ---------- CSV helpers ----------

    def _ensure_cycle_column(self):
        """
        If the CSV exists but doesn't have cycle_count in its header,
        migrate it by rewriting with the new header and adding cycle_count.
        Creates a .bak backup first.
        """
        if not os.path.isfile(self.csv_path):
            return  # nothing to migrate

        # Read header only
        with open(self.csv_path, "r", newline="", encoding="utf-8") as f:
            reader = csv.reader(f)
            try:
                existing_header = next(reader)
            except StopIteration:
                existing_header = []

        if "cycle_count" in existing_header:
            return  # already good

        # If empty file, just let header be written fresh in _open_csv
        if not existing_header:
            return

        # Backup then rewrite
        backup_path = self.csv_path + f".bak_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        os.replace(self.csv_path, backup_path)
        print(f"[INFO] Existing CSV missing cycle_count. Backed up to: {backup_path}")

        # Rewrite with new header, populate cycle_count sequentially
        cycle = 1
        with open(backup_path, "r", newline="", encoding="utf-8") as src, \
             open(self.csv_path, "w", newline="", encoding="utf-8") as dst:

            in_reader = csv.DictReader(src)
            out_writer = csv.DictWriter(dst, fieldnames=CSV_HEADER)
            out_writer.writeheader()

            for row in in_reader:
                # Carry across known fields; keep raw_line etc if present
                out_row = {h: row.get(h, "") for h in CSV_HEADER}
                out_row["cycle_count"] = cycle
                out_writer.writerow(out_row)
                cycle += 1

        print(f"[INFO] Migration complete. Rewritten CSV now includes cycle_count.")

    def _get_last_cycle_count(self) -> int:
        """
        Returns the last cycle_count in the CSV, or 0 if none.
        """
        if not os.path.isfile(self.csv_path):
            return 0

        try:
            with open(self.csv_path, "r", newline="", encoding="utf-8") as f:
                reader = csv.DictReader(f)
                if not reader.fieldnames:
                    return 0
                if "cycle_count" not in reader.fieldnames:
                    return 0

                last = 0
                for row in reader:
                    val = row.get("cycle_count", "")
                    try:
                        if val != "":
                            last = int(float(val))
                    except ValueError:
                        # ignore bad values
                        pass
                return last
        except Exception:
            return 0

    def _open_csv(self):
        # If old file exists without the new column, migrate it first
        self._ensure_cycle_column()

        file_exists = os.path.isfile(self.csv_path)

        # Determine starting cycle_count
        last = self._get_last_cycle_count()
        self.cycle_count = last + 1 if last > 0 else 1

        self._csv_file = open(self.csv_path, "a", newline="", encoding="utf-8")
        self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=CSV_HEADER)

        if not file_exists:
            self._csv_writer.writeheader()
            self._csv_file.flush()

    # ---------- lifecycle ----------

    def close(self):
        self.stop_event.set()

        if self.reader_thread and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=2.0)

        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass

        try:
            if self._csv_file:
                self._csv_file.flush()
                self._csv_file.close()
        except Exception:
            pass

        print("[OK] Closed serial + CSV")

    # ---------- serial TX/RX ----------

    def send_state_command(self, cmd: str):
        """Send a single-character command: R / I / F."""
        if not self.ser or not self.ser.is_open:
            raise RuntimeError("Serial port not connected")

        cmd = cmd.strip().upper()
        if cmd not in ("R", "I", "F"):
            raise ValueError("Command must be one of: R, I, F")

        # Send only the character (Arduino reads raw bytes).
        self.ser.write(cmd.encode("ascii"))
        self.ser.flush()
        print(f"[TX] {cmd}")

    def _reader_loop(self):
        while not self.stop_event.is_set():
            try:
                raw_bytes = self.ser.readline()
                if not raw_bytes:
                    continue

                line = raw_bytes.decode("utf-8", errors="replace")
                parsed = parse_state_line(line)
                if parsed is None:
                    # Ignore non-matching debug lines silently
                    continue

                with self._cycle_lock:
                    cycle = self.cycle_count
                    self.cycle_count += 1

                row = {
                    "timestamp_iso": now_iso(),
                    "cycle_count": cycle,  # <-- NEW COLUMN VALUE
                    "maxForce": parsed["maxForce"],
                    "targetForce": parsed["targetForce"],
                    "forceError": parsed["forceError"],
                    "nonZeroCount": parsed["nonZeroCount"],
                    "currentMeasure": parsed["currentMeasure"],
                    "systemState": parsed["systemState"],
                    "raw_line": parsed["raw_line"],
                }

                self._csv_writer.writerow(row)
                self._rows_since_flush += 1

                if self._rows_since_flush >= FLUSH_EVERY_N_ROWS:
                    self._csv_file.flush()
                    self._rows_since_flush = 0

                if PRINT_LIVE_RX:
                    print(
                        f"[RX] cycle={row['cycle_count']}, max={row['maxForce']:.3f}, "
                        f"target={row['targetForce']:.3f}, err={row['forceError']:.3f}, "
                        f"nz={row['nonZeroCount']}, I={row['currentMeasure']:.2f}, "
                        f"state={row['systemState']}"
                    )

            except serial.SerialException as e:
                print(f"[ERR] Serial exception: {e}")
                break
            except Exception as e:
                print(f"[ERR] Reader loop exception: {e}")
                time.sleep(0.1)

        # final flush on exit
        try:
            if self._csv_file:
                self._csv_file.flush()
        except Exception:
            pass


def main():
    client = SerialRigClient(SERIAL_PORT, BAUD_RATE, CSV_OUTPUT_PATH)

    try:
        client.connect()

        while True:
            user_in = input("> ").strip().upper()
            if not user_in:
                continue

            if user_in == "Q":
                break
            elif user_in in ("R", "I", "F"):
                client.send_state_command(user_in)
            else:
                print("Unknown command. Use R, I, F, or Q.")

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C received, shutting down...")
    finally:
        client.close()


if __name__ == "__main__":
    main()
