#!/usr/bin/env python3
"""Download BattleBot captures through USB serial (pyserial required).

  python3 scripts/download_log.py /dev/cu.usbmodem101 --list
  python3 scripts/download_log.py /dev/cu.usbmodem101 --all
  python3 scripts/download_log.py /dev/cu.usbmodem101 --id 3 run3.csv
  python3 scripts/download_log.py /dev/cu.usbmodem101 capture.csv
"""
import argparse
from pathlib import Path
import re
import serial
import time


def list_logs(port):
    port.reset_input_buffer()
    port.write(b"LIST\n")
    port.flush()
    files = {}
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        line = port.readline().decode("ascii", errors="replace").strip()
        if line == "BBLOG_LIST_END":
            return dict(sorted(files.items()))
        m = re.fullmatch(r"BBLOG_FILE (\d+) (\d+)", line)
        if m:
            files[int(m.group(1))] = int(m.group(2))
        elif line:
            print(line)
    raise RuntimeError("LIST response did not end")


def download(port, output, log_id=None):
    port.reset_input_buffer()
    command = "DUMP" if log_id is None else f"DUMP {log_id}"
    port.write((command + "\n").encode("ascii"))
    port.flush()
    size = None
    for _ in range(30):
        line = port.readline().decode("utf-8", errors="replace").strip()
        match = re.fullmatch(r"BBLOG_BEGIN (\d+)", line)
        if match:
            size = int(match.group(1))
            break
        if line:
            print(line)
            if line.startswith("BBLOG: no saved") or line.startswith("BBLOG: log not found") or line.startswith("BBLOG: finish saving"):
                raise RuntimeError(line)
    if size is None:
        raise RuntimeError(f"No BBLOG_BEGIN for {command}. Robot must be Idle and file saved.")

    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("wb") as dest:
        remaining = size
        while remaining:
            block = port.read(min(4096, remaining))
            if not block:
                raise TimeoutError(f"Incomplete serial transfer: {remaining} bytes missing")
            dest.write(block)
            remaining -= len(block)
    print(f"Saved {size} bytes -> {output}")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("port", help="Serial port e.g. /dev/cu.usbmodem101")
    p.add_argument("output", nargs="?", default="capture.csv", help="Single-log output CSV")
    group = p.add_mutually_exclusive_group()
    group.add_argument("--list", action="store_true", help="List all saved captures")
    group.add_argument("--all", action="store_true", help="Download every saved capture")
    group.add_argument("--id", type=int, help="Download this numbered capture")
    p.add_argument("--output-dir", default="logs", help="Directory for --all (default: logs)")
    args = p.parse_args()

    with serial.Serial(args.port, 115200, timeout=3, write_timeout=3) as port:
        if args.list or args.all:
            files = list_logs(port)
            if not files:
                print("No saved captures")
            for id_, size in files.items():
                print(f"capture_{id_:04d}.csv    {size} bytes")
            if args.all:
                for id_ in files:
                    download(port, Path(args.output_dir) / f"capture_{id_:04d}.csv", id_)
        else:
            download(port, args.output, args.id)


if __name__ == "__main__":
    main()
