#!/usr/bin/env python3
"""Download last capture from BattleBot's USB serial and save as CSV.

Usage: python3 scripts/download_log.py /dev/ttyACM0 capture.csv
Dependency: pip install pyserial
"""
import argparse
import re
import serial


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="e.g. /dev/ttyACM0 or COM5")
    parser.add_argument("output", nargs="?", default="capture.csv")
    args = parser.parse_args()

    with serial.Serial(args.port, 115200, timeout=3, write_timeout=3) as port:
        port.reset_input_buffer()
        port.write(b"DUMP\n")
        port.flush()
        size = None
        for _ in range(25):
            line = port.readline().decode("utf-8", errors="replace").strip()
            print(line)
            match = re.fullmatch(r"BBLOG_BEGIN (\d+)", line)
            if match:
                size = int(match.group(1))
                break
        if size is None:
            raise RuntimeError("No BBLOG_BEGIN response. Is the robot disarmed and the log saved?")

        with open(args.output, "wb") as output:
            left = size
            while left:
                block = port.read(min(4096, left))
                if not block:
                    raise TimeoutError(f"USB transfer incomplete: {left} bytes missing")
                output.write(block)
                left -= len(block)

        print(f"Saved {size} bytes to {args.output}")


if __name__ == "__main__":
    main()
