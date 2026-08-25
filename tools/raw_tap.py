#!/usr/bin/env python3
"""
raw_tap.py — LOSSLESS + TIMESTAMPED RS485 capture. Stdlib only.

Writes self-describing records so a narrated action can be pinned to the exact
byte on the wire:

    record = b'RTAP' | float64 wallclock | uint32 len | raw bytes

Nothing is framed, filtered or dropped at capture time — parsing happens offline.
Usage: ./raw_tap.py /dev/cu.usbserial-XXXX [seconds] [out.rtap]
"""
import os, sys, termios, select, time, struct

dev = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbserial-BG04IIPO"
dur = float(sys.argv[2]) if len(sys.argv) > 2 else 3600
out = sys.argv[3] if len(sys.argv) > 3 else "tap.rtap"

fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
a = termios.tcgetattr(fd)
a[0] = 0; a[1] = 0; a[3] = 0
a[2] = termios.CS8 | termios.PARENB | termios.CLOCAL | termios.CREAD   # 19200 8E1
a[4] = a[5] = termios.B19200
a[6][termios.VMIN] = 0; a[6][termios.VTIME] = 0
termios.tcsetattr(fd, termios.TCSANOW, a)
termios.tcflush(fd, termios.TCIFLUSH)

f = open(out, "wb"); end = time.monotonic() + dur; total = 0
try:
    while time.monotonic() < end:
        r, _, _ = select.select([fd], [], [], 0.05)
        if r:
            d = os.read(fd, 1024)
            if d:
                f.write(b'RTAP' + struct.pack('<dI', time.time(), len(d)) + d)
                f.flush()          # greppable/parseable while still running
                total += len(d)
finally:
    f.close(); os.close(fd)
    print(f"wrote {total} bytes to {out}")
