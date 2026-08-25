#!/usr/bin/env python3
# Passive Modbus-RTU sniffer for a USB-RS485 dongle. Stdlib only (termios), no pyserial.
# ms timestamps + per-frame gap (dt) so REQ->RESP turnaround is visible.
# Usage: mb_sniff.py [/dev/cu.usbserial-XXXX] [seconds]
import os, sys, glob, time, termios, select

def crc16(b):
    c = 0xFFFF
    for x in b:
        c ^= x
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return c

def open_port(dev):
    fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    a = termios.tcgetattr(fd)
    a[0] = 0; a[1] = 0; a[3] = 0
    a[2] = termios.CS8 | termios.PARENB | termios.CLOCAL | termios.CREAD
    a[4] = a[5] = termios.B19200
    a[6][termios.VMIN] = 0; a[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, a)
    termios.tcflush(fd, termios.TCIFLUSH)
    return fd

def label(f):
    fc = f[1]
    if len(f) == 8 and fc in (1, 2, 3, 4):
        st = (f[2] << 8) | f[3]; qty = (f[4] << 8) | f[5]
        tag = "  <-- HANDSHAKE reg15" if (fc == 3 and st == 15) else ""
        return f"REQ  fc{fc} start={st:<4} qty={qty}{tag}"
    if fc in (3, 4) and len(f) >= 5 and f[2] == len(f) - 5:
        return f"RESP fc{fc} bc={f[2]}  <== SLAVE REPLY"
    if fc == 1 and len(f) >= 5:
        return f"RESP fc1 bc={f[2]} data={' '.join('%02X'%x for x in f[3:3+f[2]])}  <== SLAVE REPLY"
    return f"fc{fc} len={len(f)}"

def main():
    dev = sys.argv[1] if len(sys.argv) > 1 else next(iter(glob.glob("/dev/cu.usbserial*") or [""]), "")
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else 20
    if not dev or not os.path.exists(dev):
        sys.exit(f"no serial port ({dev!r}) - plug in the dongle")
    print(f"# {dev} @19200 8E1 {dur:.0f}s   cols: time(ms)  dt=gap-since-prev-frame(ms)  decode")
    fd = open_port(dev)
    buf = bytearray(); last = time.monotonic(); bstart = None; prev_emit = None; end = time.monotonic() + dur
    def flush():
        nonlocal prev_emit
        ts = bstart
        i = 0
        while len(buf) - i >= 4:
            hit = 0
            for n in range(4, min(len(buf) - i, 260) + 1):
                if crc16(buf[i:i+n]) == 0:
                    hit = n; break
            if not hit:
                i += 1; continue
            fr = bytes(buf[i:i+hit])
            if fr[0] == 7:
                dt = "" if prev_emit is None else f"dt={ (ts - prev_emit)*1000:7.1f}ms"
                wall = time.strftime('%H:%M:%S', time.localtime(time.time())) + f".{int((ts*1000)%1000):03d}"
                print(f"{wall}  {dt:>14}  {label(fr)}  | {' '.join('%02X'%x for x in fr)}")
                prev_emit = ts
            i += hit
        del buf[:]
    while time.monotonic() < end:
        r, _, _ = select.select([fd], [], [], 0.001)
        if r:
            d = os.read(fd, 512)
            if d:
                if not buf: bstart = time.monotonic()
                buf += d; last = time.monotonic()
        elif buf and (time.monotonic() - last) > 0.0012:   # 1.2ms idle => frame gap (separates REQ/RESP)
            flush()
    flush(); os.close(fd)

if __name__ == "__main__":
    assert crc16(bytes.fromhex("07030A006F004D0000000500" + "4EC9B3")) == 0
    main()
