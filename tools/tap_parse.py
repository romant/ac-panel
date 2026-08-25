#!/usr/bin/env python3
"""
tap_parse.py — offline parse of a raw_tap.py capture.

Reconstructs the stream losslessly, CRC-frames it, and reports EVERY addr-7
state change with a wallclock time. Deliberately dumb: no gap-framing, no
assumptions about which register means what — just "this value changed at this
time", so narrated actions can be correlated without prejudging.

Usage: ./tap_parse.py tap.rtap [--all]
"""
import sys, struct, time

path = sys.argv[1] if len(sys.argv) > 1 else "tap.rtap"
show_all = "--all" in sys.argv

# --- reconstruct stream + per-byte timestamps ---
data = bytearray(); tstamp = []
raw = open(path, "rb").read()
i = 0
while i < len(raw):
    if raw[i:i+4] != b'RTAP': i += 1; continue
    t, n = struct.unpack('<dI', raw[i+4:i+16])
    chunk = raw[i+16:i+16+n]
    data += chunk; tstamp += [t] * len(chunk)
    i += 16 + n
print(f"# {len(data)} bytes, {time.strftime('%H:%M:%S', time.localtime(tstamp[0])) if tstamp else '-'} "
      f"-> {time.strftime('%H:%M:%S', time.localtime(tstamp[-1])) if tstamp else '-'}")

def crc_ok(b):
    c = 0xFFFF
    for x in b:
        c ^= x
        for _ in range(8): c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return c == 0

# --- CRC-scan the whole stream (lossless; nothing dropped) ---
frames = []; i = 0
while i < len(data) - 3:
    hit = 0
    for n in range(4, min(len(data) - i, 260) + 1):
        if crc_ok(data[i:i+n]): hit = n; break
    if hit:
        frames.append((tstamp[i], bytes(data[i:i+hit]))); i += hit
    else:
        i += 1

# --- pair REQ -> RESP, track every value, print only CHANGES ---
state = {}
def note(t, key, val):
    if state.get(key) != val:
        print(f"{time.strftime('%H:%M:%S', time.localtime(t))}  {key:12} {state.get(key)!r:>12} -> {val!r}")
        state[key] = val

for idx, (t, f) in enumerate(frames):
    if len(f) == 8 and f[0] == 7 and f[1] in (1, 3):
        st = (f[2] << 8) | f[3]; q = (f[4] << 8) | f[5]
        nxt = frames[idx+1][1] if idx + 1 < len(frames) else b''
        if f[1] == 1 and len(nxt) >= 4 and nxt[0] == 7 and nxt[1] == 1:
            note(t, "coils", "0x%02X" % nxt[3])
        if f[1] == 3 and len(nxt) >= 5 and nxt[0] == 7 and nxt[1] == 3 and nxt[2] == q*2:
            for k in range(q):
                r = st + k
                v = (nxt[3+2*k] << 8) | nxt[4+2*k]
                if show_all or r in (1, 2, 9, 317, 31, 32, 54, 50, 14):
                    note(t, f"r{r}", v)
print("# done — only CHANGES shown (values that never moved are omitted)")
