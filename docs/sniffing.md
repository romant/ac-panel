# Tapping the bus

How to observe the wall bus with a USB-RS485 dongle and a laptop. Three stdlib-only Python
scripts, no pyserial, no GUI, no firmware.

## Listen only

**Never point a Modbus master tool at this bus.** The unit controller is the master; a second
master collides with it. Everything here is receive-only.

Only one device may answer address 7, so if you are testing a replacement thermostat, the original
must be physically disconnected first.

## Wiring the dongle

Four-wire (full-duplex) dongles expose separate `TXD±` and `RXD±` pairs. The wall bus is two-wire
half-duplex. Connect bus A/B to the **`RXD+`/`RXD−`** pair and leave `TXD±` disconnected.

| Symptom | Cause |
|---|---|
| No bytes at all | Leads on the wrong pair, not contacting, or no ground reference |
| Garbage bytes (`FE FF F9 FC…`) | Right pair, polarity reversed — swap `RXD+`/`RXD−` |
| Clean frames | Correct |

Port settings are **19200 8E1** (`CS8 | PARENB`, even parity).

Ground: the A/B pair alone is sufficient where grounds are bonded through mains earth — keep the
laptop on its earthed charger. ⚠️ Do not tie the dongle's ground to the thermostat's `2` terminal.
That is power, roughly 12 V, not ground.

## Tools

| Tool | What it does | Use it when |
|---|---|---|
| [`../tools/mb_sniff.py`](../tools/mb_sniff.py) | Live decoded REQ/RESP, ms timestamps, inter-frame gap | Watching state change as you press buttons |
| [`../tools/raw_tap.py`](../tools/raw_tap.py) | Lossless timestamped raw capture | Ground truth, and anything inside a dense burst |
| [`../tools/tap_parse.py`](../tools/tap_parse.py) | Offline parser; prints only what changed | Reading a capture back |

```bash
# live decode
python3 tools/mb_sniff.py /dev/cu.usbserial-XXXX 20

# lossless capture, parsed offline
python3 tools/raw_tap.py   /dev/cu.usbserial-XXXX 300 capture.rtap
python3 tools/tap_parse.py capture.rtap

# background logger, so a capture window need not be synced to a physical action
nohup python3 -u tools/mb_sniff.py /dev/cu.usbserial-XXXX 900 > sniff.log 2>&1 &
```

Running a background logger and grepping afterwards is far easier than timing a capture window to
match plugging something in at the wall.

## Gap-framed captures drop frames

A sniffer that reconstructs frames from inter-byte silence will **silently lose frames inside a
dense burst**. On this bus the reg-31 reply lands in the tightest instant of the handshake, and
gap-framed observers miss it — which reads as "the thermostat is silent on register 31" and is
wrong.

**When it matters, dump raw and parse offline.** `raw_tap.py` writes self-describing records:

```
record = b'RTAP' | float64 wallclock | uint32 length | raw bytes
```

Nothing is framed, filtered or dropped at capture time. `tap_parse.py` then CRC-frames the stream
and reports every address-7 state change with a wallclock timestamp, making no assumptions about
what any register means.

## Reading a capture

The master polls on a ~16 ms tick with a ~36 ms response timeout. A healthy dialogue starts:

```
REQ  07 03 00 0F 00 05 B5 AC                          master reads reg 15
RESP 07 03 0A 00 6F 00 4D 00 00 00 05 00 4E C9 B3     slave identity block
REQ  07 03 00 1F 00 0A F4 6D                          reg 31 block — required
RESP 07 03 14 ...
```

`REQ start=15` reappearing during steady state means the slave lost the handshake and the master is
re-probing. It is the best health signal available.

## Captures in this repo

[`../captures/`](../captures/) holds the two recordings the protocol description was derived from.
Read them with `tap_parse.py`.

| File | Contents |
|---|---|
| `2026-07-17-tzt-controlled-experiment.rtap` | The controlled capture the register map was derived and validated from — one control moved at a time on the real thermostat |
| `2026-07-16-tzt-aux-heat-session.bin` | 593 samples with `AUX` showing on the thermostat's screen; the evidence that `AUX` has no bus signature |

## Method

The failure mode to design against: labelling a register from a capture in which it never moved.
A mode that is never changed looks like a constant. A setpoint that is never adjusted looks like
identity data. Fan bits observed only in one fan speed can be read as anything.

So:

1. **Move one variable at a time**, on the real thermostat, and narrate it.
2. **Capture losslessly**, with timestamps.
3. **Validate every rule against every sample**, not against the two you happened to read.
4. **Never treat your own replacement device's traffic as an observation.** Capture the original.
5. **State a falsifiable prediction and check it** before writing anything down.
6. **A value with two data points is unknown.** Do not invent a mapping for it.
7. **Before crediting a result to the system under test, ask what else was touching the
   instrument.** An implausible *rate* of change is the tell.
