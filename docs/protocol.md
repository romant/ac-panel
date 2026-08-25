# The TZT-100 ↔ UC8 wall-bus protocol

The undocumented protocol between a Temperzone TZT-100 wall thermostat and the UC8 indoor unit
controller it is wired to. Derived by passive observation of the bus and validated against 1,331
paired samples from a controlled capture. Everything below is confirmed against something
physically observed — a reading on the thermostat's own screen, airflow at the vent, or hot or
cold air.

This is the *wall* bus. The UC8 also has a separate BMS port (A1/B1) on which it is itself a
Modbus slave at address 44, with a documented register set. That is a different physical bus at
the unit end, and none of this page applies to it.

## Bus

| | |
|---|---|
| Physical | RS485, 2-wire, half-duplex |
| Framing | Modbus RTU |
| Line | 19200 baud, 8 data bits, **even** parity, 1 stop bit |
| Master | The UC8 indoor unit controller |
| Slave | The TZT-100, at **address 7** |
| Poll tick | ~16 ms |
| Response timeout | ~36 ms |

At the TZT's backplate the pair is labelled `Coms B` / `Coms A`; on the unit here they are the
black and green cores respectively. Two further cores carry 12 V and 0 V. See
[backplate-wiring.jpg](backplate-wiring.jpg).

## Roles

**The unit controller is the master. The thermostat is the slave.** The master only ever *reads*
the thermostat — function codes 03 (read holding registers) and 01 (read coils). There are no
command writes on this bus in either direction.

This inverts the usual assumption, and it is the single most important fact about the protocol:
the thermostat is the brain. It decides what the system should do and publishes that decision as
registers and coils; the unit controller reads them and energises its contactors accordingly.

Two consequences:

- **Replacing the thermostat requires no commands.** A device that answers address-7 polls
  convincingly *is* the thermostat.
- **The unit coasts on last-known state if the slave goes quiet.** The air conditioner keeps
  running with the thermostat unplugged, so a slow or missing slave is a low-risk failure mode.

## A second address on the bus

The wall bus is not point-to-point. Alongside the address-7 dialogue, the master polls
**address 53** with a single fixed request:

```
35 03 00 13 00 01 71 BB     fc03, start=19, qty=1
```

Once every 15 seconds, byte-for-byte identical every time. In the reference capture it is issued
105 times across 27 minutes — and **never answered**. There is no valid frame from address 53 in
the capture at any length.

The reading this supports is that the unit controller scans for an optional accessory that is not
fitted here. What that device is, and what register 19 holds on it, is unknown. It is noted because
anything answering address 7 shares the bus with these polls and must ignore them — see the
resynchronisation note under [implementing a replacement slave](#implementing-a-replacement-slave).

## Handshake

On startup the master walks a fixed sequence before settling into steady-state polling:

```
reg 15 → reg 31 → reg 54 → coils → reg 353 → reg 317 → reg 14 → reg 1 → reg 50
```

A valid reply to **reg 15** advances the master to **reg 31**. Register 31 is the gate: the master
requires a valid 10-register reply there.

⚠️ **Being silent on reg 31 — or answering it with zeros — makes the master abandon the handshake
and return to reg 15 indefinitely.** Zero is not a neutral default; it is a value with meaning.
An address you do not serve must produce *silence*, not a zero-filled reply.

The observed reg-31 block, `fc03 start=31 qty=10`:

```
1, 2, 0, 0, 0, 0, 1, 4, 1, 0
```

`REQ start=15` reappearing during steady state means the slave has lost the handshake. It is the
best available health signal.

## Registers

| Register | Meaning | Encoding |
|---|---|---|
| **r1** | Mode | `0` off · `2` heat · `3` cool · `4` seen but unidentified |
| **r2** | On/off | `1` = running |
| **r9** | Fan setting | `1` low · `2` med · `3` high · `4` auto |
| **r16** | Setpoint | `°C = r16 / 2 − 10` — 0.5 °C steps |
| **r317** | Setpoint ∓ 0.5 °C | `heat: r16 − 1` · `cool: r16 + 1` · `0` when the coils are clear |
| **r354** | Room temperature | `°C = r354 / 50` |
| r353 | A second temperature, ÷50 | Coil or outdoor; unidentified |
| r14, r15, r19, r50, r54 | Static here | `62`, `111`, `78`, `0`, `235` |
| **reg 31 block** | Handshake gate | See above |

The room temperature is on the bus because the UC8 has no room sensor of its own and reports room
temperature onward to a BMS. It must be told.

## Coils — the five relay outputs

`fc01 start=0 qty=5`. These are not an abstraction: they are the physical relay terminals the
thermostat closes, matching its own G1/G2/G3 · Y · O/B terminal block.

| Bit | Mask | Terminal | Output |
|---|---|---|---|
| 0 | `0x01` | G1 | Fan low |
| 1 | `0x02` | G2 | Fan medium |
| 2 | `0x04` | G3 | Fan high |
| 3 | `0x08` | Y | **Compressor** |
| 4 | `0x10` | O/B | **Reversing valve** (set = heat) |

Two behaviours that will bite a replacement device:

- **Bit 4 is a valve, not a mode mirror.** While the compressor is off it *holds its last
  position*. It swings to the cooling position only when a cooling compressor call begins.
- **The fan coil bits lag r9 by about 5–6 s.** r9 is the setting; the coil bit is the relay
  following it. They do not change atomically.

## Control model

To drive the unit, serve:

| Intent | r1 | r2 | r9 | Coils |
|---|---|---|---|---|
| Off | — | 0 | — | `0x00` |
| Cool, fan high, calling | 3 | 1 | 3 | `0x0C` — fan high + compressor, valve clear |
| Cool, fan high, idle | 3 | 1 | 3 | `0x14` — fan high only |
| Heat, fan high, calling | 2 | 1 | 3 | `0x1C` — fan high + compressor + valve set |
| Heat, fan high, idle | 2 | 1 | 3 | `0x14` |

Coil bit 3 is the thermostat's own decision, made by comparing room temperature (r354) against
setpoint (r16). That comparison is the control loop a replacement device must own.

## Compressor protection is the thermostat's job

The coils are the thermostat's outputs, so the protection timings are enforced by the thermostat,
not by the unit controller. On the TZT-100 they are DIP switches: `Sw5` sets the compressor
restart delay, `Sw7` the minimum run time. **Replacing the thermostat transfers this
responsibility, and nothing downstream will catch a mistake.**

⚠️ Do not assume the UC8 protects the compressor. That is unverified.

Three hazards a replacement must handle:

1. **Short-cycling.** Any stop — setpoint satisfied, powered off, or mode changed — must start a
   lockout before the compressor coil may be set again.
2. **Oil return.** Once started, the compressor must run for a minimum time even if the setpoint
   is immediately satisfied.
3. **Reversing valve under load.** Bit 4 must never move while bit 3 is set. The installer manual
   also requires the valve to stay energised for 120 minutes after a heating call ends, to avoid a
   decompression hiss and the wear that comes with it.

The values in [`../include/ac_control.h`](../include/ac_control.h) are inherited from the DIP
switch settings on the unit they were taken from, and match restart intervals of roughly four
minutes measured on that unit. **The compressor's actual requirement is a specification of your
unit, not of this protocol.** Treat them as conservative starting points.

## There is no auxiliary heat relay

The TZT-100 has exactly five relays and `Sw1` allocates them. With `Sw1` on — three-speed fan —
they are fan low, fan medium, fan high, compressor and reversing valve, with none left over. The
manual states that the W2 auxiliary-heat function operates only in single-fan-speed heat-pump mode
(`Sw1` off).

An `AUX` indicator can still appear on the thermostat's screen when its internal upstage timers
trip. It has no signature on the bus and, with `Sw1` on, no hardware output behind it.

## Deliberate deviations in this implementation

One register where the reference implementation knowingly serves something the real thermostat
would not:

**r16 in heat mode** is published as `FAUX_AUX_C` (35 °C) rather than the user's setpoint, to open
a large setpoint-to-room gap in the hope that the inverter reads it as a request for maximum
capacity. The control loop still drops the compressor coil at the *real* setpoint, so this changes
how hard the unit heats, never when it stops. r317 is computed from the published r16 and so rides
the same value.

⚠️ **This is unproven.** There is no evidence on the bus that the unit acts on r16 at all. If it
does not, this is a harmless no-op. If it does, the only thing bounding the result is the
compressor cut-out, which the thermostat owns. Cooling is honest — the real setpoint is published.

## Not established

- **What r317 is for.** Its arithmetic is solid across three setpoints and both modes. Its meaning
  is not. "Compressor switching threshold" is the tidiest story and the data partly contradicts
  it: in one heat bucket the room sat 2 °C below the implied threshold while 221 of 261 samples
  showed fan-only with no compressor. The formula is safe to serve; the story is not safe to rely
  on.
- **Whether the unit acts on r1 or r16 at all.** The coils fully specify the actions, so mode and
  setpoint are redundant for control. Defrost scheduling and inverter capacity modulation are
  plausible and untested.
- **r9 = 4.** Seen while cycling the fan control, assumed to be auto. Unconfirmed against the
  screen.
- **r353.** A temperature, ÷50, around 12.5 °C in the observed captures. Coil or outdoor, unknown.
- **r14 / r15 / r19 / r50 / r54.** Never observed changing. Whether they are genuinely constant or
  merely untouched is unknown.
- **The device at address 53**, which the master polls every 15 seconds and which never answers.
  See [above](#a-second-address-on-the-bus).

## Implementing a replacement slave

- **Only one device may answer address 7.** The original thermostat must be physically
  disconnected; two slaves at the same address will collide.
- **Seed the full register set, including the reg-31 block, before going on the bus.** An
  incomplete set never completes the handshake.
- **Use a bounded responder.** This is a busy multi-address bus, and a stock Modbus server
  implementation with an unbounded receive buffer will exhaust memory within about a minute. It
  also needs to answer fc01, which many do not.
- **Answer one request per call and discard the rest of the buffer.** The remaining bytes are the
  master's traffic to other slaves, and keeping them risks a false CRC match across a frame
  boundary.
- **Drive the transceiver's direction pin from hardware**, not from software timing.
- A ~1 ms service interval is comfortably inside the master's ~36 ms timeout.

[`../include/ac_modbus.h`](../include/ac_modbus.h) is a working responder in about 100 lines.
