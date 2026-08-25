# ac-panel

The wall-bus protocol between a **Temperzone TZT-100** thermostat and the **UC8** indoor unit
controller it is wired to, reverse-engineered by passive observation — plus a tested reference
implementation of the thermostat side.

There is no public documentation for this protocol. This repository is the description, the raw
captures it was derived from, the instrument used to take them, and about 300 lines of C++ that
answers the unit controller convincingly enough to run an air conditioner.

📄 **[docs/protocol.md](docs/protocol.md)** — the register map, the handshake, the coils, the
control model.
📡 **[docs/sniffing.md](docs/sniffing.md)** — how to tap the bus and read a capture.

## The short version

RS485, Modbus RTU, 19200 8E1. And the roles are the reverse of what most people assume:

**The air conditioner is the master. The thermostat is the slave, at address 7.** The unit
controller only ever *reads*. Nothing is ever commanded down that wire — the thermostat publishes
its decisions as registers and coils, and the unit obeys them.

Two things follow. Replacing the thermostat needs no commands at all, only convincing answers. And
the five coils are not an abstraction: they are the five relay outputs a heat-pump thermostat has —
fan low, fan medium, fan high, compressor, reversing valve.

| Register | Meaning | Encoding |
|---|---|---|
| r1 | Mode | `0` off · `2` heat · `3` cool |
| r9 | Fan | `1` low · `2` med · `3` high · `4` auto |
| r16 | Setpoint | `°C = r16 / 2 − 10` |
| r354 | Room temperature | `°C = r354 / 50` |

Register 31 is a gate: answer it wrong, or answer it with zeros, and the master abandons the
handshake and restarts it forever.

## ⚠️ Safety

### No warranty, no liability

This is a description of what was found on one air conditioner, published as
documentation and reference code. It is **not a product**. There is no support,
no maintenance commitment, and no guarantee that any of it is correct for your
unit.

The material is provided **"as is", without warranties or conditions of any
kind**, express or implied. In no event shall the author be liable for any
claim, damages, equipment failure, injury, or other liability arising from its
use. Sections 7 and 8 of the [LICENSE](LICENSE) carry this as an operative term
of the grant.

Replacing the thermostat moves compressor protection into your code. **Nothing downstream will
catch a mistake.**

On the TZT-100, the compressor restart delay and minimum run time are DIP switches on the
thermostat itself — not settings in the air conditioner. Unplug the thermostat and those
protections leave with it. A replacement that short-cycles the compressor, or swings the reversing
valve while the compressor is loaded, will damage equipment.

The timing constants in `include/ac_control.h` are inherited from the DIP settings of one
particular unit. **They are not a specification of your unit.** Read
[the protection section](docs/protocol.md#compressor-protection-is-the-thermostats-job) before
using them.

This work involves wiring at a thermostat backplate, adjacent to mains-powered equipment, and will
likely void your warranty. It is published as a description of what was found, not as an
instruction to do it.

## What's here

```
docs/         The protocol, and how to observe it
include/      Reference implementation — Modbus slave + thermostat logic (pure C++17, no deps)
tests/        Host tests for both. make test
tools/        RS485 capture and parsing scripts (Python 3, stdlib only)
captures/     The raw bus recordings the protocol description was derived from
```

`include/` is framework-free and host-testable on purpose:

| File | |
|---|---|
| `ac_modbus.h` | Modbus RTU slave: CRC, framing, bounded buffers, the silence rule |
| `ac_control.h` | Thermostat logic: hysteresis, anti-short-cycle, minimum run, valve hold, auto changeover, and the wire encodings |
| `ac_auto_fan.h` | Fan speed banding with hysteresis and dwell |

## Run the tests

```bash
make -C tests test
```

Plain `assert`, no framework, no dependencies — `g++ -std=c++17` and nothing else. The Modbus
suite validates the CRC against two request frames captured from real hardware, so it is pinned
against the bus rather than against itself.

## Read a capture

```bash
python3 tools/tap_parse.py captures/2026-07-17-tzt-controlled-experiment.rtap
```

That capture is a controlled session on the original thermostat with one control moved at a time;
it is what the register map was derived and validated from.

## Status

The map is verified against 1,331 paired samples and confirmed against physically observed
behaviour — screen readings, airflow, hot and cold air. A replacement panel built on this has been
running an apartment's air conditioning continuously.

Several things remain genuinely unknown, and
[they are listed rather than papered over](docs/protocol.md#not-established): what r317 is *for*,
whether the unit acts on r1 or r16 at all, the meaning of r353, and the identity of a second device
the master polls every 15 seconds that has never once replied.

## Acknowledgements

Reverse-engineered by a human with LLM assistance (Claude, Gemini). All claims
marked verified were confirmed against physical hardware; the rest are listed
under [Not established](docs/protocol.md#not-established).

## Licence

[Apache-2.0](LICENSE). Applies to the code, the documentation and the captures.

Temperzone, TZT-100 and UC8 are trademarks of Temperzone Ltd. This project is not affiliated with
or endorsed by Temperzone. The protocol description here was derived by passive observation of a
bus in the author's own home, for the purpose of interoperating with equipment the author owns. No
vendor documentation is redistributed.
