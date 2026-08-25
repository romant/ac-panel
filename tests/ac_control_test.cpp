// Host tests for ThermostatController -- the compressor, valve and fan safety logic.
// Build+run:  make test
// Plain asserts, no framework. Time probes derive from the constants in ac_control.h, so
// retuning the header retunes the tests; what is pinned is the SAFETY STRUCTURE.
#include "ac_control.h"
#include <cassert>
#include <cstdint>

// demand helpers: room far from setpoint so the hysteresis unambiguously wants on/off.
static constexpr float COLD = 15.0f, HOT = 30.0f, SP = 22.0f;
// A fresh controller starts LOCKED OUT (comp_off_at = 0 reads as "stopped at t=0"), so tests
// that are not about the lockout begin past it.
static constexpr uint32_t T0 = ANTI_CYCLE_MS + 1000;

static void boot_applies_anti_cycle_lockout() {
  // A panel restart must not be a licence to start the compressor. The TZT does this too:
  // on power-up it "assumes that the compressor has just stopped and applies this Anti-Rapid
  // Cycle delay time before starting" (Installer Manual, Sw5). comp_off_at = 0 at construction
  // reads as "stopped at t=0", so the boot window is covered by the same guard.
  ThermostatController c;
  c.last_mode = 0;                                            // cool, so no mode-change drop
  c.tick(1000, true, 0, 0, HOT, true, SP);                    // strong demand, moments after boot
  assert(c.compressor == false);
  c.tick(ANTI_CYCLE_MS - 1, true, 0, 0, HOT, true, SP);       // still inside the boot lockout
  assert(c.compressor == false);
  c.tick(ANTI_CYCLE_MS + 1, true, 0, 0, HOT, true, SP);       // expired -> may start
  assert(c.compressor == true);
}

static void anti_short_cycle() {
  ThermostatController c;
  c.last_mode = 0;                         // cool, no mode-change drop
  c.comp_off_at = 1000;                    // stopped at t=1000
  // strong cool demand, but inside the 4-min lockout -> must stay off
  c.tick(1000 + ANTI_CYCLE_MS - 1, true, 0, 0, HOT, true, SP);
  assert(c.compressor == false);
  // lockout expired -> may start
  c.tick(1000 + ANTI_CYCLE_MS + 1, true, 0, 0, HOT, true, SP);
  assert(c.compressor == true);
}

static void min_run_time() {
  // running, setpoint just satisfied -> must keep running until MIN_RUN_MS
  ThermostatController c;
  c.last_mode = 0; c.compressor = true; c.comp_on_at = 5000;
  c.tick(5000 + MIN_RUN_MS - 1, true, 0, 0, COLD, true, SP);   // cool + cold room = satisfied
  assert(c.compressor == true);                                // held for oil return
  c.tick(5000 + MIN_RUN_MS + 1, true, 0, 0, COLD, true, SP);
  assert(c.compressor == false);                               // min run met -> drops

  // OFF overrides min run
  ThermostatController c2;
  c2.last_mode = 0; c2.compressor = true; c2.comp_on_at = 5000;
  c2.tick(5000 + 1000, false, 0, 0, COLD, true, SP);
  assert(c2.compressor == false);

  // mode change overrides min run
  ThermostatController c3;
  c3.last_mode = 0; c3.compressor = true; c3.comp_on_at = 5000;
  c3.tick(5000 + 1000, true, 1, 0, COLD, true, SP);
  assert(c3.compressor == false);
  assert(c3.last_mode == 1);
}

static void valve_never_moves_under_load() {
  // heat, running, valve energised; switch to COOL
  ThermostatController c;
  c.last_mode = 1; c.compressor = true; c.valve = true; c.comp_on_at = 1000;
  auto o = c.tick(500000, true, 0, 0, HOT, true, SP);
  assert(c.compressor == false);           // compressor dropped on the mode change...
  assert((o.coils & 0x08) == 0);           // ...bit3 clear...
  assert(c.valve == true);                 // ...and the valve did NOT move (still heat) this tick
  // valve realigns to the new mode only once the compressor restarts fresh (post-lockout)
  auto o2 = c.tick(500000 + ANTI_CYCLE_MS + 1, true, 0, 0, HOT, true, SP);
  assert(c.compressor == true);
  assert(c.valve == false);                // now cool
  assert((o2.coils & 0x10) == 0);
}

static void valve_hold_after_heat() {
  ThermostatController c;
  c.last_mode = 1;
  c.tick(T0, true, 1, 0, COLD, true, SP);                    // heat demand -> compressor + valve
  assert(c.compressor == true && c.valve == true);
  uint32_t t = T0 + MIN_RUN_MS + 1;
  c.tick(t, true, 1, 0, HOT, true, SP);                        // satisfied -> heat call ends
  assert(c.compressor == false);
  assert(c.valve == true);                                     // held right after heat ends
  c.tick(t + VALVE_HOLD_MS - 1000, true, 1, 0, HOT, true, SP); // still within hold
  assert(c.valve == true);
  c.tick(t + VALVE_HOLD_MS + 10000, true, 1, 0, HOT, true, SP);// past hold
  assert(c.valve == false);
}

static void fan_purge() {
  // OFF right after a stop, AUTO fan -> LOW for PURGE_MS
  ThermostatController c;
  c.last_mode = 0; c.comp_off_at = 2000;
  auto o = c.tick(2000 + PURGE_MS - 1, false, 0, 0, SP, true, SP);
  assert(c.fan_actual == AF_LOW);
  assert(o.coils == 0x01);                 // fan LOW only (no compressor, no valve)
  assert(o.r317 == 0);                     // compressor off -> r317 0 even though coils != 0
  auto o2 = c.tick(2000 + PURGE_MS + 1, false, 0, 0, SP, true, SP);
  assert(c.fan_actual == 0 && o2.coils == 0);

  // manual fan -> NO purge
  ThermostatController c2;
  c2.last_mode = 0; c2.comp_off_at = 2000;
  auto o3 = c2.tick(2000 + PURGE_MS - 1, false, 0, 2, SP, true, SP);
  assert(c2.fan_actual == 0 && o3.coils == 0);
}

static void faux_aux() {
  // HEAT publishes r16 = (35+10)*2 = 90, regardless of the real setpoint...
  ThermostatController c;
  c.last_mode = 1;
  auto o = c.tick(T0, true, 1, 0, HOT, true, SP);            // warm -> no compressor
  assert(o.r16 == 90);
  assert(c.compressor == false);                              // ...but the RELAY still tracks real sp
  ThermostatController c2;
  c2.last_mode = 1;
  auto o2 = c2.tick(T0, true, 1, 0, COLD, true, SP);        // cold -> compressor on
  assert(o2.r16 == 90);
  assert(c2.compressor == true);
  // COOL publishes the real setpoint: (22+10)*2 = 64
  ThermostatController c3;
  c3.last_mode = 0;
  auto o3 = c3.tick(T0, true, 0, 0, HOT, true, SP);
  assert(o3.r16 == 64);
}

static void r317_formula_and_gate() {
  // HEAT running: r317 = r16-1, riding the faked r16 (89, not the real setpoint)
  ThermostatController c;
  c.last_mode = 1;
  auto o = c.tick(T0, true, 1, 0, COLD, true, SP);
  assert(c.compressor == true);
  assert(o.r16 == 90 && o.r317 == 89);
  // COOL running: r317 = r16+1
  ThermostatController c2;
  c2.last_mode = 0;
  auto o2 = c2.tick(T0, true, 0, 0, HOT, true, SP);
  assert(c2.compressor == true);
  assert(o2.r16 == 64 && o2.r317 == 65);
  // gate: r317 == 0 during valve-hold (coils non-zero, compressor off)
  ThermostatController c3;
  c3.last_mode = 1; c3.valve = true; c3.heat_call_ended_at = 1000; c3.comp_off_at = 1000;
  auto o3 = c3.tick(1000 + PURGE_MS + 1000, false, 1, 0, SP, true, SP);  // OFF, past purge
  assert(o3.coils == 0x10 && o3.r317 == 0);
}

static void coil_assembly() {
  // heat running with cold room: compressor + valve(heat) + fan. The 7C error is well past
  // AF_HIGH_UP and the dwell has long expired at T0, so AUTO bands straight to HIGH. The
  // MED floor path (band < AF_MED while the compressor runs) is covered separately below.
  ThermostatController c;
  c.last_mode = 1;
  auto o = c.tick(T0, true, 1, 0, COLD, true, SP);
  assert(o.coils == (0x08 | 0x10 | 0x04));    // comp | valve | fan HIGH
}

static void auto_fan_floors_to_med_under_compressor_load() {
  // the actual floor branch: `band < AF_MED` while compressor runs bumps the fan up to MED.
  // Hold the compressor on via min-run while the room sits close to setpoint, so the auto-fan
  // band naturally computes to LOW (small error, no dwell fight needed).
  ThermostatController c;
  c.last_mode = MODE_HEAT; c.compressor = true; c.comp_on_at = 1000;
  c.auto_fan = AF_LOW;
  auto o = c.tick(1000 + 500, true, MODE_HEAT, 0, SP + 0.3f, true, SP);  // error 0.3C -> bands LOW
  assert(c.compressor == true);                 // held by min-run despite being satisfied
  assert(c.fan_actual == AF_MED);               // floored up from LOW because compressor runs
  assert((o.coils & 0x02) != 0 && (o.coils & 0x08) != 0);
}

static void cutout_hysteresis() {
  // stop edge = HYST_OFF past setpoint. Probe at 0.35 over: inside the old 0.5 band (would have
  // held ON) but past the new 0.2 cut-out (must stop). fan_sel=2 keeps the fan out of it;
  // comp_on_at old so min-run doesn't force a hold.
  const uint32_t T = 10000000;
  ThermostatController h; h.last_mode = 1; h.compressor = true; h.comp_on_at = 1;
  h.tick(T, true, 1, 2, SP + 0.35f, true, SP);           // HEAT, 0.35 over -> off
  assert(h.compressor == false);
  ThermostatController c; c.last_mode = 0; c.compressor = true; c.comp_on_at = 1;
  c.tick(T, true, 0, 2, SP - 0.35f, true, SP);           // COOL, 0.35 under -> off
  assert(c.compressor == false);
  // still HOLDS just past setpoint but within HYST_OFF (0.1 over)
  ThermostatController h2; h2.last_mode = 1; h2.compressor = true; h2.comp_on_at = 1;
  h2.tick(T, true, 1, 2, SP + 0.1f, true, SP);
  assert(h2.compressor == true);
  // turn-on edge unchanged at HYST_ON: starts only once HYST_ON below setpoint
  ThermostatController h3; h3.last_mode = 1; h3.compressor = false;
  h3.tick(T, true, 1, 2, SP - 0.4f, true, SP);           // 0.4 below (< HYST_ON) -> still off
  assert(h3.compressor == false);
  h3.tick(T, true, 1, 2, SP - (HYST_ON + 0.05f), true, SP);
  assert(h3.compressor == true);
}

static void auto_changeover() {
  const uint32_t T = 10000000;
  // AUTO picks HEAT when clearly cold (room < sp - DEADBAND)
  ThermostatController h; h.last_mode = MODE_HEAT; h.auto_dir = MODE_HEAT;
  auto oh = h.tick(T, true, MODE_AUTO, 2, SP - 2.0f, true, SP);
  assert(h.auto_dir == MODE_HEAT && oh.r1 == 2 && h.compressor == true);
  // AUTO picks COOL when clearly hot (room > sp + DEADBAND)
  ThermostatController c; c.last_mode = MODE_COOL; c.auto_dir = MODE_COOL;
  auto oc = c.tick(T, true, MODE_AUTO, 2, SP + 2.0f, true, SP);
  assert(c.auto_dir == MODE_COOL && oc.r1 == 3 && c.compressor == true);
  // holds its direction through the neutral zone (0.5 over, within DEADBAND) — no flip.
  // was_on: already running, so no turn-on re-seed (see auto_seeds_on_turn_on).
  ThermostatController n; n.last_mode = MODE_HEAT; n.auto_dir = MODE_HEAT; n.was_on = true;
  auto on = n.tick(T, true, MODE_AUTO, 2, SP + 0.5f, true, SP);
  assert(n.auto_dir == MODE_HEAT && on.r1 == 2 && n.compressor == false);
  // flips only once past the dead band
  ThermostatController f; f.last_mode = MODE_HEAT; f.auto_dir = MODE_HEAT; f.was_on = true;
  f.tick(T, true, MODE_AUTO, 2, SP + DEADBAND_C + 0.1f, true, SP);
  assert(f.auto_dir == MODE_COOL);
  // blind AUTO can't pick a direction -> idle (never blindly runs)
  ThermostatController b; b.last_mode = MODE_COOL; b.auto_dir = MODE_COOL;
  b.tick(T, true, MODE_AUTO, 2, 0.0f, false, SP);
  assert(b.compressor == false);
  // a changeover is a mode change: drops the compressor + starts the lockout
  ThermostatController s; s.last_mode = MODE_HEAT; s.auto_dir = MODE_HEAT; s.was_on = true;
  s.compressor = true; s.comp_on_at = 1;
  s.tick(T, true, MODE_AUTO, 2, SP + 2.0f, true, SP);
  assert(s.auto_dir == MODE_COOL && s.compressor == false && s.comp_off_at == T);
}

// Turning on in AUTO must CHOOSE a direction, not inherit auto_dir's initialiser (COOL) whenever
// the room sits inside the dead band -- the normal case.
static void auto_seeds_on_turn_on() {
  const uint32_t T = 10000000;
  const float IN = DEADBAND_C * 0.5f;      // inside the dead band: too close to force a flip

  // cool-side initialiser + a room BELOW setpoint but inside the band -> must pick HEAT
  ThermostatController c;
  assert(c.auto_dir == MODE_COOL);         // the default this test exists to defeat
  c.tick(T, false, MODE_AUTO, 0, SP - IN, true, SP);                  // off
  auto o = c.tick(T + 1000, true, MODE_AUTO, 0, SP - IN, true, SP);   // turn-on
  assert(c.auto_dir == MODE_HEAT && o.r1 == R1_HEAT);

  // mirror: room ABOVE setpoint, inside the band -> COOL, even from a HEAT-biased start
  ThermostatController w; w.auto_dir = MODE_HEAT;
  w.tick(T, false, MODE_AUTO, 0, SP + IN, true, SP);
  auto ow = w.tick(T + 1000, true, MODE_AUTO, 0, SP + IN, true, SP);
  assert(w.auto_dir == MODE_COOL && ow.r1 == R1_COOL);

  // seeding is an EDGE, not a per-tick override: while it stays on the dead band still holds
  c.tick(T + 2000, true, MODE_AUTO, 0, SP + IN, true, SP);
  assert(c.auto_dir == MODE_HEAT);

  // ...and the next turn-on re-seeds from wherever the room is by then
  c.tick(T + 3000, false, MODE_AUTO, 0, SP + IN, true, SP);
  c.tick(T + 4000, true, MODE_AUTO, 0, SP + IN, true, SP);
  assert(c.auto_dir == MODE_COOL);

  // blind at turn-on: nothing to seed from, so leave the direction alone and idle
  ThermostatController b; b.auto_dir = MODE_HEAT;
  b.tick(T, false, MODE_AUTO, 0, 0.0f, false, SP);
  b.tick(T + 1000, true, MODE_AUTO, 0, 0.0f, false, SP);
  assert(b.auto_dir == MODE_HEAT && b.compressor == false);
}

static void degraded_modes() {
  // boots OFF -> everything open
  ThermostatController c;
  auto o = c.tick(0, false, 0, 0, SP, false, SP);
  assert(c.compressor == false && o.coils == 0 && o.r1 == 0 && o.r2 == 0);
  // blind (no room reading) ON in AUTO -> MED fan default, but AUTO can't pick a direction
  // blind so it idles (see auto_changeover's blind case for the compressor side of this; this
  // was previously mis-coded with mode=COOL, which tests manual-blind-forces-on instead of AUTO).
  ThermostatController c2;
  c2.last_mode = MODE_COOL;
  auto o2 = c2.tick(1000, true, MODE_AUTO, 0, 0.0f, false, SP);
  assert(c2.fan_actual == AF_MED);
  assert(c2.compressor == false);
  assert((o2.coils & 0x02) && !(o2.coils & 0x08));
}

static void manual_fan_bypasses_floor() {
  // AUTO fan floors to MED while the compressor runs (coil_assembly). MANUAL fan must NOT:
  // a user picking LOW should get LOW even with the compressor on.
  ThermostatController c;
  c.last_mode = 1;
  auto o = c.tick(T0, true, 1, 1, COLD, true, SP);   // HEAT, fan_sel=LOW, cold -> compressor on
  assert(c.compressor == true);
  assert(c.fan_actual == AF_LOW);
  assert(o.coils == (0x08 | 0x10 | 0x01));             // comp | valve | fan LOW (not floored to MED)
}

static void registers_r9_r354() {
  ThermostatController c;
  c.last_mode = 0;
  auto o = c.tick(1000, true, 0, 0, 12.34f, true, SP); // fan_sel=0 -> AUTO code
  assert(o.r9 == R9_AUTO);
  assert(o.r354 == (uint16_t) lroundf(12.34f * R354_SCALE));
  auto o2 = c.tick(1000, true, 0, 2, 12.34f, true, SP); // fan_sel=2 -> passthrough
  assert(o2.r9 == 2);
}

static void off_switch_ends_heat_call() {
  // Flipping OFF mid heat-call (distinct from the hysteresis-satisfied stop) must also start
  // the valve hold: heat_call_ended_at gets set from the `!on` branch, not just on demand-drop.
  ThermostatController c;
  c.last_mode = 1;
  c.tick(T0, true, 1, 0, COLD, true, SP);                     // heat call -> compressor + valve on
  assert(c.compressor == true && c.valve == true);
  uint32_t t = T0 + MIN_RUN_MS + 1;
  c.tick(t, false, 1, 0, COLD, true, SP);                       // OFF switch while still calling for heat
  assert(c.compressor == false);
  assert(c.valve == true);                                      // held right after the OFF
  c.tick(t + VALVE_HOLD_MS - 1000, false, 1, 0, COLD, true, SP); // still within hold
  assert(c.valve == true);
  c.tick(t + VALVE_HOLD_MS + 10000, false, 1, 0, COLD, true, SP); // past hold
  assert(c.valve == false);
}

static void millis_wrap_safe() {
  ThermostatController c;
  c.last_mode = 0;
  c.comp_off_at = UINT32_MAX - 1000;                          // stopped just before the wrap
  c.tick(500, true, 0, 0, HOT, true, SP);                     // now wrapped: elapsed 1501 < lockout
  assert(c.compressor == false);
  uint32_t later = (UINT32_MAX - 1000) + ANTI_CYCLE_MS + 1000;// wraps around, elapsed > lockout
  c.tick(later, true, 0, 0, HOT, true, SP);
  assert(c.compressor == true);

  // same wrap hazard for MIN_RUN_MS (comp_on_at), mirrored from the comp_off_at case above
  ThermostatController c2;
  c2.last_mode = 0; c2.compressor = true;
  c2.comp_on_at = UINT32_MAX - 1000;                          // started just before the wrap
  c2.tick(500, true, 0, 0, COLD, true, SP);                   // satisfied, but wrapped elapsed 1501 < MIN_RUN_MS
  assert(c2.compressor == true);
  uint32_t later2 = (UINT32_MAX - 1000) + MIN_RUN_MS + 1000;
  c2.tick(later2, true, 0, 0, COLD, true, SP);
  assert(c2.compressor == false);
}

static void manual_blind_forces_on() {
  // blind (room_ok=false) in a MANUAL mode must run unconditionally: "ON means ON", unlike AUTO
  // which idles blind (see degraded_modes). Covers both manual branches since the guard is
  // `mode != MODE_AUTO`, mode-agnostic.
  ThermostatController h; h.last_mode = 1;
  auto oh = h.tick(T0, true, 1, 0, 0.0f, false, SP);   // HEAT, blind
  assert(h.compressor == true);
  assert((oh.coils & 0x08) != 0);
  ThermostatController c; c.last_mode = 0;
  c.tick(T0, true, 0, 0, 0.0f, false, SP);             // COOL, blind
  assert(c.compressor == true);
}

static void manual_fan_blind_bypasses_med() {
  // fan_sel!=0 is checked BEFORE the room_ok branch, so a manual fan selection wins even blind
  // (not overridden to the blind-default MED).
  ThermostatController c; c.last_mode = 1;
  c.tick(T0, true, 1, 1, 0.0f, false, SP);    // HEAT, blind, fan_sel=LOW
  assert(c.fan_actual == AF_LOW);
}

static void auto_deadband_exact_boundary_holds() {
  // the changeover compares are strict `>` / `<`; landing exactly ON the DEADBAND_C edge must
  // NOT flip (only strictly past it does — see auto_changeover's "flips only once past" case).
  ThermostatController h;
  h.last_mode = MODE_HEAT; h.auto_dir = MODE_HEAT; h.was_on = true;   // running, not a turn-on
  auto o = h.tick(10000000, true, MODE_AUTO, 2, SP + DEADBAND_C, true, SP);
  assert(h.auto_dir == MODE_HEAT && o.r1 == 2);
}

static void mode_switch_while_idle_no_valve_retrigger() {
  // heat_call_ended_at is only stamped inside the mode-change branch's `if (compressor)` guard.
  // Switching mode while ALREADY idle (compressor false) must leave it — and any still-held
  // valve from an earlier heat call — untouched.
  ThermostatController c;
  c.last_mode = MODE_HEAT; c.compressor = false; c.valve = true; c.heat_call_ended_at = 500;
  c.tick(1000, true, MODE_COOL, 0, SP, true, SP);   // neutral room -> demand stays off this tick
  assert(c.last_mode == MODE_COOL);
  assert(c.heat_call_ended_at == 500);              // untouched: compressor was already off
  assert(c.valve == true);                          // still holding from the earlier heat call
}

static void mode_change_while_off_updates_last_mode() {
  // the mode-change block runs unconditionally, even while `on` is false.
  ThermostatController c;
  c.last_mode = MODE_COOL;
  auto o = c.tick(1000, false, MODE_HEAT, 0, SP, true, SP);
  assert(c.last_mode == MODE_HEAT);
  assert(c.compressor == false);
  assert(o.coils == 0);
}

static void room_reading_trust_boundary() {
  const uint32_t NOW = 10000000, FRESH = NOW - 1000;
  // the happy path
  assert(room_reading_ok(true, 21.5f, NOW, FRESH, true, false, 0.0f));
  // each rejection reason on its own
  assert(!room_reading_ok(false, 21.5f, NOW, FRESH, true, false, 0.0f));            // no state yet
  assert(!room_reading_ok(true, NAN, NOW, FRESH, true, false, 0.0f));               // NaN
  assert(!room_reading_ok(true, ROOM_MIN_C, NOW, FRESH, true, false, 0.0f));        // at/below the floor
  assert(!room_reading_ok(true, ROOM_MAX_C, NOW, FRESH, true, false, 0.0f));        // at/above the ceiling
  assert(!room_reading_ok(true, 21.5f, NOW, FRESH, false, false, 0.0f));            // HA link down
  assert(!room_reading_ok(true, 21.5f, NOW, 0, true, false, 0.0f));                 // never heard from HA
  // THE point of the guard: a plausible, in-range value that simply stopped arriving
  assert(!room_reading_ok(true, 21.5f, NOW, NOW - ROOM_STALE_MS - 1, true, false, 0.0f));
  assert(room_reading_ok(true, 21.5f, NOW, NOW - ROOM_STALE_MS + 1, true, false, 0.0f));   // just inside
  // age must survive the millis() wrap, like the compressor timers do
  const uint32_t BEFORE_WRAP = UINT32_MAX - 1000;
  assert(room_reading_ok(true, 21.5f, 500, BEFORE_WRAP, true, false, 0.0f));               // elapsed 1501 -> fresh
  assert(!room_reading_ok(true, 21.5f, BEFORE_WRAP + ROOM_STALE_MS + 1000, BEFORE_WRAP, true, false, 0.0f));

  // ---- with HA's real age, the push clock stops mattering ----
  // THE regression this fixes: a steady room stops pushing, so last_ms is ancient, but the sensor
  // reported to HA seconds ago. Old rule said dead; the age says alive.
  const uint32_t ANCIENT = 0;   // never pushed since boot
  assert(room_reading_ok(true, 21.5f, NOW, ANCIENT, true, true, 30.0f));
  assert(room_reading_ok(true, 21.5f, NOW, NOW - ROOM_STALE_MS - 1, true, true, 30.0f));
  // and it still catches a genuinely dead sensor
  assert(!room_reading_ok(true, 21.5f, NOW, FRESH, true, true, ROOM_AGE_MAX_S + 1.0f));
  assert(room_reading_ok(true, 21.5f, NOW, FRESH, true, true, ROOM_AGE_MAX_S - 1.0f));
  // -1 is the template's missing-entity sentinel: never trust it
  assert(!room_reading_ok(true, 21.5f, NOW, FRESH, true, true, -1.0f));
  // the other gates still bind regardless of age
  assert(!room_reading_ok(true, 21.5f, NOW, FRESH, false, true, 30.0f));   // HA link down
  assert(!room_reading_ok(true, NAN, NOW, FRESH, true, true, 30.0f));      // NaN reading
}

int main() {
  room_reading_trust_boundary();
  boot_applies_anti_cycle_lockout();
  anti_short_cycle();
  min_run_time();
  valve_never_moves_under_load();
  valve_hold_after_heat();
  fan_purge();
  faux_aux();
  r317_formula_and_gate();
  coil_assembly();
  auto_fan_floors_to_med_under_compressor_load();
  cutout_hysteresis();
  auto_changeover();
  auto_seeds_on_turn_on();
  degraded_modes();
  manual_fan_bypasses_floor();
  registers_r9_r354();
  off_switch_ends_heat_call();
  millis_wrap_safe();
  manual_blind_forces_on();
  manual_fan_blind_bypasses_med();
  auto_deadband_exact_boundary_holds();
  mode_switch_while_idle_no_valve_retrigger();
  mode_change_while_off_updates_last_mode();
  return 0;
}
