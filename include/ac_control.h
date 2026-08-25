#pragma once
#include <cstdint>
#include <cmath>
#include "ac_auto_fan.h"

// Thermostat logic for the TZT twin. Pure C++, no framework types: the caller passes live
// UI/room state into tick() and copies the returned registers and coils onto the wire.
//
// The TZT is unplugged, so compressor protection is the caller's responsibility, not the
// air conditioner's. These constants are the knobs.

constexpr uint32_t MIN_RUN_MS    = 120000;    // minimum run after a start (oil return)
constexpr uint32_t ANTI_CYCLE_MS = 240000;    // lockout after a stop (anti-short-cycle)
constexpr uint32_t VALVE_HOLD_MS = 7200000;   // reversing-valve hold after a heat call
constexpr uint32_t PURGE_MS      = 60000;     // low-fan purge after a stop, when OFF
constexpr float    FAUX_AUX_C    = 35.0f;     // setpoint published in HEAT; see docs/protocol.md
constexpr float    HYST_ON       = 0.5f;      // start this far past setpoint
constexpr float    HYST_OFF      = 0.2f;      // stop this far past setpoint
constexpr float    DEADBAND_C    = 1.5f;      // AUTO changeover neutral zone
constexpr int      MODE_COOL = 0, MODE_HEAT = 1, MODE_AUTO = 2;

// Trust boundary on the room reading: outside this is a broken sensor, not a room.
constexpr float    ROOM_MIN_C = 0.0f, ROOM_MAX_C = 50.0f;
// Age of the last push to us. Only a fallback: a change-only feed makes a still room and a
// dead sensor look identical, so prefer the sensor's own reported age where available.
constexpr uint32_t ROOM_STALE_MS = 1800000;
// The sensor's real age, in seconds since it last reported.
constexpr float    ROOM_AGE_MAX_S = 900.0f;

// Is the room reading usable? Decides whether the thermostat regulates or degrades to manual.
// Unsigned subtraction keeps the age millis()-wrap safe.
inline bool room_reading_ok(bool has_state, float v, uint32_t now, uint32_t last_ms, bool api_up,
                            bool have_age, float age_s) {
  if (!has_state || std::isnan(v) || v <= ROOM_MIN_C || v >= ROOM_MAX_C || !api_up) return false;
  if (have_age) return age_s >= 0.0f && age_s < ROOM_AGE_MAX_S;
  return last_ms != 0 && (now - last_ms) < ROOM_STALE_MS;
}

// Wire encoding. Protocol facts, not tunables -- see docs/protocol.md.
constexpr float    R16_OFFSET = 10.0f, R16_SCALE = 2.0f;   // r16   = (C + 10) * 2
constexpr float    R354_SCALE = 50.0f;                     // r354  = C * 50
constexpr uint16_t R9_AUTO = 4;
constexpr uint16_t R1_OFF = 0, R1_HEAT = 2, R1_COOL = 3;
constexpr uint16_t R317_DELTA = 1;                         // r317 = r16 -/+ 1
constexpr uint8_t  COIL_FAN_LOW = 0x01, COIL_FAN_MED = 0x02, COIL_FAN_HIGH = 0x04,
                   COIL_COMPRESSOR = 0x08, COIL_VALVE = 0x10;

struct AcOut {
  uint8_t  coils = 0;           // bit0/1/2 fan L/M/H, bit3 compressor, bit4 reversing valve
  uint16_t r1 = 0, r2 = 0;      // mode, running flag
  uint16_t r9 = 0;              // fan setting
  uint16_t r16 = 0, r354 = 0;   // published setpoint, room temp
  uint16_t r317 = 0;
};

class ThermostatController {
 public:
  bool     compressor = false;
  bool     valve = false;               // energised = heat
  uint32_t comp_on_at = 0;
  uint32_t comp_off_at = 0;
  uint32_t heat_call_ended_at = 0;
  int      last_mode = -1;              // last EFFECTIVE mode driven
  int      auto_dir = MODE_COOL;        // AUTO's chosen direction, held through the dead band
  bool     was_on = false;
  int      auto_fan = AF_MED;
  uint32_t auto_fan_at = 0;
  int      fan_actual = AF_MED;         // driven speed, after floor and purge

  // One control cycle.
  //   mode: MODE_COOL / MODE_HEAT / MODE_AUTO   fan_sel: FAN_AUTO or AF_LOW/MED/HIGH
  //   room: already resolved to last-good when room_ok is false.
  AcOut tick(uint32_t now, bool on, int mode, int fan_sel,
             float room, bool room_ok, float setpoint) {
    AcOut o;

    // AUTO changeover. A direction holds through the neutral zone and only flips when the
    // room is clearly on the other side; a flip lands below as a mode change.
    int eff_mode = mode;
    if (mode == MODE_AUTO) {
      // A turn-on has no direction worth holding, so pick one.
      if (on && !was_on && room_ok) auto_dir = (room > setpoint) ? MODE_COOL : MODE_HEAT;
      if (room_ok) {
        if (room > setpoint + DEADBAND_C)      auto_dir = MODE_COOL;
        else if (room < setpoint - DEADBAND_C) auto_dir = MODE_HEAT;
      }
      eff_mode = auto_dir;
    }

    float pub_sp = (eff_mode == MODE_HEAT) ? FAUX_AUX_C : setpoint;
    o.r16  = (uint16_t) lroundf((pub_sp + R16_OFFSET) * R16_SCALE);
    o.r354 = (uint16_t) lroundf(room * R354_SCALE);
    o.r9   = (fan_sel == 0) ? R9_AUTO : (uint16_t) fan_sel;

    // Mode change drops the compressor FIRST, so the valve never moves under load.
    if (eff_mode != last_mode) {
      if (compressor) {
        comp_off_at = now;
        if (last_mode == MODE_HEAT) heat_call_ended_at = now;
      }
      compressor = false;
      last_mode = eff_mode;
    }

    if (!on) {
      o.r1 = R1_OFF; o.r2 = 0;
      if (compressor) {
        comp_off_at = now;
        if (eff_mode == MODE_HEAT) heat_call_ended_at = now;
      }
      compressor = false;
    } else {
      o.r1 = (eff_mode == MODE_COOL) ? R1_COOL : R1_HEAT;
      o.r2 = 1;

      // Decided before the fan, so the AUTO fan floor sees it this cycle.
      bool want = compressor;
      if (!room_ok) {
        want = (mode != MODE_AUTO);                // manual stays on; AUTO cannot pick blind
      } else if (eff_mode == MODE_COOL) {
        if (room > setpoint + HYST_ON)       want = true;
        else if (room < setpoint - HYST_OFF) want = false;
      } else {
        if (room < setpoint - HYST_ON)       want = true;
        else if (room > setpoint + HYST_OFF) want = false;
      }

      if (!want && compressor && comp_on_at != 0 && (now - comp_on_at) < MIN_RUN_MS)
        want = true;
      // comp_off_at = 0 reads as "stopped at t=0", so a boot serves the lockout too.
      if (want && !compressor && (now - comp_off_at) < ANTI_CYCLE_MS)
        want = false;

      if (want && !compressor) comp_on_at = now;
      if (!want && compressor) {
        comp_off_at = now;
        if (eff_mode == MODE_HEAT) heat_call_ended_at = now;
      }
      compressor = want;
    }

    int f = 0;
    if (on) {
      if (fan_sel != 0) {
        f = fan_sel;                               // manual: no banding, no floor
      } else if (!room_ok) {
        f = AF_MED;
      } else {
        int band = auto_fan_speed(std::fabs(room - setpoint), auto_fan, now, &auto_fan_at, AF_DWELL_MS);
        auto_fan = band;
        f = (compressor && band < AF_MED) ? AF_MED : band;
      }
    } else {
      if (fan_sel == 0 && comp_off_at != 0 && (now - comp_off_at) < PURGE_MS)
        f = AF_LOW;                                // purge
    }
    fan_actual = f;

    if (on && compressor) {
      valve = (eff_mode == MODE_HEAT);
    } else if (valve && heat_call_ended_at != 0 && (now - heat_call_ended_at) > VALVE_HOLD_MS) {
      valve = false;
    }

    uint8_t c = 0;
    if (f == AF_LOW) c |= COIL_FAN_LOW;
    else if (f == AF_MED) c |= COIL_FAN_MED;
    else if (f == AF_HIGH) c |= COIL_FAN_HIGH;
    if (valve) c |= COIL_VALVE;
    if (compressor) c |= COIL_COMPRESSOR;
    o.coils = c;

    // Gated on compressor demand only: purge and valve-hold leave coils set with no demand.
    o.r317 = !compressor ? 0 : (eff_mode == MODE_HEAT ? o.r16 - R317_DELTA : o.r16 + R317_DELTA);

    was_on = on;
    return o;
  }
};
