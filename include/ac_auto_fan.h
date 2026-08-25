#pragma once
#include <cstdint>

// AUTO fan speed banding on the error e = |room - setpoint|, in degrees C.
// Each band has a separate rising and falling edge so the speed cannot chatter,
// and a dwell time because these are physical relays.
constexpr float AF_MED_UP   = 1.0f;   // LOW  -> MED
constexpr float AF_MED_DN   = 0.8f;   // MED  -> LOW
constexpr float AF_HIGH_UP  = 2.0f;   // MED  -> HIGH
constexpr float AF_HIGH_DN  = 1.7f;   // HIGH -> MED
constexpr int   AF_LOW = 1, AF_MED = 2, AF_HIGH = 3;
constexpr int   FAN_AUTO = 0;              // a user SELECTION only, never a driven speed
constexpr uint32_t AF_DWELL_MS = 180000;   // 3 min minimum between speed changes

// Speed from e, with hysteresis + dwell. The MED floor is applied by the caller.
// Holds if <dwell_ms since the last change; updates *last_change on a change.
inline int auto_fan_speed(float e, int cur, uint32_t now, uint32_t *last_change, uint32_t dwell_ms) {
  if (cur < AF_LOW || cur > AF_HIGH) cur = AF_MED;
  int t = cur;
  if (cur == AF_LOW) {
    if (e >= AF_MED_UP) t = (e >= AF_HIGH_UP) ? AF_HIGH : AF_MED;
  } else if (cur == AF_MED) {
    if (e >= AF_HIGH_UP)     t = AF_HIGH;
    else if (e < AF_MED_DN)  t = AF_LOW;
  } else {
    if (e < AF_MED_DN)       t = AF_LOW;
    else if (e < AF_HIGH_DN) t = AF_MED;
  }
  if (t != cur && (now - *last_change) >= dwell_ms) { *last_change = now; return t; }
  return cur;
}
