// Host test for AUTO fan band logic. Build+run: g++ -std=c++17 ac_auto_fan_test.cpp -o /tmp/t && /tmp/t
// Probes are derived from the tuning constants (+/- EPS across each edge), so retuning the header
// retunes the test with it; what's pinned is the hysteresis STRUCTURE, not the specific temperatures.
#include "ac_auto_fan.h"
#include <cassert>

int main() {
  const float EPS = 0.1f;
  const uint32_t T = 1000000;          // a "now" well past any dwell
  uint32_t lc;

  // rising
  lc = 0; assert(auto_fan_speed(AF_MED_UP - EPS,  AF_LOW, T, &lc, AF_DWELL_MS) == AF_LOW);   // holds below MED_UP
  lc = 0; assert(auto_fan_speed(AF_MED_UP,        AF_LOW, T, &lc, AF_DWELL_MS) == AF_MED);   // LOW -> MED
  lc = 0; assert(auto_fan_speed(AF_HIGH_UP + 1.0f, AF_LOW, T, &lc, AF_DWELL_MS) == AF_HIGH); // LOW -> HIGH (big jump)
  lc = 0; assert(auto_fan_speed(AF_HIGH_UP,       AF_MED, T, &lc, AF_DWELL_MS) == AF_HIGH);  // MED -> HIGH

  // hysteresis dead-bands: hold, don't drop
  lc = 0; assert(auto_fan_speed((AF_MED_UP + AF_HIGH_UP) * 0.5f, AF_MED, T, &lc, AF_DWELL_MS) == AF_MED);  // MED holds mid-band
  lc = 0; assert(auto_fan_speed(AF_HIGH_DN + EPS, AF_HIGH, T, &lc, AF_DWELL_MS) == AF_HIGH); // HIGH holds above HIGH_DN
  lc = 0; assert(auto_fan_speed(AF_MED_DN + EPS,  AF_MED,  T, &lc, AF_DWELL_MS) == AF_MED);  // MED holds above MED_DN

  // falling
  lc = 0; assert(auto_fan_speed(AF_MED_DN - EPS,  AF_MED,  T, &lc, AF_DWELL_MS) == AF_LOW);  // MED -> LOW
  lc = 0; assert(auto_fan_speed(AF_HIGH_DN - EPS, AF_HIGH, T, &lc, AF_DWELL_MS) == AF_MED);  // HIGH -> MED
  lc = 0; assert(auto_fan_speed(AF_MED_DN - 0.5f, AF_HIGH, T, &lc, AF_DWELL_MS) == AF_LOW);  // HIGH -> LOW straight

  // dwell
  lc = T - 1000;              assert(auto_fan_speed(AF_HIGH_UP + 1.0f, AF_MED, T, &lc, AF_DWELL_MS) == AF_MED);  // blocked
  lc = T - (AF_DWELL_MS + 1); assert(auto_fan_speed(AF_HIGH_UP + 1.0f, AF_MED, T, &lc, AF_DWELL_MS) == AF_HIGH); // allowed

  // out-of-range cur (e.g. uninitialised state) resets to MED before banding
  lc = 0; assert(auto_fan_speed(AF_MED_UP - EPS, 99, T, &lc, AF_DWELL_MS) == AF_MED);  // resets, then MED holds
  lc = 0; assert(auto_fan_speed(AF_MED_DN - EPS, 0,  T, &lc, AF_DWELL_MS) == AF_LOW);  // resets, then MED -> LOW

  // mid-band jump: LOW -> MED (not HIGH) when e clears MED_UP but not HIGH_UP
  lc = 0; assert(auto_fan_speed(AF_MED_UP + 0.5f, AF_LOW, T, &lc, AF_DWELL_MS) == AF_MED);

  // dwell boundary: exactly dwell_ms elapsed must already be allowed (edge is >=, not >)
  lc = T - AF_DWELL_MS;
  assert(auto_fan_speed(AF_HIGH_UP + 1.0f, AF_MED, T, &lc, AF_DWELL_MS) == AF_HIGH);

  // *last_change is untouched on a hold (no state change), and stamped to `now` on a change
  lc = 12345; auto_fan_speed((AF_MED_UP + AF_HIGH_UP) * 0.5f, AF_MED, T, &lc, AF_DWELL_MS); assert(lc == 12345);
  lc = 0;     auto_fan_speed(AF_MED_UP,        AF_LOW, T, &lc, AF_DWELL_MS); assert(lc == T);

  return 0;
}
