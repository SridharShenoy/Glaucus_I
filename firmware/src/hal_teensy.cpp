// hal_teensy.cpp — placeholder implementations. Replace each TODO with the real
// driver once hardware is picked. Keep the stubs returning "safe" values so the
// firmware can run on a bare Teensy on the bench (useful for comms testing).
#include <Arduino.h>

#include "hal.h"

namespace hal {

namespace {
float g_depth_offset = 0.0f;
}

bool init() {
  // TODO(glaucus): Wire.begin(); imu.begin(); depth.init(); encoders; PWM pins
  return true;
}

Imu read_imu() {
  // Bench default: robot facing the bow on a vertical starboard wall
  // (reaction-to-gravity points up = +y in robot frame -> heading 0).
  return Imu{0.0f, 9.81f, 0.0f, 0.0f, 0.0f, 0.0f, true};
}

float read_depth_m() { return 0.0f - g_depth_offset; }   // TODO(glaucus)
bool depth_ok() { return true; }
void zero_depth() { g_depth_offset = 0.0f; }             // TODO: store current reading

float read_wheel_distance_m(float* left, float* right) {
  *left = 0.0f;   // TODO(glaucus): encoder ticks * (wheel circumference / ticks per rev)
  *right = 0.0f;
  return 0.5f * (*left + *right);
}

float read_motor_current_a() { return 0.0f; }
float read_adhesion() { return 1.0f; }
bool leak_detected() { return false; }

void set_wheel_speeds(float, float) {}  // TODO(glaucus)
void set_brush(bool) {}
void set_adhesion(bool) {}
void stop_all() {
  set_wheel_speeds(0, 0);
  set_brush(false);
  // adhesion intentionally left on
}

}  // namespace hal
