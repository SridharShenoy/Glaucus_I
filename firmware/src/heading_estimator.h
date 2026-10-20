// heading_estimator.h — heading on a wall, from gravity + gyro.
//
// THE TRICK: on a (near-)vertical hull, the accelerometer can see which way is
// "down" in the robot's own frame. That gives an ABSOLUTE heading in the hull
// plane - no drift, no magnetometer (a steel hull or nearby motors make
// magnetometers useless anyway).
//
// Robot frame: x = forward, y = left, z = out of the hull (surface normal).
// Sheet frame (see docs/sweep_algorithm.md): heading 0 = facing bow,
// +pi/2 = facing straight down.
//
// When the robot sits still, the accelerometer reads the reaction to gravity,
// i.e. a vector pointing UP. Facing down (heading +pi/2) that's -x, so
//     a_x = -g * sin(heading),   a_y = side * g * cos(heading)
//     => heading_acc = atan2(-a_x, side * a_y)
// where side = +1 starboard / -1 port (the hull-plane is mirrored between sides).
//
// Accel alone is noisy (vibration from brushes!) and gyro alone drifts, so:
//     COMPLEMENTARY FILTER = trust the gyro short-term, let gravity pull it
//     back slowly. heading += gyro*dt;  heading += alpha * (heading_acc - heading)
//
// Near the bottom of the hull the wall tilts toward horizontal and the
// in-plane part of gravity shrinks -> heading_acc gets unreliable. We scale
// alpha by how much in-plane gravity we actually see, so the filter quietly
// falls back to gyro-only there. If you spend real time on the flat bottom
// you'll need another heading reference (camera features / a hull map).
#pragma once

#include <math.h>

class HeadingEstimator {
 public:
  explicit HeadingEstimator(float side = 1.0f, float alpha = 0.02f) : side_(side), alpha_(alpha) {}

  // ax, ay, az in m/s^2; gz in rad/s (about the hull normal); dt in s.
  void update(float ax, float ay, float az, float gz, float dt) {
    constexpr float g = 9.80665f;
    const float in_plane = sqrtf(ax * ax + ay * ay);
    wall_tilt_ = atan2f(fabsf(az), in_plane);           // 0 on a vertical wall
    const float heading_acc = atan2f(-ax, side_ * ay);

    heading_ = wrap(heading_ + side_ * gz * dt);       // predict with the gyro
    const float trust = fminf(in_plane / g, 1.0f);     // 1 on a vertical wall
    heading_ = wrap(heading_ + alpha_ * trust * wrap(heading_acc - heading_));
    rate_ = side_ * gz;
  }

  void reset_from_accel(float ax, float ay) { heading_ = atan2f(-ax, side_ * ay); }

  float heading() const { return heading_; }
  float rate() const { return rate_; }
  float wall_tilt() const { return wall_tilt_; }

  static float wrap(float a) {
    constexpr float kPi = 3.14159265f, kTau = 6.2831853f;
    return a - kTau * floorf((a + kPi) / kTau);
  }

 private:
  float side_, alpha_;
  float heading_ = 0.0f, rate_ = 0.0f, wall_tilt_ = 0.0f;
};
