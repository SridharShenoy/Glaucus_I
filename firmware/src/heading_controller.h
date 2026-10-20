// heading_controller.h — the INNER loop. Runs at ~200 Hz on the MCU.
//
// Cascade (see docs/control_design.md):
//   Pi, 20 Hz:   depth error  -> target heading       (outer, slow)
//   MCU, 200 Hz: heading error -> turn rate -> wheels  (inner, fast)
//
// Why split it: the inner loop must react to every wobble and must keep
// running if Linux stutters for 50 ms. The outer loop only needs to notice
// "I've drifted 2 cm deep" - that's slow.
//
// Rule of thumb for cascades: inner loop >= 5-10x faster than outer loop,
// and tune the inner loop FIRST with the outer loop disabled.
#pragma once

#include "heading_estimator.h"

struct WheelSpeeds {
  float left;
  float right;
};

class HeadingController {
 public:
  struct Gains {
    float kp = 2.5f;             // (rad/s) per rad - same default as SweepParams.kp_heading
    float kd = 0.0f;             // add if it overshoots during pivot turns
    float max_omega = 0.8f;      // rad/s
    float track_width = 0.22f;   // m, wheel/track separation  TODO(glaucus): measure
    float max_wheel_speed = 0.3f;// m/s
  };

  explicit HeadingController(Gains g) : g_(g) {}

  // Turn rate for a heading target. Uses measured rate for the D term
  // (derivative on measurement -> no kick when the target jumps by 90 deg).
  float omega(float heading_cmd, float heading, float rate) const {
    const float err = HeadingEstimator::wrap(heading_cmd - heading);
    float w = g_.kp * err - g_.kd * rate;
    if (w > g_.max_omega) w = g_.max_omega;
    if (w < -g_.max_omega) w = -g_.max_omega;
    return w;
  }

  // Differential drive mixing. Positive omega turns toward +heading.
  // NOTE: which wheel is "left" depends on hull side + motor wiring. Verify on
  // the bench: command omega > 0 and check the heading estimate increases.
  WheelSpeeds mix(float v, float w) const {
    WheelSpeeds s{v - w * g_.track_width * 0.5f, v + w * g_.track_width * 0.5f};
    // Scale both together if saturated so the turn is preserved.
    const float m = fmaxf(fabsf(s.left), fabsf(s.right));
    if (m > g_.max_wheel_speed) {
      const float k = g_.max_wheel_speed / m;
      s.left *= k;
      s.right *= k;
    }
    return s;
  }

 private:
  Gains g_;
};
