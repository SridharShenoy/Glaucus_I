// glaucus_core/types.hpp
//
// Plain data types shared by the sweep controller, estimator and detector.
//
// DESIGN CHOICE: glaucus_core has ZERO ROS dependencies. It's plain C++17.
//   - Unit-testable with nothing but a compiler (see test/).
//   - The same logic can run in the ROS node on the Pi, in a desktop replay
//     tool, or (if we ever need to) on a beefier MCU.
//   - ROS nodes in glaucus_bot are thin wrappers: subscribe -> fill Sensors ->
//     call update() -> publish Command. Keep it that way.
//
// Mirror of sim/sweep_sim.py. Change the Python first, look at the plot, then
// port here and regenerate the parity trace (see README "Algorithm workflow").
//
// Units: SI everywhere (m, s, rad). Frame: the "unrolled hull side"
//   x = along hull (m, +toward bow), z = depth (m, +down),
//   heading 0 = facing bow, pi = facing stern, +pi/2 = facing straight down.
#pragma once

#include <cmath>
#include <optional>
#include <string>
#include <vector>

namespace glaucus {

struct SweepParams {
  double brush_width = 0.30;           // m
  double overlap = 0.20;               // fraction of brush width
  double first_lane_depth = 0.15;      // m
  double max_depth = 1.15;             // m, deepest allowed lane center

  double lane_speed = 0.15;            // m/s
  double step_speed = 0.08;            // m/s

  // Depth hold (outer loop, Pi). Output: heading offset.
  double kp_depth = 2.0;               // rad / m
  double ki_depth = 0.5;               // rad / (m*s)
  double max_depth_correction = 0.35;  // rad
  double depth_integral_limit = 0.3;   // m*s (anti-windup)

  // Heading hold (inner loop, MCU). Kept here so sim + firmware share values.
  double kp_heading = 2.5;             // (rad/s) / rad
  double max_turn_rate = 0.8;          // rad/s
  double heading_tol = 0.05;           // rad
  double depth_tol = 0.02;             // m

  // End-of-lane evidence fusion
  double w_camera = 0.6;
  double w_stall = 0.6;
  double end_threshold = 0.5;
  int end_debounce_ticks = 4;
  double max_lane_length = 15.0;       // m, safety guard

  // Early-end verification
  double verify_margin = 0.6;          // m
  double verify_distance = 0.25;       // m
  double verify_speed = 0.05;          // m/s

  // Blocked step-down recovery
  double shift_inward = 0.35;          // m
  int max_step_retries = 3;

  double lane_spacing() const { return brush_width * (1.0 - overlap); }
};

// One control tick of inputs. Filled by the ROS wrapper from topics.
struct Sensors {
  double depth = 0.0;      // m, pressure sensor (absolute)
  double heading = 0.0;    // rad, gravity-referenced (MCU complementary filter)
  double odom_ds = 0.0;    // m, wheel distance this tick (>= 0)
  double cam_edge = 0.0;   // 0..1, camera "edge/obstacle ahead" confidence
  double stall = 0.0;      // 0..1, "commanded motion but not moving"
  std::optional<double> flow_ds;  // m, optical-flow distance this tick (if any)
};

// What the Pi sends to the MCU. The MCU closes the heading loop itself.
struct Command {
  double v = 0.0;          // m/s forward
  double heading = 0.0;    // rad target heading
  bool brush_on = true;
};

enum class State { STEP_DOWN, TURN, LANE, VERIFY, SHIFT, DONE };

inline const char* to_string(State s) {
  switch (s) {
    case State::STEP_DOWN: return "STEP_DOWN";
    case State::TURN: return "TURN";
    case State::LANE: return "LANE";
    case State::VERIFY: return "VERIFY";
    case State::SHIFT: return "SHIFT";
    case State::DONE: return "DONE";
  }
  return "?";
}

// Things worth telling the topside / app about. Kept as a flat struct (not
// JSON) so the ROS wrapper can map it to glaucus_interfaces/msg/SweepEvent.
struct Event {
  enum class Kind { END_OF_LANE, END_REJECTED, STEP_DOWN_BLOCKED, SWEEP_COMPLETE };
  Kind kind;
  int lane = -1;
  int direction = 0;
  double value = 0.0;      // lane length (END_OF_LANE / END_REJECTED) or retry #
  bool verified = false;
};

}  // namespace glaucus
