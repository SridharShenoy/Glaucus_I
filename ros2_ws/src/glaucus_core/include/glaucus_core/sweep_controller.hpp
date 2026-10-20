// glaucus_core/sweep_controller.hpp
//
// The sweep state machine: a boustrophedon ("lawnmower") on one hull side.
//
//            +-----------+  depth reached   +------+  aligned  +------+
//   start -> | STEP_DOWN | ---------------> | TURN | --------> | LANE |
//            +-----------+                  +------+           +------+
//               ^    | blocked                                  |    | early end?
//               |    v (retry)                                  |    v
//               |  +------+   moved shift_inward                |  +--------+
//               +--| SHIFT|<-----(via TURN)                     |  | VERIFY |
//                  +------+                                     |  +--------+
//                                    end of lane (via TURN) <---+------+  | not confirmed
//                                    next lane too deep -> DONE           v
//                                                                     back to LANE
//
// KEY DESIGN CHOICE: lanes are defined by DEPTH (absolute sensor), and lane
// ENDS are DETECTED (camera + stall), never computed from x. So the sweep
// pattern itself does not need along-hull position at all. x (from odometry /
// optical flow) only feeds the coverage map. Odometry drift therefore can't
// make the robot sweep the wrong place - it can only make the map wrong.
//
// Runs on the Pi at 20 Hz. Output Command goes to the MCU, which runs the fast
// heading loop. If the Pi dies or the link drops, the MCU watchdog stops the
// motors - this class never needs to worry about that.
#pragma once

#include <optional>
#include <vector>

#include "glaucus_core/end_of_lane_detector.hpp"
#include "glaucus_core/types.hpp"

namespace glaucus {

class SweepController {
 public:
  explicit SweepController(const SweepParams& p);

  // Call at a fixed rate. dt in seconds.
  Command update(const Sensors& s, double dt);

  // Operator abort / pause (pause = stop calling update and send v=0).
  void abort();

  // ---- read-only state for telemetry / the app ----
  State state() const { return state_; }
  int lane_index() const { return lane_idx_; }
  double lane_depth() const { return lane_depth_; }
  int direction() const { return direction_; }
  double end_score() const { return detector_.score(); }
  double lane_distance() const { return lane_dist_; }

  // Events since the last call (moved out; caller publishes them).
  std::vector<Event> take_events();

  static double lane_heading(int direction);

 private:
  void enter(State s);
  void turn_then(double target, State after);
  Command stop(double heading) const;
  double depth_hold_heading(double depth, double dt);
  Command end_lane(double hdg, bool verified);

  SweepParams p_;
  EndOfLaneDetector detector_;
  State state_ = State::STEP_DOWN;
  int lane_idx_ = -1;
  double lane_depth_ = 0.0;
  double target_depth_;
  int direction_ = -1;  // first lane flips this to +1 (toward bow)
  double turn_target_ = 0.0;
  State after_turn_ = State::LANE;
  double depth_integral_ = 0.0;
  double seg_dist_ = 0.0;
  double lane_dist_ = 0.0;
  std::optional<double> prev_lane_length_;
  int retries_ = 0;
  std::vector<Event> events_;
};

}  // namespace glaucus
