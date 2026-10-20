// glaucus_core/wall_estimator.hpp
//
// "Where am I on the unrolled hull?"  (x along hull, z depth, theta heading)
//
//   z     <- depth sensor          absolute -> just use it (light low-pass ok)
//   theta <- gravity from the IMU  absolute on a wall -> just use it
//   x     <- odometry              DRIFTS -> needs help:
//              * prefer optical flow (measures real motion, ignores wheel slip)
//              * snap to the known stern x when we hit the transom, but only if
//                we already think we're near it (gate), so a false end-of-lane
//                in mid-hull can't teleport the estimate.
//
// DESIGN CHOICE: no EKF / robot_localization here. In the wall frame the three
// states are almost decoupled and two of them are measured directly, so a
// filter buys little and costs a lot of tuning. Revisit if/when you do real
// 3D hull SLAM (then robot_localization or a factor graph earns its keep).
//
// Sim finding worth remembering (see sim/sweep_sim.py WallEstimator docstring):
// with wheel odometry, stern resets barely help because steady slip cancels on
// round trips; calibrating slip on step-downs fails because gravity helps going
// down. Optical flow is the thing that actually moves the needle.
#pragma once

#include <cmath>
#include <optional>

#include "glaucus_core/types.hpp"

namespace glaucus {

class WallEstimator {
 public:
  struct Config {
    std::optional<double> stern_reset_x;  // m; nullopt disables resets
    double reset_gate = 0.5;              // m
    bool use_flow = true;
  };

  WallEstimator(double x0, Config cfg) : x_(x0), cfg_(cfg) {}

  void update(const Sensors& s) {
    theta_ = s.heading;
    z_ = s.depth;  // TODO(glaucus): low-pass to reject wave/pressure noise at the dock
    const double ds = (cfg_.use_flow && s.flow_ds) ? *s.flow_ds : s.odom_ds;
    x_ += ds * std::cos(theta_);
  }

  // Call when the controller reports END_OF_LANE.
  void on_end_of_lane(int direction) {
    if (cfg_.stern_reset_x && direction < 0 && std::abs(x_ - *cfg_.stern_reset_x) < cfg_.reset_gate) {
      x_ = *cfg_.stern_reset_x;
    }
  }

  double x() const { return x_; }
  double z() const { return z_; }
  double theta() const { return theta_; }

 private:
  double x_ = 0.0, z_ = 0.0, theta_ = 0.0;
  Config cfg_;
};

}  // namespace glaucus
