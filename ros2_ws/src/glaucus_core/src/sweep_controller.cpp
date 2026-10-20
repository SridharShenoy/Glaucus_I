// See header for the state diagram and design rationale.
// This is a line-for-line port of SweepController in sim/sweep_sim.py.
// test/test_parity.cpp replays a Python trace through this class and checks
// that every command matches - run it after ANY change here.
#include "glaucus_core/sweep_controller.hpp"

#include <cmath>
#include <utility>

#include "glaucus_core/math.hpp"

namespace glaucus {

SweepController::SweepController(const SweepParams& p)
    : p_(p), detector_(p_), target_depth_(p.first_lane_depth) {}

double SweepController::lane_heading(int direction) { return direction > 0 ? 0.0 : kPi; }

void SweepController::enter(State s) {
  state_ = s;
  seg_dist_ = 0.0;
  detector_.reset();
}

void SweepController::turn_then(double target, State after) {
  turn_target_ = target;
  after_turn_ = after;
  enter(State::TURN);
}

Command SweepController::stop(double heading) const {
  return Command{0.0, heading, state_ != State::DONE};
}

void SweepController::abort() { enter(State::DONE); }

std::vector<Event> SweepController::take_events() { return std::exchange(events_, {}); }

// Outer loop: depth error -> heading offset.
// Intuition: you're driving along a hillside road. Too low? Aim slightly
// uphill. The further off you are, the steeper you aim (P), and if you keep
// sagging despite that (gravity), aim a bit more over time (I). Clamp so it
// never aims wildly. "Uphill" is a left turn going one way and a right turn
// going the other way - hence multiplying by direction.
double SweepController::depth_hold_heading(double depth, double dt) {
  const double e = depth - lane_depth_;
  depth_integral_ = clamp(depth_integral_ + e * dt, -p_.depth_integral_limit, p_.depth_integral_limit);
  const double corr = clamp(p_.kp_depth * e + p_.ki_depth * depth_integral_,
                            -p_.max_depth_correction, p_.max_depth_correction);
  return lane_heading(direction_) - direction_ * corr;
}

Command SweepController::end_lane(double hdg, bool verified) {
  events_.push_back({Event::Kind::END_OF_LANE, lane_idx_, direction_, lane_dist_, verified});
  prev_lane_length_ = lane_dist_;
  const double next = lane_depth_ + p_.lane_spacing();
  if (next > p_.max_depth) {
    events_.push_back({Event::Kind::SWEEP_COMPLETE, lane_idx_, direction_, double(lane_idx_ + 1), false});
    enter(State::DONE);
  } else {
    target_depth_ = next;
    retries_ = 0;
    turn_then(kPi / 2, State::STEP_DOWN);
  }
  return stop(hdg);
}

Command SweepController::update(const Sensors& s, double dt) {
  // Best available "how far did I actually move".
  const double ds = s.flow_ds ? *s.flow_ds : s.odom_ds;

  switch (state_) {
    case State::DONE:
      return Command{0.0, s.heading, false};

    case State::TURN:
      if (std::abs(wrap(turn_target_ - s.heading)) < p_.heading_tol) enter(after_turn_);
      return stop(turn_target_);

    case State::LANE:
    case State::VERIFY: {
      seg_dist_ += ds;
      lane_dist_ += ds;
      const double hdg = depth_hold_heading(s.depth, dt);
      const bool fired = detector_.update(s.cam_edge, s.stall);

      if (state_ == State::VERIFY) {
        if (fired) return end_lane(hdg, /*verified=*/true);
        if (seg_dist_ >= p_.verify_distance) {
          events_.push_back({Event::Kind::END_REJECTED, lane_idx_, direction_, lane_dist_, false});
          enter(State::LANE);
        }
        return Command{p_.verify_speed, hdg, true};
      }

      if (fired || lane_dist_ > p_.max_lane_length) {
        const bool suspicious = prev_lane_length_ && lane_dist_ < *prev_lane_length_ - p_.verify_margin;
        if (fired && suspicious) {
          enter(State::VERIFY);
          return Command{p_.verify_speed, hdg, true};
        }
        return end_lane(hdg, /*verified=*/false);
      }
      return Command{p_.lane_speed, hdg, true};
    }

    case State::STEP_DOWN: {
      seg_dist_ += ds;
      if (s.depth >= target_depth_ - p_.depth_tol) {
        lane_depth_ = target_depth_;
        lane_idx_ += 1;
        direction_ = -direction_;
        lane_dist_ = 0.0;
        turn_then(lane_heading(direction_), State::LANE);
        return stop(kPi / 2);
      }
      const bool blocked = detector_.update(s.cam_edge, s.stall);
      const bool too_far = seg_dist_ > 2.0 * p_.lane_spacing() + 0.3;
      if (blocked || too_far) {
        if (retries_ < p_.max_step_retries) {
          // Hull curves away below us (bow cutaway / keel): slide back toward
          // the middle of the lane and try again.
          retries_ += 1;
          events_.push_back({Event::Kind::STEP_DOWN_BLOCKED, lane_idx_, direction_, double(retries_), false});
          turn_then(lane_heading(-direction_), State::SHIFT);
        } else {
          events_.push_back({Event::Kind::SWEEP_COMPLETE, lane_idx_, direction_, double(lane_idx_ + 1), false});
          enter(State::DONE);
        }
        return stop(kPi / 2);
      }
      return Command{p_.step_speed, kPi / 2, true};
    }

    case State::SHIFT:
      seg_dist_ += ds;
      if (seg_dist_ >= p_.shift_inward) {
        turn_then(kPi / 2, State::STEP_DOWN);
        return stop(turn_target_);
      }
      return Command{p_.lane_speed, lane_heading(-direction_), true};
  }
  return Command{0.0, s.heading, false};
}

}  // namespace glaucus
