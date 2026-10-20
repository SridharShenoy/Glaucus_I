// Behavioural unit tests for the sweep state machine. No plant model: we hand
// the controller synthetic sensor readings and check what it decides.
#include <glaucus_core/math.hpp>
#include <glaucus_core/sweep_controller.hpp>
#include <glaucus_core/wall_estimator.hpp>

#include "check.hpp"

using namespace glaucus;

namespace {
constexpr double kDt = 0.05;

// Drive the controller through the initial descent + turn so it's in LANE 0.
void run_to_first_lane(SweepController& c, const SweepParams& p) {
  Sensors s;
  s.heading = kPi / 2;
  s.depth = p.first_lane_depth;  // already at depth -> STEP_DOWN completes
  c.update(s, kDt);
  CHECK(c.state() == State::TURN);
  s.heading = 0.0;  // pretend the MCU finished the turn
  c.update(s, kDt);
  CHECK(c.state() == State::LANE);
  CHECK(c.lane_index() == 0);
  CHECK(c.direction() == +1);
}

Sensors lane_sensors(const SweepParams& p, double ds) {
  Sensors s;
  s.depth = p.first_lane_depth;
  s.heading = 0.0;
  s.odom_ds = ds;
  return s;
}
}  // namespace

void test_wrap() {
  CHECK_NEAR(wrap(0.0), 0.0, 1e-12);
  CHECK_NEAR(wrap(kTau + 0.1), 0.1, 1e-12);
  CHECK_NEAR(wrap(-kPi - 0.1), kPi - 0.1, 1e-12);
  // "359 deg vs 1 deg" must be a 2 deg error, not 358.
  CHECK_NEAR(wrap(1.0 * kPi / 180 - 359.0 * kPi / 180), 2.0 * kPi / 180, 1e-12);
}

void test_depth_hold_sign() {
  SweepParams p;
  SweepController c(p);
  run_to_first_lane(c, p);
  auto s = lane_sensors(p, 0.0075);
  s.depth = p.first_lane_depth + 0.05;  // too deep, heading toward bow
  const Command cmd = c.update(s, kDt);
  CHECK(cmd.heading < 0.0);             // aim up (negative = toward surface)
  CHECK(cmd.heading >= -p.max_depth_correction - 1e-12);
}

void test_end_of_lane_needs_debounce() {
  SweepParams p;
  SweepController c(p);
  run_to_first_lane(c, p);
  auto s = lane_sensors(p, 0.0075);
  for (int i = 0; i < 100; ++i) c.update(s, kDt);  // build up some lane length
  s.cam_edge = 1.0;                                // strong edge evidence...
  for (int i = 0; i < p.end_debounce_ticks - 1; ++i) {
    c.update(s, kDt);
    CHECK(c.state() == State::LANE);               // ...but not long enough yet
  }
  c.update(s, kDt);
  CHECK(c.state() == State::TURN);                 // now turning to face down
  auto ev = c.take_events();
  CHECK(ev.size() == 1 && ev[0].kind == Event::Kind::END_OF_LANE);
}

void test_single_glitch_ignored() {
  SweepParams p;
  SweepController c(p);
  run_to_first_lane(c, p);
  auto s = lane_sensors(p, 0.0075);
  for (int i = 0; i < 50; ++i) {
    s.cam_edge = (i % 3 == 0) ? 1.0 : 0.0;         // flickering, never 4 in a row
    c.update(s, kDt);
    CHECK(c.state() == State::LANE);
  }
}

void test_early_end_is_verified_then_rejected() {
  SweepParams p;
  SweepController c(p);
  run_to_first_lane(c, p);

  // Lane 0: full 8 m, then a real end.
  auto s = lane_sensors(p, 0.0075);
  while (c.lane_distance() < 8.0) c.update(s, kDt);
  s.cam_edge = 1.0;
  while (c.state() == State::LANE) c.update(s, kDt);
  c.take_events();

  // Get to lane 1: finish turn down, step down, turn toward stern.
  s = Sensors{};
  s.heading = kPi / 2;
  c.update(s, kDt);                                 // TURN complete -> STEP_DOWN
  CHECK(c.state() == State::STEP_DOWN);
  s.depth = p.first_lane_depth + p.lane_spacing();
  c.update(s, kDt);                                 // depth reached -> TURN
  s.heading = kPi;
  c.update(s, kDt);                                 // -> LANE (direction -1)
  CHECK(c.state() == State::LANE);
  CHECK(c.direction() == -1);

  // Lane 1: false alarm at 3 m (way shorter than 8 m - margin) -> VERIFY.
  s.odom_ds = 0.0075;
  while (c.lane_distance() < 3.0) c.update(s, kDt);
  s.cam_edge = 1.0;
  for (int i = 0; i < p.end_debounce_ticks; ++i) c.update(s, kDt);
  CHECK(c.state() == State::VERIFY);
  s.cam_edge = 0.0;                                 // evidence disappears while creeping
  s.odom_ds = p.verify_speed * kDt;
  for (int i = 0; i < 200 && c.state() == State::VERIFY; ++i) c.update(s, kDt);
  CHECK(c.state() == State::LANE);                  // back to work
  auto ev = c.take_events();
  CHECK(ev.size() == 1 && ev[0].kind == Event::Kind::END_REJECTED);
}

void test_blocked_step_down_retries_then_done() {
  SweepParams p;
  p.max_step_retries = 2;
  SweepController c(p);
  run_to_first_lane(c, p);
  auto s = lane_sensors(p, 0.0075);
  for (int i = 0; i < 100; ++i) c.update(s, kDt);
  s.cam_edge = 1.0;
  while (c.state() == State::LANE) c.update(s, kDt);
  c.take_events();

  int blocked = 0;
  for (int i = 0; i < 2000 && c.state() != State::DONE; ++i) {
    Sensors t;
    t.depth = p.first_lane_depth;  // never gets deeper
    t.stall = 1.0;                 // hull falls away / we're stuck
    t.odom_ds = 0.004;
    // Fake an instant MCU: always already facing where the controller wants.
    switch (c.state()) {
      case State::STEP_DOWN: t.heading = kPi / 2; break;
      case State::SHIFT: t.heading = kPi; t.stall = 0.0; break;
      default: t.heading = 0.0; break;
    }
    if (c.state() == State::TURN) {
      // Answer the turn with whatever it's asking for.
      Command cmd = c.update(t, kDt);
      t.heading = cmd.heading;
    }
    c.update(t, kDt);
    for (auto& e : c.take_events())
      if (e.kind == Event::Kind::STEP_DOWN_BLOCKED) ++blocked;
  }
  CHECK(c.state() == State::DONE);
  CHECK(blocked == p.max_step_retries);
}

void test_estimator_reset_gate() {
  WallEstimator est(0.34, {0.34, 0.5, false});
  Sensors s;
  s.heading = kPi;
  s.odom_ds = 2.0;
  est.update(s);                  // x = 0.34 - 2.0
  est.on_end_of_lane(-1);         // 2 m away from the stern ref -> rejected
  CHECK_NEAR(est.x(), 0.34 - 2.0, 1e-9);

  WallEstimator est2(0.5, {0.34, 0.5, false});
  est2.on_end_of_lane(-1);        // within gate -> snap
  CHECK_NEAR(est2.x(), 0.34, 1e-12);
  est2.on_end_of_lane(+1);        // bow end never resets
  CHECK_NEAR(est2.x(), 0.34, 1e-12);
}

int main() {
  test_wrap();
  test_depth_hold_sign();
  test_end_of_lane_needs_debounce();
  test_single_glitch_ignored();
  test_early_end_is_verified_then_rejected();
  test_blocked_step_down_retries_then_done();
  test_estimator_reset_gate();
  TEST_MAIN_END();
}
