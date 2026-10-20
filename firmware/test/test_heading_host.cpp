// Host-side sanity test for the MCU heading estimator + controller (no Arduino).
//   g++ -std=c++17 -I firmware/src firmware/test/test_heading_host.cpp -o /tmp/th && /tmp/th
#include <cmath>
#include <cstdio>

#include "heading_controller.h"
#include "heading_estimator.h"

int main() {
  int fails = 0;
  const float g = 9.81f, pi = 3.14159265f;
  HeadingEstimator est(1.0f, 0.02f);

  // Gravity-only readings for three headings on a vertical starboard wall.
  struct { float heading; float ax, ay; } cases[] = {
      {0.0f, 0.0f, g}, {pi / 2, -g, 0.0f}, {pi, 0.0f, -g}};
  for (auto& c : cases) {
    est.reset_from_accel(c.ax, c.ay);
    if (std::fabs(HeadingEstimator::wrap(est.heading() - c.heading)) > 1e-4f) {
      std::printf("FAIL: accel heading %.3f expected %.3f\n", est.heading(), c.heading);
      ++fails;
    }
  }

  // A biased gyro alone would drift; gravity correction must bound it.
  est.reset_from_accel(0.0f, g);
  for (int i = 0; i < 200 * 60; ++i) est.update(0.0f, g, 0.0f, /*gz bias*/ 0.01f, 0.005f);
  if (std::fabs(est.heading()) > 0.05f) {
    std::printf("FAIL: gyro bias not corrected, heading=%.3f\n", est.heading());
    ++fails;
  }

  // Controller turns the right way and saturates.
  HeadingController ctrl(HeadingController::Gains{});
  if (!(ctrl.omega(0.5f, 0.0f, 0.0f) > 0.0f)) { std::printf("FAIL: omega sign\n"); ++fails; }
  if (ctrl.omega(3.0f, 0.0f, 0.0f) > 0.8f + 1e-6f) { std::printf("FAIL: omega clamp\n"); ++fails; }
  auto w = ctrl.mix(0.0f, 0.8f);
  if (!(w.right > 0 && w.left < 0)) { std::printf("FAIL: pivot mixing\n"); ++fails; }

  std::printf(fails ? "%d failure(s)\n" : "heading ok\n", fails);
  return fails ? 1 : 0;
}
