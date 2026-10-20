// Python <-> C++ parity test.
//
// sim/sweep_sim.py --dump-trace writes every control tick's sensor inputs and
// the Python controller's outputs. We replay the inputs through the C++
// SweepController and require identical outputs. This is what lets you
// prototype in Python and trust the port.
//
// Regenerate after changing the algorithm:
//   python sim/sweep_sim.py --quiet --dump-trace \
//       ros2_ws/src/glaucus_core/test/data/sim_trace_seed2.csv
#include <glaucus_core/sweep_controller.hpp>

#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "check.hpp"

#ifndef GLAUCUS_TRACE_FILE
#define GLAUCUS_TRACE_FILE "test/data/sim_trace_seed2.csv"
#endif

using namespace glaucus;

int main(int argc, char** argv) {
  const std::string path = argc > 1 ? argv[1] : GLAUCUS_TRACE_FILE;
  std::ifstream in(path);
  if (!in) {
    std::fprintf(stderr, "cannot open trace %s\n", path.c_str());
    return 2;
  }
  std::string line;
  std::getline(in, line);  // header

  SweepParams p;           // defaults must match the Python SweepParams
  SweepController c(p);
  const double dt = 0.05;
  int n = 0, mismatches = 0;

  while (std::getline(in, line)) {
    std::stringstream ss(line);
    std::string f[10];
    for (auto& x : f) std::getline(ss, x, ',');
    Sensors s;
    s.depth = std::stod(f[0]);
    s.heading = std::stod(f[1]);
    s.odom_ds = std::stod(f[2]);
    s.cam_edge = std::stod(f[3]);
    s.stall = std::stod(f[4]);
    const double flow = std::stod(f[5]);
    if (!std::isnan(flow)) s.flow_ds = flow;

    const Command cmd = c.update(s, dt);
    ++n;
    const bool ok = std::abs(cmd.v - std::stod(f[6])) < 1e-9 &&
                    std::abs(cmd.heading - std::stod(f[7])) < 1e-9 &&
                    int(cmd.brush_on) == std::stoi(f[8]) && f[9] == to_string(c.state());
    if (!ok && mismatches++ < 5) {
      std::fprintf(stderr, "tick %d: C++ (v=%.6f hdg=%.6f %s) vs Python (v=%s hdg=%s %s)\n", n, cmd.v,
                   cmd.heading, to_string(c.state()), f[6].c_str(), f[7].c_str(), f[9].c_str());
    }
  }
  CHECK(n > 1000);
  CHECK(mismatches == 0);
  std::printf("replayed %d ticks, %d mismatches\n", n, mismatches);
  TEST_MAIN_END();
}
