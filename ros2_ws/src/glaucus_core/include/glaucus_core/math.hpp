// glaucus_core/math.hpp — tiny helpers. Keep in sync with sim/sweep_sim.py.
#pragma once

#include <cmath>

namespace glaucus {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTau = 2.0 * kPi;

// Wrap to [-pi, pi). Same semantics as Python's ((a + pi) % tau) - pi, which
// is NOT the same as std::fmod for negative inputs - hence floor().
// ALWAYS wrap an angle *error* before feeding it to a controller, otherwise
// "359 deg vs 1 deg" looks like a 358 deg error and the robot spins.
inline double wrap(double a) { return a - kTau * std::floor((a + kPi) / kTau); }

inline double clamp(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

}  // namespace glaucus
