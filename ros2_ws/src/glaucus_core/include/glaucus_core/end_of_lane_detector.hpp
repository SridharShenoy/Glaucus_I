// glaucus_core/end_of_lane_detector.hpp
//
// Fuses weak cues into one "this lane is over" decision.
//
// DESIGN CHOICE: weighted score + debounce, not a single sensor.
//   - Camera alone: false positives (a fouling patch looks like an edge) and
//     misses in murky water.
//   - Stall alone: you only find the edge by ramming it.
//   - Requiring N consecutive ticks over threshold kills one-frame glitches.
// This is deliberately dumb and tunable. If it isn't enough, the next step is
// a tiny HMM / Bayes filter over {on_hull, at_edge} - but tune this first with
// real tank data (record bags!).
#pragma once

#include "glaucus_core/types.hpp"

namespace glaucus {

class EndOfLaneDetector {
 public:
  explicit EndOfLaneDetector(const SweepParams& p) : p_(p) {}

  void reset() {
    count_ = 0;
    score_ = 0.0;
  }

  // Returns true once the evidence has been strong for end_debounce_ticks.
  bool update(double cam_edge, double stall);

  double score() const { return score_; }

 private:
  const SweepParams& p_;
  int count_ = 0;
  double score_ = 0.0;
};

}  // namespace glaucus
