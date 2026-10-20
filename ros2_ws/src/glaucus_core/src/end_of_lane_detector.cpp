#include "glaucus_core/end_of_lane_detector.hpp"

namespace glaucus {

bool EndOfLaneDetector::update(double cam_edge, double stall) {
  score_ = p_.w_camera * cam_edge + p_.w_stall * stall;
  count_ = (score_ > p_.end_threshold) ? count_ + 1 : 0;
  return count_ >= p_.end_debounce_ticks;
}

}  // namespace glaucus
