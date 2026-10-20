// vision_node — runs ON THE PI. Camera frames -> EndOfLaneEvidence.
//
//   sub: /camera/image_raw (sensor_msgs/Image) from camera_ros (see bot.launch.py)
//   pub: /glaucus/evidence (EndOfLaneEvidence)
//
// This node answers three cheap questions per frame. Keep it CHEAP: it runs on
// a 2 GB Pi alongside everything else. Heavy vision (mosaicking, SLAM) happens
// topside on the compressed stream.
//
//  1. cam_edge — "is the hull ending / is there an obstacle just ahead?"
//     Starting idea (classical, no ML): the camera looks forward-and-down at
//     the hull. When the hull ends, the top part of the image becomes open
//     water: low texture, low contrast, different color. Score =
//       fraction of the top band whose local variance is below a threshold.
//     Calibrate thresholds from bag recordings in the tank. If it's not good
//     enough, a tiny classifier (MobileNet-ish via ONNX Runtime) on 96x96
//     crops is the next step - collect labeled frames from bags first.
//
//  2. stall — "am I commanding motion but the picture isn't moving?"
//     Comes free from #3: |flow| ~ 0 while v_cmd > 0.
//
//  3. flow_ds — optical flow between consecutive frames (cv::phaseCorrelate
//     on a downscaled grayscale center crop is fast and robust), converted to
//     meters with the camera's height above the hull and focal length.
//     The sim says this is what fixes along-hull drift (see WallEstimator).
//     Alternative: a dedicated PMW3901-style optical flow sensor on the MCU.
//
// LIGHTING: underwater, you WILL need your own light. Fixed exposure +
// fixed white balance, or every metric above drifts with the sun.
//
// STATUS: skeleton. Publishes zeros so the rest of the stack runs.
#include <cmath>
#include <memory>

#include <glaucus_interfaces/msg/drive_command.hpp>
#include <glaucus_interfaces/msg/end_of_lane_evidence.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace gi = glaucus_interfaces;

class VisionNode : public rclcpp::Node {
 public:
  VisionNode() : Node("vision_node") {
    px_per_m_ = declare_parameter<double>("flow_px_per_m", 2000.0);  // TODO(glaucus): calibrate
    pub_ = create_publisher<gi::msg::EndOfLaneEvidence>("/glaucus/evidence", rclcpp::SensorDataQoS());
    img_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr m) { on_image(*m); });
    cmd_sub_ = create_subscription<gi::msg::DriveCommand>(
        "/glaucus/drive_cmd", 10, [this](gi::msg::DriveCommand::ConstSharedPtr m) { last_v_cmd_ = m->v; });
  }

 private:
  void on_image(const sensor_msgs::msg::Image& img) {
    gi::msg::EndOfLaneEvidence ev;
    ev.header = img.header;
    ev.cam_edge = edge_score(img);              // TODO(glaucus)
    const double flow_px = forward_flow_px(img);  // TODO(glaucus)
    ev.flow_valid = false;                      // flip to true once flow works
    ev.flow_ds = flow_px / px_per_m_;
    ev.flow_quality = 0.0;
    ev.stall = (last_v_cmd_ > 0.02 && ev.flow_valid && std::abs(ev.flow_ds) < 1e-4) ? 1.0 : 0.0;
    pub_->publish(ev);
  }

  // TODO(glaucus): cv_bridge -> grayscale -> top band -> local variance -> fraction below threshold.
  float edge_score(const sensor_msgs::msg::Image&) { return 0.0f; }

  // TODO(glaucus): downscale center crop, cv::phaseCorrelate with previous frame,
  // return the shift along the image axis that corresponds to robot forward.
  double forward_flow_px(const sensor_msgs::msg::Image&) { return 0.0; }

  double px_per_m_;
  double last_v_cmd_ = 0.0;
  rclcpp::Publisher<gi::msg::EndOfLaneEvidence>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Subscription<gi::msg::DriveCommand>::SharedPtr cmd_sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisionNode>());
  rclcpp::shutdown();
  return 0;
}
