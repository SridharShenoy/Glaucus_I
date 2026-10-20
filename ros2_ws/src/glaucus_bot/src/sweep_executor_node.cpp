// sweep_executor_node — runs ON THE PI. Thin ROS wrapper around
// glaucus::SweepController + glaucus::WallEstimator.
//
//   subs:  /glaucus/telemetry (BotTelemetry, 100 Hz from mcu_bridge)
//          /glaucus/evidence  (EndOfLaneEvidence, ~20 Hz from vision_node)
//          /glaucus/topside_heartbeat (std_msgs/Empty, 2 Hz from mission_manager)
//   pubs:  /glaucus/drive_cmd (DriveCommand, 20 Hz)  -> mcu_bridge -> MCU
//          /glaucus/sweep_status (SweepStatus, 5 Hz)
//          /glaucus/sweep_events (SweepEvent)
//   action server: /glaucus/run_sweep (RunSweep)
//   services: /glaucus/pause (std_srvs/SetBool), /glaucus/estop (std_srvs/Trigger)
//
// DESIGN CHOICES
//  - Runs on the bot, not topside, so a dropped tether/Wi-Fi link doesn't
//    leave the robot mid-maneuver with no brain.
//  - Link-loss policy (topside heartbeat missing > link_timeout_s): finish the
//    current lane, then HOLD in place with adhesion on. Never keep sweeping
//    blind for a whole mission, never just let go.
//  - Fixed-rate wall timer at 20 Hz. The controller assumes a constant dt; if
//    the timer is late we pass the real dt (it only matters for the integral).
//  - Sensor fusion is "latest value wins": telemetry is 5x faster than the
//    control loop, so we sum wheel_ds / flow_ds between ticks.
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>

#include <glaucus_core/sweep_controller.hpp>
#include <glaucus_core/wall_estimator.hpp>
#include <glaucus_interfaces/action/run_sweep.hpp>
#include <glaucus_interfaces/msg/bot_telemetry.hpp>
#include <glaucus_interfaces/msg/drive_command.hpp>
#include <glaucus_interfaces/msg/end_of_lane_evidence.hpp>
#include <glaucus_interfaces/msg/sweep_event.hpp>
#include <glaucus_interfaces/msg/sweep_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;
namespace gi = glaucus_interfaces;
using RunSweep = gi::action::RunSweep;
using GoalHandle = rclcpp_action::ServerGoalHandle<RunSweep>;

class SweepExecutor : public rclcpp::Node {
 public:
  SweepExecutor() : Node("sweep_executor") {
    load_params();

    drive_pub_ = create_publisher<gi::msg::DriveCommand>("/glaucus/drive_cmd", 10);
    status_pub_ = create_publisher<gi::msg::SweepStatus>("/glaucus/sweep_status", 10);
    event_pub_ = create_publisher<gi::msg::SweepEvent>("/glaucus/sweep_events", rclcpp::QoS(50).reliable());

    telem_sub_ = create_subscription<gi::msg::BotTelemetry>(
        "/glaucus/telemetry", rclcpp::SensorDataQoS(),
        [this](gi::msg::BotTelemetry::ConstSharedPtr m) { on_telemetry(*m); });
    evidence_sub_ = create_subscription<gi::msg::EndOfLaneEvidence>(
        "/glaucus/evidence", rclcpp::SensorDataQoS(),
        [this](gi::msg::EndOfLaneEvidence::ConstSharedPtr m) { on_evidence(*m); });
    hb_sub_ = create_subscription<std_msgs::msg::Empty>(
        "/glaucus/topside_heartbeat", 10, [this](std_msgs::msg::Empty::ConstSharedPtr) { last_hb_ = now(); });

    pause_srv_ = create_service<std_srvs::srv::SetBool>(
        "/glaucus/pause", [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                 std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
          paused_ = req->data;
          res->success = true;
          res->message = paused_ ? "paused" : "resumed";
        });
    estop_srv_ = create_service<std_srvs::srv::Trigger>(
        "/glaucus/estop", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          // E-stop = HOLD (adhesion on, wheels off). Releasing the hull is a
          // separate, deliberate action from the app ("retrieve").
          if (ctrl_) ctrl_->abort();
          estopped_ = true;
          res->success = true;
        });

    action_srv_ = rclcpp_action::create_server<RunSweep>(
        this, "/glaucus/run_sweep",
        [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const RunSweep::Goal>) {
          return active_goal_ ? rclcpp_action::GoalResponse::REJECT
                              : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](const std::shared_ptr<GoalHandle>) {
          if (ctrl_) ctrl_->abort();
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<GoalHandle> gh) { start_goal(gh); });

    control_timer_ = create_wall_timer(50ms, [this] { control_tick(); });
    status_timer_ = create_wall_timer(200ms, [this] { publish_status(); });
    last_hb_ = now();
    last_tick_ = now();
  }

 private:
  void load_params() {
    // Every field of SweepParams is a ROS parameter with the same name, so
    // config/sweep_params.yaml == Python SweepParams == C++ SweepParams.
    auto d = [this](const char* n, double& v) { v = declare_parameter<double>(n, v); };
    d("brush_width", p_.brush_width);
    d("overlap", p_.overlap);
    d("first_lane_depth", p_.first_lane_depth);
    d("max_depth", p_.max_depth);
    d("lane_speed", p_.lane_speed);
    d("step_speed", p_.step_speed);
    d("kp_depth", p_.kp_depth);
    d("ki_depth", p_.ki_depth);
    d("max_depth_correction", p_.max_depth_correction);
    d("depth_integral_limit", p_.depth_integral_limit);
    d("heading_tol", p_.heading_tol);
    d("depth_tol", p_.depth_tol);
    d("w_camera", p_.w_camera);
    d("w_stall", p_.w_stall);
    d("end_threshold", p_.end_threshold);
    p_.end_debounce_ticks = declare_parameter<int>("end_debounce_ticks", p_.end_debounce_ticks);
    d("max_lane_length", p_.max_lane_length);
    d("verify_margin", p_.verify_margin);
    d("verify_distance", p_.verify_distance);
    d("verify_speed", p_.verify_speed);
    d("shift_inward", p_.shift_inward);
    p_.max_step_retries = declare_parameter<int>("max_step_retries", p_.max_step_retries);
    stern_reset_x_ = declare_parameter<double>("stern_reset_x", 0.0);
    use_flow_ = declare_parameter<bool>("use_optical_flow", true);
    link_timeout_s_ = declare_parameter<double>("link_timeout_s", 3.0);
  }

  void on_telemetry(const gi::msg::BotTelemetry& m) {
    std::lock_guard lk(mu_);
    latest_.depth = m.depth;
    latest_.heading = m.heading;
    accum_wheel_ds_ += m.wheel_ds;
    faults_ = m.faults;
    // TODO(glaucus): a second stall cue from the MCU: motor_current spike while
    // wheel speed ~0. Fuse with the camera stall below (max of the two?).
  }

  void on_evidence(const gi::msg::EndOfLaneEvidence& m) {
    std::lock_guard lk(mu_);
    latest_.cam_edge = m.cam_edge;
    latest_.stall = m.stall;
    if (m.flow_valid) {
      accum_flow_ds_ = accum_flow_ds_.value_or(0.0) + m.flow_ds;
    }
  }

  void start_goal(const std::shared_ptr<GoalHandle> gh) {
    const auto g = gh->get_goal();
    glaucus::SweepParams p = p_;
    if (g->first_lane_depth > 0) p.first_lane_depth = g->first_lane_depth;
    if (g->max_depth > 0) p.max_depth = g->max_depth;
    if (g->overlap >= 0) p.overlap = g->overlap;
    // TODO(glaucus): send hull_side to the MCU (heading sign) before starting.
    goal_params_ = p;
    ctrl_.emplace(goal_params_);
    est_.emplace(0.0, glaucus::WallEstimator::Config{stern_reset_x_, 0.5, use_flow_});
    active_goal_ = gh;
    estopped_ = false;
    paused_ = false;
    started_ = now();
    RCLCPP_INFO(get_logger(), "sweep started: lanes %.2f..%.2f m, spacing %.2f m", p.first_lane_depth,
                p.max_depth, p.lane_spacing());
  }

  void control_tick() {
    const auto t = now();
    const double dt = (t - last_tick_).seconds();
    last_tick_ = t;

    gi::msg::DriveCommand out;
    out.mode = gi::msg::DriveCommand::MODE_HOLD;  // default: stay put, stay stuck

    glaucus::Sensors s;
    {
      std::lock_guard lk(mu_);
      s = latest_;
      s.odom_ds = accum_wheel_ds_;
      s.flow_ds = accum_flow_ds_;
      accum_wheel_ds_ = 0.0;
      accum_flow_ds_.reset();
    }

    if (ctrl_ && !estopped_) {
      est_->update(s);
      const bool link_ok = (t - last_hb_).seconds() < link_timeout_s_;
      // TODO(glaucus): implement "finish lane then hold" on link loss. For now: hold immediately.
      if (!paused_ && link_ok && ctrl_->state() != glaucus::State::DONE) {
        const glaucus::Command c = ctrl_->update(s, dt);
        out.mode = gi::msg::DriveCommand::MODE_DRIVE;
        out.v = c.v;
        out.heading_cmd = c.heading;
        out.brush_on = c.brush_on;
      }
      for (const auto& e : ctrl_->take_events()) publish_event(e);
      if (ctrl_->state() == glaucus::State::DONE) finish_goal(true, "complete");
    }
    drive_pub_->publish(out);   // always publish: this IS the MCU's watchdog feed
  }

  void publish_event(const glaucus::Event& e) {
    if (e.kind == glaucus::Event::Kind::END_OF_LANE) est_->on_end_of_lane(e.direction);
    gi::msg::SweepEvent m;
    m.header.stamp = now();
    m.kind = static_cast<uint8_t>(e.kind);
    m.lane = e.lane;
    m.direction = e.direction;
    m.value = e.value;
    m.verified = e.verified;
    event_pub_->publish(m);
  }

  gi::msg::SweepStatus make_status() {
    gi::msg::SweepStatus m;
    m.header.stamp = now();
    m.state = ctrl_ ? glaucus::to_string(ctrl_->state()) : "IDLE";
    m.paused = paused_;
    if (ctrl_) {
      m.lane_index = ctrl_->lane_index();
      m.lane_depth = ctrl_->lane_depth();
      m.direction = ctrl_->direction();
      m.end_score = ctrl_->end_score();
    }
    if (est_) {
      m.depth = est_->z();
      m.heading = est_->theta();
      m.x_est = est_->x();
    }
    m.faults = faults_;
    return m;
  }

  void publish_status() {
    auto m = make_status();
    status_pub_->publish(m);
    if (active_goal_) {
      auto fb = std::make_shared<RunSweep::Feedback>();
      fb->status = m;
      active_goal_->publish_feedback(fb);
    }
  }

  void finish_goal(bool completed, const std::string& reason) {
    if (!active_goal_) return;
    auto r = std::make_shared<RunSweep::Result>();
    r->completed = completed;
    r->reason = reason;
    r->lanes = ctrl_->lane_index() + 1;
    r->duration_s = (now() - started_).seconds();
    if (active_goal_->is_canceling()) active_goal_->canceled(r);
    else active_goal_->succeed(r);
    active_goal_.reset();
  }

  glaucus::SweepParams p_, goal_params_;
  std::optional<glaucus::SweepController> ctrl_;
  std::optional<glaucus::WallEstimator> est_;
  double stern_reset_x_ = 0.0, link_timeout_s_ = 3.0;
  bool use_flow_ = true, paused_ = false, estopped_ = false;

  std::mutex mu_;
  glaucus::Sensors latest_;
  double accum_wheel_ds_ = 0.0;
  std::optional<double> accum_flow_ds_;
  uint16_t faults_ = 0;

  rclcpp::Time last_hb_, last_tick_, started_;
  std::shared_ptr<GoalHandle> active_goal_;

  rclcpp::Publisher<gi::msg::DriveCommand>::SharedPtr drive_pub_;
  rclcpp::Publisher<gi::msg::SweepStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<gi::msg::SweepEvent>::SharedPtr event_pub_;
  rclcpp::Subscription<gi::msg::BotTelemetry>::SharedPtr telem_sub_;
  rclcpp::Subscription<gi::msg::EndOfLaneEvidence>::SharedPtr evidence_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr hb_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pause_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr estop_srv_;
  rclcpp_action::Server<RunSweep>::SharedPtr action_srv_;
  rclcpp::TimerBase::SharedPtr control_timer_, status_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SweepExecutor>());
  rclcpp::shutdown();
  return 0;
}
