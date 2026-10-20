// mcu_bridge_node — runs ON THE PI. USB serial <-> ROS.
//
//   sub: /glaucus/drive_cmd  (DriveCommand)  -> CommandPacket -> serial
//   pub: /glaucus/telemetry  (BotTelemetry)  <- TelemetryPacket <- serial
//
// DESIGN CHOICES
//  - Wire format comes from shared/protocol/glaucus_protocol.h (same header the
//    firmware compiles). Never redefine structs here.
//  - Forward commands immediately; don't re-send stale ones on a timer. If the
//    executor stops publishing, the MCU watchdog SHOULD trip - that's the point.
//  - Plain POSIX termios, no serial library dependency.
//
// STATUS: skeleton. TODO(glaucus) items below.
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <glaucus_interfaces/msg/bot_telemetry.hpp>
#include <glaucus_interfaces/msg/drive_command.hpp>
#include <rclcpp/rclcpp.hpp>

#include "glaucus_protocol.h"

using namespace std::chrono_literals;
namespace gi = glaucus_interfaces;
namespace gp = glaucus_proto;

class McuBridge : public rclcpp::Node {
 public:
  McuBridge() : Node("mcu_bridge") {
    port_ = declare_parameter<std::string>("port", "/dev/ttyACM0");
    open_port();

    telem_pub_ = create_publisher<gi::msg::BotTelemetry>("/glaucus/telemetry", rclcpp::SensorDataQoS());
    cmd_sub_ = create_subscription<gi::msg::DriveCommand>(
        "/glaucus/drive_cmd", 10, [this](gi::msg::DriveCommand::ConstSharedPtr m) { send(*m); });

    // Poll serial fast; telemetry arrives at 100 Hz.
    rx_timer_ = create_wall_timer(2ms, [this] { poll(); });
  }

  ~McuBridge() override {
    if (fd_ >= 0) close(fd_);
  }

 private:
  void open_port() {
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "cannot open %s: %s", port_.c_str(), strerror(errno));
      // TODO(glaucus): retry with backoff instead of giving up (USB re-enumerates after a brownout).
      return;
    }
    termios tio{};
    tcgetattr(fd_, &tio);
    cfmakeraw(&tio);
    cfsetspeed(&tio, B115200);  // ignored by USB CDC, but set it anyway
    tcsetattr(fd_, TCSANOW, &tio);
    RCLCPP_INFO(get_logger(), "connected to MCU on %s", port_.c_str());
  }

  void send(const gi::msg::DriveCommand& m) {
    if (fd_ < 0) return;
    gp::CommandPacket c;
    c.seq = tx_seq_++;
    c.mode = m.mode;
    c.brush_on = m.brush_on;
    c.v = m.v;
    c.heading_cmd = m.heading_cmd;
    c.omega_cmd = m.omega_cmd;
    gp::seal(c);
    uint8_t buf[sizeof c + 4];
    size_t n = gp::cobs_encode(reinterpret_cast<uint8_t*>(&c), sizeof c, buf);
    buf[n++] = 0;
    if (::write(fd_, buf, n) != ssize_t(n)) {
      // TODO(glaucus): count + report write failures
    }
  }

  void poll() {
    if (fd_ < 0) return;
    uint8_t chunk[256];
    ssize_t r;
    while ((r = ::read(fd_, chunk, sizeof chunk)) > 0) {
      for (ssize_t i = 0; i < r; ++i) {
        if (chunk[i] != 0) {
          if (rx_.size() < 256) rx_.push_back(chunk[i]);
          continue;
        }
        handle_frame();
        rx_.clear();
      }
    }
  }

  void handle_frame() {
    uint8_t dec[256];
    const size_t n = gp::cobs_decode(rx_.data(), rx_.size(), dec);
    if (n != sizeof(gp::TelemetryPacket) || dec[0] != gp::MSG_TELEMETRY) return;
    gp::TelemetryPacket t;
    std::memcpy(&t, dec, sizeof t);
    if (!gp::verify(t)) {
      ++bad_packets_;   // TODO(glaucus): publish diagnostics (diagnostic_updater)
      return;
    }
    gi::msg::BotTelemetry m;
    m.header.stamp = now();
    m.mcu_time_ms = t.t_ms;
    m.mode = t.mode;
    m.faults = t.faults;
    m.heading = t.heading;
    m.heading_rate = t.heading_rate;
    m.wall_tilt = t.wall_tilt;
    m.depth = t.depth;
    m.wheel_ds = t.wheel_ds;
    m.wheel_v_left = t.wheel_v_left;
    m.wheel_v_right = t.wheel_v_right;
    m.motor_current = t.motor_current;
    m.adhesion = t.adhesion;
    m.battery_v = t.battery_v;
    telem_pub_->publish(m);
    // TODO(glaucus): detect dropped packets via t.seq gaps
  }

  std::string port_;
  int fd_ = -1;
  uint16_t tx_seq_ = 0;
  uint32_t bad_packets_ = 0;
  std::vector<uint8_t> rx_;
  rclcpp::Publisher<gi::msg::BotTelemetry>::SharedPtr telem_pub_;
  rclcpp::Subscription<gi::msg::DriveCommand>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr rx_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<McuBridge>());
  rclcpp::shutdown();
  return 0;
}
