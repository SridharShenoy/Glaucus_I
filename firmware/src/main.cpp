// Glaucus I — MCU main loop.
//
// Responsibilities (and ONLY these):
//   1. read sensors at 200 Hz, estimate heading from gravity + gyro
//   2. close the heading loop -> wheel speeds
//   3. receive CommandPackets from the Pi, send TelemetryPackets at 100 Hz
//   4. SAFETY: watchdog, adhesion monitor, leak -> stop on its own
//
// Anything smarter (sweep logic, mapping, vision) lives on the Pi/topside.
// If you find yourself adding mission logic here, stop and put it in
// glaucus_core instead.
#include <Arduino.h>

#include "glaucus_protocol.h"
#include "hal.h"
#include "heading_controller.h"
#include "heading_estimator.h"

using namespace glaucus_proto;

#ifndef GLAUCUS_HULL_SIDE
#define GLAUCUS_HULL_SIDE 1
#endif

namespace {

constexpr uint32_t kLoopUs = 5000;        // 200 Hz control
constexpr uint32_t kTelemetryEvery = 2;   // 100 Hz telemetry
constexpr float kAdhesionMin = 0.4f;      // TODO(glaucus): calibrate in the tank

HeadingEstimator g_est(GLAUCUS_HULL_SIDE);
HeadingController g_ctrl(HeadingController::Gains{});

CommandPacket g_cmd;                      // last valid command
uint32_t g_last_cmd_ms = 0;
uint16_t g_faults = FAULT_NONE;
uint16_t g_tx_seq = 0;
float g_ds_accum = 0.0f;

uint8_t g_rx_buf[128];
size_t g_rx_len = 0;

// Read bytes until a 0x00 delimiter, then COBS-decode + CRC-check one packet.
void poll_serial() {
  while (Serial.available()) {
    const uint8_t b = Serial.read();
    if (b != 0) {
      if (g_rx_len < sizeof g_rx_buf) g_rx_buf[g_rx_len++] = b;
      else g_rx_len = 0;                     // overflow: drop frame
      continue;
    }
    uint8_t dec[sizeof g_rx_buf];
    const size_t n = cobs_decode(g_rx_buf, g_rx_len, dec);
    g_rx_len = 0;
    if (n == sizeof(CommandPacket) && dec[0] == MSG_COMMAND) {
      CommandPacket c;
      memcpy(&c, dec, sizeof c);
      if (verify(c)) {
        g_cmd = c;
        g_last_cmd_ms = millis();
        g_faults &= ~FAULT_BAD_PACKET;
      } else {
        g_faults |= FAULT_BAD_PACKET;
      }
    }
  }
}

void send_telemetry(const hal::Imu& imu, float depth, float vl, float vr) {
  TelemetryPacket t;
  t.seq = g_tx_seq++;
  t.t_ms = millis();
  t.mode = g_cmd.mode;
  t.faults = g_faults;
  t.heading = g_est.heading();
  t.heading_rate = g_est.rate();
  t.wall_tilt = g_est.wall_tilt();
  t.depth = depth;
  t.wheel_ds = g_ds_accum;
  t.wheel_v_left = vl;
  t.wheel_v_right = vr;
  t.motor_current = hal::read_motor_current_a();
  t.adhesion = hal::read_adhesion();
  t.battery_v = 0.0f;  // TODO(glaucus)
  seal(t);
  g_ds_accum = 0.0f;

  uint8_t enc[sizeof(TelemetryPacket) + 4];
  const size_t n = cobs_encode(reinterpret_cast<uint8_t*>(&t), sizeof t, enc);
  Serial.write(enc, n);
  Serial.write(uint8_t(0));
  (void)imu;
}

}  // namespace

void setup() {
  Serial.begin(115200);   // USB CDC: baud is ignored, runs at USB speed
  if (!hal::init()) g_faults |= FAULT_IMU;
  const hal::Imu imu = hal::read_imu();
  g_est.reset_from_accel(imu.ax, imu.ay);
  hal::stop_all();
}

void loop() {
  static uint32_t next_us = micros();
  static uint32_t tick = 0;
  const uint32_t now_us = micros();
  if (int32_t(now_us - next_us) < 0) {
    poll_serial();
    return;
  }
  next_us += kLoopUs;
  const float dt = kLoopUs * 1e-6f;
  poll_serial();

  // ---- sense ----
  const hal::Imu imu = hal::read_imu();
  if (imu.ok) g_est.update(imu.ax, imu.ay, imu.az, imu.gz, dt);
  else g_faults |= FAULT_IMU;
  const float depth = hal::read_depth_m();
  float dl, dr;
  g_ds_accum += hal::read_wheel_distance_m(&dl, &dr);

  // ---- safety: these override whatever the Pi asked for ----
  const bool watchdog = (millis() - g_last_cmd_ms) > WATCHDOG_MS;
  g_faults = watchdog ? (g_faults | FAULT_WATCHDOG) : (g_faults & ~FAULT_WATCHDOG);
  if (hal::read_adhesion() < kAdhesionMin) g_faults |= FAULT_ADHESION_LOW;
  else g_faults &= ~FAULT_ADHESION_LOW;
  if (hal::leak_detected()) g_faults |= FAULT_LEAK;   // latched on purpose: a leak needs a human

  // ---- act ----
  WheelSpeeds w{0.0f, 0.0f};
  const bool may_drive = !watchdog && !(g_faults & (FAULT_ADHESION_LOW | FAULT_LEAK | FAULT_IMU));
  switch (may_drive ? g_cmd.mode : (g_cmd.mode == MODE_DISABLED ? MODE_DISABLED : MODE_HOLD)) {
    case MODE_DRIVE:
      w = g_ctrl.mix(g_cmd.v, g_ctrl.omega(g_cmd.heading_cmd, g_est.heading(), g_est.rate()));
      hal::set_adhesion(true);
      hal::set_brush(g_cmd.brush_on);
      break;
    case MODE_MANUAL:
      w = g_ctrl.mix(g_cmd.v, g_cmd.omega_cmd);
      hal::set_adhesion(true);
      hal::set_brush(g_cmd.brush_on);
      break;
    case MODE_HOLD:
      hal::set_adhesion(true);
      hal::set_brush(false);
      break;
    default:  // MODE_DISABLED
      hal::set_adhesion(false);
      hal::set_brush(false);
      break;
  }
  // TODO(glaucus): adhesion-loss response. Stopping is right, but if adhesion
  // keeps dropping, the robot should probably go limp and let the tether hold it.
  hal::set_wheel_speeds(w.left, w.right);

  if (++tick % kTelemetryEvery == 0) send_telemetry(imu, depth, w.left, w.right);
}
