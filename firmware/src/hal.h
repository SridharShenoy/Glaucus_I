// hal.h — hardware abstraction. Everything board/sensor-specific lives behind
// these functions, implemented in hal_teensy.cpp.
//
// DESIGN CHOICE: the MCU owns ALL low-level sensors (IMU, depth, encoders,
// motor current, adhesion, leak) and reports them in ONE telemetry packet.
// One clock, one stream, timestamps line up. The Pi only owns the cameras.
//
// Every function here is a placeholder. Fill them in once parts are chosen.
#pragma once

#include <stdint.h>

namespace hal {

struct Imu {
  float ax, ay, az;  // m/s^2, robot frame (x fwd, y left, z out of hull)
  float gx, gy, gz;  // rad/s
  bool ok;
};

bool init();                       // returns false if any critical device is missing

Imu read_imu();                    // TODO(glaucus): BNO085 / ICM-42688 over I2C/SPI
float read_depth_m();              // TODO(glaucus): Blue Robotics Bar30 (MS5837) over I2C.
                                   //   depth = (P - P_surface) / (rho * g); rho ~ 1025 sea, 1000 fresh/tank
bool depth_ok();
void zero_depth();                 // call at the surface before a mission

float read_wheel_distance_m(float* left, float* right);  // since last call; TODO: quadrature encoders
float read_motor_current_a();      // TODO: driver current sense
float read_adhesion();             // 0..1. TODO: suction pressure sensor or thruster-based
bool leak_detected();              // TODO: leak probe in the electronics can

void set_wheel_speeds(float left_mps, float right_mps);  // TODO: closed-loop wheel speed (PI on encoders)
void set_brush(bool on);
void set_adhesion(bool on);        // suction pump / thrusters
void stop_all();                   // motors + brush off, adhesion STAYS ON (don't fall off the hull)

}  // namespace hal
