// glaucus_protocol.h — the Pi <-> MCU wire format. SINGLE SOURCE OF TRUTH.
//
// Included by BOTH firmware/ (PlatformIO, via -I../shared/protocol) and
// ros2_ws/src/glaucus_bot (mcu_bridge_node). Never copy-paste this file.
//
// DESIGN CHOICES
//  - Plain USB serial (Teensy shows up as /dev/ttyACM0). No micro-ROS: it's one
//    more thing to debug on a 2 GB Pi, and we only need two packet types.
//  - Fixed-size packed structs + CRC16 + COBS framing. COBS gives us a 0x00
//    delimiter between packets, so a corrupted byte costs one packet, not the
//    stream. CRC catches the corruption.
//  - Pi sends a COMMAND at 20 Hz. MCU sends TELEMETRY at 100 Hz.
//  - The MCU watchdog stops motors if no valid COMMAND for 200 ms. This is THE
//    safety layer: it doesn't care whether the Pi crashed, ROS hung, or the
//    USB cable fell out.
//  - Bump PROTOCOL_VERSION on any struct change. Both sides reject mismatches.
//
// Units on the wire: SI floats (m, m/s, rad, rad/s, Pa). Little-endian (both
// Teensy and Pi are LE).
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace glaucus_proto {

constexpr uint8_t PROTOCOL_VERSION = 1;
constexpr uint16_t WATCHDOG_MS = 200;

enum MsgType : uint8_t { MSG_COMMAND = 0x01, MSG_TELEMETRY = 0x02 };

enum Mode : uint8_t {
  MODE_DISABLED = 0,   // motors off, adhesion off (safe on deck)
  MODE_HOLD = 1,       // adhesion on, wheels stopped (safe on hull)
  MODE_DRIVE = 2,      // follow v + heading_cmd
  MODE_MANUAL = 3,     // follow v + omega directly (joystick from the app)
};

enum FaultBits : uint16_t {
  FAULT_NONE = 0,
  FAULT_WATCHDOG = 1 << 0,       // no command in WATCHDOG_MS
  FAULT_ADHESION_LOW = 1 << 1,   // suction/adhesion below threshold -> about to fall off
  FAULT_IMU = 1 << 2,
  FAULT_DEPTH_SENSOR = 1 << 3,
  FAULT_MOTOR = 1 << 4,          // driver fault / overcurrent
  FAULT_LEAK = 1 << 5,           // leak sensor in the electronics can
  FAULT_BAD_PACKET = 1 << 6,     // version mismatch / CRC errors piling up
};

#pragma pack(push, 1)
struct CommandPacket {
  uint8_t type = MSG_COMMAND;
  uint8_t version = PROTOCOL_VERSION;
  uint16_t seq = 0;
  uint8_t mode = MODE_DISABLED;
  uint8_t brush_on = 0;
  float v = 0.0f;            // m/s
  float heading_cmd = 0.0f;  // rad, used in MODE_DRIVE
  float omega_cmd = 0.0f;    // rad/s, used in MODE_MANUAL
  uint16_t crc = 0;          // CRC16 over everything above
};

struct TelemetryPacket {
  uint8_t type = MSG_TELEMETRY;
  uint8_t version = PROTOCOL_VERSION;
  uint16_t seq = 0;
  uint32_t t_ms = 0;          // MCU millis()
  uint8_t mode = MODE_DISABLED;
  uint8_t pad = 0;
  uint16_t faults = FAULT_NONE;
  float heading = 0.0f;       // rad, gravity-referenced (complementary filter)
  float heading_rate = 0.0f;  // rad/s (gyro about hull normal)
  float wall_tilt = 0.0f;     // rad, 0 = vertical wall; heading gets unreliable as this -> pi/2
  float depth = 0.0f;         // m (pressure -> depth, zeroed at the surface)
  float wheel_ds = 0.0f;      // m, mean wheel distance since last packet
  float wheel_v_left = 0.0f;  // m/s
  float wheel_v_right = 0.0f; // m/s
  float motor_current = 0.0f; // A, summed; spikes = stall
  float adhesion = 0.0f;      // normalized 0..1 (suction pressure / thrust margin)
  float battery_v = 0.0f;     // V (or tether supply voltage)
  uint16_t crc = 0;
};
#pragma pack(pop)

// CRC-16/CCITT-FALSE. Small, table-free; fast enough at these rates.
inline uint16_t crc16(const uint8_t* data, size_t n) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < n; ++i) {
    crc ^= uint16_t(data[i]) << 8;
    for (int b = 0; b < 8; ++b) crc = (crc & 0x8000) ? uint16_t((crc << 1) ^ 0x1021) : uint16_t(crc << 1);
  }
  return crc;
}

template <typename P>
inline void seal(P& p) {
  p.crc = crc16(reinterpret_cast<const uint8_t*>(&p), sizeof(P) - sizeof(p.crc));
}

template <typename P>
inline bool verify(const P& p) {
  return p.version == PROTOCOL_VERSION &&
         p.crc == crc16(reinterpret_cast<const uint8_t*>(&p), sizeof(P) - sizeof(p.crc));
}

// COBS encode. out must hold n + n/254 + 1 bytes. Returns encoded length
// (without the trailing 0x00 delimiter - the sender appends it).
inline size_t cobs_encode(const uint8_t* in, size_t n, uint8_t* out) {
  size_t read = 0, write = 1, code_idx = 0;
  uint8_t code = 1;
  while (read < n) {
    if (in[read] == 0) {
      out[code_idx] = code;
      code = 1;
      code_idx = write++;
      ++read;
    } else {
      out[write++] = in[read++];
      if (++code == 0xFF) {
        out[code_idx] = code;
        code = 1;
        code_idx = write++;
      }
    }
  }
  out[code_idx] = code;
  return write;
}

// COBS decode (input excludes the 0x00 delimiter). Returns decoded length, or
// 0 on malformed input.
inline size_t cobs_decode(const uint8_t* in, size_t n, uint8_t* out) {
  size_t read = 0, write = 0;
  while (read < n) {
    uint8_t code = in[read];
    if (code == 0) return 0;
    ++read;
    for (uint8_t i = 1; i < code; ++i) {
      if (read >= n) return 0;
      out[write++] = in[read++];
    }
    if (code != 0xFF && read < n) out[write++] = 0;
  }
  return write;
}

}  // namespace glaucus_proto
