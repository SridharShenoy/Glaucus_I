# Raw MCU telemetry, republished 1:1 by mcu_bridge_node at ~100 Hz.
# Mirrors glaucus_proto::TelemetryPacket (shared/protocol/glaucus_protocol.h).

std_msgs/Header header       # stamp = Pi receive time (MCU t_ms kept below)
uint32 mcu_time_ms

uint8 MODE_DISABLED=0
uint8 MODE_HOLD=1
uint8 MODE_DRIVE=2
uint8 MODE_MANUAL=3
uint8 mode

uint16 faults                # bitfield, see FaultBits in glaucus_protocol.h

float32 heading              # rad, gravity-referenced, 0 = facing bow, +pi/2 = facing down
float32 heading_rate         # rad/s
float32 wall_tilt            # rad, 0 = vertical wall
float32 depth                # m, + down
float32 wheel_ds             # m since previous packet
float32 wheel_v_left         # m/s
float32 wheel_v_right        # m/s
float32 motor_current        # A
float32 adhesion             # 0..1
float32 battery_v            # V
