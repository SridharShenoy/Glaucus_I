# What the sweep executor (or manual teleop) wants the MCU to do.
# mcu_bridge_node turns this into a glaucus_proto::CommandPacket at 20 Hz.
# If nobody publishes for >200 ms, the MCU watchdog stops the wheels.

uint8 MODE_DISABLED=0
uint8 MODE_HOLD=1
uint8 MODE_DRIVE=2
uint8 MODE_MANUAL=3
uint8 mode

bool brush_on
float32 v                    # m/s forward
float32 heading_cmd          # rad, used in MODE_DRIVE (MCU closes the heading loop)
float32 omega_cmd            # rad/s, used in MODE_MANUAL (joystick)
