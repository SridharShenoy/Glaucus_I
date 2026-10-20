# Interfaces

Source of truth: `ros2_ws/src/glaucus_interfaces` (ROS) and
`shared/protocol/glaucus_protocol.h` (MCU wire format). This page is the index.

## ROS topics

| Topic | Type | Rate | From → to |
|---|---|---|---|
| `/glaucus/telemetry` | `BotTelemetry` | 100 Hz | mcu_bridge → executor, recorder |
| `/glaucus/drive_cmd` | `DriveCommand` | 20 Hz | executor → mcu_bridge (also feeds MCU watchdog) |
| `/glaucus/evidence` | `EndOfLaneEvidence` | ~20 Hz | vision_node → executor |
| `/glaucus/sweep_status` | `SweepStatus` | 5 Hz | executor → coverage_mapper, gateway |
| `/glaucus/sweep_events` | `SweepEvent` | on event | executor → gateway, recorder |
| `/glaucus/coverage` | `nav_msgs/OccupancyGrid` | 1 Hz | coverage_mapper → gateway |
| `/glaucus/topside_heartbeat` | `std_msgs/Empty` | 2 Hz | mission_manager → executor |
| `/glaucus/mission_state` | `std_msgs/String` (JSON) | latched | mission_manager → gateway |
| `/camera/image_raw` | `sensor_msgs/Image` | 15–30 Hz | camera_ros → vision_node (Pi only) |
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | 15–30 Hz | camera_ros → topside |

## Actions and services

| Name | Type | Server |
|---|---|---|
| `/glaucus/run_sweep` | `RunSweep` action | sweep_executor (Pi) |
| `/glaucus/pause` | `std_srvs/SetBool` | sweep_executor |
| `/glaucus/estop` | `std_srvs/Trigger` | sweep_executor |
| `/glaucus/mission/start` | `std_srvs/Trigger` | mission_manager (topside) |
| `/glaucus/mission/pause` | `std_srvs/SetBool` | mission_manager |
| `/glaucus/mission/abort` | `std_srvs/Trigger` | mission_manager |

## Pi ↔ MCU packets

USB serial, COBS-framed (0x00 between packets), CRC-16/CCITT-FALSE.

| Packet | Size | Rate | Direction |
|---|---|---|---|
| `CommandPacket` | 20 B | 20 Hz | Pi → MCU |
| `TelemetryPacket` | 54 B | 100 Hz | MCU → Pi |

Changing either struct: bump `PROTOCOL_VERSION`, update the `static_assert`
sizes in `shared/protocol/test_protocol.cpp`, and update `BotTelemetry.msg` /
`DriveCommand.msg` to match.

## Web gateway (HTTP)

See the docstring at the top of `webapp/gateway/server.py`. Status JSON fields
match `SweepStatus` plus `mission`, `connected`, `events`, `coverage`.

## Frames and units

SI everywhere. The "hull-side frame" (`hull_side`) is the unrolled hull:
x along hull (m, +bow), z depth (m, +down), heading 0 = bow, +π/2 = down.
