# Architecture

## Three computers, three jobs

```
 ┌──────────── phone / laptop ────────────┐
 │  web app (webapp/frontend)              │
 └──────────────┬──────────────────────────┘
                │ Wi-Fi: HTTP + WebSocket only (never ROS)
 ┌──────────────▼──────────── TOPSIDE (laptop or mini PC, Wi-Fi AP) ─────────┐
 │  web gateway (webapp/gateway)     mission_manager     coverage_mapper      │
 │  bag recorder                     hull_mapper (stretch: mosaic / SLAM)     │
 └──────────────┬─────────────────────────────────────────────────────────────┘
                │ tether: Ethernet, ROS 2 (DDS pinned to this interface)
 ┌──────────────▼──────────── ROBOT: Raspberry Pi (2 GB) ─────────────────────┐
 │  sweep_executor (glaucus_core)    vision_node        mcu_bridge            │
 │  camera_ros                                                                 │
 └──────────────┬─────────────────────────────────────────────────────────────┘
                │ USB serial: COBS + CRC packets (shared/protocol)
 ┌──────────────▼──────────── ROBOT: MCU (Teensy 4.1) ───────────────────────┐
 │  heading estimator + heading loop, wheel speed loops, watchdog,           │
 │  IMU, depth sensor, encoders, motor current, adhesion, leak               │
 └────────────────────────────────────────────────────────────────────────────┘
```

| Layer | Owns | Must keep working if... |
|---|---|---|
| MCU | Motors, fast loops, all low-level sensors, **safety** | ...the Pi crashes. Watchdog → stop wheels, keep adhesion on. |
| Pi | The sweep (state machine + estimator), camera, cheap vision | ...topside/Wi-Fi drops. Finish the lane, then hold. |
| Topside | Mission control, coverage map, logging, app gateway, heavy vision | Nothing depends on it in real time. |
| App | Start / pause / stop / e-stop, show progress | Nothing depends on it. |

The rule: **anything that has to keep working when a link drops lives below
that link.**

## Data flow during a sweep

1. App → `POST /api/mission/start` → gateway → `/glaucus/mission/start`
   (mission_manager) → `RunSweep` action goal → sweep_executor on the Pi.
2. MCU → telemetry (100 Hz: heading, depth, wheel distance, faults) →
   mcu_bridge → `/glaucus/telemetry`.
3. Camera → vision_node → `/glaucus/evidence` (edge, stall, optical flow).
4. sweep_executor (20 Hz): telemetry + evidence → `SweepController.update()`
   → `/glaucus/drive_cmd` (v, heading) → mcu_bridge → MCU heading loop → wheels.
5. sweep_executor → `/glaucus/sweep_status` (5 Hz) + `/glaucus/sweep_events`
   → coverage_mapper → `/glaucus/coverage` → gateway → WebSocket → app.

## Failure behavior

| Failure | Detected by | Response |
|---|---|---|
| Pi crashes / USB unplugged | MCU watchdog (200 ms) | Wheels + brush stop, adhesion stays on |
| Tether or topside drops | sweep_executor (no heartbeat 3 s) | Finish current lane, hold (TODO: currently holds immediately) |
| Phone disconnects | Nobody needs to | Nothing. Mission continues. |
| Adhesion low | MCU | Stop, report fault. TODO: decide escalation |
| Leak | MCU | Stop, latch fault, needs a human |
| Can't find lane end (camera blind) | Stall cue / `max_lane_length` guard | Ends lane by stall or after 15 m of odometry |
| E-stop pressed | App → gateway → `/glaucus/estop` directly | HOLD. Bypasses mission logic. |

## Repo map

| Path | What | Language |
|---|---|---|
| `sim/` | Algorithm sandbox: plant model, controller, plots | Python |
| `ros2_ws/src/glaucus_core` | The sweep logic, ROS-free | C++17 |
| `ros2_ws/src/glaucus_bot` | Pi nodes (thin wrappers) | C++ |
| `ros2_ws/src/glaucus_topside` | Topside nodes | Python |
| `ros2_ws/src/glaucus_interfaces` | Messages + action | ROS IDL |
| `ros2_ws/src/glaucus_bringup` | Launch files, params, DDS config | Python/YAML |
| `firmware/` | MCU firmware | C++ (PlatformIO/Arduino) |
| `shared/protocol` | Pi↔MCU wire format, used by both | C++ header |
| `webapp/` | Gateway (FastAPI) + frontend (vanilla JS) | Python/JS |
| `docs/decisions/` | Why things are the way they are | Markdown |

## Pi resource budget (2 GB)

Rough, to sanity-check before adding anything heavy:

| Thing | RAM |
|---|---|
| Ubuntu Server 24.04, no desktop | ~300 MB |
| ROS 2 daemon + 3 C++ nodes | ~150 MB |
| camera_ros @ 640×480 | ~80 MB |
| vision_node with OpenCV | ~100 MB |
| **Total** | **~650 MB** of 2 GB |

CPU is the tighter limit. Camera encoding + optical flow is the big one.
Pi 4 has hardware H.264; Pi 5 doesn't. Drop to 640×480 @ 15 fps before
anything else.
