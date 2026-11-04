# Testing plan: desk → bench → wall → tank → boat

Each stage has a pass condition. Don't move on until it passes; each stage
is much more expensive to debug than the one before.

| # | Stage | Where | What runs | Pass when |
|---|---|---|---|---|
| 0 | Desk | Any laptop | `tools/check_all.sh`, sim (2D + 3D, see [simulation.md](simulation.md)), app in `--mock` | All checks pass; sim coverage ≥ 95% on 10 seeds |
| 1 | Bench comms | Teensy on USB, no motors | firmware + mcu_bridge + executor | Telemetry at 100 Hz; unplug USB → watchdog fault within 200 ms |
| 2 | Bench drive | Robot on blocks | + motors | Wheel speed loop tracks ±5%; e-stop from the app stops wheels < 0.3 s |
| 3 | Dry wall | Vertical plywood/whiteboard | Heading loop, sweep with faked depth | Holds heading ±2°; full fake sweep completes on a taped-out "hull" |
| 4 | Tank | Prof. Hsieh's tank (pending) | Everything, real depth | Depth hold ±2 cm; end detection: 0 missed ends, < 1 false alarm per 10 min |
| 5 | Hull mock | Tank, curved panel | Everything | Step-down retries work at a curved edge; coverage map matches photos |
| 6 | Boat | Dock | Everything | Demo-ready |

**Record bags at every stage from 3 onward**
(`ros2 launch glaucus_bringup topside.launch.py record:=true`). A bag lets you
replay one tank afternoon all week.

## Things to measure early (they change the design)

- **Wheel slip on a slimy surface**: tells you how bad wheel-only odometry
  really is (sim assumes 6% ± 3%).
- **Slip going down vs. sideways**: if they're close, step-down slip
  calibration might work after all.
- **Camera image of a hull edge underwater, with your lights**: decides how
  far classical CV gets you before you need a learned edge detector.
- **Optical flow quality on clean vs. fouled hull**: decides whether flow can
  replace wheels for x.
