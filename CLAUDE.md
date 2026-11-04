# CLAUDE.md — instructions for Claude Code in this repo

## What this is
**Glaucus I**: a senior-design autonomous robot that cleans boat hulls in the
water (KeelCrab-style). It crawls on the hull, sweeps it in horizontal lanes,
and is supervised from a phone. Four-person team, ~$2k budget (no DVL).

Read these before non-trivial work:
- `docs/architecture.md`: who runs what (MCU / Pi / topside / app) and why
- `docs/sweep_algorithm.md`: the algorithm, explained from scratch
- `docs/simulation.md`: which simulations exist / are needed, and the 3D environment
- `docs/decisions/`: the reasons behind the big choices. Don't silently undo one;
  if you think one is wrong, say so and propose a new ADR.

## Golden rules
1. **Safety lives on the MCU.** The watchdog in `firmware/src/main.cpp` must
   stop the wheels if commands stop arriving. Never add a code path where the
   MCU keeps driving without fresh commands. Never let anything turn adhesion
   off except an explicit `MODE_DISABLED`.
2. **Algorithm changes go Python first.** Edit `sim/sweep_sim.py`, run it, look
   at the plot, then port 1:1 to `ros2_ws/src/glaucus_core`, regenerate the
   parity trace, and run `./tools/check_all.sh`. The parity test must pass.
3. **`glaucus_core` stays ROS-free.** Plain C++17, no rclcpp includes. ROS
   nodes in `glaucus_bot` are thin wrappers: subscribe → fill `Sensors` →
   `update()` → publish.
4. **Parameters live in three places and must match**: Python `SweepParams`,
   C++ `SweepParams` (`types.hpp`), and `config/sweep_params.yaml`.
   `tools/check_params.py` enforces it.
5. **The wire protocol has one definition**: `shared/protocol/glaucus_protocol.h`.
   Change a struct → bump `PROTOCOL_VERSION`, fix the `static_assert`s in
   `shared/protocol/test_protocol.cpp`, update `BotTelemetry.msg`/`DriveCommand.msg`.
6. **Phones never join the ROS graph.** The app talks HTTP/WebSocket to
   `webapp/gateway`. DDS is pinned to the tether (`cyclonedds_tether.xml`).
7. **No control loops in Python.** Topside Python is for mission logic,
   mapping, logging and the gateway.

## Commands
```bash
./tools/check_all.sh                         # everything checkable without ROS/hardware. Run before finishing any task.
python3 sim/sweep_sim.py                     # run the sweep sim, writes docs/img/sweep_sim.png
python3 sim/sweep_sim.py --seed 7 --show     # other seeds, interactive plot
python3 sim/sim3d.py [--html build/sim3d.html]   # 3D hull sim: same controller, curved hull (--hull plane = 2D control case)
python3 sim/test_sim3d.py                    # its tests; the plane==2D-sim test must always pass
python3 sim/sweep_sim.py --quiet --dump-trace ros2_ws/src/glaucus_core/test/data/sim_trace_seed2.csv   # regenerate parity trace
python3 webapp/gateway/server.py --mock      # app on http://localhost:8000 driven by the sim

# with ROS 2 Jazzy installed:
cd ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch glaucus_bringup bot.launch.py        # on the Pi
ros2 launch glaucus_bringup topside.launch.py record:=true   # on topside
python3 webapp/gateway/server.py --ros

# firmware (PlatformIO):
cd firmware && pio run && pio run -t upload
```

## Conventions
- C++17, `-Wall -Wextra` clean. Python 3.10+, type hints.
- SI units everywhere (m, s, rad). Angles: always `wrap()` an angle *error*.
- Hull-side frame: x along hull (+bow), z depth (+down), heading 0 = bow,
  +π/2 = facing down. See `docs/interfaces.md`.
- Placeholders are marked `TODO(glaucus)`. `grep -rn "TODO(glaucus)"` is the backlog.
- Comments explain *why* (design choices), not what the next line does.
- Keep the plain-language explanations in docs accurate when you change behavior.
  The team reads them to learn the system.

## What's real vs. placeholder
| Part | Status |
|---|---|
| `sim/sweep_sim.py` | **Working.** Plant model, controller, three estimators, coverage, plots |
| `sim/sim3d.py` | **Working + tested.** 3D hull kinematics, IMU/gravity model, area-weighted coverage. Imports the controller from `sweep_sim.py`; never fork the algorithm here. Hull dims are placeholders |
| `glaucus_core` (controller, detector, estimator) | **Working + tested**, parity with the sim |
| `shared/protocol` | **Working + tested** (COBS, CRC, packet layout) |
| `firmware/` heading estimator + controller | **Working + host-tested** |
| `firmware/` HAL (`hal_teensy.cpp`) | Placeholder: needs real sensor/motor drivers |
| `glaucus_bot/sweep_executor_node` | Written, not yet built against ROS. Link-loss policy is simplified |
| `glaucus_bot/mcu_bridge_node` | Written, not yet built against ROS |
| `glaucus_bot/vision_node` | Skeleton: publishes zeros. Edge + optical flow are TODO |
| `glaucus_topside/*` | mission_manager + coverage_mapper written; hull_mapper is a stretch-goal stub |
| `webapp/` | **Working in `--mock`.** `--ros` backend written, untested |

## Suggested next tasks (in priority order)
1. Get the ROS workspace building on Jazzy (`colcon build`). Fix whatever
   breaks in `glaucus_bot` / `glaucus_topside`. Add a launch test that runs
   the executor against a fake MCU node that integrates the sim's plant model.
2. `vision_node`: optical flow via `cv::phaseCorrelate` on a downscaled center
   crop → `flow_ds`, `stall`. (Highest-value perception item; see ADR 0005.)
3. `vision_node`: classical edge score (texture variance in the top band).
   Add a tool to replay bags and plot score vs. time.
4. Executor link-loss policy: "finish current lane, then hold" (currently
   holds immediately). Implement in `glaucus_core` + sim first.
5. Fill in the firmware HAL once parts are chosen (IMU, Bar30, encoders,
   motor driver).
6. Obstacle bypass for mid-lane obstacles (see `docs/sweep_algorithm.md` §10).

## Don't
- Don't add an EKF / robot_localization without reading ADR 0005 first.
- Don't put mission logic on the MCU.
- Don't commit raw-image bags (huge). Compressed images only.
- Don't change parameter defaults in only one place.
