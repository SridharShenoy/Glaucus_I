# Control design and tuning

Companion to [sweep_algorithm.md](sweep_algorithm.md), which explains *why*.
This doc is the *how to tune it* checklist.

## The loops

| Loop | Runs on | Rate | Input → output | Gains |
|---|---|---|---|---|
| Wheel speed | MCU | 200 Hz+ | wheel speed error → PWM | TODO, per motor driver |
| Heading | MCU | 200 Hz | heading error → turn rate | `kp_heading`, (`kd`) |
| Depth hold | Pi | 20 Hz | depth error → heading offset | `kp_depth`, `ki_depth` |
| Sweep logic | Pi | 20 Hz | state machine | thresholds, distances |

Inner loops first. Never tune an outer loop on top of an untuned inner loop.

## Tuning order

### 1. Wheel speed (bench, robot on blocks)
- Command 0.1 m/s; measure with encoders. Add PI until it tracks within ~5%.
- Check both wheels match. A mismatch shows up later as heading drift.

### 2. Heading (dry vertical wall)
Build a test wall (plywood or a whiteboard, vertical). The robot needs to
stick to it the same way it sticks to the hull, or you hold it with a light
counterweight.

- Command heading steps: 0 → 90° → 180° → 0, with `v = 0` (pivot turns).
- Start `kp_heading = 1.0`. Increase until it turns briskly; back off when it
  overshoots more than ~5° or oscillates. Add `kd` if you need more `kp` but
  get overshoot.
- Then repeat with `v = 0.15 m/s`. It should hold heading within ~2°.
- Check the **sign** first: command +10° and confirm the heading estimate
  increases. A flipped sign is the #1 first-day bug (hull side, motor wiring).

### 3. Depth hold (tank)
The depth sensor doesn't work on a dry wall. Options for dry testing:
fake depth from odometry along the vertical axis, or tape a scale on the
wall and use the camera. In the tank:

- `ki_depth = 0`. Start `kp_depth = 1.0`. Drive a lane. Plot depth vs time
  from the bag.
- Raise `kp_depth` until corrections are snappy but the path doesn't weave.
  Weaving = too much `kp`, or the heading loop is too slow for it.
- You'll see a steady sag below the lane (gravity). Add `ki_depth` (start
  ~0.2× `kp_depth`) until the sag goes away over ~5–10 s.
- Target: ±2 cm. Lanes overlap by 6 cm (20% of 30 cm), so ±2 cm leaves margin.

### 4. End-of-lane detection (tank, from bags)
- Record runs where the robot drives to the tank wall / a hull mock-up edge.
- Plot `cam_edge`, `stall`, and `score` vs time around each real end.
- Pick weights/threshold so real ends cross within ~0.2 s and normal driving
  never crosses for 4 ticks. Count false alarms per hour of driving.
- Replay bags through the C++ detector offline to iterate fast.

### 5. Whole sweep
- Run the full sweep on a hull mock-up (or the tank wall with taped "edges").
- Compare the coverage map against photos.

## Why not something fancier?

- **MPC / LQR**: the robot is slow (0.15 m/s) and the dynamics are simple.
  A well-tuned PI cascade will be within a centimeter of anything fancier, and
  you can explain it on a slide.
- **Full 3D SLAM for control**: not needed. Depth + gravity-heading + edge
  detection fully determine the sweep. SLAM is a coverage-map / demo upgrade
  (see `glaucus_topside/hull_mapper.py`).
