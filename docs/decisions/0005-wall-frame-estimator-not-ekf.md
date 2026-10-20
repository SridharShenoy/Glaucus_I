# 0005: A tiny wall-frame estimator instead of an EKF

**Status:** accepted (validated in sim)

## Decision
`WallEstimator`: z = depth sensor, heading = gravity complementary filter (on
the MCU), x = odometry (optical flow preferred, wheels as fallback), with a
gated snap to the known stern x at stern lane ends.

## Why not robot_localization / an EKF
In the unrolled-hull frame the states are nearly decoupled and two of the three
are measured directly. An EKF adds covariance tuning and frame bookkeeping
(robot_localization assumes a 3D world frame, awkward on a curved wall) for
little gain. Revisit if doing real 3D hull SLAM.

## Sim findings (`python sim/sweep_sim.py`)
| x source | mean / max error |
|---|---|
| Wheels only | ~0.22 / 0.5 m |
| Wheels + stern snap | ~0.25 / 0.55 m (no better: slip cancels on round trips) |
| Optical flow + stern snap | ~0.04 / 0.08 m |

Also tried: calibrating wheel slip during step-downs using the depth sensor.
Failed in sim because gravity makes down-slip different from sideways-slip.
Measure both on the real robot before trying again.

## Consequences
Optical flow (camera phase correlation, or a PMW3901-style sensor) is the
highest-value perception item after edge detection.
