# 0003: Lanes are defined by depth; lane ends are detected, not computed

**Status:** accepted (validated in sim)

## Context
Localization budget: depth sensor, IMU, wheel odometry, camera. No DVL
($2k budget). Wheel odometry drifts on slimy hulls.

## Decision
- A lane = constant depth, held by a PI loop on the pressure sensor.
- A lane ends when fused camera + stall evidence says so (debounced, with
  a VERIFY step for suspiciously short lanes).
- Heading comes from gravity (absolute on a wall), not a magnetometer.
- Along-hull position x is only used for the coverage map.

## Why
Two of the three coordinates (depth, heading) are measured absolutely. The
pattern itself doesn't need the third (x) at all, so odometry drift can only
make the *map* wrong, never make the robot clean the wrong place.

## Consequences
- Need a reliable edge cue. Camera + stall fusion is the plan; tune from bags.
- The coverage map's x accuracy depends on odometry. Sim says optical flow
  gets it to ~5 cm; wheel-only is ~0.5 m worst case. See ADR 0005.
- Doesn't handle the flat bottom of the hull (gravity heading degrades).
  Out of scope for now.
