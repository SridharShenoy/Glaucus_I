# 0001: ROS 2 for plumbing, but the algorithm core is ROS-free

**Status:** accepted

## Context
Four-person team, one semester, SLAM-ish ambitions, a 2 GB Pi on the robot and
a laptop topside. Options: ROS 2, or hand-rolled messaging (ZeroMQ/protobuf).

## Decision
Use ROS 2 Jazzy on the Pi and topside. Keep all decision-making logic in
`glaucus_core`, a plain C++17 library with no ROS dependency. ROS nodes are thin
wrappers.

## Why
- **Bags.** Record one tank session, replay it all week while tuning. This
  alone justifies ROS for a project where tank time is scarce.
- Off-the-shelf camera driver, image transport, visualization (Foxglove/RViz),
  launch files, parameters.
- Clean ownership boundaries for four people: each person owns nodes; the
  interfaces are messages.
- ROS-free core means: unit tests with only a compiler, a Python↔C++ parity
  test, and no ROS needed to work on the algorithm.

## Costs we accept
- colcon/ament learning curve (~2 weeks).
- DDS over Wi-Fi is flaky → mitigated by ADR 0004 (DDS only on the tether).

## Rejected
- **micro-ROS on the MCU**: one more thing to debug for two packet types. See
  `shared/protocol/glaucus_protocol.h`.
- **No ROS**: you'd rebuild logging, replay and visualization yourselves.
