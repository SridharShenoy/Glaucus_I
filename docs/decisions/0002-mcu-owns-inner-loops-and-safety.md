# 0002: The MCU owns the fast loops, the low-level sensors, and safety

**Status:** accepted

## Decision
A Teensy 4.1 runs heading estimation (gravity + gyro), the heading loop, wheel
speed loops, and the watchdog, and reads every low-level sensor (IMU, depth,
encoders, motor current, adhesion, leak). The Pi only sends `(mode, v,
heading_cmd, brush_on)` at 20 Hz.

## Why
- Linux isn't real-time. The Pi will occasionally stall for tens of ms; a
  200 Hz heading loop must not care.
- **Safety can't depend on Linux.** If the Pi crashes, ROS hangs, or the USB
  cable falls out, the MCU stops the wheels within 200 ms and keeps adhesion on.
- One sensor stream with one clock makes timestamps trivial.

## Consequences
- The heading loop gains live in firmware (default values mirrored in
  `SweepParams` so the sim matches).
- Hull side (port/starboard) flips the heading sign. Currently a build flag;
  TODO make it a command field.
