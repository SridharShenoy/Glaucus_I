# 0004: Phones talk to a web gateway, never to ROS

**Status:** accepted

## Decision
The topside runs a FastAPI gateway serving a plain web app (no build step).
Phones connect over the topside's Wi-Fi with HTTP + WebSocket. DDS is pinned
to the tether interface (`config/cyclonedds_tether.xml`).

## Why
- DDS discovery and traffic over Wi-Fi is notoriously flaky and would compete
  with the app for bandwidth.
- A web app works on every phone and laptop with zero install, and one
  person can change it without a toolchain.
- One choke point for commands: easy to add a PIN/auth, rate limiting, and
  logging later.
- `--mock` mode runs the simulator behind the same API, so the app can be
  built and demoed with no robot.

## Rejected
- rosbridge/foxglove_bridge straight to the phone: exposes the whole ROS
  graph to whoever's on Wi-Fi, and still puts ROS-shaped traffic over Wi-Fi.
  (Foxglove *is* great for engineers on the topside laptop. Use it there.)
- Native app: app-store friction for zero benefit at this stage. Wrap the web
  app with Capacitor later if needed.
