"""Backends for the web gateway.

The gateway (server.py) only talks to a Backend. Two implementations:

  MockBackend - runs sim/sweep_sim.py in a background thread. Lets you build
                and demo the app with no robot, no ROS, on any laptop.
  RosBackend  - an rclpy node that subscribes to the real topics and calls
                the mission_manager services.

DESIGN CHOICE: the app NEVER talks to the bot directly and NEVER joins the ROS
graph. Phones speak HTTP + WebSocket to this gateway on the topside machine;
the gateway speaks ROS over the tether. That keeps DDS off Wi-Fi and gives one
place to add auth / rate limits later.
"""
from __future__ import annotations

import math
import sys
import threading
import time
from collections import deque
from pathlib import Path
from typing import Protocol

REPO = Path(__file__).resolve().parents[2]


class Backend(Protocol):
    def start(self) -> tuple[bool, str]: ...
    def pause(self, paused: bool) -> tuple[bool, str]: ...
    def abort(self) -> tuple[bool, str]: ...
    def estop(self) -> tuple[bool, str]: ...
    def status(self) -> dict: ...
    def hull(self) -> dict: ...
    def coverage(self) -> dict: ...


# ---------------------------------------------------------------------------
class MockBackend:
    """The simulator, stepped in real time x `speed`."""

    def __init__(self, speed: float = 4.0, seed: int = 2):
        sys.path.insert(0, str(REPO / "sim"))
        import sweep_sim  # noqa: E402  (path set above)

        self._mod = sweep_sim
        self.speed, self.seed = speed, seed
        self._lock = threading.Lock()
        self._sim = None
        self._mission = "IDLE"
        self._events: deque = deque(maxlen=50)
        self._seen_events = 0
        self._t0 = time.time()
        threading.Thread(target=self._loop, daemon=True).start()

    # -- commands --------------------------------------------------------
    def start(self):
        with self._lock:
            if self._sim is not None and not self._sim.done:
                return False, "A sweep is already running."
            self.seed += 1
            self._sim = self._mod.Simulation(seed=self.seed)
            self._events.clear()
            self._seen_events = 0
            self._mission = "RUNNING"
            self._log("mission_started", {})
        return True, "Sweep started."

    def pause(self, paused: bool):
        with self._lock:
            if self._sim is None or self._sim.done:
                return False, "No sweep is running."
            self._sim.paused = paused
            self._mission = "PAUSED" if paused else "RUNNING"
            self._log("paused" if paused else "resumed", {})
        return True, "Paused." if paused else "Resumed."

    def abort(self):
        with self._lock:
            if self._sim is None or self._sim.done:
                return False, "No sweep is running."
            self._sim.ctrl.abort()
            self._mission = "STOPPED"
            self._log("mission_stopped", {})
        return True, "Sweep stopped. Robot is holding on the hull."

    def estop(self):
        with self._lock:
            if self._sim is not None:
                self._sim.ctrl.abort()
            self._mission = "E-STOP"
            self._log("estop", {})
        return True, "Emergency stop. Motors off, adhesion on."

    # -- queries ---------------------------------------------------------
    def status(self) -> dict:
        with self._lock:
            if self._sim is None:
                return {"mission": "IDLE", "state": "IDLE", "connected": True, "mock": True,
                        "events": list(self._events)}
            s = self._sim.status()
            s.update(mission=self._mission, connected=True, mock=True,
                     planned_lanes=self._planned_lanes(), events=list(self._events),
                     x_true=round(self._sim.plant.x, 3))
            return s

    def hull(self) -> dict:
        h = self._mod.Hull()
        p = self._mod.SweepParams()
        zs = [i * h.draft / 40 for i in range(41)]
        outline = ([[h.x_min(z), z] for z in zs] + [[h.x_max(z), z] for z in reversed(zs)])
        return {"length": h.length, "draft": h.draft, "outline": outline,
                "lane_spacing": p.lane_spacing, "first_lane_depth": p.first_lane_depth,
                "max_depth": p.max_depth, "brush_width": p.brush_width}

    def coverage(self, factor: int = 3) -> dict:
        """Downsampled grid as rows of chars: '#' cleaned, '.' hull, ' ' not hull."""
        with self._lock:
            if self._sim is None:
                return {"res": 0, "rows": []}
            g = self._sim.grid
            rows = []
            for i in range(0, g.nz, factor):
                row = []
                for j in range(0, g.nx, factor):
                    hull = g.cleanable[i:i + factor, j:j + factor]
                    if not hull.any():
                        row.append(" ")
                    else:
                        done = (g.cleaned[i:i + factor, j:j + factor] & hull).sum() / hull.sum()
                        row.append("#" if done > 0.5 else ".")
                rows.append("".join(row))
            return {"res": g.res * factor, "rows": rows, "coverage": round(g.coverage(), 4)}

    # -- internals -------------------------------------------------------
    def _planned_lanes(self) -> int:
        p = self._mod.SweepParams()
        return int(math.floor((p.max_depth - p.first_lane_depth) / p.lane_spacing + 1e-9)) + 1

    def _log(self, name: str, info: dict) -> None:
        self._events.appendleft({"name": name, "t": round(time.time() - self._t0, 1), **info})

    def _loop(self) -> None:
        dt = self._mod.Simulation.dt
        while True:
            time.sleep(dt / self.speed)
            with self._lock:
                sim = self._sim
                if sim is None or sim.done or sim.paused:
                    continue
                sim.step()
                for name, info in sim.ctrl.events[self._seen_events:]:
                    self._log(name, info)
                self._seen_events = len(sim.ctrl.events)
                if sim.done and self._mission == "RUNNING":
                    self._mission = "COMPLETE"


# ---------------------------------------------------------------------------
class RosBackend:
    """Bridges to the real robot through mission_manager.

    STATUS: written against the interfaces in ros2_ws but NOT yet run against
    a live graph. TODO(glaucus): test on the topside machine with the bot up.
    """

    def __init__(self):
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from std_msgs.msg import String
        from std_srvs.srv import SetBool, Trigger
        from nav_msgs.msg import OccupancyGrid
        from glaucus_interfaces.msg import SweepEvent, SweepStatus

        rclpy.init()
        self._SetBool, self._Trigger = SetBool, Trigger
        node = Node("web_gateway")
        self._node = node
        self._status: dict = {"mission": "IDLE", "state": "IDLE", "connected": False}
        self._events: deque = deque(maxlen=50)
        self._grid: OccupancyGrid | None = None
        self._last_status_t = 0.0
        kinds = {0: "end_of_lane", 1: "end_rejected", 2: "step_down_blocked", 3: "sweep_complete"}

        def on_status(m: SweepStatus):
            self._last_status_t = time.time()
            self._status.update(state=m.state, paused=m.paused, lane_index=m.lane_index,
                                lane_depth=round(m.lane_depth, 3), direction=m.direction,
                                depth=round(m.depth, 3), heading_deg=round(math.degrees(m.heading), 1),
                                x_est=round(m.x_est, 3), end_score=round(m.end_score, 3), faults=m.faults)

        def on_event(m: SweepEvent):
            self._events.appendleft({"name": kinds.get(m.kind, str(m.kind)), "lane": m.lane,
                                     "direction": m.direction, "value": round(m.value, 2),
                                     "verified": m.verified})

        def on_mission(m: String):
            import json
            self._status["mission"] = json.loads(m.data).get("mission", "UNKNOWN")

        node.create_subscription(SweepStatus, "/glaucus/sweep_status", on_status, 10)
        node.create_subscription(SweepEvent, "/glaucus/sweep_events", on_event, 50)
        node.create_subscription(String, "/glaucus/mission_state", on_mission, 1)
        node.create_subscription(OccupancyGrid, "/glaucus/coverage", lambda g: setattr(self, "_grid", g), 1)
        self._start = node.create_client(Trigger, "/glaucus/mission/start")
        self._abort = node.create_client(Trigger, "/glaucus/mission/abort")
        self._pause = node.create_client(SetBool, "/glaucus/mission/pause")
        self._estop = node.create_client(Trigger, "/glaucus/estop")

        ex = MultiThreadedExecutor()
        ex.add_node(node)
        threading.Thread(target=ex.spin, daemon=True).start()

    def _call(self, client, req) -> tuple[bool, str]:
        if not client.wait_for_service(timeout_sec=1.0):
            return False, "The robot isn't answering. Check the tether and that the bot stack is running."
        fut = client.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 3.0:
            time.sleep(0.02)
        if not fut.done():
            return False, "The robot didn't confirm in time."
        r = fut.result()
        return bool(r.success), r.message

    def start(self):
        return self._call(self._start, self._Trigger.Request())

    def pause(self, paused: bool):
        return self._call(self._pause, self._SetBool.Request(data=paused))

    def abort(self):
        return self._call(self._abort, self._Trigger.Request())

    def estop(self):
        return self._call(self._estop, self._Trigger.Request())

    def status(self) -> dict:
        s = dict(self._status)
        s["connected"] = time.time() - self._last_status_t < 2.0
        s["events"] = list(self._events)
        return s

    def hull(self) -> dict:
        # TODO(glaucus): load the boat's hull profile from config instead of a box.
        L, D = 10.0, 1.5
        return {"length": L, "draft": D, "outline": [[0, 0], [0, D], [L, D], [L, 0]],
                "lane_spacing": 0.24, "first_lane_depth": 0.15, "max_depth": 1.15, "brush_width": 0.3}

    def coverage(self) -> dict:
        g = self._grid
        if g is None:
            return {"res": 0, "rows": []}
        w, h = g.info.width, g.info.height
        rows = ["".join("#" if g.data[i * w + j] >= 50 else "." for j in range(0, w, 3))
                for i in range(0, h, 3)]
        return {"res": g.info.resolution * 3, "rows": rows}
