"""
Glaucus I — sweep algorithm sandbox (Python).

Why this file exists
--------------------
We don't know yet exactly how the sweep should behave, so we prototype it HERE,
where iterating is cheap, then port the logic 1:1 into C++
(ros2_ws/src/glaucus_core). The class names and state names match the C++ core
on purpose. If you change the algorithm, change it here first, look at the plot,
then port.

The model (deliberately simple)
-------------------------------
We "unroll" one side of the hull into a flat sheet:
    x = distance along the hull (m), 0 = stern, increasing toward the bow
    z = depth below the waterline (m), positive DOWN
The robot lives on that sheet. Heading theta is measured in the sheet:
    theta = 0      -> facing the bow (+x)
    theta = pi     -> facing the stern (-x)
    theta = +pi/2  -> facing straight down (+z)

What's realistic about it
-------------------------
- Depth sensor: noisy but drift-free (absolute).
- Heading: from gravity (accelerometer) -> noisy but drift-free. On a near-
  vertical wall "which way is down" is visible to the IMU; no magnetometer
  needed (a steel hull wrecks magnetometers anyway).
- Wheel odometry: wheels turn at the commanded speed but the robot SLIPS, so
  odometry overestimates distance -> along-hull position x DRIFTS. When the
  robot stalls against an edge, the wheels keep spinning -> big odometry error.
- Gravity sag: the robot slowly slides down the wall -> depth hold needs an
  integral term.
- Camera end-of-lane cue: sees the hull edge when it's close, sometimes misses
  (turbidity), sometimes fires falsely (a fouling patch that looks like an edge).
- Stall cue: "I'm commanding motion but not moving" (on the real bot this comes
  from camera optical flow or IMU vibration, NOT from wheel encoders).

Run it
------
    python sim/sweep_sim.py                 # run + save docs/img/sweep_sim.png
    python sim/sweep_sim.py --no-camera     # stall-only end detection
    python sim/sweep_sim.py --seed 3 --show
"""
from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path

import numpy as np

TAU = 2.0 * math.pi


def wrap(a: float) -> float:
    """Wrap an angle to [-pi, pi). Always wrap before using an angle ERROR."""
    return (a + math.pi) % TAU - math.pi


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


# ---------------------------------------------------------------------------
# World
# ---------------------------------------------------------------------------
@dataclass
class Hull:
    """One side of the hull, unrolled. Lanes get shorter at the bow as you go
    deeper (bow cutaway), the transom (stern) is roughly vertical."""

    length: float = 9.0        # waterline length (m)
    draft: float = 1.3         # deepest point of the side we clean (m)
    stern_x: float = 0.3       # transom position: our "ruler zero" for resets
    bow_cutaway: float = 2.6   # deepest lane is this much shorter at the bow (m)

    def x_min(self, z: float) -> float:
        return self.stern_x

    def x_max(self, z: float) -> float:
        t = clamp(z / self.draft, 0.0, 1.0)
        return self.length - self.bow_cutaway * t ** 1.5

    def contains(self, x: float, z: float) -> bool:
        return 0.02 <= z <= self.draft and self.x_min(z) <= x <= self.x_max(z)


# ---------------------------------------------------------------------------
# Parameters (mirror: ros2_ws/src/glaucus_bringup/config/sweep_params.yaml)
# ---------------------------------------------------------------------------
@dataclass
class SweepParams:
    brush_width: float = 0.30          # m, cleaning footprint
    overlap: float = 0.20              # fraction of brush width overlapped per lane
    first_lane_depth: float = 0.15     # m, center of the top lane
    max_depth: float = 1.15            # m, don't plan lane centers deeper than this

    lane_speed: float = 0.15           # m/s along a lane
    step_speed: float = 0.08           # m/s while stepping down

    # Depth-hold (outer loop, runs on the Pi). Output is a heading OFFSET.
    kp_depth: float = 2.0              # rad of heading per m of depth error
    ki_depth: float = 0.5              # rad per (m*s)
    max_depth_correction: float = 0.35 # rad (~20 deg) - don't let it steer wildly
    depth_integral_limit: float = 0.3  # anti-windup clamp (m*s)

    # Heading hold (inner loop, runs on the MCU). Output is a turn rate.
    kp_heading: float = 2.5            # (rad/s) per rad
    max_turn_rate: float = 0.8         # rad/s
    heading_tol: float = 0.05          # rad, "turn is done"
    depth_tol: float = 0.02            # m, "reached the next lane depth"

    # End-of-lane evidence fusion
    w_camera: float = 0.6
    w_stall: float = 0.6
    end_threshold: float = 0.5         # score must exceed this...
    end_debounce_ticks: int = 4        # ...for this many control ticks in a row
    max_lane_length: float = 15.0      # m, hard safety guard on odometry

    # "Are you sure?" check for suspiciously SHORT lanes. A lane ending much
    # earlier than the previous one is more likely a false alarm (or a real
    # mid-hull obstacle) than the hull edge. Creep forward and look again.
    verify_margin: float = 0.6         # m; verify if lane < previous lane - margin
                                       # (lanes legitimately shrink ~0.5 m/lane at the bow)
    verify_distance: float = 0.25      # m to creep while verifying
    verify_speed: float = 0.05         # m/s

    # Blocked step-down recovery ("can't go deeper here -> slide inward, retry")
    shift_inward: float = 0.35         # m
    max_step_retries: int = 3

    @property
    def lane_spacing(self) -> float:
        return self.brush_width * (1.0 - self.overlap)


# ---------------------------------------------------------------------------
# Controller (mirror: glaucus_core/sweep_controller.hpp)
# ---------------------------------------------------------------------------
class State(str, Enum):
    STEP_DOWN = "STEP_DOWN"   # facing down, descend to the next lane depth
    TURN = "TURN"             # pivot in place to a target heading
    LANE = "LANE"             # drive along the hull holding constant depth
    VERIFY = "VERIFY"         # early end-of-lane? creep forward and re-check
    SHIFT = "SHIFT"           # step-down was blocked: slide inward along the lane
    DONE = "DONE"


@dataclass
class Sensors:
    depth: float      # m (absolute, from pressure sensor)
    heading: float    # rad (gravity-referenced, from IMU)
    odom_ds: float    # m travelled by the wheels this tick (forward, >= 0)
    cam_edge: float   # 0..1 "edge/obstacle ahead" confidence from the camera
    stall: float      # 0..1 "commanded motion but not moving"
    flow_ds: float | None = None  # m actually moved (optical flow), None = no reading


@dataclass
class Command:
    v: float           # m/s forward
    heading: float     # rad, target heading for the MCU's heading loop
    brush_on: bool = True


class EndOfLaneDetector:
    """Fuse weak cues into one decision.

    Why fusion + debounce instead of "camera says stop":
      - camera alone: false positives (fouling looks like an edge), misses in
        murky water
      - stall alone: you only find the edge by ramming it
      - requiring N ticks in a row kills one-frame glitches
    """

    def __init__(self, p: SweepParams):
        self.p = p
        self.count = 0
        self.score = 0.0

    def reset(self) -> None:
        self.count = 0
        self.score = 0.0

    def update(self, cam_edge: float, stall: float) -> bool:
        self.score = self.p.w_camera * cam_edge + self.p.w_stall * stall
        self.count = self.count + 1 if self.score > self.p.end_threshold else 0
        return self.count >= self.p.end_debounce_ticks


class SweepController:
    """Boustrophedon ("lawnmower") sweep on the hull side.

    Key idea: lanes are defined by DEPTH, which we measure absolutely. Lane ends
    are DETECTED, not computed from x. So the sweep pattern never needs to know
    the along-hull position x at all - x only matters for the coverage map.
    """

    def __init__(self, p: SweepParams):
        self.p = p
        self.detector = EndOfLaneDetector(p)
        self.state = State.STEP_DOWN
        self.lane_idx = -1                 # becomes 0 when the first lane starts
        self.lane_depth = 0.0
        self.target_depth = p.first_lane_depth
        # direction of the lane we're about to start is -self.direction;
        # start at -1 so the first lane goes toward the bow (+1).
        self.direction = -1
        self.turn_target = 0.0
        self.after_turn = State.LANE
        self.depth_integral = 0.0
        self.seg_dist = 0.0                # odometry since entering the state
        self.lane_dist = 0.0               # odometry since this lane started
        self.prev_lane_length: float | None = None
        self.retries = 0
        self.events: list[tuple[str, dict]] = []   # for logging / the app

    # -- helpers -----------------------------------------------------------
    @staticmethod
    def lane_heading(direction: int) -> float:
        return 0.0 if direction > 0 else math.pi

    def _enter(self, state: State) -> None:
        self.state = state
        self.seg_dist = 0.0
        self.detector.reset()

    def _turn_then(self, target: float, after: State) -> None:
        self.turn_target = target
        self.after_turn = after
        self._enter(State.TURN)

    def _stop(self, heading: float) -> Command:
        return Command(0.0, heading, self.state != State.DONE)

    def _depth_hold_heading(self, depth: float, dt: float) -> float:
        """Outer loop: depth error -> heading offset ("too deep? aim slightly
        uphill"). The sign flips with lane direction because 'uphill' is a
        left turn one way and a right turn the other way."""
        p = self.p
        e = depth - self.lane_depth
        self.depth_integral = clamp(self.depth_integral + e * dt,
                                    -p.depth_integral_limit, p.depth_integral_limit)
        corr = clamp(p.kp_depth * e + p.ki_depth * self.depth_integral,
                     -p.max_depth_correction, p.max_depth_correction)
        return self.lane_heading(self.direction) - self.direction * corr

    def _end_lane(self, hdg: float, verified: bool) -> Command:
        p = self.p
        self.events.append(("end_of_lane", {"lane": self.lane_idx,
                                            "direction": self.direction,
                                            "length_odom": round(self.lane_dist, 3),
                                            "verified": verified}))
        self.prev_lane_length = self.lane_dist
        nxt = self.lane_depth + p.lane_spacing
        if nxt > p.max_depth:
            self.events.append(("sweep_complete", {"lanes": self.lane_idx + 1}))
            self._enter(State.DONE)
        else:
            self.target_depth = nxt
            self.retries = 0
            self._turn_then(math.pi / 2, State.STEP_DOWN)
        return self._stop(hdg)

    def abort(self) -> None:
        self._enter(State.DONE)

    # -- main update, call at a fixed rate (20 Hz on the Pi) --------------
    def update(self, s: Sensors, dt: float) -> Command:
        p = self.p
        # Best available "how far did I actually move": optical flow if we have
        # a reading this tick, else wheel odometry.
        ds = s.flow_ds if s.flow_ds is not None else s.odom_ds

        if self.state == State.DONE:
            return Command(0.0, s.heading, False)

        if self.state == State.TURN:
            if abs(wrap(self.turn_target - s.heading)) < p.heading_tol:
                self._enter(self.after_turn)
            return self._stop(self.turn_target)

        if self.state in (State.LANE, State.VERIFY):
            self.seg_dist += ds
            self.lane_dist += ds
            hdg = self._depth_hold_heading(s.depth, dt)
            fired = self.detector.update(s.cam_edge, s.stall)

            if self.state == State.VERIFY:
                if fired:                                   # confirmed: real end
                    return self._end_lane(hdg, verified=True)
                if self.seg_dist >= p.verify_distance:      # false alarm: carry on
                    self.events.append(("end_rejected", {"lane": self.lane_idx,
                                                         "at": round(self.lane_dist, 2)}))
                    self._enter(State.LANE)
                return Command(p.verify_speed, hdg, True)

            if fired or self.lane_dist > p.max_lane_length:
                suspicious = (self.prev_lane_length is not None and
                              self.lane_dist < self.prev_lane_length - p.verify_margin)
                if fired and suspicious:
                    self._enter(State.VERIFY)
                    return Command(p.verify_speed, hdg, True)
                return self._end_lane(hdg, verified=False)
            return Command(p.lane_speed, hdg, True)

        if self.state == State.STEP_DOWN:
            self.seg_dist += ds
            if s.depth >= self.target_depth - p.depth_tol:
                self.lane_depth = self.target_depth
                self.lane_idx += 1
                self.direction = -self.direction
                self.lane_dist = 0.0
                self._turn_then(self.lane_heading(self.direction), State.LANE)
                return self._stop(math.pi / 2)
            blocked = self.detector.update(s.cam_edge, s.stall)
            too_far = self.seg_dist > 2.0 * p.lane_spacing + 0.3
            if blocked or too_far:
                if self.retries < p.max_step_retries:
                    # Hull curves away below us (bow cutaway / keel): slide back
                    # toward the middle of the lane and try again.
                    self.retries += 1
                    self.events.append(("step_down_blocked", {"retry": self.retries}))
                    self._turn_then(self.lane_heading(-self.direction), State.SHIFT)
                else:
                    self.events.append(("sweep_complete", {"lanes": self.lane_idx + 1,
                                                           "reason": "cannot_descend"}))
                    self._enter(State.DONE)
                return self._stop(math.pi / 2)
            return Command(p.step_speed, math.pi / 2, True)

        if self.state == State.SHIFT:
            self.seg_dist += ds
            if self.seg_dist >= p.shift_inward:
                self._turn_then(math.pi / 2, State.STEP_DOWN)
                return self._stop(self.turn_target)
            return Command(p.lane_speed, self.lane_heading(-self.direction), True)

        raise RuntimeError(self.state)


# ---------------------------------------------------------------------------
# Inner heading loop (mirror: firmware/src/heading_controller.h)
# ---------------------------------------------------------------------------
def mcu_heading_loop(p: SweepParams, heading_cmd: float, heading_meas: float) -> float:
    """P controller on heading -> turn rate. On the real bot this runs at
    ~200 Hz on the MCU and turns into left/right wheel speeds:
        v_left  = v - omega * track/2
        v_right = v + omega * track/2
    """
    return clamp(p.kp_heading * wrap(heading_cmd - heading_meas), -p.max_turn_rate, p.max_turn_rate)


# ---------------------------------------------------------------------------
# Estimator (mirror: glaucus_core/wall_estimator.hpp)
# ---------------------------------------------------------------------------
class WallEstimator:
    """Where am I on the unrolled hull?

        z     <- depth sensor            (absolute -> trust it)
        theta <- gravity from the IMU    (absolute on a wall -> trust it)
        x     <- odometry                (DRIFTS -> needs help)

    Two knobs, both compared in the sim plot:
      use_flow  - integrate an optical-flow sensor ("optical mouse" looking at
                  the hull) instead of wheel encoders. Flow measures what the
                  robot ACTUALLY moved, so wheel slip and wheel spin-at-the-edge
                  don't fool it. It drops out on featureless patches, where we
                  fall back to wheel odometry for that tick.
      reset_x   - when we hit the stern edge (transom ~vertical, so its x is
                  known), snap x to that value. Bounds the random-walk part of
                  the drift so it can't grow over a long mission.

    Honest sim finding: with WHEEL odometry, stern resets barely help, because
    a steady slip over-counts going toward the bow and over-counts coming back,
    so it mostly cancels on each round trip anyway. The error you're left with
    (~0.2-0.4 m) is "which part of the lane" error, not "which lane" error,
    because depth pins the lane. Flow + resets is the upgrade if the coverage
    map needs to be tighter.

    We also tried calibrating wheel slip during STEP_DOWN (depth sensor gives
    the true distance travelled). It did NOT work: gravity helps you going down,
    so down-slip != sideways-slip. Don't repeat that experiment without
    measuring both on the real robot.
    """

    def __init__(self, x0: float, reset_x: float | None, use_flow: bool, reset_gate: float = 0.5):
        self.x = x0
        self.z = 0.0
        self.theta = 0.0
        self.reset_x = reset_x   # None -> no stern resets
        self.use_flow = use_flow
        # Only accept a reset if we already think we're near the stern. A false
        # end-of-lane in mid-hull would otherwise teleport x by meters.
        self.reset_gate = reset_gate

    def update(self, s: Sensors) -> None:
        self.theta = s.heading
        self.z = s.depth
        ds = s.flow_ds if (self.use_flow and s.flow_ds is not None) else s.odom_ds
        self.x += ds * math.cos(self.theta)

    def on_end_of_lane(self, direction: int) -> None:
        if (self.reset_x is not None and direction < 0
                and abs(self.x - self.reset_x) < self.reset_gate):
            self.x = self.reset_x


# ---------------------------------------------------------------------------
# Plant + sensor models
# ---------------------------------------------------------------------------
class Plant:
    def __init__(self, hull: Hull, rng: np.random.Generator, x0: float, z0: float):
        self.hull, self.rng = hull, rng
        self.x, self.z, self.theta = x0, z0, 0.0
        self.sag = 0.004            # m/s, gravity slide down the wall
        self.last_ratio = 1.0       # actual / commanded motion last tick
        self.last_moved = 0.0

    def step(self, v_cmd: float, omega: float, dt: float) -> float:
        slip = clamp(self.rng.normal(0.06, 0.03), 0.0, 0.3)
        v = v_cmd * (1.0 - slip)
        self.theta = wrap(self.theta + omega * dt + self.rng.normal(0, 0.002))
        nx = self.x + v * math.cos(self.theta) * dt
        nz = self.z + v * math.sin(self.theta) * dt + self.sag * dt
        moved = 0.0
        if self.hull.contains(nx, nz):
            moved = math.hypot(nx - self.x, nz - self.z)
            self.x, self.z = nx, nz
        self.last_moved = moved
        self.last_ratio = moved / (v_cmd * dt) if v_cmd > 1e-3 else 1.0
        return v_cmd * dt   # wheel odometry: wheels turn at the commanded speed

    def edge_distance_ahead(self, max_d: float = 0.6) -> float:
        c, s = math.cos(self.theta), math.sin(self.theta)
        d = 0.0
        while d < max_d:
            d += 0.01
            if not self.hull.contains(self.x + c * d, self.z + s * d):
                return d
        return max_d

    def sense(self, odom_ds: float, moving_cmd: bool, camera: bool) -> Sensors:
        r = self.rng
        cam = 0.0
        if camera:
            cam = clamp(1.0 - self.edge_distance_ahead() / 0.35, 0.0, 1.0)
            if r.random() < 0.10:
                cam *= 0.3                              # missed (turbid water)
            if r.random() < 0.03:
                cam = max(cam, r.uniform(0.7, 1.0))     # false positive spike
        stall = 1.0 if (moving_cmd and self.last_ratio < 0.3) else 0.0
        if r.random() < 0.02:
            stall = 1.0 - stall                         # sensor glitch
        # optical flow: true motion with 3% noise, drops out 10% of ticks
        flow = None if r.random() < 0.10 else self.last_moved * (1.0 + r.normal(0, 0.03))
        return Sensors(depth=self.z + r.normal(0, 0.005),
                       heading=wrap(self.theta + r.normal(0, 0.01)),
                       odom_ds=odom_ds, cam_edge=cam, stall=stall, flow_ds=flow)


# ---------------------------------------------------------------------------
# Coverage grid (mirror: glaucus_topside/coverage_mapper.py)
# ---------------------------------------------------------------------------
class CoverageGrid:
    def __init__(self, hull: Hull, brush_width: float, res: float = 0.02):
        self.res = res
        self.nx = int(math.ceil((hull.length + 0.2) / res))
        self.nz = int(math.ceil((hull.draft + 0.1) / res))
        xs = (np.arange(self.nx) + 0.5) * res
        zs = (np.arange(self.nz) + 0.5) * res
        self.cleanable = np.array([[hull.contains(x, z) or (0 <= z < 0.02 and hull.x_min(z) <= x <= hull.x_max(z))
                                    for x in xs] for z in zs])
        self.cleaned = np.zeros_like(self.cleanable)
        r = brush_width / 2.0
        k = int(math.ceil(r / res))
        self._offsets = [(i, j) for i in range(-k, k + 1) for j in range(-k, k + 1)
                         if (i * res) ** 2 + (j * res) ** 2 <= r * r]

    def mark(self, x: float, z: float) -> None:
        ci, cj = int(z / self.res), int(x / self.res)
        for di, dj in self._offsets:
            i, j = ci + di, cj + dj
            if 0 <= i < self.nz and 0 <= j < self.nx:
                self.cleaned[i, j] = True

    def coverage(self) -> float:
        n = self.cleanable.sum()
        return float((self.cleaned & self.cleanable).sum() / n) if n else 0.0


# ---------------------------------------------------------------------------
# Simulation wrapper (also used by the web app's --mock mode)
# ---------------------------------------------------------------------------
@dataclass
class Simulation:
    seed: int = 1
    camera: bool = True
    dt: float = 0.05                    # 20 Hz control loop
    hull: Hull = field(default_factory=Hull)
    params: SweepParams = field(default_factory=SweepParams)

    def __post_init__(self):
        self.rng = np.random.default_rng(self.seed)
        # Deploy at the STERN corner (transom, just below the waterline): the
        # first lane then runs the full length, and x starts at a known value.
        x0 = self.hull.stern_x + 0.25
        self.plant = Plant(self.hull, self.rng, x0=x0, z0=0.04)
        self.plant.theta = math.pi / 2
        self.ctrl = SweepController(self.params)
        stern_ref = self.hull.stern_x + 0.04   # we stop ~4 cm short of the edge
        # Three estimators run side by side on the SAME data so we can compare:
        self.est_dr = WallEstimator(x0, reset_x=None, use_flow=False)       # wheel odometry only
        self.est_rs = WallEstimator(x0, reset_x=stern_ref, use_flow=False)  # wheels + stern resets
        self.est = WallEstimator(x0, reset_x=stern_ref, use_flow=True)      # flow + stern resets
        self.estimators = (self.est_dr, self.est_rs, self.est)
        self.grid = CoverageGrid(self.hull, self.params.brush_width)
        self.t = 0.0
        self.cmd = Command(0.0, math.pi / 2)
        self.odom = 0.0
        self.paused = False
        self.log: dict[str, list] = {k: [] for k in
                                     ("t", "x", "z", "x_dr", "x_rs", "x_full", "state", "score")}
        self._n_events = 0
        self.last_sensors: Sensors | None = None

    @property
    def done(self) -> bool:
        return self.ctrl.state == State.DONE

    def step(self) -> None:
        if self.done or self.paused:
            return
        p = self.params
        omega = mcu_heading_loop(p, self.cmd.heading, self.plant.theta)
        odom = self.plant.step(self.cmd.v, omega, self.dt)
        s = self.plant.sense(odom, self.cmd.v > 1e-3, self.camera)
        self.last_sensors = s
        for e in self.estimators:
            e.update(s)
        self.cmd = self.ctrl.update(s, self.dt)
        for name, info in self.ctrl.events[self._n_events:]:
            if name == "end_of_lane":
                for e in self.estimators:
                    e.on_end_of_lane(info["direction"])
        self._n_events = len(self.ctrl.events)
        if self.cmd.brush_on:
            self.grid.mark(self.plant.x, self.plant.z)
        self.t += self.dt
        L = self.log
        L["t"].append(self.t); L["x"].append(self.plant.x); L["z"].append(self.plant.z)
        L["x_dr"].append(self.est_dr.x); L["x_rs"].append(self.est_rs.x); L["x_full"].append(self.est.x)
        L["state"].append(self.ctrl.state.value); L["score"].append(self.ctrl.detector.score)

    def run(self, max_time: float = 3600.0) -> None:
        while not self.done and self.t < max_time:
            self.step()

    def status(self) -> dict:
        """Snapshot shaped like the SweepStatus ROS message (see docs/interfaces.md)."""
        s = self.last_sensors
        return {
            "t": round(self.t, 2),
            "state": self.ctrl.state.value,
            "paused": self.paused,
            "lane_index": self.ctrl.lane_idx,
            "lane_depth": round(self.ctrl.lane_depth, 3),
            "direction": self.ctrl.direction,
            "depth": round(s.depth, 3) if s else 0.0,
            "heading_deg": round(math.degrees(s.heading), 1) if s else 0.0,
            "x_est": round(self.est.x, 3),
            "end_score": round(self.ctrl.detector.score, 3),
            "coverage": round(self.grid.coverage(), 4),
            "events": [{"name": n, **i} for n, i in self.ctrl.events[-8:]],
        }


# ---------------------------------------------------------------------------
# Plot
# ---------------------------------------------------------------------------
def plot(sim: Simulation, out: Path, show: bool) -> None:
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap

    # validated categorical slots 1-4 (blue, orange, aqua, yellow); dash styles
    # are the secondary encoding so color is never the only cue
    TRUTH, DR, FULL, RS = "#2a78d6", "#eb6834", "#1baf7a", "#eda100"
    INK, MUTED, GRID = "#21201c", "#6b6a63", "#e4e3dd"

    L, h = sim.log, sim.hull
    from matplotlib.patches import Patch
    fig, (ax, ax2) = plt.subplots(2, 1, figsize=(11, 8), height_ratios=[1.6, 1],
                                  constrained_layout=True)
    fig.patch.set_facecolor("white")

    img = np.where(sim.grid.cleanable, np.where(sim.grid.cleaned, 2, 1), 0)
    ax.imshow(img, origin="upper", extent=[0, sim.grid.nx * sim.grid.res, sim.grid.nz * sim.grid.res, 0],
              cmap=ListedColormap(["white", "#f6d3c4", "#cde2fb"]), vmin=0, vmax=2,
              interpolation="nearest", aspect="auto")
    zs = np.linspace(0, h.draft, 60)
    ax.plot([h.x_max(z) for z in zs], zs, color=MUTED, lw=1)
    ax.plot([h.x_min(z) for z in zs], zs, color=MUTED, lw=1)
    ax.plot([h.x_min(h.draft), h.x_max(h.draft)], [h.draft] * 2, color=MUTED, lw=1)
    ax.plot(L["x"], L["z"], color=TRUTH, lw=2, label="True path")
    ax.plot(L["x_dr"], L["z"], color=DR, lw=1.5, ls=(0, (2, 2)), label="Estimate: wheel odometry only")
    ax.plot(L["x_full"], L["z"], color=FULL, lw=1.5, ls=(0, (6, 3)),
            label="Estimate: optical flow + stern resets")
    ax.set_xlim(-0.2, h.length + 0.4)
    ax.set_ylim(h.draft + 0.12, -0.05)
    ax.set_xlabel("Along hull, x (m)  →  bow", color=INK)
    ax.set_ylabel("Depth, z (m)", color=INK)
    ax.set_title(f"Sweep on unrolled hull side  ·  coverage {sim.grid.coverage():.1%}  ·  "
                 f"{sim.ctrl.lane_idx + 1} lanes  ·  {sim.t / 60:.1f} min",
                 loc="left", color=INK, fontsize=12)
    handles, _ = ax.get_legend_handles_labels()
    handles += [Patch(facecolor="#cde2fb", label="Cleaned"), Patch(facecolor="#f6d3c4", label="Missed")]
    ax.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, -0.13), ncol=5,
              frameon=False, fontsize=9)

    t = np.array(L["t"]) / 60
    ax2.plot(t, np.abs(np.array(L["x_dr"]) - L["x"]), color=DR, lw=2, ls=(0, (4, 2)), label="Wheel odometry only")
    ax2.plot(t, np.abs(np.array(L["x_rs"]) - L["x"]), color=RS, lw=2, ls=(0, (1, 1.5)),
             label="Wheel odometry + stern resets")
    ax2.plot(t, np.abs(np.array(L["x_full"]) - L["x"]), color=FULL, lw=2,
             label="Optical flow + stern resets")
    ax2.set_xlabel("Time (min)", color=INK)
    ax2.set_ylabel("|x error| (m)", color=INK)
    ax2.set_title("Where the coverage map thinks the robot is vs. the truth", loc="left",
                  color=INK, fontsize=11)
    ax2.set_ylim(0, max(0.3, float(np.max(np.abs(np.array(L["x_dr"]) - L["x"])))) * 1.45)
    ax2.legend(loc="upper center", ncol=3, frameon=False, fontsize=9)

    for a in (ax, ax2):
        for sp in ("top", "right"):
            a.spines[sp].set_visible(False)
        for sp in ("left", "bottom"):
            a.spines[sp].set_color(GRID)
        a.tick_params(colors=MUTED)
    ax2.grid(axis="y", color=GRID, lw=0.8)

    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=140)
    if show:
        plt.show()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seed", type=int, default=2)  # seed 2 includes a rejected false alarm
    ap.add_argument("--no-camera", action="store_true", help="end detection from stall only")
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--quiet", action="store_true", help="don't print the event log")
    ap.add_argument("--dump-trace", metavar="CSV",
                    help="write per-tick sensors + controller outputs; the C++ parity test replays it")
    ap.add_argument("--out", default=str(Path(__file__).resolve().parents[1] / "docs/img/sweep_sim.png"))
    a = ap.parse_args()

    sim = Simulation(seed=a.seed, camera=not a.no_camera)
    trace = [] if a.dump_trace else None
    if trace is not None:
        _step = sim.step
        def traced_step():
            _step()
            s, c = sim.last_sensors, sim.cmd
            if s is not None:
                trace.append((s.depth, s.heading, s.odom_ds, s.cam_edge, s.stall,
                              float("nan") if s.flow_ds is None else s.flow_ds,
                              c.v, c.heading, int(c.brush_on), sim.ctrl.state.value))
        sim.step = traced_step
    sim.run()
    if trace is not None:
        with open(a.dump_trace, "w") as f:
            f.write("depth,heading,odom_ds,cam_edge,stall,flow_ds,v,heading_cmd,brush_on,state\n")
            for row in trace:
                f.write(",".join(repr(v) if isinstance(v, float) else str(v) for v in row) + "\n")
        print(f"trace ({len(trace)} ticks) -> {a.dump_trace}")
    xe_dr = np.abs(np.array(sim.log["x_dr"]) - sim.log["x"])
    xe_rs = np.abs(np.array(sim.log["x_rs"]) - sim.log["x"])
    xe_full = np.abs(np.array(sim.log["x_full"]) - sim.log["x"])
    print(f"finished: {sim.ctrl.state.value}  t={sim.t / 60:.1f} min  lanes={sim.ctrl.lane_idx + 1}  "
          f"coverage={sim.grid.coverage():.1%}")
    print(f"x error  wheels only:          mean {xe_dr.mean():.2f} m, max {xe_dr.max():.2f} m")
    print(f"x error  wheels + stern reset: mean {xe_rs.mean():.2f} m, max {xe_rs.max():.2f} m")
    print(f"x error  flow + stern reset:   mean {xe_full.mean():.2f} m, max {xe_full.max():.2f} m")
    if not a.quiet:
        for name, info in sim.ctrl.events:
            print(f"  {name:18s} {info}")
    plot(sim, Path(a.out), a.show)
    print(f"plot -> {a.out}")


if __name__ == "__main__":
    main()
