"""
Glaucus I — 3D hull simulation environment.

Why this exists
---------------
sweep_sim.py flattens the hull into a sheet. That's the right abstraction for
developing the algorithm, but it quietly assumes four things that are only true
on a perfectly vertical wall:

  1. distance driven == change in depth, when facing down
  2. gravity always points along the wall (so heading-from-gravity is as good
     at the bilge as at the waterline)
  3. depth-hold has the same authority everywhere (heading offset -> depth rate)
  4. coverage = area of the flat sheet

This file puts the SAME controller, detector, estimators and parameters (all
imported from sweep_sim.py, nothing copied) on a curved 3D hull so we can see
which of those break. It does not change the algorithm. If it shows the
algorithm needs to change, that goes through the normal route:
sweep_sim.py first, then the C++ port + parity test (see CLAUDE.md rule 2).

The model
---------
Hull skin (one side; the other side is a mirror, shown for context only):

    y = Y(x, z) = B(x) * (1 - (z / D(x))^n)^(1/n)        y = outboard, z = depth (+down)

    B(x)  half-beam: full amidships, closing to a wedge at the stem (~24 deg
          half-angle, like a real bow; an elliptical bow has an infinite-slope
          tip that no hull has), slightly narrower at the transom
    D(x)  keel depth: = sweep_sim.Hull draft, rising toward the bow so that
          the lane ends match the 2D sim's `bow_cutaway` EXACTLY
    n     section "boxiness". n=2 is an ellipse; larger = flatter sides and a
          tighter bilge. Default 4.

The robot's state is (x, z) on that skin. Its 3D position is (x, Y(x,z), z).
Heading theta is measured the way the real IMU measures it, from gravity:

    e_h  the LEVEL direction in the tangent plane  (toward the bow)   theta = 0
    e_d  the DOWNHILL direction in the tangent plane                  theta = +pi/2

On a vertical wall that is exactly the sheet's heading. On an inclined patch,
"heading 0" is still "constant depth", which is what makes depth-defined lanes
work at all. What changes is how much gravity there is IN the tangent plane
(sin(alpha), alpha = incline from horizontal): the IMU gets noisier, the
firmware's complementary filter trusts gravity less, depth-hold loses
authority, and a step of dz in depth costs dz / sin(alpha) of drivinclg.

Run it
------
    python3 sim/sim3d.py                          # run, print summary, write PNG
    python3 sim/sim3d.py --html build/sim3d.html  # + interactive 3D (needs plotly)
    python3 sim/sim3d.py --hull plane             # control case: vertical plane = the 2D sim's world
    python3 sim/sim3d.py --seed 7 --section-exp 3
"""
from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import NamedTuple

import numpy as np

import sweep_sim as s2
from sweep_sim import clamp, wrap

Vec = tuple[float, float, float]
GRAVITY_DOWN: Vec = (0.0, 0.0, 1.0)


def _dot(a: Vec, b: Vec) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a: Vec, b: Vec) -> Vec:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _unit(a: Vec) -> Vec:
    m = math.sqrt(_dot(a, a))
    return (a[0] / m, a[1] / m, a[2] / m)


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------
class Frame(NamedTuple):
    """Local frame on the skin at (x, z)."""
    p: Vec          # 3D position
    n: Vec          # outward surface normal
    e_h: Vec        # level direction in the tangent plane, toward the bow (theta = 0)
    e_d: Vec        # downhill direction in the tangent plane               (theta = +pi/2)
    sin_a: float    # in-plane share of gravity = sin(incline from horizontal); 1 = vertical wall


@dataclass
class Hull3D:
    plan: s2.Hull = field(default_factory=s2.Hull)   # length/draft/stern/bow-cutaway shared with the 2D sim
    half_beam: float = 1.5      # m, widest half-beam
    section_exp: float = 4.0    # superellipse exponent of the cross-section
    taper_x: float = 4.0        # m, station where the hull starts closing toward the bow
    stern_taper: float = 0.25   # transom half-beam is this much narrower than amidships
    z_min: float = 0.02         # m, robot can't go above the waterline
    edge_margin: float = 0.04   # m, the keel edge is y = edge_margin. Below ~3 cm the (x, z) chart can't
                                # resolve the skin (y ~ (D-z)^(1/n)); same spirit as stopping 4 cm short of the transom.
    plane: bool = False         # control case: a vertical plane with the same planform, i.e. exactly
                                # the 2D sim's world. Must reproduce sweep_sim's results.

    # -- planform ----------------------------------------------------------
    def keel_depth(self, x: float) -> float:
        """D(x): depth of the keel line. Inverse of Hull.x_max(z), so the 3D
        lane ends coincide with the 2D sim's."""
        h = self.plan
        if x <= h.length - h.bow_cutaway:
            return h.draft
        return h.draft * (max(h.length - x, 0.0) / h.bow_cutaway) ** (2.0 / 3.0)

    def beam(self, x: float) -> float:
        h = self.plan
        if x >= self.taper_x:
            t = min((x - self.taper_x) / (h.length - self.taper_x), 1.0)
            s = 1.0 - t ** 1.5
        else:
            t = (self.taper_x - x) / (self.taper_x - h.stern_x)
            s = 1.0 - self.stern_taper * t * t
        return self.half_beam * s

    def contains(self, x: float, z: float) -> bool:
        return (self.plan.stern_x <= x <= self.plan.length
                and self.z_min <= z <= self.keel_depth(x))

    # -- skin --------------------------------------------------------------
    def y(self, x: float, z: float) -> float:
        if self.plane:
            return self.half_beam
        d = self.keel_depth(x)
        if d <= 0.0:
            return 0.0
        u = clamp(z / d, 0.0, 1.0)
        return self.beam(x) * (1.0 - u ** self.section_exp) ** (1.0 / self.section_exp)

    def point(self, x: float, z: float) -> Vec:
        return (x, self.y(x, z), z)

    def _partials(self, x: float, z: float) -> tuple[float, float, float]:
        """(Y, dY/dx, dY/dz). dY/dz -> -inf at the keel line, where the skin turns
        horizontal and depth stops being a usable coordinate."""
        n_exp = self.section_exp
        d = max(self.keel_depth(x), 1e-6)
        u = clamp(z / d, 0.0, 1.0)
        w = max(1.0 - u ** n_exp, 1e-6)
        y_z = -(self.beam(x) / d) * u ** (n_exp - 1.0) * w ** (1.0 / n_exp - 1.0)
        h = 1e-4
        y_x = (self.y(x + h, z) - self.y(x - h, z)) / (2.0 * h)
        return self.y(x, z), y_x, y_z

    def frame(self, x: float, z: float) -> Frame:
        if self.plane:
            return Frame((x, self.half_beam, z), (0.0, 1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0), 1.0)
        y, y_x, y_z = self._partials(x, z)
        n = _unit((-y_x, 1.0, -y_z))
        g_n = _dot(GRAVITY_DOWN, n)
        down = (GRAVITY_DOWN[0] - g_n * n[0], GRAVITY_DOWN[1] - g_n * n[1], GRAVITY_DOWN[2] - g_n * n[2])
        sin_a = math.sqrt(max(1.0 - g_n * g_n, 0.0))
        # On a flat bottom "downhill" is undefined; pick "toward the centerline"
        # so the frame stays continuous (the IMU stops trusting it there anyway).
        e_d = _unit(down) if sin_a > 1e-6 else _unit((0.0, -n[1], 0.0))
        e_h = _cross(n, e_d)
        return Frame((x, y, z), n, e_h, e_d, sin_a)

    def advance(self, x: float, z: float, p: Vec, disp: Vec) -> tuple[float, float] | None:
        """Move from skin point (x, z) (3D position p) by a 3D displacement, and
        return the new (x, z), or None if that runs off the hull.

        The displacement is tangent, so the target q = p + disp is just off the
        skin; we project it back by Gauss-Newton on (x, z). Stepping x and z by
        the displacement's own x/z parts is NOT enough: at the keel line the skin
        is horizontal, a sideways step is almost pure y, and (x, z) barely change.
        The keel centerline (y = edge_margin) is the edge of the port side we simulate.
        """
        q = (p[0] + disp[0], p[1] + disp[1], p[2] + disp[2])
        if self.plane:
            return (q[0], q[2]) if self.contains(q[0], q[2]) else None
        if q[1] < self.edge_margin:
            return None
        xn, zn = x + disp[0], z + disp[2]
        for _ in range(3):
            y, y_x, y_z = self._partials(xn, zn)
            r0, r1, r2 = q[0] - xn, q[1] - y, q[2] - zn
            b1, b2 = r0 + y_x * r1, y_z * r1 + r2            # J^T r, J = [(1,y_x,0), (0,y_z,1)]
            det = 1.0 + y_x * y_x + y_z * y_z                # det(J^T J), without cancellation
            xn += ((1.0 + y_z * y_z) * b1 - y_x * y_z * b2) / det
            zn += ((1.0 + y_x * y_x) * b2 - y_x * y_z * b1) / det
        return (xn, zn) if self.contains(xn, zn) else None

    def arc_down(self, x: float, z0: float, z1: float, steps: int = 200) -> float:
        """Surface distance between two depths at station x (down the section)."""
        zs = np.linspace(z0, z1, steps + 1)
        pts = np.array([self.point(x, float(z)) for z in zs])
        return float(np.linalg.norm(np.diff(pts, axis=0), axis=1).sum())

    # -- meshes ------------------------------------------------------------
    def mesh(self, nx: int, nu: int) -> "SkinMesh":
        return SkinMesh(self, nx, nu)


class SkinMesh:
    """Structured grid over the skin: nx cells along the hull, nu cells from the
    waterline (u=0) to the keel line (u=1) with u = z / D(x). Every cell has a
    3D center and a true surface area, so coverage can be area-weighted."""

    def __init__(self, hull: Hull3D, nx: int, nu: int):
        self.hull, self.nx, self.nu = hull, nx, nu
        h = hull.plan
        xs = np.linspace(h.stern_x, h.length, nx + 1)
        us = np.linspace(0.0, 1.0, nu + 1)
        # corner vertices, shape (nu+1, nx+1, 3)
        self.v = np.empty((nu + 1, nx + 1, 3))
        for j, u in enumerate(us):
            for i, x in enumerate(xs):
                z = max(u * hull.keel_depth(x), 0.0)
                self.v[j, i] = hull.point(float(x), float(z))
        xc, uc = 0.5 * (xs[:-1] + xs[1:]), 0.5 * (us[:-1] + us[1:])
        self.c = np.empty((nu, nx, 3))
        for j, u in enumerate(uc):
            for i, x in enumerate(xc):
                self.c[j, i] = hull.point(float(x), float(u * hull.keel_depth(float(x))))
        a, b, c, d = self.v[:-1, :-1], self.v[:-1, 1:], self.v[1:, 1:], self.v[1:, :-1]
        self.area = 0.5 * (np.linalg.norm(np.cross(b - a, c - a), axis=2)
                           + np.linalg.norm(np.cross(c - a, d - a), axis=2))
        self.xc = self.c[0, :, 0]
        self.dx = float(xs[1] - xs[0])


class Coverage3D:
    """Brush footprint = a disc of radius brush_width/2 on the skin, tested in 3D
    (chord ~ arc for a 15 cm radius on a ~1 m-radius bilge)."""

    def __init__(self, mesh: SkinMesh, brush_width: float):
        self.mesh, self.r = mesh, brush_width / 2.0
        self.cleaned = np.zeros((mesh.nu, mesh.nx), dtype=bool)
        self._reach = int(math.ceil(self.r / mesh.dx)) + 1

    def mark(self, p: Vec) -> None:
        m = self.mesh
        i0 = int((p[0] - m.hull.plan.stern_x) / m.dx)
        lo, hi = max(i0 - self._reach, 0), min(i0 + self._reach + 1, m.nx)
        if lo >= hi:
            return
        d = m.c[:, lo:hi, :] - np.asarray(p)
        self.cleaned[:, lo:hi] |= (d * d).sum(axis=2) <= self.r * self.r

    def coverage(self, z_max: float | None = None) -> float:
        """Area-weighted fraction cleaned. z_max limits it to skin shallower than
        that depth, i.e. 'the part of the hull the sweep is meant to reach'."""
        m = self.mesh
        sel = np.ones_like(self.cleaned) if z_max is None else (m.c[:, :, 2] <= z_max)
        return float(m.area[sel & self.cleaned].sum() / m.area[sel].sum())

    def by_depth_band(self, edges: list[float]) -> list[tuple[float, float, float]]:
        """(z_lo, z_hi, area-weighted coverage) for each depth band."""
        m, out = self.mesh, []
        z = m.c[:, :, 2]
        for lo, hi in zip(edges[:-1], edges[1:]):
            sel = (z >= lo) & (z < hi)
            a = m.area[sel].sum()
            out.append((lo, hi, float(m.area[sel & self.cleaned].sum() / a) if a > 0 else float("nan")))
        return out


# ---------------------------------------------------------------------------
# Sensors / plant
# ---------------------------------------------------------------------------
class IMU:
    """Gyro + gravity complementary filter, same structure and constants as
    firmware/src/heading_estimator.h (alpha = 0.02 per 200 Hz step, scaled by
    the in-plane share of gravity so it falls back to gyro on flat patches).

    The 3D-specific parts:
      * the accelerometer's heading noise is sigma_acc / sin(alpha): less
        in-plane gravity means a worse angle estimate;
      * the gyro measures the body's rotation, which on a curved surface is
        d(theta)/dt PLUS the twist of the gravity frame along the path. The
        filter has no model for that twist, so on a flat patch (where gravity
        can't correct it) it shows up as heading drift.
    """
    SIGMA_ACC = 0.05      # rad, vibration-dominated accel heading noise on a vertical wall
    SIGMA_GYRO = 0.01     # rad/s white noise
    BIAS_SIGMA = 0.003    # rad/s constant bias, drawn once

    def __init__(self, rng: np.random.Generator, theta0: float, alpha: float = 0.02, substeps: int = 10):
        self.rng = rng
        self.alpha_eff = 1.0 - (1.0 - alpha) ** substeps   # 200 Hz firmware filter -> one 20 Hz tick
        self.bias = float(rng.normal(0.0, self.BIAS_SIGMA))
        self.theta = theta0

    def update(self, theta_true: float, body_rate: float, sin_a: float, dt: float) -> float:
        r = self.rng
        self.theta = wrap(self.theta + (body_rate + self.bias + r.normal(0.0, self.SIGMA_GYRO)) * dt)
        theta_acc = theta_true + r.normal(0.0, self.SIGMA_ACC / max(sin_a, 0.02))
        self.theta = wrap(self.theta + self.alpha_eff * min(sin_a, 1.0) * wrap(theta_acc - self.theta))
        return self.theta


class Plant3D:
    """Same slip / sag / sensor-failure statistics as sweep_sim.Plant, but the
    robot lives on Hull3D and its sensors read the 3D geometry."""

    def __init__(self, hull: Hull3D, rng: np.random.Generator, x0: float, z0: float):
        self.hull, self.rng = hull, rng
        self.x, self.z = x0, z0
        self.theta = math.pi / 2
        self.fr = hull.frame(x0, z0)
        self.imu = IMU(rng, self.theta)
        self.sag = 0.004            # m/s on a vertical wall; scales with the in-plane share of gravity
        self.last_ratio = 1.0
        self.last_moved = 0.0
        self._body_rate = 0.0

    @property
    def pos(self) -> Vec:
        return self.fr.p

    def step(self, v_cmd: float, omega: float, dt: float) -> float:
        r, hull, fr = self.rng, self.hull, self.fr
        slip = clamp(r.normal(0.06, 0.03), 0.0, 0.3)
        v = v_cmd * (1.0 - slip)
        th0 = self.theta
        self.theta = wrap(self.theta + omega * dt + r.normal(0.0, 0.002))
        c, s = math.cos(self.theta), math.sin(self.theta)
        sag = self.sag * fr.sin_a                      # along e_d
        disp = tuple((v * (c * fr.e_h[k] + s * fr.e_d[k]) + sag * fr.e_d[k]) * dt for k in range(3))
        moved, twist = 0.0, 0.0
        nxz = hull.advance(self.x, self.z, fr.p, disp)
        if nxz is not None:
            nf = hull.frame(*nxz)
            moved = math.dist(fr.p, nf.p)
            twist = self._frame_twist(fr, nf)
            self.x, self.z = nxz
            self.fr = nf
        self.last_moved = moved
        self.last_ratio = moved / (v_cmd * dt) if v_cmd > 1e-3 else 1.0
        # What a gyro about the surface normal reads: heading change + frame twist.
        self._body_rate = wrap(self.theta - th0) / dt + twist / dt
        return v_cmd * dt   # wheel odometry: wheels turn at the commanded speed

    @staticmethod
    def _frame_twist(old: Frame, new: Frame) -> float:
        """How much the gravity frame (e_h) rotated about the normal, relative
        to parallel transport from old to new. Positive = same sense as theta."""
        t = old.e_h
        k = _dot(t, new.n)
        t = _unit((t[0] - k * new.n[0], t[1] - k * new.n[1], t[2] - k * new.n[2]))
        axis = _cross(new.e_h, new.e_d)                 # rotation that takes e_h toward e_d
        return math.atan2(_dot(axis, _cross(t, new.e_h)), _dot(t, new.e_h))

    def edge_distance_ahead(self, max_d: float = 0.36, step: float = 0.03) -> float:
        """March along the skin on the current (gravity-frame) heading until the
        hull ends: keel line, transom or waterline."""
        hull, th = self.hull, self.theta
        c, s = math.cos(th), math.sin(th)
        x, z, fr, d = self.x, self.z, self.fr, 0.0
        while d < max_d:
            d += step
            nxz = hull.advance(x, z, fr.p, tuple(step * (c * fr.e_h[k] + s * fr.e_d[k]) for k in range(3)))
            if nxz is None:
                return d
            x, z = nxz
            fr = hull.frame(x, z)
        return max_d

    def sense(self, odom_ds: float, moving_cmd: bool, camera: bool, dt: float) -> s2.Sensors:
        r = self.rng
        cam = 0.0
        if camera:
            cam = clamp(1.0 - self.edge_distance_ahead() / 0.35, 0.0, 1.0)
            if r.random() < 0.10:
                cam *= 0.3
            if r.random() < 0.03:
                cam = max(cam, r.uniform(0.7, 1.0))
        stall = 1.0 if (moving_cmd and self.last_ratio < 0.3) else 0.0
        if r.random() < 0.02:
            stall = 1.0 - stall
        flow = None if r.random() < 0.10 else self.last_moved * (1.0 + r.normal(0.0, 0.03))
        heading = self.imu.update(self.theta, self._body_rate, self.fr.sin_a, dt)
        return s2.Sensors(depth=self.z + r.normal(0.0, 0.005), heading=heading,
                          odom_ds=odom_ds, cam_edge=cam, stall=stall, flow_ds=flow)


# ---------------------------------------------------------------------------
# Simulation wrapper
# ---------------------------------------------------------------------------
@dataclass
class Simulation3D:
    seed: int = 1
    camera: bool = True
    dt: float = 0.05
    hull: Hull3D = field(default_factory=Hull3D)
    params: s2.SweepParams = field(default_factory=s2.SweepParams)
    mesh_nx: int = 220
    mesh_nu: int = 60
    snapshot_every: int = 0     # ticks between coverage snapshots (for the animation); 0 = none

    def __post_init__(self):
        self.rng = np.random.default_rng(self.seed)
        # Same deployment as the 2D sim: stern corner, just below the waterline, facing down.
        x0 = self.hull.plan.stern_x + 0.25
        self.plant = Plant3D(self.hull, self.rng, x0=x0, z0=0.04)
        self.ctrl = s2.SweepController(self.params)
        stern_ref = self.hull.plan.stern_x + 0.04
        self.est_dr = s2.WallEstimator(x0, reset_x=None, use_flow=False)
        self.est_rs = s2.WallEstimator(x0, reset_x=stern_ref, use_flow=False)
        self.est = s2.WallEstimator(x0, reset_x=stern_ref, use_flow=True)
        self.estimators = (self.est_dr, self.est_rs, self.est)
        self.mesh = self.hull.mesh(self.mesh_nx, self.mesh_nu)
        self.grid = Coverage3D(self.mesh, self.params.brush_width)
        self.t = 0.0
        self.cmd = s2.Command(0.0, math.pi / 2)
        self.log: dict[str, list] = {k: [] for k in
                                     ("t", "x", "y", "z", "theta", "theta_hat", "sin_a", "state",
                                      "lane", "x_est", "depth_err")}
        self.snapshots: list[tuple[int, np.ndarray]] = []
        self._n_events = 0
        self._tick = 0

    @property
    def done(self) -> bool:
        return self.ctrl.state == s2.State.DONE

    def step(self) -> None:
        # Keep in step with sweep_sim.Simulation.step: the control path must be identical.
        if self.done:
            return
        p, pl = self.params, self.plant
        omega = s2.mcu_heading_loop(p, self.cmd.heading, pl.theta)
        odom = pl.step(self.cmd.v, omega, self.dt)
        s = pl.sense(odom, self.cmd.v > 1e-3, self.camera, self.dt)
        for e in self.estimators:
            e.update(s)
        self.cmd = self.ctrl.update(s, self.dt)
        for name, info in self.ctrl.events[self._n_events:]:
            if name == "end_of_lane":
                for e in self.estimators:
                    e.on_end_of_lane(info["direction"])
        self._n_events = len(self.ctrl.events)
        if self.cmd.brush_on:
            self.grid.mark(pl.pos)
        self.t += self.dt
        self._tick += 1
        L, pos = self.log, pl.pos
        L["t"].append(self.t); L["x"].append(pos[0]); L["y"].append(pos[1]); L["z"].append(pos[2])
        L["theta"].append(pl.theta); L["theta_hat"].append(s.heading); L["sin_a"].append(pl.fr.sin_a)
        L["state"].append(self.ctrl.state.value); L["lane"].append(self.ctrl.lane_idx)
        L["x_est"].append(self.est.x)
        L["depth_err"].append(pl.z - self.ctrl.lane_depth)
        if self.snapshot_every and self._tick % self.snapshot_every == 0:
            self.snapshots.append((self._tick, self.grid.cleaned.copy()))

    def run(self, max_time: float = 3600.0) -> None:
        while not self.done and self.t < max_time:
            self.step()
        if self.snapshot_every:
            self.snapshots.append((self._tick, self.grid.cleaned.copy()))

    # -- analysis ------------------------------------------------------------
    def lane_report(self) -> list[dict]:
        """Per-lane numbers measured while the robot was in LANE state."""
        L = {k: np.asarray(v) for k, v in self.log.items() if k != "state"}
        state = np.asarray(self.log["state"])
        rows = []
        for lane in range(self.ctrl.lane_idx + 1):
            m = (L["lane"] == lane) & (state == "LANE")
            if m.sum() < 5:
                continue
            herr = np.degrees(np.abs(np.array([wrap(a) for a in (L["theta_hat"][m] - L["theta"][m])])))
            rows.append({"lane": lane, "depth": float(np.median(L["z"][m])),
                         "sin_a": float(np.median(L["sin_a"][m])),
                         "min_sin_a": float(L["sin_a"][m].min()),
                         "depth_rms_cm": float(100 * np.sqrt(np.mean(L["depth_err"][m] ** 2))),
                         "heading_err_deg": float(herr.mean()),
                         "x_start": float(L["x"][m][0]), "x_end": float(L["x"][m][-1])})
        return rows


def make_hull(kind: str, section_exp: float, half_beam: float) -> Hull3D:
    if kind == "plane":
        return Hull3D(half_beam=half_beam, plane=True)
    return Hull3D(half_beam=half_beam, section_exp=section_exp)


def summary(sim: Simulation3D, two_d: s2.Simulation | None) -> str:
    out = []
    g = sim.grid
    p = sim.params
    out.append(f"finished: {sim.ctrl.state.value}  t={sim.t / 60:.1f} min  lanes={sim.ctrl.lane_idx + 1}")
    out.append(f"coverage, hull surface area:  target (z <= {p.max_depth:.2f} m) {g.coverage(p.max_depth):.1%}   "
               f"whole port skin to the keel line {g.coverage():.1%}")
    if two_d is not None:
        out.append(f"same seed, 2D sheet sim:      {two_d.grid.coverage():.1%}   "
                   f"lanes={two_d.ctrl.lane_idx + 1}  t={two_d.t / 60:.1f} min")
    h = sim.hull
    out.append("")
    out.append("lane  depth(m)  slope sin(a)  surface spacing to prev lane   depth RMS   heading err")
    prev = None
    for r in sim.lane_report():
        gap = ""
        if prev is not None:
            arc = h.arc_down(0.5 * (h.plan.stern_x + h.plan.length - h.plan.bow_cutaway), prev["depth"], r["depth"])
            gap = f"{arc:5.2f} m  (brush {p.brush_width:.2f})" + ("  <-- GAP" if arc > p.brush_width else "")
        out.append(f"{r['lane']:3d}   {r['depth']:6.2f}    {r['sin_a']:5.2f} (min {r['min_sin_a']:4.2f})  "
                   f"{gap:34s}  {r['depth_rms_cm']:5.1f} cm    {r['heading_err_deg']:4.1f} deg")
        prev = r
    out.append("")
    out.append("coverage by depth band (area-weighted):")
    edges = [0.0, 0.3, 0.6, 0.9, 1.1, 1.4]
    for lo, hi, c in g.by_depth_band(edges):
        out.append(f"  {lo:.1f}-{hi:.1f} m  {c:6.1%}  " + "#" * int(round(c * 30)) if c == c else f"  {lo:.1f}-{hi:.1f} m   n/a")
    xe = np.abs(np.array(sim.log["x_est"]) - np.array(sim.log["x"]))
    out.append("")
    out.append(f"x estimate (flow + stern reset) vs true station: mean {xe.mean():.2f} m, max {xe.max():.2f} m")
    return "\n".join(out)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seed", type=int, default=2)
    ap.add_argument("--hull", choices=["default", "plane"], default="default")
    ap.add_argument("--section-exp", type=float, default=4.0, help="cross-section boxiness (2 = ellipse)")
    ap.add_argument("--half-beam", type=float, default=1.5)
    ap.add_argument("--no-camera", action="store_true")
    ap.add_argument("--quiet", action="store_true", help="don't print the event log")
    ap.add_argument("--png", default=str(Path(__file__).resolve().parents[1] / "docs/img/sim3d.png"),
                    help="static render ('' to skip)")
    ap.add_argument("--html", default="", help="interactive 3D viewer (needs plotly)")
    a = ap.parse_args()

    hull = make_hull(a.hull, a.section_exp, a.half_beam)
    sim = Simulation3D(seed=a.seed, camera=not a.no_camera, hull=hull,
                       snapshot_every=60 if a.html else 0)
    sim.run()
    two_d = s2.Simulation(seed=a.seed, camera=not a.no_camera)
    two_d.run()
    print(summary(sim, two_d))
    if not a.quiet:
        print()
        for name, info in sim.ctrl.events:
            print(f"  {name:18s} {info}")
    import viz3d
    if a.png:
        viz3d.render_png(sim, Path(a.png))
        print(f"png  -> {a.png}")
    if a.html:
        viz3d.write_html(sim, Path(a.html))
        print(f"html -> {a.html}")


if __name__ == "__main__":
    main()
