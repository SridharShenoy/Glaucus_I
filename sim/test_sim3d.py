"""
Tests for the 3D simulation environment. Plain asserts, numpy only:

    python3 sim/test_sim3d.py        (also collectable by pytest)

The test that matters most is test_plane_reproduces_2d_sim: the 3D plant on a
vertical plane IS the 2D sheet world, so it must give the same answer. If it
doesn't, a result on the curved hull can't be trusted.
"""
from __future__ import annotations

import math
import random
import sys

import numpy as np

import sim3d
import sweep_sim as s2
from sim3d import Hull3D, Simulation3D

G = (0.0, 0.0, 1.0)


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _interior_points(h: Hull3D, n: int, seed: int = 0):
    rnd = random.Random(seed)
    for _ in range(n):
        x = rnd.uniform(h.plan.stern_x + 0.05, h.plan.length - 0.05)
        z = rnd.uniform(0.05, 0.97 * h.keel_depth(x))
        yield x, z


# -- geometry ----------------------------------------------------------------
def test_planform_matches_2d_hull():
    """Lane ends in 3D must coincide with the 2D sim's, or the two aren't comparable."""
    h3, h2 = Hull3D(), s2.Hull()
    rnd = random.Random(1)
    checked = 0
    for _ in range(20000):
        x, z = rnd.uniform(0.0, 9.2), rnd.uniform(0.0, 1.45)
        if abs(z - h3.keel_depth(x)) < 1e-6 or abs(z - 0.02) < 1e-9:
            continue
        assert h3.contains(x, z) == h2.contains(x, z), (x, z)
        checked += 1
    assert checked > 19000


def test_skin_touches_centerline_at_keel_and_is_full_beam_at_waterline():
    h = Hull3D()
    for x in (1.0, 3.0, 6.0, 8.0):
        assert abs(h.y(x, h.keel_depth(x))) < 1e-9
        assert abs(h.y(x, 0.0) - h.beam(x)) < 1e-9


def test_frame_is_orthonormal_level_and_points_bow_ward():
    h = Hull3D()
    for x, z in _interior_points(h, 3000):
        f = h.frame(x, z)
        assert abs(_dot(f.e_h, f.e_h) - 1) < 1e-9 and abs(_dot(f.e_d, f.e_d) - 1) < 1e-9
        assert abs(_dot(f.e_h, f.e_d)) < 1e-9 and abs(_dot(f.e_h, f.n)) < 1e-9 and abs(_dot(f.e_d, f.n)) < 1e-9
        assert abs(_dot(f.e_h, G)) < 1e-9, "theta = 0 must be constant depth"
        assert f.e_h[0] > 0.0, "theta = 0 must point toward the bow"
        assert _dot(f.e_d, G) > 0.0, "theta = +pi/2 must point downhill"
        assert abs(f.sin_a - math.sqrt(1 - _dot(G, f.n) ** 2)) < 1e-12


def test_incline_grows_with_depth_amidships():
    h = Hull3D()
    s = [h.frame(3.0, z).sin_a for z in np.linspace(0.02, 1.2, 25)]
    assert s[0] > 0.999
    assert all(a >= b - 1e-9 for a, b in zip(s, s[1:])), "side should only tip over as you go down"
    assert s[-1] < 0.6


def test_advance_follows_the_surface_even_at_the_keel_line():
    """Regression: stepping x,z by the displacement's own x/z parts pins the robot at
    the keel line, where the skin is horizontal and a sideways step is pure y."""
    h = Hull3D()
    rnd = random.Random(3)
    for x, z in list(_interior_points(h, 400, seed=5)) + [(8.03, 0.669), (5.0, 1.29), (7.0, 1.05)]:
        fr = h.frame(x, z)
        for _ in range(4):
            a = rnd.uniform(0, 2 * math.pi)
            step = 0.01
            disp = tuple(step * (math.cos(a) * fr.e_h[k] + math.sin(a) * fr.e_d[k]) for k in range(3))
            nxz = h.advance(x, z, fr.p, disp)
            if nxz is None:
                continue
            moved = math.dist(fr.p, h.frame(*nxz).p)
            assert abs(moved - step) < 0.05 * step, (x, z, a, moved)


def test_advance_refuses_to_cross_the_keel_centerline_and_leave_the_hull():
    h = Hull3D()
    fr = h.frame(6.0, 1.0)
    # walk downhill until something stops us; it must be the centerline, never a crash or a wrap-around
    x, z = 6.0, 1.0
    for _ in range(500):
        fr = h.frame(x, z)
        nxz = h.advance(x, z, fr.p, tuple(0.01 * fr.e_d[k] for k in range(3)))
        if nxz is None:
            break
        x, z = nxz
    else:
        raise AssertionError("never reached the keel")
    assert h.frame(x, z).p[1] < h.edge_margin + 0.011
    assert h.advance(0.5, 0.5, h.frame(0.5, 0.5).p, (-1.0, 0.0, 0.0)) is None   # off the transom


def test_mesh_area_is_converged_and_plane_area_is_exact():
    h = Hull3D()
    a1, a2 = h.mesh(110, 30).area.sum(), h.mesh(220, 60).area.sum()
    assert abs(a1 - a2) / a2 < 0.01
    hp = Hull3D(plane=True)
    mp = hp.mesh(100, 40)
    # sheet area = integral of D(x) dx
    xs = np.linspace(hp.plan.stern_x, hp.plan.length, 20001)
    depth = np.array([hp.keel_depth(float(x)) for x in xs])
    exact = float(((depth[:-1] + depth[1:]) / 2 * np.diff(xs)).sum())     # trapezoid rule
    assert abs(mp.area.sum() - exact) / exact < 0.01


def test_coverage_marks_a_disc():
    h = Hull3D(plane=True)
    g = sim3d.Coverage3D(h.mesh(180, 52), 0.30)
    g.mark((3.0, h.half_beam, 0.5))
    got = g.mesh.area[g.cleaned].sum()
    assert abs(got - math.pi * 0.15 ** 2) / (math.pi * 0.15 ** 2) < 0.08


# -- sensors -----------------------------------------------------------------
def test_imu_locks_to_gravity_on_a_wall_and_goes_gyro_only_on_a_flat_patch():
    rng = np.random.default_rng(0)
    imu = sim3d.IMU(rng, theta0=0.5)           # start 0.5 rad wrong
    for _ in range(400):                       # 20 s, robot sitting at theta = 0
        th = imu.update(0.0, 0.0, 1.0, 0.05)
    assert abs(th) < 0.1
    imu = sim3d.IMU(rng, theta0=0.5)
    for _ in range(400):                       # flat patch: gravity says nothing, so the error must stay
        th = imu.update(0.0, 0.0, 0.0, 0.05)
    assert abs(th - 0.5) < 0.2


# -- whole-sweep behavior ----------------------------------------------------
def test_plane_reproduces_2d_sim():
    for seed in (1, 2, 3):
        two = s2.Simulation(seed=seed)
        two.run()
        thr = Simulation3D(seed=seed, hull=sim3d.make_hull("plane", 4.0, 1.5))
        thr.run()
        assert thr.ctrl.state == s2.State.DONE
        assert thr.ctrl.lane_idx == two.ctrl.lane_idx, (seed, thr.ctrl.lane_idx, two.ctrl.lane_idx)
        assert abs(thr.t - two.t) / two.t < 0.10, (seed, thr.t, two.t)
        assert abs(thr.grid.coverage() - two.grid.coverage()) < 0.03, (seed, thr.grid.coverage(), two.grid.coverage())


def test_curved_hull_sweep_completes_and_holds_depth():
    sim = Simulation3D(seed=2)
    sim.run()
    assert sim.ctrl.state == s2.State.DONE
    assert sim.ctrl.events[-1][1].get("reason") is None, "sweep gave up instead of finishing"
    assert sim.ctrl.lane_idx + 1 == 5
    assert sim.grid.coverage(sim.params.max_depth) > 0.90
    rep = sim.lane_report()
    assert len(rep) == 5
    assert max(r["depth_rms_cm"] for r in rep) < 3.0, "depth hold regressed on the inclined lanes"


def main() -> int:
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_") and callable(f)]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            print(f"  ok    {name}")
        except Exception as e:  # noqa: BLE001
            failed += 1
            print(f"  FAIL  {name}: {type(e).__name__}: {e}")
    print(f"{len(tests) - failed}/{len(tests)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
