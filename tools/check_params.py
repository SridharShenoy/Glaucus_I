#!/usr/bin/env python3
"""Fail if sweep parameter defaults drift apart between the three places they live:

    sim/sweep_sim.py                                   (Python SweepParams)
    ros2_ws/src/glaucus_core/include/glaucus_core/types.hpp   (C++ SweepParams)
    ros2_ws/src/glaucus_bringup/config/sweep_params.yaml      (what the robot actually loads)

Python and C++ must match exactly. The YAML may add robot-only keys, but every
key it shares with SweepParams must have the same value. (When you tune on the
robot, copy the new value into all three - that's the point.)
"""
from __future__ import annotations

import dataclasses
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sim"))
import sweep_sim  # noqa: E402


def cpp_params() -> dict[str, float]:
    src = (ROOT / "ros2_ws/src/glaucus_core/include/glaucus_core/types.hpp").read_text()
    body = src[src.index("struct SweepParams"):]
    body = body[:body.index("};")]
    return {m.group(2): float(m.group(3))
            for m in re.finditer(r"\b(double|int)\s+(\w+)\s*=\s*([-0-9.eE]+)\s*;", body)}


def yaml_params() -> dict[str, float]:
    out = {}
    for line in (ROOT / "ros2_ws/src/glaucus_bringup/config/sweep_params.yaml").read_text().splitlines():
        m = re.match(r"\s+(\w+):\s*([-0-9.eE]+)\s*(#.*)?$", line)
        if m:
            out[m.group(1)] = float(m.group(2))
    return out


def main() -> int:
    py = {f.name: float(f.default) for f in dataclasses.fields(sweep_sim.SweepParams)}
    cpp, yml = cpp_params(), yaml_params()
    errors = []
    for k in sorted(set(py) | set(cpp)):
        if k not in py or k not in cpp:
            errors.append(f"{k}: only in {'Python' if k in py else 'C++'}")
        elif py[k] != cpp[k]:
            errors.append(f"{k}: Python {py[k]} != C++ {cpp[k]}")
    for k in sorted(set(yml) & set(py)):
        if yml[k] != py[k]:
            errors.append(f"{k}: YAML {yml[k]} != Python/C++ {py[k]}")
    if errors:
        print("parameter drift:\n  " + "\n  ".join(errors))
        return 1
    print(f"params in sync ({len(py)} SweepParams fields, {len(set(yml) & set(py))} shared with YAML)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
