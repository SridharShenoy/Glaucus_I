"""coverage_mapper — topside. "Which parts of the hull have we cleaned?"

    sub: /glaucus/sweep_status (SweepStatus, 5 Hz)   x_est, depth, state
    pub: /glaucus/coverage      (nav_msgs/OccupancyGrid, 1 Hz)
         0 = not cleaned, 100 = cleaned, -1 = unknown / not hull

Same algorithm as CoverageGrid in sim/sweep_sim.py: stamp a disk of the brush
width at every pose where the brush is on. The grid lives in the "unrolled
hull side" frame (x along hull, z depth), NOT a world frame.

DESIGN CHOICES
  - Coverage is computed topside from the status stream, not on the Pi. It's
    bookkeeping, not control, and it's where the app wants it anyway.
  - The map is only as good as x_est. Depth (rows) is trustworthy; columns can
    be smeared by along-hull drift. Show that honestly in the app (e.g. blur
    the x axis by the expected error) rather than claiming 100%.
  - 5 Hz status at 0.15 m/s = 3 cm between samples < 2 cm cells... close.
    Interpolate between consecutive poses before stamping (TODO below).

STATUS: working skeleton (no interpolation, fixed grid extent).
TODO(glaucus): grid extent from a hull profile config; save/load per boat for resume.
"""
from __future__ import annotations

import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node

from glaucus_interfaces.msg import SweepStatus

BRUSH_ON_STATES = {"LANE", "VERIFY", "STEP_DOWN", "SHIFT"}


class CoverageMapper(Node):
    def __init__(self) -> None:
        super().__init__("coverage_mapper")
        self.res = self.declare_parameter("resolution", 0.02).value
        self.length = self.declare_parameter("hull_length", 10.0).value
        self.draft = self.declare_parameter("hull_draft", 1.5).value
        brush = self.declare_parameter("brush_width", 0.30).value

        self.nx = int(math.ceil(self.length / self.res))
        self.nz = int(math.ceil(self.draft / self.res))
        self.cleaned = np.zeros((self.nz, self.nx), dtype=bool)
        r, k = brush / 2, int(math.ceil(brush / 2 / self.res))
        self.offsets = [(i, j) for i in range(-k, k + 1) for j in range(-k, k + 1)
                        if (i * self.res) ** 2 + (j * self.res) ** 2 <= r * r]

        self.create_subscription(SweepStatus, "/glaucus/sweep_status", self.on_status, 10)
        self.pub = self.create_publisher(OccupancyGrid, "/glaucus/coverage", 1)
        self.create_timer(1.0, self.publish)

    def on_status(self, m: SweepStatus) -> None:
        if m.state not in BRUSH_ON_STATES or m.paused:
            return
        # TODO(glaucus): interpolate from the previous pose to this one.
        ci, cj = int(m.depth / self.res), int(m.x_est / self.res)
        for di, dj in self.offsets:
            i, j = ci + di, cj + dj
            if 0 <= i < self.nz and 0 <= j < self.nx:
                self.cleaned[i, j] = True

    def publish(self) -> None:
        g = OccupancyGrid()
        g.header.stamp = self.get_clock().now().to_msg()
        g.header.frame_id = "hull_side"
        g.info.resolution = float(self.res)
        g.info.width, g.info.height = self.nx, self.nz
        g.data = np.where(self.cleaned, 100, 0).astype(np.int8).ravel().tolist()
        self.pub.publish(g)

    def coverage_fraction(self) -> float:
        return float(self.cleaned.mean())


def main() -> None:
    rclpy.init()
    node = CoverageMapper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
