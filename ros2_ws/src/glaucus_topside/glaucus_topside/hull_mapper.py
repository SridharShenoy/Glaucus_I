"""hull_mapper — topside. STRETCH GOAL. Not needed for the core demo.

The sweep works without a map: depth defines lanes, edges are detected.
A hull map is what you'd add to go further:

  Level 1 - photo mosaic: stitch the (compressed) camera stream into an image
            of the hull side, placed by (x_est, depth). Great demo/visual,
            shows fouling before/after. Cheap: no SLAM needed.
  Level 2 - visual odometry correction: register each new frame against the
            mosaic to correct x_est (loop closure on the previous lane, since
            adjacent lanes overlap by `overlap`). This is "SLAM-lite" in 2D
            and is the realistic SLAM story for a senior design project.
  Level 3 - full 3D hull reconstruction / ORB-SLAM3. Expensive, fragile
            underwater, and the sweep doesn't need it. Only if everything
            else is done.

DESIGN CHOICE: runs topside on the compressed stream. A 2 GB Pi can technically
run ORB-SLAM3, but slowly, and it would starve the control stack.

    sub: /camera/image_raw/compressed, /glaucus/sweep_status
    pub: /glaucus/hull_mosaic (sensor_msgs/CompressedImage, ~0.2 Hz)

STATUS: placeholder.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node


class HullMapper(Node):
    def __init__(self) -> None:
        super().__init__("hull_mapper")
        self.get_logger().info("hull_mapper is a placeholder (stretch goal) - see module docstring")
        # TODO(glaucus): Level 1 mosaic: for each frame, warp to hull-plane pixels at
        # (x_est, depth) and alpha-blend into a big canvas; publish a downscaled JPEG.


def main() -> None:
    rclpy.init()
    node = HullMapper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
