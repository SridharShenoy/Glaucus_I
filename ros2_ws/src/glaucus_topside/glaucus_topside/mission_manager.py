"""mission_manager — topside. Owns "what mission are we running".

    pubs:   /glaucus/topside_heartbeat (std_msgs/Empty, 2 Hz)
            /glaucus/mission_state     (std_msgs/String, JSON, latched)
    action client: /glaucus/run_sweep (RunSweep) -> sweep_executor on the bot
    service clients: /glaucus/pause, /glaucus/estop (forwarded from the app)

The web gateway (webapp/gateway/ros_bridge.py) calls the methods on this node
indirectly through the services below, so the app never talks to the bot
directly. One place decides what the robot is doing.

    services offered (std_srvs/Trigger):
            /glaucus/mission/start
            /glaucus/mission/abort
    and (std_srvs/SetBool): /glaucus/mission/pause

DESIGN CHOICES
  - Heartbeat goes OUT from topside. If the bot stops hearing it (tether
    unplugged, topside crashed), the executor finishes its lane and holds.
  - Only one goal at a time. Starting while running is rejected, not queued.
  - Mission parameters come from ROS params (config/sweep_params.yaml) with
    per-mission overrides from the app.

STATUS: skeleton - start/abort/pause wired, no mission history / resume yet.
TODO(glaucus): resume-from-lane-N using the coverage map after an abort.
"""
from __future__ import annotations

import json

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Empty, String
from std_srvs.srv import SetBool, Trigger

from glaucus_interfaces.action import RunSweep


class MissionManager(Node):
    def __init__(self) -> None:
        super().__init__("mission_manager")
        self.declare_parameter("hull_side", 1)
        self.declare_parameter("first_lane_depth", 0.0)  # 0 -> bot default
        self.declare_parameter("max_depth", 0.0)
        self.declare_parameter("overlap", -1.0)

        self._hb = self.create_publisher(Empty, "/glaucus/topside_heartbeat", 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._state_pub = self.create_publisher(String, "/glaucus/mission_state", latched)
        self.create_timer(0.5, lambda: self._hb.publish(Empty()))

        self._sweep = ActionClient(self, RunSweep, "/glaucus/run_sweep")
        self._pause_cli = self.create_client(SetBool, "/glaucus/pause")
        self._estop_cli = self.create_client(Trigger, "/glaucus/estop")
        self._goal_handle = None
        self._state = "IDLE"

        self.create_service(Trigger, "/glaucus/mission/start", self._on_start)
        self.create_service(Trigger, "/glaucus/mission/abort", self._on_abort)
        self.create_service(SetBool, "/glaucus/mission/pause", self._on_pause)
        self._set_state("IDLE")

    # ---- app-facing services ------------------------------------------------
    def _on_start(self, _req, res):
        if self._goal_handle is not None:
            res.success, res.message = False, "a mission is already running"
            return res
        if not self._sweep.wait_for_server(timeout_sec=1.0):
            res.success, res.message = False, "bot not reachable (sweep_executor not up)"
            return res
        goal = RunSweep.Goal()
        goal.hull_side = int(self.get_parameter("hull_side").value)
        goal.first_lane_depth = float(self.get_parameter("first_lane_depth").value)
        goal.max_depth = float(self.get_parameter("max_depth").value)
        goal.overlap = float(self.get_parameter("overlap").value)
        fut = self._sweep.send_goal_async(goal, feedback_callback=self._on_feedback)
        fut.add_done_callback(self._on_goal_response)
        self._set_state("STARTING")
        res.success, res.message = True, "goal sent"
        return res

    def _on_abort(self, _req, res):
        # Abort = cancel the goal AND e-stop (hold). Belt and braces.
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        if self._estop_cli.service_is_ready():
            self._estop_cli.call_async(Trigger.Request())
        self._set_state("ABORTED")
        res.success, res.message = True, "abort sent"
        return res

    def _on_pause(self, req, res):
        if self._pause_cli.service_is_ready():
            self._pause_cli.call_async(SetBool.Request(data=req.data))
            self._set_state("PAUSED" if req.data else "RUNNING")
            res.success = True
        else:
            res.success, res.message = False, "bot pause service not available"
        return res

    # ---- action plumbing ----------------------------------------------------
    def _on_goal_response(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self._set_state("REJECTED")
            return
        self._goal_handle = gh
        self._set_state("RUNNING")
        gh.get_result_async().add_done_callback(self._on_result)

    def _on_feedback(self, fb):
        pass  # status is also on /glaucus/sweep_status; coverage_mapper uses that

    def _on_result(self, fut):
        r = fut.result().result
        self.get_logger().info(f"sweep finished: completed={r.completed} lanes={r.lanes} "
                               f"in {r.duration_s:.0f}s ({r.reason})")
        self._goal_handle = None
        self._set_state("COMPLETE" if r.completed else "STOPPED")

    def _set_state(self, s: str) -> None:
        self._state = s
        self._state_pub.publish(String(data=json.dumps({"mission": s})))


def main() -> None:
    rclpy.init()
    node = MissionManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
