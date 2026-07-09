#!/usr/bin/env python3
"""
axis_sweep.py

Sends each of your 12 actual marker grid points (from
marker_positions_actual.json) to MoveIt as a hover target at fixed Z,
one at a time, waiting for each motion to finish before sending the
next. Logs a pass/fail table at the end so you can see exactly which
of your real placement points are reachable under the current fixed
hover orientation.

Usage:
    python3 axis_sweep.py

Requires your MoveIt launch (fake hardware / real robot) to already
be running -- this script talks to /move_action directly, so
hover_xy_node.py does NOT need to be running at the same time.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup as MoveGroupAction
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints,
    PositionConstraint, OrientationConstraint, BoundingVolume,
)
from shape_msgs.msg import SolidPrimitive


HOVER_Z_MM = 100.0
PLANNING_GROUP = "manipulator"
PLANNING_FRAME = "world"
EEF_LINK = "tool0"

ORI_X = 0.0090
ORI_Y = 0.9280
ORI_Z = 0.0184
ORI_W = 0.3719
ORIENTATION_TOLERANCE_RAD = 0.2


# --- Sweep definitions (mm) ---
# Loaded from your actual marker placement data (marker_positions_actual.json)
MARKER_POINTS = [
    {"id": 0,  "x_mm": 340,   "y_mm": 0},
    '''{"id": 1,  "x_mm": 490,   "y_mm": 0},
    {"id": 2,  "x_mm": 190,   "y_mm": 0},
    {"id": 3,  "x_mm": 415,   "y_mm": 0},
    {"id": 4,  "x_mm": 384,   "y_mm": -145},
    {"id": 5,  "x_mm": 465,   "y_mm": -85},
    {"id": 6,  "x_mm": 508,   "y_mm": 89},
    {"id": 7,  "x_mm": 453.5, "y_mm": 210},
    {"id": 8,  "x_mm": 540,   "y_mm": -328},
    {"id": 9,  "x_mm": 320,   "y_mm": 120},
    {"id": 10, "x_mm": 336,   "y_mm": 275},
    {"id": 11, "x_mm": 235,   "y_mm": -225},'''
]


class AxisSweepNode(Node):
    def __init__(self):
        super().__init__('axis_sweep_node')
        self._client = ActionClient(self, MoveGroupAction, 'move_action')
        self.results = []

    def wait_for_server(self):
        self.get_logger().info("Waiting for /move_action server...")
        self._client.wait_for_server()
        self.get_logger().info("Server ready.")

    def _build_goal(self, x_mm, y_mm, z_mm):
        x, y, z = x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0

        goal = MoveGroupAction.Goal()
        req = MotionPlanRequest()
        req.group_name = PLANNING_GROUP
        req.pipeline_id = "pilz_industrial_motion_planner"
        req.planner_id = "PTP"
        req.num_planning_attempts = 5
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = 0.05
        req.max_acceleration_scaling_factor = 0.05

        constraints = Constraints()

        pos_c = PositionConstraint()
        pos_c.header.frame_id = PLANNING_FRAME
        pos_c.link_name = EEF_LINK
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.002, 0.002, 0.002]
        bv = BoundingVolume()
        bv.primitives.append(box)
        p = Pose()
        p.position.x, p.position.y, p.position.z = x, y, z
        p.orientation.w = 1.0
        bv.primitive_poses.append(p)
        pos_c.constraint_region = bv
        pos_c.weight = 1.0
        constraints.position_constraints.append(pos_c)

        ori_c = OrientationConstraint()
        ori_c.header.frame_id = PLANNING_FRAME
        ori_c.link_name = EEF_LINK
        ori_c.orientation.x = ORI_X
        ori_c.orientation.y = ORI_Y
        ori_c.orientation.z = ORI_Z
        ori_c.orientation.w = ORI_W
        ori_c.absolute_x_axis_tolerance = ORIENTATION_TOLERANCE_RAD
        ori_c.absolute_y_axis_tolerance = ORIENTATION_TOLERANCE_RAD
        ori_c.absolute_z_axis_tolerance = ORIENTATION_TOLERANCE_RAD
        ori_c.weight = 1.0
        constraints.orientation_constraints.append(ori_c)

        req.goal_constraints.append(constraints)
        goal.request = req
        goal.planning_options.plan_only = False
        goal.planning_options.replan = False
        goal.planning_options.look_around = False
        return goal

    def send_and_wait(self, x_mm, y_mm, z_mm, timeout_sec=40.0):
        goal = self._build_goal(x_mm, y_mm, z_mm)
        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        handle = future.result()

        if handle is None or not handle.accepted:
            return False, "goal rejected"

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        result = result_future.result()

        if result is None:
            # Timed out waiting -- actively cancel so the controller doesn't
            # stay mid-goal and reject the next request.
            self.get_logger().warn("  (timed out waiting for result -- cancelling goal)")
            cancel_future = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
            time.sleep(2.0)  # let controller fully settle after cancel
            return False, "timeout"

        ec = result.result.error_code.val
        return (ec == 1), f"error_code={ec}"

    def run_sweep(self, points, label):
        self.get_logger().info(f"--- {label} ---")
        for m in points:
            x_mm, y_mm = m["x_mm"], m["y_mm"]
            self.get_logger().info(f"  Sending ID={m['id']}  X={x_mm:.1f}  Y={y_mm:.1f} ...")
            ok, detail = self.send_and_wait(x_mm, y_mm, HOVER_Z_MM)
            status = "PASS" if ok else "FAIL"
            self.get_logger().info(f"  ID={m['id']:>2}  X={x_mm:>6.1f}  Y={y_mm:>6.1f}  -> {status} ({detail})")
            self.results.append((m['id'], x_mm, y_mm, ok, detail))
            time.sleep(1.5)  # settle time between goals regardless of outcome

    def print_summary(self):
        self.get_logger().info("\n=== SUMMARY ===")
        n_pass = sum(1 for r in self.results if r[3])
        n_total = len(self.results)
        for mid, x, y, ok, detail in self.results:
            status = "PASS" if ok else "FAIL"
            self.get_logger().info(f"ID={mid:>2}  X={x:>6.1f}  Y={y:>6.1f}  -> {status}")
        self.get_logger().info(f"\n{n_pass}/{n_total} points reachable at Z={HOVER_Z_MM}mm hover.")


def main():
    rclpy.init()
    node = AxisSweepNode()
    node.wait_for_server()

    node.run_sweep(MARKER_POINTS, "MARKER GRID SWEEP")

    node.print_summary()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()