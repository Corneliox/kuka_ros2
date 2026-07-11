#!/usr/bin/env python3
"""
pick_place_coordinator.py

Replaces vision_logic_mock.py. Same role in the pipeline (listens to
/voice_command, dispatches to /execute_task), but pick coordinates now
come from a live /detect_object service call instead of a hardcoded
STORAGE_COORDS table, and every object goes to the same fixed handoff
point instead of a per-object spaced HANDOFF_COORDS table.

Flow:
  1. /voice_command gives a color name (e.g. "red").
  2. Call /detect_object with that color -> get (x, y) in metres, or
     found=False if nothing matching was detected right now.
  3. Build a TaskPickPlace request:
       pick_pose  = (x, y, PICK_Z_M) with the gripper-calibrated orientation
       place_pose = fixed HANDOFF pose (same point every time)
  4. Call /execute_task -- surgical_control_server handles the rest
     (transit, descent, gripper on, transit to handoff, gripper off,
     retract, park) exactly as it always has.

Unlike the old mock, there's no "odd call = tray, even call = handoff"
toggle logic -- every request is simply "find this color, pick it up,
bring it to the handoff point." If you need round-trip (return the
object from handoff back to a tray spot), that's a separate behavior
to design later, not handled here.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup as MoveGroupAction
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints,
    PositionConstraint, OrientationConstraint, BoundingVolume,
)
from shape_msgs.msg import SolidPrimitive
from surgical_msgs.srv import TaskPickPlace, DetectObject

from kuka_surgical_demo.pick_place_constants import (
    ORI_X, ORI_Y, ORI_Z, ORI_W, ORIENTATION_TOLERANCE_RAD,
    PICK_Z_M, HANDOFF_X_M, HANDOFF_Y_M, HANDOFF_Z_M,
    PARK_X_M, PARK_Y_M, PARK_Z_M,
    PARK_ORI_X, PARK_ORI_Y, PARK_ORI_Z, PARK_ORI_W,
)

KNOWN_COLORS = {"red", "blue", "green", "yellow"}
PLANNING_GROUP = "manipulator"
PLANNING_FRAME = "world"
EEF_LINK = "tool0"


class PickPlaceCoordinator(Node):

    def __init__(self):
        super().__init__('pick_place_coordinator')
        self.cb = ReentrantCallbackGroup()

        self._task_client = self.create_client(
            TaskPickPlace, '/execute_task', callback_group=self.cb)
        self._detect_client = self.create_client(
            DetectObject, '/detect_object', callback_group=self.cb)
        self._move_client = ActionClient(
            self, MoveGroupAction, '/move_action', callback_group=self.cb)

        self.create_subscription(
            String, '/voice_command', self._voice_callback, 10)

        self.get_logger().info(
            'Pick-Place Coordinator online. '
            f'Known colors: {sorted(KNOWN_COLORS)}')

    def _voice_callback(self, msg: String) -> None:
        color = msg.data.strip().lower()

        if color not in KNOWN_COLORS:
            self.get_logger().warn(
                f'Unknown color: "{color}". Valid: {sorted(KNOWN_COLORS)}')
            return

        if not self._detect_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(
                '/detect_object service not available -- is vision_node running?')
            return
        if not self._task_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(
                '/execute_task service not available -- is surgical_control_server running?')
            return
        if not self._move_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                '/move_action server not available -- is MoveIt running?')
            return

        self.get_logger().info(
            f'"{color}" requested -- moving to parked observation pose before detecting...')
        self._move_to_park(color)

    # ── Park move (must happen before every detection -- the homography is ──
    # ── only valid at this exact pose) ────────────────────────────────────

    def _build_park_goal(self):
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
        p.position.x, p.position.y, p.position.z = PARK_X_M, PARK_Y_M, PARK_Z_M
        p.orientation.w = 1.0
        bv.primitive_poses.append(p)
        pos_c.constraint_region = bv
        pos_c.weight = 1.0
        constraints.position_constraints.append(pos_c)

        ori_c = OrientationConstraint()
        ori_c.header.frame_id = PLANNING_FRAME
        ori_c.link_name = EEF_LINK
        ori_c.orientation.x = PARK_ORI_X
        ori_c.orientation.y = PARK_ORI_Y
        ori_c.orientation.z = PARK_ORI_Z
        ori_c.orientation.w = PARK_ORI_W
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

    def _move_to_park(self, color: str) -> None:
        goal = self._build_park_goal()
        future = self._move_client.send_goal_async(goal)
        future.add_done_callback(lambda f, c=color: self._park_goal_response_cb(f, c))

    def _park_goal_response_cb(self, future, color: str) -> None:
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('Park move goal rejected -- aborting before detection.')
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda f, c=color: self._park_result_cb(f, c))

    def _park_result_cb(self, future, color: str) -> None:
        result = future.result()
        ec = result.result.error_code.val
        if ec != 1:
            self.get_logger().error(
                f'Park move failed (error_code={ec}) -- aborting before detection. '
                f'Refusing to detect from an unverified pose.')
            return

        self.get_logger().info('At parked observation pose -- requesting detection...')
        self._request_detection(color)

    # ── Detection + task dispatch (unchanged logic, just now called after ───
    # ── the park move confirms success) ──────────────────────────────────

    def _request_detection(self, color: str) -> None:
        req = DetectObject.Request()
        req.color_name = color
        future = self._detect_client.call_async(req)
        future.add_done_callback(lambda f, c=color: self._on_detected(f, c))

    def _on_detected(self, future, color: str) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'Detection service call exception: {exc}')
            return

        if not result.found:
            self.get_logger().warn(f'No "{color}" object currently detected -- aborting.')
            return

        self.get_logger().info(
            f'Detected "{color}" at x={result.x:.4f}m y={result.y:.4f}m')

        pick_xyz = (result.x, result.y, PICK_Z_M)
        place_xyz = (HANDOFF_X_M, HANDOFF_Y_M, HANDOFF_Z_M)

        task_req = TaskPickPlace.Request()
        task_req.object_id = f'{color}_cube'
        task_req.pick_pose = self._make_pose(pick_xyz)
        task_req.place_pose = self._make_pose(place_xyz)

        self.get_logger().info(f'Dispatching {color.upper()}_CUBE -> handoff')
        future = self._task_client.call_async(task_req)
        future.add_done_callback(lambda f, c=color: self._on_task_done(f, c))

    def _make_pose(self, xyz: tuple) -> Pose:
        p = Pose()
        p.position.x, p.position.y, p.position.z = xyz
        p.orientation.x = ORI_X
        p.orientation.y = ORI_Y
        p.orientation.z = ORI_Z
        p.orientation.w = ORI_W
        return p

    def _on_task_done(self, future, color: str) -> None:
        try:
            result = future.result()
            if result.success:
                self.get_logger().info(f'✓ {color.upper()}_CUBE delivered')
            else:
                self.get_logger().error(f'✗ {color.upper()}_CUBE failed: {result.message}')
        except Exception as exc:
            self.get_logger().error(f'Task service call exception: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceCoordinator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()