#!/usr/bin/env python3
"""
pick_place_coordinator.py

Replaces vision_logic_mock.py. Same role in the pipeline (listens to
/voice_command, dispatches to /execute_task), but pick coordinates now
come from a live /detect_object service call.

STATE MACHINE:
When commanded to pick a color, it locks, moves to park, detects the
object, picks it up, places it, and loops back to park to check for
MORE objects of the same color. It stops and releases the lock when
either:
  - vision confirms 0 remaining objects of that color, or
  - a task fails repeatedly at the same detected location (see RECOVERY
    below) -- most commonly a persistent false detection (reflection,
    glare, or the robot's own housing) sitting in the unreachable zone
    near the base, which would otherwise block picking real objects
    forever since vision_node always returns the single largest blob.

RECOVERY:
  - If a task fails, we record the failed (x, y). If the NEXT detection
    for this color lands within FAILED_POSITION_RADIUS_M of a position
    that already failed, we skip it immediately without re-attempting
    the motion -- it's almost certainly the same persistent false
    detection, not a new object.
  - If MAX_CONSECUTIVE_FAILURES distinct failures happen in a row for
    the same color (without an intervening success), we give up on that
    color for this call rather than retrying indefinitely.
  - Any success resets both the failure count and the failed-position
    history for that color, since a working pick/place cycle means
    we're not stuck on a bad spot anymore.
"""

import math
import threading
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

from kuka_ros2_demo.pick_place_constants import (
    ORI_X, ORI_Y, ORI_Z, ORI_W, ORIENTATION_TOLERANCE_RAD,
    PICK_Z_M, HANDOFF_X_M, HANDOFF_Y_M, HANDOFF_Z_M,
    PARK_X_M, PARK_Y_M, PARK_Z_M,
    PARK_ORI_X, PARK_ORI_Y, PARK_ORI_Z, PARK_ORI_W,
)

KNOWN_COLORS = {"red", "blue", "green", "yellow"}
PLANNING_GROUP = "manipulator"
PLANNING_FRAME = "world"
EEF_LINK = "tool0"

# ── Recovery tuning ────────────────────────────────────────────────────────
FAILED_POSITION_RADIUS_M = 0.03   # 3cm -- close enough to call it "the same spot"
MAX_CONSECUTIVE_FAILURES = 3      # give up on this color after this many in a row


class PickPlaceCoordinator(Node):

    def __init__(self):
        super().__init__('pick_place_coordinator')
        self.cb = ReentrantCallbackGroup()

        self._busy = False
        self._busy_lock = threading.Lock()

        # Recovery state, keyed by color -- reset on success, checked on failure
        self._failed_positions = {}   # color -> list of (x, y) that failed
        self._consecutive_failures = {}  # color -> int

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

        with self._busy_lock:
            if self._busy:
                self.get_logger().warn(f'Arm is busy. Ignoring command "{color}".')
                return
            self._busy = True

        # Fresh recovery state for this color at the start of a new command
        self._failed_positions[color] = []
        self._consecutive_failures[color] = 0

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
            self._release(color)
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
            self._release(color)
            return

        self.get_logger().info('At parked observation pose -- requesting detection...')
        self._request_detection(color)

    # ── Detection + task dispatch ──────────────────────────────────────

    def _request_detection(self, color: str) -> None:
        req = DetectObject.Request()
        req.color_name = color
        future = self._detect_client.call_async(req)
        future.add_done_callback(lambda f, c=color: self._on_detected(f, c))

    def _matches_failed_position(self, color: str, x: float, y: float) -> bool:
        for fx, fy in self._failed_positions.get(color, []):
            if math.hypot(x - fx, y - fy) <= FAILED_POSITION_RADIUS_M:
                return True
        return False

    def _on_detected(self, future, color: str) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'Detection service call exception: {exc}')
            self._release(color)
            return

        if not result.found:
            self.get_logger().info(f'No (more) "{color}" objects detected. Task sequence complete!')
            self._release(color)
            return

        x, y = result.x, result.y

        # RECOVERY: skip immediately if this is the same spot that already
        # failed -- almost certainly a persistent false detection (reflection,
        # glare, robot's own housing), not a new object. Don't waste a motion
        # attempt re-proving what we already know.
        if self._matches_failed_position(color, x, y):
            self.get_logger().warn(
                f'Detected "{color}" at x={x:.4f}m y={y:.4f}m -- matches a '
                f'previously-failed position (within {FAILED_POSITION_RADIUS_M*100:.0f}cm). '
                f'Skipping, giving up on this color for now.')
            self._release(color)
            return

        self.get_logger().info(f'Detected "{color}" at x={x:.4f}m y={y:.4f}m')

        pick_xyz = (x, y, PICK_Z_M)
        place_xyz = (HANDOFF_X_M, HANDOFF_Y_M, HANDOFF_Z_M)

        task_req = TaskPickPlace.Request()
        task_req.object_id = f'{color}_cube'
        task_req.pick_pose = self._make_pose(pick_xyz)
        task_req.place_pose = self._make_pose(place_xyz)

        self.get_logger().info(f'Dispatching {color.upper()}_CUBE -> handoff')
        future = self._task_client.call_async(task_req)
        future.add_done_callback(lambda f, c=color, px=x, py=y: self._on_task_done(f, c, px, py))

    def _make_pose(self, xyz: tuple) -> Pose:
        p = Pose()
        p.position.x, p.position.y, p.position.z = xyz
        p.orientation.x = ORI_X
        p.orientation.y = ORI_Y
        p.orientation.z = ORI_Z
        p.orientation.w = ORI_W
        return p

    def _on_task_done(self, future, color: str, px: float, py: float) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'Task service call exception: {exc}')
            self._release(color)
            return

        if result.success:
            self.get_logger().info(f'✓ {color.upper()}_CUBE delivered. Checking for more...')
            # Success -- this color's recovery state resets, we're clearly
            # not stuck on a bad spot anymore.
            self._failed_positions[color] = []
            self._consecutive_failures[color] = 0
            self._move_to_park(color)
            return

        # Failure -- record this position and count it toward the retry cap.
        self.get_logger().error(f'✗ {color.upper()}_CUBE failed: {result.message}')
        self._failed_positions.setdefault(color, []).append((px, py))
        self._consecutive_failures[color] = self._consecutive_failures.get(color, 0) + 1

        if self._consecutive_failures[color] >= MAX_CONSECUTIVE_FAILURES:
            self.get_logger().error(
                f'{MAX_CONSECUTIVE_FAILURES} consecutive failures for "{color}" -- '
                f'giving up for now rather than retrying indefinitely.')
            self._release(color)
            return

        # Otherwise: could be a transient issue (lighting flicker, momentary
        # glare) -- worth trying again rather than giving up on the first
        # failure. Loop back through park -> detect.
        self.get_logger().warn(
            f'Retrying "{color}" ({self._consecutive_failures[color]}/{MAX_CONSECUTIVE_FAILURES} '
            f'consecutive failures so far)...')
        self._move_to_park(color)

    def _release(self, color: str) -> None:
        with self._busy_lock:
            self._busy = False


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceCoordinator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()