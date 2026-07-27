#!/usr/bin/env python3
"""
pick_place_coordinator.py

Replaces vision_logic_mock.py. Same role in the pipeline (listens to
/voice_command, dispatches to /execute_task), now supporting TWO detection
paths that can run side by side without conflict, since they're on
different services:

  - COLOR path  -> /detect_object       (surgical_msgs/srv/DetectObject)
                   served by vision_node.py
  - SCREW path  -> /detect_object_yolo  (surgical_msgs/srv/DetectObjectYolo)
                   served by vision_detect_node.py

A voice command is routed based on which set it matches: KNOWN_COLORS or
KNOWN_SCREW_CLASSES. Everything downstream (park -> detect -> pick/place ->
loop -> recovery) is identical for both paths; only which detect client
gets called, and how the object label is derived, differs.

STATE MACHINE:
When commanded to fetch a target (color or screw class), it locks, moves
to park, detects the object, picks it up, places it, and loops back to
park to check for MORE objects of the same target. It stops and releases
the lock when either:
  - vision confirms 0 remaining objects of that target, or
  - a task fails repeatedly at the same detected location (see RECOVERY
    below) -- most commonly a persistent false detection (reflection,
    glare, or the robot's own housing) sitting in the unreachable zone
    near the base, which would otherwise block picking real objects
    forever since both detection paths return a single best/largest hit.

RECOVERY:
  - If a task fails, we record the failed (x, y). If the NEXT detection
    for this target lands within FAILED_POSITION_RADIUS_M of a position
    that already failed, we skip it immediately without re-attempting
    the motion -- it's almost certainly the same persistent false
    detection, not a new object.
  - If MAX_CONSECUTIVE_FAILURES distinct failures happen in a row for
    the same target (without an intervening success), we give up on
    that target for this call rather than retrying indefinitely.
  - Any success resets both the failure count and the failed-position
    history for that target, since a working pick/place cycle means
    we're not stuck on a bad spot anymore.

NOTE on KNOWN_SCREW_CLASSES: imported from hardware_database.py, the same
dependency-free module vision_detect_node.py's SCREW_DATABASE lives in.
Both nodes stay in sync automatically -- no hand-duplicated lists here.

PARK MOVE (2026-07-XX): back to Cartesian, NOT joint-space. The prior
joint-space PARK_JOINTS_RAD approach was dropped when the park joints were
re-jogged on the SmartPad -- only the fresh Cartesian readout (position +
orientation) was saved this time, not a re-derived joint solution, so
there is no PARK_JOINTS_RAD to import anymore. pick_place_constants.py now
carries PARK_X_M/PARK_Y_M/PARK_Z_M + PARK_ORI_X/Y/Z/W straight from the
SmartPad "Actual position" readout (2026-07-09 ground truth), and the park
goal is built the same way as every other motion in this pipeline: a
Cartesian position + orientation constraint through MoveIt. This still
reproduces the exact physical pose the homography was solved at -- it's
just doing it via Cartesian target + IK instead of a fixed joint solution.
"""

import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.action import MoveGroup as MoveGroupAction
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint,
)
from surgical_msgs.srv import TaskPickPlace, DetectObject, DetectObjectYolo

from kuka_ros2_demo.pick_place_constants import (
    ORI_X, ORI_Y, ORI_Z, ORI_W,
    HANDOFF_X_M, HANDOFF_Y_M, HANDOFF_Z_M,
    PARK_X_M, PARK_Y_M, PARK_Z_M,
    PARK_ORI_X, PARK_ORI_Y, PARK_ORI_Z, PARK_ORI_W,
    ORIENTATION_TOLERANCE_RAD,
    get_pick_z_for_object,
)

from kuka_ros2_demo.hardware_database import KNOWN_COLORS, KNOWN_SCREW_CLASSES

PLANNING_GROUP = "manipulator"
BASE_FRAME_ID = "base_link"
EE_LINK = "tool0"

# ── Park move tuning ────────────────────────────────────────────────────────
# Tight position tolerance -- the homography is only valid at this exact
# pose, so we want the planner landing close to the SmartPad ground truth,
# not just "close enough" for a generic Cartesian move.
PARK_POSITION_TOLERANCE_M = 0.003

# ── Recovery tuning ────────────────────────────────────────────────────────
FAILED_POSITION_RADIUS_M = 0.03   # 3cm -- close enough to call it "the same spot"
MAX_CONSECUTIVE_FAILURES = 3      # give up on this target after this many in a row


class PickPlaceCoordinator(Node):

    def __init__(self):
        super().__init__('pick_place_coordinator')
        self.cb = ReentrantCallbackGroup()

        self._busy = False
        self._busy_lock = threading.Lock()

        # Recovery state, keyed by target (color OR screw class) -- reset on
        # success, checked on failure
        self._failed_positions = {}      # target -> list of (x, y) that failed
        self._consecutive_failures = {}  # target -> int

        self._task_client = self.create_client(
            TaskPickPlace, '/execute_task', callback_group=self.cb)
        self._detect_client = self.create_client(
            DetectObject, '/detect_object', callback_group=self.cb)
        self._detect_yolo_client = self.create_client(
            DetectObjectYolo, '/detect_object_yolo', callback_group=self.cb)
        self._move_client = ActionClient(
            self, MoveGroupAction, '/move_action', callback_group=self.cb)

        self.create_subscription(
            String, '/voice_command', self._voice_callback, 10)

        self.get_logger().info(
            'Pick-Place Coordinator online. '
            f'Known colors: {sorted(KNOWN_COLORS)} | '
            f'Known screw classes: {sorted(KNOWN_SCREW_CLASSES)}')

    def _voice_callback(self, msg: String) -> None:
        target = msg.data.strip().lower()

        if target in KNOWN_COLORS:
            mode = 'color'
        elif target in KNOWN_SCREW_CLASSES:
            mode = 'yolo'
        else:
            self.get_logger().warn(
                f'Unknown target: "{target}". '
                f'Valid colors: {sorted(KNOWN_COLORS)}, '
                f'valid screw classes: {sorted(KNOWN_SCREW_CLASSES)}')
            return

        detect_client = self._detect_client if mode == 'color' else self._detect_yolo_client
        detect_service_name = '/detect_object' if mode == 'color' else '/detect_object_yolo'
        detect_node_hint = 'vision_node' if mode == 'color' else 'vision_detect_node'

        if not detect_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(
                f'{detect_service_name} service not available -- is {detect_node_hint} running?')
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
                self.get_logger().warn(f'Arm is busy. Ignoring command "{target}".')
                return
            self._busy = True

        # Fresh recovery state for this target at the start of a new command
        self._failed_positions[target] = []
        self._consecutive_failures[target] = 0

        self.get_logger().info(
            f'"{target}" requested ({mode} path) -- moving to parked '
            f'observation pose before detecting...')
        self._move_to_park(target, mode)

    # ── Park move (must happen before every detection -- the homography is ──
    # ── only valid at this exact pose) ────────────────────────────────────

    def _build_park_goal(self):
        """Cartesian park target -- position + orientation constraints from
        the SmartPad ground-truth readout. See module docstring for why this
        is Cartesian again rather than joint-space."""
        goal = MoveGroupAction.Goal()
        req = MotionPlanRequest()
        req.group_name = PLANNING_GROUP
        req.pipeline_id = "pilz_industrial_motion_planner"
        req.planner_id = "PTP"
        req.num_planning_attempts = 5
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = 0.05
        req.max_acceleration_scaling_factor = 0.05

        target_pose = PoseStamped()
        target_pose.header.frame_id = BASE_FRAME_ID
        target_pose.pose.position.x = PARK_X_M
        target_pose.pose.position.y = PARK_Y_M
        target_pose.pose.position.z = PARK_Z_M
        target_pose.pose.orientation.x = PARK_ORI_X
        target_pose.pose.orientation.y = PARK_ORI_Y
        target_pose.pose.orientation.z = PARK_ORI_Z
        target_pose.pose.orientation.w = PARK_ORI_W

        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = BASE_FRAME_ID
        pos_constraint.link_name = EE_LINK
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0
        region = SolidPrimitive()
        region.type = SolidPrimitive.SPHERE
        region.dimensions = [PARK_POSITION_TOLERANCE_M]
        pos_constraint.constraint_region.primitives.append(region)
        pos_constraint.constraint_region.primitive_poses.append(target_pose.pose)
        pos_constraint.weight = 1.0

        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = BASE_FRAME_ID
        ori_constraint.link_name = EE_LINK
        ori_constraint.orientation = target_pose.pose.orientation
        ori_constraint.absolute_x_axis_tolerance = ORIENTATION_TOLERANCE_RAD
        ori_constraint.absolute_y_axis_tolerance = ORIENTATION_TOLERANCE_RAD
        ori_constraint.absolute_z_axis_tolerance = ORIENTATION_TOLERANCE_RAD
        ori_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(ori_constraint)

        req.goal_constraints.append(constraints)
        goal.request = req
        goal.planning_options.plan_only = False
        goal.planning_options.replan = False
        goal.planning_options.look_around = False
        return goal

    def _move_to_park(self, target: str, mode: str) -> None:
        goal = self._build_park_goal()
        future = self._move_client.send_goal_async(goal)
        future.add_done_callback(
            lambda f, t=target, m=mode: self._park_goal_response_cb(f, t, m))

    def _park_goal_response_cb(self, future, target: str, mode: str) -> None:
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('Park move goal rejected -- aborting before detection.')
            self._release(target)
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda f, t=target, m=mode: self._park_result_cb(f, t, m))

    def _park_result_cb(self, future, target: str, mode: str) -> None:
        result = future.result()
        ec = result.result.error_code.val
        if ec != 1:
            self.get_logger().error(
                f'Park move failed (error_code={ec}) -- aborting before detection. '
                f'Refusing to detect from an unverified pose.')
            self._release(target)
            return

        self.get_logger().info('At parked observation pose -- requesting detection...')
        self._request_detection(target, mode)

    # ── Detection + task dispatch ──────────────────────────────────────

    def _request_detection(self, target: str, mode: str) -> None:
        if mode == 'color':
            req = DetectObject.Request()
            req.color_name = target
            future = self._detect_client.call_async(req)
        else:
            req = DetectObjectYolo.Request()
            req.target_class = target
            future = self._detect_yolo_client.call_async(req)

        future.add_done_callback(
            lambda f, t=target, m=mode: self._on_detected(f, t, m))

    def _matches_failed_position(self, target: str, x: float, y: float) -> bool:
        for fx, fy in self._failed_positions.get(target, []):
            if math.hypot(x - fx, y - fy) <= FAILED_POSITION_RADIUS_M:
                return True
        return False

    def _on_detected(self, future, target: str, mode: str) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'Detection service call exception: {exc}')
            self._release(target)
            return

        if not result.found:
            self.get_logger().info(f'No (more) "{target}" objects detected. Task sequence complete!')
            self._release(target)
            return

        x, y = result.x, result.y

        # RECOVERY: skip immediately if this is the same spot that already
        # failed -- almost certainly a persistent false detection (reflection,
        # glare, robot's own housing), not a new object. Don't waste a motion
        # attempt re-proving what we already know.
        if self._matches_failed_position(target, x, y):
            self.get_logger().warn(
                f'Detected "{target}" at x={x:.4f}m y={y:.4f}m -- matches a '
                f'previously-failed position (within {FAILED_POSITION_RADIUS_M*100:.0f}cm). '
                f'Skipping, giving up on this target for now.')
            self._release(target)
            return

        # For the YOLO path, the resolved class can differ from the
        # requested target_class filter (disambiguation may correct it) --
        # use what vision_detect_node actually decided for the object_id and
        # logging, falling back to the requested target if it's blank.
        if mode == 'yolo':
            object_label = result.object_class or target
            self.get_logger().info(
                f'Detected "{target}" -> resolved as "{object_label}" '
                f'(confidence={result.confidence:.2f}) at x={x:.4f}m y={y:.4f}m')
        else:
            object_label = f'{target}_cube'
            self.get_logger().info(f'Detected "{target}" at x={x:.4f}m y={y:.4f}m')

        pick_z = get_pick_z_for_object(object_label)
        pick_xyz = (x, y, pick_z)
        place_xyz = (HANDOFF_X_M, HANDOFF_Y_M, HANDOFF_Z_M)

        task_req = TaskPickPlace.Request()
        task_req.object_id = object_label
        task_req.pick_pose = self._make_pose(pick_xyz)
        task_req.place_pose = self._make_pose(place_xyz)

        self.get_logger().info(f'Dispatching {object_label.upper()} -> handoff')
        future = self._task_client.call_async(task_req)
        future.add_done_callback(
            lambda f, t=target, m=mode, px=x, py=y, lbl=object_label:
                self._on_task_done(f, t, m, px, py, lbl))

    def _make_pose(self, xyz: tuple) -> Pose:
        p = Pose()
        p.position.x, p.position.y, p.position.z = xyz
        p.orientation.x = ORI_X
        p.orientation.y = ORI_Y
        p.orientation.z = ORI_Z
        p.orientation.w = ORI_W
        return p

    def _on_task_done(self, future, target: str, mode: str, px: float, py: float, label: str) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'Task service call exception: {exc}')
            self._release(target)
            return

        if result.success:
            self.get_logger().info(f'✓ {label.upper()} delivered. Checking for more...')
            # Success -- this target's recovery state resets, we're clearly
            # not stuck on a bad spot anymore.
            self._failed_positions[target] = []
            self._consecutive_failures[target] = 0
            self._move_to_park(target, mode)
            return

        # Failure -- record this position and count it toward the retry cap.
        self.get_logger().error(f'✗ {label.upper()} failed: {result.message}')
        self._failed_positions.setdefault(target, []).append((px, py))
        self._consecutive_failures[target] = self._consecutive_failures.get(target, 0) + 1

        if self._consecutive_failures[target] >= MAX_CONSECUTIVE_FAILURES:
            self.get_logger().error(
                f'{MAX_CONSECUTIVE_FAILURES} consecutive failures for "{target}" -- '
                f'giving up for now rather than retrying indefinitely.')
            self._release(target)
            return

        # Otherwise: could be a transient issue (lighting flicker, momentary
        # glare, a mid-air misclassification) -- worth trying again rather
        # than giving up on the first failure. Loop back through park -> detect.
        self.get_logger().warn(
            f'Retrying "{target}" ({self._consecutive_failures[target]}/{MAX_CONSECUTIVE_FAILURES} '
            f'consecutive failures so far)...')
        self._move_to_park(target, mode)

    def _release(self, target: str) -> None:
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