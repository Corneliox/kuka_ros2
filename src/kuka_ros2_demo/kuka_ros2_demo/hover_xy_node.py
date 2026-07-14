"""
hover_xy_node.py — Move end-effector to (X, Y) at fixed hover height Z=400mm.

Subscribes to /target_xy (geometry_msgs/Point, values in mm, robot base frame).
Plans and executes via MoveIt2 (Pilz PTP).

If bridge_node is running  → motion goes to real KRC4.
If bridge_node is NOT running → motion shows in RViz only (display_planned_path).

No gripper attached — A6 is empty.

Usage (simulation / RViz):
  ros2 run kuka_ros2_demo hover_xy_node

Send a target:
  ros2 topic pub --once /target_xy geometry_msgs/msg/Point \
      "{x: 400.0, y: 0.0, z: 0.0}"

  z field is ignored — hover height is fixed via ~hover_z_mm parameter:
    ros2 run kuka_ros2_demo hover_xy_node --ros-args -p hover_z_mm:=400.0
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, WorkspaceParameters,
    Constraints, PositionConstraint, OrientationConstraint,
    BoundingVolume, RobotState
)
from shape_msgs.msg import SolidPrimitive
from action_msgs.msg import GoalStatus

import rclpy.action
from moveit_msgs.action import MoveGroup as MoveGroupAction

# Use the simpler move_group python interface via pymoveit2 if available,
# otherwise fall back to raw MoveGroup action.
# This node uses the moveit_commander / pymoveit2 pattern consistent with
# the rest of the surgical_control_server.py stack.

try:
    from moveit.planning import MoveItPy
    from moveit.core.robot_state import RobotState as MoveItRobotState
    HAS_MOVEIT_PY = True
except ImportError:
    HAS_MOVEIT_PY = False

import threading
import math


HOVER_Z_DEFAULT_MM = 75.0

# Pre-calibrated end-effector orientation for downward-facing grasp
# (same quaternion used across surgical_control_server.py)
ORI_X = 0.0321
ORI_Y = 0.9235
ORI_Z = 0.0197
ORI_W = 0.3816

PLANNING_GROUP  = "manipulator"
PLANNING_FRAME  = "world"
EEF_LINK = "tool0"


class HoverXYNode(Node):

    def __init__(self):
        super().__init__('hover_xy_node')

        self.declare_parameter('hover_z_mm', HOVER_Z_DEFAULT_MM)
        self.declare_parameter('velocity_scaling',     0.1)
        self.declare_parameter('acceleration_scaling',  0.1)
        self.declare_parameter('planner_id', 'PTP')   # Pilz PTP

        self._lock = threading.Lock()
        self._busy = False

        # ── MoveIt via MoveGroup action ───────────────────────────────────────
        self._mg_client = rclpy.action.ActionClient(
            self, MoveGroupAction, 'move_action')

        self.get_logger().info("Waiting for move_group action server...")
        self._mg_client.wait_for_server()
        self.get_logger().info("move_group ready.")

        # ── Subscriber ────────────────────────────────────────────────────────
        self.create_subscription(Point, '/target_xy', self._target_cb, 10)

        hover_z = self.get_parameter('hover_z_mm').value
        self.get_logger().info(
            f"hover_xy_node ready — hover Z={hover_z:.0f}mm\n"
            f"  Publish target: ros2 topic pub --once /target_xy "
            f"geometry_msgs/msg/Point '{{x: 400.0, y: 0.0, z: 0.0}}'"
        )

    # ── Callback ──────────────────────────────────────────────────────────────

    def _target_cb(self, msg: Point):
        with self._lock:
            if self._busy:
                self.get_logger().warn("Motion in progress — ignoring new target.")
                return
            self._busy = True

        x_m = msg.x / 1000.0
        y_m = msg.y / 1000.0
        z_m = self.get_parameter('hover_z_mm').value / 1000.0

        self.get_logger().info(
            f"Target received: X={msg.x:.1f}mm  Y={msg.y:.1f}mm  "
            f"Z={z_m*1000:.0f}mm (fixed hover)"
        )

        goal = self._build_goal(x_m, y_m, z_m)

        future = self._mg_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("MoveGroup goal rejected.")
            with self._lock:
                self._busy = False
            return
        self.get_logger().info("Goal accepted — executing...")
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result().result
        status = future.result().status
        ec = result.error_code.val

        if ec == 1:   # SUCCESS
            self.get_logger().info("Motion complete.")
        else:
            self.get_logger().error(
                f"Motion failed — MoveIt error code: {ec}  status: {status}"
            )
        with self._lock:
            self._busy = False

    # ── Goal builder ──────────────────────────────────────────────────────────

    def _build_goal(self, x: float, y: float, z: float) -> MoveGroupAction.Goal:
        goal = MoveGroupAction.Goal()
        req  = MotionPlanRequest()

        req.group_name         = PLANNING_GROUP
        req.num_planning_attempts = 5
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor     = \
            self.get_parameter('velocity_scaling').value
        req.max_acceleration_scaling_factor = \
            self.get_parameter('acceleration_scaling').value
        req.planner_id = self.get_parameter('planner_id').value

        # ── Pose goal constraint ──────────────────────────────────────────────
        constraints = Constraints()

        # Position
        pos_c = PositionConstraint()
        pos_c.header.frame_id = PLANNING_FRAME
        pos_c.link_name       = EEF_LINK
        pos_c.target_point_offset.x = 0.0
        pos_c.target_point_offset.y = 0.0
        pos_c.target_point_offset.z = 0.0

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.001, 0.001, 0.001]   # 1mm tolerance box

        bv = BoundingVolume()
        bv.primitives.append(box)
        primitive_pose = Pose()
        primitive_pose.position.x = x
        primitive_pose.position.y = y
        primitive_pose.position.z = z
        primitive_pose.orientation.w = 1.0
        bv.primitive_poses.append(primitive_pose)

        pos_c.constraint_region = bv
        pos_c.weight = 1.0
        constraints.position_constraints.append(pos_c)

        # Orientation
        ori_c = OrientationConstraint()
        ori_c.header.frame_id  = PLANNING_FRAME
        ori_c.link_name        = EEF_LINK
        ori_c.orientation.x    = ORI_X
        ori_c.orientation.y    = ORI_Y
        ori_c.orientation.z    = ORI_Z
        ori_c.orientation.w    = ORI_W
        ori_c.absolute_x_axis_tolerance = 0.1
        ori_c.absolute_y_axis_tolerance = 0.1
        ori_c.absolute_z_axis_tolerance = 0.1
        ori_c.weight = 1.0
        constraints.orientation_constraints.append(ori_c)

        req.goal_constraints.append(constraints)

        goal.request          = req
        goal.planning_options.plan_only           = False
        goal.planning_options.replan              = False
        goal.planning_options.look_around         = False

        return goal


# ── Entrypoint ────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = HoverXYNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()