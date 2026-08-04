#!/usr/bin/env python3
"""
control_server.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Exposes /execute_task (TaskPickPlace.srv).

CHANGES (2026-07-09):
  - ORI, ORIENTATION_TOLERANCE_RAD, and the PARK pose now import from
    kuka_ros2_demo.pick_place_constants instead of being hardcoded
    here separately -- this is the single source of truth shared with
    pick_place_coordinator.py and vision_node.py.
  - PARK is now the SAME pose used as the pre-detection observation
    pose (PARK_X_M/Y_M/Z_M/PARK_ORI_*), not a separate generic resting
    spot. Ending every task there means the arm is already correctly
    positioned for the next detection cycle -- no extra hop through
    two different "park" positions.
  - move_to() now accepts an optional `orientation` dict, defaulting to
    the pick/place gripper orientation (ORI). The final park step passes
    PARK_ORI explicitly, since the observation pose's orientation is
    genuinely different from the pick/place orientation.

CHANGES (2026-07-28):
  - Split the old step 3 ("PTP arc to above place") into 3a/3b: a LIN
    straight-up retract to transit_z at the PICK xy, followed by a PTP
    transit to above place -- both now happen at the same safe height.
    Previously a single PTP went directly from the low pick-contact
    pose to (dx, dy, transit_z). PTP interpolates in joint space, not
    Cartesian space, so it made no guarantee about the tool's path
    between those two points -- in practice this caused a low sweep
    toward +X that could clip the workspace/tray on the way up. Since
    the tool is now already at transit_z before any horizontal PTP
    crossing happens, there is nothing below it left to hit. Mirrors
    the retract-then-transit pattern already used on the place side
    (steps 5 -> "park").

Gripper architecture
────────────────────
  This node does NOT open an EKI socket. Instead it publishes
  gripper commands on /gripper_cmd (std_msgs/Int8: 1=ON, 0=OFF).
  gripper_bridge.py subscribes to that topic and fires the EKI
  packet through its own motion socket — the one that also sends
  robot motion commands. This avoids the EKI single-client-per-
  channel constraint that would otherwise block motion commands.

Z reference values (tool0, base_link frame, with gripper mounted)
──────────────────────────────────────────────────────────────────
  Fixed pick height (PICK_Z_M) now comes from pick_place_constants --
  all objects are uniform 2x2x2cm cubes, no per-object height lookup.
  APPROACH_CLEARANCE = 120 mm
  Z_SAFE       = +250 mm    (must be above approach height)

Collision strategy
──────────────────
  Only per-object collision boxes (if ever added) get attached/detached
  around the grasp. The table/tray surface is NOT added as a static
  collision object -- it previously caused link_5/6 collision during
  LIN descent and a remove/restore workaround caused an executor
  deadlock (blocking spin_until_future_complete inside async). The
  attach_object()/detach_object() sequence still functions correctly
  with no static world objects present.

Motion sequence (7 steps)
──────────────────────────
  1.  PTP to pick XY at transit_z
  2.  Disable tray collisions → LIN down to contact
  3a. Gripper ON → LIN straight up to transit_z at pick XY
  3b. PTP transit → above place (same safe height, no descent involved)
  4.  PTP down to place contact (see move_to() call for why -- not LIN)
  5.  Gripper OFF → LIN retract
  6.  PTP to PARK (== observation pose, PARK_ORI orientation)

Velocity
────────
  VEL_TRANSIT = 5%  (long PTP transits, park)
  VEL_NEAR    = 3%  (approach, contact, retract near tray)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

import rclpy
import threading
import time

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import Int8
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    CollisionObject, AttachedCollisionObject, PlanningScene,
    MotionPlanRequest, Constraints,
    PositionConstraint, OrientationConstraint, BoundingVolume,
)
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.action import MoveGroup
from surgical_msgs.srv import TaskPickPlace

from kuka_ros2_demo.pick_place_constants import (
    ORI_X, ORI_Y, ORI_Z, ORI_W, ORIENTATION_TOLERANCE_RAD,
    APPROACH_CLEARANCE_M, Z_SAFE_M,
    WS_X_MIN, WS_X_MAX, WS_Y_MIN, WS_Y_MAX, WS_Z_MIN, WS_Z_MAX,
    PARK_X_M, PARK_Y_M, PARK_Z_M,
    PARK_ORI_X, PARK_ORI_Y, PARK_ORI_Z, PARK_ORI_W,
    get_pick_z_for_object,
)

# ── Frames ────────────────────────────────────────────────────────────────────
FRAME = 'base_link'
TIP   = 'tool0'

GRIPPER_CMD_TOPIC = '/gripper_cmd'

# ── Orientation (imported from pick_place_constants -- single source of truth) ─
ORI = dict(qx=ORI_X, qy=ORI_Y, qz=ORI_Z, qw=ORI_W)
PARK_ORI = dict(qx=PARK_ORI_X, qy=PARK_ORI_Y, qz=PARK_ORI_Z, qw=PARK_ORI_W)

APPROACH_CLEARANCE = APPROACH_CLEARANCE_M
Z_SAFE = Z_SAFE_M

def approach_z(z_contact: float) -> float:
    return z_contact + APPROACH_CLEARANCE

# ── Workspace bounds (imported) ───────────────────────────────────────────────
# (WS_X_MIN etc. imported directly above)

# ── Park position == observation pose (imported) ─────────────────────────────
PARK_X, PARK_Y, PARK_Z = PARK_X_M, PARK_Y_M, PARK_Z_M

# ── Velocity scaling ──────────────────────────────────────────────────────────
VEL_TRANSIT = 0.5
VEL_NEAR    = 0.2


def _ros_sleep(node, seconds):
    end = node.get_clock().now().nanoseconds + int(seconds * 1e9)
    while node.get_clock().now().nanoseconds < end:
        time.sleep(0.01)


class ControlServer(Node):

    def __init__(self):
        super().__init__('control_server')
        self.cb = ReentrantCallbackGroup()

        self._move_client = ActionClient(
            self, MoveGroup, '/move_action', callback_group=self.cb)
        self._scene_client = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene')
        self.create_service(
            TaskPickPlace, '/execute_task',
            self.execute_task_callback, callback_group=self.cb)

        self._task_lock = threading.Lock()

        self._gripper_pub = self.create_publisher(Int8, GRIPPER_CMD_TOPIC, 10)
        self.get_logger().info(f'Publishing gripper commands on {GRIPPER_CMD_TOPIC}')

        self.get_logger().info('Waiting for MoveGroup action server...')
        self._move_client.wait_for_server()
        self.get_logger().info('Waiting for ApplyPlanningScene service...')
        self._scene_client.wait_for_service()
        self.get_logger().info('Control Server online.')

    # ── Gripper ───────────────────────────────────────────────────────────────

    def _send_gripper(self, state: int):
        msg = Int8()
        msg.data = state
        self._gripper_pub.publish(msg)
        label = "ON  (pick)" if state else "OFF (place)"
        self.get_logger().info(f'  [GRIPPER CMD → {label}]')

    # ── Bounds check ──────────────────────────────────────────────────────────

    def _in_bounds(self, x, y, z) -> bool:
        return (WS_X_MIN <= x <= WS_X_MAX and
                WS_Y_MIN <= y <= WS_Y_MAX and
                WS_Z_MIN <= z <= WS_Z_MAX)

    # ── Service callback ──────────────────────────────────────────────────────

    async def execute_task_callback(self, request, response):
        obj = request.object_id
        if not self._task_lock.acquire(blocking=False):
            msg = f'Arm busy — rejected task for "{obj}"'
            self.get_logger().warn(msg)
            response.success = False
            response.message = msg
            return response
        try:
            return await self._run_task(request, response)
        finally:
            self._task_lock.release()

    async def _run_task(self, request, response):
        obj = request.object_id
        self.get_logger().info(f'=== Task: {obj.upper()} ===')

        px = request.pick_pose.position.x
        py = request.pick_pose.position.y
        pz = request.pick_pose.position.z
        dx = request.place_pose.position.x
        dy = request.place_pose.position.y
        dz = request.place_pose.position.z

        if pz <= 0.0:
            requested_z = get_pick_z_for_object(obj)
            self.get_logger().info(
                f'Using object-specific pick height for {obj}: {requested_z:.5f} m')
            pz = requested_z

        # Only the PICK pose gets checked here. It comes from live vision output,
        # which can occasionally be wrong. The PLACE/handoff pose is a fixed,
        # already-proven-safe destination that is intentionally outside the
        # table-region bounds and should not be rejected.
        if not self._in_bounds(px, py, pz):
            return self._fail(response,
                f'pick pose ({px:.3f},{py:.3f},{pz:.3f}) outside workspace')

        pick_app  = pz + APPROACH_CLEARANCE
        place_app = dz + APPROACH_CLEARANCE
        transit_z = max(pick_app, place_app, Z_SAFE)

        # ── 1. Transit to above pick ──────────────────────────────────────────
        self.get_logger().info('  [1/7] Transit → above pick (PTP)')
        if not await self.move_to(px, py, transit_z, 'PTP', VEL_TRANSIT):
            return self._fail(response, 'Transit to pick column failed')

        # ── 2. LIN down to pick contact ───────────────────────────────────────
        self.get_logger().info('  [2/7] Pick → contact (LIN)')
        await self.remove_object(obj)   # remove before descent so no self-collision
        if not await self.move_to(px, py, pz, 'LIN', VEL_NEAR):
            return self._fail(response, 'Pick contact failed')

        self._send_gripper(1)
        await self.attach_object(obj)
        _ros_sleep(self, 0.5)

        # ── 3a. LIN straight up to safe height at pick XY ─────────────────────
        # PTP interpolates in joint space and gives no guarantee about the
        # Cartesian path between the low pick pose and a distant, higher
        # target -- that previously caused a low sweep toward +X that could
        # clip the workspace/tray. LIN's z increases monotonically, so this
        # leg can't dip below its start height.
        self.get_logger().info('  [3a/7] Retract straight up (LIN)')
        if not await self.move_to(px, py, transit_z, 'LIN', VEL_NEAR):
            return self._fail(response, 'Pick retract failed')

        # ── 3b. Transit → above place (PTP, at safe height only) ──────────────
        # Now that the tool is already at transit_z, a PTP crossing between
        # two points at the same height is safe -- there is nothing below it
        # left to hit.
        self.get_logger().info('  [3b/7] Transit → above place (PTP)')
        if not await self.move_to(dx, dy, transit_z, 'PTP', VEL_TRANSIT):
            return self._fail(response, 'Transit to place column failed')

        # ── 4. Down to place contact (PTP, not LIN -- see note below) ────────
        self.get_logger().info('  [4/7] Place → contact (PTP)')
        if not await self.move_to(dx, dy, dz, 'PTP', VEL_NEAR):
            return self._fail(response, 'Place contact failed')

        self._send_gripper(0)
        await self.detach_object(obj)
        _ros_sleep(self, 0.4)

        # ── 5. LIN retract ────────────────────────────────────────────────────
        self.get_logger().info('  [5/7] Place → retract (LIN)')
        if not await self.move_to(dx, dy, place_app, 'LIN', VEL_NEAR):
            return self._fail(response, 'Place retract failed')

        # ── 6. Park == observation pose (own orientation, NOT gripper ORI) ────
        self.get_logger().info('  [6/7] Parking at observation pose (PTP)')
        await self.move_to(PARK_X, PARK_Y, PARK_Z, 'PTP', VEL_TRANSIT,
                            orientation=PARK_ORI)

        self.get_logger().info(f'=== Complete: {obj.upper()} ===')
        response.success = True
        response.message = f'{obj} transferred OK'
        return response

    def _fail(self, response, msg):
        self.get_logger().error(f'  ABORTED: {msg}')
        self._send_gripper(0)
        response.success = False
        response.message = msg
        return response

    # ── Planning scene ────────────────────────────────────────────────────────

    def _apply_scene_sync(self, scene):
        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = self._scene_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def add_box(self, name, xyz_m, size_m):
        p = Pose()
        p.position.x, p.position.y, p.position.z = xyz_m
        p.orientation.w = 1.0
        co = CollisionObject()
        co.header.frame_id = FRAME
        co.id = name
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = list(size_m)
        co.primitives = [box]
        co.primitive_poses = [p]
        co.operation = CollisionObject.ADD
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [co]
        self._apply_scene_sync(scene)
        self.get_logger().info(f'  Scene: added "{name}"')

    async def remove_object(self, object_id):
        co = CollisionObject()
        co.header.frame_id = FRAME
        co.id = object_id
        co.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [co]
        req = ApplyPlanningScene.Request()
        req.scene = scene
        await self._scene_client.call_async(req)
        self.get_logger().info(f'  Scene: removed world object "{object_id}"')

    async def attach_object(self, object_id):
        aco = AttachedCollisionObject()
        aco.link_name = TIP
        aco.object.id = object_id
        aco.object.operation = CollisionObject.ADD
        aco.touch_links = [
            'tool0', 'gripper_gripper_base',
            'gripper_suction_cup', 'gripper_tcp',
        ]
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [aco]
        req = ApplyPlanningScene.Request()
        req.scene = scene
        await self._scene_client.call_async(req)
        self.get_logger().info(f'  Scene: "{object_id}" attached to {TIP}')

    async def detach_object(self, object_id):
        aco = AttachedCollisionObject()
        aco.object.id = object_id
        aco.object.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [aco]
        req = ApplyPlanningScene.Request()
        req.scene = scene
        await self._scene_client.call_async(req)
        self.get_logger().info(f'  Scene: "{object_id}" detached')

    def setup_scene(self):
        self.get_logger().info('=== Building scene ===')
        # No static collision objects added -- see module docstring.
        self.get_logger().info('=== Scene ready ===')

    # ── Motion ────────────────────────────────────────────────────────────────

    async def move_to(self, x, y, z, planner='PTP', vel=VEL_NEAR, orientation=None):
        """orientation: optional dict(qx,qy,qz,qw). Defaults to the gripper
        pick/place orientation (ORI). Pass PARK_ORI explicitly for the park
        step, since the observation pose's orientation genuinely differs."""
        ori_to_use = orientation if orientation is not None else ORI

        target = Pose()
        target.position.x = x
        target.position.y = y
        target.position.z = z
        target.orientation.x = ori_to_use['qx']
        target.orientation.y = ori_to_use['qy']
        target.orientation.z = ori_to_use['qz']
        target.orientation.w = ori_to_use['qw']

        req = MotionPlanRequest()
        req.group_name = 'manipulator'
        req.planner_id = planner
        req.pipeline_id = 'pilz_industrial_motion_planner'
        req.num_planning_attempts = 3
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = vel
        req.max_acceleration_scaling_factor = vel

        pos = PositionConstraint()
        pos.header.frame_id = FRAME
        pos.link_name = TIP
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.002]
        bv = BoundingVolume()
        bv.primitives = [sphere]
        bv.primitive_poses = [target]
        pos.constraint_region = bv
        pos.weight = 1.0

        ori = OrientationConstraint()
        ori.header.frame_id = FRAME
        ori.link_name = TIP
        ori.orientation = target.orientation
        ori.absolute_x_axis_tolerance = ORIENTATION_TOLERANCE_RAD
        ori.absolute_y_axis_tolerance = ORIENTATION_TOLERANCE_RAD
        ori.absolute_z_axis_tolerance = ORIENTATION_TOLERANCE_RAD
        ori.weight = 1.0

        goal_con = Constraints()
        goal_con.position_constraints = [pos]
        goal_con.orientation_constraints = [ori]
        req.goal_constraints = [goal_con]

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False
        goal.planning_options.replan = False

        self.get_logger().info(
            f'  [{planner} {int(vel*100)}%] → ({x:.4f}, {y:.4f}, {z:.4f}) m')
        goal_handle = await self._move_client.send_goal_async(goal)
        if not goal_handle.accepted:
            self.get_logger().error('  Goal REJECTED by MoveGroup')
            return False
        result_resp = await goal_handle.get_result_async()
        code = result_resp.result.error_code.val
        if code == 1:
            self.get_logger().info('  ✓ SUCCESS')
            return True
        self.get_logger().error(f'  ✗ FAILED (error_code={code})')
        if code == -31:
            self.get_logger().error(
                '    -31 = NO_IK_SOLUTION. Likely a genuine reachability limit '
                'at this (x,y) with the fixed orientation constraint -- see '
                'notes on workspace-edge picks near the base. Not necessarily '
                'a bug; consider whether this pick location is too close to '
                'the base/workspace boundary.')
        return False


def main(args=None):
    rclpy.init(args=args)
    node = ControlServer()
    node.setup_scene()
    time.sleep(1.0)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()