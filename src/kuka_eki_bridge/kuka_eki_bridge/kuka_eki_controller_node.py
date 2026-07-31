"""
kuka_eki_controller_node.py

The single authoritative interface between MoveIt and the real KRC4, given
this cell has no RSI technology package licensed (ruling out kuka_rsi_driver
entirely -- rsi_only/eki_rsi/mxa_rsi all require the RSI channel for cyclic
control, EKI alone only handles handshaking in that stack). This node stands
in for a ros2_control hardware interface: it is the FollowJointTrajectory
action server MoveGroup's trajectory execution manager talks to directly
(see eki_controllers.yaml), and it is the only thing that talks to the
KRC4, over the existing EKI motion + state sockets.

There is no ros2_control, no controller_manager, and no mock/fake hardware
in this path. /joint_states here is the REAL robot's state, not a loopback
simulation -- robot_state_publisher/RViz/TF downstream of this node reflect
the actual cell.

Replaces (deleted): bridge_node.py, gripper_bridge.py.

CHANGES (2026-07-30):
  goal_handle.succeed() used to fire purely off summed time.sleep(dt) from
  the PLANNED trajectory timing -- nothing here ever checked the real KRC4
  state before declaring a trajectory done. That let control_server fire
  gripper/retract commands while the real arm was still mid-move.
  Fix v1: block on real /joint_states-sourced arrival for the FINAL
  waypoint only, via _wait_until_arrived(), instead of trusting planned dt.

CHANGES (2026-07-30, later same day):
  Fix v1 above exposed a SEPARATE, pre-existing structural issue rather
  than causing a new one: MoveGroup's own trajectory_execution_manager
  watchdog (execution_duration_monitoring) independently cancels a goal
  if it runs past the PLANNED duration (scaled by a margin) -- it was
  written assuming the controller reports completion promptly and
  honestly. Real KRC4 execution is running well behind Pilz's planned
  timing (see 2026-07-30 field data), so MoveGroup's watchdog now fires
  and cancels the goal before our real-arrival wait can succeed, on
  nearly every non-trivial trajectory.

  _wait_until_arrived() was also blocking through that cancel signal --
  it never checked self._preempted while polling, so a client-side cancel
  was silently ignored for the full ARRIVAL_TIMEOUT_SEC, and this code
  then called goal_handle.abort() on a goal the client had already given
  up on. Fixed: the wait loop now checks self._preempted every iteration
  and returns 'preempted' immediately, so the goal is properly resolved
  with goal_handle.canceled() instead of a late, invalid abort().

  This does NOT fix MoveGroup's watchdog itself -- that is a separate,
  required change on the moveit_config / launch side (raise or disable
  trajectory_execution.execution_duration_monitoring /
  allowed_execution_duration_scaling / allowed_goal_duration_margin),
  since MoveGroup's timing assumption is fundamentally miscalibrated
  against this bridge's real (non-ros2_control) execution characteristics.
  See eki_moveit_planning.launch.py for where to add the override.

  Also holding off on checking EVERY kept waypoint against real state
  (not just the final one) until the watchdog issue above is fixed --
  extending real-waits to every waypoint would only make MoveGroup's
  watchdog fire earlier and more often, not less.
"""

import math
import sys
import threading
import time
import tty
import termios

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8

from kuka_eki.eki import EkiMotionClient, EkiStateClient
from kuka_eki.krl import Axis

KUKA_IP = "192.168.1.147"

GRIPPER_CMD_TOPIC = '/gripper_cmd'          # Int8: 1 = ON, 0 = OFF
JOINT_STATE_TOPIC = '/joint_states'
ACTION_NAME = 'eki_arm_controller/follow_joint_trajectory'  # moveit_simple_controller_manager
                                                             # builds the full action name as
                                                             # <controller_name>/<action_ns> --
                                                             # must match eki_controllers.yaml's
                                                             # eki_arm_controller + action_ns pair.

JOINT_ORDER = [f'joint_{i}' for i in range(1, 7)]  # must match the URDF's joint names

MIN_STEP_DEG = 2.0    # skip a waypoint unless it's moved at least this far
                       # from the last one actually sent -- each ptp() is a
                       # discrete exact-stop move on the KRC4 (no blending),
                       # so forwarding every fine-grained interpolated point
                       # from AddTimeOptimalParameterization causes a visible
                       # stop-start stutter ("brr brr brr"). Raise this if it
                       # still stutters; lower it if path fidelity matters
                       # more than smoothness for a given move. The real fix
                       # is continuous-path blending (C_PTP/C_DIS) on the KRL
                       # side, if that program is ever open for editing.

ARRIVAL_TOLERANCE_DEG = 0.5   # per-axis max error vs commanded target to call
                               # the FINAL waypoint "arrived". Intermediate
                               # waypoints are not checked this way -- only
                               # the goal, since that's what downstream nodes
                               # (vision, gripper) act on. Deliberately NOT
                               # extended to every waypoint yet -- see module
                               # docstring, that would worsen the MoveGroup
                               # watchdog race until that's fixed separately.
ARRIVAL_TIMEOUT_SEC = 120.0     # if the real robot hasn't converged within this
                               # long after the last ptp() was sent, abort
                               # rather than silently succeeding on a stale
                               # timing assumption. In practice MoveGroup's
                               # own watchdog (see module docstring) will
                               # usually cancel before this fires unless that
                               # watchdog is also relaxed on the launch side.
ARRIVAL_POLL_SEC = 0.02


def build_gripper_packet(state: int) -> bytes:
    """Type=0 -> no motion in the KRL switch, but $OUT[1] still gets set from Gripper."""
    return (
        b'<RobotCommand>'
        b'<Type>0</Type>'
        b'<Axis A1="0" A2="0" A3="0" A4="0" A5="0" A6="0"/>'
        b'<Cart X="0" Y="0" Z="0" A="0" B="0" C="0"/>'
        b'<Velocity>0.1</Velocity>'
        b'<Gripper>' + str(state).encode() + b'</Gripper>'
        b'</RobotCommand>'
    )


class KukaEkiControllerNode(Node):
    def __init__(self):
        super().__init__('kuka_eki_controller_node')

        self.get_logger().info(f"Connecting to KUKA at {KUKA_IP} (motion + state)...")
        self.motion_client = EkiMotionClient(KUKA_IP)
        self.motion_client.connect()
        self._eki_lock = threading.Lock()  # motion socket also carries gripper packets

        self.state_client = EkiStateClient(KUKA_IP)
        self.state_client.connect()
        self.get_logger().info("--- EKI CONTROLLER CONNECTED (motion + state) ---")

        # ---- Real joint state feedback ----
        # state_client.state() blocks on recv() -- it's paced by whatever
        # cycle the KRC4's state channel actually pushes on, not something to
        # poll on a ROS timer (a blocking recv() inside a timer callback is a
        # bad pattern regardless of executor threading). Runs as its own loop.
        self._joint_state_pub = self.create_publisher(JointState, JOINT_STATE_TOPIC, 10)

        # Latest REAL joint angles in degrees, as reported by the KRC4 state
        # socket -- NOT the planned/commanded angles. _execute_trajectory
        # polls this to confirm physical arrival instead of trusting planned
        # segment timing. Written only by _state_loop, read only through
        # _get_latest_positions_deg().
        self._latest_state_lock = threading.Lock()
        self._latest_positions_deg = None

        self._state_thread = threading.Thread(target=self._state_loop, daemon=True)
        self._state_thread.start()

        # ---- Motion: FollowJointTrajectory action server (the "controller") ----
        self._cb_group = ReentrantCallbackGroup()
        self._preempted = False
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            ACTION_NAME,
            execute_callback=self._execute_trajectory,
            goal_callback=self._handle_goal,
            cancel_callback=self._handle_cancel,
            callback_group=self._cb_group,
        )
        self.get_logger().info(f"FollowJointTrajectory action server up on '{ACTION_NAME}'")

        # ---- Gripper ----
        self._gripper_state = 0
        self._gripper_lock = threading.Lock()
        self.create_subscription(Int8, GRIPPER_CMD_TOPIC, self._gripper_cmd_callback, 10)
        self.get_logger().info(f"Listening on {GRIPPER_CMD_TOPIC} for gripper commands ...")

        self._kb_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._kb_thread.start()
        self.get_logger().info("Gripper ready — SPACE to toggle manually, or via /gripper_cmd")

    # ── Real state feedback ──────────────────────────────────────────────────

    def _state_loop(self):
        while rclpy.ok():
            try:
                state = self.state_client.state()  # blocks until the KRC4 sends one
            except Exception as e:
                self.get_logger().warn(f"State read failed: {e}", throttle_duration_sec=2.0)
                time.sleep(0.5)  # avoid a hot loop if the socket is down
                continue

            # RobotState.from_xml (kuka_eki/krl.py) passes raw XML attribute
            # strings into Axis's constructor without casting -- Axis.a1..a6
            # are typed float but arrive as str at runtime (dataclasses don't
            # enforce their type hints). Cast explicitly here rather than
            # patching their library.
            try:
                axis = state.axis
                angles_deg = [float(getattr(axis, f'a{i}')) for i in range(1, 7)]
            except (AttributeError, ValueError) as e:
                self.get_logger().error(
                    f"RobotState field mismatch, update _state_loop: {e}", throttle_duration_sec=5.0)
                continue

            with self._latest_state_lock:
                self._latest_positions_deg = angles_deg

            positions_rad = [math.radians(a) for a in angles_deg]
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = JOINT_ORDER
            msg.position = positions_rad
            self._joint_state_pub.publish(msg)

    def _get_latest_positions_deg(self):
        with self._latest_state_lock:
            return None if self._latest_positions_deg is None else list(self._latest_positions_deg)

    def _wait_until_arrived(self, target_angles_deg,
                             tolerance_deg=ARRIVAL_TOLERANCE_DEG,
                             timeout_sec=ARRIVAL_TIMEOUT_SEC):
        """Block until the REAL robot (via the EKI state socket, not planned
        trajectory timing) is within tolerance_deg of target on every axis.
        Returns 'arrived', 'preempted' (client canceled -- e.g. MoveGroup's
        own trajectory_execution_manager watchdog gave up), or 'timeout'.

        MUST check self._preempted on every iteration -- without this, a
        cancel from the client (_handle_cancel sets _preempted) is silently
        ignored for the full timeout_sec, and the caller would then call
        goal_handle.abort() on a goal the client already gave up on."""
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self._preempted:
                return 'preempted'
            current = self._get_latest_positions_deg()
            if current is not None:
                max_err = max(abs(c - t) for c, t in zip(current, target_angles_deg))
                if max_err <= tolerance_deg:
                    return 'arrived'
            time.sleep(ARRIVAL_POLL_SEC)
        return 'timeout'

    # ── Gripper ──────────────────────────────────────────────────────────────

    def _send_gripper(self, state: int, source: str = ''):
        try:
            with self._eki_lock:
                self.motion_client._tcp_client.sendall(build_gripper_packet(state))
            label = "ON  (pick)" if state else "OFF (place)"
            self.get_logger().info(f"Gripper {label}{f'  [{source}]' if source else ''}")
        except Exception as e:
            self.get_logger().error(f"Gripper send failed: {e}")

    def _set_gripper(self, state: int, source: str = ''):
        with self._gripper_lock:
            if self._gripper_state != state:
                self._gripper_state = state
                self._send_gripper(state, source)

    def _toggle_gripper(self):
        with self._gripper_lock:
            self._gripper_state ^= 1
            self._send_gripper(self._gripper_state, 'keyboard')

    def _gripper_cmd_callback(self, msg: Int8):
        state = int(msg.data)
        if state not in (0, 1):
            self.get_logger().warn(f"Invalid gripper_cmd value: {state} (expected 0 or 1)")
            return
        self._set_gripper(state, 'control_server')

    def _keyboard_loop(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while True:
                ch = sys.stdin.read(1)
                if ch == ' ':
                    self._toggle_gripper()
                elif ch in ('q', 'Q', '\x03'):
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    # ── Trajectory execution (the actual "controller" behaviour) ────────────

    def _handle_goal(self, goal_request):
        if not goal_request.trajectory.points:
            self.get_logger().warn("Goal rejected: empty trajectory.")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _handle_cancel(self, goal_handle):
        self._preempted = True
        return CancelResponse.ACCEPT

    def _joint_index_map(self, joint_names):
        idx = {}
        for i, name in enumerate(joint_names):
            for axis_n, canonical in enumerate(JOINT_ORDER, start=1):
                if canonical in name:
                    idx[axis_n] = i
                    break
        return idx

    def _execute_trajectory(self, goal_handle):
        traj = goal_handle.request.trajectory
        idx = self._joint_index_map(traj.joint_names)
        if len(idx) != 6:
            self.get_logger().error(f"Could not map all 6 joints from {traj.joint_names}")
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            return result

        self._preempted = False
        points = traj.points
        self.get_logger().info(f"Trajectory has {len(points)} waypoints from the planner")

        # Thin: each ptp() below is a discrete exact-stop move on the KRC4,
        # so forwarding every finely-interpolated point causes a stop-start
        # stutter. Keep a point only if it's moved MIN_STEP_DEG from the last
        # one we're keeping -- except the last point, which is always kept
        # so the actual goal target is never dropped.
        kept = []
        last_kept_deg = None
        for i, point in enumerate(points):
            angles_deg = [math.degrees(point.positions[idx[a]]) for a in range(1, 7)]
            is_last = (i == len(points) - 1)
            if last_kept_deg is None or is_last:
                kept.append((i, point, angles_deg))
                last_kept_deg = angles_deg
                continue
            max_delta = max(abs(a - b) for a, b in zip(angles_deg, last_kept_deg))
            if max_delta >= MIN_STEP_DEG:
                kept.append((i, point, angles_deg))
                last_kept_deg = angles_deg

        self.get_logger().info(
            f"Executing {len(kept)}/{len(points)} waypoints after thinning "
            f"(MIN_STEP_DEG={MIN_STEP_DEG})")

        prev_t = 0.0
        for k, (i, point, angles_deg) in enumerate(kept):
            if self._preempted:
                self.get_logger().info("Trajectory preempted.")
                goal_handle.canceled()
                return FollowJointTrajectory.Result()

            target = Axis(
                a1=angles_deg[0], a2=angles_deg[1], a3=angles_deg[2],
                a4=angles_deg[3], a5=angles_deg[4], a6=angles_deg[5],
            )

            # NOTE: always sent as a joint-space ptp(). FollowJointTrajectory
            # only carries joint positions per point -- whether Pilz planned
            # this segment as LIN or PTP is known at the MoveIt/Pilz planning
            # layer, not recoverable here.
            try:
                with self._eki_lock:
                    self.motion_client.ptp(target, max_velocity_scaling=0.1)
            except Exception as e:
                self.get_logger().error(f"Transmission failed at waypoint {i}: {e}")
                goal_handle.abort()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                return result

            is_last = (k == len(kept) - 1)
            if is_last:
                # Don't trust planned timing for the waypoint everything
                # downstream (gripper, vision) depends on. Block on the REAL
                # EKI state socket until the robot has actually converged,
                # honoring cancellation the whole time (see _wait_until_arrived
                # docstring for why that matters).
                status = self._wait_until_arrived(angles_deg)
                if status == 'preempted':
                    self.get_logger().info(
                        "Trajectory canceled by client while waiting for final "
                        "arrival (likely MoveGroup's execution-duration watchdog "
                        "-- see module docstring).")
                    goal_handle.canceled()
                    return FollowJointTrajectory.Result()
                if status == 'timeout':
                    current = self._get_latest_positions_deg()
                    self.get_logger().error(
                        f"Robot did not reach final waypoint within "
                        f"{ARRIVAL_TIMEOUT_SEC}s (tolerance={ARRIVAL_TOLERANCE_DEG} deg). "
                        f"target={['%.2f' % a for a in angles_deg]} "
                        f"last_seen={['%.2f' % a for a in current] if current else 'unknown'}. "
                        f"Aborting -- NOT safe for downstream (gripper/vision) to proceed.")
                    goal_handle.abort()
                    result = FollowJointTrajectory.Result()
                    result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                    return result
                # status == 'arrived' -> fall through to feedback + succeed()
            else:
                t = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
                dt = max(0.0, t - prev_t)
                prev_t = t
                time.sleep(dt)

            fb = FollowJointTrajectory.Feedback()
            fb.desired = point
            goal_handle.publish_feedback(fb)

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result


def main(args=None):
    rclpy.init(args=args)
    node = KukaEkiControllerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down controller.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()