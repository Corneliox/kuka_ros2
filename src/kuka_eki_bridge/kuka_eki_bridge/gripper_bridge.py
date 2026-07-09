import rclpy
from rclpy.node import Node
from moveit_msgs.msg import DisplayTrajectory
from std_msgs.msg import Int8
import math
import threading
import tty
import termios
import sys
from kuka_eki.eki import EkiMotionClient
from kuka_eki.krl import Axis

GRIPPER_CMD_TOPIC = '/gripper_cmd'   # Int8: 1 = ON, 0 = OFF

def build_gripper_packet(state: int) -> bytes:
    """Type=0 → no motion case in KRL switch, but Gripper field is still read and $OUT[1] set."""
    return (
        b'<RobotCommand>'
        b'<Type>0</Type>'
        b'<Axis A1="0" A2="0" A3="0" A4="0" A5="0" A6="0"/>'
        b'<Cart X="0" Y="0" Z="0" A="0" B="0" C="0"/>'
        b'<Velocity>0.05</Velocity>'
        b'<Gripper>' + str(state).encode() + b'</Gripper>'
        b'</RobotCommand>'
    )

class MoveItEkiBridge(Node):
    def __init__(self):
        super().__init__('moveit_eki_bridge')
        self.kuka_ip = "192.168.1.147"
        self.get_logger().info(f"Connecting to KUKA at {self.kuka_ip}...")

        self.motion_client = EkiMotionClient(self.kuka_ip)
        self.motion_client.connect()
        self.get_logger().info("--- EKI BRIDGE CONNECTED ---")

        # Motion — listen to RViz planned path
        self.subscription = self.create_subscription(
            DisplayTrajectory,
            '/display_planned_path',
            self.display_trajectory_callback,
            10
        )
        self.get_logger().info("Listening on /display_planned_path ...")

        # Gripper state and lock (shared between keyboard and topic)
        self._gripper_state = 0
        self._gripper_lock = threading.Lock()

        # Subscribe to gripper commands from surgical_control_server
        self.create_subscription(
            Int8,
            GRIPPER_CMD_TOPIC,
            self._gripper_cmd_callback,
            10
        )
        self.get_logger().info(f"Listening on {GRIPPER_CMD_TOPIC} for gripper commands ...")

        # Keyboard thread — spacebar still works for manual override
        self._kb_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._kb_thread.start()
        self.get_logger().info("Gripper ready — SPACE to toggle manually, or commanded via /gripper_cmd")

    # ── Gripper ───────────────────────────────────────────────────────────────

    def _send_gripper(self, state: int, source: str = ''):
        """Send gripper packet through the existing EKI motion socket."""
        try:
            self.motion_client._tcp_client.sendall(build_gripper_packet(state))
            label = "ON  (pick)" if state else "OFF (place)"
            self.get_logger().info(f"Gripper {label}{f'  [{source}]' if source else ''}")
        except Exception as e:
            self.get_logger().error(f"Gripper send failed: {e}")

    def _set_gripper(self, state: int, source: str = ''):
        """Set gripper to an explicit state (idempotent — only sends if state changes)."""
        with self._gripper_lock:
            if self._gripper_state != state:
                self._gripper_state = state
                self._send_gripper(state, source)
            else:
                self.get_logger().debug(
                    f"Gripper already {'ON' if state else 'OFF'} — no change")

    def _toggle_gripper(self):
        """Toggle gripper state (keyboard use)."""
        with self._gripper_lock:
            self._gripper_state ^= 1
            self._send_gripper(self._gripper_state, 'keyboard')

    # ── Topic callback from surgical_control_server ───────────────────────────

    def _gripper_cmd_callback(self, msg: Int8):
        state = int(msg.data)
        if state not in (0, 1):
            self.get_logger().warn(f"Invalid gripper_cmd value: {state} (expected 0 or 1)")
            return
        self._set_gripper(state, 'control_server')

    # ── Keyboard ──────────────────────────────────────────────────────────────

    def _keyboard_loop(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while True:
                ch = sys.stdin.read(1)
                if ch == ' ':
                    self._toggle_gripper()
                elif ch in ('q', 'Q', '\x03'):  # q or Ctrl-C
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    # ── Motion ────────────────────────────────────────────────────────────────

    def display_trajectory_callback(self, msg: DisplayTrajectory):
        if not msg.trajectory:
            self.get_logger().warn("Empty trajectory received, skipping.")
            return

        joint_traj = msg.trajectory[0].joint_trajectory

        if not joint_traj.points:
            self.get_logger().warn("No points in trajectory.")
            return

        final_point = joint_traj.points[-1]
        joint_names = joint_traj.joint_names

        target_angles = [0.0] * 6
        for i, name in enumerate(joint_names):
            if "joint_1" in name:   target_angles[0] = math.degrees(final_point.positions[i])
            elif "joint_2" in name: target_angles[1] = math.degrees(final_point.positions[i])
            elif "joint_3" in name: target_angles[2] = math.degrees(final_point.positions[i])
            elif "joint_4" in name: target_angles[3] = math.degrees(final_point.positions[i])
            elif "joint_5" in name: target_angles[4] = math.degrees(final_point.positions[i])
            elif "joint_6" in name: target_angles[5] = math.degrees(final_point.positions[i])

        target = Axis(
            a1=target_angles[0], a2=target_angles[1], a3=target_angles[2],
            a4=target_angles[3], a5=target_angles[4], a6=target_angles[5]
        )
        self.get_logger().info(f"Sending: {target}")

        try:
            self.motion_client.ptp(target, max_velocity_scaling=0.05)
            self.get_logger().info("Command sent.")
        except Exception as e:
            self.get_logger().error(f"Transmission failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    bridge = MoveItEkiBridge()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        bridge.get_logger().info("Shutting down bridge.")
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
