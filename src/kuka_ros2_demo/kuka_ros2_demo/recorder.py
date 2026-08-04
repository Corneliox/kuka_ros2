#!/usr/bin/env python3
"""
episode_recorder.py

Records pick-and-place episodes for later conversion into Octo/RLDS
fine-tuning data. Runs alongside your existing scripted pipeline
(vision_detect_node / vision_node + pick_place_coordinator +
kuka_eki_controller_node) and taps:

  - the wrist camera, via /camera/image_raw published by vision_node.py
  - the robot's end-effector pose via TF (base_link -> tool0)
  - the robot's REAL joint positions, via /joint_states published by
    kuka_eki_controller_node.py's state loop (actual KRC4 feedback, not
    a mock/loopback)
  - the COMMANDED joint target most recently dispatched to the KRC4, via
    /commanded_joint_states published by kuka_eki_controller_node.py
    (see ACTION NOTE below -- important caveat on this one)
  - the last commanded gripper state (/gripper_cmd)

...and writes one .npz + one .json per episode to output_dir.

Episode control (no new .srv/.msg needed -- just two topics, matching
the lightweight style already used for /voice_command in this repo):

  /episode_start  (std_msgs/String)  -- payload is the language
                   instruction, e.g. "pick up the red cube and place
                   it in the tray". Starts buffering steps.
  /episode_end    (std_msgs/Bool)    -- True = success, False = fail.
                   Finalizes the episode, writes it to disk, and
                   clears the buffer.

Recording rate is fixed via the `record_hz` parameter (default 10 Hz,
matching Octo's typical control rate) -- NOT distance-based. Each tick
grabs whatever the latest received camera frame / EE pose / joint
state / commanded target / gripper state are, regardless of how far the
arm moved since the last tick. A stalled moment (e.g. gripper closing
while the arm holds still) is still its own step.

Per-step data captured:
  - RGB image (wrist camera, from /camera/image_raw) -> becomes image_primary
  - EE position + quaternion (TF)          -> proprio (task space)
  - Real joint positions (/joint_states)    -> proprio (joint space),
                                                6-vector, radians, order
                                                matches kuka_eki_controller_node's
                                                JOINT_ORDER (joint_1..joint_6)
  - Commanded joint target (/commanded_joint_states) -> explicit action
                                                target, same 6-vector /
                                                radians / order. See
                                                ACTION NOTE below.
  - gripper state (0/1)                     -> last proprio/action dim
  - timestamp

Per-episode metadata:
  - language instruction
  - success flag
  - step count, start/end wall-clock time

CAMERA NOTE (2026-07):
  vision_node.py owns the camera and publishes /camera/image_raw at a
  fixed rate; this node subscribes rather than opening its own
  cv2.VideoCapture (avoids two readers fighting over one UVC device).
  Camera framerate and this node's `record_hz` are NOT phase-locked --
  a recorded step's image can be up to one publish-interval stale. If
  you need tighter sync, raise vision_node's publish_rate_hz well above
  this node's record_hz (e.g. 20 Hz publish vs 10 Hz record).

ACTION NOTE (2026-08-03) -- READ BEFORE USING commanded_joint_positions:
  kuka_eki_controller_node.py executes motion as discrete, blocking
  joint-space PTP waypoints (this cell has no RSI license, so there is
  no continuous cyclic command channel -- see that node's module
  docstring). /commanded_joint_states is therefore a STEP signal: it
  only changes when the controller dispatches a new kept waypoint
  (empirically every ~0.1-0.6s, not fixed-rate), and holds that value
  constant while _wait_until_arrived() polls real state in between.

  At this recorder's fixed record_hz, that means several consecutive
  steps will often show an IDENTICAL commanded_joint_positions row while
  joint_positions (real state) is still catching up to it. This is
  correct, expected data -- not a stalled recorder and not a bug. Do not
  treat repeated commanded rows as missing data.

  What this buys you over the old state-only approach: the commanded
  target is the controller's actual intent at each moment, not an
  inference. A downstream RLDS conversion script can compute a per-step
  "action" as (commanded_joint_positions - joint_positions) directly
  from these two arrays without having to diff consecutive *observed*
  states and hope that approximates what was actually commanded.

  Until the controller has dispatched its first waypoint after
  /episode_start (i.e. before any trajectory has been sent this
  episode), commanded_joint_positions rows are filled with NaN rather
  than a stale value from a previous episode -- check for NaN before
  using this array downstream.

Usage:
  ros2 run kuka_ros2_demo episode_recorder \
      --ros-args -p camera_topic:=/camera/image_raw \
      -p joint_states_topic:=/joint_states \
      -p commanded_joint_topic:=/commanded_joint_states \
      -p record_hz:=10.0 \
      -p output_dir:=/home/emil/kuka_ros2/demo_data

  # In another terminal, once your scripted pick-and-place run starts:
  ros2 topic pub --once /episode_start std_msgs/msg/String \
      "{data: 'pick up the red cube and place it in the tray'}"

  # After the run finishes (watch pick_place_coordinator's terminal for
  # the checkmark/x line -- that's your real completion signal, not a
  # fixed wait):
  ros2 topic pub --once /episode_end std_msgs/msg/Bool "{data: true}"

RAW FORMAT ONLY: this script does NOT produce RLDS/TFDS directly. It
writes one file per episode in a simple, inspectable numpy/json
format. Converting a folder of these into an RLDS TFDS dataset for
Octo's finetune.py is a separate one-time script -- ask for it once
you have real episodes on disk to convert.
"""

import json
import os
import threading
import time
from datetime import datetime

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from std_msgs.msg import String, Bool, Int8
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge

import tf2_ros


FRAME = 'base_link'   # matches FRAME convention in control_server.py etc.
TIP = 'tool0'         # matches TIP convention in the same nodes

DEFAULT_CAMERA_TOPIC = '/camera/image_raw'                   # published by vision_node.py
DEFAULT_JOINT_STATES_TOPIC = '/joint_states'                  # published by kuka_eki_controller_node.py
DEFAULT_COMMANDED_JOINT_TOPIC = '/commanded_joint_states'     # published by kuka_eki_controller_node.py
DEFAULT_RECORD_HZ = 10.0
DEFAULT_OUTPUT_DIR = os.path.expanduser('~/kuka_ros2/demo_data')

# Must match kuka_eki_controller_node.JOINT_ORDER -- not imported directly
# since this node shouldn't need a hard package dependency on
# kuka_eki_bridge just to know a naming convention. Kept in sync manually;
# if you rename joints on either side, update both.
JOINT_ORDER = [f'joint_{i}' for i in range(1, 7)]

TF_LOOKUP_TIMEOUT_SEC = 0.05


def _extract_ordered_positions(msg: JointState):
    """Reorder a JointState message's positions to match JOINT_ORDER,
    regardless of what order the publisher sent them in. Returns None if
    any expected joint name is missing from the message."""
    name_to_pos = dict(zip(msg.name, msg.position))
    try:
        return [float(name_to_pos[n]) for n in JOINT_ORDER]
    except KeyError:
        return None


class EpisodeRecorder(Node):

    def __init__(self):
        super().__init__('episode_recorder')

        self.declare_parameter('camera_topic', DEFAULT_CAMERA_TOPIC)
        self.declare_parameter('joint_states_topic', DEFAULT_JOINT_STATES_TOPIC)
        self.declare_parameter('commanded_joint_topic', DEFAULT_COMMANDED_JOINT_TOPIC)
        self.declare_parameter('record_hz', DEFAULT_RECORD_HZ)
        self.declare_parameter('output_dir', DEFAULT_OUTPUT_DIR)

        self.camera_topic = self.get_parameter('camera_topic').value
        self.joint_states_topic = self.get_parameter('joint_states_topic').value
        self.commanded_joint_topic = self.get_parameter('commanded_joint_topic').value
        self.record_hz = self.get_parameter('record_hz').value
        self.output_dir = self.get_parameter('output_dir').value
        os.makedirs(self.output_dir, exist_ok=True)

        # -- Camera (subscription, not a device we own) --------------------------
        self.bridge = CvBridge()
        self._latest_frame = None
        self._frame_lock = threading.Lock()
        self.create_subscription(Image, self.camera_topic, self._image_cb, 10)

        # -- Real joint state (proprio, joint space) ------------------------------
        self._latest_joint_positions = None   # list[6] radians, or None until first msg
        self._joint_lock = threading.Lock()
        self.create_subscription(
            JointState, self.joint_states_topic, self._joint_state_cb, 10)

        # -- Commanded joint target (explicit action) ------------------------------
        # See ACTION NOTE in module docstring -- step signal, not continuous.
        self._latest_commanded_positions = None  # list[6] radians, or None until first waypoint sent
        self._commanded_lock = threading.Lock()
        self.create_subscription(
            JointState, self.commanded_joint_topic, self._commanded_joint_cb, 10)

        # -- TF -----------------------------------------------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # -- Gripper state tracking ----------------------------------------------
        self._gripper_state = 0
        self.create_subscription(Int8, '/gripper_cmd', self._gripper_cb, 10)

        # -- Episode control -------------------------------------------------------
        self.create_subscription(String, '/episode_start', self._start_cb, 10)
        self.create_subscription(Bool, '/episode_end', self._end_cb, 10)

        # -- Recording state --------------------------------------------------------
        self._recording = False
        self._instruction = ''
        self._episode_start_wall = None
        self._images = []
        self._positions = []
        self._orientations = []          # quaternion xyzw
        self._joint_positions = []       # [N,6] radians, real state
        self._commanded_positions = []   # [N,6] radians, commanded target (may be NaN rows)
        self._grippers = []
        self._timestamps = []

        self._timer = self.create_timer(1.0 / self.record_hz, self._tick)

        self.get_logger().info(
            f'episode_recorder ready. camera_topic={self.camera_topic} '
            f'joint_states_topic={self.joint_states_topic} '
            f'commanded_joint_topic={self.commanded_joint_topic} '
            f'record_hz={self.record_hz} output_dir={self.output_dir}\n'
            f"  Start:  ros2 topic pub --once /episode_start std_msgs/msg/String "
            f"\"{{data: 'pick up the red cube and place it in the tray'}}\"\n"
            f"  End:    ros2 topic pub --once /episode_end std_msgs/msg/Bool "
            f"\"{{data: true}}\""
        )

    # -- Callbacks -----------------------------------------------------------------

    def _image_cb(self, msg: Image):
        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge conversion failed: {e}', throttle_duration_sec=5.0)
            return
        with self._frame_lock:
            self._latest_frame = frame_bgr

    def _joint_state_cb(self, msg: JointState):
        ordered = _extract_ordered_positions(msg)
        if ordered is None:
            self.get_logger().warn(
                f'{self.joint_states_topic} message missing expected joint names '
                f'{JOINT_ORDER} -- got {list(msg.name)}. Skipping this message.',
                throttle_duration_sec=5.0)
            return
        with self._joint_lock:
            self._latest_joint_positions = ordered

    def _commanded_joint_cb(self, msg: JointState):
        ordered = _extract_ordered_positions(msg)
        if ordered is None:
            self.get_logger().warn(
                f'{self.commanded_joint_topic} message missing expected joint names '
                f'{JOINT_ORDER} -- got {list(msg.name)}. Skipping this message.',
                throttle_duration_sec=5.0)
            return
        with self._commanded_lock:
            self._latest_commanded_positions = ordered

    def _gripper_cb(self, msg: Int8):
        self._gripper_state = int(msg.data)

    def _start_cb(self, msg: String):
        if self._recording:
            self.get_logger().warn(
                'Already recording an episode -- ignoring new /episode_start. '
                'Send /episode_end first.')
            return

        with self._frame_lock:
            have_frame = self._latest_frame is not None
        with self._joint_lock:
            have_joints = self._latest_joint_positions is not None
        if not have_frame:
            self.get_logger().warn(
                f'No frame received yet on {self.camera_topic} -- is vision_node '
                f'running and publishing? Starting anyway; early steps may be '
                f'skipped until the first frame arrives.')
        if not have_joints:
            self.get_logger().warn(
                f'No joint state received yet on {self.joint_states_topic} -- is '
                f'kuka_eki_controller running? Starting anyway; early steps may '
                f'be skipped until the first joint state arrives.')

        # Commanded target is expected to be unset at episode start (no
        # waypoint dispatched yet) -- that's normal, not a warning. Reset it
        # to None here so we never carry a stale target from a PREVIOUS
        # episode's last waypoint into this one's early rows.
        with self._commanded_lock:
            self._latest_commanded_positions = None

        self._instruction = msg.data.strip()
        self._images = []
        self._positions = []
        self._orientations = []
        self._joint_positions = []
        self._commanded_positions = []
        self._grippers = []
        self._timestamps = []
        self._episode_start_wall = time.time()
        self._recording = True
        self.get_logger().info(f'Episode started: "{self._instruction}"')

    def _end_cb(self, msg: Bool):
        if not self._recording:
            self.get_logger().warn('Got /episode_end but no episode is in progress.')
            return
        self._recording = False
        success = bool(msg.data)
        self._save_episode(success)

    # -- Recording tick --------------------------------------------------------------

    def _tick(self):
        if not self._recording:
            return

        with self._frame_lock:
            if self._latest_frame is None:
                self.get_logger().warn(
                    f'No frame received yet on {self.camera_topic} -- skipping this step.',
                    throttle_duration_sec=2.0,
                )
                return
            frame_bgr = self._latest_frame.copy()
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

        try:
            tf = self.tf_buffer.lookup_transform(
                FRAME, TIP, Time(), timeout=Duration(seconds=TF_LOOKUP_TIMEOUT_SEC))
        except Exception as e:
            self.get_logger().warn(
                f'TF lookup {FRAME}->{TIP} failed -- skipping this step: {e}')
            return

        with self._joint_lock:
            if self._latest_joint_positions is None:
                self.get_logger().warn(
                    f'No joint state received yet on {self.joint_states_topic} -- '
                    f'skipping this step.',
                    throttle_duration_sec=2.0,
                )
                return
            joint_positions = list(self._latest_joint_positions)

        with self._commanded_lock:
            # None before the first waypoint this episode -- fill with NaN
            # rather than skipping the step entirely, so image/EE/joint data
            # for this tick isn't thrown away just because no command has
            # been dispatched yet (e.g. during the initial park-for-detection
            # move, which may finish before pick_place_coordinator triggers
            # the actual pick trajectory).
            if self._latest_commanded_positions is None:
                commanded_positions = [float('nan')] * 6
            else:
                commanded_positions = list(self._latest_commanded_positions)

        t = tf.transform.translation
        q = tf.transform.rotation

        self._images.append(frame_rgb)
        self._positions.append([t.x, t.y, t.z])
        self._orientations.append([q.x, q.y, q.z, q.w])
        self._joint_positions.append(joint_positions)
        self._commanded_positions.append(commanded_positions)
        self._grippers.append(self._gripper_state)
        self._timestamps.append(time.time())

    # -- Save --------------------------------------------------------------------------

    def _save_episode(self, success: bool):
        n_steps = len(self._images)
        if n_steps == 0:
            self.get_logger().warn('Episode had zero steps -- not saving.')
            return

        stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        npz_path = os.path.join(self.output_dir, f'episode_{stamp}.npz')
        json_path = os.path.join(self.output_dir, f'episode_{stamp}.json')

        commanded_arr = np.array(self._commanded_positions, dtype=np.float32)
        n_nan_rows = int(np.isnan(commanded_arr).any(axis=1).sum())

        np.savez_compressed(
            npz_path,
            images=np.stack(self._images).astype(np.uint8),               # [N,H,W,3]
            positions=np.array(self._positions, dtype=np.float32),        # [N,3]
            orientations=np.array(self._orientations, dtype=np.float32),  # [N,4] xyzw
            joint_positions=np.array(self._joint_positions, dtype=np.float32),   # [N,6] rad, real
            commanded_joint_positions=commanded_arr,                      # [N,6] rad, may contain NaN rows
            grippers=np.array(self._grippers, dtype=np.float32),          # [N]
            timestamps=np.array(self._timestamps, dtype=np.float64),
        )

        meta = {
            'instruction': self._instruction,
            'success': success,
            'n_steps': n_steps,
            'record_hz': self.record_hz,
            'camera_topic': self.camera_topic,
            'joint_states_topic': self.joint_states_topic,
            'commanded_joint_topic': self.commanded_joint_topic,
            'joint_order': JOINT_ORDER,
            'commanded_joint_positions_nan_rows': n_nan_rows,
            'start_wall_time': self._episode_start_wall,
            'end_wall_time': time.time(),
            'npz_file': os.path.basename(npz_path),
        }
        with open(json_path, 'w') as f:
            json.dump(meta, f, indent=2)

        self.get_logger().info(
            f'Episode saved: {n_steps} steps -> {npz_path}  '
            f'(success={success}, instruction="{self._instruction}", '
            f'commanded_joint_positions NaN rows={n_nan_rows}/{n_steps})')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = EpisodeRecorder()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()