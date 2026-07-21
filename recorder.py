#!/usr/bin/env python3
"""
episode_recorder.py

Records pick-and-place episodes for later conversion into Octo/RLDS
fine-tuning data. Runs alongside your existing scripted pipeline
(vision_detect_node / vision_node + pick_place_coordinator +
surgical_control_server) and taps:

  - the wrist camera (same physical device your vision nodes read from)
  - the robot's end-effector pose via TF (base_link -> tool0)
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
grabs whatever the current camera frame / EE pose / gripper state are,
regardless of how far the arm moved since the last tick. A stalled
moment (e.g. gripper closing while the arm holds still) is still its
own step.

Per-step data captured:
  - RGB image (wrist camera)            -> becomes image_primary
  - EE position + quaternion (TF)       -> becomes proprio; consecutive
                                            poses are subtracted later
                                            (in the RLDS conversion
                                            script) to derive the
                                            action label
  - gripper state (0/1)                 -> last proprio/action dim
  - timestamp

Per-episode metadata:
  - language instruction
  - success flag
  - step count, start/end wall-clock time

Usage:
  ros2 run kuka_ros2_demo episode_recorder \
      --ros-args -p camera_index:=2 -p record_hz:=10.0 \
      -p output_dir:=/home/emil/kuka_ros2/demo_data

  # In another terminal, once your scripted pick-and-place run starts:
  ros2 topic pub --once /episode_start std_msgs/msg/String \
      "{data: 'pick up the red cube and place it in the tray'}"

  # After the run finishes (object placed or dropped):
  ros2 topic pub --once /episode_end std_msgs/msg/Bool "{data: true}"

NOTE on the camera: this recorder opens its own cv2.VideoCapture on the
same camera index vision_node.py / vision_detect_node.py already use.
Some UVC webcam drivers refuse a second simultaneous reader on one
device -- if you hit that, the fix is to have one of the vision nodes
publish frames on a sensor_msgs/Image topic and have this node
subscribe to that topic instead of opening the device a second time.
Left as cv2.VideoCapture here to match your existing nodes' pattern and
get you recording quickly; swap to a topic subscription if a
device-busy error shows up.

RAW FORMAT ONLY: this script does NOT produce RLDS/TFDS directly. It
writes one file per episode in a simple, inspectable numpy/json
format. Converting a folder of these into an RLDS TFDS dataset for
Octo's finetune.py is a separate one-time script -- ask for it once
you have real episodes on disk to convert.
"""

import json
import os
import time
from datetime import datetime

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from std_msgs.msg import String, Bool, Int8

import tf2_ros


FRAME = 'base_link'   # matches FRAME convention in surgical_control_server.py etc.
TIP = 'tool0'         # matches TIP convention in the same nodes

DEFAULT_CAMERA_INDEX = 2       # matches vision_node.py / vision_detect_node.py
DEFAULT_RECORD_HZ = 10.0
DEFAULT_OUTPUT_DIR = os.path.expanduser('~/kuka_ros2/demo_data')

BUFFER_FLUSH_FRAMES = 5        # same stale-frame flush trick as vision_node.py
TF_LOOKUP_TIMEOUT_SEC = 0.05


class EpisodeRecorder(Node):

    def __init__(self):
        super().__init__('episode_recorder')

        self.declare_parameter('camera_index', DEFAULT_CAMERA_INDEX)
        self.declare_parameter('record_hz', DEFAULT_RECORD_HZ)
        self.declare_parameter('output_dir', DEFAULT_OUTPUT_DIR)

        self.camera_index = self.get_parameter('camera_index').value
        self.record_hz = self.get_parameter('record_hz').value
        self.output_dir = self.get_parameter('output_dir').value
        os.makedirs(self.output_dir, exist_ok=True)

        # -- Camera -----------------------------------------------------------
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            self.get_logger().error(
                f'Could not open camera index {self.camera_index}. '
                f'Is it already held open by vision_node/vision_detect_node? '
                f'See module docstring for the topic-subscription workaround.')
            raise RuntimeError('Camera open failed')

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
        self._orientations = []   # quaternion xyzw
        self._grippers = []
        self._timestamps = []

        self._timer = self.create_timer(1.0 / self.record_hz, self._tick)

        self.get_logger().info(
            f'episode_recorder ready. camera_index={self.camera_index} '
            f'record_hz={self.record_hz} output_dir={self.output_dir}\n'
            f"  Start:  ros2 topic pub --once /episode_start std_msgs/msg/String "
            f"\"{{data: 'pick up the red cube and place it in the tray'}}\"\n"
            f"  End:    ros2 topic pub --once /episode_end std_msgs/msg/Bool "
            f"\"{{data: true}}\""
        )

    # -- Callbacks -----------------------------------------------------------------

    def _gripper_cb(self, msg: Int8):
        self._gripper_state = int(msg.data)

    def _start_cb(self, msg: String):
        if self._recording:
            self.get_logger().warn(
                'Already recording an episode -- ignoring new /episode_start. '
                'Send /episode_end first.')
            return
        self._instruction = msg.data.strip()
        self._images = []
        self._positions = []
        self._orientations = []
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

        # Flush stale buffered frames so we grab a fresh one -- same trick
        # vision_node.py uses before reading.
        for _ in range(BUFFER_FLUSH_FRAMES):
            self.cap.grab()
        ok, frame_bgr = self.cap.read()
        if not ok:
            self.get_logger().warn('Camera read failed -- skipping this step.')
            return
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

        try:
            tf = self.tf_buffer.lookup_transform(
                FRAME, TIP, Time(), timeout=Duration(seconds=TF_LOOKUP_TIMEOUT_SEC))
        except Exception as e:
            self.get_logger().warn(
                f'TF lookup {FRAME}->{TIP} failed -- skipping this step: {e}')
            return

        t = tf.transform.translation
        q = tf.transform.rotation

        self._images.append(frame_rgb)
        self._positions.append([t.x, t.y, t.z])
        self._orientations.append([q.x, q.y, q.z, q.w])
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

        np.savez_compressed(
            npz_path,
            images=np.stack(self._images).astype(np.uint8),             # [N,H,W,3]
            positions=np.array(self._positions, dtype=np.float32),      # [N,3]
            orientations=np.array(self._orientations, dtype=np.float32),  # [N,4] xyzw
            grippers=np.array(self._grippers, dtype=np.float32),        # [N]
            timestamps=np.array(self._timestamps, dtype=np.float64),
        )

        meta = {
            'instruction': self._instruction,
            'success': success,
            'n_steps': n_steps,
            'record_hz': self.record_hz,
            'start_wall_time': self._episode_start_wall,
            'end_wall_time': time.time(),
            'npz_file': os.path.basename(npz_path),
        }
        with open(json_path, 'w') as f:
            json.dump(meta, f, indent=2)

        self.get_logger().info(
            f'Episode saved: {n_steps} steps -> {npz_path}  '
            f'(success={success}, instruction="{self._instruction}")')

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


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