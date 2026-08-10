#!/usr/bin/env python3
"""
benchmark_logger.py

Auto-logging node for the 50-run benchmark plan (benchmark.md). Drop this
into kuka_ros2_demo/kuka_ros2_demo/ alongside recorder.py (it follows the
same start/end-topic pattern already used there for episode recording) and
add an entry_point in setup.py:

    'benchmark_logger = kuka_ros2_demo.benchmark_logger:main',

WHAT IT AUTO-CAPTURES (no human input needed):
  - completion_time_s        : /benchmark_run_start -> /benchmark_run_end
  - decision_latency_s       : last /voice_command before start ->
                                first observed change in /commanded_joint_states
                                during the run (i.e. "robot begins motion")
  - tracking_error_mean_deg  : mean |commanded - real| across all 6 joints,
                                sampled every time /joint_states arrives
                                during the run (commanded pulled from
                                /commanded_joint_states, see NaN caveat below)
  - tracking_error_max_deg   : max of the same
  - gripper_on_count         : number of gripper ON (1) commands seen during
                                the run -- proxy for retries/attempts
  - moveit_error_codes       : any non-success MoveGroup-style error code
                                seen on /task_status (see NOTE below)

WHAT STILL NEEDS A HUMAN (filled in via the /benchmark_run_end payload,
since they require visual confirmation or ground-truth measurement):
  - task_success, first_attempt_success, pick_success, place_success
  - position_error_mm  (measured object position vs. commanded target)
  - collision_count, drop (bool)
  - free-form notes (lighting condition, clutter level, occlusion %, etc.)

NOTE on moveit_error_codes / collisions:
  This node does NOT intercept the /execute_task service directly (that
  would require wrapping control_server's service client, which is out of
  scope for a passive logger). If you want automatic collision/abort
  detection, have control_server also publish a small std_msgs/String
  status line (e.g. "STEP 2 FAILED error_code=-31") on a new /task_status
  topic -- this node already subscribes to it opportunistically and will
  record anything it sees, but works fine (with that field blank) if you
  never add the topic.

CONTROL PROTOCOL (mirrors recorder.py's /episode_start //episode_end):
  /benchmark_run_start (std_msgs/String) -- JSON payload:
      {"test": "baseline", "run": 3, "params": {"color": "red",
       "destination": "green_mat"}}
    test must be one of: baseline, generalization, vision_robustness,
    repeatability (matches benchmark.md's four test blocks) -- anything
    else is accepted but flagged in the CSV for you to fix up later.

  /benchmark_run_end (std_msgs/String) -- JSON payload, all fields optional
  (missing ones are written as blank in the CSV):
      {"task_success": true, "first_attempt_success": true,
       "pick_success": true, "place_success": true,
       "position_error_mm": 3.2, "collision_count": 0, "drop": false,
       "retries": 0, "notes": "normal lighting"}

Usage while running your benchmark:
  ros2 topic pub --once /benchmark_run_start std_msgs/msg/String \
      '{data: "{\"test\": \"baseline\", \"run\": 1, \"params\": {\"color\": \"red\"}}"}'
  ... run the pick-and-place task through your normal voice/coordinator flow ...
  ros2 topic pub --once /benchmark_run_end std_msgs/msg/String \
      '{data: "{\"task_success\": true, \"position_error_mm\": 4.1}"}'

Output: one CSV row appended per run to output_dir/benchmark_results.csv,
created with a header if it doesn't already exist. Safe to stop/restart the
node between runs -- it always appends.
"""

import csv
import json
import math
import os
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Int8
from sensor_msgs.msg import JointState


JOINT_ORDER = [f'joint_{i}' for i in range(1, 7)]  # matches kuka_eki_controller_node

CSV_FIELDS = [
    'test', 'run', 'timestamp', 'params',
    'task_success', 'first_attempt_success', 'pick_success', 'place_success',
    'completion_time_s', 'decision_latency_s',
    'position_error_mm',
    'tracking_error_mean_deg', 'tracking_error_max_deg',
    'gripper_on_count',
    'collision_count', 'drop', 'retries',
    'moveit_error_codes', 'notes',
]

DEFAULT_OUTPUT_DIR = os.path.expanduser('~/kuka_ros2/benchmark_data')
DEFAULT_JOINT_STATES_TOPIC = '/joint_states'
DEFAULT_COMMANDED_JOINT_TOPIC = '/commanded_joint_states'


def _extract_ordered_positions(msg: JointState):
    name_to_pos = dict(zip(msg.name, msg.position))
    try:
        return [float(name_to_pos[n]) for n in JOINT_ORDER]
    except KeyError:
        return None


class BenchmarkLogger(Node):

    def __init__(self):
        super().__init__('benchmark_logger')

        self.declare_parameter('output_dir', DEFAULT_OUTPUT_DIR)
        self.declare_parameter('joint_states_topic', DEFAULT_JOINT_STATES_TOPIC)
        self.declare_parameter('commanded_joint_topic', DEFAULT_COMMANDED_JOINT_TOPIC)

        self.output_dir = self.get_parameter('output_dir').value
        os.makedirs(self.output_dir, exist_ok=True)
        self.csv_path = os.path.join(self.output_dir, 'benchmark_results.csv')
        self._ensure_csv_header()

        joint_states_topic = self.get_parameter('joint_states_topic').value
        commanded_joint_topic = self.get_parameter('commanded_joint_topic').value

        self._lock = threading.Lock()
        self._recording = False
        self._run_meta = {}
        self._start_wall = None
        self._last_voice_time = None
        self._latest_commanded = None
        self._baseline_commanded = None
        self._motion_start_time = None
        self._tracking_errors = []  # list of mean-abs-error-degrees samples
        self._gripper_on_count = 0
        self._error_codes_seen = []

        self.create_subscription(String, '/voice_command', self._voice_cb, 10)
        self.create_subscription(Int8, '/gripper_cmd', self._gripper_cb, 10)
        self.create_subscription(JointState, joint_states_topic, self._joint_state_cb, 10)
        self.create_subscription(JointState, commanded_joint_topic, self._commanded_cb, 10)
        # Optional -- only useful if control_server is extended to publish it.
        # See module docstring NOTE. Harmless no-op if nothing ever publishes here.
        self.create_subscription(String, '/task_status', self._task_status_cb, 10)

        self.create_subscription(String, '/benchmark_run_start', self._run_start_cb, 10)
        self.create_subscription(String, '/benchmark_run_end', self._run_end_cb, 10)

        self.get_logger().info(
            f'benchmark_logger ready. Writing to {self.csv_path}\n'
            f'  Start a run:  ros2 topic pub --once /benchmark_run_start std_msgs/msg/String '
            f'\'{{data: "{{\\"test\\": \\"baseline\\", \\"run\\": 1}}"}}\'\n'
            f'  End a run:    ros2 topic pub --once /benchmark_run_end std_msgs/msg/String '
            f'\'{{data: "{{\\"task_success\\": true}}"}}\''
        )

    # -- CSV setup ------------------------------------------------------------

    def _ensure_csv_header(self):
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
                writer.writeheader()

    # -- Passive subscriptions --------------------------------------------------

    def _voice_cb(self, msg: String):
        self._last_voice_time = time.monotonic()

    def _gripper_cb(self, msg: Int8):
        with self._lock:
            if self._recording and int(msg.data) == 1:
                self._gripper_on_count += 1

    def _commanded_cb(self, msg: JointState):
        ordered = _extract_ordered_positions(msg)
        if ordered is None:
            return
        with self._lock:
            self._latest_commanded = ordered
            if self._recording and self._motion_start_time is None:
                # First commanded update we see after the baseline snapshot
                # taken at run start counts as "robot begins motion" --
                # unless it's identical to the baseline (duplicate/no-op).
                if self._baseline_commanded is None or ordered != self._baseline_commanded:
                    self._motion_start_time = time.monotonic()

    def _joint_state_cb(self, msg: JointState):
        ordered = _extract_ordered_positions(msg)
        if ordered is None:
            return
        with self._lock:
            if not self._recording or self._latest_commanded is None:
                return
            errs = [abs(math.degrees(c) - math.degrees(r))
                    for c, r in zip(self._latest_commanded, ordered)]
            self._tracking_errors.append(sum(errs) / len(errs))

    def _task_status_cb(self, msg: String):
        with self._lock:
            if self._recording:
                self._error_codes_seen.append(msg.data)

    # -- Run control -----------------------------------------------------------

    def _run_start_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'/benchmark_run_start payload not valid JSON: {e}')
            return

        with self._lock:
            if self._recording:
                self.get_logger().warn(
                    'Run already in progress -- ignoring new /benchmark_run_start. '
                    'Send /benchmark_run_end first.')
                return

            self._run_meta = {
                'test': payload.get('test', 'unknown'),
                'run': payload.get('run', ''),
                'params': json.dumps(payload.get('params', {})),
            }
            self._start_wall = time.monotonic()
            self._baseline_commanded = self._latest_commanded  # snapshot, may be None
            self._motion_start_time = None
            self._tracking_errors = []
            self._gripper_on_count = 0
            self._error_codes_seen = []
            self._recording = True

        self.get_logger().info(
            f"Run started: test={self._run_meta['test']} run={self._run_meta['run']}")

    def _run_end_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'/benchmark_run_end payload not valid JSON: {e}')
            payload = {}

        with self._lock:
            if not self._recording:
                self.get_logger().warn('Got /benchmark_run_end but no run is in progress.')
                return

            end_wall = time.monotonic()
            completion_time_s = end_wall - self._start_wall

            decision_latency_s = ''
            if self._motion_start_time is not None:
                t0 = self._last_voice_time if self._last_voice_time is not None else self._start_wall
                decision_latency_s = round(self._motion_start_time - t0, 3)

            tracking_mean = (round(sum(self._tracking_errors) / len(self._tracking_errors), 4)
                              if self._tracking_errors else '')
            tracking_max = round(max(self._tracking_errors), 4) if self._tracking_errors else ''

            row = {
                'test': self._run_meta.get('test', 'unknown'),
                'run': self._run_meta.get('run', ''),
                'timestamp': datetime.now().isoformat(timespec='seconds'),
                'params': self._run_meta.get('params', '{}'),
                'task_success': payload.get('task_success', ''),
                'first_attempt_success': payload.get('first_attempt_success', ''),
                'pick_success': payload.get('pick_success', ''),
                'place_success': payload.get('place_success', ''),
                'completion_time_s': round(completion_time_s, 3),
                'decision_latency_s': decision_latency_s,
                'position_error_mm': payload.get('position_error_mm', ''),
                'tracking_error_mean_deg': tracking_mean,
                'tracking_error_max_deg': tracking_max,
                'gripper_on_count': self._gripper_on_count,
                'collision_count': payload.get('collision_count', ''),
                'drop': payload.get('drop', ''),
                'retries': payload.get('retries', ''),
                'moveit_error_codes': ';'.join(self._error_codes_seen),
                'notes': payload.get('notes', ''),
            }

            self._recording = False

        with open(self.csv_path, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
            writer.writerow(row)

        self.get_logger().info(
            f"Run ended: test={row['test']} run={row['run']} "
            f"completion_time_s={row['completion_time_s']} "
            f"tracking_error_mean_deg={row['tracking_error_mean_deg']} -> logged to {self.csv_path}")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BenchmarkLogger()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
