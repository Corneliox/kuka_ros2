"""
vision.py  —  Eye-in-hand visual detection node
Replaces: vision_logic_mock.py

Pipeline:
  Camera image
    → HSV black-object detection (contour → centroid pixel)
    → Depth Anything V2 (metric, indoor) → Z_cam at centroid pixel
    → Pinhole deprojection → X_cam, Y_cam  (requires calibrated intrinsics)
    → TF2 chain: camera_optical_frame → tool0 → ... → world
    → Publish PoseStamped on /detected_object_pose (world frame)

Prereqs:
  1. Camera calibrated — /camera_info must have non-zero fx, fy
       ros2 run camera_calibration cameracalibrator \
           --size 8x6 --square 0.025 \
           --ros-args -r image:=/image_raw -r camera:=/camera
  2. Depth Anything V2 checkpoint downloaded:
       mkdir -p ~/checkpoints
       wget -O ~/checkpoints/depth_anything_v2_metric_hypersim_vits.pth \
         https://huggingface.co/depth-anything/Depth-Anything-V2-Metric-Hypersim-Small/resolve/main/depth_anything_v2_metric_hypersim_vits.pth
  3. Depth Anything V2 repo cloned alongside checkpoint:
       git clone https://github.com/DepthAnything/Depth-Anything-V2 ~/Depth-Anything-V2
  4. TF tree: camera_optical_frame child of tool0
       Placeholder until easy_handeye2:
         ros2 run tf2_ros static_transform_publisher \
           --x 0.05 --y 0.0 --z 0.08 \
           --roll 0 --pitch 1.5708 --yaw 0 \
           --frame-id tool0 --child-frame-id camera_optical_frame
  5. surgical_control_server.py subscriber addition — see bottom of file

Run:
  ros2 run kuka_ros2_demo vision_node --ros-args \
    -p camera_topic:=/image_raw \
    -p camera_info_topic:=/camera_info \
    -p depth_model_path:=/home/emil/checkpoints/depth_anything_v2_metric_hypersim_vits.pth \
    -p depth_repo_path:=/home/emil/Depth-Anything-V2
"""

import sys
import threading

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

import tf2_ros
import tf2_geometry_msgs


# ── Tunables ──────────────────────────────────────────────────────────────────
BLACK_H_LO, BLACK_H_HI = 0,   180
BLACK_S_LO, BLACK_S_HI = 0,   100   # low saturation  → achromatic
BLACK_V_LO, BLACK_V_HI = 0,    20   # low brightness  → dark

CONTOUR_AREA_MIN  = 500    # px² — raise if background noise triggers detection
MORPH_OPEN_K      = 5      # px  — remove speckle
MORPH_CLOSE_K     = 15     # px  — close holes in blob
DETECTION_RATE_HZ = 10.0
# ─────────────────────────────────────────────────────────────────────────────


def _load_depth_model(repo_path: str, checkpoint_path: str):
    """Import DepthAnythingV2 from local clone and load vits indoor checkpoint."""
    if repo_path not in sys.path:
        sys.path.insert(0, f"{repo_path}/metric_depth")

    import torch
    from depth_anything_v2.dpt import DepthAnythingV2

    cfg = {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384],
           'max_depth': 20}
    model = DepthAnythingV2(**cfg)
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    model.load_state_dict(
        torch.load(checkpoint_path, map_location=device, weights_only=True)
    )
    model = model.to(device).eval()
    return model, device


class VisionNode(Node):

    def __init__(self):
        super().__init__('vision_node')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('camera_frame',      'camera_optical_frame')
        self.declare_parameter('world_frame',       'world')
        self.declare_parameter('camera_topic',      '/image_raw')
        self.declare_parameter('camera_info_topic', '/camera_info')
        self.declare_parameter('min_confidence_area', float(CONTOUR_AREA_MIN))
        self.declare_parameter(
            'depth_model_path',
            '/home/emil/checkpoints/depth_anything_v2_metric_hypersim_vits.pth'
        )
        self.declare_parameter(
            'depth_repo_path',
            '/home/emil/Depth-Anything-V2'
        )

        # ── State ────────────────────────────────────────────────────────────
        self.K: np.ndarray | None = None   # set once from /camera_info
        self.bridge = CvBridge()
        self._latest_frame: np.ndarray | None = None
        self._frame_lock = threading.Lock()

        # ── Depth Anything V2 ────────────────────────────────────────────────
        model_path = self.get_parameter('depth_model_path').value
        repo_path  = self.get_parameter('depth_repo_path').value
        self.get_logger().info(f"Loading Depth Anything V2 from {model_path} ...")
        try:
            self._depth_model, self._depth_device = _load_depth_model(repo_path, model_path)
            self.get_logger().info(f"Depth model loaded — device: {self._depth_device}")
        except Exception as e:
            self.get_logger().error(f"Depth model load failed: {e}")
            raise

        # ── TF2 ──────────────────────────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Subscriptions ────────────────────────────────────────────────────
        self.create_subscription(
            Image,      self.get_parameter('camera_topic').value,      self._image_callback, 10)
        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value, self._camera_info_callback, 10)

        # ── Publishers ───────────────────────────────────────────────────────
        self.pose_pub  = self.create_publisher(PoseStamped, '/detected_object_pose', 10)
        self.debug_pub = self.create_publisher(Image,       '/vision_debug',         10)

        # ── Timer ─────────────────────────────────────────────────────────────
        self.create_timer(1.0 / DETECTION_RATE_HZ, self._detect)

        self.get_logger().info(
            f"vision_node ready  camera={self.get_parameter('camera_topic').value}"
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _camera_info_callback(self, msg: CameraInfo):
        if self.K is not None:
            return
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        if K[0, 0] == 0.0:
            self.get_logger().warn(
                "CameraInfo has fx=0 — camera not calibrated. "
                "X/Y will be wrong. Run camera_calibration first.",
                throttle_duration_sec=5.0,
            )
            return
        self.K = K
        self.get_logger().info(
            f"Intrinsics: fx={K[0,0]:.1f}  fy={K[1,1]:.1f}  "
            f"cx={K[0,2]:.1f}  cy={K[1,2]:.1f}"
        )

    def _image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge: {e}")
            return
        with self._frame_lock:
            self._latest_frame = frame

    # ── Detection ─────────────────────────────────────────────────────────────

    def _detect(self):
        if self.K is None:
            self.get_logger().warn(
                "Waiting for valid camera intrinsics — calibrate camera first.",
                throttle_duration_sec=5.0,
            )
            return

        with self._frame_lock:
            if self._latest_frame is None:
                return
            frame = self._latest_frame.copy()

        # ── 1. HSV segmentation ───────────────────────────────────────────────
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            (BLACK_H_LO, BLACK_S_LO, BLACK_V_LO),
            (BLACK_H_HI, BLACK_S_HI, BLACK_V_HI),
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                                np.ones((MORPH_OPEN_K,  MORPH_OPEN_K),  np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                                np.ones((MORPH_CLOSE_K, MORPH_CLOSE_K), np.uint8))

        # ── 2. Contour → centroid ─────────────────────────────────────────────
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        min_area = self.get_parameter('min_confidence_area').value
        h, w = frame.shape[:2]

        def touches_border(c):
            x, y, cw, ch = cv2.boundingRect(c)
            return x <= 2 or y <= 2 or x + cw >= w - 2 or y + ch >= h - 2

        valid = [c for c in contours
                 if min_area <= cv2.contourArea(c) <= 50000
                 and not touches_border(c)]

        if not valid:
            self._publish_debug(frame, mask)
            return

        c = max(valid, key=cv2.contourArea)
        M = cv2.moments(c)
        if M['m00'] == 0.0:
            return
        u = M['m10'] / M['m00']   # centroid pixel col
        v = M['m01'] / M['m00']   # centroid pixel row

        # ── 3. Depth Anything V2 → metric Z at centroid ───────────────────────
        try:
            depth_map = self._depth_model.infer_image(frame)   # HxW float32, metres
        except Exception as e:
            self.get_logger().error(f"Depth inference failed: {e}")
            return

        Z_cam = float(depth_map[int(v), int(u)])

        if Z_cam <= 0.01:
            self.get_logger().warn(f"Depth at centroid is {Z_cam:.3f}m — skipping.")
            return

        # ── 4. Pinhole deprojection → X, Y ────────────────────────────────────
        fx = self.K[0, 0];  fy = self.K[1, 1]
        cx = self.K[0, 2];  cy = self.K[1, 2]

        X_cam = (u - cx) * Z_cam / fx
        Y_cam = (v - cy) * Z_cam / fy

        # ── 5. PoseStamped in camera frame ────────────────────────────────────
        camera_frame = self.get_parameter('camera_frame').value
        world_frame  = self.get_parameter('world_frame').value

        ps_cam = PoseStamped()
        ps_cam.header.stamp    = self.get_clock().now().to_msg()
        ps_cam.header.frame_id = camera_frame
        ps_cam.pose.position.x = X_cam
        ps_cam.pose.position.y = Y_cam
        ps_cam.pose.position.z = Z_cam
        # Orientation identity — surgical_control_server.py overrides with
        # pre-calibrated grasp quaternion before set_pose_target().
        ps_cam.pose.orientation.w = 1.0

        # ── 6. TF2: camera_optical_frame → world ──────────────────────────────
        try:
            transform = self.tf_buffer.lookup_transform(
                world_frame, camera_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"TF2 lookup failed: {e}", throttle_duration_sec=2.0)
            return

        pose_world = tf2_geometry_msgs.do_transform_pose(ps_cam.pose, transform)

        ps_world = PoseStamped()
        ps_world.header.stamp    = ps_cam.header.stamp
        ps_world.header.frame_id = world_frame
        ps_world.pose            = pose_world

        self.pose_pub.publish(ps_world)

        self.get_logger().info(
            f"[DETECT] world  X={pose_world.position.x*1000:+.1f}mm "
            f"Y={pose_world.position.y*1000:+.1f}mm "
            f"Z={pose_world.position.z*1000:+.1f}mm  "
            f"(cam_Z={Z_cam*1000:.0f}mm)"
        )

        # ── 7. Debug image ─────────────────────────────────────────────────────
        debug = frame.copy()
        cv2.drawContours(debug, [c], -1, (0, 255, 0), 2)
        cv2.circle(debug, (int(u), int(v)), 6, (0, 0, 255), -1)
        cv2.putText(
            debug,
            f"Z={Z_cam*1000:.0f}mm",
            (int(u) + 10, int(v) - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2,
        )
        # Depth map overlaid as colourmap on right panel
        depth_vis = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX)
        depth_vis = cv2.applyColorMap(depth_vis.astype(np.uint8), cv2.COLORMAP_INFERNO)
        depth_vis = cv2.resize(depth_vis, (frame.shape[1], frame.shape[0]))
        self._publish_debug(debug, right=depth_vis)

    def _publish_debug(self, left: np.ndarray, mask: np.ndarray | None = None,
                       right: np.ndarray | None = None):
        if right is not None:
            out = np.hstack([left, right])
        elif mask is not None:
            out = np.hstack([left, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)])
        else:
            out = left
        try:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(out, encoding='bgr8'))
        except Exception:
            pass


# ── Entrypoint ────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


# ─────────────────────────────────────────────────────────────────────────────
# ADDITIONS REQUIRED IN surgical_control_server.py
# ─────────────────────────────────────────────────────────────────────────────
#
# 1. Add imports:
#    import copy, threading
#    from geometry_msgs.msg import PoseStamped
#
# 2. In __init__():
#    self._detected_pose: PoseStamped | None = None
#    self._pose_lock = threading.Lock()
#    self.create_subscription(
#        PoseStamped, '/detected_object_pose',
#        self._detected_pose_callback, 10)
#
# 3. Add callback:
#    def _detected_pose_callback(self, msg: PoseStamped):
#        with self._pose_lock:
#            self._detected_pose = msg
#
# 4. Replace hardcoded tray pose in pick sequence (step 2):
#    with self._pose_lock:
#        if self._detected_pose is None:
#            self.get_logger().error("No object detected — aborting pick.")
#            return
#        pick_pose = copy.deepcopy(self._detected_pose.pose)
#    pick_pose.orientation.x = 0.0321
#    pick_pose.orientation.y = 0.9235
#    pick_pose.orientation.z = 0.0197
#    pick_pose.orientation.w = 0.3816
#    move_group.set_pose_target(pick_pose)
# ─────────────────────────────────────────────────────────────────────────────
