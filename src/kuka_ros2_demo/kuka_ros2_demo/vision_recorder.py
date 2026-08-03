#!/usr/bin/env python3
"""
vision_node.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Trigger-based (NOT continuous-stream) object detection service.

Why trigger-based, not livestreamed:
  This pipeline is designed to run on a Minisforum (no GPU). Continuous
  per-frame processing is fine for cheap HSV color thresholding, but the
  moment this gets swapped for a real trained detector (see TODO below),
  running that at video framerate on CPU-only hardware would be far too
  slow. Trigger-based (one frame, on demand, per voice command) stays
  cheap regardless of what runs inside detect_color(), so the swap to a
  real detector later doesn't require an architecture change -- only
  the internals of detect_color() change.

Exposes:
  /detect_object  (surgical_msgs/srv/DetectObject)
    request:  color_name (string)   e.g. "red", "blue", "yellow", "green"
    response: found (bool), x (float64, metres), y (float64, metres)
              -- in base_link frame, matching MoveIt/geometry_msgs convention.
              NOTE: internally this script works in millimetres (matching
              vision.py / the homography's native units) and converts to
              METRES only at the service response boundary. Keeping the
              ROS-facing contract in metres, consistent with every other
              node in this pipeline, is intentional -- this project has
              already been bitten once by a units mismatch (mm vs cm)
              that silently corrupted a calibration. Do not let internal
              mm-based math leak out of this boundary.

TODO (future upgrade path):
  Replace the body of detect_color() with a real object detector
  (.pt/.onnx model) call. Nothing else in this node, or in
  pick_place_coordinator.py, needs to change -- the service contract
  (color_name in, found/x/y out) stays the same regardless of what runs
  underneath. If the detector returns a class label instead of a color
  name, just adjust the request field's meaning accordingly.
"""

import rclpy
from rclpy.node import Node

import numpy as np
import cv2

from surgical_msgs.srv import DetectObject


# --- Camera intrinsics (wrist camera, default_cam.yaml) ---
CAMERA_MATRIX = np.array([
    [865.3064988411312, 0.0,               257.05723081254752],
    [0.0,               861.37645533726345, 266.08102298248292],
    [0.0,               0.0,               1.0],
])
DIST_COEFFS = np.array([
    0.20180575610957421, -0.16658050726097001,
    0.005021470803111815, -0.022511455420610536, 0.0
])

# Default path -- override at launch with:
#   ros2 run kuka_ros2_demo vision_node --ros-args -p homography_path:=/some/other/path.npy
DEFAULT_HOMOGRAPHY_PATH = "/home/emil/kuka_ros2/src/kuka_ros2_demo/data/aruco_homography.npy"

CAMERA_DEVICE_INDEX = 0

# --- HSV color ranges -- same as vision.py, still needs tuning per lighting ---
COLOR_RANGES = {
    "red": [
        (np.array([0,   100, 80]),  np.array([10,  255, 255])),
        (np.array([170, 100, 80]),  np.array([179, 255, 255])),
    ],
    "yellow": [(np.array([20, 100, 80]), np.array([35, 255, 255]))],
    "green":  [(np.array([40, 80, 60]),  np.array([85, 255, 255]))],
    "blue":   [(np.array([95, 100, 60]), np.array([130, 255, 255]))],
}
MIN_CONTOUR_AREA_PX = 200
MAX_CONTOUR_AREA_PX = 20000

# Number of throwaway grab() calls before reading a real frame -- USB webcams
# commonly buffer several stale frames internally, so without this you can
# end up processing an image from a second or more ago (e.g. before the arm
# finished retracting to the parked observation pose).
BUFFER_FLUSH_FRAMES = 5


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        self.declare_parameter('homography_path', DEFAULT_HOMOGRAPHY_PATH)
        homography_path = self.get_parameter('homography_path').value

        try:
            self.H = np.load(homography_path)
            self.get_logger().info(f"Loaded homography from {homography_path}")
        except FileNotFoundError:
            self.get_logger().error(
                f"Homography file not found at {homography_path}. "
                f"Run the calibration script first, or pass a different path via "
                f"--ros-args -p homography_path:=/your/path.npy"
            )
            raise

        self.cap = cv2.VideoCapture(CAMERA_DEVICE_INDEX)
        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open camera device index {CAMERA_DEVICE_INDEX}")
            raise RuntimeError("Camera open failed")

        self.srv = self.create_service(DetectObject, 'detect_object', self._handle_request)
        self.get_logger().info("vision_node ready -- serving /detect_object")

    def _capture_fresh_frame(self):
        for _ in range(BUFFER_FLUSH_FRAMES):
            self.cap.grab()
        ret, frame = self.cap.read()
        if not ret:
            return None
        return frame

    def _undistort(self, frame):
        h, w = frame.shape[:2]
        new_K, _ = cv2.getOptimalNewCameraMatrix(CAMERA_MATRIX, DIST_COEFFS, (w, h), 1, (w, h))
        return cv2.undistort(frame, CAMERA_MATRIX, DIST_COEFFS, None, new_K)

    def detect_color(self, hsv, color_name):
        """
        TODO: replace this method's body with a real object detector call
        when ready. Keep the same return contract: list of
        {"px":..., "py":..., "area":...} detections.
        """
        if color_name not in COLOR_RANGES:
            return []

        ranges = COLOR_RANGES[color_name]
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in ranges:
            mask |= cv2.inRange(hsv, lower, upper)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detections = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_CONTOUR_AREA_PX or area > MAX_CONTOUR_AREA_PX:
                continue
            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            cx, cy = M["m10"] / M["m00"], M["m01"] / M["m00"]
            detections.append({"px": cx, "py": cy, "area": area})
        return detections

    def _pixel_to_world_mm(self, px, py):
        p = np.array([px, py, 1.0])
        proj = self.H @ p
        world = proj[:2] / proj[2]
        return world[0], world[1]

    def _handle_request(self, request, response):
        color_name = request.color_name.strip().lower()
        self.get_logger().info(f"Detection request: color='{color_name}'")

        frame = self._capture_fresh_frame()
        if frame is None:
            self.get_logger().error("Frame capture failed")
            response.found = False
            return response

        undistorted = self._undistort(frame)
        hsv = cv2.cvtColor(undistorted, cv2.COLOR_BGR2HSV)

        detections = self.detect_color(hsv, color_name)
        if not detections:
            self.get_logger().warn(f"No '{color_name}' object detected")
            response.found = False
            return response

        best = max(detections, key=lambda d: d["area"])
        x_mm, y_mm = self._pixel_to_world_mm(best["px"], best["py"])

        # Convert at the boundary only -- everything ROS-facing is metres.
        response.found = True
        response.x = x_mm / 1000.0
        response.y = y_mm / 1000.0

        self.get_logger().info(
            f"Found '{color_name}' -> pixel=({best['px']:.1f},{best['py']:.1f})  "
            f"x={response.x:.4f}m  y={response.y:.4f}m  area={best['area']:.0f}px"
        )
        return response

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


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