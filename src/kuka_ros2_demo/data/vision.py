#!/usr/bin/env python3
"""
vision.py

Detects red, blue, green, and yellow 2x2x2cm cubes on the table using
HSV color segmentation (no trained model needed -- fixed, distinct
colors and known cube size make this simpler and more reliable than
YOLO/onnx for this task).

Pipeline:
    1. Load a captured frame (from your fixed observation pose).
    2. Undistort using the wrist camera's calibrated intrinsics.
    3. Threshold in HSV space per color, find contours, filter by area
       to reject noise/background.
    4. Compute each cube's pixel centroid.
    5. Apply the saved homography (aruco_homography.npy, produced by
       aruco_homography_board.py -- SAME FOLDER, run from here) to
       convert each centroid to base-frame (X, Y) in mm.
    6. Print a list of detected objects: color, pixel location, and
       real-world (X, Y) -- ready to feed into your pick-and-place loop.

Usage:
    python3 vision.py /path/to/frame.png

Debug:
    Saves per-color masks and an annotated overlay to this same folder
    so you can tune the HSV ranges below if detection is unreliable at
    first -- lighting and camera white-balance vary a lot between
    setups, so expect to adjust these once against your actual scene.
"""

import sys
import numpy as np
import cv2


# --- Camera intrinsics from default_cam.yaml (wrist camera) ---
CAMERA_MATRIX = np.array([
    [865.3064988411312, 0.0,               257.05723081254752],
    [0.0,               861.37645533726345, 266.08102298248292],
    [0.0,               0.0,               1.0],
])
DIST_COEFFS = np.array([
    0.20180575610957421, -0.16658050726097001,
    0.005021470803111815, -0.022511455420610536, 0.0
])

# Same folder as aruco_homography_board.py's output -- run both scripts
# from the same directory, always.
HOMOGRAPHY_PATH = "aruco_homography.npy"

# --- HSV color ranges (H: 0-179, S/V: 0-255 in OpenCV) ---
# NOTE: these are starting points -- tune against your actual lighting.
# Use the saved mask_<color>.png debug images to adjust.
COLOR_RANGES = {
    "red": [
        (np.array([0,   100, 80]),  np.array([10,  255, 255])),   # low-hue red
        (np.array([170, 100, 80]),  np.array([179, 255, 255])),   # high-hue red (wraps)
    ],
    "yellow": [
        (np.array([20, 100, 80]), np.array([35, 255, 255])),
    ],
    "green": [
        (np.array([40, 80, 60]), np.array([85, 255, 255])),
    ],
    "blue": [
        (np.array([95, 100, 60]), np.array([130, 255, 255])),
    ],
}

# --- Cube size filtering ---
# Adjust these once you know your camera's working distance / pixel scale.
# A 2cm cube's apparent pixel area depends heavily on hover height --
# these are generous bounds to start with; tighten once you see real data.
MIN_CONTOUR_AREA_PX = 150
MAX_CONTOUR_AREA_PX = 20000


def undistort(frame):
    h, w = frame.shape[:2]
    new_K, _ = cv2.getOptimalNewCameraMatrix(CAMERA_MATRIX, DIST_COEFFS, (w, h), 1, (w, h))
    return cv2.undistort(frame, CAMERA_MATRIX, DIST_COEFFS, None, new_K)


def detect_color_cubes(hsv, color_name, ranges):
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lower, upper in ranges:
        mask |= cv2.inRange(hsv, lower, upper)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    cv2.imwrite(f"mask_{color_name}.png", mask)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    detections = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_CONTOUR_AREA_PX or area > MAX_CONTOUR_AREA_PX:
            continue
        M = cv2.moments(cnt)
        if M["m00"] == 0:
            continue
        cx = M["m10"] / M["m00"]
        cy = M["m01"] / M["m00"]
        detections.append({"color": color_name, "px": cx, "py": cy, "area": area})

    return detections


def pixel_to_world(H, px, py):
    p = np.array([px, py, 1.0])
    proj = H @ p
    world = proj[:2] / proj[2]
    return world[0], world[1]


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 vision.py /path/to/frame.png")
        sys.exit(1)

    frame = cv2.imread(sys.argv[1])
    if frame is None:
        print(f"Could not read image: {sys.argv[1]}")
        sys.exit(1)

    try:
        H = np.load(HOMOGRAPHY_PATH)
    except FileNotFoundError:
        print(f"ERROR -- homography file not found at {HOMOGRAPHY_PATH}. "
              f"Run aruco_homography_board.py first, from this same "
              f"folder, to generate it.")
        sys.exit(1)

    undistorted = undistort(frame)
    hsv = cv2.cvtColor(undistorted, cv2.COLOR_BGR2HSV)

    all_detections = []
    for color_name, ranges in COLOR_RANGES.items():
        dets = detect_color_cubes(hsv, color_name, ranges)
        all_detections.extend(dets)

    annotated = undistorted.copy()
    print(f"Detected {len(all_detections)} cube(s):\n")

    for d in all_detections:
        x_mm, y_mm = pixel_to_world(H, d["px"], d["py"])
        d["x_mm"] = x_mm
        d["y_mm"] = y_mm
        print(f"  {d['color']:>6}  pixel=({d['px']:6.1f},{d['py']:6.1f})  "
              f"area={d['area']:6.0f}px  ->  X={x_mm:7.1f}mm  Y={y_mm:7.1f}mm")

        color_bgr = {
            "red": (0, 0, 255), "blue": (255, 0, 0),
            "green": (0, 255, 0), "yellow": (0, 255, 255),
        }[d["color"]]
        cv2.circle(annotated, (int(d["px"]), int(d["py"])), 8, color_bgr, 2)
        cv2.putText(annotated, d["color"], (int(d["px"]) + 10, int(d["py"])),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_bgr, 2)

    cv2.imwrite("vision_debug.png", annotated)
    print(f"\nSaved annotated debug image to vision_debug.png")
    print(f"Saved per-color masks to mask_<color>.png for tuning")

    if not all_detections:
        print("\nNo cubes detected -- check mask_*.png to see if your "
              "HSV ranges need adjusting for this lighting.")


if __name__ == "__main__":
    main()