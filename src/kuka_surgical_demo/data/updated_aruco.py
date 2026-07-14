#!/usr/bin/env python3
"""
updated_aruco.py

Plain OpenCV/numpy script -- no ROS2 required. Takes a saved image file,
undistorts it using your calibrated intrinsics, applies a preprocessing
step (unsharp mask + loosened detector params) to recover marginally
out-of-focus markers, detects ArUco markers (DICT_5X5_50), and solves
the pixel -> base-frame (X, Y) homography against your verified
ground-truth marker positions.

Changes vs. aruco_homography.py:
  - Preprocessing: unsharp mask applied before detection (recovered 1 extra
    marker in testing -- id=5 -- going from 3 to 4 detected on a test frame).
    This is a modest, not miraculous, improvement: it cannot recover markers
    that are genuinely out of the lens's depth-of-field. Root cause (mat
    flatness / mount / focus distance) should still be investigated.
  - Loosened ArUco detector parameters (wider adaptive threshold window
    range, lower minimum marker perimeter rate, subpixel corner refinement)
    to tolerate the added noise from sharpening.
  - Degeneracy guard: warns if the matched marker set has too little spread
    in X or Y, since a homography fit from near-collinear points can report
    a perfect (0.00mm) reprojection error while actually being unreliable
    off that line. This was a real failure mode seen in earlier runs.

Usage:
    python3 updated_aruco.py /path/to/frame.png
    python3 updated_aruco.py /path/to/frame.png --no-preprocess   # baseline behavior
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

# --- Ground-truth marker positions (base frame, mm) ---
MARKER_POINTS = [
    {"id": 0,  "x_mm": 303.11,   "y_mm": 299.80},
    {"id": 1,  "x_mm": 112.55,   "y_mm": -344.22},
    {"id": 2,  "x_mm": 157.78,   "y_mm": -201.06},
    {"id": 3,  "x_mm": 115.11,   "y_mm": 342.09},
    {"id": 4,  "x_mm": 522.11,   "y_mm": 337.59},
    {"id": 5,  "x_mm": 521.76,   "y_mm": 6.73},
    {"id": 6,  "x_mm": 527.19,   "y_mm": -337.73},
    {"id": 7,  "x_mm": 298.08, "y_mm": -344.08},
    {"id": 8,  "x_mm": 293.85,   "y_mm": 9.34},
    {"id": 9,  "x_mm": 216.57,   "y_mm": 174.20},
    {"id": 10, "x_mm": 422.84,   "y_mm": 174.79},
    {"id": 11, "x_mm": 298.34,   "y_mm": 176.92},
]

ARUCO_DICT = cv2.aruco.DICT_5X5_50

# Minimum spread (mm) required in X and Y among matched ground-truth points
# before we trust the homography solve. Points clustered/collinear below
# this threshold can still "solve" with 4 points and report 0.00mm error
# while being unreliable everywhere off that line.
MIN_SPREAD_MM = 50.0


def unsharp_mask(img, amount=1.5, radius=3):
    """Sharpen edges to partially recover mildly out-of-focus marker borders."""
    blurred = cv2.GaussianBlur(img, (0, 0), radius)
    return cv2.addWeighted(img, 1 + amount, blurred, -amount, 0)


def loose_detector_params():
    """Detector params loosened to tolerate sharpening noise and smaller/softer
    markers than the OpenCV defaults assume."""
    p = cv2.aruco.DetectorParameters()
    p.adaptiveThreshWinSizeMin = 3
    p.adaptiveThreshWinSizeMax = 53
    p.adaptiveThreshWinSizeStep = 4
    p.minMarkerPerimeterRate = 0.01
    p.polygonalApproxAccuracyRate = 0.05
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return p


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 updated_aruco.py /path/to/frame.png [--no-preprocess]")
        sys.exit(1)

    use_preprocess = "--no-preprocess" not in sys.argv

    frame = cv2.imread(sys.argv[1])
    if frame is None:
        print(f"Could not read image: {sys.argv[1]}")
        sys.exit(1)

    # --- Undistort ---
    h, w = frame.shape[:2]
    new_K, roi = cv2.getOptimalNewCameraMatrix(CAMERA_MATRIX, DIST_COEFFS, (w, h), 1, (w, h))
    undistorted = cv2.undistort(frame, CAMERA_MATRIX, DIST_COEFFS, None, new_K)

    # --- Preprocess (optional) ---
    if use_preprocess:
        detect_img = unsharp_mask(undistorted)
        parameters = loose_detector_params()
        print("Preprocessing: unsharp mask + loosened detector params (use --no-preprocess to disable)")
    else:
        detect_img = undistorted
        parameters = cv2.aruco.DetectorParameters()
        print("Preprocessing disabled -- using baseline detection")

    # --- Detect markers ---
    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)
    corners, ids, rejected = detector.detectMarkers(detect_img)

    # Annotate on the undistorted (non-sharpened) image for a clean debug view
    annotated = undistorted.copy()
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(annotated, corners, ids)
    cv2.imwrite("aruco_debug.png", annotated)
    print("Saved annotated debug image to aruco_debug.png")

    if ids is None:
        print("NO markers detected at all. Check camera focus, lighting, "
              "and that markers are in frame.")
        sys.exit(1)

    detected_ids = ids.flatten().tolist()
    centroids = {}
    for i, marker_id in enumerate(detected_ids):
        c = corners[i][0]  # 4x2 array of corner points
        cx, cy = c[:, 0].mean(), c[:, 1].mean()
        centroids[marker_id] = (cx, cy)

    print(f"Detected {len(detected_ids)} marker(s): {sorted(detected_ids)}")
    for mid, (cx, cy) in sorted(centroids.items()):
        print(f"  ID {mid:2d} -> pixel ({cx:7.2f}, {cy:7.2f})")

    # --- Check for missing expected markers ---
    expected_ids = {m["id"] for m in MARKER_POINTS}
    found_ids = set(centroids.keys())
    missing = expected_ids - found_ids
    extra = found_ids - expected_ids
    if missing:
        print(f"WARNING -- Missing expected marker IDs: {sorted(missing)}")
    if extra:
        print(f"WARNING -- Detected unexpected marker IDs (not in ground truth): {sorted(extra)}")

    # --- Solve homography using whatever matched pairs we have ---
    matched = [m for m in MARKER_POINTS if m["id"] in centroids]
    if len(matched) < 4:
        print(f"ERROR -- Only {len(matched)} matched marker(s) -- need at least 4 "
              f"to solve a homography. Aborting.")
        sys.exit(1)

    img_pts = np.array([centroids[m["id"]] for m in matched], dtype=np.float32)
    world_pts = np.array([[m["x_mm"], m["y_mm"]] for m in matched], dtype=np.float32)

    # --- Degeneracy guard ---
    x_spread = world_pts[:, 0].max() - world_pts[:, 0].min()
    y_spread = world_pts[:, 1].max() - world_pts[:, 1].min()
    if x_spread < MIN_SPREAD_MM or y_spread < MIN_SPREAD_MM:
        print(f"WARNING -- Matched ground-truth points have narrow spread "
              f"(X: {x_spread:.0f}mm, Y: {y_spread:.0f}mm, min required: {MIN_SPREAD_MM:.0f}mm). "
              f"Homography may be poorly constrained (e.g. near-collinear points) even if "
              f"reprojection error looks low. Try to include markers off the y=0 baseline row.")

    H, mask = cv2.findHomography(img_pts, world_pts, cv2.RANSAC, 5.0)

    if H is None:
        print("ERROR -- Homography solve failed.")
        sys.exit(1)

    inliers = int(mask.sum())
    print(f"Homography solved using {inliers}/{len(matched)} inlier points.")
    print(f"H =\n{H}")

    # --- Reprojection error check ---
    errors = []
    for i, m in enumerate(matched):
        px = np.array([img_pts[i][0], img_pts[i][1], 1.0])
        proj = H @ px
        proj_xy = proj[:2] / proj[2]
        true_xy = world_pts[i]
        err = np.linalg.norm(proj_xy - true_xy)
        errors.append(err)
        print(f"  ID {m['id']:2d}: true=({true_xy[0]:.1f},{true_xy[1]:.1f})mm  "
              f"reproj=({proj_xy[0]:.1f},{proj_xy[1]:.1f})mm  err={err:.2f}mm")

    print(f"\nReprojection error -- mean: {np.mean(errors):.2f}mm  max: {np.max(errors):.2f}mm")
    if len(matched) == 4:
        print("NOTE -- Exactly 4 points were used, so this homography fits them exactly "
              "(0.00mm error is expected and does NOT by itself indicate a good/reliable fit). "
              "Add more matched markers with good spread for a meaningful residual check.")

    np.save("aruco_homography.npy", H)
    print("Saved homography matrix to aruco_homography.npy")


if __name__ == "__main__":
    main()
