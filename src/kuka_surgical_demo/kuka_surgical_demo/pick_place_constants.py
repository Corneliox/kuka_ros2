#!/usr/bin/env python3
"""
pick_place_constants.py

Single source of truth for constants shared between surgical_control_server
and pick_place_coordinator. Import from here in BOTH places -- do not
copy-paste these values again. Several bugs earlier in this project traced
directly back to the same constant (orientation quaternion, units) being
hardcoded independently in multiple files and drifting apart when one got
updated and the other didn't.
"""

# ── Parked / observation pose (2026-07-09 SmartPad ground truth) ─────────────
# The homography (aruco_homography.npy) was solved with the arm at EXACTLY
# this pose. Detection is only valid here -- any other pose invalidates the
# calibration silently (no error, just wrong coordinates). The coordinator
# moves here automatically before every /detect_object call so this is no
# longer a manual "remember to jog there" step.
PARK_X_M = 0.24314
PARK_Y_M = 0.04136
PARK_Z_M = 1.02226
PARK_ORI_X = -0.0080
PARK_ORI_Y = 0.9299
PARK_ORI_Z = -0.0033
PARK_ORI_W = 0.3677

# ── Gripper-calibrated orientation (2026-07-07 SmartPad ground truth) ────────
# Derived from KUKA A=177.59 B=43.63 C=176.76 (deg) -> quaternion.
ORI_X = 0.0090
ORI_Y = 0.9280
ORI_Z = 0.0184
ORI_W = 0.3719
ORIENTATION_TOLERANCE_RAD = 0.2

# ── Fixed pick height ──────────────────────────────────────────────────────
# All cubes are uniform 2x2x2cm -- no per-object height lookup needed.
# This REPLACES the old Z_TABLE + INST_H formula from vision_logic_mock.
PICK_Z_M = 0.073  # metres, base_link frame

# ── Fixed handoff / drop-off pose ─────────────────────────────────────────
# Single constant point -- every object goes to the same place, no per-color
# spacing. If cubes need to be kept apart (stacking/collision risk), revisit
# this later; for now it's one point as specified.
HANDOFF_X_M = 0.3476
HANDOFF_Y_M = -0.7690
HANDOFF_Z_M = 0.1127

# ── Safety / transit ──────────────────────────────────────────────────────
APPROACH_CLEARANCE_M = 0.120
Z_SAFE_M = 0.250

# ── Workspace bounds (unchanged from surgical_control_server) ────────────
WS_X_MIN, WS_X_MAX = 0.250, 0.600
WS_Y_MIN, WS_Y_MAX = -0.860, 0.250
WS_Z_MIN, WS_Z_MAX = 0.030, 0.600