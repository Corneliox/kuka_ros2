#!/usr/bin/env python3
"""
pick_place_constants.py

Single source of truth for constants shared between surgical_control_server
and pick_place_coordinator. Import from here in BOTH places -- do not
copy-paste these values again.
"""

# ── Parked / observation pose (2026-07-13 SmartPad ground truth) ─────────────
PARK_X_M = 0.21490
PARK_Y_M = 0.03466
PARK_Z_M = 0.99458

# Derived from KUKA A=-179.82 B=44.54 C=-179.87 (deg) -> quaternion
PARK_ORI_X = -0.0010
PARK_ORI_Y =  0.9254
PARK_ORI_Z = -0.0005
PARK_ORI_W =  0.3790

# ── Gripper-calibrated orientation (2026-07-07 SmartPad ground truth) ────────
ORI_X = 0.0090
ORI_Y = 0.9280
ORI_Z = 0.0184
ORI_W = 0.3719
ORIENTATION_TOLERANCE_RAD = 0.2

# ── Fixed pick height ──────────────────────────────────────────────────────
PICK_Z_M = 0.073  # metres, base_link frame

# ── Fixed handoff / drop-off pose ─────────────────────────────────────────
HANDOFF_X_M = 0.3476
HANDOFF_Y_M = -0.7690
HANDOFF_Z_M = 0.1127

# ── Safety / transit ──────────────────────────────────────────────────────
APPROACH_CLEARANCE_M = 0.120
Z_SAFE_M = 0.250

# ── Workspace bounds (Extended for closer reach) ──────────────────────────
# WS_X_MIN lowered to 0.010m to allow reaching objects near the base.
# If motion planner complains about IK solution now, we are at the physical
# limit of the arm.
WS_X_MIN, WS_X_MAX = 0.010, 0.600
WS_Y_MIN, WS_Y_MAX = -0.860, 0.250
WS_Z_MIN, WS_Z_MAX = 0.030, 0.600