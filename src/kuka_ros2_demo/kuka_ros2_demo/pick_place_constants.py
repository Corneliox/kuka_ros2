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

# ── Parked / observation pose (2026-07-27 SmartPad ground truth, latest) ────
# The homography (aruco_homography.npy) was solved with the arm at EXACTLY
# this pose. Detection is only valid here -- any other pose invalidates the
# calibration silently (no error, just wrong coordinates). The coordinator
# moves here automatically before every /detect_object call so this is no
# longer a manual "remember to jog there" step.
#
# Cartesian target -- park move goes through MoveIt's Cartesian position +
# orientation constraint, same as every other motion in this pipeline.
#
# Re-run aruco_homography.npy with the arm at THIS pose before trusting
# detections -- any park pose change invalidates the prior calibration.
#
# From SmartPad "Actual position" readout (base frame):
#   X=338.56mm  Y=10.12mm  Z=1091.51mm  A=-173.42°  B=28.68°  C=-179.71°
PARK_X_M = 0.33856
PARK_Y_M = 0.01012
PARK_Z_M = 1.09151
PARK_ORI_X = -0.0550
PARK_ORI_Y = 0.9673
PARK_ORI_Z = 0.0118
PARK_ORI_W = 0.2474

# ── Gripper-calibrated orientation (2026-07-27 SmartPad ground truth) ────────
# Same SmartPad readout as PARK_ORI above (A=-173.42 B=28.68 C=-179.71) --
# pick orientation set to match park orientation per explicit instruction.
ORI_X = -0.0550
ORI_Y = 0.9673
ORI_Z = 0.0118
ORI_W = 0.2474
ORIENTATION_TOLERANCE_RAD = 0.2

# ── Default pick height for color cubes ───────────────────────────────────
# Color cubes are the current default suction-gripper target with a fixed
# height of 60 mm. Other object classes can override this with their own
# specific pick height below.
PICK_Z_M = 0.08200  # metres, base_link frame
PICK_Z_M_CLAMP = 0.07915  # metres, base_link frame
PICK_Z_M_WASHER = 0.05631  # metres, base_link frame


def get_pick_z_for_object(object_name: str) -> float:
    """Return the appropriate pick height for a known object class."""
    name = (object_name or '').strip().lower()
    if name.endswith('_cube'):
        return PICK_Z_M
    if 'clamp' in name:
        return PICK_Z_M_CLAMP
    if 'washer' in name:
        return PICK_Z_M_WASHER
    return PICK_Z_M

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

# ── Workspace bounds ──────────────────────────────────────────────────────
# Updated 2026-07-XX to match the actual physical table/detection region
# (X: 0 to 59cm from robot base, Y: +-37cm), replacing the old bounds that
# were sized around the original green instrument mat + a stretch to reach
# the handoff point. See note below re: this being a coarse sanity check,
# NOT a substitute for the real reachability limits already characterized
# in axis_sweep.py.
WS_X_MIN, WS_X_MAX = 0.00, 0.59
WS_Y_MIN, WS_Y_MAX = -0.37, 0.37
WS_Z_MIN, WS_Z_MAX = 0.030, 0.600