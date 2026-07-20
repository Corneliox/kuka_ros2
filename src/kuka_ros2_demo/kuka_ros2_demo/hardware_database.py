"""
hardware_database.py

Single source of truth for the screw hardware catalogue used by the YOLO
detection path. Deliberately has ZERO dependencies beyond the stdlib --
no cv2, no ultralytics, no rclpy -- so it's safe and cheap for
pick_place_coordinator.py to import for voice-command routing, without
dragging in vision/inference libraries a coordinator node has no business
loading.

vision_detect_node.py imports SCREW_DATABASE, CONFUSED_GROUP, and
PIXELS_PER_MM from here for its classification/disambiguation logic.
pick_place_coordinator.py imports KNOWN_SCREW_CLASSES from here for
voice-command routing.

If you add, rename, or remove a class here, both nodes pick it up
automatically -- no more hand-duplicated lists to drift out of sync.
"""

# ============================================================
# Physical attributes database
#
# Covers all 10 classes actually present in best.pt (confirmed via
# checkpoint inspection). "black_screw" and "screw_2in_small" are
# placeholders -- MEASURE THE REAL HARDWARE and replace these two before
# trusting them for an actual pick.
# ============================================================
SCREW_DATABASE = {
    "screw_2in":        {"length_mm": 50.40, "diameter_mm": 8.10, "threads": 20, "is_silver": False},
    "screw_1_3_4in":    {"length_mm": 43.35, "diameter_mm": 8.35, "threads": 16, "is_silver": False},
    "screw_1in":        {"length_mm": 16.10, "diameter_mm": 7.20, "threads": 6,  "is_silver": True},
    "screw_3_16_1_2in": {"length_mm": 15.15, "diameter_mm": 9.30, "threads": 5,  "is_silver": False},
    "screw_5_32_3_8in": {"length_mm": 12.00, "diameter_mm": 8.30, "threads": 4,  "is_silver": False},
    "screw_5_32_3in":   {"length_mm": 78.20, "diameter_mm": 8.30, "threads": 56, "is_silver": False},
    "screw_1_1_4in":    {"length_mm": 31.25, "diameter_mm": 8.05, "threads": 22, "is_silver": False},
    "screw_5_32_2in":   {"length_mm": 50.80, "diameter_mm": 4.00, "threads": 36, "is_silver": False},
    # --- PLACEHOLDERS -- measure real parts and replace ---
    "black_screw":      {"length_mm": 25.00, "diameter_mm": 8.00, "threads": 10, "is_silver": False},
    "screw_2in_small":  {"length_mm": 48.00, "diameter_mm": 6.00, "threads": 18, "is_silver": False},
}

# Classes visually similar enough that YOLO's raw class guess gets
# double-checked against measured dimensions before being trusted.
CONFUSED_GROUP = [
    "screw_2in", "screw_1_3_4in", "screw_3_16_1_2in", "screw_5_32_3_8in", "screw_1in"
]

# ONLY used to convert mask dimensions (length/diameter) into mm for the
# disambiguation logic in vision_detect_node.py -- NOT used to localize the
# pick point. Position comes entirely from the ArUco homography. Recalibrate
# against your actual camera height/FOV if the disambiguation thresholds
# start missing.
PIXELS_PER_MM = 8.0

# Derived automatically -- this is what pick_place_coordinator.py routes
# voice commands against. Stays correct as long as SCREW_DATABASE is
# correct; nothing to hand-maintain separately.
KNOWN_SCREW_CLASSES = set(SCREW_DATABASE.keys())
