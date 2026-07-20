"""
hardware_database.py

Single source of truth for the voice-command targets used across the
pipeline: known colors (for the HSV/color detection path) and known
hardware classes (for the YOLO detection path). Deliberately has ZERO
dependencies beyond the stdlib -- no cv2, no ultralytics, no rclpy -- so
it's safe and cheap for pick_place_coordinator.py and voice_terminal_mock.py
to import, without dragging in vision/inference libraries those nodes have
no business loading.

vision_detect_node.py imports SCREW_DATABASE, CONFUSED_GROUP,
PIXELS_PER_MM, and NEVER_PICK_CLASSES from here for its
classification/disambiguation logic. pick_place_coordinator.py and
voice_terminal_mock.py import KNOWN_COLORS and KNOWN_SCREW_CLASSES from
here for voice-command routing/menus.

If you add, rename, or remove a class here, every consumer picks it up
automatically -- no more hand-duplicated lists to drift out of sync.

--- v2 fine-tune, 2026-07 ---
This now reflects the fine-tuned model trained on the restricted 6-class
dataset (black_screw, clamp, golden_screw, hinge, marker, washer), not the
original 16-class checkpoint. That earlier model's class names
(screw_5_32_3in, large_washer, metal_hinge, pipe_clamp, etc.) are no
longer routable through this module -- this is a full swap, not an
addition. If both models need to run side by side later, split this into
versioned modules instead of trying to merge two class vocabularies here.
"""

# Known colors for the HSV/color detection path (vision_node.py /
# DetectObject.srv).
KNOWN_COLORS = {"red", "blue", "green", "yellow"}


# ============================================================
# Physical attributes database -- v2 model, 5 pickable classes.
#
# "golden_screw" reuses the measured spec from the old "screw_5_32_3in"
# entry (5/32 x 3in screw) -- that one was real, not a placeholder.
# The other four are carried over as unmeasured placeholder guesses from
# their old counterparts (large_washer -> washer, metal_hinge -> hinge,
# pipe_clamp -> clamp) or from the original inspection (black_screw).
# MEASURE THE REAL HARDWARE and replace these before trusting them for an
# actual pick.
# ============================================================
SCREW_DATABASE = {
    "black_screw":  {"length_mm": 25.00, "diameter_mm": 8.00,  "threads": 10, "is_silver": False},
    "golden_screw": {"length_mm": 78.20, "diameter_mm": 8.30,  "threads": 56, "is_silver": False},
    "washer":       {"length_mm": 25.00, "diameter_mm": 25.00, "threads": 0,  "is_silver": True},
    "hinge":        {"length_mm": 40.00, "diameter_mm": 20.00, "threads": 0,  "is_silver": True},
    "clamp":        {"length_mm": 35.00, "diameter_mm": 25.00, "threads": 0,  "is_silver": True},
}

# No confusable-subtype pairs in this class set -- black_screw and
# golden_screw are already separated by color, not size, so the old
# size-based disambiguation logic isn't needed here. Left empty rather
# than removed, so the resolver's structure still works if a future class
# set reintroduces a genuinely confusable pair.
CONFUSED_GROUP = []

# ONLY used to convert mask dimensions (length/diameter) into mm for the
# disambiguation logic in vision_detect_node.py -- NOT used to localize the
# pick point. Position comes entirely from the ArUco homography. With
# CONFUSED_GROUP empty this value currently has no effect on routing, but
# is kept in case a future class set needs it again.
PIXELS_PER_MM = 8.0

# Classes the model can detect but that must NEVER be treated as a pick
# target, regardless of target_class filtering. "marker" is the ArUco
# calibration tag -- it needs to be recognized (so it doesn't get
# misclassified as hardware) but must be hard-excluded from ever being the
# chosen detection, even on an unfiltered request. Enforced in
# vision_detect_node.py's candidate selection, not just by omission from
# SCREW_DATABASE below.
NEVER_PICK_CLASSES = {"marker"}

# Derived automatically -- this is what pick_place_coordinator.py routes
# voice commands against. Stays correct as long as SCREW_DATABASE is
# correct; nothing to hand-maintain separately. NEVER_PICK_CLASSES is
# deliberately not part of this set, so a voice command for "marker" is
# already rejected as "unknown target" before it ever reaches detection.
KNOWN_SCREW_CLASSES = set(SCREW_DATABASE.keys())