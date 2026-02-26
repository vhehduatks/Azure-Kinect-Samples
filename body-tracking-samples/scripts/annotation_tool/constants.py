"""Constants and color utilities for the annotation tool.

Re-exports from visualize_ego_dataset / visualize_batch_ego_dataset
and adds PySide6 QColor conversions.
"""

import sys
from pathlib import Path
from PySide6.QtGui import QColor

# Ensure parent scripts directory is on sys.path
_script_dir = str(Path(__file__).resolve().parent.parent)
if _script_dir not in sys.path:
    sys.path.insert(0, _script_dir)

from visualize_ego_dataset import (
    JOINT_NAMES,
    BONE_CONNECTIONS,
    EXCLUDED_JOINTS,
    PART_COLORS_RGB,
    get_joint_part,
    get_bone_part,
    CONFIDENCE_RADIUS,
    EgoDataset,
)
from visualize_batch_ego_dataset import HMDData, discover_batch_sessions

NUM_JOINTS = 32

# Body-part name -> QColor
PART_COLORS_QT = {
    k: QColor(int(r * 255), int(g * 255), int(b * 255))
    for k, (r, g, b) in PART_COLORS_RGB.items()
}


def joint_qt_color(joint_id: int) -> QColor:
    """Return a QColor for the body part this joint belongs to."""
    return QColor(PART_COLORS_QT[get_joint_part(joint_id)])


def bone_qt_color(parent_id: int, child_id: int) -> QColor:
    """Return a QColor for the body part this bone belongs to."""
    return QColor(PART_COLORS_QT[get_bone_part(parent_id, child_id)])


# Ordered body-part groupings for the properties tree widget
BODY_PART_JOINTS = [
    ("Spine", [0, 1, 2, 3]),
    ("Head", [26, 27, 28, 29, 30, 31]),
    ("Left Arm", [4, 5, 6, 7, 8, 9, 10]),
    ("Right Arm", [11, 12, 13, 14, 15, 16, 17]),
    ("Left Leg", [18, 19, 20, 21]),
    ("Right Leg", [22, 23, 24, 25]),
]
