"""Properties side panel: joint tree + HMD info."""

from __future__ import annotations

import math
from typing import Dict

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QColor, QBrush
from PySide6.QtWidgets import (
    QTreeWidget,
    QTreeWidgetItem,
    QGroupBox,
    QFormLayout,
    QLabel,
    QHeaderView,
)

from .constants import (
    JOINT_NAMES,
    BODY_PART_JOINTS,
    EXCLUDED_JOINTS,
    NUM_JOINTS,
    joint_qt_color,
)
from .data_model import AnnotationModel


# =====================================================================
# Joint tree
# =====================================================================
class JointTreeWidget(QTreeWidget):
    """Displays all 32 joints grouped by body part.

    Columns: Name | Conf | Vis | KF | U | V

    Signals
    -------
    visibility_toggled(frame, joint_id, new_visible)
    keyframe_toggled(frame, joint_id, new_is_keyframe)
    """

    visibility_toggled = Signal(int, int, bool)
    keyframe_toggled = Signal(int, int, bool)

    _COL_NAME = 0
    _COL_CONF = 1
    _COL_VIS = 2
    _COL_KF = 3
    _COL_U = 4
    _COL_V = 5

    def __init__(self, model: AnnotationModel, parent=None):
        super().__init__(parent)
        self.model = model
        self._updating = False  # suppress itemChanged during programmatic updates

        self.setHeaderLabels(["Joint", "Conf", "Vis", "KF", "U", "V"])
        self.setColumnCount(6)
        header = self.header()
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        for c in range(1, 6):
            header.setSectionResizeMode(c, QHeaderView.ResizeMode.ResizeToContents)
        self.setRootIsDecorated(True)
        self.setIndentation(16)
        self.setAlternatingRowColors(True)

        self._joint_items: dict[int, QTreeWidgetItem] = {}
        self._build_tree()

        # Connections
        model.session_loaded.connect(self._refresh)
        model.frame_changed.connect(lambda _: self._refresh())
        model.joint_moved.connect(self._on_joint_moved)
        model.joint_selected.connect(self._on_joint_selected)
        model.pruning_changed.connect(self._refresh)
        self.currentItemChanged.connect(self._on_current_changed)
        self.itemChanged.connect(self._on_item_changed)

    # ---- tree construction -------------------------------------------
    def _build_tree(self):
        self.clear()
        self._joint_items.clear()
        for part_name, jids in BODY_PART_JOINTS:
            group = QTreeWidgetItem(self, [part_name])
            group.setFlags(Qt.ItemIsEnabled)
            group.setExpanded(True)
            color = joint_qt_color(jids[0])
            color.setAlpha(40)
            for c in range(self.columnCount()):
                group.setBackground(c, QBrush(color))
            for jid in jids:
                name = JOINT_NAMES[jid] if jid < len(JOINT_NAMES) else f"J{jid}"
                if jid in EXCLUDED_JOINTS:
                    name += " *"
                item = QTreeWidgetItem(group, [name, "", "", "", "", ""])
                item.setData(0, Qt.UserRole, jid)
                item.setFlags(
                    Qt.ItemIsEnabled | Qt.ItemIsSelectable | Qt.ItemIsUserCheckable
                )
                item.setCheckState(self._COL_VIS, Qt.Unchecked)
                item.setCheckState(self._COL_KF, Qt.Unchecked)
                self._joint_items[jid] = item

    # ---- refresh all joints for current frame ------------------------
    _GRAY = QColor(120, 120, 120)

    def _refresh(self):
        self._updating = True
        frame = self.model.current_frame
        for jid in range(NUM_JOINTS):
            item = self._joint_items.get(jid)
            if item is None:
                continue
            pruned = self.model.is_joint_pruned(jid)
            u, v, conf, vis = self.model.get_joint_2d(frame, jid)
            item.setText(self._COL_CONF, str(conf))
            item.setCheckState(
                self._COL_VIS, Qt.Checked if vis else Qt.Unchecked
            )
            is_kf = self.model.has_keyframe_at(frame) and self.model.is_joint_edited(frame, jid)
            # More precise: check if this specific joint has a keyframe here
            is_kf = frame in self.model.get_keyframes(jid)
            item.setCheckState(
                self._COL_KF, Qt.Checked if is_kf else Qt.Unchecked
            )
            item.setText(self._COL_U, f"{u:.1f}")
            item.setText(self._COL_V, f"{v:.1f}")
            # Gray out pruned joints
            fg = QBrush(self._GRAY) if pruned else QBrush()
            for c in range(self.columnCount()):
                item.setForeground(c, fg)
            # Show pruning state in the Conf column
            if pruned:
                item.setText(self._COL_CONF, "pruned")
        self._updating = False

    # ---- incremental joint update ------------------------------------
    def _on_joint_moved(self, frame: int, joint_id: int):
        if frame != self.model.current_frame:
            return
        item = self._joint_items.get(joint_id)
        if item is None:
            return
        self._updating = True
        u, v, conf, vis = self.model.get_joint_2d(frame, joint_id)
        item.setText(self._COL_CONF, str(conf))
        item.setCheckState(self._COL_VIS, Qt.Checked if vis else Qt.Unchecked)
        is_kf = frame in self.model.get_keyframes(joint_id)
        item.setCheckState(self._COL_KF, Qt.Checked if is_kf else Qt.Unchecked)
        item.setText(self._COL_U, f"{u:.1f}")
        item.setText(self._COL_V, f"{v:.1f}")
        self._updating = False

    # ---- selection sync (model ↔ tree) -------------------------------
    def _on_joint_selected(self, joint_id: int):
        if joint_id < 0:
            self.clearSelection()
            return
        item = self._joint_items.get(joint_id)
        if item and item is not self.currentItem():
            self.setCurrentItem(item)

    def _on_current_changed(self, current, _previous):
        if current is None:
            return
        jid = current.data(0, Qt.UserRole)
        if jid is not None:
            self.model.selected_joint = jid

    # ---- checkbox changes (user-initiated) ---------------------------
    def _on_item_changed(self, item: QTreeWidgetItem, column: int):
        if self._updating:
            return
        jid = item.data(0, Qt.UserRole)
        if jid is None:
            return
        frame = self.model.current_frame

        if column == self._COL_VIS:
            new_vis = item.checkState(self._COL_VIS) == Qt.Checked
            self.visibility_toggled.emit(frame, jid, new_vis)

        elif column == self._COL_KF:
            new_kf = item.checkState(self._COL_KF) == Qt.Checked
            self.keyframe_toggled.emit(frame, jid, new_kf)


# =====================================================================
# HMD info panel
# =====================================================================
class HMDInfoPanel(QGroupBox):
    """Read-only display of HMD position, rotation and speed."""

    def __init__(self, model: AnnotationModel, parent=None):
        super().__init__("HMD Info", parent)
        self.model = model

        layout = QFormLayout(self)
        layout.setContentsMargins(6, 10, 6, 6)

        self._pos_label = QLabel("--")
        self._rot_label = QLabel("--")
        self._speed_label = QLabel("--")
        self._status_label = QLabel("No HMD data")

        layout.addRow("Position:", self._pos_label)
        layout.addRow("Rotation:", self._rot_label)
        layout.addRow("Speed:", self._speed_label)
        layout.addRow("Status:", self._status_label)

        model.session_loaded.connect(self._on_session_loaded)
        model.frame_changed.connect(self._on_frame_changed)

    def _on_session_loaded(self):
        if self.model.hmd_data is None:
            self._status_label.setText("No HMD data")
            self._pos_label.setText("--")
            self._rot_label.setText("--")
            self._speed_label.setText("--")
        else:
            self._status_label.setText(f"{len(self.model.hmd_data)} frames")
            self._on_frame_changed(0)

    def _on_frame_changed(self, frame: int):
        hmd = self.model.hmd_data
        if hmd is None:
            return
        idx = hmd.get_idx_by_ratio(frame, self.model.frame_count)

        pos = hmd.get_positions_at(idx)["hmd"]
        self._pos_label.setText(f"({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) m")

        rot = hmd.get_rotations_at(idx)["hmd"]
        euler = self._quat_to_euler(rot)
        self._rot_label.setText(
            f"({euler[0]:.1f}, {euler[1]:.1f}, {euler[2]:.1f}) deg"
        )

        speed = float(hmd.hmd_speed[idx])
        self._speed_label.setText(f"{speed:.3f} m/s")

    @staticmethod
    def _quat_to_euler(q):
        """Convert quaternion (x, y, z, w) to euler angles (pitch, yaw, roll) in degrees."""
        x, y, z, w = q
        # Pitch (x-axis)
        sinp = 2.0 * (w * x - y * z)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)
        # Yaw (y-axis)
        siny_cosp = 2.0 * (w * y + z * x)
        cosy_cosp = 1.0 - 2.0 * (x * x + y * y)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        # Roll (z-axis)
        sinr_cosp = 2.0 * (w * z + x * y)
        cosr_cosp = 1.0 - 2.0 * (y * y + z * z)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        return (math.degrees(pitch), math.degrees(yaw), math.degrees(roll))
