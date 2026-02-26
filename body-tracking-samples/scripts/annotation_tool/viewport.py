"""QGraphicsView viewport with draggable joint items and bone connections."""

from __future__ import annotations

from typing import Dict, List, Optional

from PySide6.QtCore import Qt, Signal, QPointF
from PySide6.QtGui import QColor, QPen, QBrush, QPainter, QPixmap
from PySide6.QtWidgets import (
    QGraphicsView,
    QGraphicsScene,
    QGraphicsEllipseItem,
    QGraphicsLineItem,
    QGraphicsPixmapItem,
    QGraphicsItem,
)

from .constants import (
    NUM_JOINTS,
    BONE_CONNECTIONS,
    EXCLUDED_JOINTS,
    CONFIDENCE_RADIUS,
    JOINT_NAMES,
    joint_qt_color,
    bone_qt_color,
    get_joint_part,
    PART_COLORS_QT,
)
from .data_model import AnnotationModel


# =====================================================================
# Joint item (draggable circle)
# =====================================================================
class JointItem(QGraphicsEllipseItem):
    """Draggable circle representing one skeleton joint."""

    BASE_RADIUS = 7

    def __init__(self, joint_id: int, viewport: "AnnotationViewport"):
        r = self.BASE_RADIUS
        super().__init__(-r, -r, 2 * r, 2 * r)
        self.joint_id = joint_id
        self.viewport = viewport
        self._updating = False  # suppress itemChange during programmatic moves

        self.setFlag(QGraphicsItem.ItemIsMovable, True)
        self.setFlag(QGraphicsItem.ItemIsSelectable, True)
        self.setFlag(QGraphicsItem.ItemSendsGeometryChanges, True)
        self.setCursor(Qt.PointingHandCursor)
        self.setZValue(2)

        # Default colour
        color = joint_qt_color(joint_id)
        self.setBrush(QBrush(color))
        self.setPen(QPen(Qt.white, 1.5))

        # Cached state for refresh
        self._confidence = 0
        self._visible = False
        self._excluded = joint_id in EXCLUDED_JOINTS

    # ---- appearance --------------------------------------------------
    def update_appearance(self, confidence: int, visible: bool):
        self._confidence = confidence
        self._visible = visible
        self._refresh_style()

    def _refresh_style(self):
        if self._excluded:
            self.setVisible(False)
            return

        color = joint_qt_color(self.joint_id)
        if not self._visible or self._confidence == 0:
            color.setAlpha(80)
        else:
            color.setAlpha(220)
        self.setBrush(QBrush(color))

        r = CONFIDENCE_RADIUS.get(self._confidence, 3) + 2
        self.prepareGeometryChange()
        self.setRect(-r, -r, 2 * r, 2 * r)

        if self.isSelected():
            self.setPen(QPen(QColor(255, 255, 0), 2.5))
        else:
            self.setPen(QPen(Qt.white, 1.5))

        self.setVisible(True)

    # ---- programmatic position update --------------------------------
    def set_position_from_model(self, u: float, v: float):
        self._updating = True
        self.setPos(u, v)
        self._updating = False

    # ---- interaction -------------------------------------------------
    def mousePressEvent(self, event):
        # Let Qt handle selection logic (Ctrl-toggle, deselect others, etc.)
        super().mousePressEvent(event)
        if event.button() == Qt.LeftButton:
            # Snapshot positions of ALL currently selected joints so that a
            # group drag is recorded as a single compound undo step.
            self.viewport.snapshot_group_drag_start()

    def mouseReleaseEvent(self, event):
        super().mouseReleaseEvent(event)
        if event.button() == Qt.LeftButton:
            self.viewport.finalize_group_drag()

    def itemChange(self, change, value):
        if change == QGraphicsItem.ItemPositionHasChanged and not self._updating:
            self.viewport.update_bones_for_joint(self.joint_id)
        if change == QGraphicsItem.ItemSelectedHasChanged:
            if value:
                self.viewport.model.selected_joint = self.joint_id
            self._refresh_style()
        return super().itemChange(change, value)


# =====================================================================
# Bone item (line between two joints)
# =====================================================================
class BoneItem(QGraphicsLineItem):
    """Line connecting two JointItems."""

    def __init__(self, parent_id: int, child_id: int):
        super().__init__()
        self.parent_id = parent_id
        self.child_id = child_id
        color = bone_qt_color(parent_id, child_id)
        color.setAlpha(200)
        self.setPen(QPen(color, 2.5))
        self.setZValue(1)

    def update_from_joints(self, joint_items: dict):
        p = joint_items.get(self.parent_id)
        c = joint_items.get(self.child_id)
        if p is None or c is None or not p.isVisible() or not c.isVisible():
            self.setVisible(False)
            return
        self.setVisible(True)
        pp = p.pos()
        cp = c.pos()
        self.setLine(pp.x(), pp.y(), cp.x(), cp.y())


# =====================================================================
# Main viewport
# =====================================================================
class AnnotationViewport(QGraphicsView):
    """Interactive viewport: image background + draggable skeleton overlay.

    Left-drag on empty space draws a rubber-band to select multiple joints.
    Left-drag on a joint (or group of selected joints) moves them together.
    Middle-drag pans the view.  Mouse wheel zooms.
    """

    # Emitted once on mouse-release after a drag.  Payload is a list of
    # (joint_id, old_u, old_v, new_u, new_v) tuples -- one entry per
    # joint that actually moved.
    group_drag_finished = Signal(list)

    def __init__(self, model: AnnotationModel, parent=None):
        super().__init__(parent)
        self.model = model
        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)

        self._bg_item: Optional[QGraphicsPixmapItem] = None
        self._joint_items: Dict[int, JointItem] = {}
        self._bone_items: List[BoneItem] = []
        self._pan_active = False
        self._pan_last: Optional[QPointF] = None

        # Positions snapshotted at the start of a group drag.
        # Maps joint_id -> (u, v) at the moment the mouse was pressed.
        self._drag_start_positions: Dict[int, tuple] = {}

        # Rendering
        self.setRenderHints(QPainter.Antialiasing | QPainter.SmoothPixmapTransform)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorUnderMouse)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.setBackgroundBrush(QBrush(QColor(40, 40, 40)))

        # Rubber-band selection: left-drag on empty space selects joints;
        # left-drag on an item still moves it (Qt sends events to the item).
        self.setDragMode(QGraphicsView.RubberBandDrag)

        # Connect to model
        model.session_loaded.connect(self._on_session_loaded)
        model.frame_changed.connect(self._on_frame_changed)
        model.joint_moved.connect(self._on_joint_moved)
        model.extrinsic_preview_changed.connect(self._on_frame_changed)
        model.pruning_changed.connect(
            lambda: self._load_frame(self.model.current_frame)
        )

    # ---- setup -------------------------------------------------------
    def _on_session_loaded(self):
        self._create_items()
        self._load_frame(0)
        self.fit_to_view()

    def _create_items(self):
        self._scene.clear()
        self._joint_items.clear()
        self._bone_items.clear()

        # Background image
        self._bg_item = self._scene.addPixmap(QPixmap())
        self._bg_item.setZValue(-1)

        # Bones (drawn under joints)
        for pid, cid in BONE_CONNECTIONS:
            bone = BoneItem(pid, cid)
            self._scene.addItem(bone)
            self._bone_items.append(bone)

        # Joints
        for jid in range(NUM_JOINTS):
            item = JointItem(jid, self)
            self._scene.addItem(item)
            self._joint_items[jid] = item

    # ---- frame updates -----------------------------------------------
    def _on_frame_changed(self, frame: int):
        self._load_frame(frame)

    def _load_frame(self, frame: int):
        if self.model.dataset is None:
            return

        # Background pixmap
        pix = self.model.get_image_pixmap(frame)
        if not pix.isNull():
            self._bg_item.setPixmap(pix)
            self._scene.setSceneRect(0, 0, pix.width(), pix.height())

        has_skel = self.model.dataset.get_joints_2d(frame) is not None
        has_extrinsic = self.model.has_extrinsic_delta()

        for jid in range(NUM_JOINTS):
            item = self._joint_items.get(jid)
            if item is None:
                continue
            if has_skel:
                if has_extrinsic:
                    u, v, conf, vis = self.model.get_adjusted_joint_2d(frame, jid)
                else:
                    u, v, conf, vis = self.model.get_joint_2d(frame, jid)
                item.set_position_from_model(u, v)
                item.update_appearance(conf, vis)
                if self.model.is_joint_pruned(jid):
                    item.setVisible(False)
            else:
                item.setVisible(False)

        for bone in self._bone_items:
            if has_skel:
                bone.update_from_joints(self._joint_items)
            else:
                bone.setVisible(False)

    def _on_joint_moved(self, frame: int, joint_id: int):
        if frame != self.model.current_frame:
            return
        u, v, conf, vis = self.model.get_joint_2d(frame, joint_id)
        item = self._joint_items.get(joint_id)
        if item is None:
            return
        item.set_position_from_model(u, v)
        item.update_appearance(conf, vis)
        self.update_bones_for_joint(joint_id)

    # ---- bone helpers ------------------------------------------------
    def update_bones_for_joint(self, joint_id: int):
        for bone in self._bone_items:
            if bone.parent_id == joint_id or bone.child_id == joint_id:
                bone.update_from_joints(self._joint_items)

    # ---- group drag tracking (called by JointItem) -------------------
    def snapshot_group_drag_start(self):
        """Record pre-drag positions of every currently selected joint."""
        self._drag_start_positions.clear()
        for jid, item in self._joint_items.items():
            if item.isSelected():
                pos = item.pos()
                self._drag_start_positions[jid] = (pos.x(), pos.y())

    def finalize_group_drag(self):
        """Compare current positions to the snapshot and emit moves."""
        if not self._drag_start_positions:
            return
        moves = []
        for jid, (old_u, old_v) in self._drag_start_positions.items():
            item = self._joint_items.get(jid)
            if item is None:
                continue
            new_pos = item.pos()
            new_u, new_v = new_pos.x(), new_pos.y()
            if abs(new_u - old_u) > 0.01 or abs(new_v - old_v) > 0.01:
                moves.append((jid, old_u, old_v, new_u, new_v))
        self._drag_start_positions.clear()
        if moves:
            self.group_drag_finished.emit(moves)

    def get_selected_joint_ids(self) -> List[int]:
        """Return joint IDs of all currently selected (highlighted) joints."""
        return [jid for jid, item in self._joint_items.items()
                if item.isSelected()]

    # ---- zoom --------------------------------------------------------
    def wheelEvent(self, event):
        factor = 1.15 if event.angleDelta().y() > 0 else 1.0 / 1.15
        self.scale(factor, factor)

    # ---- pan (middle mouse) ------------------------------------------
    def mousePressEvent(self, event):
        if event.button() == Qt.MiddleButton:
            self._pan_active = True
            self._pan_last = event.position()
            self.setCursor(Qt.ClosedHandCursor)
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self._pan_active and self._pan_last is not None:
            delta = event.position() - self._pan_last
            self._pan_last = event.position()
            self.horizontalScrollBar().setValue(
                self.horizontalScrollBar().value() - int(delta.x()))
            self.verticalScrollBar().setValue(
                self.verticalScrollBar().value() - int(delta.y()))
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MiddleButton:
            self._pan_active = False
            self._pan_last = None
            self.setCursor(Qt.ArrowCursor)
            event.accept()
            return
        super().mouseReleaseEvent(event)

    # ---- fit view ----------------------------------------------------
    def fit_to_view(self):
        if self._bg_item and not self._bg_item.pixmap().isNull():
            self.fitInView(self._scene.sceneRect(), Qt.KeepAspectRatio)
