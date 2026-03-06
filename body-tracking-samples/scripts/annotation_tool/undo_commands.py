"""QUndoCommand subclasses for the annotation tool."""

from PySide6.QtGui import QUndoCommand

from .constants import JOINT_NAMES


class MoveJointCommand(QUndoCommand):
    """Move a single joint to a new (u, v) position."""

    def __init__(self, model, frame, joint_id, old_u, old_v, new_u, new_v):
        name = JOINT_NAMES[joint_id] if joint_id < len(JOINT_NAMES) else f"Joint {joint_id}"
        super().__init__(f"Move {name} @ frame {frame}")
        self._model = model
        self._frame = frame
        self._jid = joint_id
        self._old_u, self._old_v = old_u, old_v
        self._new_u, self._new_v = new_u, new_v
        self._undo_snapshot = None  # captured on first redo()

    def redo(self):
        # Snapshot 3D state before IK runs
        self._undo_snapshot = self._model.snapshot_edits_3d(self._frame)
        self._model.set_joint_2d(self._frame, self._jid, self._new_u, self._new_v)

    def undo(self):
        self._model.set_joint_2d(self._frame, self._jid, self._old_u, self._old_v)
        # Restore 3D state to clean up IK side effects
        if self._undo_snapshot is not None:
            values, existing = self._undo_snapshot
            self._model.restore_edits_3d(self._frame, values, existing)


class ToggleVisibilityCommand(QUndoCommand):
    """Toggle the visible flag for a joint."""

    def __init__(self, model, frame, joint_id, old_visible, new_visible):
        name = JOINT_NAMES[joint_id] if joint_id < len(JOINT_NAMES) else f"Joint {joint_id}"
        state = "Show" if new_visible else "Hide"
        super().__init__(f"{state} {name} @ frame {frame}")
        self._model = model
        self._frame = frame
        self._jid = joint_id
        self._old = old_visible
        self._new = new_visible

    def redo(self):
        self._model.set_joint_visible(self._frame, self._jid, self._new)

    def undo(self):
        self._model.set_joint_visible(self._frame, self._jid, self._old)


class BatchVisibilityCommand(QUndoCommand):
    """Toggle visible flag for one joint across a range of frames."""

    def __init__(self, model, joint_id, changes):
        """
        Args:
            changes: list of (frame, old_visible, new_visible)
        """
        name = JOINT_NAMES[joint_id] if joint_id < len(JOINT_NAMES) else f"Joint {joint_id}"
        state = "Show" if changes[0][2] else "Hide"
        super().__init__(f"{state} {name} frames {changes[0][0]}-{changes[-1][0]}")
        self._model = model
        self._jid = joint_id
        self._changes = changes

    def redo(self):
        for frame, _old, new in self._changes:
            self._model.set_joint_visible(frame, self._jid, new)

    def undo(self):
        for frame, old, _new in reversed(self._changes):
            self._model.set_joint_visible(frame, self._jid, old)


class BatchMultiJointVisibilityCommand(QUndoCommand):
    """Toggle visible flag for multiple joints across one or more frames."""

    def __init__(self, model, changes, description=None):
        """
        Args:
            changes: list of (frame, joint_id, old_visible, new_visible)
        """
        if description:
            super().__init__(description)
        else:
            super().__init__(f"Toggle visibility ({len(changes)} changes)")
        self._model = model
        self._changes = changes

    def redo(self):
        for frame, jid, _old, new in self._changes:
            self._model.set_joint_visible(frame, jid, new)

    def undo(self):
        for frame, jid, old, _new in reversed(self._changes):
            self._model.set_joint_visible(frame, jid, old)


class SetKeyframeCommand(QUndoCommand):
    """Toggle the keyframe flag for a joint at a given frame."""

    def __init__(self, model, frame, joint_id, is_keyframe):
        name = JOINT_NAMES[joint_id] if joint_id < len(JOINT_NAMES) else f"Joint {joint_id}"
        action = "Set" if is_keyframe else "Unset"
        super().__init__(f"{action} keyframe {name} @ frame {frame}")
        self._model = model
        self._frame = frame
        self._jid = joint_id
        self._new_kf = is_keyframe

    def redo(self):
        self._model.set_keyframe(self._frame, self._jid, self._new_kf)

    def undo(self):
        self._model.set_keyframe(self._frame, self._jid, not self._new_kf)


class MoveMultipleJointsCommand(QUndoCommand):
    """Move several joints at the same frame (group drag / multi-reset)."""

    def __init__(self, model, frame, moves):
        """
        Args:
            moves: list of (joint_id, old_u, old_v, new_u, new_v)
        """
        super().__init__(f"Move {len(moves)} joints @ frame {frame}")
        self._model = model
        self._frame = frame
        self._moves = moves
        self._undo_snapshot = None

    def redo(self):
        self._undo_snapshot = self._model.snapshot_edits_3d(self._frame)
        for jid, _old_u, _old_v, new_u, new_v in self._moves:
            self._model.set_joint_2d(self._frame, jid, new_u, new_v)

    def undo(self):
        for jid, old_u, old_v, _new_u, _new_v in reversed(self._moves):
            self._model.set_joint_2d(self._frame, jid, old_u, old_v)
        if self._undo_snapshot is not None:
            values, existing = self._undo_snapshot
            self._model.restore_edits_3d(self._frame, values, existing)


class BatchMoveCommand(QUndoCommand):
    """Apply interpolated positions to many frames at once (compound)."""

    def __init__(self, model, joint_id, moves):
        """
        Args:
            moves: list of (frame, old_u, old_v, new_u, new_v)
        """
        name = JOINT_NAMES[joint_id] if joint_id < len(JOINT_NAMES) else f"Joint {joint_id}"
        super().__init__(f"Interpolate {name} ({len(moves)} frames)")
        self._model = model
        self._jid = joint_id
        self._moves = moves
        self._undo_snapshots = {}  # {frame: (values, existing)}

    def redo(self):
        for frame, _old_u, _old_v, new_u, new_v in self._moves:
            self._undo_snapshots[frame] = self._model.snapshot_edits_3d(frame)
            self._model.set_joint_2d(frame, self._jid, new_u, new_v)

    def undo(self):
        for frame, old_u, old_v, _new_u, _new_v in reversed(self._moves):
            self._model.set_joint_2d(frame, self._jid, old_u, old_v)
            if frame in self._undo_snapshots:
                values, existing = self._undo_snapshots[frame]
                self._model.restore_edits_3d(frame, values, existing)
