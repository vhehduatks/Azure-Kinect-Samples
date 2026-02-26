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

    def redo(self):
        self._model.set_joint_2d(self._frame, self._jid, self._new_u, self._new_v)

    def undo(self):
        self._model.set_joint_2d(self._frame, self._jid, self._old_u, self._old_v)


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

    def redo(self):
        for jid, _old_u, _old_v, new_u, new_v in self._moves:
            self._model.set_joint_2d(self._frame, jid, new_u, new_v)

    def undo(self):
        for jid, old_u, old_v, _new_u, _new_v in reversed(self._moves):
            self._model.set_joint_2d(self._frame, jid, old_u, old_v)


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

    def redo(self):
        for frame, _old_u, _old_v, new_u, new_v in self._moves:
            self._model.set_joint_2d(frame, self._jid, new_u, new_v)

    def undo(self):
        for frame, old_u, old_v, _new_u, _new_v in reversed(self._moves):
            self._model.set_joint_2d(frame, self._jid, old_u, old_v)
