"""AnnotationModel -- sparse edit overlay on top of a read-only EgoDataset."""

import json
import math
import shutil
from collections import OrderedDict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

import numpy as np
from PySide6.QtCore import QObject, Signal
from PySide6.QtGui import QPixmap

from .constants import EgoDataset, HMDData, NUM_JOINTS


# ======================================================================
# Geometry helpers
# ======================================================================
def _euler_to_rotation_matrix(rx_deg: float, ry_deg: float, rz_deg: float) -> np.ndarray:
    """Convert Euler angles (degrees, XYZ order) to a 3x3 rotation matrix."""
    rx = math.radians(rx_deg)
    ry = math.radians(ry_deg)
    rz = math.radians(rz_deg)
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    # R = Rz * Ry * Rx
    return np.array([
        [cy * cz, sx * sy * cz - cx * sz, cx * sy * cz + sx * sz],
        [cy * sz, sx * sy * sz + cx * cz, cx * sy * sz - sx * cz],
        [-sy,     sx * cy,                cx * cy],
    ], dtype=np.float64)


def _project_pinhole(x: float, y: float, z: float,
                     fx: float, fy: float, cx: float, cy: float) -> Tuple[float, float]:
    """Pinhole projection: (x,y,z) -> (u,v).  Returns (0,0) if z <= 0."""
    if z <= 0:
        return (0.0, 0.0)
    return (fx * x / z + cx, fy * y / z + cy)


@dataclass
class JointEdit:
    """One joint's edited state at a specific frame."""

    u: float
    v: float
    visible: bool
    is_keyframe: bool = False
    original_u: float = 0.0
    original_v: float = 0.0
    original_visible: bool = True


class AnnotationModel(QObject):
    """Central data model wrapping an EgoDataset with a sparse edit layer.

    Signals
    -------
    frame_changed(int)       -- current frame index changed
    joint_moved(int, int)    -- (frame, joint_id) after set_joint_2d / set_joint_visible
    session_loaded()         -- new session loaded successfully
    dirty_changed(bool)      -- dirty flag toggled
    joint_selected(int)      -- selected joint changed (-1 = none)
    """

    frame_changed = Signal(int)
    joint_moved = Signal(int, int)
    session_loaded = Signal()
    dirty_changed = Signal(bool)
    joint_selected = Signal(int)
    extrinsic_preview_changed = Signal(int)  # current frame index
    pruning_changed = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._dataset: Optional[EgoDataset] = None
        self._hmd_data: Optional[HMDData] = None
        self._session_name: str = ""
        self._edits: Dict[int, Dict[int, JointEdit]] = {}
        self._keyframes: Dict[int, Set[int]] = {}  # joint_id -> set of frame indices
        self._dirty_frames: Set[int] = set()
        self._current_frame: int = 0
        self._selected_joint: int = -1
        self._image_cache: OrderedDict = OrderedDict()
        self._image_cache_size: int = 5
        # Head-joint pruning state
        self._pruned_joints: Set[int] = set()
        # Extrinsic fine-tuning state
        self._extrinsic_delta: Tuple[float, float, float, float, float, float] = (
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        )  # (rx, ry, rz, tx, ty, tz)
        self._intrinsics: Optional[Tuple[float, float, float, float]] = None  # (fx, fy, cx, cy)

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------
    @property
    def dataset(self) -> Optional[EgoDataset]:
        return self._dataset

    @property
    def hmd_data(self) -> Optional[HMDData]:
        return self._hmd_data

    @property
    def session_name(self) -> str:
        return self._session_name

    @property
    def frame_count(self) -> int:
        return len(self._dataset) if self._dataset else 0

    @property
    def current_frame(self) -> int:
        return self._current_frame

    @current_frame.setter
    def current_frame(self, value: int):
        if self._dataset is None:
            return
        value = max(0, min(value, self.frame_count - 1))
        if value != self._current_frame:
            self._current_frame = value
            self.frame_changed.emit(value)

    @property
    def selected_joint(self) -> int:
        return self._selected_joint

    @selected_joint.setter
    def selected_joint(self, value: int):
        if value != self._selected_joint:
            self._selected_joint = value
            self.joint_selected.emit(value)

    @property
    def is_dirty(self) -> bool:
        return len(self._dirty_frames) > 0

    # ------------------------------------------------------------------
    # Head-joint pruning
    # ------------------------------------------------------------------
    def set_pruning(self, enabled: bool,
                    joint_ids: Set[int] = frozenset({26, 27, 28, 29, 30, 31})):
        """Enable/disable pruning for the given joint IDs."""
        self._pruned_joints = set(joint_ids) if enabled else set()
        self.pruning_changed.emit()

    def is_joint_pruned(self, joint_id: int) -> bool:
        return joint_id in self._pruned_joints

    @property
    def pruning_enabled(self) -> bool:
        return len(self._pruned_joints) > 0

    # ------------------------------------------------------------------
    # Session management
    # ------------------------------------------------------------------
    def load_session(self, ego_dir: str, hmd_csv: Optional[str] = None,
                     session_name: str = ""):
        self._dataset = EgoDataset(ego_dir)
        self._hmd_data = HMDData(hmd_csv) if hmd_csv else None
        self._session_name = session_name or Path(ego_dir).parent.name
        self._edits.clear()
        self._keyframes.clear()
        self._dirty_frames.clear()
        self._current_frame = 0
        self._selected_joint = -1
        self._image_cache.clear()
        self._pruned_joints = set()
        self._extrinsic_delta = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self._intrinsics = None
        self.dirty_changed.emit(False)
        self.session_loaded.emit()
        # Estimate intrinsics from 3D/2D correspondences
        self.estimate_intrinsics()

    # ------------------------------------------------------------------
    # Frame data access
    # ------------------------------------------------------------------
    def get_frame_data(self, frame: int) -> Optional[dict]:
        if self._dataset is None or frame < 0 or frame >= self.frame_count:
            return None
        return self._dataset.get_frame(frame)

    def get_image_pixmap(self, frame: int) -> QPixmap:
        """Load image as QPixmap with LRU cache."""
        if frame in self._image_cache:
            self._image_cache.move_to_end(frame)
            return self._image_cache[frame]

        pixmap = QPixmap()
        if self._dataset is not None and 0 <= frame < self.frame_count:
            img_path = self._dataset.get_frame(frame).get("_image_path", "")
            if img_path and Path(img_path).exists():
                pixmap = QPixmap(img_path)

        self._image_cache[frame] = pixmap
        if len(self._image_cache) > self._image_cache_size:
            self._image_cache.popitem(last=False)
        return pixmap

    # ------------------------------------------------------------------
    # Joint 2-D access (edit overlay first, then EgoDataset)
    # ------------------------------------------------------------------
    def get_joint_2d(self, frame: int, joint_id: int) -> Tuple[float, float, int, bool]:
        """Return (u, v, confidence, visible)."""
        if frame in self._edits and joint_id in self._edits[frame]:
            edit = self._edits[frame][joint_id]
            orig = self._get_original_joint(frame, joint_id)
            conf = orig[2] if orig else 0
            return (edit.u, edit.v, conf, edit.visible)

        orig = self._get_original_joint(frame, joint_id)
        if orig is not None:
            return orig
        return (0.0, 0.0, 0, False)

    def _get_original_joint(self, frame: int, joint_id: int):
        """Read original (u, v, conf, vis) from the underlying EgoDataset."""
        if self._dataset is None:
            return None
        joints = self._dataset.get_joints_2d(frame)
        if joints is None or joint_id >= len(joints):
            return None
        u, v, conf, vis = joints[joint_id]
        return (float(u), float(v), int(conf), bool(vis > 0.5))

    def is_joint_edited(self, frame: int, joint_id: int) -> bool:
        return frame in self._edits and joint_id in self._edits[frame]

    # ------------------------------------------------------------------
    # Editing
    # ------------------------------------------------------------------
    def _ensure_edit(self, frame: int, joint_id: int) -> JointEdit:
        """Get or create the JointEdit entry for frame/joint_id."""
        if frame not in self._edits:
            self._edits[frame] = {}
        if joint_id not in self._edits[frame]:
            orig = self._get_original_joint(frame, joint_id)
            if orig:
                ou, ov, _oc, ovis = orig
            else:
                ou, ov, ovis = 0.0, 0.0, True
            self._edits[frame][joint_id] = JointEdit(
                u=ou, v=ov, visible=ovis,
                original_u=ou, original_v=ov, original_visible=ovis,
            )
        return self._edits[frame][joint_id]

    def _mark_dirty(self, frame: int):
        was_dirty = self.is_dirty
        self._dirty_frames.add(frame)
        if not was_dirty:
            self.dirty_changed.emit(True)

    def set_joint_2d(self, frame: int, joint_id: int, u: float, v: float):
        edit = self._ensure_edit(frame, joint_id)
        edit.u = u
        edit.v = v
        self._mark_dirty(frame)
        self.joint_moved.emit(frame, joint_id)

    def set_joint_visible(self, frame: int, joint_id: int, visible: bool):
        edit = self._ensure_edit(frame, joint_id)
        edit.visible = visible
        self._mark_dirty(frame)
        self.joint_moved.emit(frame, joint_id)

    def set_keyframe(self, frame: int, joint_id: int, is_keyframe: bool):
        edit = self._ensure_edit(frame, joint_id)
        edit.is_keyframe = is_keyframe
        if joint_id not in self._keyframes:
            self._keyframes[joint_id] = set()
        if is_keyframe:
            self._keyframes[joint_id].add(frame)
        else:
            self._keyframes[joint_id].discard(frame)

    def get_keyframes(self, joint_id: int) -> Set[int]:
        return self._keyframes.get(joint_id, set())

    def reset_joint(self, frame: int, joint_id: int):
        """Revert a joint to its original EgoDataset position."""
        if frame in self._edits and joint_id in self._edits[frame]:
            del self._edits[frame][joint_id]
            if not self._edits[frame]:
                del self._edits[frame]
                self._dirty_frames.discard(frame)
                if not self.is_dirty:
                    self.dirty_changed.emit(False)
            self.joint_moved.emit(frame, joint_id)

    # ------------------------------------------------------------------
    # Frame-level queries (for timeline color bar)
    # ------------------------------------------------------------------
    def is_frame_edited(self, frame: int) -> bool:
        return frame in self._edits and len(self._edits[frame]) > 0

    def has_keyframe_at(self, frame: int) -> bool:
        for frames_set in self._keyframes.values():
            if frame in frames_set:
                return True
        return False

    # ------------------------------------------------------------------
    # Save
    # ------------------------------------------------------------------
    def save(self) -> int:
        """Write modified annotation JSONs.  Returns number of files written."""
        saved_count = 0
        for frame_idx in sorted(self._dirty_frames):
            if frame_idx not in self._edits:
                continue

            # Check whether any joint actually differs from original
            has_real_change = False
            for jid, edit in self._edits[frame_idx].items():
                orig = self._get_original_joint(frame_idx, jid)
                if orig is None:
                    continue
                if (abs(edit.u - orig[0]) > 0.01
                        or abs(edit.v - orig[1]) > 0.01
                        or edit.visible != orig[3]):
                    has_real_change = True
                    break
            if not has_real_change:
                continue

            frame_data = self._dataset.get_frame(frame_idx)
            json_path = frame_data["_json_path"]

            # Backup original
            shutil.copy2(json_path, json_path + ".bak")

            with open(json_path) as f:
                data = json.load(f)

            skel_2d = data.get("skeleton_2d", [])
            for entry in skel_2d:
                jid = entry["joint_id"]
                if jid in self._edits[frame_idx]:
                    edit = self._edits[frame_idx][jid]
                    entry["u"] = round(edit.u, 2)
                    entry["v"] = round(edit.v, 2)
                    entry["visible"] = edit.visible
                if jid in self._pruned_joints:
                    entry["visible"] = False
            data["skeleton_2d"] = skel_2d

            with open(json_path, "w") as f:
                json.dump(data, f, indent=2)
            saved_count += 1

        self._dirty_frames.clear()
        self.dirty_changed.emit(False)
        return saved_count

    # ------------------------------------------------------------------
    # Extrinsic fine-tuning
    # ------------------------------------------------------------------
    def estimate_intrinsics(self):
        """Estimate pinhole camera params (fx, fy, cx, cy) from 3D/2D pairs.

        Uses first frames with valid skeleton data.  Solves the pinhole
        equations via least-squares:
            u = fx * (x / z) + cx
            v = fy * (y / z) + cy
        """
        if self._dataset is None:
            return
        A_rows: List[List[float]] = []
        b_rows: List[float] = []
        for frame_idx in range(min(self.frame_count, 50)):
            joints_3d = self._dataset.get_joints_3d(frame_idx)
            joints_2d = self._dataset.get_joints_2d(frame_idx)
            if joints_3d is None or joints_2d is None:
                continue
            for jid in range(min(len(joints_3d), len(joints_2d))):
                x, y, z, conf3 = joints_3d[jid]
                u, v, conf2, vis = joints_2d[jid]
                if conf3 < 2 or conf2 < 2 or z < 100:
                    continue
                xz = float(x / z)
                yz = float(y / z)
                # u = fx * x/z + cx  -->  [x/z, 1, 0, 0] . [fx, cx, fy, cy] = u
                A_rows.append([xz, 1.0, 0.0, 0.0])
                b_rows.append(float(u))
                # v = fy * y/z + cy  -->  [0, 0, y/z, 1] . [fx, cx, fy, cy] = v
                A_rows.append([0.0, 0.0, yz, 1.0])
                b_rows.append(float(v))
            if len(A_rows) >= 60:  # well over-determined
                break

        if len(A_rows) < 8:
            print("[ExtrinsicTuning] Not enough 3D/2D pairs for intrinsic estimation")
            self._intrinsics = None
            return

        A = np.array(A_rows, dtype=np.float64)
        b = np.array(b_rows, dtype=np.float64)
        result, residuals, rank, sv = np.linalg.lstsq(A, b, rcond=None)
        fx, cx, fy, cy = result
        self._intrinsics = (float(fx), float(fy), float(cx), float(cy))
        print(f"[ExtrinsicTuning] Estimated intrinsics: fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f} ({len(A_rows)//2} pairs)")

    def has_extrinsic_delta(self) -> bool:
        """True if any extrinsic parameter is non-zero."""
        return any(abs(v) > 1e-9 for v in self._extrinsic_delta)

    def has_intrinsics(self) -> bool:
        return self._intrinsics is not None

    def set_extrinsic_delta(self, rx: float, ry: float, rz: float,
                            tx: float, ty: float, tz: float):
        """Update the extrinsic delta and emit preview signal."""
        self._extrinsic_delta = (rx, ry, rz, tx, ty, tz)
        self.extrinsic_preview_changed.emit(self._current_frame)

    def get_adjusted_joint_2d(self, frame: int, joint_id: int) -> Tuple[float, float, int, bool]:
        """Return (u, v, conf, vis) with extrinsic delta applied.

        Uses a differential approach: project the 3D point with and without
        the delta, then add only the *difference* to the original 2D coords.
        This avoids intrinsic-estimation error replacing the ground-truth 2D
        positions, which would cause the skeleton to visibly shift/shrink
        the moment any slider is touched.

        Falls back to get_joint_2d if intrinsics are unavailable or 3D data missing.
        """
        if self._intrinsics is None or self._dataset is None:
            return self.get_joint_2d(frame, joint_id)

        joints_3d = self._dataset.get_joints_3d(frame)
        if joints_3d is None or joint_id >= len(joints_3d):
            return self.get_joint_2d(frame, joint_id)

        x, y, z, conf3 = joints_3d[joint_id]
        if z < 1:
            return self.get_joint_2d(frame, joint_id)

        orig_u, orig_v, conf, vis = self.get_joint_2d(frame, joint_id)
        fx, fy, cx, cy = self._intrinsics

        # Baseline: project the unmodified 3D point
        base_u, base_v = _project_pinhole(x, y, z, fx, fy, cx, cy)

        # Delta: project the transformed 3D point
        rx, ry, rz, tx, ty, tz = self._extrinsic_delta
        R = _euler_to_rotation_matrix(rx, ry, rz)
        pt = R @ np.array([x, y, z], dtype=np.float64) + np.array([tx, ty, tz])
        if pt[2] <= 0:
            return self.get_joint_2d(frame, joint_id)
        delta_u, delta_v = _project_pinhole(pt[0], pt[1], pt[2], fx, fy, cx, cy)

        # Apply only the projection offset to the original 2D coordinates
        u = orig_u + (delta_u - base_u)
        v = orig_v + (delta_v - base_v)
        return (u, v, conf, vis)

    def apply_extrinsic_to_all_frames(self) -> int:
        """Apply current delta to all frames, overwriting skeleton_2d in JSONs.

        Returns number of frames updated.
        """
        if self._intrinsics is None or self._dataset is None:
            return 0
        if not self.has_extrinsic_delta():
            return 0

        rx, ry, rz, tx, ty, tz = self._extrinsic_delta
        R = _euler_to_rotation_matrix(rx, ry, rz)
        t_vec = np.array([tx, ty, tz], dtype=np.float64)
        fx, fy, cx, cy = self._intrinsics

        updated = 0
        for frame_idx in range(self.frame_count):
            joints_3d = self._dataset.get_joints_3d(frame_idx)
            if joints_3d is None:
                continue

            frame_data = self._dataset.get_frame(frame_idx)
            json_path = frame_data["_json_path"]

            with open(json_path) as f:
                data = json.load(f)

            skel_2d = data.get("skeleton_2d", [])
            if not skel_2d:
                continue

            changed = False
            for entry in skel_2d:
                jid = entry["joint_id"]
                if jid >= len(joints_3d):
                    continue
                x, y, z, conf3 = joints_3d[jid]
                if z < 1:
                    continue
                # Baseline projection (no delta)
                base_u, base_v = _project_pinhole(x, y, z, fx, fy, cx, cy)
                # Transformed projection (with delta)
                pt = R @ np.array([x, y, z], dtype=np.float64) + t_vec
                if pt[2] <= 0:
                    continue
                new_u, new_v = _project_pinhole(pt[0], pt[1], pt[2], fx, fy, cx, cy)
                # Apply only the offset to the original 2D coordinates
                entry["u"] = round(entry["u"] + (new_u - base_u), 2)
                entry["v"] = round(entry["v"] + (new_v - base_v), 2)
                changed = True

            # Force pruned joints invisible
            for entry in skel_2d:
                if entry["joint_id"] in self._pruned_joints:
                    entry["visible"] = False
                    changed = True

            if changed:
                # Backup
                shutil.copy2(json_path, json_path + ".bak")
                data["skeleton_2d"] = skel_2d
                with open(json_path, "w") as f:
                    json.dump(data, f, indent=2)
                updated += 1

        # Reload dataset to pick up new values
        if updated > 0:
            ego_dir = str(self._dataset.root)
            self._dataset = EgoDataset(ego_dir)
            self._edits.clear()
            self._dirty_frames.clear()
            self._image_cache.clear()
            self.dirty_changed.emit(False)
            self.frame_changed.emit(self._current_frame)

        return updated

    def export_adjusted_transform(self, original_path: str, output_path: str):
        """Compose delta with original T_checker_to_A and save as JSON.

        T_new = delta . T_original
        R_new = R_delta * R_original
        t_new = R_delta * t_original + t_delta
        """
        with open(original_path) as f:
            orig = json.load(f)

        R_orig = np.array(orig["rotation"], dtype=np.float64)
        t_orig = np.array(orig["translation"], dtype=np.float64).flatten()

        rx, ry, rz, tx, ty, tz = self._extrinsic_delta
        R_delta = _euler_to_rotation_matrix(rx, ry, rz)
        t_delta = np.array([tx, ty, tz], dtype=np.float64)

        R_new = R_delta @ R_orig
        t_new = R_delta @ t_orig + t_delta

        result = {
            "rotation": R_new.tolist(),
            "translation": t_new.tolist(),
            "delta_applied": {
                "rx_deg": rx, "ry_deg": ry, "rz_deg": rz,
                "tx_mm": tx, "ty_mm": ty, "tz_mm": tz,
            },
        }
        with open(output_path, "w") as f:
            json.dump(result, f, indent=2)
        return output_path
