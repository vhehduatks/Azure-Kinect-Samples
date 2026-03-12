"""Auto-fit extrinsic delta from annotated 2D joints.

Computes the optimal 6DOF extrinsic delta (rx, ry, rz, tx, ty, tz) by
minimizing 2D reprojection error between projected original 3D joints and
the annotated (corrected) 2D joints across the full dataset hierarchy.

Algorithm:
  1. Estimate pinhole intrinsics (fx, fy, cx, cy) from original .bak 3D/2D pairs
  2. Collect correspondences: original 3D (.bak) + annotated 2D (.json)
  3. Optimize 6DOF delta via scipy.optimize.least_squares
  4. Report optimal delta, RMS reprojection error before/after
  5. Optionally apply the delta (differential 2D + optional 3D transform)

Dataset structure:
    <dataset_root>/
        <participant>/              e.g. 021903/
            <session>/              e.g. Dancing1_20260219_200818/
                ego_dataset/
                    annotations/    frame_*.json  (+  frame_*.json.bak if edited)

Usage:
    # Dry run: compute optimal delta from manual edits
    python fit_extrinsic.py <dataset_root>

    # Apply the fitted delta to a target dataset
    python fit_extrinsic.py <source_root> --target <target_root> --apply

    # Apply with 3D transform
    python fit_extrinsic.py <source_root> --target <target_root> --apply --3d

    # Single annotations dir
    python fit_extrinsic.py <annotations_dir> --flat

    # From model predictions (single session):
    python fit_extrinsic.py <annotations_dir> --flat --predictions <predictions_dir>

    # From model predictions (hierarchical, auto-discovers predictions/ dirs):
    python fit_extrinsic.py <dataset_root> --predictions

    # Per-session fitting (separate delta per session):
    python fit_extrinsic.py <dataset_root> --predictions --per-session
    python fit_extrinsic.py <dataset_root> --predictions --per-session --apply

    # Per-frame refinement (session fit + regularized per-frame adjustment):
    python fit_extrinsic.py <dataset_root> --predictions --per-frame
    python fit_extrinsic.py <dataset_root> --predictions --per-frame --frame-reg 10.0 --apply
"""

import argparse
import datetime
import json
import math
import os
import shutil
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy.optimize import least_squares

try:
    from tqdm import tqdm
except ImportError:
    def tqdm(iterable, **kwargs):
        return iterable

NUM_JOINTS = 32

JOINT_NAMES = [
    "PELVIS", "SPINE_NAVAL", "SPINE_CHEST", "NECK",
    "CLAVICLE_LEFT", "SHOULDER_LEFT", "ELBOW_LEFT", "WRIST_LEFT",
    "HAND_LEFT", "HANDTIP_LEFT", "THUMB_LEFT",
    "CLAVICLE_RIGHT", "SHOULDER_RIGHT", "ELBOW_RIGHT", "WRIST_RIGHT",
    "HAND_RIGHT", "HANDTIP_RIGHT", "THUMB_RIGHT",
    "HIP_LEFT", "KNEE_LEFT", "ANKLE_LEFT", "FOOT_LEFT",
    "HIP_RIGHT", "KNEE_RIGHT", "ANKLE_RIGHT", "FOOT_RIGHT",
    "HEAD", "NOSE", "EYE_LEFT", "EYE_RIGHT", "EAR_LEFT", "EAR_RIGHT",
]

# Distal hand joints — annotation only extends to wrist, so these have
# unreliable 3D tracking and noisy predictions.  Excluded by default.
EXCLUDED_JOINTS_DEFAULT = {8, 9, 10, 15, 16, 17}  # HAND/HANDTIP/THUMB L+R


# ------------------------------------------------------------------
# Geometry helpers (reused from data_model.py)
# ------------------------------------------------------------------

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


# ------------------------------------------------------------------
# Dataset discovery (reused from apply_avg_offset.py)
# ------------------------------------------------------------------

def _session_label(ann_dir_str: str, source_dir: str) -> str:
    """Short label for a session annotations dir, for display."""
    try:
        rel = Path(ann_dir_str).relative_to(Path(source_dir))
        parts = rel.parts
        return "/".join(parts[:2]) if len(parts) >= 2 else str(rel)
    except ValueError:
        return str(Path(ann_dir_str).name)


def discover_annotations_dirs(dataset_root: str) -> List[Path]:
    """Find all annotations/ dirs under <dataset_root>/<participant>/<session>/ego_dataset/."""
    root = Path(dataset_root)
    dirs = sorted(root.glob("*/*/ego_dataset/annotations"))
    return [d for d in dirs if d.is_dir()]


# ------------------------------------------------------------------
# Intrinsics estimation (reused from data_model.py)
# ------------------------------------------------------------------

def estimate_intrinsics_from_pairs(
    points_3d: List[Tuple[float, float, float]],
    points_2d: List[Tuple[float, float]],
) -> Optional[Tuple[float, float, float, float]]:
    """Estimate pinhole (fx, fy, cx, cy) from 3D/2D correspondences.

    Solves via least-squares:
        u = fx * x/z + cx
        v = fy * y/z + cy
    """
    A_rows: List[List[float]] = []
    b_rows: List[float] = []

    for (x, y, z), (u, v) in zip(points_3d, points_2d):
        if z < 100:
            continue
        xz = float(x / z)
        yz = float(y / z)
        # u = fx * x/z + cx  -->  [x/z, 1, 0, 0] . [fx, cx, fy, cy] = u
        A_rows.append([xz, 1.0, 0.0, 0.0])
        b_rows.append(float(u))
        # v = fy * y/z + cy  -->  [0, 0, y/z, 1] . [fx, cx, fy, cy] = v
        A_rows.append([0.0, 0.0, yz, 1.0])
        b_rows.append(float(v))

    if len(A_rows) < 8:
        return None

    A = np.array(A_rows, dtype=np.float64)
    b = np.array(b_rows, dtype=np.float64)
    result, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    fx, cx, fy, cy = result
    return (float(fx), float(fy), float(cx), float(cy))


def estimate_intrinsics_from_dir(annotations_dir: Path) -> Optional[Tuple[float, float, float, float]]:
    """Estimate intrinsics from .bak files in an annotations directory."""
    bak_files = sorted(annotations_dir.glob("frame_*.json.bak"))
    pts_3d: List[Tuple[float, float, float]] = []
    pts_2d: List[Tuple[float, float]] = []

    for bak_path in bak_files[:50]:  # first 50 frames with backups
        with open(bak_path, encoding="utf-8") as f:
            data = json.load(f)
        s3d = {e["joint_id"]: e for e in data.get("skeleton_3d", [])}
        s2d = {e["joint_id"]: e for e in data.get("skeleton_2d", [])}
        for jid in range(NUM_JOINTS):
            if jid not in s3d or jid not in s2d:
                continue
            j3 = s3d[jid]
            j2 = s2d[jid]
            if j3.get("confidence", 0) < 2 or j2.get("confidence", 0) < 2:
                continue
            if j3["z"] < 100:
                continue
            pts_3d.append((j3["x"], j3["y"], j3["z"]))
            pts_2d.append((j2["u"], j2["v"]))
        if len(pts_3d) >= 30:
            break

    if not pts_3d:
        # Fall back to non-bak json files
        json_files = sorted(annotations_dir.glob("frame_*.json"))
        for json_path in json_files[:50]:
            if Path(str(json_path) + ".bak").exists():
                continue  # skip edited frames, use only unedited
            with open(json_path, encoding="utf-8") as f:
                data = json.load(f)
            s3d = {e["joint_id"]: e for e in data.get("skeleton_3d", [])}
            s2d = {e["joint_id"]: e for e in data.get("skeleton_2d", [])}
            for jid in range(NUM_JOINTS):
                if jid not in s3d or jid not in s2d:
                    continue
                j3 = s3d[jid]
                j2 = s2d[jid]
                if j3.get("confidence", 0) < 2 or j2.get("confidence", 0) < 2:
                    continue
                if j3["z"] < 100:
                    continue
                pts_3d.append((j3["x"], j3["y"], j3["z"]))
                pts_2d.append((j2["u"], j2["v"]))
            if len(pts_3d) >= 30:
                break

    return estimate_intrinsics_from_pairs(pts_3d, pts_2d)


# ------------------------------------------------------------------
# Correspondence collection
# ------------------------------------------------------------------

def collect_correspondences(
    annotations_dir: Path,
    excluded_joints: Optional[set] = None,
) -> List[Tuple[np.ndarray, np.ndarray, int]]:
    """Collect (original_3d, annotated_2d, joint_id) triples from edited frames.

    Returns list of (P_3d [3,], uv_ann [2,], joint_id) for joints where:
      - .bak (original) has confidence >= 2 for both 3D and 2D
      - .json (edited) has a 2D change (|du| > 0.5 or |dv| > 0.5)
      - 3D depth > 100mm
      - joint_id not in excluded_joints
    """
    excl = excluded_joints or set()
    pairs: List[Tuple[np.ndarray, np.ndarray, int]] = []

    json_files = sorted(annotations_dir.glob("frame_*.json"))
    for json_path in json_files:
        bak_path = Path(str(json_path) + ".bak")
        if not bak_path.exists():
            continue

        with open(bak_path, encoding="utf-8") as f:
            original = json.load(f)
        with open(json_path, encoding="utf-8") as f:
            edited = json.load(f)

        orig_3d = {e["joint_id"]: e for e in original.get("skeleton_3d", [])}
        orig_2d = {e["joint_id"]: e for e in original.get("skeleton_2d", [])}
        edit_2d = {e["joint_id"]: e for e in edited.get("skeleton_2d", [])}

        for jid in range(NUM_JOINTS):
            if jid in excl:
                continue
            if jid not in orig_3d or jid not in orig_2d or jid not in edit_2d:
                continue
            o3 = orig_3d[jid]
            o2 = orig_2d[jid]
            e2 = edit_2d[jid]

            # Need confident original data
            if o3.get("confidence", 0) < 2 or o2.get("confidence", 0) < 2:
                continue
            if e2.get("confidence", 0) < 2:
                continue
            if o3["z"] < 100:
                continue

            # Only include joints where the annotation actually changed
            du = e2["u"] - o2["u"]
            dv = e2["v"] - o2["v"]
            if abs(du) < 0.5 and abs(dv) < 0.5:
                continue

            p3d = np.array([o3["x"], o3["y"], o3["z"]], dtype=np.float64)
            uv_ann = np.array([e2["u"], e2["v"]], dtype=np.float64)
            pairs.append((p3d, uv_ann, jid))

    return pairs


def collect_correspondences_from_predictions(
    annotations_dir: Path,
    predictions_dir: Path,
    min_pred_confidence: float = 0.5,
    excluded_joints: Optional[set] = None,
) -> List[Tuple[np.ndarray, np.ndarray, int]]:
    """Collect (original_3d, predicted_2d, joint_id) triples from model predictions.

    For each prediction JSON, finds the matching annotation JSON to get
    the original 3D joints, then pairs with the predicted 2D joints.

    Returns list of (P_3d [3,], uv_pred [2,], joint_id) for joints where:
      - annotation has 3D confidence >= 2 and depth > 100mm
      - prediction confidence >= min_pred_confidence
      - joint_id not in excluded_joints
    """
    excl = excluded_joints or set()
    pairs: List[Tuple[np.ndarray, np.ndarray, int]] = []

    pred_files = sorted(predictions_dir.glob("frame_*.json"))
    for pred_path in pred_files:
        ann_path = annotations_dir / pred_path.name
        if not ann_path.exists():
            continue

        with open(ann_path, encoding="utf-8") as f:
            ann_data = json.load(f)
        with open(pred_path, encoding="utf-8") as f:
            pred_data = json.load(f)

        skel_3d = {e["joint_id"]: e for e in ann_data.get("skeleton_3d", [])}
        skel_2d_pred = {e["joint_id"]: e
                        for e in pred_data.get("skeleton_2d_predicted", [])}

        for jid in range(NUM_JOINTS):
            if jid in excl:
                continue
            if jid not in skel_3d or jid not in skel_2d_pred:
                continue
            j3 = skel_3d[jid]
            j2p = skel_2d_pred[jid]

            if j3.get("confidence", 0) < 2:
                continue
            if j2p.get("confidence", 0) < min_pred_confidence:
                continue
            if j3["z"] < 100:
                continue

            p3d = np.array([j3["x"], j3["y"], j3["z"]], dtype=np.float64)
            uv_pred = np.array([j2p["u"], j2p["v"]], dtype=np.float64)
            pairs.append((p3d, uv_pred, jid))

    return pairs


def collect_correspondences_per_frame(
    annotations_dir: Path,
    predictions_dir: Path,
    min_pred_confidence: float = 0.5,
    excluded_joints: Optional[set] = None,
) -> Dict[str, List[Tuple[np.ndarray, np.ndarray, int]]]:
    """Collect correspondences grouped by frame filename.

    Returns dict: frame_filename (e.g. "frame_000000.json") ->
        list of (P_3d [3,], uv_pred [2,], joint_id)
    """
    excl = excluded_joints or set()
    per_frame: Dict[str, List[Tuple[np.ndarray, np.ndarray, int]]] = {}

    pred_files = sorted(predictions_dir.glob("frame_*.json"))
    for pred_path in pred_files:
        ann_path = annotations_dir / pred_path.name
        if not ann_path.exists():
            continue

        with open(ann_path, encoding="utf-8") as f:
            ann_data = json.load(f)
        with open(pred_path, encoding="utf-8") as f:
            pred_data = json.load(f)

        skel_3d = {e["joint_id"]: e for e in ann_data.get("skeleton_3d", [])}
        skel_2d_pred = {e["joint_id"]: e
                        for e in pred_data.get("skeleton_2d_predicted", [])}

        frame_pairs: List[Tuple[np.ndarray, np.ndarray, int]] = []
        for jid in range(NUM_JOINTS):
            if jid in excl:
                continue
            if jid not in skel_3d or jid not in skel_2d_pred:
                continue
            j3 = skel_3d[jid]
            j2p = skel_2d_pred[jid]
            if j3.get("confidence", 0) < 2:
                continue
            if j2p.get("confidence", 0) < min_pred_confidence:
                continue
            if j3["z"] < 100:
                continue
            p3d = np.array([j3["x"], j3["y"], j3["z"]], dtype=np.float64)
            uv_pred = np.array([j2p["u"], j2p["v"]], dtype=np.float64)
            frame_pairs.append((p3d, uv_pred, jid))

        if frame_pairs:
            per_frame[pred_path.name] = frame_pairs

    return per_frame


def collect_correspondences_per_frame_from_edits(
    annotations_dir: Path,
    excluded_joints: Optional[set] = None,
) -> Dict[str, List[Tuple[np.ndarray, np.ndarray, int]]]:
    """Collect correspondences grouped by frame filename from .bak/.json diffs."""
    excl = excluded_joints or set()
    per_frame: Dict[str, List[Tuple[np.ndarray, np.ndarray, int]]] = {}

    json_files = sorted(annotations_dir.glob("frame_*.json"))
    for json_path in json_files:
        bak_path = Path(str(json_path) + ".bak")
        if not bak_path.exists():
            continue

        with open(bak_path, encoding="utf-8") as f:
            original = json.load(f)
        with open(json_path, encoding="utf-8") as f:
            edited = json.load(f)

        orig_3d = {e["joint_id"]: e for e in original.get("skeleton_3d", [])}
        orig_2d = {e["joint_id"]: e for e in original.get("skeleton_2d", [])}
        edit_2d = {e["joint_id"]: e for e in edited.get("skeleton_2d", [])}

        frame_pairs: List[Tuple[np.ndarray, np.ndarray, int]] = []
        for jid in range(NUM_JOINTS):
            if jid in excl:
                continue
            if jid not in orig_3d or jid not in orig_2d or jid not in edit_2d:
                continue
            o3 = orig_3d[jid]
            o2 = orig_2d[jid]
            e2 = edit_2d[jid]
            if o3.get("confidence", 0) < 2 or o2.get("confidence", 0) < 2:
                continue
            if e2.get("confidence", 0) < 2:
                continue
            if o3["z"] < 100:
                continue
            du = e2["u"] - o2["u"]
            dv = e2["v"] - o2["v"]
            if abs(du) < 0.5 and abs(dv) < 0.5:
                continue
            p3d = np.array([o3["x"], o3["y"], o3["z"]], dtype=np.float64)
            uv_ann = np.array([e2["u"], e2["v"]], dtype=np.float64)
            frame_pairs.append((p3d, uv_ann, jid))

        if frame_pairs:
            per_frame[json_path.name] = frame_pairs

    return per_frame


# ------------------------------------------------------------------
# Joint balancing weights
# ------------------------------------------------------------------

# Body part grouping for weight computation
# NOTE: hand joints (8-10, 15-17) excluded — annotation only goes to wrist
_BODY_PARTS = {
    'spine':     [0, 1, 2, 3],
    'head':      [26, 27, 28, 29, 30, 31],
    'left_arm':  [4, 5, 6, 7],
    'right_arm': [11, 12, 13, 14],
    'left_leg':  [18, 19, 20, 21],
    'right_leg': [22, 23, 24, 25],
}

# Reverse map: joint_id -> body part name
_JOINT_TO_PART = {}
for _part, _jids in _BODY_PARTS.items():
    for _jid in _jids:
        _JOINT_TO_PART[_jid] = _part


def compute_joint_weights(
    correspondences: List[Tuple[np.ndarray, np.ndarray, int]],
) -> Dict[int, float]:
    """Compute inverse-frequency weights per joint so all body parts
    contribute equally to the optimization cost.

    Algorithm:
      1. Count correspondences per body part (spine, head, arms, legs)
      2. Target count = total / num_parts_present (equal share per part)
      3. Weight for joints in part P = target_count / count(P)
      4. Normalize so mean weight = 1.0

    Returns: {joint_id: weight}. Joints not in correspondences get 0.
    """
    # Count per body part
    part_counts: Dict[str, int] = {}
    for _, _, jid in correspondences:
        part = _JOINT_TO_PART.get(jid, 'spine')
        part_counts[part] = part_counts.get(part, 0) + 1

    if not part_counts:
        return {}

    total = sum(part_counts.values())
    n_parts = len(part_counts)
    target_per_part = total / n_parts

    # Weight per part = target / actual_count
    part_weights: Dict[str, float] = {}
    for part, count in part_counts.items():
        part_weights[part] = target_per_part / count

    # Assign to joints
    joint_weights: Dict[int, float] = {}
    for _, _, jid in correspondences:
        part = _JOINT_TO_PART.get(jid, 'spine')
        joint_weights[jid] = part_weights[part]

    # Normalize so mean weight = 1.0
    if joint_weights:
        # Weighted by occurrence: compute mean weight across all correspondences
        w_sum = sum(joint_weights.get(jid, 1.0) for _, _, jid in correspondences)
        w_mean = w_sum / len(correspondences)
        if w_mean > 0:
            for jid in joint_weights:
                joint_weights[jid] /= w_mean

    return joint_weights


def build_weight_vector(
    joint_ids: np.ndarray,   # (N,) int array of joint IDs
    joint_weights: Dict[int, float],
) -> np.ndarray:
    """Build per-correspondence weight vector from joint_ids and weight dict.

    Returns (N,) array of sqrt(weight) — applied to residuals so that
    least_squares minimizes weighted sum of squares.
    """
    weights = np.ones(len(joint_ids), dtype=np.float64)
    for i, jid in enumerate(joint_ids):
        weights[i] = joint_weights.get(int(jid), 1.0)
    return np.sqrt(weights)


# ------------------------------------------------------------------
# Optimization
# ------------------------------------------------------------------

def compute_residuals(
    params: np.ndarray,
    points_3d: np.ndarray,   # (N, 3)
    points_2d: np.ndarray,   # (N, 2)
    fx: float, fy: float, cx: float, cy: float,
) -> np.ndarray:
    """Compute reprojection residuals for the given 6DOF delta.

    params: [rx_deg, ry_deg, rz_deg, tx_mm, ty_mm, tz_mm]
    Returns: (2*N,) residual vector [u_err_0, v_err_0, u_err_1, v_err_1, ...]
    """
    rx, ry, rz, tx, ty, tz = params
    R = _euler_to_rotation_matrix(rx, ry, rz)
    t_vec = np.array([tx, ty, tz], dtype=np.float64)

    # Transform all 3D points: P' = R @ P + t
    transformed = (R @ points_3d.T).T + t_vec  # (N, 3)

    residuals = np.empty(2 * len(points_3d), dtype=np.float64)
    for i in range(len(points_3d)):
        x, y, z = transformed[i]
        if z <= 0:
            # Penalize points behind camera
            residuals[2 * i] = 1000.0
            residuals[2 * i + 1] = 1000.0
        else:
            u_proj = fx * x / z + cx
            v_proj = fy * y / z + cy
            residuals[2 * i] = u_proj - points_2d[i, 0]
            residuals[2 * i + 1] = v_proj - points_2d[i, 1]
    return residuals


def compute_residuals_weighted(
    params: np.ndarray,
    points_3d: np.ndarray,
    points_2d: np.ndarray,
    fx: float, fy: float, cx: float, cy: float,
    sqrt_weights: np.ndarray,  # (N,) — sqrt of per-correspondence weights
) -> np.ndarray:
    """Weighted reprojection residuals. Each correspondence's (u,v) residuals
    are multiplied by sqrt(weight) so that least_squares minimizes the
    weighted sum of squares."""
    raw = compute_residuals(params, points_3d, points_2d, fx, fy, cx, cy)
    # raw is (2*N,): interleaved [u0, v0, u1, v1, ...].  Apply weight per pair.
    weighted = raw.copy()
    for i in range(len(sqrt_weights)):
        weighted[2 * i] *= sqrt_weights[i]
        weighted[2 * i + 1] *= sqrt_weights[i]
    return weighted


def fit_extrinsic_delta(
    points_3d: np.ndarray,   # (N, 3)
    points_2d: np.ndarray,   # (N, 2)
    fx: float, fy: float, cx: float, cy: float,
    sqrt_weights: Optional[np.ndarray] = None,  # (N,) sqrt per-correspondence weights
) -> Tuple[np.ndarray, float, float]:
    """Fit optimal 6DOF extrinsic delta minimizing reprojection error.

    Args:
        sqrt_weights: If provided, applies per-correspondence weighting.
            Pass sqrt(w) so that least_squares minimizes sum(w_i * r_i^2).

    Returns:
        params: [rx, ry, rz, tx, ty, tz] (degrees, mm)
        rms_before: RMS reprojection error with identity transform (unweighted)
        rms_after: RMS reprojection error with fitted transform (unweighted)
    """
    use_weights = sqrt_weights is not None and len(sqrt_weights) == len(points_3d)

    # RMS before (identity, unweighted for reporting)
    res_before = compute_residuals(
        np.zeros(6), points_3d, points_2d, fx, fy, cx, cy
    )
    rms_before = np.sqrt(np.mean(res_before ** 2))

    # Optimize (weighted or unweighted)
    if use_weights:
        result = least_squares(
            compute_residuals_weighted,
            x0=np.zeros(6),
            args=(points_3d, points_2d, fx, fy, cx, cy, sqrt_weights),
            method="lm",
            ftol=1e-10,
            xtol=1e-10,
            gtol=1e-10,
            max_nfev=5000,
        )
    else:
        result = least_squares(
            compute_residuals,
            x0=np.zeros(6),
            args=(points_3d, points_2d, fx, fy, cx, cy),
            method="lm",
            ftol=1e-10,
            xtol=1e-10,
            gtol=1e-10,
            max_nfev=5000,
        )

    params = result.x
    # Report unweighted RMS for consistent comparison
    res_after = compute_residuals(params, points_3d, points_2d, fx, fy, cx, cy)
    rms_after = np.sqrt(np.mean(res_after ** 2))

    return params, rms_before, rms_after


# Regularization scale: 1 deg rotation ≈ 10mm translation in cost
_REG_SCALES = np.array([1.0, 1.0, 1.0, 0.1, 0.1, 0.1], dtype=np.float64)


def compute_residuals_regularized(
    params: np.ndarray,
    points_3d: np.ndarray,
    points_2d: np.ndarray,
    fx: float, fy: float, cx: float, cy: float,
    prior_params: np.ndarray,
    reg_weight: float,
    sqrt_weights: Optional[np.ndarray] = None,
) -> np.ndarray:
    """Reprojection residuals + L2 regularization toward a prior.

    The regularization appends 6 terms:
        reg_weight * scale_i * (param_i - prior_i)
    where scale normalizes so that 1 deg rotation ≈ 10mm translation.
    """
    if sqrt_weights is not None and len(sqrt_weights) == len(points_3d):
        reproj = compute_residuals_weighted(
            params, points_3d, points_2d, fx, fy, cx, cy, sqrt_weights
        )
    else:
        reproj = compute_residuals(params, points_3d, points_2d, fx, fy, cx, cy)
    reg = reg_weight * _REG_SCALES * (params - prior_params)
    return np.concatenate([reproj, reg])


def fit_frame_deltas(
    per_frame_corr: Dict[str, List[Tuple[np.ndarray, np.ndarray, int]]],
    session_params: np.ndarray,
    fx: float, fy: float, cx: float, cy: float,
    reg_weight: float = 5.0,
    min_joints: int = 4,
    joint_weights: Optional[Dict[int, float]] = None,
) -> Dict[str, np.ndarray]:
    """Fit per-frame deltas regularized toward the session delta.

    Args:
        per_frame_corr: frame_filename -> [(p3d, uv, joint_id), ...]
        session_params: [rx, ry, rz, tx, ty, tz] session-level prior
        reg_weight: regularization strength (higher = closer to session)
        min_joints: minimum correspondences per frame to attempt fit
        joint_weights: if provided, applies body-part balancing weights

    Returns:
        frame_filename -> params[6].  Frames with too few joints get session_params.
    """
    frame_params: Dict[str, np.ndarray] = {}

    for fname, corr in per_frame_corr.items():
        if len(corr) < min_joints:
            frame_params[fname] = session_params.copy()
            continue

        pts_3d = np.array([p[0] for p in corr])
        pts_2d = np.array([p[1] for p in corr])

        # Per-frame weights from joint balancing
        sw = None
        if joint_weights:
            jids = np.array([p[2] for p in corr], dtype=np.int32)
            sw = build_weight_vector(jids, joint_weights)

        result = least_squares(
            compute_residuals_regularized,
            x0=session_params.copy(),
            args=(pts_3d, pts_2d, fx, fy, cx, cy, session_params, reg_weight, sw),
            method="lm",
            ftol=1e-8,
            xtol=1e-8,
            max_nfev=1000,
        )
        frame_params[fname] = result.x

    return frame_params


# ------------------------------------------------------------------
# Apply delta to annotations
# ------------------------------------------------------------------

def apply_delta_to_dir(
    annotations_dir: Path,
    R: np.ndarray,
    t_vec: np.ndarray,
    fx: float, fy: float, cx: float, cy: float,
    transform_3d: bool = False,
    create_backup: bool = True,
) -> int:
    """Apply extrinsic delta to all annotation JSONs in a directory.

    Uses the differential projection approach from data_model.py:
        u' = u + [proj(R@P+t) - proj(P)]
    This cancels intrinsic estimation error.

    If transform_3d is True, also transforms skeleton_3d.
    """
    json_files = sorted(annotations_dir.glob("frame_*.json"))
    updated = 0

    for json_path in json_files:
        with open(json_path, encoding="utf-8") as f:
            data = json.load(f)

        skel_3d = data.get("skeleton_3d", [])
        skel_2d = data.get("skeleton_2d", [])
        if not skel_3d:
            continue

        # Build transform pairs: joint_id -> (orig_xyz, new_xyz)
        transform_pairs: Dict[int, Tuple[np.ndarray, np.ndarray]] = {}
        for entry in skel_3d:
            jid = entry["joint_id"]
            orig = np.array([entry["x"], entry["y"], entry["z"]], dtype=np.float64)
            pt = R @ orig + t_vec
            transform_pairs[jid] = (orig, pt)
            if transform_3d:
                entry["x"] = round(float(pt[0]), 2)
                entry["y"] = round(float(pt[1]), 2)
                entry["z"] = round(float(pt[2]), 2)

        # Read image dimensions for visibility check
        ci = data.get("camera_intrinsics", {})
        img_w = ci.get("width", int(2 * ci["cx"]) if "cx" in ci else None)
        img_h = ci.get("height", int(2 * ci["cy"]) if "cy" in ci else None)

        # Differential 2D update
        changed = False
        for entry in skel_2d:
            jid = entry["joint_id"]
            pair = transform_pairs.get(jid)
            if pair is None or entry.get("confidence", 0) == 0:
                continue

            orig_pt, new_pt = pair
            if orig_pt[2] <= 0 or new_pt[2] <= 0:
                if new_pt is not None and new_pt[2] <= 0 and entry.get("visible", False):
                    entry["visible"] = False
                    changed = True
                continue

            base_u, base_v = _project_pinhole(
                orig_pt[0], orig_pt[1], orig_pt[2], fx, fy, cx, cy
            )
            proj_u, proj_v = _project_pinhole(
                new_pt[0], new_pt[1], new_pt[2], fx, fy, cx, cy
            )
            new_u = entry["u"] + (proj_u - base_u)
            new_v = entry["v"] + (proj_v - base_v)
            if abs(new_u - entry["u"]) > 0.001 or abs(new_v - entry["v"]) > 0.001:
                entry["u"] = round(new_u, 2)
                entry["v"] = round(new_v, 2)
                changed = True
            # Recalculate visible flag
            if img_w is not None and img_h is not None:
                new_vis = bool(0 <= entry["u"] < img_w and 0 <= entry["v"] < img_h)
                if entry.get("visible") != new_vis:
                    entry["visible"] = new_vis
                    changed = True

        if changed or transform_3d:
            if create_backup:
                bak_path = str(json_path) + ".bak"
                if not os.path.exists(bak_path):
                    shutil.copy2(json_path, bak_path)
            data["skeleton_3d"] = skel_3d
            data["skeleton_2d"] = skel_2d
            tmp = str(json_path) + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
            os.replace(tmp, json_path)
            updated += 1

    return updated


def apply_per_frame_deltas_to_dir(
    annotations_dir: Path,
    frame_deltas: Dict[str, np.ndarray],
    session_params: np.ndarray,
    fx: float, fy: float, cx: float, cy: float,
    transform_3d: bool = False,
    create_backup: bool = True,
) -> int:
    """Apply per-frame extrinsic deltas to annotation JSONs.

    Each frame uses its own fitted delta if available, otherwise
    falls back to the session-level delta.
    """
    json_files = sorted(annotations_dir.glob("frame_*.json"))
    updated = 0

    for json_path in json_files:
        fname = json_path.name
        params = frame_deltas.get(fname, session_params)
        rx, ry, rz, tx, ty, tz = params
        R = _euler_to_rotation_matrix(rx, ry, rz)
        t_vec = np.array([tx, ty, tz], dtype=np.float64)

        with open(json_path, encoding="utf-8") as f:
            data = json.load(f)

        skel_3d = data.get("skeleton_3d", [])
        skel_2d = data.get("skeleton_2d", [])
        if not skel_3d:
            continue

        transform_pairs: Dict[int, Tuple[np.ndarray, np.ndarray]] = {}
        for entry in skel_3d:
            jid = entry["joint_id"]
            orig = np.array([entry["x"], entry["y"], entry["z"]], dtype=np.float64)
            pt = R @ orig + t_vec
            transform_pairs[jid] = (orig, pt)
            if transform_3d:
                entry["x"] = round(float(pt[0]), 2)
                entry["y"] = round(float(pt[1]), 2)
                entry["z"] = round(float(pt[2]), 2)

        # Read image dimensions for visibility check
        ci = data.get("camera_intrinsics", {})
        img_w = ci.get("width", int(2 * ci["cx"]) if "cx" in ci else None)
        img_h = ci.get("height", int(2 * ci["cy"]) if "cy" in ci else None)

        changed = False
        for entry in skel_2d:
            jid = entry["joint_id"]
            pair = transform_pairs.get(jid)
            if pair is None or entry.get("confidence", 0) == 0:
                continue
            orig_pt, new_pt = pair
            if orig_pt[2] <= 0 or new_pt[2] <= 0:
                if new_pt is not None and new_pt[2] <= 0 and entry.get("visible", False):
                    entry["visible"] = False
                    changed = True
                continue
            base_u, base_v = _project_pinhole(
                orig_pt[0], orig_pt[1], orig_pt[2], fx, fy, cx, cy
            )
            proj_u, proj_v = _project_pinhole(
                new_pt[0], new_pt[1], new_pt[2], fx, fy, cx, cy
            )
            new_u = entry["u"] + (proj_u - base_u)
            new_v = entry["v"] + (proj_v - base_v)
            if abs(new_u - entry["u"]) > 0.001 or abs(new_v - entry["v"]) > 0.001:
                entry["u"] = round(new_u, 2)
                entry["v"] = round(new_v, 2)
                changed = True
            # Recalculate visible flag
            if img_w is not None and img_h is not None:
                new_vis = bool(0 <= entry["u"] < img_w and 0 <= entry["v"] < img_h)
                if entry.get("visible") != new_vis:
                    entry["visible"] = new_vis
                    changed = True

        if changed or transform_3d:
            if create_backup:
                bak_path = str(json_path) + ".bak"
                if not os.path.exists(bak_path):
                    shutil.copy2(json_path, bak_path)
            data["skeleton_3d"] = skel_3d
            data["skeleton_2d"] = skel_2d
            tmp = str(json_path) + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
            os.replace(tmp, json_path)
            updated += 1

    return updated


# ------------------------------------------------------------------
# Optimization record saving
# ------------------------------------------------------------------

def save_optimization_log(
    log_path: str,
    args,
    intrinsics: Tuple[float, float, float, float],
    excluded_joints: set,
    per_joint_counts: Dict[int, int],
    joint_weights: Optional[Dict[int, float]],
    session_results: Dict[str, Tuple[np.ndarray, float, float, int]],
    source_dir: str,
    per_frame_results: Optional[Dict[str, Dict[str, np.ndarray]]] = None,
    intrinsics_from_json: bool = False,
    session_joint_weights: Optional[Dict[str, Dict[int, float]]] = None,
):
    """Save full optimization record as JSON for reproducibility and analysis."""
    fx, fy, cx, cy = intrinsics
    now = datetime.datetime.now()

    record: Dict = {
        "timestamp": now.isoformat(),
        "command": {
            "source": args.source,
            "target": getattr(args, "target", None),
            "predictions": args.predictions,
            "min_pred_confidence": args.min_pred_confidence,
            "per_session": args.per_session,
            "per_frame": args.per_frame,
            "frame_reg": args.frame_reg,
            "joint_weights_enabled": args.joint_weights,
            "joint_weights_scope": "per-session" if (args.joint_weights and args.per_session) else ("global" if args.joint_weights else "none"),
            "exclude_joints": sorted(excluded_joints),
            "apply": args.apply,
            "apply_3d": args.apply_3d,
        },
        "intrinsics": {"fx": fx, "fy": fy, "cx": cx, "cy": cy, "source": "calibration" if intrinsics_from_json else "estimated"},
    }

    # Joint detection and weights
    joint_info = []
    for jid in range(NUM_JOINTS):
        if per_joint_counts.get(jid, 0) > 0:
            name = JOINT_NAMES[jid] if jid < len(JOINT_NAMES) else "JOINT_%d" % jid
            part = _JOINT_TO_PART.get(jid, "unknown")
            entry = {
                "joint_id": jid,
                "name": name,
                "body_part": part,
                "count": per_joint_counts[jid],
            }
            if joint_weights:
                entry["weight"] = round(joint_weights.get(jid, 1.0), 4)
            joint_info.append(entry)
    record["joint_detection"] = joint_info

    # Body-part summary
    if joint_weights:
        part_summary = {}
        part_counts: Dict[str, int] = {}
        for ji in joint_info:
            part = ji["body_part"]
            part_counts[part] = part_counts.get(part, 0) + ji["count"]
        total = sum(part_counts.values())
        for part in ['spine', 'head', 'left_arm', 'right_arm', 'left_leg', 'right_leg']:
            count = part_counts.get(part, 0)
            w = joint_weights.get(_BODY_PARTS[part][0], 1.0) if _BODY_PARTS.get(part) else 1.0
            part_summary[part] = {
                "count": count,
                "percentage": round(100.0 * count / total, 2) if total > 0 else 0,
                "weight": round(w, 4),
            }
        record["body_part_distribution"] = part_summary

    # Per-session results
    sessions_list = []
    for key in sorted(session_results.keys()):
        params, rms_before, rms_after, n_pairs = session_results[key]
        label = _session_label(key, source_dir) if key != "_global" else "GLOBAL"
        rx, ry, rz, tx, ty, tz = params
        entry = {
            "session": label,
            "n_pairs": n_pairs,
            "rms_before_px": round(rms_before, 3),
            "rms_after_px": round(rms_after, 3),
            "delta": {
                "rx_deg": round(float(rx), 5),
                "ry_deg": round(float(ry), 5),
                "rz_deg": round(float(rz), 5),
                "tx_mm": round(float(tx), 3),
                "ty_mm": round(float(ty), 3),
                "tz_mm": round(float(tz), 3),
            },
        }

        # Per-session joint weights
        if session_joint_weights and key in session_joint_weights:
            sw_dict = session_joint_weights[key]
            session_parts = {}
            for part in ['spine', 'head', 'left_arm', 'right_arm', 'left_leg', 'right_leg']:
                jids = _BODY_PARTS.get(part, [])
                w = sw_dict.get(jids[0], 0.0) if jids and jids[0] in sw_dict else 0.0
                session_parts[part] = round(w, 4)
            entry["joint_weights"] = session_parts

        # Per-frame stats for this session
        if per_frame_results and key in per_frame_results:
            frame_deltas = per_frame_results[key]
            deviations = []
            for fp in frame_deltas.values():
                diff = fp - params
                dev = float(np.sqrt(np.sum((_REG_SCALES * diff) ** 2)))
                deviations.append(dev)
            entry["per_frame"] = {
                "n_frames": len(frame_deltas),
                "deviation_mean": round(np.mean(deviations), 4) if deviations else 0,
                "deviation_max": round(np.max(deviations), 4) if deviations else 0,
            }

        sessions_list.append(entry)
    record["sessions"] = sessions_list

    # Summary statistics (for per-session mode)
    if len(session_results) > 1 or (len(session_results) == 1 and "_global" not in session_results):
        all_p = np.array([r[0] for r in session_results.values()])
        all_rms_b = [r[1] for r in session_results.values()]
        all_rms_a = [r[2] for r in session_results.values()]
        labels = ["rx_deg", "ry_deg", "rz_deg", "tx_mm", "ty_mm", "tz_mm"]
        summary_params = {}
        for i, lbl in enumerate(labels):
            vals = all_p[:, i]
            summary_params[lbl] = {
                "mean": round(float(vals.mean()), 5),
                "std": round(float(vals.std()), 5),
                "min": round(float(vals.min()), 5),
                "max": round(float(vals.max()), 5),
            }
        record["summary"] = {
            "n_sessions": len(session_results),
            "parameters": summary_params,
            "rms_before_mean_px": round(float(np.mean(all_rms_b)), 3),
            "rms_before_median_px": round(float(np.median(all_rms_b)), 3),
            "rms_after_mean_px": round(float(np.mean(all_rms_a)), 3),
            "rms_after_median_px": round(float(np.median(all_rms_a)), 3),
        }

    # Write
    log_dir = Path(log_path).parent
    log_dir.mkdir(parents=True, exist_ok=True)
    tmp = log_path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(record, f, indent=2, ensure_ascii=False)
    os.replace(tmp, log_path)
    print("\nOptimization log saved: %s" % log_path)


# ------------------------------------------------------------------
# Main
# ------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Auto-fit extrinsic delta from annotated 2D joints. "
        "Computes optimal (rx, ry, rz, tx, ty, tz) by minimizing "
        "2D reprojection error across the dataset.",
    )
    parser.add_argument(
        "source",
        help="Source dataset root with annotated .json and .json.bak files",
    )
    parser.add_argument(
        "--target",
        help="Target dataset root to apply the fitted delta to. "
        "If omitted with --apply, applies back to source.",
    )
    parser.add_argument(
        "--apply", action="store_true",
        help="Apply the fitted delta (without this flag, dry-run only)",
    )
    parser.add_argument(
        "--3d", dest="apply_3d", action="store_true",
        help="Also transform 3D joints when applying",
    )
    parser.add_argument(
        "--no-backup", action="store_true",
        help="Skip creating .bak backup files on the target",
    )
    parser.add_argument(
        "--flat", action="store_true",
        help="Treat source/target as a single annotations/ dir (no hierarchy)",
    )
    parser.add_argument(
        "--predictions", nargs="?", const="auto", default=None,
        help="Use model predictions instead of manual edits. "
        "With --flat: path to predictions directory. "
        "Without --flat: auto-discovers ego_dataset/predictions/ alongside annotations/.",
    )
    parser.add_argument(
        "--min-pred-confidence", type=float, default=0.5,
        help="Minimum prediction confidence for correspondences (default: 0.5)",
    )
    parser.add_argument(
        "--per-session", action="store_true",
        help="Fit a separate 6DOF delta per session instead of one global delta. "
        "Accounts for helmet re-wearing between sessions.",
    )
    parser.add_argument(
        "--per-frame", action="store_true",
        help="Per-frame refinement on top of per-session fitting. "
        "Each frame gets a small adjustment regularized toward its session delta.",
    )
    parser.add_argument(
        "--frame-reg", type=float, default=5.0,
        help="Per-frame regularization weight (default: 5.0). "
        "Higher = frame deltas stay closer to the session average. "
        "Scale: 1 deg rotation ~ 10mm translation in cost.",
    )
    parser.add_argument(
        "--joint-weights", action="store_true", default=True,
        help="Balance body-part contribution via inverse-frequency weighting "
        "(default: enabled). Upper body joints that appear less often get "
        "higher weight so the fit isn't dominated by always-visible lower body.",
    )
    parser.add_argument(
        "--no-joint-weights", dest="joint_weights", action="store_false",
        help="Disable body-part balancing (all correspondences equal weight).",
    )
    parser.add_argument(
        "--exclude-joints", type=str, default="hand",
        help="Joints to exclude from fitting. "
        "'hand' (default) = HAND/HANDTIP/THUMB (8-10,15-17). "
        "'none' = include all. "
        "Comma-separated IDs for custom, e.g. '8,9,10,15,16,17,26,27'.",
    )
    parser.add_argument(
        "--log", type=str, default=None,
        help="Save optimization record to a JSON file. "
        "If omitted, auto-saves to <source>/fit_extrinsic_log_<timestamp>.json.",
    )
    parser.add_argument(
        "--no-log", action="store_true",
        help="Disable auto-saving the optimization log.",
    )
    parser.add_argument(
        "--restore", action="store_true",
        help="Restore annotations from .bak files (undo --apply). "
        "Copies every frame_*.json.bak back to frame_*.json, then exits. "
        "Use --flat for a single annotations dir.",
    )
    args = parser.parse_args()

    # Parse excluded joints
    if args.exclude_joints.lower() == "none":
        excluded_joints: set = set()
    elif args.exclude_joints.lower() == "hand":
        excluded_joints = set(EXCLUDED_JOINTS_DEFAULT)
    else:
        try:
            excluded_joints = {int(x.strip()) for x in args.exclude_joints.split(",")}
        except ValueError:
            print("Error: --exclude-joints must be 'hand', 'none', or comma-separated IDs")
            return

    # --per-frame implies --per-session
    if args.per_frame:
        args.per_session = True

    source_dir = args.source
    target_dir = args.target or source_dir

    if not Path(source_dir).is_dir():
        print("Error: source directory not found: %s" % source_dir)
        return
    if not Path(target_dir).is_dir():
        print("Error: target directory not found: %s" % target_dir)
        return

    # -----------------------------------------------------------
    # Restore mode: undo --apply by copying .bak -> .json
    # -----------------------------------------------------------
    if args.restore:
        root = Path(target_dir)
        if args.flat:
            bak_files = sorted(root.glob("frame_*.json.bak"))
        else:
            bak_files = sorted(root.rglob("frame_*.json.bak"))

        if not bak_files:
            print("No .bak files found under %s" % target_dir)
            return

        restored = 0
        for bak_path in tqdm(bak_files, desc="Restoring", unit="file"):
            json_path = bak_path.with_suffix("")  # remove .bak
            shutil.copy2(bak_path, json_path)
            restored += 1

        print("Restored %d annotation files from .bak backups" % restored)

        # Optionally clean up .bak files
        print("Backup files (.bak) are still present. "
              "Delete them manually if no longer needed.")
        return

    # -----------------------------------------------------------
    # Step 1: Discover annotation directories (+ predictions dirs)
    # -----------------------------------------------------------
    print("Source: %s" % source_dir)
    use_predictions = args.predictions is not None

    if args.flat:
        ann_dirs = [Path(source_dir)]
    else:
        ann_dirs_all = discover_annotations_dirs(source_dir)
        if use_predictions:
            # No .bak filter -- match annotations with sibling predictions/ dirs
            ann_dirs = []
            for d in ann_dirs_all:
                pred_dir = d.parent / "predictions"
                if pred_dir.is_dir() and list(pred_dir.glob("frame_*.json")):
                    ann_dirs.append(d)
            if not ann_dirs:
                print("No annotation dirs with sibling predictions/ found under: %s" % source_dir)
                print("Expected: <root>/<participant>/<session>/ego_dataset/predictions/")
                return
            print("Found %d annotation dirs with predictions" % len(ann_dirs))
        else:
            # Filter to dirs that have .bak files
            ann_dirs = [d for d in ann_dirs_all if list(d.glob("frame_*.json.bak"))]
            if not ann_dirs:
                print("No annotations dirs with .bak files found under: %s" % source_dir)
                print("Expected: <root>/<participant>/<session>/ego_dataset/annotations/")
                print("Use --flat if pointing directly at a single annotations/ dir.")
                return
            print("Found %d annotation dirs with edits" % len(ann_dirs))

    # Resolve predictions dirs for each annotations dir
    pred_dirs_map: Dict[str, Path] = {}  # ann_dir_str -> predictions_dir
    if use_predictions:
        if args.flat:
            if args.predictions == "auto":
                # Look for sibling predictions/ next to the annotations dir
                auto_pred = Path(source_dir).parent / "predictions"
                if not auto_pred.is_dir():
                    print("Error: no predictions/ dir found alongside %s" % source_dir)
                    print("Use --predictions <dir> to specify explicitly.")
                    return
                pred_dirs_map[str(ann_dirs[0])] = auto_pred
            else:
                pred_dir = Path(args.predictions)
                if not pred_dir.is_dir():
                    print("Error: predictions directory not found: %s" % args.predictions)
                    return
                pred_dirs_map[str(ann_dirs[0])] = pred_dir
        else:
            # Hierarchical: each annotations dir maps to sibling predictions/
            for d in ann_dirs:
                pred_dirs_map[str(d)] = d.parent / "predictions"

    # -----------------------------------------------------------
    # Step 2: Get intrinsics (from annotation JSON or estimate)
    # -----------------------------------------------------------
    # Try to read camera_intrinsics from annotation JSON (written by offline processor)
    intrinsics_from_json = False
    fx = fy = cx = cy = 0.0
    for ann_dir in ann_dirs:
        src_files = sorted(ann_dir.glob("frame_*.json.bak"))
        if not src_files:
            src_files = sorted(ann_dir.glob("frame_*.json"))
        for fpath in src_files[:5]:
            with open(fpath, encoding="utf-8") as f:
                data = json.load(f)
            ci = data.get("camera_intrinsics")
            if ci and "fx" in ci:
                fx, fy = ci["fx"], ci["fy"]
                cx, cy = ci["cx"], ci["cy"]
                intrinsics_from_json = True
                break
        if intrinsics_from_json:
            break

    if intrinsics_from_json:
        print("\nIntrinsics from camera calibration (annotation JSON):")
        print("  fx=%.1f fy=%.1f cx=%.1f cy=%.1f" % (fx, fy, cx, cy))
    else:
        print("\nEstimating intrinsics from original data...")
        all_pts_3d: List[Tuple[float, float, float]] = []
        all_pts_2d: List[Tuple[float, float]] = []

        for ann_dir in ann_dirs:
            src_files = sorted(ann_dir.glob("frame_*.json.bak"))
            if not src_files:
                src_files = sorted(ann_dir.glob("frame_*.json"))
            for fpath in src_files[:50]:
                with open(fpath, encoding="utf-8") as f:
                    data = json.load(f)
                s3d = {e["joint_id"]: e for e in data.get("skeleton_3d", [])}
                s2d = {e["joint_id"]: e for e in data.get("skeleton_2d", [])}
                for jid in range(NUM_JOINTS):
                    if jid not in s3d or jid not in s2d:
                        continue
                    j3 = s3d[jid]
                    j2 = s2d[jid]
                    if j3.get("confidence", 0) < 2 or j2.get("confidence", 0) < 2:
                        continue
                    if j3["z"] < 100:
                        continue
                    all_pts_3d.append((j3["x"], j3["y"], j3["z"]))
                    all_pts_2d.append((j2["u"], j2["v"]))
            if len(all_pts_3d) >= 200:
                break

        intrinsics = estimate_intrinsics_from_pairs(all_pts_3d, all_pts_2d)
        if intrinsics is None:
            print("Error: could not estimate intrinsics (not enough 3D/2D pairs)")
            return
        fx, fy, cx, cy = intrinsics
        print("  Intrinsics (estimated): fx=%.1f fy=%.1f cx=%.1f cy=%.1f (%d pairs)"
              % (fx, fy, cx, cy, len(all_pts_3d)))

    # -----------------------------------------------------------
    # Step 3: Collect correspondences
    # -----------------------------------------------------------
    # Keyed by ann_dir string for per-session mode, or "_global" for global mode.
    # Each correspondence is (p3d, uv, joint_id).
    session_correspondences: Dict[str, List[Tuple[np.ndarray, np.ndarray, int]]] = {}

    if excluded_joints:
        excl_names = [JOINT_NAMES[j] if j < len(JOINT_NAMES) else str(j) for j in sorted(excluded_joints)]
        print("\nExcluded joints: %s" % ", ".join(excl_names))

    if use_predictions:
        print("\nCollecting correspondences from model predictions "
              "(min_confidence=%.2f)..." % args.min_pred_confidence)
        for ann_dir in ann_dirs:
            p_dir = pred_dirs_map[str(ann_dir)]
            pairs = collect_correspondences_from_predictions(
                ann_dir, p_dir, min_pred_confidence=args.min_pred_confidence,
                excluded_joints=excluded_joints,
            )
            if not pairs:
                continue
            if not args.flat:
                rel = ann_dir.relative_to(Path(source_dir))
                parts = rel.parts
                label = "/".join(parts[:2]) if len(parts) >= 2 else str(rel)
                print("  [%s] %d pairs" % (label, len(pairs)))

            key = str(ann_dir) if args.per_session else "_global"
            session_correspondences.setdefault(key, []).extend(pairs)
    else:
        print("\nCollecting correspondences from edited frames...")
        for ann_dir in ann_dirs:
            pairs = collect_correspondences(ann_dir, excluded_joints=excluded_joints)
            if pairs:
                if not args.flat:
                    rel = ann_dir.relative_to(Path(source_dir))
                    parts = rel.parts
                    label = "/".join(parts[:2]) if len(parts) >= 2 else str(rel)
                    print("  [%s] %d pairs" % (label, len(pairs)))

                key = str(ann_dir) if args.per_session else "_global"
                session_correspondences.setdefault(key, []).extend(pairs)

    total_pairs = sum(len(v) for v in session_correspondences.values())
    if total_pairs == 0:
        kind = "prediction" if use_predictions else "edited"
        print("No %s correspondences found." % kind)
        return

    # Count per-joint from collected correspondences (global aggregate for reporting)
    per_joint_counts: Dict[int, int] = {jid: 0 for jid in range(NUM_JOINTS)}
    all_corr_flat = []
    for corr_list in session_correspondences.values():
        for _, _, jid in corr_list:
            per_joint_counts[jid] += 1
        all_corr_flat.extend(corr_list)

    n_used_joints = sum(1 for c in per_joint_counts.values() if c > 0)
    print("\nTotal: %d correspondences across %d joint types" % (total_pairs, n_used_joints))

    # -----------------------------------------------------------
    # Compute joint weights
    # -----------------------------------------------------------
    # In per-session mode: compute weights per session so each session's fit
    # depends only on its own body-part distribution.
    # In global mode: compute once from all pooled correspondences.
    # session_joint_weights: key -> {joint_id: weight} (or None if disabled)
    session_joint_weights: Dict[str, Dict[int, float]] = {}
    joint_weights: Optional[Dict[int, float]] = None  # global (for reporting / global mode)

    if args.joint_weights:
        if args.per_session:
            # Per-session weights
            for key, corr_list in session_correspondences.items():
                session_joint_weights[key] = compute_joint_weights(corr_list)
            # Also compute global aggregate for the summary report
            joint_weights = compute_joint_weights(all_corr_flat)
        else:
            # Global weights
            joint_weights = compute_joint_weights(all_corr_flat)

    # Show per-joint breakdown (global aggregate)
    print("\n%3s %-18s %7s %5s %6s" % ("ID", "Joint", "Part", "N", "Weight"))
    print("-" * 45)
    for jid in range(NUM_JOINTS):
        if per_joint_counts[jid] > 0:
            name = JOINT_NAMES[jid] if jid < len(JOINT_NAMES) else "JOINT_%d" % jid
            part = _JOINT_TO_PART.get(jid, "?")[:7]
            w = joint_weights.get(jid, 1.0) if joint_weights else 1.0
            print("%3d %-18s %7s %5d %6.2f" % (jid, name, part, per_joint_counts[jid], w))

    if args.joint_weights and joint_weights:
        # Show per-body-part summary (global aggregate)
        part_counts: Dict[str, int] = {}
        for _, _, jid in all_corr_flat:
            part = _JOINT_TO_PART.get(jid, 'spine')
            part_counts[part] = part_counts.get(part, 0) + 1
        print("\nBody-part distribution (global aggregate):")
        for part in ['spine', 'head', 'left_arm', 'right_arm', 'left_leg', 'right_leg']:
            n = part_counts.get(part, 0)
            pct = 100.0 * n / total_pairs if total_pairs > 0 else 0
            w = joint_weights.get(_BODY_PARTS[part][0], 1.0) if joint_weights else 1.0
            print("  %-12s %6d (%5.1f%%)  weight=%.2f" % (part, n, pct, w))
        if args.per_session:
            print("  Joint balancing: ENABLED (per-session weights)")
        else:
            print("  Joint balancing: ENABLED")
    else:
        print("\n  Joint balancing: DISABLED")
    print()

    # -----------------------------------------------------------
    # Step 4: Optimize 6DOF delta (global or per-session)
    # -----------------------------------------------------------
    # session_results: key -> (params, rms_before, rms_after, n_pairs)
    session_results: Dict[str, Tuple[np.ndarray, float, float, int]] = {}

    # Helper: build sqrt-weight vector for a correspondence list
    def _make_sqrt_weights(corr_list, weights_dict):
        if not weights_dict:
            return None
        jids = np.array([p[2] for p in corr_list], dtype=np.int32)
        return build_weight_vector(jids, weights_dict)

    if args.per_session:
        print("Optimizing per-session 6DOF deltas (%d sessions)..."
              % len(session_correspondences))
        print()
        for key in sorted(session_correspondences.keys()):
            corr = session_correspondences[key]
            if len(corr) < 6:
                print("  [%s] skipped (%d pairs, need >= 6)" % (_session_label(key, source_dir), len(corr)))
                continue
            pts_3d = np.array([p[0] for p in corr])
            pts_2d = np.array([p[1] for p in corr])
            # Use this session's own weights
            sw = _make_sqrt_weights(corr, session_joint_weights.get(key))
            params, rms_before, rms_after = fit_extrinsic_delta(
                pts_3d, pts_2d, fx, fy, cx, cy, sqrt_weights=sw,
            )
            session_results[key] = (params, rms_before, rms_after, len(corr))
            rx, ry, rz, tx, ty, tz = params
            print("  [%s] %d pairs | RMS %.1f -> %.1f px | "
                  "r=(%.3f, %.3f, %.3f) t=(%.1f, %.1f, %.1f)"
                  % (_session_label(key, source_dir), len(corr),
                     rms_before, rms_after, rx, ry, rz, tx, ty, tz))

        if not session_results:
            print("No sessions had enough correspondences to fit.")
            return

        # Per-session weight distribution report
        if args.joint_weights and session_joint_weights:
            print()
            _PARTS_ORDER = ['spine', 'head', 'left_arm', 'right_arm', 'left_leg', 'right_leg']
            hdr = "%-30s" % "Session"
            for p in _PARTS_ORDER:
                hdr += " %8s" % p[:8]
            print(hdr)
            print("-" * (30 + 9 * len(_PARTS_ORDER)))
            for key in sorted(session_results.keys()):
                sw_dict = session_joint_weights.get(key, {})
                label = _session_label(key, source_dir)
                if len(label) > 28:
                    label = "..." + label[-25:]
                row = "%-30s" % label
                for p in _PARTS_ORDER:
                    jids = _BODY_PARTS.get(p, [])
                    w = sw_dict.get(jids[0], 1.0) if jids and jids[0] in sw_dict else 0.0
                    row += " %8.2f" % w
                print(row)

        # Summary statistics
        all_params = np.array([r[0] for r in session_results.values()])
        all_rms_before = [r[1] for r in session_results.values()]
        all_rms_after = [r[2] for r in session_results.values()]
        print("\n" + "=" * 60)
        print("PER-SESSION SUMMARY (%d sessions)" % len(session_results))
        print("=" * 60)
        labels = ["rx(deg)", "ry(deg)", "rz(deg)", "tx(mm)", "ty(mm)", "tz(mm)"]
        print("  %12s %8s %8s %8s" % ("param", "mean", "std", "range"))
        for i, lbl in enumerate(labels):
            vals = all_params[:, i]
            print("  %12s %+8.3f %8.3f  [%+.3f, %+.3f]"
                  % (lbl, vals.mean(), vals.std(), vals.min(), vals.max()))
        print()
        print("  RMS reprojection error:")
        print("    Before: %.2f px (mean), %.2f px (median)"
              % (np.mean(all_rms_before), np.median(all_rms_before)))
        print("    After:  %.2f px (mean), %.2f px (median)"
              % (np.mean(all_rms_after), np.median(all_rms_after)))
        print("=" * 60)
    else:
        print("Optimizing 6DOF extrinsic delta...")
        all_corr = session_correspondences["_global"]
        pts_3d = np.array([p[0] for p in all_corr])
        pts_2d = np.array([p[1] for p in all_corr])
        sw = _make_sqrt_weights(all_corr, joint_weights)

        params, rms_before, rms_after = fit_extrinsic_delta(
            pts_3d, pts_2d, fx, fy, cx, cy, sqrt_weights=sw,
        )
        session_results["_global"] = (params, rms_before, rms_after, len(all_corr))

        rx, ry, rz, tx, ty, tz = params

        print("\n" + "=" * 60)
        print("FITTED EXTRINSIC DELTA")
        print("=" * 60)
        print("  rx = %+.4f deg" % rx)
        print("  ry = %+.4f deg" % ry)
        print("  rz = %+.4f deg" % rz)
        print("  tx = %+.2f mm" % tx)
        print("  ty = %+.2f mm" % ty)
        print("  tz = %+.2f mm" % tz)
        print()
        print("  RMS reprojection error:")
        print("    Before: %.2f px" % rms_before)
        print("    After:  %.2f px" % rms_after)
        print("    Reduction: %.1f%%" % (100.0 * (1.0 - rms_after / rms_before) if rms_before > 0 else 0))
        print("=" * 60)

        # Sanity checks
        if abs(rx) > 10 or abs(ry) > 10 or abs(rz) > 10:
            print("\nWARNING: Large rotation (>10 deg) -- result may be unreliable.")
        if abs(tx) > 500 or abs(ty) > 500 or abs(tz) > 500:
            print("\nWARNING: Large translation (>500mm) -- result may be unreliable.")

    # -----------------------------------------------------------
    # Step 4b: Per-frame refinement (if --per-frame)
    # -----------------------------------------------------------
    # per_frame_results: ann_dir_str -> {frame_filename -> params[6]}
    per_frame_results: Dict[str, Dict[str, np.ndarray]] = {}

    if args.per_frame and session_results:
        print("\nRefining per-frame deltas (reg_weight=%.1f)..." % args.frame_reg)
        total_frames = 0
        total_refined = 0

        for ann_dir in ann_dirs:
            ann_key = str(ann_dir)
            if ann_key not in session_results:
                continue
            session_params = session_results[ann_key][0]

            # Collect per-frame correspondences
            if use_predictions:
                p_dir = pred_dirs_map[ann_key]
                pf_corr = collect_correspondences_per_frame(
                    ann_dir, p_dir, min_pred_confidence=args.min_pred_confidence,
                    excluded_joints=excluded_joints,
                )
            else:
                pf_corr = collect_correspondences_per_frame_from_edits(
                    ann_dir, excluded_joints=excluded_joints,
                )

            if not pf_corr:
                continue

            # Use session's own weights in per-session mode, global weights otherwise
            frame_jw = session_joint_weights.get(ann_key, joint_weights)
            frame_deltas = fit_frame_deltas(
                pf_corr, session_params, fx, fy, cx, cy,
                reg_weight=args.frame_reg,
                joint_weights=frame_jw,
            )
            per_frame_results[ann_key] = frame_deltas

            # Stats: how much did frames deviate from session?
            deviations = []
            for fname, fp in frame_deltas.items():
                diff = fp - session_params
                # Weighted norm: 1 deg ~ 10mm
                dev = np.sqrt(np.sum((_REG_SCALES * diff) ** 2))
                deviations.append(dev)
            n_frames = len(frame_deltas)
            total_frames += n_frames
            total_refined += sum(1 for d in deviations if d > 0.01)

            if not args.flat:
                label = _session_label(ann_key, source_dir)
                print("  [%s] %d frames, mean deviation=%.3f, max=%.3f"
                      % (label, n_frames, np.mean(deviations), np.max(deviations)))

        print("Per-frame refinement: %d frames (%d adjusted) across %d sessions"
              % (total_frames, total_refined, len(per_frame_results)))

    # -----------------------------------------------------------
    # Step 5: Optionally apply to target
    # -----------------------------------------------------------
    if args.apply:
        mode = "2D + 3D" if args.apply_3d else "2D only"
        if per_frame_results:
            mode += " per-frame"
        print("\nTarget: %s" % target_dir)
        print("Applying fitted delta (%s)..." % mode)

        if args.flat:
            tgt_dirs = [Path(target_dir)]
        else:
            tgt_dirs = discover_annotations_dirs(target_dir)
            if not tgt_dirs:
                print("No annotations dirs found under target: %s" % target_dir)
                return
            print("Applying to %d annotation dirs..." % len(tgt_dirs))

        # Need intrinsics for the target too (for differential projection)
        if target_dir != source_dir:
            tgt_intrinsics = None
            # Try reading from annotation JSON first
            for td in tgt_dirs[:5]:
                src_files = sorted(td.glob("frame_*.json.bak"))
                if not src_files:
                    src_files = sorted(td.glob("frame_*.json"))
                for fpath in src_files[:5]:
                    with open(fpath, encoding="utf-8") as f:
                        data = json.load(f)
                    ci = data.get("camera_intrinsics")
                    if ci and "fx" in ci:
                        tgt_intrinsics = (ci["fx"], ci["fy"], ci["cx"], ci["cy"])
                        print("Target intrinsics from camera calibration (annotation JSON):")
                        break
                if tgt_intrinsics:
                    break
            # Fall back to estimation
            if tgt_intrinsics is None:
                print("Estimating target intrinsics...")
                for td in tgt_dirs[:5]:
                    tgt_intrinsics = estimate_intrinsics_from_dir(td)
                    if tgt_intrinsics is not None:
                        break
            if tgt_intrinsics is None:
                print("Warning: could not estimate target intrinsics, using source intrinsics")
                tgt_fx, tgt_fy, tgt_cx, tgt_cy = fx, fy, cx, cy
            else:
                tgt_fx, tgt_fy, tgt_cx, tgt_cy = tgt_intrinsics
                print("  Target intrinsics: fx=%.1f fy=%.1f cx=%.1f cy=%.1f"
                      % (tgt_fx, tgt_fy, tgt_cx, tgt_cy))
        else:
            tgt_fx, tgt_fy, tgt_cx, tgt_cy = fx, fy, cx, cy

        total_modified = 0
        for ann_dir in tgt_dirs:
            ann_key = str(ann_dir)

            # Pick session params for this dir
            if ann_key in session_results:
                session_params = session_results[ann_key][0]
            elif "_global" in session_results:
                session_params = session_results["_global"][0]
            else:
                continue

            # Use per-frame deltas if available
            if ann_key in per_frame_results:
                n = apply_per_frame_deltas_to_dir(
                    ann_dir, per_frame_results[ann_key], session_params,
                    tgt_fx, tgt_fy, tgt_cx, tgt_cy,
                    transform_3d=args.apply_3d,
                    create_backup=not args.no_backup,
                )
            else:
                rx, ry, rz, tx, ty, tz = session_params
                R = _euler_to_rotation_matrix(rx, ry, rz)
                t_vec = np.array([tx, ty, tz], dtype=np.float64)
                n = apply_delta_to_dir(
                    ann_dir, R, t_vec,
                    tgt_fx, tgt_fy, tgt_cx, tgt_cy,
                    transform_3d=args.apply_3d,
                    create_backup=not args.no_backup,
                )

            if n > 0:
                if not args.flat:
                    rel = ann_dir.relative_to(Path(target_dir))
                    parts = rel.parts
                    label = "/".join(parts[:2]) if len(parts) >= 2 else str(rel)
                    print("  [%s] %d files" % (label, n))
                total_modified += n

        print("Applied fitted delta (%s) to %d files total." % (mode, total_modified))
    else:
        # Dry-run: show what would happen
        if args.flat:
            n_target = len(list(Path(target_dir).glob("frame_*.json")))
        else:
            tgt_dirs = discover_annotations_dirs(target_dir)
            n_target = sum(len(list(d.glob("frame_*.json"))) for d in tgt_dirs)
            print("\nTarget: %s (%d annotation dirs)" % (target_dir, len(tgt_dirs)))

        print("Dry run -- %d files would be modified. Use --apply to proceed." % n_target)

    # -----------------------------------------------------------
    # Step 6: Save optimization log
    # -----------------------------------------------------------
    if not args.no_log and session_results:
        if args.log:
            log_path = args.log
        else:
            ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            log_path = str(Path(source_dir) / ("fit_extrinsic_log_%s.json" % ts))

        save_optimization_log(
            log_path=log_path,
            args=args,
            intrinsics=(fx, fy, cx, cy),
            excluded_joints=excluded_joints,
            per_joint_counts=per_joint_counts,
            joint_weights=joint_weights,
            session_results=session_results,
            source_dir=source_dir,
            per_frame_results=per_frame_results if per_frame_results else None,
            intrinsics_from_json=intrinsics_from_json,
            session_joint_weights=session_joint_weights if session_joint_weights else None,
        )


if __name__ == "__main__":
    main()
