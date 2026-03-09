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
"""

import argparse
import json
import math
import os
import shutil
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy.optimize import least_squares

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

def collect_correspondences(annotations_dir: Path) -> List[Tuple[np.ndarray, np.ndarray]]:
    """Collect (original_3d, annotated_2d) pairs from edited frames.

    Returns list of (P_3d [3,], uv_ann [2,]) for joints where:
      - .bak (original) has confidence >= 2 for both 3D and 2D
      - .json (edited) has a 2D change (|du| > 0.5 or |dv| > 0.5)
      - 3D depth > 100mm
    """
    pairs: List[Tuple[np.ndarray, np.ndarray]] = []

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
            pairs.append((p3d, uv_ann))

    return pairs


def collect_correspondences_from_predictions(
    annotations_dir: Path,
    predictions_dir: Path,
    min_pred_confidence: float = 0.5,
) -> List[Tuple[np.ndarray, np.ndarray]]:
    """Collect (original_3d, predicted_2d) pairs from model predictions.

    For each prediction JSON, finds the matching annotation JSON to get
    the original 3D joints, then pairs with the predicted 2D joints.

    Returns list of (P_3d [3,], uv_pred [2,]) for joints where:
      - annotation has 3D confidence >= 2 and depth > 100mm
      - prediction confidence >= min_pred_confidence
    """
    pairs: List[Tuple[np.ndarray, np.ndarray]] = []

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
            pairs.append((p3d, uv_pred))

    return pairs


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


def fit_extrinsic_delta(
    points_3d: np.ndarray,   # (N, 3)
    points_2d: np.ndarray,   # (N, 2)
    fx: float, fy: float, cx: float, cy: float,
) -> Tuple[np.ndarray, float, float]:
    """Fit optimal 6DOF extrinsic delta minimizing reprojection error.

    Returns:
        params: [rx, ry, rz, tx, ty, tz] (degrees, mm)
        rms_before: RMS reprojection error with identity transform
        rms_after: RMS reprojection error with fitted transform
    """
    # RMS before (identity transform)
    res_before = compute_residuals(
        np.zeros(6), points_3d, points_2d, fx, fy, cx, cy
    )
    rms_before = np.sqrt(np.mean(res_before ** 2))

    # Optimize
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
    rms_after = np.sqrt(np.mean(result.fun ** 2))

    return params, rms_before, rms_after


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

        # Differential 2D update
        changed = False
        for entry in skel_2d:
            jid = entry["joint_id"]
            pair = transform_pairs.get(jid)
            if pair is None or entry.get("confidence", 0) == 0:
                continue

            orig_pt, new_pt = pair
            if orig_pt[2] <= 0 or new_pt[2] <= 0:
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
    args = parser.parse_args()

    source_dir = args.source
    target_dir = args.target or source_dir

    if not Path(source_dir).is_dir():
        print("Error: source directory not found: %s" % source_dir)
        return
    if not Path(target_dir).is_dir():
        print("Error: target directory not found: %s" % target_dir)
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
    # Step 2: Estimate intrinsics from original 3D/2D data
    # -----------------------------------------------------------
    print("\nEstimating intrinsics from original data...")
    all_pts_3d: List[Tuple[float, float, float]] = []
    all_pts_2d: List[Tuple[float, float]] = []

    for ann_dir in ann_dirs:
        # Use .bak files if available, otherwise annotation JSONs directly
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
    print("  Intrinsics: fx=%.1f fy=%.1f cx=%.1f cy=%.1f (%d pairs)"
          % (fx, fy, cx, cy, len(all_pts_3d)))

    # -----------------------------------------------------------
    # Step 3: Collect correspondences
    # -----------------------------------------------------------
    # Keyed by ann_dir string for per-session mode, or "_global" for global mode.
    session_correspondences: Dict[str, List[Tuple[np.ndarray, np.ndarray]]] = {}
    per_joint_counts: Dict[int, int] = {jid: 0 for jid in range(NUM_JOINTS)}

    if use_predictions:
        print("\nCollecting correspondences from model predictions "
              "(min_confidence=%.2f)..." % args.min_pred_confidence)
        for ann_dir in ann_dirs:
            p_dir = pred_dirs_map[str(ann_dir)]
            pairs = collect_correspondences_from_predictions(
                ann_dir, p_dir, min_pred_confidence=args.min_pred_confidence,
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

            # Per-joint counts
            pred_files = sorted(p_dir.glob("frame_*.json"))
            for pred_path in pred_files:
                ann_path = ann_dir / pred_path.name
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
                    if jid not in skel_3d or jid not in skel_2d_pred:
                        continue
                    j3 = skel_3d[jid]
                    j2p = skel_2d_pred[jid]
                    if j3.get("confidence", 0) < 2:
                        continue
                    if j2p.get("confidence", 0) < args.min_pred_confidence:
                        continue
                    if j3["z"] < 100:
                        continue
                    per_joint_counts[jid] += 1
    else:
        print("\nCollecting correspondences from edited frames...")
        for ann_dir in ann_dirs:
            pairs = collect_correspondences(ann_dir)
            if pairs:
                if not args.flat:
                    rel = ann_dir.relative_to(Path(source_dir))
                    parts = rel.parts
                    label = "/".join(parts[:2]) if len(parts) >= 2 else str(rel)
                    print("  [%s] %d pairs" % (label, len(pairs)))

                key = str(ann_dir) if args.per_session else "_global"
                session_correspondences.setdefault(key, []).extend(pairs)

            # Count per-joint for reporting
            json_files = sorted(ann_dir.glob("frame_*.json"))
            for json_path in json_files:
                bak_path = Path(str(json_path) + ".bak")
                if not bak_path.exists():
                    continue
                with open(bak_path, encoding="utf-8") as f:
                    original = json.load(f)
                with open(json_path, encoding="utf-8") as f:
                    edited = json.load(f)
                orig_2d = {e["joint_id"]: e for e in original.get("skeleton_2d", [])}
                edit_2d = {e["joint_id"]: e for e in edited.get("skeleton_2d", [])}
                for jid in range(NUM_JOINTS):
                    if jid not in orig_2d or jid not in edit_2d:
                        continue
                    o2 = orig_2d[jid]
                    e2 = edit_2d[jid]
                    if o2.get("confidence", 0) < 2 or e2.get("confidence", 0) < 2:
                        continue
                    du = e2["u"] - o2["u"]
                    dv = e2["v"] - o2["v"]
                    if abs(du) >= 0.5 or abs(dv) >= 0.5:
                        per_joint_counts[jid] += 1

    total_pairs = sum(len(v) for v in session_correspondences.values())
    if total_pairs == 0:
        kind = "prediction" if use_predictions else "edited"
        print("No %s correspondences found." % kind)
        return

    n_used_joints = sum(1 for c in per_joint_counts.values() if c > 0)
    print("\nTotal: %d correspondences across %d joint types" % (total_pairs, n_used_joints))

    # Show per-joint breakdown
    print("\n%3s %-18s %5s" % ("ID", "Joint", "N"))
    print("-" * 30)
    for jid in range(NUM_JOINTS):
        if per_joint_counts[jid] > 0:
            name = JOINT_NAMES[jid] if jid < len(JOINT_NAMES) else "JOINT_%d" % jid
            print("%3d %-18s %5d" % (jid, name, per_joint_counts[jid]))
    print()

    # -----------------------------------------------------------
    # Step 4: Optimize 6DOF delta (global or per-session)
    # -----------------------------------------------------------
    # session_results: key -> (params, rms_before, rms_after, n_pairs)
    session_results: Dict[str, Tuple[np.ndarray, float, float, int]] = {}

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
            params, rms_before, rms_after = fit_extrinsic_delta(
                pts_3d, pts_2d, fx, fy, cx, cy
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

        params, rms_before, rms_after = fit_extrinsic_delta(
            pts_3d, pts_2d, fx, fy, cx, cy
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
    # Step 5: Optionally apply to target
    # -----------------------------------------------------------
    if args.apply:
        mode = "2D + 3D" if args.apply_3d else "2D only"
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
            print("Estimating target intrinsics...")
            tgt_intrinsics = None
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
            # Pick the right delta for this annotations dir
            ann_key = str(ann_dir)
            if ann_key in session_results:
                params = session_results[ann_key][0]
            elif "_global" in session_results:
                params = session_results["_global"][0]
            else:
                # Per-session mode but this session wasn't in source -- skip
                continue

            rx, ry, rz, tx, ty, tz = params
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


if __name__ == "__main__":
    main()
