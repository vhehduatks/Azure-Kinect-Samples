#!/usr/bin/env python3
"""Validate extrinsic fit results: detect outlier sessions/frames and
compare projected 3D joints against model predictions.

Reads the optimization log JSON from fit_extrinsic.py plus the original
annotation and prediction data to produce per-frame diagnostic reports.

Usage:
    # Basic validation from log file
    python validate_fit.py <dataset_root> --log fit_extrinsic_log_xxx.json

    # Custom thresholds
    python validate_fit.py <dataset_root> --log fit.json --tz-max 150 --rotation-max 5

    # Include per-frame projection vs prediction comparison
    python validate_fit.py <dataset_root> --log fit.json --predictions

    # Export flagged frames to CSV
    python validate_fit.py <dataset_root> --log fit.json --predictions -o flagged.csv
"""

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Constants (shared with fit_extrinsic.py)
# ---------------------------------------------------------------------------

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

EXCLUDED_JOINTS_DEFAULT = {8, 9, 10, 15, 16, 17}

_BODY_PARTS = {
    'spine':     [0, 1, 2, 3],
    'head':      [26, 27, 28, 29, 30, 31],
    'left_arm':  [4, 5, 6, 7],
    'right_arm': [11, 12, 13, 14],
    'left_leg':  [18, 19, 20, 21],
    'right_leg': [22, 23, 24, 25],
}

_JOINT_TO_PART = {}
for _part, _jids in _BODY_PARTS.items():
    for _jid in _jids:
        _JOINT_TO_PART[_jid] = _part


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _euler_to_rotation_matrix(rx_deg: float, ry_deg: float, rz_deg: float) -> np.ndarray:
    rx, ry, rz = math.radians(rx_deg), math.radians(ry_deg), math.radians(rz_deg)
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _project_pinhole(x, y, z, fx, fy, cx, cy):
    if z <= 0:
        return (0.0, 0.0)
    return (fx * x / z + cx, fy * y / z + cy)


def discover_annotations_dirs(dataset_root: str) -> List[Path]:
    root = Path(dataset_root)
    dirs = sorted(root.glob("*/*/ego_dataset/annotations"))
    return [d for d in dirs if d.is_dir()]


def _session_label(ann_dir_str: str, source_dir: str) -> str:
    try:
        rel = Path(ann_dir_str).relative_to(Path(source_dir))
        parts = rel.parts
        return "/".join(parts[:2]) if len(parts) >= 2 else str(rel)
    except ValueError:
        return Path(ann_dir_str).name


# ---------------------------------------------------------------------------
# 1. Delta outlier detection
# ---------------------------------------------------------------------------

def detect_delta_outliers(
    log_data: dict,
    rotation_max: float = 5.0,
    translation_max: float = 150.0,
    tz_max: Optional[float] = None,
    iqr_factor: float = 1.5,
) -> dict:
    """Detect outlier sessions based on 6DOF delta values.

    Two methods:
      - Absolute thresholds: flag if any parameter exceeds max
      - IQR-based: flag if outside [Q1 - iqr_factor*IQR, Q3 + iqr_factor*IQR]

    Returns dict with outlier info.
    """
    sessions = log_data.get("sessions", [])
    if not sessions:
        return {"outliers": [], "summary": {}}

    # Extract parameter arrays
    param_names = ["rx_deg", "ry_deg", "rz_deg", "tx_mm", "ty_mm", "tz_mm"]
    param_arrays = {p: [] for p in param_names}
    for s in sessions:
        d = s.get("delta", {})
        for p in param_names:
            param_arrays[p].append(d.get(p, 0.0))

    for p in param_names:
        param_arrays[p] = np.array(param_arrays[p])

    # Compute IQR bounds
    iqr_bounds = {}
    for p in param_names:
        vals = param_arrays[p]
        q1, q3 = np.percentile(vals, 25), np.percentile(vals, 75)
        iqr = q3 - q1
        iqr_bounds[p] = {
            "q1": q1, "q3": q3, "iqr": iqr,
            "lower": q1 - iqr_factor * iqr,
            "upper": q3 + iqr_factor * iqr,
        }

    # Check each session
    outliers = []
    for s in sessions:
        d = s.get("delta", {})
        flags = []

        # Absolute threshold checks
        for rp in ["rx_deg", "ry_deg", "rz_deg"]:
            v = d.get(rp, 0.0)
            if abs(v) > rotation_max:
                flags.append(("abs_threshold", rp, v, rotation_max))

        for tp in ["tx_mm", "ty_mm", "tz_mm"]:
            v = d.get(tp, 0.0)
            limit = tz_max if (tp == "tz_mm" and tz_max is not None) else translation_max
            if abs(v) > limit:
                flags.append(("abs_threshold", tp, v, limit))

        # IQR checks
        for p in param_names:
            v = d.get(p, 0.0)
            bounds = iqr_bounds[p]
            if v < bounds["lower"] or v > bounds["upper"]:
                flags.append(("iqr_outlier", p, v, (bounds["lower"], bounds["upper"])))

        if flags:
            outliers.append({
                "session": s.get("session", "?"),
                "n_pairs": s.get("n_pairs", 0),
                "delta": d,
                "flags": flags,
            })

    # Parameter statistics
    stats = {}
    for p in param_names:
        vals = param_arrays[p]
        stats[p] = {
            "mean": float(vals.mean()),
            "std": float(vals.std()),
            "median": float(np.median(vals)),
            "q1": float(iqr_bounds[p]["q1"]),
            "q3": float(iqr_bounds[p]["q3"]),
            "iqr": float(iqr_bounds[p]["iqr"]),
            "iqr_lower": float(iqr_bounds[p]["lower"]),
            "iqr_upper": float(iqr_bounds[p]["upper"]),
            "min": float(vals.min()),
            "max": float(vals.max()),
        }

    return {
        "outliers": outliers,
        "stats": stats,
        "n_sessions": len(sessions),
        "n_outliers": len(outliers),
    }


# ---------------------------------------------------------------------------
# 2. Per-frame projection vs prediction comparison
# ---------------------------------------------------------------------------

def compare_projection_vs_prediction(
    dataset_root: str,
    log_data: dict,
    min_pred_confidence: float = 0.5,
    excluded_joints: Optional[set] = None,
    distance_threshold: float = 30.0,
) -> List[dict]:
    """Compare proj(R @ P3d + t) against model-predicted 2D joints per frame.

    For each frame that has both annotation (3D) and prediction (2D) data,
    project the 3D skeleton using the session's fitted delta, then compute
    per-joint pixel distance to the model's predicted 2D positions.

    Flags frames where mean or max distance exceeds the threshold.

    Returns list of per-frame records (flagged frames only).
    """
    excl = excluded_joints if excluded_joints is not None else EXCLUDED_JOINTS_DEFAULT

    # Get intrinsics from log
    intr = log_data.get("intrinsics", {})
    fx, fy = intr.get("fx", 0), intr.get("fy", 0)
    cx, cy = intr.get("cx", 0), intr.get("cy", 0)
    if fx == 0 or fy == 0:
        print("Error: no intrinsics in log file")
        return []

    # Build session -> delta mapping from log
    session_deltas: Dict[str, np.ndarray] = {}
    for s in log_data.get("sessions", []):
        d = s.get("delta", {})
        params = np.array([
            d.get("rx_deg", 0), d.get("ry_deg", 0), d.get("rz_deg", 0),
            d.get("tx_mm", 0), d.get("ty_mm", 0), d.get("tz_mm", 0),
        ])
        session_deltas[s["session"]] = params

    # Discover annotation dirs and match to sessions
    root = Path(dataset_root)
    ann_dirs = discover_annotations_dirs(dataset_root)

    flagged_frames = []
    total_frames = 0
    total_compared = 0

    for ann_dir in ann_dirs:
        # Find matching session in log
        label = _session_label(str(ann_dir), dataset_root)
        if label not in session_deltas:
            continue

        params = session_deltas[label]
        rx, ry, rz, tx, ty, tz = params
        R = _euler_to_rotation_matrix(rx, ry, rz)
        t_vec = np.array([tx, ty, tz], dtype=np.float64)

        # Check for predictions dir
        pred_dir = ann_dir.parent / "predictions"
        if not pred_dir.exists():
            continue

        pred_files = sorted(pred_dir.glob("frame_*.json"))
        for pred_path in pred_files:
            ann_path = ann_dir / pred_path.name
            if not ann_path.exists():
                continue

            total_frames += 1

            with open(ann_path, encoding="utf-8") as f:
                ann_data = json.load(f)
            with open(pred_path, encoding="utf-8") as f:
                pred_data = json.load(f)

            skel_3d = {e["joint_id"]: e for e in ann_data.get("skeleton_3d", [])}
            skel_pred = {e["joint_id"]: e
                         for e in pred_data.get("skeleton_2d_predicted", [])}

            # Also try reading per-frame intrinsics from annotation
            ci = ann_data.get("camera_intrinsics")
            f_fx, f_fy, f_cx, f_cy = fx, fy, cx, cy
            if ci and "fx" in ci:
                f_fx, f_fy = ci["fx"], ci["fy"]
                f_cx, f_cy = ci["cx"], ci["cy"]

            joint_distances = []
            joint_details = []

            for jid in range(NUM_JOINTS):
                if jid in excl:
                    continue
                if jid not in skel_3d or jid not in skel_pred:
                    continue

                j3 = skel_3d[jid]
                jp = skel_pred[jid]

                if j3.get("confidence", 0) < 2:
                    continue
                if jp.get("confidence", 0) < min_pred_confidence:
                    continue
                if j3["z"] < 100:
                    continue

                # Project 3D with fitted delta
                p3d = np.array([j3["x"], j3["y"], j3["z"]], dtype=np.float64)
                p_transformed = R @ p3d + t_vec
                if p_transformed[2] <= 0:
                    continue

                proj_u, proj_v = _project_pinhole(
                    p_transformed[0], p_transformed[1], p_transformed[2],
                    f_fx, f_fy, f_cx, f_cy,
                )

                pred_u, pred_v = jp["u"], jp["v"]
                dist = math.sqrt((proj_u - pred_u) ** 2 + (proj_v - pred_v) ** 2)

                joint_distances.append(dist)
                joint_details.append({
                    "joint_id": jid,
                    "name": JOINT_NAMES[jid] if jid < len(JOINT_NAMES) else str(jid),
                    "part": _JOINT_TO_PART.get(jid, "?"),
                    "proj_u": round(proj_u, 1),
                    "proj_v": round(proj_v, 1),
                    "pred_u": round(pred_u, 1),
                    "pred_v": round(pred_v, 1),
                    "distance_px": round(dist, 1),
                })

            if not joint_distances:
                continue
            total_compared += 1

            mean_dist = float(np.mean(joint_distances))
            max_dist = float(np.max(joint_distances))
            median_dist = float(np.median(joint_distances))

            # Flag if exceeds threshold
            is_flagged = mean_dist > distance_threshold or max_dist > distance_threshold * 3

            if is_flagged:
                # Sort details by distance descending
                joint_details.sort(key=lambda x: x["distance_px"], reverse=True)
                flagged_frames.append({
                    "session": label,
                    "frame": pred_path.stem,
                    "n_joints": len(joint_distances),
                    "mean_dist_px": round(mean_dist, 1),
                    "median_dist_px": round(median_dist, 1),
                    "max_dist_px": round(max_dist, 1),
                    "worst_joints": joint_details[:5],
                })

    return flagged_frames


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------

def print_delta_report(result: dict):
    """Print delta outlier detection report."""
    stats = result.get("stats", {})
    outliers = result.get("outliers", [])

    print("=" * 70)
    print("DELTA OUTLIER REPORT (%d sessions, %d outliers)"
          % (result["n_sessions"], result["n_outliers"]))
    print("=" * 70)

    # Statistics table
    print("\n%10s %8s %8s %8s %8s %10s %10s" %
          ("param", "mean", "std", "median", "IQR", "IQR_lower", "IQR_upper"))
    print("-" * 70)
    for p in ["rx_deg", "ry_deg", "rz_deg", "tx_mm", "ty_mm", "tz_mm"]:
        s = stats.get(p, {})
        print("%10s %+8.3f %8.3f %+8.3f %8.3f %+10.3f %+10.3f" % (
            p, s.get("mean", 0), s.get("std", 0), s.get("median", 0),
            s.get("iqr", 0), s.get("iqr_lower", 0), s.get("iqr_upper", 0),
        ))

    if not outliers:
        print("\nNo outlier sessions detected.")
        return

    # Outlier details
    print("\nOutlier sessions:")
    print("-" * 70)
    for o in outliers:
        d = o["delta"]
        print("\n  Session: %s (%d pairs)" % (o["session"], o["n_pairs"]))
        print("  Delta: r=(%+.3f, %+.3f, %+.3f) t=(%+.1f, %+.1f, %+.1f)" % (
            d["rx_deg"], d["ry_deg"], d["rz_deg"],
            d["tx_mm"], d["ty_mm"], d["tz_mm"],
        ))
        for flag_type, param, val, limit in o["flags"]:
            if flag_type == "abs_threshold":
                print("    [ABS] %s = %+.3f (threshold: %.3f)" % (param, val, limit))
            elif flag_type == "iqr_outlier":
                lo, hi = limit
                print("    [IQR] %s = %+.3f (bounds: [%+.3f, %+.3f])" % (param, val, lo, hi))


def print_projection_report(flagged: List[dict], total_compared: int = 0):
    """Print projection vs prediction comparison report."""
    print("\n" + "=" * 70)
    print("PROJECTION VS PREDICTION REPORT (%d flagged frames)" % len(flagged))
    print("=" * 70)

    if not flagged:
        print("\nNo frames flagged (all within threshold).")
        return

    # Group by session
    by_session: Dict[str, List[dict]] = {}
    for f in flagged:
        by_session.setdefault(f["session"], []).append(f)

    print("\nSessions with flagged frames: %d" % len(by_session))
    print("-" * 70)

    for session in sorted(by_session.keys()):
        frames = by_session[session]
        # Sort by mean_dist descending
        frames.sort(key=lambda x: x["mean_dist_px"], reverse=True)

        mean_of_means = np.mean([f["mean_dist_px"] for f in frames])
        max_of_maxes = max(f["max_dist_px"] for f in frames)

        print("\n  [%s] %d flagged frames (avg mean=%.1f px, worst max=%.1f px)"
              % (session, len(frames), mean_of_means, max_of_maxes))

        # Show top 5 worst frames
        for f in frames[:5]:
            worst = f["worst_joints"]
            worst_str = ", ".join(
                "%s:%.0fpx" % (j["name"], j["distance_px"])
                for j in worst[:3]
            )
            print("    %s: mean=%.1f median=%.1f max=%.1f (%d joints) [%s]"
                  % (f["frame"], f["mean_dist_px"], f["median_dist_px"],
                     f["max_dist_px"], f["n_joints"], worst_str))

        if len(frames) > 5:
            print("    ... and %d more" % (len(frames) - 5))


def export_flagged_csv(flagged: List[dict], output_path: str):
    """Export flagged frames to CSV."""
    if not flagged:
        print("No flagged frames to export.")
        return

    with open(output_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            "session", "frame", "n_joints",
            "mean_dist_px", "median_dist_px", "max_dist_px",
            "worst_joint", "worst_joint_dist_px",
        ])
        for fr in flagged:
            worst = fr["worst_joints"][0] if fr["worst_joints"] else {}
            writer.writerow([
                fr["session"], fr["frame"], fr["n_joints"],
                fr["mean_dist_px"], fr["median_dist_px"], fr["max_dist_px"],
                worst.get("name", ""), worst.get("distance_px", ""),
            ])

    print("\nFlagged frames exported to: %s (%d rows)" % (output_path, len(flagged)))


def export_report_json(delta_result: dict, flagged: List[dict], output_path: str):
    """Export full validation report as JSON."""
    report = {
        "delta_outliers": delta_result,
        "flagged_frames": flagged,
        "n_flagged_frames": len(flagged),
    }
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, ensure_ascii=False, default=str)
    print("Full report exported to: %s" % output_path)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Validate extrinsic fit results: detect outlier sessions "
        "and compare projected 3D vs predicted 2D joints.",
    )
    parser.add_argument("dataset_root", help="Dataset root directory")
    parser.add_argument("--log", required=True,
                        help="Path to fit_extrinsic_log_xxx.json")

    # Delta outlier thresholds
    parser.add_argument("--rotation-max", type=float, default=5.0,
                        help="Absolute rotation threshold (deg, default: 5.0)")
    parser.add_argument("--translation-max", type=float, default=150.0,
                        help="Absolute translation threshold (mm, default: 150.0)")
    parser.add_argument("--tz-max", type=float, default=None,
                        help="Override tz threshold (mm, default: same as --translation-max)")
    parser.add_argument("--iqr-factor", type=float, default=1.5,
                        help="IQR multiplier for outlier detection (default: 1.5)")

    # Projection vs prediction
    parser.add_argument("--predictions", action="store_true",
                        help="Compare projected 3D joints vs model predictions")
    parser.add_argument("--min-pred-confidence", type=float, default=0.5,
                        help="Min prediction confidence for comparison (default: 0.5)")
    parser.add_argument("--distance-threshold", type=float, default=30.0,
                        help="Flag frames where mean joint distance > threshold px (default: 30.0)")

    # Exclusions
    parser.add_argument("--exclude-joints", type=str, default=None,
                        help="Comma-separated joint IDs to exclude (default: hand joints 8-10,15-17). "
                             "Use 'none' to include all joints.")

    # Output
    parser.add_argument("-o", "--output", type=str, default=None,
                        help="Export flagged frames to CSV file")
    parser.add_argument("--json", type=str, default=None,
                        help="Export full validation report as JSON")

    args = parser.parse_args()

    # Load log
    log_path = Path(args.log)
    if not log_path.exists():
        print("Error: log file not found: %s" % args.log)
        sys.exit(1)

    with open(log_path, encoding="utf-8") as f:
        log_data = json.load(f)

    print("Loaded log: %s" % args.log)
    print("  Sessions: %d" % len(log_data.get("sessions", [])))
    intr = log_data.get("intrinsics", {})
    print("  Intrinsics: fx=%.1f fy=%.1f cx=%.1f cy=%.1f (%s)"
          % (intr.get("fx", 0), intr.get("fy", 0),
             intr.get("cx", 0), intr.get("cy", 0),
             intr.get("source", "unknown")))

    # Parse excluded joints
    if args.exclude_joints is not None:
        if args.exclude_joints.lower() == "none":
            excluded_joints = set()
        else:
            excluded_joints = set(int(x.strip()) for x in args.exclude_joints.split(","))
    else:
        excluded_joints = EXCLUDED_JOINTS_DEFAULT

    # 1. Delta outlier detection
    delta_result = detect_delta_outliers(
        log_data,
        rotation_max=args.rotation_max,
        translation_max=args.translation_max,
        tz_max=args.tz_max,
        iqr_factor=args.iqr_factor,
    )
    print_delta_report(delta_result)

    # 2. Projection vs prediction comparison
    flagged = []
    if args.predictions:
        print("\nComparing projected 3D vs predicted 2D joints...")
        flagged = compare_projection_vs_prediction(
            dataset_root=args.dataset_root,
            log_data=log_data,
            min_pred_confidence=args.min_pred_confidence,
            excluded_joints=excluded_joints,
            distance_threshold=args.distance_threshold,
        )
        print_projection_report(flagged)

    # Export
    if args.output and flagged:
        export_flagged_csv(flagged, args.output)

    if args.json:
        export_report_json(delta_result, flagged, args.json)


if __name__ == "__main__":
    main()
