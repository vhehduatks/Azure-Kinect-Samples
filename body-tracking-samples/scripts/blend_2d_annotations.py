"""Blend projected 2D annotations with model-predicted 2D keypoints.

Four modes:
    vis-fix     — only recalculate visible flag from current u,v (no predictions needed)
    projection  — keep current u,v, recalculate visible (needs predictions dir to exist)
    prediction  — replace u,v with model predictions
    blend       — weighted average: u = (1-α)*u_proj + α*u_pred  where α = pred_confidence

Dataset structure:
    <dataset_root>/
        <participant>/
            <session>/
                ego_dataset/
                    annotations/    frame_*.json
                    predictions/    frame_*.json  (from train_pose_model.py predict-all)

Usage:
    python blend_2d_annotations.py --dataset-root <root> --mode vis-fix
    python blend_2d_annotations.py --dataset-root <root> --mode blend
    python blend_2d_annotations.py --session-dir <ego_dataset_dir> --mode prediction
"""

import argparse
import datetime
import json
import os
import shutil
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

NUM_JOINTS = 32


def discover_annotations_dirs(dataset_root: str) -> List[Path]:
    """Find all annotations/ dirs under <dataset_root>/<participant>/<session>/ego_dataset/."""
    root = Path(dataset_root)
    dirs = sorted(root.glob("*/*/ego_dataset/annotations"))
    return [d for d in dirs if d.is_dir()]


def blend_frame(
    ann_data: Dict,
    pred_data: Optional[Dict],
    mode: str,
    min_pred_conf: float,
) -> Tuple[bool, Dict]:
    """Blend a single frame's skeleton_2d with predictions.

    Returns (changed, stats) where stats tracks blend decisions.
    """
    stats = {
        "joints_processed": 0,
        "joints_blended": 0,
        "joints_projection_only": 0,
        "joints_no_prediction": 0,
        "alpha_sum": 0.0,
        "vis_to_invis": 0,
        "invis_to_vis": 0,
    }

    skel_2d = ann_data.get("skeleton_2d", [])
    if not skel_2d:
        return False, stats

    # Read image dimensions for visibility check
    ci = ann_data.get("camera_intrinsics", {})
    img_w = ci.get("width", int(2 * ci["cx"]) if "cx" in ci else None)
    img_h = ci.get("height", int(2 * ci["cy"]) if "cy" in ci else None)

    # Build prediction lookup
    pred_lookup: Dict[int, Dict] = {}
    if pred_data is not None:
        for e in pred_data.get("skeleton_2d_predicted", []):
            pred_lookup[e["joint_id"]] = e

    changed = False
    for entry in skel_2d:
        jid = entry["joint_id"]
        conf = entry.get("confidence", 0)

        if mode == "vis-fix":
            # Recalculate visible for ALL joints (including conf==0)
            # Treat (0,0) as a failed detection (not a real joint position)
            stats["joints_processed"] += 1
            old_vis = entry.get("visible", True)
            if img_w is not None and img_h is not None:
                at_origin = (entry["u"] == 0 and entry["v"] == 0)
                new_vis = bool(conf > 0 and not at_origin
                               and 0 <= entry["u"] < img_w and 0 <= entry["v"] < img_h)
                if new_vis != old_vis:
                    entry["visible"] = new_vis
                    changed = True
                    if old_vis and not new_vis:
                        stats["vis_to_invis"] += 1
                    elif not old_vis and new_vis:
                        stats["invis_to_vis"] += 1
            continue

        if conf == 0:
            continue

        stats["joints_processed"] += 1
        old_vis = entry.get("visible", True)

        pred = pred_lookup.get(jid)
        pred_conf = pred.get("confidence", 0) if pred else 0
        # Treat (0,0) predictions as failed detections
        if pred and pred_conf > 0 and pred.get("u", 0) == 0 and pred.get("v", 0) == 0:
            pred_conf = 0

        if mode == "projection":
            # Only recalculate visible flag, keep u,v
            pass

        elif mode == "prediction":
            if pred and pred_conf >= min_pred_conf:
                new_u = round(pred["u"], 2)
                new_v = round(pred["v"], 2)
                if abs(new_u - entry["u"]) > 0.001 or abs(new_v - entry["v"]) > 0.001:
                    entry["u"] = new_u
                    entry["v"] = new_v
                    changed = True
                stats["joints_blended"] += 1
                stats["alpha_sum"] += 1.0
            else:
                stats["joints_no_prediction"] += 1

        elif mode == "blend":
            alpha = pred_conf if (pred and pred_conf >= min_pred_conf) else 0.0
            stats["alpha_sum"] += alpha

            if alpha > 0:
                new_u = round((1 - alpha) * entry["u"] + alpha * pred["u"], 2)
                new_v = round((1 - alpha) * entry["v"] + alpha * pred["v"], 2)
                if abs(new_u - entry["u"]) > 0.001 or abs(new_v - entry["v"]) > 0.001:
                    entry["u"] = new_u
                    entry["v"] = new_v
                    changed = True
                stats["joints_blended"] += 1
            else:
                stats["joints_projection_only"] += 1
                if not pred or pred_conf < min_pred_conf:
                    stats["joints_no_prediction"] += 1

        # Recalculate visible flag (treat (0,0) as failed detection)
        if img_w is not None and img_h is not None:
            at_origin = (entry["u"] == 0 and entry["v"] == 0)
            new_vis = bool(conf > 0 and not at_origin
                           and 0 <= entry["u"] < img_w and 0 <= entry["v"] < img_h)
            if new_vis != old_vis:
                entry["visible"] = new_vis
                changed = True
                if old_vis and not new_vis:
                    stats["vis_to_invis"] += 1
                elif not old_vis and new_vis:
                    stats["invis_to_vis"] += 1

    return changed, stats


def blend_session(
    ego_dir: Path,
    mode: str,
    min_pred_conf: float,
    create_backup: bool,
    dry_run: bool,
) -> Dict:
    """Blend all frames in one ego_dataset session.

    Returns aggregate stats dict.
    """
    ann_dir = ego_dir / "annotations"
    pred_dir = ego_dir / "predictions"

    if not ann_dir.is_dir():
        return {"error": "no annotations/ dir"}

    has_predictions = pred_dir.is_dir()
    if mode not in ("projection", "vis-fix") and not has_predictions:
        return {"error": f"mode={mode} requires predictions/ dir"}

    json_files = sorted(ann_dir.glob("frame_*.json"))
    agg = {
        "frames": 0,
        "frames_changed": 0,
        "joints_processed": 0,
        "joints_blended": 0,
        "joints_projection_only": 0,
        "joints_no_prediction": 0,
        "alpha_sum": 0.0,
        "vis_to_invis": 0,
        "invis_to_vis": 0,
    }

    for json_path in json_files:
        with open(json_path, encoding="utf-8") as f:
            ann_data = json.load(f)

        pred_data = None
        if has_predictions:
            pred_path = pred_dir / json_path.name
            if pred_path.exists():
                with open(pred_path, encoding="utf-8") as f:
                    pred_data = json.load(f)

        frame_changed, stats = blend_frame(ann_data, pred_data, mode, min_pred_conf)
        agg["frames"] += 1
        for k in ("joints_processed", "joints_blended", "joints_projection_only",
                   "joints_no_prediction", "alpha_sum", "vis_to_invis", "invis_to_vis"):
            agg[k] += stats[k]

        if frame_changed:
            agg["frames_changed"] += 1
            if not dry_run:
                if create_backup:
                    bak_path = str(json_path) + ".bak"
                    if not os.path.exists(bak_path):
                        shutil.copy2(json_path, bak_path)
                tmp = str(json_path) + ".tmp"
                with open(tmp, "w", encoding="utf-8") as f:
                    json.dump(ann_data, f, indent=2)
                os.replace(tmp, json_path)

    return agg


def print_summary(all_stats: List[Tuple[str, Dict]], mode: str):
    """Print aggregate summary table."""
    totals = {
        "sessions": 0,
        "frames": 0,
        "frames_changed": 0,
        "joints_processed": 0,
        "joints_blended": 0,
        "joints_projection_only": 0,
        "joints_no_prediction": 0,
        "alpha_sum": 0.0,
        "vis_to_invis": 0,
        "invis_to_vis": 0,
        "errors": 0,
    }
    for name, stats in all_stats:
        if "error" in stats:
            totals["errors"] += 1
            continue
        totals["sessions"] += 1
        for k in ("frames", "frames_changed", "joints_processed", "joints_blended",
                   "joints_projection_only", "joints_no_prediction", "alpha_sum",
                   "vis_to_invis", "invis_to_vis"):
            totals[k] += stats[k]

    print(f"\n{'='*60}")
    print(f"Blend 2D Summary  (mode={mode})")
    print(f"{'='*60}")
    print(f"  Sessions:            {totals['sessions']}")
    if totals["errors"]:
        print(f"  Errors/skipped:      {totals['errors']}")
    print(f"  Frames:              {totals['frames']}")
    print(f"  Frames changed:      {totals['frames_changed']}")
    print(f"  Joints processed:    {totals['joints_processed']}")
    if mode not in ("projection", "vis-fix"):
        print(f"  Joints blended:      {totals['joints_blended']}")
        print(f"  Joints proj-only:    {totals['joints_projection_only']}")
        print(f"  Joints no-pred:      {totals['joints_no_prediction']}")
        avg_alpha = (totals["alpha_sum"] / totals["joints_processed"]
                     if totals["joints_processed"] else 0)
        print(f"  Average alpha:       {avg_alpha:.4f}")
    print(f"  Vis -> Invis:        {totals['vis_to_invis']}")
    print(f"  Invis -> Vis:        {totals['invis_to_vis']}")
    print(f"{'='*60}")


def main():
    parser = argparse.ArgumentParser(
        description="Blend projected 2D annotations with model predictions.")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--dataset-root",
                       help="Root of hierarchical dataset (discovers all sessions)")
    group.add_argument("--session-dir",
                       help="Single ego_dataset/ directory")
    parser.add_argument("--mode", required=True,
                        choices=["vis-fix", "projection", "prediction", "blend"],
                        help="Blending mode (vis-fix: only recalculate visible flags)")
    parser.add_argument("--min-pred-confidence", type=float, default=0.1,
                        help="Minimum prediction confidence to use (default: 0.1)")
    parser.add_argument("--no-backup", action="store_true",
                        help="Skip creating .bak backup files")
    parser.add_argument("--dry-run", action="store_true",
                        help="Report changes without writing files")
    args = parser.parse_args()

    try:
        from tqdm import tqdm
    except ImportError:
        def tqdm(it, **kw):
            return it

    # Discover sessions
    sessions: List[Tuple[str, Path]] = []
    if args.dataset_root:
        ann_dirs = discover_annotations_dirs(args.dataset_root)
        for ad in ann_dirs:
            ego_dir = ad.parent  # annotations/ -> ego_dataset/
            label = str(ego_dir.relative_to(args.dataset_root))
            sessions.append((label, ego_dir))
    else:
        ego_dir = Path(args.session_dir)
        sessions.append((ego_dir.name, ego_dir))

    if not sessions:
        print("No sessions found.", file=sys.stderr)
        sys.exit(1)

    print(f"Mode: {args.mode} | Sessions: {len(sessions)} | "
          f"Min pred conf: {args.min_pred_confidence}"
          + (" | DRY RUN" if args.dry_run else ""))

    all_stats: List[Tuple[str, Dict]] = []
    for label, ego_dir in tqdm(sessions, desc="Sessions"):
        stats = blend_session(
            ego_dir,
            mode=args.mode,
            min_pred_conf=args.min_pred_confidence,
            create_backup=not args.no_backup,
            dry_run=args.dry_run,
        )
        all_stats.append((label, stats))
        if "error" in stats:
            print(f"  SKIP {label}: {stats['error']}")
        else:
            print(f"  {label}: {stats['frames_changed']}/{stats['frames']} frames changed, "
                  f"vis: +{stats['invis_to_vis']}/-{stats['vis_to_invis']}")

    print_summary(all_stats, args.mode)

    # Save log file
    if not args.dry_run:
        dataset_dir = args.dataset_root or args.session_dir
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = os.path.join(dataset_dir, f"blend_log_{timestamp}.json")

        totals = {
            "sessions": 0,
            "frames": 0,
            "frames_changed": 0,
            "joints_processed": 0,
            "joints_blended": 0,
            "joints_projection_only": 0,
            "joints_no_prediction": 0,
            "alpha_sum": 0.0,
            "vis_to_invis": 0,
            "invis_to_vis": 0,
            "errors": 0,
        }
        session_logs = []
        for name, stats in all_stats:
            if "error" in stats:
                totals["errors"] += 1
                session_logs.append({"session": name, "error": stats["error"]})
                continue
            totals["sessions"] += 1
            for k in ("frames", "frames_changed", "joints_processed", "joints_blended",
                       "joints_projection_only", "joints_no_prediction", "alpha_sum",
                       "vis_to_invis", "invis_to_vis"):
                totals[k] += stats[k]
            avg_alpha = (stats["alpha_sum"] / stats["joints_processed"]
                         if stats["joints_processed"] else 0)
            session_logs.append({
                "session": name,
                "frames": stats["frames"],
                "frames_changed": stats["frames_changed"],
                "joints_processed": stats["joints_processed"],
                "joints_blended": stats["joints_blended"],
                "joints_projection_only": stats["joints_projection_only"],
                "joints_no_prediction": stats["joints_no_prediction"],
                "avg_alpha": round(avg_alpha, 4),
                "vis_to_invis": stats["vis_to_invis"],
                "invis_to_vis": stats["invis_to_vis"],
            })

        avg_alpha_total = (totals["alpha_sum"] / totals["joints_processed"]
                           if totals["joints_processed"] else 0)
        log_data = {
            "timestamp": timestamp,
            "command": {
                "dataset_root": args.dataset_root,
                "session_dir": args.session_dir,
                "mode": args.mode,
                "min_pred_confidence": args.min_pred_confidence,
                "no_backup": args.no_backup,
            },
            "summary": {
                "sessions": totals["sessions"],
                "errors": totals["errors"],
                "frames": totals["frames"],
                "frames_changed": totals["frames_changed"],
                "joints_processed": totals["joints_processed"],
                "joints_blended": totals["joints_blended"],
                "joints_projection_only": totals["joints_projection_only"],
                "joints_no_prediction": totals["joints_no_prediction"],
                "avg_alpha": round(avg_alpha_total, 4),
                "vis_to_invis": totals["vis_to_invis"],
                "invis_to_vis": totals["invis_to_vis"],
            },
            "sessions": session_logs,
        }

        try:
            with open(log_path, "w", encoding="utf-8") as f:
                json.dump(log_data, f, indent=2)
            print(f"\nBlend log saved: {log_path}")
        except PermissionError:
            # Fall back to current directory
            fallback = f"blend_log_{timestamp}.json"
            with open(fallback, "w", encoding="utf-8") as f:
                json.dump(log_data, f, indent=2)
            print(f"\nBlend log saved: {fallback}")


if __name__ == "__main__":
    main()
