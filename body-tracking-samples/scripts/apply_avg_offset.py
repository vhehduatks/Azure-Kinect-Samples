"""Compute and apply average annotation offsets across a full dataset.

Dataset structure:
    <dataset_root>/
        <participant>/              e.g. 021903/
            <session>/              e.g. Dancing1_20260219_200818/
                ego_dataset/
                    annotations/    frame_*.json  (+  frame_*.json.bak if edited)
                    images/
                    metadata.json

Function 1: compute_average_offsets()
    Compares edited annotation JSONs (.json) against their backups (.json.bak)
    to compute per-joint average offsets in both 2D (du, dv) and 3D (dx, dy, dz).

Function 2: apply_offsets()
    Applies the computed offsets to all annotation JSONs in a directory.

Usage:
    # Print offsets from dataset (a)
    python apply_avg_offset.py <source_dataset_root>

    # Compute from (a), apply to (b)
    python apply_avg_offset.py <source_root> --target <target_root> --apply

    # Apply 2D + 3D
    python apply_avg_offset.py <source_root> --target <target_root> --apply --3d

    # Single annotations dir (legacy)
    python apply_avg_offset.py <annotations_dir> --flat
"""

import argparse
import json
import os
import shutil
from pathlib import Path
from typing import Dict, List, Optional, Tuple

NUM_JOINTS = 32


def discover_annotations_dirs(dataset_root: str) -> List[Path]:
    """Find all annotations/ dirs under <dataset_root>/<participant>/<session>/ego_dataset/."""
    root = Path(dataset_root)
    dirs = sorted(root.glob("*/*/ego_dataset/annotations"))
    return [d for d in dirs if d.is_dir()]


def compute_average_offsets(
    annotations_dir: str,
) -> Tuple[Dict[int, Tuple[float, float]], Dict[int, Tuple[float, float, float]], Dict[int, int]]:
    """Compare edited JSONs against .bak originals to compute per-joint avg offsets.

    Only considers frames where a .bak file exists (i.e., the annotation tool
    saved edits) and where the joint has confidence > 0 in both versions.

    Parameters
    ----------
    annotations_dir : path to a single annotations/ folder containing .json and .json.bak

    Returns
    -------
    offsets_2d : {joint_id: (avg_du, avg_dv)}
    offsets_3d : {joint_id: (avg_dx, avg_dy, avg_dz)}
    counts     : {joint_id: number_of_samples}
    """
    ann_dir = Path(annotations_dir)
    json_files = sorted(ann_dir.glob("frame_*.json"))

    # Accumulators: joint_id -> [sum_du, sum_dv, sum_dx, sum_dy, sum_dz, count]
    accum: Dict[int, List[float]] = {}
    for jid in range(NUM_JOINTS):
        accum[jid] = [0.0, 0.0, 0.0, 0.0, 0.0, 0]

    edited_frames = 0

    for json_path in json_files:
        bak_path = Path(str(json_path) + ".bak")
        if not bak_path.exists():
            continue

        with open(json_path, encoding="utf-8") as f:
            edited = json.load(f)
        with open(bak_path, encoding="utf-8") as f:
            original = json.load(f)

        # Build lookup by joint_id
        orig_2d = {e["joint_id"]: e for e in original.get("skeleton_2d", [])}
        edit_2d = {e["joint_id"]: e for e in edited.get("skeleton_2d", [])}
        orig_3d = {e["joint_id"]: e for e in original.get("skeleton_3d", [])}
        edit_3d = {e["joint_id"]: e for e in edited.get("skeleton_3d", [])}

        frame_has_edit = False

        for jid in range(NUM_JOINTS):
            # 2D offset
            if jid in orig_2d and jid in edit_2d:
                o2 = orig_2d[jid]
                e2 = edit_2d[jid]
                # Only consider confident joints
                if o2.get("confidence", 0) > 0 and e2.get("confidence", 0) > 0:
                    du = e2["u"] - o2["u"]
                    dv = e2["v"] - o2["v"]
                    if abs(du) > 0.001 or abs(dv) > 0.001:
                        accum[jid][0] += du
                        accum[jid][1] += dv
                        frame_has_edit = True

            # 3D offset
            if jid in orig_3d and jid in edit_3d:
                o3 = orig_3d[jid]
                e3 = edit_3d[jid]
                if o3.get("confidence", 0) > 0 and e3.get("confidence", 0) > 0:
                    dx = e3["x"] - o3["x"]
                    dy = e3["y"] - o3["y"]
                    dz = e3["z"] - o3["z"]
                    if abs(dx) > 0.001 or abs(dy) > 0.001 or abs(dz) > 0.001:
                        accum[jid][2] += dx
                        accum[jid][3] += dy
                        accum[jid][4] += dz

            # Count frames with any 2D or 3D change
            if jid in orig_2d and jid in edit_2d:
                o2 = orig_2d[jid]
                e2 = edit_2d[jid]
                if o2.get("confidence", 0) > 0 and e2.get("confidence", 0) > 0:
                    du = e2["u"] - o2["u"]
                    dv = e2["v"] - o2["v"]
                    has_2d = abs(du) > 0.001 or abs(dv) > 0.001
                else:
                    has_2d = False
            else:
                has_2d = False

            if jid in orig_3d and jid in edit_3d:
                o3 = orig_3d[jid]
                e3 = edit_3d[jid]
                if o3.get("confidence", 0) > 0 and e3.get("confidence", 0) > 0:
                    dx = e3["x"] - o3["x"]
                    dy = e3["y"] - o3["y"]
                    dz = e3["z"] - o3["z"]
                    has_3d = abs(dx) > 0.001 or abs(dy) > 0.001 or abs(dz) > 0.001
                else:
                    has_3d = False
            else:
                has_3d = False

            if has_2d or has_3d:
                accum[jid][5] += 1

        if frame_has_edit:
            edited_frames += 1

    # Compute averages
    offsets_2d: Dict[int, Tuple[float, float]] = {}
    offsets_3d: Dict[int, Tuple[float, float, float]] = {}
    counts: Dict[int, int] = {}

    for jid in range(NUM_JOINTS):
        n = int(accum[jid][5])
        counts[jid] = n
        if n > 0:
            offsets_2d[jid] = (accum[jid][0] / n, accum[jid][1] / n)
            offsets_3d[jid] = (accum[jid][2] / n, accum[jid][3] / n, accum[jid][4] / n)
        else:
            offsets_2d[jid] = (0.0, 0.0)
            offsets_3d[jid] = (0.0, 0.0, 0.0)

    print(f"  Scanned {len(json_files)} frames, "
          f"{edited_frames} have edits (with .bak)")

    return offsets_2d, offsets_3d, counts


def merge_offsets(
    all_results: List[Tuple[
        Dict[int, Tuple[float, float]],
        Dict[int, Tuple[float, float, float]],
        Dict[int, int],
    ]],
) -> Tuple[Dict[int, Tuple[float, float]], Dict[int, Tuple[float, float, float]], Dict[int, int]]:
    """Merge per-session offset results into a single weighted average."""
    total_accum: Dict[int, List[float]] = {}
    for jid in range(NUM_JOINTS):
        total_accum[jid] = [0.0, 0.0, 0.0, 0.0, 0.0, 0]

    for o2d, o3d, cnts in all_results:
        for jid in range(NUM_JOINTS):
            n = cnts[jid]
            if n > 0:
                total_accum[jid][0] += o2d[jid][0] * n
                total_accum[jid][1] += o2d[jid][1] * n
                total_accum[jid][2] += o3d[jid][0] * n
                total_accum[jid][3] += o3d[jid][1] * n
                total_accum[jid][4] += o3d[jid][2] * n
                total_accum[jid][5] += n

    offsets_2d: Dict[int, Tuple[float, float]] = {}
    offsets_3d: Dict[int, Tuple[float, float, float]] = {}
    counts: Dict[int, int] = {}

    for jid in range(NUM_JOINTS):
        n = int(total_accum[jid][5])
        counts[jid] = n
        if n > 0:
            offsets_2d[jid] = (total_accum[jid][0] / n, total_accum[jid][1] / n)
            offsets_3d[jid] = (
                total_accum[jid][2] / n,
                total_accum[jid][3] / n,
                total_accum[jid][4] / n,
            )
        else:
            offsets_2d[jid] = (0.0, 0.0)
            offsets_3d[jid] = (0.0, 0.0, 0.0)

    return offsets_2d, offsets_3d, counts


def apply_offsets(
    annotations_dir: str,
    offsets_2d: Dict[int, Tuple[float, float]],
    offsets_3d: Optional[Dict[int, Tuple[float, float, float]]] = None,
    create_backup: bool = True,
) -> int:
    """Apply per-joint offsets to all annotation JSONs in a single annotations/ dir.

    Parameters
    ----------
    annotations_dir : path to the annotations/ folder
    offsets_2d : {joint_id: (du, dv)} to add to each joint's 2D position
    offsets_3d : {joint_id: (dx, dy, dz)} to add to each joint's 3D position.
                 If None, only 2D is modified.
    create_backup : if True, create .bak backup before overwriting

    Returns
    -------
    Number of files modified.
    """
    ann_dir = Path(annotations_dir)
    json_files = sorted(ann_dir.glob("frame_*.json"))

    modified = 0
    for json_path in json_files:
        with open(json_path, encoding="utf-8") as f:
            data = json.load(f)

        changed = False

        # Read image dimensions for visibility check
        ci = data.get("camera_intrinsics", {})
        img_w = ci.get("width", int(2 * ci["cx"]) if "cx" in ci else None)
        img_h = ci.get("height", int(2 * ci["cy"]) if "cy" in ci else None)

        # Apply 2D offsets
        for entry in data.get("skeleton_2d", []):
            jid = entry["joint_id"]
            if jid in offsets_2d:
                du, dv = offsets_2d[jid]
                if abs(du) > 0.001 or abs(dv) > 0.001:
                    conf = entry.get("confidence", 0)
                    if conf > 0:
                        entry["u"] = round(entry["u"] + du, 2)
                        entry["v"] = round(entry["v"] + dv, 2)
                        changed = True
                        # Recalculate visible flag
                        if img_w is not None and img_h is not None:
                            new_vis = bool(conf > 0 and 0 <= entry["u"] < img_w and 0 <= entry["v"] < img_h)
                            if entry.get("visible") != new_vis:
                                entry["visible"] = new_vis

        # Apply 3D offsets
        if offsets_3d:
            for entry in data.get("skeleton_3d", []):
                jid = entry["joint_id"]
                if jid in offsets_3d:
                    dx, dy, dz = offsets_3d[jid]
                    if abs(dx) > 0.001 or abs(dy) > 0.001 or abs(dz) > 0.001:
                        if entry.get("confidence", 0) > 0:
                            entry["x"] = round(entry["x"] + dx, 2)
                            entry["y"] = round(entry["y"] + dy, 2)
                            entry["z"] = round(entry["z"] + dz, 2)
                            changed = True

        if changed:
            if create_backup:
                bak_path = str(json_path) + ".bak"
                if not os.path.exists(bak_path):
                    shutil.copy2(json_path, bak_path)

            tmp = str(json_path) + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
            os.replace(tmp, json_path)
            modified += 1

    return modified


def print_offsets(
    offsets_2d: Dict[int, Tuple[float, float]],
    offsets_3d: Dict[int, Tuple[float, float, float]],
    counts: Dict[int, int],
):
    """Pretty-print the computed offsets."""
    # Joint names from constants
    joint_names = [
        "PELVIS", "SPINE_NAVAL", "SPINE_CHEST", "NECK",
        "CLAVICLE_LEFT", "SHOULDER_LEFT", "ELBOW_LEFT", "WRIST_LEFT",
        "HAND_LEFT", "HANDTIP_LEFT", "THUMB_LEFT",
        "CLAVICLE_RIGHT", "SHOULDER_RIGHT", "ELBOW_RIGHT", "WRIST_RIGHT",
        "HAND_RIGHT", "HANDTIP_RIGHT", "THUMB_RIGHT",
        "HIP_LEFT", "KNEE_LEFT", "ANKLE_LEFT", "FOOT_LEFT",
        "HIP_RIGHT", "KNEE_RIGHT", "ANKLE_RIGHT", "FOOT_RIGHT",
        "HEAD", "NOSE", "EYE_LEFT", "EYE_RIGHT", "EAR_LEFT", "EAR_RIGHT",
    ]

    print(f"\n{'ID':>3} {'Joint':<18} {'N':>4}  {'du':>8} {'dv':>8}  "
          f"{'dx':>9} {'dy':>9} {'dz':>9}")
    print("-" * 90)

    for jid in range(NUM_JOINTS):
        n = counts[jid]
        if n == 0:
            continue
        du, dv = offsets_2d[jid]
        dx, dy, dz = offsets_3d[jid]
        name = joint_names[jid] if jid < len(joint_names) else f"JOINT_{jid}"
        print(f"{jid:>3} {name:<18} {n:>4}  "
              f"{du:>+8.2f} {dv:>+8.2f}  "
              f"{dx:>+9.2f} {dy:>+9.2f} {dz:>+9.2f}")

    # Global average (across all joints that had edits)
    total_n = sum(counts.values())
    if total_n > 0:
        avg_du = sum(offsets_2d[j][0] * counts[j] for j in range(NUM_JOINTS)) / total_n
        avg_dv = sum(offsets_2d[j][1] * counts[j] for j in range(NUM_JOINTS)) / total_n
        avg_dx = sum(offsets_3d[j][0] * counts[j] for j in range(NUM_JOINTS)) / total_n
        avg_dy = sum(offsets_3d[j][1] * counts[j] for j in range(NUM_JOINTS)) / total_n
        avg_dz = sum(offsets_3d[j][2] * counts[j] for j in range(NUM_JOINTS)) / total_n
        print("-" * 90)
        print(f"    {'GLOBAL AVG':<18} {total_n:>4}  "
              f"{avg_du:>+8.2f} {avg_dv:>+8.2f}  "
              f"{avg_dx:>+9.2f} {avg_dy:>+9.2f} {avg_dz:>+9.2f}")
    print()


def main():
    parser = argparse.ArgumentParser(
        description="Compute average annotation offsets from a source dataset "
        "and optionally apply them to a target dataset. "
        "Expects dataset structure: <root>/<participant>/<session>/ego_dataset/annotations/",
    )
    parser.add_argument(
        "source",
        help="Source dataset root (a) -- contains participant/session dirs with .json.bak edits",
    )
    parser.add_argument(
        "--target",
        help="Target dataset root (b) to apply offsets to. "
        "If omitted, offsets are applied back to the source dataset.",
    )
    parser.add_argument(
        "--apply", action="store_true",
        help="Apply the computed offsets (without this flag, dry-run only)",
    )
    parser.add_argument(
        "--3d", dest="apply_3d", action="store_true",
        help="Also apply 3D offsets (default: 2D only)",
    )
    parser.add_argument(
        "--no-backup", action="store_true",
        help="Skip creating .bak backup files on the target",
    )
    parser.add_argument(
        "--flat", action="store_true",
        help="Treat source/target as a single annotations/ dir (no hierarchy)",
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
    # Step 1: Compute average offsets from source (a)
    # -----------------------------------------------------------
    print("Source (a): %s" % source_dir)

    if args.flat:
        # Legacy single-dir mode
        offsets_2d, offsets_3d, counts = compute_average_offsets(source_dir)
    else:
        # Walk dataset hierarchy
        src_ann_dirs = discover_annotations_dirs(source_dir)
        if not src_ann_dirs:
            print("No annotations dirs found under: %s" % source_dir)
            print("Expected: <root>/<participant>/<session>/ego_dataset/annotations/")
            print("Use --flat if pointing directly at a single annotations/ dir.")
            return

        print("Found %d annotation dirs across dataset" % len(src_ann_dirs))
        all_results = []
        sessions_with_edits = 0
        for ann_dir in src_ann_dirs:
            # Check if this session has any .bak files before processing
            bak_files = list(ann_dir.glob("frame_*.json.bak"))
            if not bak_files:
                continue
            # Show relative path: participant/session
            rel = ann_dir.relative_to(Path(source_dir))
            parts = rel.parts  # e.g. ('021903', 'Dancing1_...', 'ego_dataset', 'annotations')
            label = "/".join(parts[:2]) if len(parts) >= 2 else str(rel)
            print("  [%s]" % label)
            result = compute_average_offsets(str(ann_dir))
            has_edits = any(result[2][j] > 0 for j in range(NUM_JOINTS))
            if has_edits:
                all_results.append(result)
                sessions_with_edits += 1

        if not all_results:
            print("No edited frames found (no .bak files with changes).")
            return

        print("\n%d sessions with annotation edits" % sessions_with_edits)
        offsets_2d, offsets_3d, counts = merge_offsets(all_results)

    print_offsets(offsets_2d, offsets_3d, counts)

    total_edited = sum(1 for n in counts.values() if n > 0)
    if total_edited == 0:
        print("No edited frames found (no .bak files with changes).")
        return

    # -----------------------------------------------------------
    # Step 2: Apply offsets to target (b)
    # -----------------------------------------------------------
    if args.apply:
        print("Target (b): %s" % target_dir)
        o3d = offsets_3d if args.apply_3d else None
        mode = "2D + 3D" if args.apply_3d else "2D only"

        if args.flat:
            n = apply_offsets(target_dir, offsets_2d, o3d,
                              create_backup=not args.no_backup)
            print("Applied offsets (%s) to %d files." % (mode, n))
        else:
            tgt_ann_dirs = discover_annotations_dirs(target_dir)
            if not tgt_ann_dirs:
                print("No annotations dirs found under target: %s" % target_dir)
                return
            print("Applying to %d annotation dirs..." % len(tgt_ann_dirs))
            total_modified = 0
            for ann_dir in tgt_ann_dirs:
                n = apply_offsets(str(ann_dir), offsets_2d, o3d,
                                  create_backup=not args.no_backup)
                if n > 0:
                    rel = ann_dir.relative_to(Path(target_dir))
                    parts = rel.parts
                    label = "/".join(parts[:2]) if len(parts) >= 2 else str(rel)
                    print("  [%s] %d files" % (label, n))
                total_modified += n
            print("Applied offsets (%s) to %d files total." % (mode, total_modified))
    else:
        print("Target (b): %s" % target_dir)
        if args.flat:
            n_target = len(list(Path(target_dir).glob("frame_*.json")))
        else:
            tgt_ann_dirs = discover_annotations_dirs(target_dir)
            n_target = sum(
                len(list(d.glob("frame_*.json"))) for d in tgt_ann_dirs
            )
            print("Found %d annotation dirs in target" % len(tgt_ann_dirs))
        print("Dry run -- %d files would be modified. Use --apply to proceed." % n_target)


if __name__ == "__main__":
    main()
