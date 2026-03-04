#!/usr/bin/env python
"""
Run batch_ego_dataset.py on all numeric participant folders in a source directory.

Usage:
    python run_all_participants.py

Edit INPUT_ROOT and OUTPUT_ROOT below to change paths.
"""

import os
import re
import subprocess
import sys
import time
from pathlib import Path

# =============================================================================
# Configuration — edit these paths
# =============================================================================
INPUT_ROOT = r"\\ARVRLAB_SERVER\arvr_lab\[DATASET]\egodataset_2026"
OUTPUT_ROOT = r"\\ARVRLAB_SERVER\arvr_lab\[DATASET]\preprocessing_egodataset_weightmodify"

SCRIPTS_DIR = Path(__file__).parent.resolve()
BATCH_SCRIPT = SCRIPTS_DIR / "batch_ego_dataset.py"

# =============================================================================
# Main
# =============================================================================
def main():
    input_root = Path(INPUT_ROOT)
    output_root = Path(OUTPUT_ROOT)

    if not input_root.is_dir():
        print(f"Error: Input root not found: {input_root}")
        sys.exit(1)

    # Find numeric folders (e.g. 022111, 031502) and sort ascending
    folders = []
    for entry in input_root.iterdir():
        if entry.is_dir() and re.fullmatch(r"\d+", entry.name):
            folders.append(entry)
    folders.sort(key=lambda p: p.name)

    if not folders:
        print(f"No numeric folders found in {input_root}")
        sys.exit(1)

    print("=" * 60)
    print("Run All Participants")
    print("=" * 60)
    print(f"  Input root:  {input_root}")
    print(f"  Output root: {output_root}")
    print(f"  Participants: {len(folders)}")
    for i, f in enumerate(folders):
        print(f"    {i+1:3d}. {f.name}")
    print()

    total_start = time.time()
    results = []

    for i, folder in enumerate(folders):
        participant = folder.name
        output_dir = output_root / participant

        print(f"\n{'=' * 60}")
        print(f"[{i+1}/{len(folders)}] Participant {participant}")
        print(f"  Input:  {folder}")
        print(f"  Output: {output_dir}")
        print(f"{'=' * 60}")

        cmd = [
            sys.executable,
            str(BATCH_SCRIPT),
            "--input-dir", str(folder),
            "--output-dir", str(output_dir),
            "--resume",
            "--ego-fusion-mode", "world",
        ]

        t0 = time.time()
        ret = subprocess.run(cmd)
        elapsed = time.time() - t0

        status = "OK" if ret.returncode == 0 else f"FAILED (exit {ret.returncode})"
        results.append((participant, status, elapsed))
        print(f"\n  -> {participant}: {status} ({elapsed:.0f}s)")

    # Summary
    total_elapsed = time.time() - total_start
    print(f"\n{'=' * 60}")
    print("Summary")
    print(f"{'=' * 60}")
    for participant, status, elapsed in results:
        print(f"  {participant}: {status} ({elapsed:.0f}s)")

    ok = sum(1 for _, s, _ in results if s == "OK")
    fail = len(results) - ok
    print(f"\n  Total: {ok} OK, {fail} failed, {total_elapsed:.0f}s elapsed")


if __name__ == "__main__":
    main()
