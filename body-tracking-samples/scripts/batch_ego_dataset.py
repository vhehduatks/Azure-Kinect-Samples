#!/usr/bin/env python3
"""
Batch Ego Dataset Generator

Automates running multi_device_offline_processor.exe and sync_skeleton_hmd.py
for all recorded sessions in a directory. Groups MKV files and HMD CSVs by
matching timestamps, then processes each session sequentially.

Usage:
    # Dry run - list all discovered sessions
    python batch_ego_dataset.py --input-dir Test/ --output-dir batch_out/ --dry-run

    # Process all sessions
    python batch_ego_dataset.py --input-dir Test/ --output-dir batch_out/

    # Resume (skip already-completed sessions)
    python batch_ego_dataset.py --input-dir Test/ --output-dir batch_out/ --resume

    # Custom processor path and calibration
    python batch_ego_dataset.py --input-dir Test/ --output-dir batch_out/ \\
        --processor-exe path/to/multi_device_offline_processor.exe \\
        --calib-dir path/to/calibration/
"""

import argparse
import json
import os
import re
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional


# =============================================================================
# Constants
# =============================================================================
HELMET_SERIAL = "CL8T75400GD"
CB_ROWS = 4
CB_COLS = 5
CB_SQUARE = 55.0

# MKV filename pattern: recording_cam{N}_{SERIAL}_{Pose}_{Date}_{Time}.mkv
MKV_PATTERN = re.compile(
    r"recording_cam(\d+)_(\w+)_(.+)_(\d{8})_(\d{6})\.mkv$"
)

# HMD filename pattern: HMD_{ParticipantID}_{Pose}_{Date}_{Time}.csv
# ParticipantID is typically "Test", "P01", etc.
HMD_PATTERN = re.compile(
    r"HMD_\w+_(.+)_(\d{8})_(\d{6})\.csv$"
)


# =============================================================================
# Data Structures
# =============================================================================
@dataclass
class Session:
    """A recording session grouped by matching timestamp."""
    name: str                          # e.g. "Dancing1_20260214_001511"
    pose: str                          # e.g. "Dancing1"
    date: str                          # e.g. "20260214"
    time: str                          # e.g. "001511"
    mkv_by_serial: Dict[str, Path] = field(default_factory=dict)
    hmd_csv: Optional[Path] = None

    @property
    def session_key(self) -> str:
        return f"{self.pose}_{self.date}_{self.time}"


@dataclass
class SessionResult:
    """Result of processing a single session."""
    name: str
    status: str              # "success", "failed", "skipped"
    processor_time: float = 0.0
    sync_time: float = 0.0
    error: str = ""


# =============================================================================
# Session Discovery
# =============================================================================
def discover_sessions(input_dir: Path) -> List[Session]:
    """Parse MKV filenames and group into sessions by matching timestamp."""
    sessions: Dict[str, Session] = {}

    # Scan for MKV files
    for f in sorted(input_dir.iterdir()):
        if f.suffix.lower() != ".mkv":
            continue

        m = MKV_PATTERN.match(f.name)
        if not m:
            print(f"  Warning: MKV file doesn't match pattern: {f.name}")
            continue

        cam_idx, serial, pose, date, timestamp = m.groups()
        key = f"{pose}_{date}_{timestamp}"

        if key not in sessions:
            sessions[key] = Session(
                name=key,
                pose=pose,
                date=date,
                time=timestamp,
            )

        sessions[key].mkv_by_serial[serial] = f

    # Match HMD CSVs to sessions
    for f in sorted(input_dir.iterdir()):
        if f.suffix.lower() != ".csv":
            continue

        m = HMD_PATTERN.match(f.name)
        if not m:
            continue

        pose, date, timestamp = m.groups()
        key = f"{pose}_{date}_{timestamp}"

        if key in sessions:
            sessions[key].hmd_csv = f

    # Sort by name and return
    result = sorted(sessions.values(), key=lambda s: s.name)
    return result


def validate_sessions(sessions: List[Session]) -> List[str]:
    """Validate sessions and return list of warnings."""
    warnings = []
    for s in sessions:
        if len(s.mkv_by_serial) != 4:
            warnings.append(
                f"  {s.name}: expected 4 MKVs, found {len(s.mkv_by_serial)} "
                f"(serials: {', '.join(sorted(s.mkv_by_serial.keys()))})"
            )
        if HELMET_SERIAL not in s.mkv_by_serial:
            warnings.append(
                f"  {s.name}: helmet serial {HELMET_SERIAL} not found in MKVs"
            )
        if s.hmd_csv is None:
            warnings.append(f"  {s.name}: no matching HMD CSV found")
    return warnings


# =============================================================================
# Processing Steps
# =============================================================================
def run_offline_processor(
    session: Session,
    processor_exe: Path,
    calib_dir: Path,
    output_dir: Path,
    sensor_orientation: str,
    smoothing: float,
    ego_fusion_mode: str = "world",
) -> bool:
    """Run multi_device_offline_processor.exe for a session."""
    session_dir = output_dir / session.name
    session_dir.mkdir(parents=True, exist_ok=True)

    output_csv = session_dir / "output.csv"
    ego_output = session_dir / "ego_dataset"
    log_file = session_dir / "processor.log"

    calibration_json = calib_dir / "calibration.json"
    t_checker_to_a = calib_dir / "T_checker_to_A.json"

    # Build MKV list sorted by serial (processor handles remapping via serial)
    mkv_paths = [
        str(session.mkv_by_serial[serial])
        for serial in sorted(session.mkv_by_serial.keys())
    ]

    cmd = [
        str(processor_exe),
        "--calibration", str(calibration_json),
        "--helmet-serial", HELMET_SERIAL,
        "--t-checker-to-a", str(t_checker_to_a),
        "--helmet-cb-rows", str(CB_ROWS),
        "--helmet-cb-cols", str(CB_COLS),
        "--helmet-cb-square", str(CB_SQUARE),
        "--sensor-orientation", sensor_orientation,
        "--smoothing", str(smoothing),
        "--output", str(output_csv),
        "--ego-output", str(ego_output),
        "--ego-fusion-mode", ego_fusion_mode,
    ] + mkv_paths

    print(f"    Command: {' '.join(cmd[:6])} ... ({len(mkv_paths)} MKVs)")

    with open(log_file, "w") as lf:
        result = subprocess.run(
            cmd,
            stdout=lf,
            stderr=subprocess.STDOUT,
            timeout=3600,  # 1 hour timeout per session
        )

    if result.returncode != 0:
        # Read last few lines of log for error context
        try:
            with open(log_file) as lf:
                lines = lf.readlines()
                tail = "".join(lines[-10:])
        except Exception:
            tail = "(could not read log)"
        raise RuntimeError(
            f"Processor exited with code {result.returncode}. "
            f"Log tail:\n{tail}"
        )

    return True


def run_hmd_sync(
    session: Session,
    output_dir: Path,
    scripts_dir: Path,
) -> bool:
    """Run sync_skeleton_hmd.py for a session."""
    session_dir = output_dir / session.name
    skeleton_csv = session_dir / "output.csv"
    synced_csv = session_dir / "synced_data.csv"
    sync_script = scripts_dir / "sync_skeleton_hmd.py"

    if session.hmd_csv is None:
        print(f"    Skipping HMD sync: no HMD CSV for {session.name}")
        return True

    if not skeleton_csv.exists():
        raise RuntimeError(f"Skeleton CSV not found: {skeleton_csv}")

    cmd = [
        sys.executable,
        str(sync_script),
        "--skeleton", str(skeleton_csv),
        "--hmd", str(session.hmd_csv),
        "--output", str(synced_csv),
        "--method", "distance",
    ]

    print(f"    Running HMD sync...")

    result = subprocess.run(
        cmd,
        capture_output=True,
        text=True,
        timeout=300,  # 5 minute timeout
    )

    if result.returncode != 0:
        raise RuntimeError(
            f"Sync script exited with code {result.returncode}. "
            f"stderr: {result.stderr[-500:]}"
        )

    return True


def process_session(
    session: Session,
    processor_exe: Path,
    calib_dir: Path,
    output_dir: Path,
    scripts_dir: Path,
    sensor_orientation: str,
    smoothing: float,
    resume: bool,
    ego_fusion_mode: str = "world",
) -> SessionResult:
    """Orchestrate processing of a single session."""
    result = SessionResult(name=session.name, status="pending")

    metadata_path = output_dir / session.name / "ego_dataset" / "metadata.json"
    synced_path = output_dir / session.name / "synced_data.csv"
    needs_hmd_sync = session.hmd_csv is not None

    # Check resume: skip only if ALL steps are already done
    if resume:
        processor_done = metadata_path.exists()
        sync_done = synced_path.exists() or not needs_hmd_sync
        if processor_done and sync_done:
            print(f"  [{session.name}] Already completed, skipping (--resume)")
            result.status = "skipped"
            return result

    print(f"  [{session.name}] Processing...")
    print(f"    MKVs: {len(session.mkv_by_serial)} cameras")
    print(f"    HMD: {session.hmd_csv.name if session.hmd_csv else 'NONE'}")

    # Step 1: Run offline processor (skip if already done under --resume)
    skip_processor = resume and metadata_path.exists()
    if skip_processor:
        print(f"    Processor already done, skipping (--resume)")
    else:
        try:
            t0 = time.time()
            run_offline_processor(
                session, processor_exe, calib_dir, output_dir,
                sensor_orientation, smoothing, ego_fusion_mode,
            )
            result.processor_time = time.time() - t0
            print(f"    Processor done ({result.processor_time:.1f}s)")
        except Exception as e:
            result.status = "failed"
            result.error = f"Processor: {e}"
            print(f"    FAILED (processor): {e}")
            return result

    # Step 2: Run HMD sync (skip if already done under --resume)
    skip_sync = resume and synced_path.exists()
    if skip_sync:
        print(f"    HMD sync already done, skipping (--resume)")
    else:
        try:
            t0 = time.time()
            run_hmd_sync(session, output_dir, scripts_dir)
            result.sync_time = time.time() - t0
            print(f"    HMD sync done ({result.sync_time:.1f}s)")
        except Exception as e:
            result.status = "failed"
            result.error = f"HMD sync: {e}"
            print(f"    FAILED (HMD sync): {e}")
            return result

    result.status = "success"
    total_time = result.processor_time + result.sync_time
    print(f"    Completed in {total_time:.1f}s")
    return result


# =============================================================================
# Batch Summary
# =============================================================================
def write_batch_summary(
    output_dir: Path,
    results: List[SessionResult],
) -> None:
    """Write batch_summary.json with overall results."""
    succeeded = sum(1 for r in results if r.status == "success")
    failed = sum(1 for r in results if r.status == "failed")
    skipped = sum(1 for r in results if r.status == "skipped")

    summary = {
        "total": len(results),
        "succeeded": succeeded,
        "failed": failed,
        "skipped": skipped,
        "sessions": [],
    }

    for r in results:
        entry = {
            "name": r.name,
            "status": r.status,
            "time_sec": round(r.processor_time + r.sync_time, 1),
        }
        if r.error:
            entry["error"] = r.error
        if r.status == "success":
            entry["processor_time_sec"] = round(r.processor_time, 1)
            entry["sync_time_sec"] = round(r.sync_time, 1)
        summary["sessions"].append(entry)

    summary_path = output_dir / "batch_summary.json"
    with open(summary_path, "w") as f:
        json.dump(summary, f, indent=2)

    print(f"\nBatch summary written to: {summary_path}")


# =============================================================================
# Auto-detect Paths
# =============================================================================
def find_processor_exe() -> Optional[Path]:
    """Try to find multi_device_offline_processor.exe automatically."""
    candidates = [
        Path(r"C:\Program Files\Azure-Kinect-Samples\body-tracking-samples"
             r"\multi_device_offline_processor\build\bin\Release"
             r"\multi_device_offline_processor.exe"),
    ]
    for c in candidates:
        if c.exists():
            return c
    return None


def find_calib_dir(processor_exe: Path) -> Path:
    """Default calibration directory is same as processor exe."""
    return processor_exe.parent


# =============================================================================
# CLI
# =============================================================================
def main():
    parser = argparse.ArgumentParser(
        description="Batch ego dataset generation from multi-camera recordings",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Dry run - discover sessions without processing
    python batch_ego_dataset.py --input-dir Test/ --output-dir batch_out/ --dry-run

    # Process all sessions
    python batch_ego_dataset.py --input-dir Test/ --output-dir batch_out/

    # Resume interrupted batch (skip completed sessions)
    python batch_ego_dataset.py --input-dir Test/ --output-dir batch_out/ --resume
        """,
    )

    parser.add_argument(
        "--input-dir", required=True,
        help="Directory containing MKV recordings and HMD CSVs",
    )
    parser.add_argument(
        "--output-dir", required=True,
        help="Output root directory for all sessions",
    )
    parser.add_argument(
        "--processor-exe", default=None,
        help="Path to multi_device_offline_processor.exe (auto-detected if omitted)",
    )
    parser.add_argument(
        "--calib-dir", default=None,
        help="Directory with calibration.json and T_checker_to_A.json "
             "(defaults to same directory as processor exe)",
    )
    parser.add_argument(
        "--sensor-orientation", default="ccw90",
        choices=["default", "cw90", "ccw90", "flip180"],
        help="Sensor orientation (default: ccw90)",
    )
    parser.add_argument(
        "--smoothing", type=float, default=0.5,
        help="Temporal smoothing factor 0.0-1.0 (default: 0.5)",
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="List discovered sessions without processing",
    )
    parser.add_argument(
        "--resume", action="store_true",
        help="Skip sessions that already have ego_dataset/metadata.json",
    )
    parser.add_argument(
        "--ego-fusion-mode", default="world",
        choices=["world", "local"],
        help="Ego skeleton fusion mode: world (default) or local",
    )

    args = parser.parse_args()

    input_dir = Path(args.input_dir).resolve()
    output_dir = Path(args.output_dir).resolve()
    scripts_dir = Path(__file__).parent.resolve()

    # Validate input directory
    if not input_dir.is_dir():
        print(f"Error: Input directory not found: {input_dir}")
        sys.exit(1)

    # Resolve processor exe
    if args.processor_exe:
        processor_exe = Path(args.processor_exe).resolve()
    else:
        processor_exe = find_processor_exe()
        if processor_exe is None:
            print("Error: Could not auto-detect multi_device_offline_processor.exe")
            print("       Use --processor-exe to specify the path.")
            sys.exit(1)

    if not processor_exe.exists():
        print(f"Error: Processor exe not found: {processor_exe}")
        sys.exit(1)

    # Resolve calibration directory
    if args.calib_dir:
        calib_dir = Path(args.calib_dir).resolve()
    else:
        calib_dir = find_calib_dir(processor_exe)

    # Validate calibration files
    calib_json = calib_dir / "calibration.json"
    t_checker = calib_dir / "T_checker_to_A.json"
    if not calib_json.exists():
        print(f"Error: calibration.json not found: {calib_json}")
        sys.exit(1)
    if not t_checker.exists():
        print(f"Error: T_checker_to_A.json not found: {t_checker}")
        sys.exit(1)

    # Print configuration
    print("=" * 60)
    print("Batch Ego Dataset Generator")
    print("=" * 60)
    print(f"  Input:        {input_dir}")
    print(f"  Output:       {output_dir}")
    print(f"  Processor:    {processor_exe}")
    print(f"  Calibration:  {calib_dir}")
    print(f"  Orientation:  {args.sensor_orientation}")
    print(f"  Smoothing:    {args.smoothing}")
    print(f"  Ego fusion:   {args.ego_fusion_mode}")
    print(f"  Resume:       {args.resume}")
    print()

    # Discover sessions
    print("Discovering sessions...")
    sessions = discover_sessions(input_dir)

    if not sessions:
        print("No sessions found! Check that the input directory contains")
        print("MKV files matching: recording_cam{N}_{SERIAL}_{Pose}_{Date}_{Time}.mkv")
        sys.exit(1)

    # Validate
    warnings = validate_sessions(sessions)

    # Print session list
    print(f"\nFound {len(sessions)} sessions:")
    for i, s in enumerate(sessions):
        n_mkv = len(s.mkv_by_serial)
        has_helmet = HELMET_SERIAL in s.mkv_by_serial
        has_hmd = s.hmd_csv is not None
        status_flags = []
        if not has_helmet:
            status_flags.append("NO HELMET")
        if not has_hmd:
            status_flags.append("NO HMD")
        if n_mkv != 4:
            status_flags.append(f"{n_mkv} cams")
        flag_str = f" [{', '.join(status_flags)}]" if status_flags else ""
        print(f"  {i+1:3d}. {s.name}{flag_str}")

    if warnings:
        print(f"\nWarnings ({len(warnings)}):")
        for w in warnings:
            print(w)

    if args.dry_run:
        print("\n(Dry run - no processing performed)")
        return

    # Process sessions
    print(f"\n{'=' * 60}")
    print("Processing sessions...")
    print(f"{'=' * 60}\n")

    output_dir.mkdir(parents=True, exist_ok=True)

    results: List[SessionResult] = []
    batch_start = time.time()

    for i, session in enumerate(sessions):
        print(f"\n[{i+1}/{len(sessions)}] {session.name}")

        # Skip sessions with validation issues
        if HELMET_SERIAL not in session.mkv_by_serial:
            print(f"  Skipping: helmet serial {HELMET_SERIAL} not found")
            results.append(SessionResult(
                name=session.name,
                status="failed",
                error=f"Helmet serial {HELMET_SERIAL} not in MKVs",
            ))
            continue

        if len(session.mkv_by_serial) < 2:
            print(f"  Skipping: only {len(session.mkv_by_serial)} camera(s)")
            results.append(SessionResult(
                name=session.name,
                status="failed",
                error=f"Only {len(session.mkv_by_serial)} camera(s), need at least 2",
            ))
            continue

        result = process_session(
            session=session,
            processor_exe=processor_exe,
            calib_dir=calib_dir,
            output_dir=output_dir,
            scripts_dir=scripts_dir,
            sensor_orientation=args.sensor_orientation,
            smoothing=args.smoothing,
            resume=args.resume,
            ego_fusion_mode=args.ego_fusion_mode,
        )
        results.append(result)

    # Write summary
    batch_time = time.time() - batch_start
    write_batch_summary(output_dir, results)

    # Print final report
    succeeded = sum(1 for r in results if r.status == "success")
    failed = sum(1 for r in results if r.status == "failed")
    skipped = sum(1 for r in results if r.status == "skipped")

    print(f"\n{'=' * 60}")
    print("BATCH COMPLETE")
    print(f"{'=' * 60}")
    print(f"  Total:     {len(results)}")
    print(f"  Succeeded: {succeeded}")
    print(f"  Failed:    {failed}")
    print(f"  Skipped:   {skipped}")
    print(f"  Time:      {batch_time:.1f}s ({batch_time/60:.1f}min)")

    if failed > 0:
        print(f"\nFailed sessions:")
        for r in results:
            if r.status == "failed":
                print(f"  {r.name}: {r.error}")

    print(f"\nOutput: {output_dir}")


if __name__ == "__main__":
    main()
