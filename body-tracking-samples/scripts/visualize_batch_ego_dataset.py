#!/usr/bin/env python3
"""
Batch Ego Dataset Visualizer with HMD Overlay

Visualizes ego-view datasets produced by batch_ego_dataset.py, with optional
HMD/controller trajectory overlay in the 3D view.

Usage:
    # List all sessions in batch output
    python visualize_batch_ego_dataset.py --batch-dir batch_out/ --list

    # Interactive preview of a single session
    python visualize_batch_ego_dataset.py --batch-dir batch_out/ --session Dancing1_20260214_001511

    # Interactive preview with HMD overlay
    python visualize_batch_ego_dataset.py --batch-dir batch_out/ --session Dancing1_20260214_001511 \
        --hmd-dir Test/

    # Export videos for all sessions
    python visualize_batch_ego_dataset.py --batch-dir batch_out/ --mode video --hmd-dir Test/

    # Export video for one session
    python visualize_batch_ego_dataset.py --batch-dir batch_out/ --session Dancing1_20260214_001511 \
        --mode video --hmd-dir Test/
"""

import argparse
import json
import re
import sys
import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from pathlib import Path
from typing import Dict, List, Optional, Tuple

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

# Import from existing visualizer
sys.path.insert(0, str(Path(__file__).parent))
from visualize_ego_dataset import (
    EgoDataset,
    BONE_CONNECTIONS, EXCLUDED_JOINTS,
    PART_COLORS_RGB, PART_COLORS_BGR,
    get_joint_part, get_bone_part, CONFIDENCE_RADIUS,
    draw_skeleton_2d_cv, draw_skeleton_2d_mpl,
    draw_skeleton_3d, compute_fixed_bounds,
)


# =============================================================================
# HMD Data Loading
# =============================================================================
# HMD CSV pattern: HMD_{ID}_{Pose}_{Date}_{Time}.csv
HMD_PATTERN = re.compile(r"HMD_\w+_(.+)_(\d{8})_(\d{6})\.csv$")


class HMDData:
    """Loads and provides access to HMD/controller trajectory data."""

    def __init__(self, csv_path: str):
        self.path = Path(csv_path)
        self.df = pd.read_csv(self.path)

        # Normalize column names (strip whitespace)
        self.df.columns = [c.strip() for c in self.df.columns]

        # Parse timestamps
        if 'milisecond' in self.df.columns:
            self.timestamps_ms = self.df['milisecond'].values.astype(np.float64)
        elif 'timestamp_ms' in self.df.columns:
            ts_col = self.df['timestamp_ms']
            # Handle HH:MM:SS:mmm format
            if ts_col.dtype == object:
                self.timestamps_ms = self._parse_time_strings(ts_col)
            else:
                self.timestamps_ms = ts_col.values.astype(np.float64)
        else:
            self.timestamps_ms = np.arange(len(self.df), dtype=np.float64) * (1000.0 / 72)

        # Extract positions (meters)
        self.hmd_pos = self.df[['hmd_pos_x', 'hmd_pos_y', 'hmd_pos_z']].values
        self.left_pos = self.df[['left_pos_x', 'left_pos_y', 'left_pos_z']].values
        self.right_pos = self.df[['right_pos_x', 'right_pos_y', 'right_pos_z']].values

        # Extract rotations (quaternion xyzw)
        self.hmd_rot = self.df[['hmd_rot_x', 'hmd_rot_y', 'hmd_rot_z', 'hmd_rot_w']].values
        self.left_rot = self.df[['left_rot_x', 'left_rot_y', 'left_rot_z', 'left_rot_w']].values
        self.right_rot = self.df[['right_rot_x', 'right_rot_y', 'right_rot_z', 'right_rot_w']].values

        print(f"Loaded HMD data: {len(self.df)} frames from {self.path.name}")
        print(f"  Time range: {self.timestamps_ms[0]:.0f} - {self.timestamps_ms[-1]:.0f} ms")

    def _parse_time_strings(self, series) -> np.ndarray:
        """Parse HH:MM:SS:mmm format to epoch-like ms."""
        results = []
        for s in series:
            parts = str(s).split(':')
            if len(parts) == 4:
                h, m, sec, ms = int(parts[0]), int(parts[1]), int(parts[2]), int(parts[3])
                total_ms = ((h * 3600 + m * 60 + sec) * 1000) + ms
            else:
                total_ms = 0
            results.append(total_ms)
        return np.array(results, dtype=np.float64)

    def __len__(self):
        return len(self.df)

    def get_nearest_idx(self, timestamp_usec: int) -> int:
        """Find the nearest HMD frame index for a given ego timestamp (usec)."""
        target_ms = timestamp_usec / 1000.0
        idx = np.argmin(np.abs(self.timestamps_ms - target_ms))
        return int(idx)

    def get_positions_at(self, idx: int) -> Dict[str, np.ndarray]:
        """Get HMD/controller positions at frame index (in meters)."""
        return {
            'hmd': self.hmd_pos[idx],
            'left': self.left_pos[idx],
            'right': self.right_pos[idx],
        }

    def get_trajectory(self, start_idx: int, end_idx: int) -> Dict[str, np.ndarray]:
        """Get position trajectories over a range of frames."""
        s, e = max(0, start_idx), min(len(self.df), end_idx)
        return {
            'hmd': self.hmd_pos[s:e],
            'left': self.left_pos[s:e],
            'right': self.right_pos[s:e],
        }


def find_hmd_csv(session_name: str, hmd_dir: Path) -> Optional[Path]:
    """Find HMD CSV matching a session name in the given directory."""
    # session_name is like "Dancing1_20260214_001511"
    for f in hmd_dir.iterdir():
        if f.suffix.lower() != '.csv':
            continue
        m = HMD_PATTERN.match(f.name)
        if not m:
            continue
        pose, date, time_str = m.groups()
        key = f"{pose}_{date}_{time_str}"
        if key == session_name:
            return f
    return None


# =============================================================================
# 3D Visualization with HMD
# =============================================================================
def draw_skeleton_3d_with_hmd(
    ax,
    joints_3d: Optional[np.ndarray],
    hmd_data: Optional[HMDData],
    hmd_idx: int,
    camera_pose: Optional[Dict],
    min_confidence: int = 1,
    fixed_bounds: Optional[Tuple] = None,
    trail_length: int = 30,
):
    """Draw 3D skeleton + HMD/controller positions in world space.

    The skeleton joints are in helmet camera frame (from ego annotations).
    To show them with HMD data, we transform skeleton joints back to world
    space using the camera pose, then plot everything in world coordinates.
    HMD positions are in meters (Unity), so we convert to mm for consistency.
    """
    ax.clear()

    has_skeleton = joints_3d is not None and len(joints_3d) > 0
    has_hmd = hmd_data is not None and hmd_idx >= 0

    # Draw skeleton in camera frame (mm) if no HMD to merge with
    if has_skeleton and not has_hmd:
        draw_skeleton_3d(ax, joints_3d, min_confidence, fixed_bounds)
        return

    # Draw HMD/controller data
    if has_hmd:
        pos = hmd_data.get_positions_at(hmd_idx)

        # Convert meters to mm for display
        hmd_mm = pos['hmd'] * 1000
        left_mm = pos['left'] * 1000
        right_mm = pos['right'] * 1000

        # Current positions as large markers
        ax.scatter(*hmd_mm, c='red', s=120, marker='D', label='HMD',
                   edgecolors='white', linewidths=0.5, zorder=5)
        ax.scatter(*left_mm, c='dodgerblue', s=80, marker='o', label='L Ctrl',
                   edgecolors='white', linewidths=0.5, zorder=5)
        ax.scatter(*right_mm, c='orange', s=80, marker='o', label='R Ctrl',
                   edgecolors='white', linewidths=0.5, zorder=5)

        # Draw trailing trajectory
        trail_start = max(0, hmd_idx - trail_length)
        trail = hmd_data.get_trajectory(trail_start, hmd_idx + 1)

        for key, color, alpha in [
            ('hmd', 'red', 0.4),
            ('left', 'dodgerblue', 0.3),
            ('right', 'orange', 0.3),
        ]:
            pts = trail[key] * 1000  # to mm
            if len(pts) > 1:
                ax.plot(pts[:, 0], -pts[:, 1], pts[:, 2],
                        color=color, alpha=alpha, linewidth=1.5)

        # Draw lines from HMD to controllers (arm-like connection)
        ax.plot([hmd_mm[0], left_mm[0]], [-hmd_mm[1], -left_mm[1]],
                [hmd_mm[2], left_mm[2]], 'b--', alpha=0.3, linewidth=1)
        ax.plot([hmd_mm[0], right_mm[0]], [-hmd_mm[1], -right_mm[1]],
                [hmd_mm[2], right_mm[2]], color='orange', linestyle='--',
                alpha=0.3, linewidth=1)

        # Also draw skeleton if available (in camera frame)
        if has_skeleton:
            # Draw skeleton joints offset or in camera frame
            xs = joints_3d[:, 0]
            ys = joints_3d[:, 1]
            zs = joints_3d[:, 2]

            for parent, child in BONE_CONNECTIONS:
                if parent >= len(joints_3d) or child >= len(joints_3d):
                    continue
                if joints_3d[parent, 3] < min_confidence or joints_3d[child, 3] < min_confidence:
                    continue
                part = get_bone_part(parent, child)
                color = PART_COLORS_RGB[part]
                ax.plot([xs[parent], xs[child]],
                        [-ys[parent], -ys[child]],
                        [zs[parent], zs[child]],
                        color=color, linewidth=2, alpha=0.6)

            for jid in range(min(len(joints_3d), 32)):
                if jid in EXCLUDED_JOINTS:
                    continue
                if joints_3d[jid, 3] < min_confidence:
                    continue
                part = get_joint_part(jid)
                color = PART_COLORS_RGB[part]
                ax.scatter(xs[jid], -ys[jid], zs[jid],
                           c=[color], s=20, edgecolors='white', linewidths=0.3,
                           alpha=0.6)

        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_zlabel('Z (mm)')

        if fixed_bounds:
            cx, cy, cz, hr = fixed_bounds
            ax.set_xlim(cx - hr, cx + hr)
            ax.set_ylim(cy - hr, cy + hr)
            ax.set_zlim(cz - hr, cz + hr)
        else:
            # Auto-scale around HMD position
            center = hmd_mm
            hr = 1500.0
            ax.set_xlim(center[0] - hr, center[0] + hr)
            ax.set_ylim(-center[1] - hr, -center[1] + hr)
            ax.set_zlim(center[2] - hr, center[2] + hr)

        ax.legend(loc='upper right', fontsize=8)


def draw_hmd_timeseries(ax, hmd_data: HMDData, current_idx: int):
    """Draw HMD position timeseries with current frame indicator."""
    ax.clear()

    t = (hmd_data.timestamps_ms - hmd_data.timestamps_ms[0]) / 1000.0  # seconds

    # Plot Y (height) for all three devices
    ax.plot(t, hmd_data.hmd_pos[:, 1], color='red', alpha=0.7, linewidth=1, label='HMD Y')
    ax.plot(t, hmd_data.left_pos[:, 1], color='dodgerblue', alpha=0.5, linewidth=1, label='L Ctrl Y')
    ax.plot(t, hmd_data.right_pos[:, 1], color='orange', alpha=0.5, linewidth=1, label='R Ctrl Y')

    # Current frame marker
    if 0 <= current_idx < len(t):
        ax.axvline(t[current_idx], color='black', linewidth=1.5, alpha=0.7, linestyle='--')
        ax.scatter(t[current_idx], hmd_data.hmd_pos[current_idx, 1],
                   c='red', s=50, zorder=5, edgecolors='black', linewidths=0.5)

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Height (m)')
    ax.set_title('HMD / Controller Height')
    ax.legend(loc='upper right', fontsize=7)
    ax.grid(True, alpha=0.3)


# =============================================================================
# Session Discovery
# =============================================================================
def discover_batch_sessions(batch_dir: Path) -> List[Dict]:
    """Find all sessions in batch output directory."""
    sessions = []

    # Check batch_summary.json first
    summary_path = batch_dir / 'batch_summary.json'
    if summary_path.exists():
        with open(summary_path) as f:
            summary = json.load(f)
        for s in summary.get('sessions', []):
            ego_dir = batch_dir / s['name'] / 'ego_dataset'
            if ego_dir.exists():
                sessions.append({
                    'name': s['name'],
                    'ego_dir': ego_dir,
                    'status': s.get('status', 'unknown'),
                    'output_csv': batch_dir / s['name'] / 'output.csv',
                })
        return sessions

    # Fallback: scan directories
    for d in sorted(batch_dir.iterdir()):
        if not d.is_dir():
            continue
        ego_dir = d / 'ego_dataset'
        if ego_dir.exists() and (ego_dir / 'metadata.json').exists():
            sessions.append({
                'name': d.name,
                'ego_dir': ego_dir,
                'status': 'success',
                'output_csv': d / 'output.csv',
            })

    return sessions


# =============================================================================
# HMD Bounds Computation
# =============================================================================
def compute_hmd_bounds(hmd_data: HMDData) -> Tuple:
    """Compute 3D axis bounds centered on HMD median position (in mm)."""
    hmd_mm = hmd_data.hmd_pos * 1000
    cx = float(np.median(hmd_mm[:, 0]))
    cy = float(-np.median(hmd_mm[:, 1]))  # invert Y
    cz = float(np.median(hmd_mm[:, 2]))
    hr = 1500.0
    return (cx, cy, cz, hr)


# =============================================================================
# Interactive Preview
# =============================================================================
def preview_session(
    dataset: EgoDataset,
    hmd_data: Optional[HMDData] = None,
    session_name: str = "",
    min_confidence: int = 1,
):
    """Interactive preview of one session with optional HMD overlay."""
    if len(dataset) == 0:
        print("No frames to display.")
        return

    has_hmd = hmd_data is not None

    # Pre-compute bounds
    if has_hmd:
        bounds_3d = compute_hmd_bounds(hmd_data)
    else:
        bounds_3d = compute_fixed_bounds(dataset, min_confidence)

    # Layout: 2D overlay | 3D skeleton+HMD | HMD timeseries (if HMD available)
    if has_hmd:
        fig = plt.figure(figsize=(20, 9))
        ax_2d = fig.add_subplot(131)
        ax_3d = fig.add_subplot(132, projection='3d')
        ax_ts = fig.add_subplot(133)
    else:
        fig = plt.figure(figsize=(16, 8))
        ax_2d = fig.add_subplot(121)
        ax_3d = fig.add_subplot(122, projection='3d')
        ax_ts = None

    fig.suptitle(session_name or 'Ego Dataset', fontsize=12, fontweight='bold')
    plt.subplots_adjust(bottom=0.12, top=0.92, wspace=0.3)

    ax_slider = plt.axes([0.15, 0.02, 0.70, 0.03])
    slider = Slider(ax_slider, 'Frame', 0, max(len(dataset) - 1, 1),
                    valinit=0, valstep=1, valfmt='%d')

    def update(frame_idx):
        frame_idx = int(frame_idx)
        frame = dataset.get_frame(frame_idx)
        joints_2d = dataset.get_joints_2d(frame_idx)
        joints_3d = dataset.get_joints_3d(frame_idx)
        cb = frame.get('checkerboard_detected', False)
        ts_usec = frame.get('timestamp_usec', 0)
        n_bodies = frame.get('num_bodies', 0)
        camera_pose = frame.get('camera_pose', None)

        info = f"Frame {frame_idx} | ts={ts_usec} | bodies={n_bodies} | CB={'Y' if cb else 'N'}"

        # 2D overlay
        ax_2d.clear()
        img = dataset.get_image(frame_idx)
        if img is not None:
            if HAS_CV2 and len(img.shape) == 3 and img.shape[2] == 3:
                img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            else:
                img_rgb = img
            ax_2d.imshow(img_rgb)
            if joints_2d is not None and len(joints_2d) > 0:
                draw_skeleton_2d_mpl(ax_2d, joints_2d, img.shape[:2], min_confidence)
        ax_2d.set_title(f'Ego-View 2D\n{info}')
        ax_2d.axis('off')

        # 3D skeleton + HMD
        hmd_idx = -1
        if has_hmd:
            hmd_idx = hmd_data.get_nearest_idx(ts_usec)

        draw_skeleton_3d_with_hmd(
            ax_3d, joints_3d, hmd_data, hmd_idx, camera_pose,
            min_confidence, bounds_3d,
        )
        ax_3d.set_title('3D Skeleton + HMD' if has_hmd else '3D Skeleton')

        # HMD timeseries
        if ax_ts is not None and has_hmd:
            draw_hmd_timeseries(ax_ts, hmd_data, hmd_idx)

        fig.canvas.draw_idle()

    slider.on_changed(update)
    update(0)

    def on_key(event):
        if event.key == 'right':
            slider.set_val(min(slider.val + 1, len(dataset) - 1))
        elif event.key == 'left':
            slider.set_val(max(slider.val - 1, 0))
        elif event.key == 'pagedown':
            slider.set_val(min(slider.val + 10, len(dataset) - 1))
        elif event.key == 'pageup':
            slider.set_val(max(slider.val - 10, 0))
        elif event.key == 'home':
            slider.set_val(0)
        elif event.key == 'end':
            slider.set_val(len(dataset) - 1)

    fig.canvas.mpl_connect('key_press_event', on_key)
    plt.show()


# =============================================================================
# Video Export
# =============================================================================
def export_session_video(
    dataset: EgoDataset,
    output_path: str,
    hmd_data: Optional[HMDData] = None,
    session_name: str = "",
    fps: float = 30.0,
    min_confidence: int = 1,
):
    """Export session visualization as MP4 video."""
    from matplotlib.animation import FuncAnimation, FFMpegWriter

    has_hmd = hmd_data is not None

    if has_hmd:
        bounds_3d = compute_hmd_bounds(hmd_data)
    else:
        bounds_3d = compute_fixed_bounds(dataset, min_confidence)

    if has_hmd:
        fig = plt.figure(figsize=(20, 7))
        ax_2d = fig.add_subplot(131)
        ax_3d = fig.add_subplot(132, projection='3d')
        ax_ts = fig.add_subplot(133)
    else:
        fig = plt.figure(figsize=(16, 7))
        ax_2d = fig.add_subplot(121)
        ax_3d = fig.add_subplot(122, projection='3d')
        ax_ts = None

    fig.suptitle(session_name, fontsize=11, fontweight='bold')
    plt.subplots_adjust(top=0.92, wspace=0.3)

    def update(frame_idx):
        frame = dataset.get_frame(frame_idx)
        joints_2d = dataset.get_joints_2d(frame_idx)
        joints_3d = dataset.get_joints_3d(frame_idx)
        cb = frame.get('checkerboard_detected', False)
        ts_usec = frame.get('timestamp_usec', 0)
        camera_pose = frame.get('camera_pose', None)
        info = f"Frame {frame_idx} | CB={'Y' if cb else 'N'}"

        ax_2d.clear()
        img = dataset.get_image(frame_idx)
        if img is not None:
            if HAS_CV2:
                img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            else:
                img_rgb = img
            ax_2d.imshow(img_rgb)
            if joints_2d is not None:
                draw_skeleton_2d_mpl(ax_2d, joints_2d, img.shape[:2], min_confidence)
        ax_2d.set_title(f'Ego-View 2D\n{info}')
        ax_2d.axis('off')

        hmd_idx = hmd_data.get_nearest_idx(ts_usec) if has_hmd else -1
        draw_skeleton_3d_with_hmd(
            ax_3d, joints_3d, hmd_data, hmd_idx, camera_pose,
            min_confidence, bounds_3d,
        )
        ax_3d.set_title('3D + HMD' if has_hmd else '3D Skeleton')

        if ax_ts is not None and has_hmd:
            draw_hmd_timeseries(ax_ts, hmd_data, hmd_idx)

        if (frame_idx + 1) % 50 == 0:
            print(f"  Rendering {frame_idx + 1}/{len(dataset)}...")

        return []

    anim = FuncAnimation(fig, update, frames=len(dataset), blit=False)

    try:
        writer = FFMpegWriter(fps=fps, codec='libx264',
                              extra_args=['-pix_fmt', 'yuv420p'])
        anim.save(output_path, writer=writer)
        print(f"  Video saved: {output_path}")
    except Exception as e:
        print(f"  FFmpeg export failed: {e}")
        print("  Install ffmpeg for video export: conda install ffmpeg")

    plt.close(fig)


# =============================================================================
# CLI
# =============================================================================
def main():
    parser = argparse.ArgumentParser(
        description='Visualize batch ego-view datasets with optional HMD overlay',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # List sessions
    python visualize_batch_ego_dataset.py --batch-dir batch_out/ --list

    # Preview one session
    python visualize_batch_ego_dataset.py --batch-dir batch_out/ --session Dancing1_20260214_001511

    # Preview with HMD data
    python visualize_batch_ego_dataset.py --batch-dir batch_out/ --session Dancing1_20260214_001511 \\
        --hmd-dir Test/

    # Export all sessions as video
    python visualize_batch_ego_dataset.py --batch-dir batch_out/ --mode video --hmd-dir Test/

Keyboard shortcuts (preview mode):
    Left/Right      Previous/Next frame
    PgUp/PgDown     Skip 10 frames
    Home/End        First/Last frame
""")

    parser.add_argument('--batch-dir', '-b', required=True,
                        help='Batch output directory (from batch_ego_dataset.py)')
    parser.add_argument('--session', '-s', default=None,
                        help='Session name to visualize (default: all for video, first for preview)')
    parser.add_argument('--hmd-dir', default=None,
                        help='Directory containing HMD CSV files (e.g. Test/)')
    parser.add_argument('--mode', choices=['preview', 'video'], default='preview',
                        help='Visualization mode (default: preview)')
    parser.add_argument('--fps', type=float, default=30.0,
                        help='Video FPS (default: 30)')
    parser.add_argument('--min-confidence', type=int, default=1, choices=[0, 1, 2, 3],
                        help='Minimum joint confidence to display (default: 1)')
    parser.add_argument('--list', action='store_true',
                        help='List all available sessions and exit')

    args = parser.parse_args()

    batch_dir = Path(args.batch_dir).resolve()
    if not batch_dir.is_dir():
        print(f"Error: Batch directory not found: {batch_dir}")
        sys.exit(1)

    hmd_dir = Path(args.hmd_dir).resolve() if args.hmd_dir else None

    # Discover sessions
    sessions = discover_batch_sessions(batch_dir)
    if not sessions:
        print(f"No ego_dataset sessions found in {batch_dir}")
        print("Run batch_ego_dataset.py first to generate datasets.")
        sys.exit(1)

    # List mode
    if args.list:
        print(f"Sessions in {batch_dir} ({len(sessions)} total):\n")
        for i, s in enumerate(sessions):
            ego_dir = s['ego_dir']
            n_frames = len(list(ego_dir.glob('annotations/frame_*.json')))
            hmd_found = ""
            if hmd_dir:
                hmd_csv = find_hmd_csv(s['name'], hmd_dir)
                hmd_found = f" | HMD: {'found' if hmd_csv else 'not found'}"
            print(f"  {i+1:3d}. {s['name']}  ({n_frames} frames, {s['status']}){hmd_found}")
        return

    # Filter to requested session
    if args.session:
        sessions = [s for s in sessions if s['name'] == args.session]
        if not sessions:
            print(f"Session '{args.session}' not found. Use --list to see available sessions.")
            sys.exit(1)

    # Preview mode: show one session interactively
    if args.mode == 'preview':
        s = sessions[0]
        print(f"\nLoading session: {s['name']}")
        dataset = EgoDataset(str(s['ego_dir']))

        hmd_data = None
        if hmd_dir:
            hmd_csv = find_hmd_csv(s['name'], hmd_dir)
            if hmd_csv:
                hmd_data = HMDData(str(hmd_csv))
            else:
                print(f"  Warning: No HMD CSV found for {s['name']} in {hmd_dir}")

        preview_session(
            dataset, hmd_data,
            session_name=s['name'],
            min_confidence=args.min_confidence,
        )

    # Video mode: export video for each session
    elif args.mode == 'video':
        for i, s in enumerate(sessions):
            print(f"\n[{i+1}/{len(sessions)}] {s['name']}")
            dataset = EgoDataset(str(s['ego_dir']))

            if len(dataset) == 0:
                print("  Skipping: no frames")
                continue

            hmd_data = None
            if hmd_dir:
                hmd_csv = find_hmd_csv(s['name'], hmd_dir)
                if hmd_csv:
                    hmd_data = HMDData(str(hmd_csv))

            output_path = str(batch_dir / s['name'] / f"{s['name']}_visualization.mp4")
            export_session_video(
                dataset, output_path, hmd_data,
                session_name=s['name'],
                fps=args.fps,
                min_confidence=args.min_confidence,
            )

        print(f"\nDone. Videos saved in session subdirectories of {batch_dir}")


if __name__ == '__main__':
    main()
