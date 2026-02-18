#!/usr/bin/env python3
"""
Batch Ego Dataset Visualizer with HMD Overlay

Visualizes ego-view datasets produced by batch_ego_dataset.py, with optional
HMD/controller trajectory overlay.

Layout (with HMD):
    +-------------------+-------------------+
    | Ego-View 2D       | 3D Skeleton       |
    | (helmet camera)   | (camera frame)    |
    +-------------------+-------------------+
    | HMD 3D Trajectory | HMD Timeseries    |
    | (Unity world)     | (height + speed)  |
    +-------------------+-------------------+

Layout (without HMD):
    +-------------------+-------------------+
    | Ego-View 2D       | 3D Skeleton       |
    +-------------------+-------------------+

HMD data is auto-loaded from synced_data.csv in each session directory (produced by
batch_ego_dataset.py). Use --hmd-dir only as a fallback to load raw HMD CSVs.

Usage:
    # List all sessions in batch output
    python visualize_batch_ego_dataset.py --batch-dir batch_out/ --list

    # Interactive preview (auto-loads HMD from synced_data.csv)
    python visualize_batch_ego_dataset.py --batch-dir batch_out/ --session Dancing1_20260214_001511

    # Export videos for all sessions
    python visualize_batch_ego_dataset.py --batch-dir batch_out/ --mode video

    # Fallback: use raw HMD CSVs from a separate directory
    python visualize_batch_ego_dataset.py --batch-dir batch_out/ --hmd-dir Test/
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

        # Strip whitespace from column names (some CSVs have " timestamp_ms")
        self.df.columns = [c.strip() for c in self.df.columns]

        # Parse timestamps (epoch ms)
        if 'milisecond' in self.df.columns:
            self.timestamps_ms = self.df['milisecond'].values.astype(np.float64)
        elif 'timestamp_ms' in self.df.columns:
            ts_col = self.df['timestamp_ms']
            if ts_col.dtype == object:
                self.timestamps_ms = self._parse_time_strings(ts_col)
            else:
                self.timestamps_ms = ts_col.values.astype(np.float64)
        elif 'frame' in self.df.columns:
            self.timestamps_ms = self.df['frame'].values.astype(np.float64) * (1000.0 / 72)
        else:
            self.timestamps_ms = np.arange(len(self.df), dtype=np.float64) * (1000.0 / 72)

        # Extract positions (meters, Unity Y-up coordinate system)
        self.hmd_pos = self.df[['hmd_pos_x', 'hmd_pos_y', 'hmd_pos_z']].values.astype(np.float64)
        self.left_pos = self.df[['left_pos_x', 'left_pos_y', 'left_pos_z']].values.astype(np.float64)
        self.right_pos = self.df[['right_pos_x', 'right_pos_y', 'right_pos_z']].values.astype(np.float64)

        # Extract rotations (quaternion xyzw)
        self.hmd_rot = self.df[['hmd_rot_x', 'hmd_rot_y', 'hmd_rot_z', 'hmd_rot_w']].values.astype(np.float64)
        self.left_rot = self.df[['left_rot_x', 'left_rot_y', 'left_rot_z', 'left_rot_w']].values.astype(np.float64)
        self.right_rot = self.df[['right_rot_x', 'right_rot_y', 'right_rot_z', 'right_rot_w']].values.astype(np.float64)

        # Pre-compute speed (m/s) for timeseries
        dt = np.diff(self.timestamps_ms) / 1000.0  # seconds
        dp = np.linalg.norm(np.diff(self.hmd_pos, axis=0), axis=1)
        self.hmd_speed = np.zeros(len(self.df))
        valid_dt = dt > 1e-6
        self.hmd_speed[1:][valid_dt] = dp[valid_dt] / dt[valid_dt]

        print(f"Loaded HMD data: {len(self.df)} frames from {self.path.name}")
        dur_s = (self.timestamps_ms[-1] - self.timestamps_ms[0]) / 1000.0
        print(f"  Duration: {dur_s:.1f}s, {len(self.df)} frames")

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

    def get_idx_by_ratio(self, frame_idx: int, total_frames: int) -> int:
        """Map ego frame position to HMD frame position linearly.

        Both datasets cover the same recording, so ratio mapping is correct
        even when timestamp domains differ (camera-relative usec vs Unix epoch ms).
        """
        if total_frames <= 1:
            return 0
        ratio = frame_idx / (total_frames - 1)
        return int(round(ratio * (len(self) - 1)))

    def get_positions_at(self, idx: int) -> Dict[str, np.ndarray]:
        """Get HMD/controller positions at frame index (in meters)."""
        idx = max(0, min(idx, len(self.df) - 1))
        return {
            'hmd': self.hmd_pos[idx],
            'left': self.left_pos[idx],
            'right': self.right_pos[idx],
        }

    def get_trajectory(self, start_idx: int, end_idx: int) -> Dict[str, np.ndarray]:
        """Get position trajectories over a range of frames (meters)."""
        s = max(0, start_idx)
        e = min(len(self.df), end_idx)
        return {
            'hmd': self.hmd_pos[s:e],
            'left': self.left_pos[s:e],
            'right': self.right_pos[s:e],
        }

    def get_rotations_at(self, idx: int) -> Dict[str, np.ndarray]:
        """Get HMD/controller rotations (quaternion xyzw) at frame index."""
        idx = max(0, min(idx, len(self.df) - 1))
        return {
            'hmd': self.hmd_rot[idx],
            'left': self.left_rot[idx],
            'right': self.right_rot[idx],
        }


def find_hmd_csv(session_name: str, hmd_dir: Path) -> Optional[Path]:
    """Find HMD CSV matching a session name in the given directory."""
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
# HMD 3D Trajectory Visualization
# =============================================================================
def draw_hmd_3d(
    ax,
    hmd_data: HMDData,
    hmd_idx: int,
    fixed_bounds: Optional[Tuple] = None,
    trail_length: int = 60,
):
    """Draw HMD and controller positions/trajectories in Unity world space.

    Coordinate system (Unity): X=right, Y=up, Z=forward. Units: meters.
    """
    ax.clear()

    pos = hmd_data.get_positions_at(hmd_idx)
    hmd = pos['hmd']
    left = pos['left']
    right = pos['right']

    # Current positions as markers
    ax.scatter(hmd[0], hmd[1], hmd[2],
               c='red', s=120, marker='D', label='HMD',
               edgecolors='white', linewidths=0.5, zorder=5)
    ax.scatter(left[0], left[1], left[2],
               c='dodgerblue', s=80, marker='o', label='L Ctrl',
               edgecolors='white', linewidths=0.5, zorder=5)
    ax.scatter(right[0], right[1], right[2],
               c='orange', s=80, marker='o', label='R Ctrl',
               edgecolors='white', linewidths=0.5, zorder=5)

    # Lines from HMD to controllers
    ax.plot([hmd[0], left[0]], [hmd[1], left[1]], [hmd[2], left[2]],
            'b--', alpha=0.4, linewidth=1)
    ax.plot([hmd[0], right[0]], [hmd[1], right[1]], [hmd[2], right[2]],
            color='orange', linestyle='--', alpha=0.4, linewidth=1)

    # Trailing trajectories
    trail_start = max(0, hmd_idx - trail_length)
    trail = hmd_data.get_trajectory(trail_start, hmd_idx + 1)

    for key, color, alpha in [
        ('hmd', 'red', 0.5),
        ('left', 'dodgerblue', 0.3),
        ('right', 'orange', 0.3),
    ]:
        pts = trail[key]
        if len(pts) > 1:
            ax.plot(pts[:, 0], pts[:, 1], pts[:, 2],
                    color=color, alpha=alpha, linewidth=1.5)

    # HMD forward direction indicator (from quaternion)
    rot = hmd_data.get_rotations_at(hmd_idx)['hmd']
    qx, qy, qz, qw = rot
    # Quaternion to forward vector: rotate (0,0,1) by quaternion
    fx = 2 * (qx * qz + qw * qy)
    fy = 2 * (qy * qz - qw * qx)
    fz = 1 - 2 * (qx * qx + qy * qy)
    arrow_len = 0.15  # meters
    ax.plot([hmd[0], hmd[0] + fx * arrow_len],
            [hmd[1], hmd[1] + fy * arrow_len],
            [hmd[2], hmd[2] + fz * arrow_len],
            color='red', linewidth=2.5, alpha=0.8)

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('HMD 3D Trajectory')

    if fixed_bounds:
        cx, cy, cz, hr = fixed_bounds
        ax.set_xlim(cx - hr, cx + hr)
        ax.set_ylim(cy - hr, cy + hr)
        ax.set_zlim(cz - hr, cz + hr)
    else:
        # Auto-scale around current HMD position
        hr = 0.8  # meters
        ax.set_xlim(hmd[0] - hr, hmd[0] + hr)
        ax.set_ylim(hmd[1] - hr, hmd[1] + hr)
        ax.set_zlim(hmd[2] - hr, hmd[2] + hr)

    ax.legend(loc='upper right', fontsize=7)


def draw_hmd_timeseries(ax, hmd_data: HMDData, current_idx: int):
    """Draw HMD/controller height and HMD speed over time."""
    ax.clear()

    t = (hmd_data.timestamps_ms - hmd_data.timestamps_ms[0]) / 1000.0  # seconds

    # Height (Y position) for all three devices
    ax.plot(t, hmd_data.hmd_pos[:, 1], color='red', alpha=0.8,
            linewidth=1.2, label='HMD height')
    ax.plot(t, hmd_data.left_pos[:, 1], color='dodgerblue', alpha=0.5,
            linewidth=1, label='L Ctrl height')
    ax.plot(t, hmd_data.right_pos[:, 1], color='orange', alpha=0.5,
            linewidth=1, label='R Ctrl height')

    # Speed on secondary axis
    ax2 = ax.twinx()
    ax2.plot(t, hmd_data.hmd_speed, color='gray', alpha=0.3,
             linewidth=0.8, label='HMD speed')
    ax2.set_ylabel('Speed (m/s)', color='gray', fontsize=8)
    ax2.tick_params(axis='y', labelcolor='gray', labelsize=7)
    ax2.set_ylim(0, max(np.percentile(hmd_data.hmd_speed, 99) * 1.3, 0.5))

    # Current frame marker
    if 0 <= current_idx < len(t):
        ax.axvline(t[current_idx], color='black', linewidth=1.5,
                   alpha=0.6, linestyle='--')
        ax.scatter(t[current_idx], hmd_data.hmd_pos[current_idx, 1],
                   c='red', s=40, zorder=5, edgecolors='black', linewidths=0.5)

    ax.set_xlabel('Time (s)', fontsize=9)
    ax.set_ylabel('Height (m)', fontsize=9)
    ax.set_title('HMD / Controller Timeseries')
    ax.legend(loc='upper left', fontsize=7)
    ax.grid(True, alpha=0.3)


# =============================================================================
# Session Discovery
# =============================================================================
def discover_batch_sessions(batch_dir: Path) -> List[Dict]:
    """Find all sessions in batch output directory.

    Each session dict contains:
        name, ego_dir, status, output_csv, synced_csv (Path or None)
    """
    sessions = []

    def _build_session(name: str, status: str = 'unknown') -> Optional[Dict]:
        session_dir = batch_dir / name
        ego_dir = session_dir / 'ego_dataset'
        if not ego_dir.exists():
            return None
        synced_csv = session_dir / 'synced_data.csv'
        return {
            'name': name,
            'ego_dir': ego_dir,
            'status': status,
            'output_csv': session_dir / 'output.csv',
            'synced_csv': synced_csv if synced_csv.exists() else None,
        }

    # Check batch_summary.json first
    summary_path = batch_dir / 'batch_summary.json'
    if summary_path.exists():
        with open(summary_path) as f:
            summary = json.load(f)
        for s in summary.get('sessions', []):
            entry = _build_session(s['name'], s.get('status', 'unknown'))
            if entry:
                sessions.append(entry)
        return sessions

    # Fallback: scan directories
    for d in sorted(batch_dir.iterdir()):
        if not d.is_dir():
            continue
        if not (d / 'ego_dataset' / 'metadata.json').exists():
            continue
        entry = _build_session(d.name, 'success')
        if entry:
            sessions.append(entry)

    return sessions


# =============================================================================
# Bounds Computation
# =============================================================================
def compute_hmd_bounds(hmd_data: HMDData) -> Tuple:
    """Compute stable 3D axis bounds from HMD trajectory (meters)."""
    cx = float(np.median(hmd_data.hmd_pos[:, 0]))
    cy = float(np.median(hmd_data.hmd_pos[:, 1]))
    cz = float(np.median(hmd_data.hmd_pos[:, 2]))

    # Range covers full trajectory + margin
    all_pos = np.vstack([hmd_data.hmd_pos, hmd_data.left_pos, hmd_data.right_pos])
    span = max(
        np.ptp(all_pos[:, 0]),
        np.ptp(all_pos[:, 1]),
        np.ptp(all_pos[:, 2]),
    )
    hr = max(span / 2 * 1.2, 0.5)  # at least 0.5m

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

    # Pre-compute stable bounds
    skel_bounds = compute_fixed_bounds(dataset, min_confidence)
    hmd_bounds = compute_hmd_bounds(hmd_data) if has_hmd else None

    # Layout: 2x2 with HMD, 1x2 without
    if has_hmd:
        fig = plt.figure(figsize=(18, 12))
        ax_2d = fig.add_subplot(221)
        ax_3d = fig.add_subplot(222, projection='3d')
        ax_hmd3d = fig.add_subplot(223, projection='3d')
        ax_ts = fig.add_subplot(224)
    else:
        fig = plt.figure(figsize=(16, 8))
        ax_2d = fig.add_subplot(121)
        ax_3d = fig.add_subplot(122, projection='3d')
        ax_hmd3d = None
        ax_ts = None

    fig.suptitle(session_name or 'Ego Dataset', fontsize=13, fontweight='bold')
    plt.subplots_adjust(bottom=0.10, top=0.93, hspace=0.30, wspace=0.25)

    ax_slider = plt.axes([0.15, 0.02, 0.70, 0.025])
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

        info = f"Frame {frame_idx}/{len(dataset)-1} | bodies={n_bodies} | CB={'Y' if cb else 'N'}"

        # Panel 1: Ego-view 2D overlay
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

        # Panel 2: 3D skeleton (helmet camera frame, mm)
        if joints_3d is not None and len(joints_3d) > 0:
            draw_skeleton_3d(ax_3d, joints_3d, min_confidence, fixed_bounds=skel_bounds)
            ax_3d.set_title('3D Skeleton (Camera Frame)')
        else:
            ax_3d.clear()
            ax_3d.set_title('3D Skeleton - No Data')

        # Panels 3 & 4: HMD
        if has_hmd:
            hmd_idx = hmd_data.get_idx_by_ratio(frame_idx, len(dataset))

            draw_hmd_3d(ax_hmd3d, hmd_data, hmd_idx,
                        fixed_bounds=hmd_bounds)
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
# OpenCV-based Video Export Rendering
# =============================================================================
def project_3d_ortho(points, center, half_range, w, h, az=-50, el=25):
    """Project 3D points to 2D pixels via orthographic projection with rotation.

    Args:
        points: Nx3 array of 3D points
        center: (cx, cy, cz) centre of view volume
        half_range: half the side length of the view cube
        w, h: output pixel dimensions
        az, el: azimuth / elevation in degrees

    Returns:
        (xs, ys) int32 pixel coordinate arrays
    """
    pts = np.asarray(points, dtype=np.float64) - np.array(center, dtype=np.float64)

    ca, sa = np.cos(np.radians(az)), np.sin(np.radians(az))
    ce, se = np.cos(np.radians(el)), np.sin(np.radians(el))

    # Ry then Rx
    rx = ca * pts[:, 0] + sa * pts[:, 2]
    ry = pts[:, 1]
    rz = -sa * pts[:, 0] + ca * pts[:, 2]
    fx = rx
    fy = ce * ry - se * rz

    scale = min(w, h) / (2.0 * half_range) * 0.85
    xs = (fx * scale + w / 2.0).astype(np.int32)
    ys = (-fy * scale + h / 2.0).astype(np.int32)
    return xs, ys


def render_panel_2d(dataset, idx, w, h, min_conf):
    """Render ego-view 2D panel with skeleton overlay (OpenCV)."""
    img = dataset.get_image(idx)
    if img is None:
        canvas = np.zeros((h, w, 3), dtype=np.uint8)
        cv2.putText(canvas, "No Image", (w // 4, h // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (128, 128, 128), 2)
        return canvas

    orig_h, orig_w = img.shape[:2]
    canvas = cv2.resize(img, (w, h))

    joints_2d = dataset.get_joints_2d(idx)
    if joints_2d is not None and len(joints_2d) > 0:
        scaled = joints_2d.copy()
        scaled[:, 0] *= w / orig_w
        scaled[:, 1] *= h / orig_h
        canvas = draw_skeleton_2d_cv(canvas, scaled, min_conf)

    frame = dataset.get_frame(idx)
    n_bodies = frame.get('num_bodies', 0)
    cb = frame.get('checkerboard_detected', False)
    info = f"Frame {idx}/{len(dataset)-1} | bodies={n_bodies} | CB={'Y' if cb else 'N'}"
    cv2.putText(canvas, info, (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(canvas, info, (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return canvas


def render_panel_skeleton_3d(joints_3d, w, h, min_conf, bounds):
    """Render 3D skeleton panel using OpenCV orthographic projection."""
    canvas = np.zeros((h, w, 3), dtype=np.uint8)
    canvas[:] = (30, 30, 30)

    if joints_3d is None or len(joints_3d) == 0:
        cv2.putText(canvas, "No Skeleton Data", (w // 4, h // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (128, 128, 128), 2)
        return canvas

    # Y-inverted to match draw_skeleton_3d convention
    points = np.column_stack([joints_3d[:, 0], -joints_3d[:, 1], joints_3d[:, 2]])
    cx, cy, cz, hr = bounds if bounds else (0.0, 0.0, 1000.0, 500.0)
    xs, ys = project_3d_ortho(points, (cx, cy, cz), hr, w, h)

    for parent, child in BONE_CONNECTIONS:
        if parent >= len(joints_3d) or child >= len(joints_3d):
            continue
        if parent in EXCLUDED_JOINTS or child in EXCLUDED_JOINTS:
            continue
        if joints_3d[parent, 3] < min_conf or joints_3d[child, 3] < min_conf:
            continue
        color = PART_COLORS_BGR[get_bone_part(parent, child)]
        cv2.line(canvas, (int(xs[parent]), int(ys[parent])),
                 (int(xs[child]), int(ys[child])), color, 2, cv2.LINE_AA)

    for jid in range(min(len(joints_3d), 32)):
        if jid in EXCLUDED_JOINTS:
            continue
        conf = int(joints_3d[jid, 3])
        if conf < min_conf:
            continue
        color = PART_COLORS_BGR[get_joint_part(jid)]
        r = CONFIDENCE_RADIUS.get(conf, 3) + 1
        cv2.circle(canvas, (int(xs[jid]), int(ys[jid])), r, color, -1, cv2.LINE_AA)
        cv2.circle(canvas, (int(xs[jid]), int(ys[jid])), r, (255, 255, 255), 1, cv2.LINE_AA)

    cv2.putText(canvas, "3D Skeleton", (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (200, 200, 200), 1, cv2.LINE_AA)
    return canvas


# BGR colours matching the matplotlib HMD palette
_CLR_HMD = (0, 0, 255)        # red
_CLR_LEFT = (255, 144, 30)    # dodgerblue
_CLR_RIGHT = (0, 165, 255)    # orange
_CLR_GRAY = (128, 128, 128)


def render_panel_hmd_3d(hmd_data, idx, w, h, bounds, trail=60):
    """Render HMD / controller 3D trajectory panel (OpenCV)."""
    canvas = np.zeros((h, w, 3), dtype=np.uint8)
    canvas[:] = (30, 30, 30)
    cx, cy, cz, hr = bounds

    pos = hmd_data.get_positions_at(idx)
    hmd_pt, left_pt, right_pt = pos['hmd'], pos['left'], pos['right']

    # Trailing polylines with fade
    trail_start = max(0, idx - trail)
    trail_data = hmd_data.get_trajectory(trail_start, idx + 1)
    for key, color in [('hmd', _CLR_HMD), ('left', _CLR_LEFT), ('right', _CLR_RIGHT)]:
        pts = trail_data[key]
        if len(pts) > 1:
            txs, tys = project_3d_ortho(pts, (cx, cy, cz), hr, w, h)
            n = len(pts)
            seg_len = max(1, n // 4)
            for ss in range(0, n - 1, seg_len):
                se = min(ss + seg_len + 1, n)
                alpha = 0.3 + 0.7 * (ss / n)
                c = tuple(int(v * alpha) for v in color)
                seg = np.column_stack([txs[ss:se], tys[ss:se]]).reshape(-1, 1, 2)
                if len(seg) > 1:
                    cv2.polylines(canvas, [seg], False, c, 2, cv2.LINE_AA)

    # Forward direction arrow from quaternion
    rot = hmd_data.get_rotations_at(idx)['hmd']
    qx, qy, qz, qw = rot
    fwd = np.array([2*(qx*qz + qw*qy), 2*(qy*qz - qw*qx), 1 - 2*(qx*qx + qy*qy)])
    arrow_end = hmd_pt + fwd * 0.15
    axs, ays = project_3d_ortho(np.array([hmd_pt, arrow_end]), (cx, cy, cz), hr, w, h)
    cv2.arrowedLine(canvas, (int(axs[0]), int(ays[0])), (int(axs[1]), int(ays[1])),
                    _CLR_HMD, 2, cv2.LINE_AA, tipLength=0.3)

    # Lines + markers for current positions
    cur = np.array([hmd_pt, left_pt, right_pt])
    cxs, cys = project_3d_ortho(cur, (cx, cy, cz), hr, w, h)
    cv2.line(canvas, (int(cxs[0]), int(cys[0])), (int(cxs[1]), int(cys[1])),
             _CLR_LEFT, 1, cv2.LINE_AA)
    cv2.line(canvas, (int(cxs[0]), int(cys[0])), (int(cxs[2]), int(cys[2])),
             _CLR_RIGHT, 1, cv2.LINE_AA)
    for i, (color, rad) in enumerate([(_CLR_HMD, 7), (_CLR_LEFT, 5), (_CLR_RIGHT, 5)]):
        cv2.circle(canvas, (int(cxs[i]), int(cys[i])), rad, color, -1, cv2.LINE_AA)
        cv2.circle(canvas, (int(cxs[i]), int(cys[i])), rad, (255, 255, 255), 1, cv2.LINE_AA)

    # Title + legend
    cv2.putText(canvas, "HMD 3D Trajectory", (8, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
    ly = h - 50
    for label, color in [("HMD", _CLR_HMD), ("L Ctrl", _CLR_LEFT), ("R Ctrl", _CLR_RIGHT)]:
        cv2.circle(canvas, (w - 100, ly), 4, color, -1)
        cv2.putText(canvas, label, (w - 90, ly + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)
        ly += 15
    return canvas


def build_timeseries_cache(hmd_data, w, h):
    """Pre-render static timeseries background (traces, axes, grid).

    Returns dict with 'image' (BGR ndarray) and coordinate-mapping metadata.
    """
    canvas = np.zeros((h, w, 3), dtype=np.uint8)
    canvas[:] = (40, 40, 40)

    ml, mr, mt, mb = 55, 55, 30, 35
    pw, ph = w - ml - mr, h - mt - mb

    t = (hmd_data.timestamps_ms - hmd_data.timestamps_ms[0]) / 1000.0
    t_max = float(t[-1]) if len(t) > 0 else 1.0

    all_h = np.concatenate([hmd_data.hmd_pos[:, 1],
                            hmd_data.left_pos[:, 1],
                            hmd_data.right_pos[:, 1]])
    h_min = float(np.percentile(all_h, 1)) - 0.05
    h_max = float(np.percentile(all_h, 99)) + 0.05
    h_rng = max(h_max - h_min, 0.1)
    s_max = max(float(np.percentile(hmd_data.hmd_speed, 99)) * 1.3, 0.5)

    # Grid + border
    for i in range(5):
        yg = mt + int(ph * i / 4)
        cv2.line(canvas, (ml, yg), (ml + pw, yg), (60, 60, 60), 1)
        xg = ml + int(pw * i / 4)
        cv2.line(canvas, (xg, mt), (xg, mt + ph), (60, 60, 60), 1)
    cv2.rectangle(canvas, (ml, mt), (ml + pw, mt + ph), (100, 100, 100), 1)

    # Subsample for performance
    step = max(1, len(t) // (pw * 2))
    idx_r = np.arange(0, len(t), step)
    t_sub = t[idx_r]
    x_px = (ml + t_sub / t_max * pw).astype(np.int32) if t_max > 0 else np.full(len(t_sub), ml, dtype=np.int32)

    # Height traces
    for data, color in [
        (hmd_data.hmd_pos[idx_r, 1], _CLR_HMD),
        (hmd_data.left_pos[idx_r, 1], _CLR_LEFT),
        (hmd_data.right_pos[idx_r, 1], _CLR_RIGHT),
    ]:
        y_px = (mt + ph - (data - h_min) / h_rng * ph).astype(np.int32)
        pts = np.column_stack([x_px, y_px]).reshape(-1, 1, 2)
        cv2.polylines(canvas, [pts], False, color, 1, cv2.LINE_AA)

    # Speed trace
    y_spd = (mt + ph - hmd_data.hmd_speed[idx_r] / s_max * ph).astype(np.int32)
    cv2.polylines(canvas, [np.column_stack([x_px, y_spd]).reshape(-1, 1, 2)],
                  False, _CLR_GRAY, 1, cv2.LINE_AA)

    # Labels
    cv2.putText(canvas, "HMD / Controller Timeseries", (ml, mt - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
    cv2.putText(canvas, "Time (s)", (ml + pw // 2 - 25, h - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (180, 180, 180), 1, cv2.LINE_AA)
    cv2.putText(canvas, "Height(m)", (2, mt + ph // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.3, (180, 180, 180), 1, cv2.LINE_AA)
    cv2.putText(canvas, "Speed", (w - 48, mt + ph // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.3, (128, 128, 128), 1, cv2.LINE_AA)

    # Tick labels
    for i in range(5):
        tv = t_max * i / 4
        tx = ml + int(tv / t_max * pw) if t_max > 0 else ml
        cv2.putText(canvas, f"{tv:.1f}", (tx - 10, mt + ph + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (150, 150, 150), 1)
        hv = h_min + h_rng * (4 - i) / 4
        cv2.putText(canvas, f"{hv:.2f}", (ml - 42, mt + int(ph * i / 4) + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (150, 150, 150), 1)
        sv = s_max * (4 - i) / 4
        cv2.putText(canvas, f"{sv:.1f}", (ml + pw + 5, mt + int(ph * i / 4) + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (128, 128, 128), 1)

    # Legend
    ly = mt + 10
    for label, color in [("HMD", _CLR_HMD), ("L Ctrl", _CLR_LEFT),
                          ("R Ctrl", _CLR_RIGHT), ("Speed", _CLR_GRAY)]:
        cv2.line(canvas, (ml + 5, ly), (ml + 20, ly), color, 2)
        cv2.putText(canvas, label, (ml + 25, ly + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (200, 200, 200), 1)
        ly += 14

    return {
        'image': canvas, 't': t, 't_max': t_max,
        'h_min': h_min, 'h_rng': h_rng,
        'ml': ml, 'mt': mt, 'pw': pw, 'ph': ph,
    }


def render_panel_timeseries(cache, hmd_data, idx, w, h):
    """Render timeseries panel: copy cached background + draw cursor."""
    canvas = cache['image'].copy()
    t = cache['t']
    t_max = cache['t_max']
    ml, mt, pw, ph = cache['ml'], cache['mt'], cache['pw'], cache['ph']

    if 0 <= idx < len(t) and t_max > 0:
        x_cur = ml + int(t[idx] / t_max * pw)
        cv2.line(canvas, (x_cur, mt), (x_cur, mt + ph),
                 (200, 200, 200), 1, cv2.LINE_AA)
        y_dot = mt + ph - int((hmd_data.hmd_pos[idx, 1] - cache['h_min']) / cache['h_rng'] * ph)
        cv2.circle(canvas, (x_cur, y_dot), 4, _CLR_HMD, -1, cv2.LINE_AA)
        cv2.circle(canvas, (x_cur, y_dot), 4, (255, 255, 255), 1, cv2.LINE_AA)
    return canvas


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
    """Export session visualization as MP4 video using OpenCV (fast)."""
    import time

    if not HAS_CV2:
        print("  Error: OpenCV required for video export. pip install opencv-python")
        return

    has_hmd = hmd_data is not None
    pw, ph = 640, 480
    canvas_w = pw * 2
    canvas_h = ph * 2 if has_hmd else ph

    # Pre-compute bounds and caches
    skel_bounds = compute_fixed_bounds(dataset, min_confidence)
    hmd_bounds = compute_hmd_bounds(hmd_data) if has_hmd else None
    ts_cache = build_timeseries_cache(hmd_data, pw, ph) if has_hmd else None

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(output_path, fourcc, fps, (canvas_w, canvas_h))
    if not writer.isOpened():
        print(f"  Error: Could not open video writer for {output_path}")
        return

    t0 = time.time()
    n = len(dataset)

    for i in range(n):
        p_2d = render_panel_2d(dataset, i, pw, ph, min_confidence)
        p_3d = render_panel_skeleton_3d(
            dataset.get_joints_3d(i), pw, ph, min_confidence, skel_bounds)
        top_row = np.hstack([p_2d, p_3d])

        if has_hmd:
            hmd_idx = hmd_data.get_idx_by_ratio(i, n)
            p_hmd = render_panel_hmd_3d(hmd_data, hmd_idx, pw, ph, hmd_bounds)
            p_ts = render_panel_timeseries(ts_cache, hmd_data, hmd_idx, pw, ph)
            canvas = np.vstack([top_row, np.hstack([p_hmd, p_ts])])
        else:
            canvas = top_row

        if session_name:
            tw = cv2.getTextSize(session_name, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)[0][0]
            tx = (canvas_w - tw) // 2
            cv2.putText(canvas, session_name, (tx, 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(canvas, session_name, (tx, 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 1, cv2.LINE_AA)

        writer.write(canvas)

        if (i + 1) % 50 == 0:
            elapsed = time.time() - t0
            rate = (i + 1) / elapsed
            eta = (n - i - 1) / rate
            print(f"  Rendering {i+1}/{n}... ({rate:.1f} fps, ETA {eta:.0f}s)")

    writer.release()
    elapsed = time.time() - t0
    print(f"  Video saved: {output_path} ({n} frames in {elapsed:.1f}s, {n/elapsed:.1f} fps)")


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

    # Preview one session (HMD auto-loaded from synced_data.csv)
    python visualize_batch_ego_dataset.py --batch-dir batch_out/ --session Dancing1_20260214_001511

    # Export all sessions as video
    python visualize_batch_ego_dataset.py --batch-dir batch_out/ --mode video

    # Fallback: load raw HMD CSVs from a separate directory
    python visualize_batch_ego_dataset.py --batch-dir batch_out/ --hmd-dir Test/

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
                        help='Fallback directory with raw HMD CSVs (auto-loaded from synced_data.csv if available)')
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
            hmd_status = "synced_data.csv" if s['synced_csv'] else "none"
            if not s['synced_csv'] and hmd_dir:
                hmd_csv = find_hmd_csv(s['name'], hmd_dir)
                hmd_status = f"hmd-dir: {'found' if hmd_csv else 'not found'}"
            print(f"  {i+1:3d}. {s['name']}  ({n_frames} frames, {s['status']}) | HMD: {hmd_status}")
        return

    # Filter to requested session
    if args.session:
        sessions = [s for s in sessions if s['name'] == args.session]
        if not sessions:
            print(f"Session '{args.session}' not found. Use --list to see available sessions.")
            sys.exit(1)

    def load_hmd_for_session(s: Dict) -> Optional[HMDData]:
        """Load HMD data: prefer synced_data.csv, fall back to --hmd-dir."""
        if s['synced_csv']:
            print(f"  Loading HMD from {s['synced_csv'].name}")
            return HMDData(str(s['synced_csv']))
        if hmd_dir:
            hmd_csv = find_hmd_csv(s['name'], hmd_dir)
            if hmd_csv:
                print(f"  Loading HMD from {hmd_csv.name} (--hmd-dir fallback)")
                return HMDData(str(hmd_csv))
            print(f"  Warning: No HMD CSV found for {s['name']}")
        return None

    # Preview mode: show one session interactively
    if args.mode == 'preview':
        s = sessions[0]
        print(f"\nLoading session: {s['name']}")
        dataset = EgoDataset(str(s['ego_dir']))
        hmd_data = load_hmd_for_session(s)

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

            hmd_data = load_hmd_for_session(s)

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
