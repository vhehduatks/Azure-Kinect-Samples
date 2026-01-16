#!/usr/bin/env python3
"""
Synchronized Skeleton and HMD Data Visualizer

Visualizes synchronized skeleton and HMD data in separate 3D coordinate systems.
- Skeleton: 3D joints with bone connections
- HMD: 3 labeled points (Head, Left Controller, Right Controller)

Usage:
    # Output as video
    python visualize_synced_data.py --input synced.csv --skeleton skeleton.csv --output video.mp4

    # Output as images
    python visualize_synced_data.py --input synced.csv --skeleton skeleton.csv --output frames/ --mode images

    # Preview without saving
    python visualize_synced_data.py --input synced.csv --skeleton skeleton.csv --preview
"""

import argparse
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation, FFMpegWriter
from pathlib import Path
from typing import Optional, List, Tuple
import warnings
warnings.filterwarnings('ignore')


# =============================================================================
# Azure Kinect Body Tracking Joint IDs and Bone Connections
# =============================================================================
JOINT_NAMES = [
    'PELVIS', 'SPINE_NAVEL', 'SPINE_CHEST', 'NECK',
    'CLAVICLE_LEFT', 'SHOULDER_LEFT', 'ELBOW_LEFT', 'WRIST_LEFT',
    'HAND_LEFT', 'HANDTIP_LEFT', 'THUMB_LEFT',
    'CLAVICLE_RIGHT', 'SHOULDER_RIGHT', 'ELBOW_RIGHT', 'WRIST_RIGHT',
    'HAND_RIGHT', 'HANDTIP_RIGHT', 'THUMB_RIGHT',
    'HIP_LEFT', 'KNEE_LEFT', 'ANKLE_LEFT', 'FOOT_LEFT',
    'HIP_RIGHT', 'KNEE_RIGHT', 'ANKLE_RIGHT', 'FOOT_RIGHT',
    'HEAD', 'NOSE', 'EYE_LEFT', 'EAR_LEFT', 'EYE_RIGHT', 'EAR_RIGHT'
]

# Bone connections (parent, child)
BONE_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 26),  # Spine to head
    (26, 27), (27, 28), (28, 29), (27, 30), (30, 31),  # Head/face
    (2, 4), (4, 5), (5, 6), (6, 7), (7, 8), (8, 9), (7, 10),  # Left arm
    (2, 11), (11, 12), (12, 13), (13, 14), (14, 15), (15, 16), (14, 17),  # Right arm
    (0, 18), (18, 19), (19, 20), (20, 21),  # Left leg
    (0, 22), (22, 23), (23, 24), (24, 25),  # Right leg
]

# Joint colors by body part
JOINT_COLORS = {
    'spine': '#2ecc71',      # Green
    'head': '#e74c3c',       # Red
    'left_arm': '#3498db',   # Blue
    'right_arm': '#f39c12',  # Orange
    'left_leg': '#9b59b6',   # Purple
    'right_leg': '#1abc9c',  # Teal
}

def get_joint_color(joint_id: int) -> str:
    """Get color for joint based on body part."""
    if joint_id in [0, 1, 2, 3]:
        return JOINT_COLORS['spine']
    elif joint_id in [26, 27, 28, 29, 30, 31]:
        return JOINT_COLORS['head']
    elif joint_id in [4, 5, 6, 7, 8, 9, 10]:
        return JOINT_COLORS['left_arm']
    elif joint_id in [11, 12, 13, 14, 15, 16, 17]:
        return JOINT_COLORS['right_arm']
    elif joint_id in [18, 19, 20, 21]:
        return JOINT_COLORS['left_leg']
    elif joint_id in [22, 23, 24, 25]:
        return JOINT_COLORS['right_leg']
    return '#95a5a6'


# =============================================================================
# Data Loading
# =============================================================================

def load_synced_data(filepath: str) -> pd.DataFrame:
    """Load synchronized HMD data."""
    print(f"Loading synced HMD data: {filepath}")
    df = pd.read_csv(filepath)
    print(f"  Rows: {len(df)}")
    return df


def load_skeleton_data(filepath: str) -> pd.DataFrame:
    """Load skeleton data."""
    print(f"Loading skeleton data: {filepath}")
    df = pd.read_csv(filepath)
    print(f"  Rows: {len(df)}")
    return df


def extract_skeleton_joints(df: pd.DataFrame, frame_idx: int) -> Optional[np.ndarray]:
    """
    Extract all joint positions for a given frame.

    Returns:
        (32, 3) array of joint positions or None if no data
    """
    if frame_idx >= len(df):
        return None

    row = df.iloc[frame_idx]
    joints = np.zeros((32, 3))

    for j in range(32):
        x_col = f'J{j}_x'
        y_col = f'J{j}_y'
        z_col = f'J{j}_z'

        if x_col in df.columns:
            joints[j] = [row[x_col], row[y_col], row[z_col]]

    return joints


def extract_hmd_points(df: pd.DataFrame, frame_idx: int) -> Optional[dict]:
    """
    Extract HMD and controller positions for a given frame.

    Returns:
        dict with 'head', 'left', 'right' positions or None
    """
    if frame_idx >= len(df):
        return None

    row = df.iloc[frame_idx]

    result = {
        'head': np.array([row['hmd_pos_x'], row['hmd_pos_y'], row['hmd_pos_z']]),
        'left': None,
        'right': None,
        'timestamp': row.get('timestamp_ms', frame_idx)
    }

    # Check for controller data
    if 'left_pos_x' in df.columns:
        result['left'] = np.array([row['left_pos_x'], row['left_pos_y'], row['left_pos_z']])
    if 'right_pos_x' in df.columns:
        result['right'] = np.array([row['right_pos_x'], row['right_pos_y'], row['right_pos_z']])

    return result


# =============================================================================
# Visualization
# =============================================================================

class DualViewVisualizer:
    """Visualizer for skeleton and HMD data in separate coordinate systems."""

    def __init__(self, skeleton_df: pd.DataFrame, hmd_df: pd.DataFrame,
                 figsize: Tuple[int, int] = (16, 8)):
        self.skeleton_df = skeleton_df
        self.hmd_df = hmd_df
        self.figsize = figsize

        # Compute data bounds for consistent axis limits
        self._compute_bounds()

    def _compute_bounds(self):
        """Compute axis bounds from data."""
        # Skeleton bounds (in mm)
        skel_points = []
        for i in range(min(len(self.skeleton_df), 100)):  # Sample frames
            joints = extract_skeleton_joints(self.skeleton_df, i)
            if joints is not None:
                # Filter out zero joints
                valid = joints[np.any(joints != 0, axis=1)]
                skel_points.extend(valid)

        if skel_points:
            skel_points = np.array(skel_points)
            self.skel_center = np.mean(skel_points, axis=0)
            skel_range = np.max(np.abs(skel_points - self.skel_center)) * 1.2
            self.skel_range = max(skel_range, 500)  # At least 500mm
        else:
            self.skel_center = np.array([0, 0, 1000])
            self.skel_range = 1000

        # HMD bounds (in meters, Unity coordinate)
        hmd_points = []
        for i in range(min(len(self.hmd_df), 100)):
            hmd = extract_hmd_points(self.hmd_df, i)
            if hmd:
                hmd_points.append(hmd['head'])
                if hmd['left'] is not None:
                    hmd_points.append(hmd['left'])
                if hmd['right'] is not None:
                    hmd_points.append(hmd['right'])

        if hmd_points:
            hmd_points = np.array(hmd_points)
            self.hmd_center = np.mean(hmd_points, axis=0)
            hmd_range = np.max(np.abs(hmd_points - self.hmd_center)) * 1.5
            self.hmd_range = max(hmd_range, 1.0)  # At least 1m
        else:
            self.hmd_center = np.array([0, 1, 0])
            self.hmd_range = 1.0

    def _setup_skeleton_axes(self, ax):
        """Setup skeleton 3D axes.

        Kinect coordinate: X=right, Y=up, Z=forward (into scene)
        Matplotlib 3D: X=right, Y=depth, Z=up
        We swap Y<->Z so that XZ plane (floor) is at the bottom.
        """
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Z (mm)')  # Kinect Z -> matplotlib Y
        ax.set_zlabel('Y (mm)')  # Kinect Y -> matplotlib Z (vertical)
        ax.set_title('Skeleton (Kinect Coordinate)', fontsize=12, fontweight='bold')

        # Set consistent limits (swap Y and Z, negate Y)
        ax.set_xlim(self.skel_center[0] - self.skel_range,
                    self.skel_center[0] + self.skel_range)
        ax.set_ylim(self.skel_center[2] - self.skel_range,  # Z -> Y axis
                    self.skel_center[2] + self.skel_range)
        ax.set_zlim(-self.skel_center[1] - self.skel_range,  # -Y -> Z axis
                    -self.skel_center[1] + self.skel_range)

        # View angle (front view: looking at -Y in matplotlib, which is -Z in Kinect)
        ax.view_init(elev=20, azim=-90)

    def _setup_hmd_axes(self, ax):
        """Setup HMD 3D axes."""
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title('HMD & Controllers (Unity Coordinate)', fontsize=12, fontweight='bold')

        # Set consistent limits
        ax.set_xlim(self.hmd_center[0] - self.hmd_range,
                    self.hmd_center[0] + self.hmd_range)
        ax.set_ylim(self.hmd_center[1] - self.hmd_range,
                    self.hmd_center[1] + self.hmd_range)
        ax.set_zlim(self.hmd_center[2] - self.hmd_range,
                    self.hmd_center[2] + self.hmd_range)

        # View angle
        ax.view_init(elev=20, azim=-60)

    def _draw_skeleton(self, ax, joints: np.ndarray):
        """Draw skeleton with joints and bones.

        Swaps Y and Z for proper floor plane visualization.
        Kinect: (X, Y, Z) -> Matplotlib: (X, Z, -Y)
        Y is negated because Kinect Y points down from camera.
        """
        # Draw bones
        for parent, child in BONE_CONNECTIONS:
            if parent < len(joints) and child < len(joints):
                p1, p2 = joints[parent], joints[child]
                # Skip if either joint is at origin (invalid)
                if np.allclose(p1, 0) or np.allclose(p2, 0):
                    continue
                # Swap Y and Z, negate Y: plot (X, Z, -Y)
                ax.plot([p1[0], p2[0]], [p1[2], p2[2]], [-p1[1], -p2[1]],
                       color='#34495e', linewidth=2, alpha=0.7)

        # Draw joints
        for j, pos in enumerate(joints):
            if np.allclose(pos, 0):
                continue
            color = get_joint_color(j)
            # Swap Y and Z, negate Y: plot (X, Z, -Y)
            ax.scatter(pos[0], pos[2], -pos[1], c=color, s=50, alpha=0.9)

    def _draw_hmd(self, ax, hmd_data: dict):
        """Draw HMD and controller points with labels."""
        # HMD (Head)
        head = hmd_data['head']
        ax.scatter(head[0], head[1], head[2], c='#e74c3c', s=200, marker='o',
                  label='Head', edgecolors='white', linewidths=2)
        ax.text(head[0], head[1] + 0.1, head[2], 'Head', fontsize=10,
               fontweight='bold', ha='center', color='#e74c3c')

        # Left Controller
        if hmd_data['left'] is not None:
            left = hmd_data['left']
            ax.scatter(left[0], left[1], left[2], c='#3498db', s=150, marker='s',
                      label='Left Ctrl', edgecolors='white', linewidths=2)
            ax.text(left[0], left[1] - 0.1, left[2], 'L-Ctrl', fontsize=9,
                   fontweight='bold', ha='center', color='#3498db')

        # Right Controller
        if hmd_data['right'] is not None:
            right = hmd_data['right']
            ax.scatter(right[0], right[1], right[2], c='#f39c12', s=150, marker='s',
                      label='Right Ctrl', edgecolors='white', linewidths=2)
            ax.text(right[0], right[1] - 0.1, right[2], 'R-Ctrl', fontsize=9,
                   fontweight='bold', ha='center', color='#f39c12')

        # Draw lines connecting head to controllers
        if hmd_data['left'] is not None:
            left = hmd_data['left']
            ax.plot([head[0], left[0]], [head[1], left[1]], [head[2], left[2]],
                   color='#3498db', linewidth=1, linestyle='--', alpha=0.5)
        if hmd_data['right'] is not None:
            right = hmd_data['right']
            ax.plot([head[0], right[0]], [head[1], right[1]], [head[2], right[2]],
                   color='#f39c12', linewidth=1, linestyle='--', alpha=0.5)

    def render_frame(self, frame_idx: int, fig=None, axes=None) -> Tuple[plt.Figure, List]:
        """Render a single frame."""
        if fig is None:
            fig = plt.figure(figsize=self.figsize)
            ax1 = fig.add_subplot(121, projection='3d')
            ax2 = fig.add_subplot(122, projection='3d')
            axes = [ax1, ax2]
        else:
            for ax in axes:
                ax.clear()

        ax1, ax2 = axes

        # Setup axes
        self._setup_skeleton_axes(ax1)
        self._setup_hmd_axes(ax2)

        # Match frames by timestamp
        skel_frame = self._find_matching_skeleton_frame(frame_idx)

        # Draw skeleton
        if skel_frame is not None:
            joints = extract_skeleton_joints(self.skeleton_df, skel_frame)
            if joints is not None:
                self._draw_skeleton(ax1, joints)

        # Draw HMD
        hmd_data = extract_hmd_points(self.hmd_df, frame_idx)
        if hmd_data:
            self._draw_hmd(ax2, hmd_data)

        # Add frame info
        timestamp = self.hmd_df.iloc[frame_idx].get('timestamp_ms', frame_idx) if frame_idx < len(self.hmd_df) else 0
        fig.suptitle(f'Frame {frame_idx} | Timestamp: {timestamp:.0f} ms',
                    fontsize=14, fontweight='bold')

        plt.tight_layout()
        return fig, axes

    def _find_matching_skeleton_frame(self, hmd_frame_idx: int) -> Optional[int]:
        """Find skeleton frame matching HMD timestamp."""
        if hmd_frame_idx >= len(self.hmd_df):
            return None

        hmd_ts = self.hmd_df.iloc[hmd_frame_idx].get('timestamp_ms', 0)

        # Get skeleton timestamps
        if 'timestamp_usec' in self.skeleton_df.columns:
            skel_ts = self.skeleton_df['timestamp_usec'].values / 1000.0
        elif 'timestamp_ms' in self.skeleton_df.columns:
            skel_ts = self.skeleton_df['timestamp_ms'].values
        else:
            return hmd_frame_idx if hmd_frame_idx < len(self.skeleton_df) else None

        # Find closest timestamp
        diff = np.abs(skel_ts - hmd_ts)
        if np.min(diff) < 100:  # Within 100ms
            return int(np.argmin(diff))
        return None

    def save_images(self, output_dir: str, frame_step: int = 1):
        """Save frames as images."""
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)

        num_frames = len(self.hmd_df)
        print(f"Saving {num_frames // frame_step} images to {output_dir}")

        fig = plt.figure(figsize=self.figsize)
        ax1 = fig.add_subplot(121, projection='3d')
        ax2 = fig.add_subplot(122, projection='3d')
        axes = [ax1, ax2]

        for i in range(0, num_frames, frame_step):
            self.render_frame(i, fig, axes)
            fig.savefig(output_path / f'frame_{i:05d}.png', dpi=100,
                       bbox_inches='tight', facecolor='white')

            if (i // frame_step) % 10 == 0:
                print(f"  Saved frame {i}/{num_frames}")

        plt.close(fig)
        print(f"Done! Saved {num_frames // frame_step} images.")

    def save_video(self, output_path: str, fps: int = 30):
        """Save as video file."""
        print(f"Creating video: {output_path}")

        fig = plt.figure(figsize=self.figsize)
        ax1 = fig.add_subplot(121, projection='3d')
        ax2 = fig.add_subplot(122, projection='3d')
        axes = [ax1, ax2]

        num_frames = len(self.hmd_df)

        def update(frame_idx):
            self.render_frame(frame_idx, fig, axes)
            if frame_idx % 10 == 0:
                print(f"  Rendering frame {frame_idx}/{num_frames}")
            return []

        anim = FuncAnimation(fig, update, frames=num_frames, interval=1000/fps, blit=False)

        # Save video
        writer = FFMpegWriter(fps=fps, metadata={'title': 'Skeleton-HMD Visualization'})
        anim.save(output_path, writer=writer, dpi=100)

        plt.close(fig)
        print(f"Done! Saved video to {output_path}")

    def preview(self, start_frame: int = 0):
        """Interactive preview with slider."""
        fig = plt.figure(figsize=self.figsize)
        ax1 = fig.add_subplot(121, projection='3d')
        ax2 = fig.add_subplot(122, projection='3d')
        axes = [ax1, ax2]

        from matplotlib.widgets import Slider

        # Add slider
        ax_slider = plt.axes([0.2, 0.02, 0.6, 0.03])
        slider = Slider(ax_slider, 'Frame', 0, len(self.hmd_df)-1,
                       valinit=start_frame, valstep=1)

        def update(val):
            frame_idx = int(slider.val)
            self.render_frame(frame_idx, fig, axes)
            fig.canvas.draw_idle()

        slider.on_changed(update)

        # Initial render
        self.render_frame(start_frame, fig, axes)

        plt.show()


# =============================================================================
# Main
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='Visualize synchronized skeleton and HMD data',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Output as video
    python visualize_synced_data.py --input synced.csv --skeleton skeleton.csv --output video.mp4

    # Output as images
    python visualize_synced_data.py --input synced.csv --skeleton skeleton.csv --output frames/ --mode images

    # Preview interactively
    python visualize_synced_data.py --input synced.csv --skeleton skeleton.csv --preview
        """
    )

    parser.add_argument('--input', required=True, help='Synchronized CSV file (HMD data)')
    parser.add_argument('--skeleton', required=True, help='Skeleton CSV file')
    parser.add_argument('--output', help='Output path (video file or directory for images)')
    parser.add_argument('--mode', choices=['video', 'images'], default='video',
                       help='Output mode: video or images (default: video)')
    parser.add_argument('--fps', type=int, default=30, help='Video FPS (default: 30)')
    parser.add_argument('--step', type=int, default=1,
                       help='Frame step for images (default: 1, every frame)')
    parser.add_argument('--preview', action='store_true',
                       help='Interactive preview mode')

    args = parser.parse_args()

    # Load data
    hmd_df = load_synced_data(args.input)
    skeleton_df = load_skeleton_data(args.skeleton)

    # Create visualizer
    visualizer = DualViewVisualizer(skeleton_df, hmd_df)

    if args.preview:
        visualizer.preview()
    elif args.output:
        if args.mode == 'video':
            visualizer.save_video(args.output, fps=args.fps)
        else:
            visualizer.save_images(args.output, frame_step=args.step)
    else:
        print("Error: Specify --output or --preview")
        parser.print_help()


if __name__ == '__main__':
    main()
