#!/usr/bin/env python3
"""
Ego-View Dataset Visualizer

Visualizes ego_dataset output from multi_device_offline_processor:
- 2D joint overlay on helmet camera images
- 3D skeleton visualization in helmet camera frame

Usage:
    # Interactive preview (frame-by-frame with slider)
    python visualize_ego_dataset.py --input ego_dataset/

    # Save overlay images
    python visualize_ego_dataset.py --input ego_dataset/ --output overlay_frames/ --mode images

    # Save as video
    python visualize_ego_dataset.py --input ego_dataset/ --output overlay.mp4 --mode video

    # 3D skeleton view only
    python visualize_ego_dataset.py --input ego_dataset/ --view 3d

    # Side-by-side 2D overlay + 3D skeleton
    python visualize_ego_dataset.py --input ego_dataset/ --view both
"""

import argparse
import json
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider
from pathlib import Path
from typing import List, Dict, Optional, Tuple
import warnings
warnings.filterwarnings('ignore')

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

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
    (0, 1), (1, 2), (2, 3), (3, 26),       # Spine to head
    (26, 27), (27, 28), (28, 29), (27, 30), (30, 31),  # Head/face
    (2, 4), (4, 5), (5, 6), (6, 7),        # Left arm (stop at wrist)
    (2, 11), (11, 12), (12, 13), (13, 14),  # Right arm (stop at wrist)
    (0, 18), (18, 19), (19, 20), (20, 21),  # Left leg
    (0, 22), (22, 23), (23, 24), (24, 25),  # Right leg
]

# Distal hand joints excluded from rendering (confidence=0 in ~70% of frames)
# 8=HAND_LEFT, 9=HANDTIP_LEFT, 10=THUMB_LEFT,
# 15=HAND_RIGHT, 16=HANDTIP_RIGHT, 17=THUMB_RIGHT
EXCLUDED_JOINTS = {8, 9, 10, 15, 16, 17}

# Colors by body part (BGR for OpenCV, RGB for matplotlib)
PART_COLORS_RGB = {
    'spine':     (0.18, 0.80, 0.44),  # Green
    'head':      (0.91, 0.30, 0.24),  # Red
    'left_arm':  (0.20, 0.60, 0.86),  # Blue
    'right_arm': (0.95, 0.61, 0.07),  # Orange
    'left_leg':  (0.61, 0.35, 0.71),  # Purple
    'right_leg': (0.10, 0.74, 0.61),  # Teal
}

PART_COLORS_BGR = {
    k: (int(v[2]*255), int(v[1]*255), int(v[0]*255))
    for k, v in PART_COLORS_RGB.items()
}

def get_joint_part(joint_id: int) -> str:
    if joint_id in (0, 1, 2, 3):
        return 'spine'
    elif joint_id in (26, 27, 28, 29, 30, 31):
        return 'head'
    elif joint_id in (4, 5, 6, 7, 8, 9, 10):
        return 'left_arm'
    elif joint_id in (11, 12, 13, 14, 15, 16, 17):
        return 'right_arm'
    elif joint_id in (18, 19, 20, 21):
        return 'left_leg'
    elif joint_id in (22, 23, 24, 25):
        return 'right_leg'
    return 'spine'

def get_bone_part(parent_id: int, child_id: int) -> str:
    """Determine bone color from parent joint."""
    return get_joint_part(parent_id)

CONFIDENCE_RADIUS = {0: 0, 1: 3, 2: 5, 3: 7}


# =============================================================================
# Data Loading
# =============================================================================
class EgoDataset:
    """Loads ego_dataset output directory."""

    def __init__(self, dataset_dir: str):
        self.root = Path(dataset_dir)
        self.images_dir = self.root / 'images'
        self.annotations_dir = self.root / 'annotations'
        self.metadata_path = self.root / 'metadata.json'

        self.frames: List[Dict] = []
        self.metadata: Dict = {}

        self._load()

    def _load(self):
        if not self.annotations_dir.exists():
            raise FileNotFoundError(f"Annotations directory not found: {self.annotations_dir}")
        if not self.images_dir.exists():
            raise FileNotFoundError(f"Images directory not found: {self.images_dir}")

        # Load metadata
        if self.metadata_path.exists():
            with open(self.metadata_path) as f:
                self.metadata = json.load(f)

        # Load all annotation files sorted by name
        json_files = sorted(self.annotations_dir.glob('frame_*.json'))
        for jf in json_files:
            with open(jf) as f:
                frame = json.load(f)
            frame['_json_path'] = str(jf)
            frame['_image_path'] = str(self.images_dir / frame.get('image_file', ''))
            self.frames.append(frame)

        print(f"Loaded {len(self.frames)} frames from {self.root}")
        if self.metadata:
            total = self.metadata.get('total_frames', '?')
            cb = self.metadata.get('checkerboard_detected_frames', '?')
            print(f"  Metadata: {total} total frames, {cb} with checkerboard")

    def __len__(self):
        return len(self.frames)

    def get_frame(self, idx: int) -> Dict:
        return self.frames[idx]

    def get_image(self, idx: int) -> Optional[np.ndarray]:
        """Load image for frame index. Returns BGR numpy array or None."""
        path = self.frames[idx]['_image_path']
        if not Path(path).exists():
            return None
        if HAS_CV2:
            return cv2.imread(path)
        else:
            # Fall back to matplotlib imread (returns RGB)
            img = plt.imread(path)
            if img.dtype == np.float32:
                img = (img * 255).astype(np.uint8)
            return img

    def get_joints_3d(self, idx: int) -> Optional[np.ndarray]:
        """Return Nx4 array: [x, y, z, confidence] for 32 joints."""
        frame = self.frames[idx]
        skel = frame.get('skeleton_3d', [])
        if not skel:
            return None
        arr = np.zeros((len(skel), 4), dtype=np.float32)
        for j in skel:
            jid = j['joint_id']
            if jid < len(arr):
                arr[jid] = [j['x'], j['y'], j['z'], j['confidence']]
        return arr

    def get_joints_2d(self, idx: int) -> Optional[np.ndarray]:
        """Return Nx4 array: [u, v, confidence, visible] for 32 joints."""
        frame = self.frames[idx]
        skel = frame.get('skeleton_2d', [])
        if not skel:
            return None
        arr = np.zeros((len(skel), 4), dtype=np.float32)
        for j in skel:
            jid = j['joint_id']
            if jid < len(arr):
                arr[jid] = [j['u'], j['v'], j['confidence'], 1.0 if j['visible'] else 0.0]
        return arr


# =============================================================================
# 2D Overlay (OpenCV)
# =============================================================================
def draw_skeleton_2d_cv(image: np.ndarray, joints_2d: np.ndarray,
                        min_confidence: int = 1) -> np.ndarray:
    """Draw 2D skeleton overlay on image using OpenCV. Returns annotated image."""
    img = image.copy()

    # Draw bones first (under joints)
    for parent, child in BONE_CONNECTIONS:
        if parent >= len(joints_2d) or child >= len(joints_2d):
            continue
        p_u, p_v, p_conf, p_vis = joints_2d[parent]
        c_u, c_v, c_conf, c_vis = joints_2d[child]

        if p_conf < min_confidence or c_conf < min_confidence:
            continue
        if p_vis < 0.5 or c_vis < 0.5:
            continue

        part = get_bone_part(parent, child)
        color = PART_COLORS_BGR[part]
        thickness = 2
        cv2.line(img, (int(p_u), int(p_v)), (int(c_u), int(c_v)), color, thickness, cv2.LINE_AA)

    # Draw joints
    for jid in range(min(len(joints_2d), 32)):
        if jid in EXCLUDED_JOINTS:
            continue
        u, v, conf, vis = joints_2d[jid]
        if conf < min_confidence or vis < 0.5:
            continue

        part = get_joint_part(jid)
        color = PART_COLORS_BGR[part]
        radius = CONFIDENCE_RADIUS.get(int(conf), 3)
        cv2.circle(img, (int(u), int(v)), radius, color, -1, cv2.LINE_AA)
        cv2.circle(img, (int(u), int(v)), radius, (255, 255, 255), 1, cv2.LINE_AA)

    return img


def draw_skeleton_2d_mpl(ax, joints_2d: np.ndarray, img_shape: Tuple[int, int],
                          min_confidence: int = 1):
    """Draw 2D skeleton on a matplotlib axis (for when cv2 is unavailable)."""
    # Draw bones
    for parent, child in BONE_CONNECTIONS:
        if parent >= len(joints_2d) or child >= len(joints_2d):
            continue
        p_u, p_v, p_conf, p_vis = joints_2d[parent]
        c_u, c_v, c_conf, c_vis = joints_2d[child]

        if p_conf < min_confidence or c_conf < min_confidence:
            continue
        if p_vis < 0.5 or c_vis < 0.5:
            continue

        part = get_bone_part(parent, child)
        color = PART_COLORS_RGB[part]
        ax.plot([p_u, c_u], [p_v, c_v], color=color, linewidth=2, zorder=1)

    # Draw joints
    for jid in range(min(len(joints_2d), 32)):
        if jid in EXCLUDED_JOINTS:
            continue
        u, v, conf, vis = joints_2d[jid]
        if conf < min_confidence or vis < 0.5:
            continue

        part = get_joint_part(jid)
        color = PART_COLORS_RGB[part]
        size = CONFIDENCE_RADIUS.get(int(conf), 3) * 8
        ax.scatter(u, v, c=[color], s=size, zorder=2, edgecolors='white', linewidths=0.5)


# =============================================================================
# 3D Visualization (Matplotlib)
# =============================================================================
def draw_skeleton_3d(ax, joints_3d: np.ndarray, min_confidence: int = 1,
                     fixed_bounds: Optional[Tuple] = None):
    """Draw 3D skeleton on a matplotlib 3D axis.

    Args:
        fixed_bounds: Optional (center_x, center_y, center_z, half_range) to lock the
                      coordinate system and prevent frame-to-frame shaking.
    """
    ax.clear()

    # In helmet camera frame: X=right, Y=down, Z=forward
    # For visualization: plot X vs Z (top-down like), Y as height (inverted)
    xs = joints_3d[:, 0]
    ys = joints_3d[:, 1]
    zs = joints_3d[:, 2]

    # Draw bones
    for parent, child in BONE_CONNECTIONS:
        if parent >= len(joints_3d) or child >= len(joints_3d):
            continue
        p_conf = joints_3d[parent, 3]
        c_conf = joints_3d[child, 3]

        if p_conf < min_confidence or c_conf < min_confidence:
            continue

        part = get_bone_part(parent, child)
        color = PART_COLORS_RGB[part]
        ax.plot([xs[parent], xs[child]],
                [-ys[parent], -ys[child]],  # Invert Y for display (Y-down → Y-up)
                [zs[parent], zs[child]],
                color=color, linewidth=2)

    # Draw joints
    for jid in range(min(len(joints_3d), 32)):
        if jid in EXCLUDED_JOINTS:
            continue
        conf = joints_3d[jid, 3]
        if conf < min_confidence:
            continue

        part = get_joint_part(jid)
        color = PART_COLORS_RGB[part]
        size = 20 + conf * 10
        ax.scatter(xs[jid], -ys[jid], zs[jid],
                   c=[color], s=size, edgecolors='white', linewidths=0.3)

    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z (mm)')
    ax.set_title('3D Skeleton (Helmet Camera Frame)')

    # Use fixed bounds if provided (prevents frame-to-frame shaking)
    if fixed_bounds is not None:
        cx, cy, cz, hr = fixed_bounds
        ax.set_xlim(cx - hr, cx + hr)
        ax.set_ylim(cy - hr, cy + hr)
        ax.set_zlim(cz - hr, cz + hr)
    else:
        # Fallback: compute from current frame (only confident, non-excluded joints)
        mask = np.array([(joints_3d[j, 3] >= min_confidence and j not in EXCLUDED_JOINTS)
                         for j in range(len(joints_3d))])
        valid = joints_3d[mask]
        if len(valid) > 0:
            center_x = np.mean(valid[:, 0])
            center_y = -np.mean(valid[:, 1])
            center_z = np.mean(valid[:, 2])
            max_range = max(
                np.ptp(valid[:, 0]),
                np.ptp(valid[:, 1]),
                np.ptp(valid[:, 2])
            ) / 2
            max_range = max(max_range, 200)  # At least 200mm range
            ax.set_xlim(center_x - max_range, center_x + max_range)
            ax.set_ylim(center_y - max_range, center_y + max_range)
            ax.set_zlim(center_z - max_range, center_z + max_range)


def compute_fixed_bounds(dataset, min_confidence: int = 1) -> Optional[Tuple]:
    """Pre-compute stable 3D axis bounds from the entire dataset.

    Samples frames across the dataset, collects all confident joint positions,
    and computes tight bounds from the actual data spread. This ensures
    small-range activities (dancing, gesturing) are visible rather than
    being lost in an oversized fixed window.
    """
    # Sample at most ~100 frames for speed
    n = len(dataset)
    step = max(1, n // 100)
    sample_indices = range(0, n, step)

    all_xs, all_ys, all_zs = [], [], []

    for i in sample_indices:
        joints = dataset.get_joints_3d(i)
        if joints is None or len(joints) == 0:
            continue
        for jid in range(min(len(joints), 32)):
            if jid in EXCLUDED_JOINTS:
                continue
            if joints[jid, 3] < min_confidence:
                continue
            all_xs.append(joints[jid, 0])
            all_ys.append(-joints[jid, 1])  # Invert Y to match draw_skeleton_3d
            all_zs.append(joints[jid, 2])

    if not all_xs:
        return None

    all_xs = np.array(all_xs)
    all_ys = np.array(all_ys)
    all_zs = np.array(all_zs)

    # Center on median of all joint positions (robust to outliers)
    cx = float(np.median(all_xs))
    cy = float(np.median(all_ys))
    cz = float(np.median(all_zs))

    # Half-range from 90th-percentile spread per axis, with 1.3x margin
    def spread_90(vals):
        p5, p95 = np.percentile(vals, [5, 95])
        return (p95 - p5) / 2.0

    hr = max(spread_90(all_xs), spread_90(all_ys), spread_90(all_zs)) * 1.3
    hr = max(hr, 300.0)  # Floor: at least 300mm

    print(f"  3D bounds: center=({cx:.0f}, {cy:.0f}, {cz:.0f}), range=+/-{hr:.0f}mm")
    return (cx, cy, cz, hr)


# =============================================================================
# Interactive Preview
# =============================================================================
def preview_interactive(dataset: EgoDataset, view: str = 'both',
                        min_confidence: int = 1):
    """Interactive preview with slider."""
    if len(dataset) == 0:
        print("No frames to display.")
        return

    # Pre-compute fixed 3D bounds from entire dataset to prevent shaking
    bounds_3d = None
    if view in ('3d', 'both'):
        print("  Computing stable 3D bounds from dataset...")
        bounds_3d = compute_fixed_bounds(dataset, min_confidence)

    if view == '2d':
        fig, ax_2d = plt.subplots(1, 1, figsize=(12, 8))
        ax_3d = None
    elif view == '3d':
        fig = plt.figure(figsize=(10, 8))
        ax_3d = fig.add_subplot(111, projection='3d')
        ax_2d = None
    else:  # 'both'
        fig = plt.figure(figsize=(18, 8))
        ax_2d = fig.add_subplot(121)
        ax_3d = fig.add_subplot(122, projection='3d')

    plt.subplots_adjust(bottom=0.15)
    ax_slider = plt.axes([0.15, 0.02, 0.70, 0.03])
    slider = Slider(ax_slider, 'Frame', 0, max(len(dataset) - 1, 1),
                    valinit=0, valstep=1, valfmt='%d')

    def update(frame_idx):
        frame_idx = int(frame_idx)
        frame = dataset.get_frame(frame_idx)
        joints_2d = dataset.get_joints_2d(frame_idx)
        joints_3d = dataset.get_joints_3d(frame_idx)
        cb = frame.get('checkerboard_detected', False)
        ts = frame.get('timestamp_usec', 0)
        n_bodies = frame.get('num_bodies', 0)

        info = f"Frame {frame_idx} | ts={ts} | bodies={n_bodies} | CB={'Yes' if cb else 'No'}"

        if ax_2d is not None:
            ax_2d.clear()
            img = dataset.get_image(frame_idx)
            if img is not None:
                # Convert BGR to RGB for matplotlib display
                if HAS_CV2 and len(img.shape) == 3 and img.shape[2] == 3:
                    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                else:
                    img_rgb = img
                ax_2d.imshow(img_rgb)

                if joints_2d is not None and len(joints_2d) > 0:
                    draw_skeleton_2d_mpl(ax_2d, joints_2d, img.shape[:2], min_confidence)

            ax_2d.set_title(f'Ego-View 2D Overlay\n{info}')
            ax_2d.axis('off')

        if ax_3d is not None:
            if joints_3d is not None and len(joints_3d) > 0:
                draw_skeleton_3d(ax_3d, joints_3d, min_confidence, fixed_bounds=bounds_3d)
                ax_3d.set_title(f'3D Skeleton (Helmet Frame)\n{info}')
            else:
                ax_3d.clear()
                ax_3d.set_title(f'3D Skeleton - No Data\n{info}')

        fig.canvas.draw_idle()

    slider.on_changed(update)
    update(0)

    # Keyboard navigation
    def on_key(event):
        if event.key == 'right':
            new_val = min(slider.val + 1, len(dataset) - 1)
            slider.set_val(new_val)
        elif event.key == 'left':
            new_val = max(slider.val - 1, 0)
            slider.set_val(new_val)
        elif event.key == 'pagedown':
            new_val = min(slider.val + 10, len(dataset) - 1)
            slider.set_val(new_val)
        elif event.key == 'pageup':
            new_val = max(slider.val - 10, 0)
            slider.set_val(new_val)
        elif event.key == 'home':
            slider.set_val(0)
        elif event.key == 'end':
            slider.set_val(len(dataset) - 1)

    fig.canvas.mpl_connect('key_press_event', on_key)

    plt.show()


# =============================================================================
# Batch Output: Overlay Images
# =============================================================================
def export_overlay_images(dataset: EgoDataset, output_dir: str,
                          min_confidence: int = 1):
    """Save 2D overlay images to directory."""
    if not HAS_CV2:
        print("Error: OpenCV (cv2) required for image export. Install with: pip install opencv-python")
        return

    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

    for i in range(len(dataset)):
        img = dataset.get_image(i)
        if img is None:
            continue

        joints_2d = dataset.get_joints_2d(i)
        if joints_2d is not None and len(joints_2d) > 0:
            img = draw_skeleton_2d_cv(img, joints_2d, min_confidence)

        frame = dataset.get_frame(i)
        cb = frame.get('checkerboard_detected', False)
        ts = frame.get('timestamp_usec', 0)

        # Add text overlay
        label = f"Frame {i} | CB: {'Y' if cb else 'N'} | ts: {ts}"
        cv2.putText(img, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(img, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 0, 0), 1, cv2.LINE_AA)

        filename = f"frame_{i:06d}.jpg"
        cv2.imwrite(str(out / filename), img, [cv2.IMWRITE_JPEG_QUALITY, 95])

        if (i + 1) % 100 == 0:
            print(f"  Exported {i + 1}/{len(dataset)} frames...")

    print(f"Exported {len(dataset)} overlay images to {out}")


# =============================================================================
# Batch Output: Video
# =============================================================================
def export_video(dataset: EgoDataset, output_path: str, fps: float = 30.0,
                 view: str = 'both', min_confidence: int = 1):
    """Export visualization as MP4 video."""
    if view == '2d' and HAS_CV2:
        _export_video_cv(dataset, output_path, fps, min_confidence)
    else:
        _export_video_mpl(dataset, output_path, fps, view, min_confidence)


def _export_video_cv(dataset: EgoDataset, output_path: str, fps: float,
                     min_confidence: int):
    """Fast video export of 2D overlay using OpenCV VideoWriter."""
    # Get frame size from first image
    first_img = None
    for i in range(len(dataset)):
        first_img = dataset.get_image(i)
        if first_img is not None:
            break

    if first_img is None:
        print("Error: No valid images found.")
        return

    h, w = first_img.shape[:2]
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(output_path, fourcc, fps, (w, h))

    for i in range(len(dataset)):
        img = dataset.get_image(i)
        if img is None:
            continue

        joints_2d = dataset.get_joints_2d(i)
        if joints_2d is not None and len(joints_2d) > 0:
            img = draw_skeleton_2d_cv(img, joints_2d, min_confidence)

        frame = dataset.get_frame(i)
        cb = frame.get('checkerboard_detected', False)
        label = f"Frame {i} | CB: {'Y' if cb else 'N'}"
        cv2.putText(img, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(img, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 0, 0), 1, cv2.LINE_AA)

        writer.write(img)

        if (i + 1) % 100 == 0:
            print(f"  Encoded {i + 1}/{len(dataset)} frames...")

    writer.release()
    print(f"Video saved to {output_path}")


def _export_video_mpl(dataset: EgoDataset, output_path: str, fps: float,
                      view: str, min_confidence: int):
    """Video export using matplotlib (supports 3D and both views)."""
    from matplotlib.animation import FuncAnimation, FFMpegWriter

    # Pre-compute fixed 3D bounds
    bounds_3d = None
    if view in ('3d', 'both'):
        print("  Computing stable 3D bounds from dataset...")
        bounds_3d = compute_fixed_bounds(dataset, min_confidence)

    if view == '2d':
        fig, ax_2d = plt.subplots(1, 1, figsize=(12, 8))
        ax_3d = None
    elif view == '3d':
        fig = plt.figure(figsize=(10, 8))
        ax_3d = fig.add_subplot(111, projection='3d')
        ax_2d = None
    else:
        fig = plt.figure(figsize=(18, 8))
        ax_2d = fig.add_subplot(121)
        ax_3d = fig.add_subplot(122, projection='3d')

    def update(frame_idx):
        frame = dataset.get_frame(frame_idx)
        joints_2d = dataset.get_joints_2d(frame_idx)
        joints_3d = dataset.get_joints_3d(frame_idx)
        cb = frame.get('checkerboard_detected', False)
        info = f"Frame {frame_idx} | CB={'Yes' if cb else 'No'}"

        if ax_2d is not None:
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

        if ax_3d is not None:
            if joints_3d is not None and len(joints_3d) > 0:
                draw_skeleton_3d(ax_3d, joints_3d, min_confidence, fixed_bounds=bounds_3d)
                ax_3d.set_title(f'3D Skeleton\n{info}')
            else:
                ax_3d.clear()
                ax_3d.set_title(f'3D - No Data\n{info}')

        if (frame_idx + 1) % 50 == 0:
            print(f"  Rendering {frame_idx + 1}/{len(dataset)}...")

        return []

    anim = FuncAnimation(fig, update, frames=len(dataset), blit=False)

    try:
        writer = FFMpegWriter(fps=fps, codec='libx264',
                              extra_args=['-pix_fmt', 'yuv420p'])
        anim.save(output_path, writer=writer)
        print(f"Video saved to {output_path}")
    except Exception as e:
        print(f"FFmpeg export failed: {e}")
        print("Trying fallback with pillow writer...")
        try:
            anim.save(output_path.replace('.mp4', '.gif'), writer='pillow', fps=fps)
            print(f"GIF saved to {output_path.replace('.mp4', '.gif')}")
        except Exception as e2:
            print(f"Fallback also failed: {e2}")
            print("Install ffmpeg for video export: conda install ffmpeg")

    plt.close(fig)


# =============================================================================
# CLI
# =============================================================================
def main():
    parser = argparse.ArgumentParser(
        description='Visualize ego-view dataset from multi_device_offline_processor',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Interactive preview
  python visualize_ego_dataset.py --input ego_dataset/

  # Save overlay images
  python visualize_ego_dataset.py --input ego_dataset/ --output overlays/ --mode images

  # Save video (2D overlay only, fast with OpenCV)
  python visualize_ego_dataset.py --input ego_dataset/ --output overlay.mp4 --mode video --view 2d

  # Save video (side-by-side 2D + 3D)
  python visualize_ego_dataset.py --input ego_dataset/ --output combined.mp4 --mode video --view both

Keyboard shortcuts (preview mode):
  Left/Right    Previous/Next frame
  PgUp/PgDown   Skip 10 frames
  Home/End      First/Last frame
""")

    parser.add_argument('--input', '-i', required=True,
                        help='Path to ego_dataset directory')
    parser.add_argument('--output', '-o', default=None,
                        help='Output path (directory for images, file for video)')
    parser.add_argument('--mode', choices=['preview', 'images', 'video'],
                        default='preview',
                        help='Output mode (default: preview)')
    parser.add_argument('--view', choices=['2d', '3d', 'both'],
                        default='both',
                        help='Visualization view (default: both)')
    parser.add_argument('--fps', type=float, default=30.0,
                        help='Video FPS (default: 30)')
    parser.add_argument('--min-confidence', type=int, default=1, choices=[0, 1, 2, 3],
                        help='Minimum joint confidence to display (default: 1)')

    args = parser.parse_args()

    # Load dataset
    print(f"Loading ego dataset: {args.input}")
    dataset = EgoDataset(args.input)

    if len(dataset) == 0:
        print("Error: No frames found in dataset.")
        return

    # Print summary
    cb_count = sum(1 for f in dataset.frames if f.get('checkerboard_detected', False))
    skel_count = sum(1 for f in dataset.frames if f.get('skeleton_3d', []))
    print(f"  Total frames: {len(dataset)}")
    print(f"  With checkerboard: {cb_count}")
    print(f"  With skeleton: {skel_count}")

    if args.mode == 'preview' or (args.mode == 'preview' and args.output is None):
        print("\nStarting interactive preview...")
        print("  Use Left/Right arrows or slider to navigate frames.")
        preview_interactive(dataset, view=args.view,
                          min_confidence=args.min_confidence)

    elif args.mode == 'images':
        if args.output is None:
            args.output = str(Path(args.input) / 'overlays')
        print(f"\nExporting overlay images to: {args.output}")
        export_overlay_images(dataset, args.output,
                            min_confidence=args.min_confidence)

    elif args.mode == 'video':
        if args.output is None:
            args.output = str(Path(args.input) / 'ego_overlay.mp4')
        print(f"\nExporting video to: {args.output}")
        export_video(dataset, args.output, fps=args.fps, view=args.view,
                    min_confidence=args.min_confidence)


if __name__ == '__main__':
    main()
