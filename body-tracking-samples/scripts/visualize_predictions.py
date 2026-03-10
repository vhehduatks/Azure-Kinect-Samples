#!/usr/bin/env python3
"""
Prediction Visualization Tool

Visualizes and compares annotation (ground truth) vs model prediction skeletons
on ego-view images. Supports side-by-side, overlay, and batch export modes.

Usage:
    # Interactive overlay (annotations muted, predictions bright)
    python visualize_predictions.py F:\preprocessing_egodataset_weightmodify_ver2

    # Side-by-side comparison
    python visualize_predictions.py F:\preprocessing_egodataset_weightmodify_ver2 --mode side-by-side

    # Export comparison images
    python visualize_predictions.py F:\preprocessing_egodataset_weightmodify_ver2 --mode export -o comparison_out/

    # Specific session index
    python visualize_predictions.py F:\preprocessing_egodataset_weightmodify_ver2 --session 3

    # Adjust confidence thresholds
    python visualize_predictions.py F:\preprocessing_egodataset_weightmodify_ver2 --min-pred-conf 0.5 --min-annot-conf 2

Keyboard shortcuts (interactive modes):
    Left/Right    Previous/Next frame
    PgUp/PgDown   Skip 10 frames
    Home/End      First/Last frame
    N/P           Next/Previous session
"""

import argparse
import json
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
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
# Constants (shared with visualize_ego_dataset.py)
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

BONE_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 26),
    (26, 27), (27, 28), (28, 29), (27, 30), (30, 31),
    (2, 4), (4, 5), (5, 6), (6, 7),
    (2, 11), (11, 12), (12, 13), (13, 14),
    (0, 18), (18, 19), (19, 20), (20, 21),
    (0, 22), (22, 23), (23, 24), (24, 25),
]

EXCLUDED_JOINTS = {8, 9, 10, 15, 16, 17}

NUM_JOINTS = 32


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
    return get_joint_part(parent_id)


# =============================================================================
# Color Palettes
# =============================================================================

# Muted/desaturated for annotations (ground truth)
ANNOT_COLORS_BGR = {
    'spine':     (100, 140, 80),
    'head':      (80, 80, 140),
    'left_arm':  (140, 120, 80),
    'right_arm': (80, 120, 140),
    'left_leg':  (130, 100, 120),
    'right_leg': (120, 130, 100),
}

ANNOT_COLORS_RGB = {
    k: (v[2]/255, v[1]/255, v[0]/255) for k, v in ANNOT_COLORS_BGR.items()
}

# Bright for predictions
PRED_COLORS_BGR = {
    'spine':     (50, 255, 50),
    'head':      (50, 50, 255),
    'left_arm':  (255, 180, 0),
    'right_arm': (0, 180, 255),
    'left_leg':  (255, 50, 200),
    'right_leg': (200, 255, 50),
}

PRED_COLORS_RGB = {
    k: (v[2]/255, v[1]/255, v[0]/255) for k, v in PRED_COLORS_BGR.items()
}

# Standard body-part colors (for side-by-side, each panel uses its own)
PART_COLORS_RGB = {
    'spine':     (0.18, 0.80, 0.44),
    'head':      (0.91, 0.30, 0.24),
    'left_arm':  (0.20, 0.60, 0.86),
    'right_arm': (0.95, 0.61, 0.07),
    'left_leg':  (0.61, 0.35, 0.71),
    'right_leg': (0.10, 0.74, 0.61),
}


# =============================================================================
# Data Loading
# =============================================================================

def load_annotations(annotations_dir: Path) -> List[Dict]:
    """Load annotation JSONs sorted by frame number."""
    frames = []
    for jf in sorted(annotations_dir.glob('frame_*.json')):
        with open(jf) as f:
            data = json.load(f)
        data['_json_path'] = str(jf)
        frames.append(data)
    return frames


def load_predictions(predictions_dir: Path) -> Dict[int, Dict]:
    """Load prediction JSONs. Returns {frame_id: data}."""
    preds = {}
    for jf in sorted(predictions_dir.glob('frame_*.json')):
        with open(jf) as f:
            data = json.load(f)
        fid = data.get('frame_id', int(jf.stem.split('_')[-1]))
        preds[fid] = data
    return preds


def annot_to_joints_2d(frame: Dict) -> Optional[np.ndarray]:
    """Convert annotation skeleton_2d to Nx4 [u, v, confidence, visible]."""
    skel = frame.get('skeleton_2d', [])
    if not skel:
        return None
    arr = np.zeros((NUM_JOINTS, 4), dtype=np.float32)
    for j in skel:
        jid = j['joint_id']
        if jid < NUM_JOINTS:
            arr[jid] = [j['u'], j['v'], j['confidence'], 1.0 if j['visible'] else 0.0]
    return arr


def pred_to_joints_2d(pred_data: Dict, min_confidence: float = 0.0) -> np.ndarray:
    """Convert prediction skeleton_2d_predicted to Nx4 [u, v, confidence, visible]."""
    joints = pred_data.get('skeleton_2d_predicted', [])
    arr = np.zeros((NUM_JOINTS, 4), dtype=np.float32)
    for j in joints:
        jid = j['joint_id']
        if jid < NUM_JOINTS:
            conf = j.get('confidence', 0.0)
            arr[jid] = [j['u'], j['v'], conf, 1.0 if conf > min_confidence else 0.0]
    return arr


def discover_sessions(dataset_root: str) -> List[Tuple[Path, Path, Path]]:
    """Discover sessions with both annotations and predictions.

    Returns list of (ego_dataset_dir, annotations_dir, predictions_dir).
    """
    root = Path(dataset_root)
    results = []
    for ann_dir in sorted(root.rglob('annotations')):
        pred_dir = ann_dir.parent / 'predictions'
        img_dir = ann_dir.parent / 'images'
        if pred_dir.exists() and img_dir.exists():
            results.append((ann_dir.parent, ann_dir, pred_dir))
    return results


# =============================================================================
# Drawing Functions
# =============================================================================

def draw_skeleton_mpl(ax, joints_2d: np.ndarray, colors_rgb: Dict,
                      min_confidence: float = 0.3, marker_size: int = 40,
                      linewidth: float = 2.0, label: str = ''):
    """Draw 2D skeleton on matplotlib axis with a specific color palette."""
    labeled = False
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
        color = colors_rgb[part]
        kw = {}
        if not labeled and label:
            kw['label'] = label
            labeled = True
        ax.plot([p_u, c_u], [p_v, c_v], color=color, linewidth=linewidth, zorder=1, **kw)

    for jid in range(min(len(joints_2d), NUM_JOINTS)):
        if jid in EXCLUDED_JOINTS:
            continue
        u, v, conf, vis = joints_2d[jid]
        if conf < min_confidence or vis < 0.5:
            continue
        part = get_joint_part(jid)
        color = colors_rgb[part]
        ax.scatter(u, v, c=[color], s=marker_size, zorder=2,
                   edgecolors='white', linewidths=0.5)


def draw_skeleton_cv(image: np.ndarray, joints_2d: np.ndarray,
                     colors_bgr: Dict, min_confidence: float = 0.3,
                     radius: int = 5, thickness: int = 2) -> np.ndarray:
    """Draw 2D skeleton overlay using OpenCV. Modifies image in-place."""
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
        color = colors_bgr[part]
        cv2.line(image, (int(p_u), int(p_v)), (int(c_u), int(c_v)),
                 color, thickness, cv2.LINE_AA)

    for jid in range(min(len(joints_2d), NUM_JOINTS)):
        if jid in EXCLUDED_JOINTS:
            continue
        u, v, conf, vis = joints_2d[jid]
        if conf < min_confidence or vis < 0.5:
            continue
        part = get_joint_part(jid)
        color = colors_bgr[part]
        cv2.circle(image, (int(u), int(v)), radius, color, -1, cv2.LINE_AA)
        cv2.circle(image, (int(u), int(v)), radius, (255, 255, 255), 1, cv2.LINE_AA)

    return image


def compute_joint_errors(joints_ann: np.ndarray, joints_pred: np.ndarray,
                         min_annot_conf: float, min_pred_conf: float) -> List[float]:
    """Compute per-joint Euclidean errors (pixels) for matched visible joints."""
    diffs = []
    for jid in range(NUM_JOINTS):
        if jid in EXCLUDED_JOINTS:
            continue
        a_u, a_v, a_conf, a_vis = joints_ann[jid]
        p_u, p_v, p_conf, p_vis = joints_pred[jid]
        if (a_conf >= min_annot_conf and a_vis > 0.5
                and p_conf >= min_pred_conf and p_vis > 0.5):
            diffs.append(np.sqrt((a_u - p_u)**2 + (a_v - p_v)**2))
    return diffs


# =============================================================================
# Interactive Overlay Preview
# =============================================================================

def preview_overlay(dataset_root: str, min_pred_conf: float = 0.3,
                    min_annot_conf: int = 1, session_idx: int = 0):
    """Interactive overlay: annotations (muted) + predictions (bright) on same image."""
    sessions = discover_sessions(dataset_root)
    if not sessions:
        print(f"No sessions with predictions/ found under {dataset_root}")
        return

    print(f"Found {len(sessions)} sessions with predictions:")
    for i, (ego_dir, ann_dir, pred_dir) in enumerate(sessions):
        n_pred = len(list(pred_dir.glob('frame_*.json')))
        marker = " <--" if i == session_idx else ""
        print(f"  [{i}] {ego_dir.relative_to(dataset_root)}  ({n_pred} pred){marker}")

    if session_idx >= len(sessions):
        session_idx = 0

    ego_dir, ann_dir, pred_dir = sessions[session_idx]
    images_dir = ego_dir / 'images'
    print(f"\nViewing: {ego_dir.relative_to(dataset_root)}")

    annotations = load_annotations(ann_dir)
    predictions = load_predictions(pred_dir)

    if not annotations:
        print("No annotation frames found.")
        return

    fig, ax = plt.subplots(1, 1, figsize=(14, 9))
    plt.subplots_adjust(bottom=0.15, top=0.92)

    ax_slider = plt.axes([0.15, 0.02, 0.70, 0.03])
    slider = Slider(ax_slider, 'Frame', 0, max(len(annotations) - 1, 1),
                    valinit=0, valstep=1, valfmt='%d')

    fig.text(0.5, 0.97,
             f"Session: {ego_dir.relative_to(dataset_root)}  "
             f"[{session_idx+1}/{len(sessions)}]  "
             f"Muted=GT  Bright=Pred  (N/P=session, arrows=frame)",
             ha='center', va='top', fontsize=10, color='gray')

    def update(frame_idx):
        frame_idx = int(frame_idx)
        ax.clear()

        frame = annotations[frame_idx]
        fid = frame.get('frame_id', frame_idx)
        joints_ann = annot_to_joints_2d(frame)

        pred_data = predictions.get(fid)
        joints_pred = pred_to_joints_2d(pred_data, min_pred_conf) if pred_data else None

        # Load image
        img_file = frame.get('image_file', f'frame_{fid:06d}.jpg')
        img_path = images_dir / img_file
        img = None
        if img_path.exists():
            if HAS_CV2:
                img = cv2.imread(str(img_path))
                if img is not None:
                    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            else:
                img = plt.imread(str(img_path))
                if img.dtype == np.float32:
                    img = (img * 255).astype(np.uint8)

        if img is not None:
            ax.imshow(img)

        # Draw annotation (muted, thicker underneath)
        if joints_ann is not None:
            draw_skeleton_mpl(ax, joints_ann, ANNOT_COLORS_RGB,
                              min_confidence=min_annot_conf,
                              linewidth=3.0, marker_size=50,
                              label='Annotation (GT)')

        # Draw prediction (bright, thinner on top)
        if joints_pred is not None:
            draw_skeleton_mpl(ax, joints_pred, PRED_COLORS_RGB,
                              min_confidence=min_pred_conf,
                              linewidth=2.0, marker_size=30,
                              label='Prediction')

        # Compute error
        err_text = ""
        if joints_ann is not None and joints_pred is not None:
            diffs = compute_joint_errors(joints_ann, joints_pred,
                                         min_annot_conf, min_pred_conf)
            if diffs:
                err_text = (f"  |  Mean: {np.mean(diffs):.1f}px  "
                            f"Med: {np.median(diffs):.1f}px  "
                            f"Max: {np.max(diffs):.1f}px  ({len(diffs)} joints)")

        has_pred = pred_data is not None
        title_color = 'red' if not has_pred else 'black'
        ax.set_title(f'Frame {fid}{" [NO PREDICTION]" if not has_pred else ""}{err_text}',
                     fontsize=11, color=title_color)
        ax.axis('off')
        if joints_ann is not None or joints_pred is not None:
            ax.legend(loc='upper right', fontsize=9, framealpha=0.7)
        fig.canvas.draw_idle()

    slider.on_changed(update)
    update(0)

    def on_key(event):
        nonlocal session_idx
        if event.key == 'right':
            slider.set_val(min(slider.val + 1, len(annotations) - 1))
        elif event.key == 'left':
            slider.set_val(max(slider.val - 1, 0))
        elif event.key == 'pagedown':
            slider.set_val(min(slider.val + 10, len(annotations) - 1))
        elif event.key == 'pageup':
            slider.set_val(max(slider.val - 10, 0))
        elif event.key == 'home':
            slider.set_val(0)
        elif event.key == 'end':
            slider.set_val(len(annotations) - 1)
        elif event.key in ('n', 'N'):
            if session_idx < len(sessions) - 1:
                plt.close(fig)
                preview_overlay(dataset_root, min_pred_conf,
                                min_annot_conf, session_idx + 1)
        elif event.key in ('p', 'P'):
            if session_idx > 0:
                plt.close(fig)
                preview_overlay(dataset_root, min_pred_conf,
                                min_annot_conf, session_idx - 1)

    fig.canvas.mpl_connect('key_press_event', on_key)
    plt.show()


# =============================================================================
# Interactive Side-by-Side Preview
# =============================================================================

def preview_side_by_side(dataset_root: str, min_pred_conf: float = 0.3,
                         min_annot_conf: int = 1, session_idx: int = 0):
    """Side-by-side: annotation on left, prediction on right."""
    sessions = discover_sessions(dataset_root)
    if not sessions:
        print(f"No sessions with predictions/ found under {dataset_root}")
        return

    print(f"Found {len(sessions)} sessions with predictions:")
    for i, (ego_dir, _, _) in enumerate(sessions):
        marker = " <--" if i == session_idx else ""
        print(f"  [{i}] {ego_dir.relative_to(dataset_root)}{marker}")

    if session_idx >= len(sessions):
        session_idx = 0

    ego_dir, ann_dir, pred_dir = sessions[session_idx]
    images_dir = ego_dir / 'images'

    annotations = load_annotations(ann_dir)
    predictions = load_predictions(pred_dir)

    if not annotations:
        print("No annotation frames found.")
        return

    fig, (ax_ann, ax_pred) = plt.subplots(1, 2, figsize=(20, 8))
    plt.subplots_adjust(bottom=0.15, top=0.92, wspace=0.05)

    ax_slider = plt.axes([0.15, 0.02, 0.70, 0.03])
    slider = Slider(ax_slider, 'Frame', 0, max(len(annotations) - 1, 1),
                    valinit=0, valstep=1, valfmt='%d')

    fig.text(0.5, 0.97,
             f"Session: {ego_dir.relative_to(dataset_root)}  "
             f"[{session_idx+1}/{len(sessions)}]  (N/P=session)",
             ha='center', va='top', fontsize=10, color='gray')

    def update(frame_idx):
        frame_idx = int(frame_idx)
        frame = annotations[frame_idx]
        fid = frame.get('frame_id', frame_idx)
        joints_ann = annot_to_joints_2d(frame)

        pred_data = predictions.get(fid)
        joints_pred = pred_to_joints_2d(pred_data, min_pred_conf) if pred_data else None

        img_file = frame.get('image_file', f'frame_{fid:06d}.jpg')
        img_path = images_dir / img_file
        img_rgb = None
        if img_path.exists():
            if HAS_CV2:
                raw = cv2.imread(str(img_path))
                if raw is not None:
                    img_rgb = cv2.cvtColor(raw, cv2.COLOR_BGR2RGB)
            else:
                img_rgb = plt.imread(str(img_path))

        for a in (ax_ann, ax_pred):
            a.clear()

        if img_rgb is not None:
            ax_ann.imshow(img_rgb)
            ax_pred.imshow(img_rgb)

        if joints_ann is not None:
            draw_skeleton_mpl(ax_ann, joints_ann, PART_COLORS_RGB,
                              min_confidence=min_annot_conf)

        if joints_pred is not None:
            draw_skeleton_mpl(ax_pred, joints_pred, PART_COLORS_RGB,
                              min_confidence=min_pred_conf)

        ax_ann.set_title(f'Annotation (GT)\nFrame {fid}', fontsize=11)
        has_pred = pred_data is not None
        ax_pred.set_title(f'Prediction{" - MISSING" if not has_pred else ""}\nFrame {fid}',
                          fontsize=11, color='red' if not has_pred else 'black')
        ax_ann.axis('off')
        ax_pred.axis('off')
        fig.canvas.draw_idle()

    slider.on_changed(update)
    update(0)

    def on_key(event):
        nonlocal session_idx
        if event.key == 'right':
            slider.set_val(min(slider.val + 1, len(annotations) - 1))
        elif event.key == 'left':
            slider.set_val(max(slider.val - 1, 0))
        elif event.key == 'pagedown':
            slider.set_val(min(slider.val + 10, len(annotations) - 1))
        elif event.key == 'pageup':
            slider.set_val(max(slider.val - 10, 0))
        elif event.key == 'home':
            slider.set_val(0)
        elif event.key == 'end':
            slider.set_val(len(annotations) - 1)
        elif event.key in ('n', 'N'):
            if session_idx < len(sessions) - 1:
                plt.close(fig)
                preview_side_by_side(dataset_root, min_pred_conf,
                                     min_annot_conf, session_idx + 1)
        elif event.key in ('p', 'P'):
            if session_idx > 0:
                plt.close(fig)
                preview_side_by_side(dataset_root, min_pred_conf,
                                     min_annot_conf, session_idx - 1)

    fig.canvas.mpl_connect('key_press_event', on_key)
    plt.show()


# =============================================================================
# Batch Export
# =============================================================================

def export_comparison(dataset_root: str, output_dir: str,
                      min_pred_conf: float = 0.3,
                      min_annot_conf: int = 1,
                      max_frames: int = 0):
    """Export overlay comparison images (annotation muted + prediction bright)."""
    if not HAS_CV2:
        print("Error: OpenCV required. pip install opencv-python")
        return

    sessions = discover_sessions(dataset_root)
    if not sessions:
        print(f"No sessions with predictions/ found under {dataset_root}")
        return

    out_root = Path(output_dir)
    out_root.mkdir(parents=True, exist_ok=True)
    total = 0

    for ego_dir, ann_dir, pred_dir in sessions:
        rel = ego_dir.relative_to(dataset_root)
        session_out = out_root / rel
        session_out.mkdir(parents=True, exist_ok=True)

        annotations = load_annotations(ann_dir)
        predictions = load_predictions(pred_dir)
        images_dir = ego_dir / 'images'

        n = len(annotations)
        if max_frames > 0:
            n = min(n, max_frames)

        for i in range(n):
            frame = annotations[i]
            fid = frame.get('frame_id', i)
            img_file = frame.get('image_file', f'frame_{fid:06d}.jpg')
            img_path = images_dir / img_file

            if not img_path.exists():
                continue
            img = cv2.imread(str(img_path))
            if img is None:
                continue

            joints_ann = annot_to_joints_2d(frame)
            pred_data = predictions.get(fid)
            joints_pred = pred_to_joints_2d(pred_data, min_pred_conf) if pred_data else None

            # Annotation (muted, thicker)
            if joints_ann is not None:
                draw_skeleton_cv(img, joints_ann, ANNOT_COLORS_BGR,
                                 min_confidence=min_annot_conf,
                                 radius=6, thickness=3)
            # Prediction (bright, thinner on top)
            if joints_pred is not None:
                draw_skeleton_cv(img, joints_pred, PRED_COLORS_BGR,
                                 min_confidence=min_pred_conf,
                                 radius=4, thickness=2)

            # Text labels
            label = f"Frame {fid}"
            if pred_data is None:
                label += " [NO PREDICTION]"
            cv2.putText(img, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(img, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 0), 1, cv2.LINE_AA)
            cv2.putText(img, "Muted=GT  Bright=Pred", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(img, "Muted=GT  Bright=Pred", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 1, cv2.LINE_AA)

            # Error stats
            if joints_ann is not None and joints_pred is not None:
                diffs = compute_joint_errors(joints_ann, joints_pred,
                                             min_annot_conf, min_pred_conf)
                if diffs:
                    err_label = (f"Err: mean={np.mean(diffs):.1f}px "
                                 f"med={np.median(diffs):.1f}px ({len(diffs)}j)")
                    cv2.putText(img, err_label, (10, 85),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
                    cv2.putText(img, err_label, (10, 85),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)

            cv2.imwrite(str(session_out / f"frame_{fid:06d}.jpg"), img,
                        [cv2.IMWRITE_JPEG_QUALITY, 95])
            total += 1

        print(f"  {rel}: {min(n, len(annotations))} frames")

    print(f"\nExported {total} comparison images to {out_root}")


# =============================================================================
# Summary Statistics
# =============================================================================

def print_summary(dataset_root: str, min_pred_conf: float = 0.3,
                  min_annot_conf: int = 1):
    """Print per-session and overall prediction error statistics."""
    sessions = discover_sessions(dataset_root)
    if not sessions:
        print(f"No sessions with predictions/ found under {dataset_root}")
        return

    all_errors = []
    print(f"\n{'Session':<60s} {'Frames':>6s} {'Pred':>6s} "
          f"{'Mean':>7s} {'Med':>7s} {'Max':>7s} {'Joints':>6s}")
    print("-" * 105)

    for ego_dir, ann_dir, pred_dir in sessions:
        rel = str(ego_dir.relative_to(dataset_root))
        annotations = load_annotations(ann_dir)
        predictions = load_predictions(pred_dir)

        session_errors = []
        matched = 0
        for frame in annotations:
            fid = frame.get('frame_id', 0)
            pred_data = predictions.get(fid)
            if pred_data is None:
                continue
            matched += 1
            joints_ann = annot_to_joints_2d(frame)
            joints_pred = pred_to_joints_2d(pred_data, min_pred_conf)
            if joints_ann is not None:
                diffs = compute_joint_errors(joints_ann, joints_pred,
                                             min_annot_conf, min_pred_conf)
                session_errors.extend(diffs)

        all_errors.extend(session_errors)

        if session_errors:
            print(f"{rel:<60s} {len(annotations):>6d} {matched:>6d} "
                  f"{np.mean(session_errors):>6.1f}px {np.median(session_errors):>6.1f}px "
                  f"{np.max(session_errors):>6.1f}px {len(session_errors):>6d}")
        else:
            print(f"{rel:<60s} {len(annotations):>6d} {matched:>6d}      -       -       -      0")

    print("-" * 105)
    if all_errors:
        print(f"{'OVERALL':<60s} {'':>6s} {'':>6s} "
              f"{np.mean(all_errors):>6.1f}px {np.median(all_errors):>6.1f}px "
              f"{np.max(all_errors):>6.1f}px {len(all_errors):>6d}")
    else:
        print("No matched prediction-annotation pairs found.")


# =============================================================================
# CLI
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='Visualize prediction vs annotation comparison on ego-view images',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Modes:
  overlay      Annotations (muted) + predictions (bright) on same image (default)
  side-by-side Annotation on left, prediction on right
  export       Save overlay comparison images to disk
  summary      Print per-session error statistics table

Examples:
  python visualize_predictions.py F:\\dataset_root
  python visualize_predictions.py F:\\dataset_root --mode side-by-side
  python visualize_predictions.py F:\\dataset_root --mode export -o comparison/
  python visualize_predictions.py F:\\dataset_root --mode summary
  python visualize_predictions.py F:\\dataset_root --session 5 --min-pred-conf 0.5
""")

    parser.add_argument('dataset_root', help='Dataset root with <participant>/<session>/ego_dataset/')
    parser.add_argument('--mode', choices=['overlay', 'side-by-side', 'export', 'summary'],
                        default='overlay', help='Visualization mode (default: overlay)')
    parser.add_argument('--session', type=int, default=0,
                        help='Session index to start viewing (default: 0)')
    parser.add_argument('--min-pred-conf', type=float, default=0.3,
                        help='Min prediction confidence to display (default: 0.3)')
    parser.add_argument('--min-annot-conf', type=int, default=1, choices=[0, 1, 2, 3],
                        help='Min annotation confidence to display (default: 1)')
    parser.add_argument('-o', '--output', default=None,
                        help='Output directory for export mode')
    parser.add_argument('--max-frames', type=int, default=0,
                        help='Max frames per session for export (0=all)')

    args = parser.parse_args()

    if args.mode == 'overlay':
        preview_overlay(args.dataset_root, args.min_pred_conf,
                        args.min_annot_conf, args.session)
    elif args.mode == 'side-by-side':
        preview_side_by_side(args.dataset_root, args.min_pred_conf,
                             args.min_annot_conf, args.session)
    elif args.mode == 'export':
        if args.output is None:
            args.output = str(Path(args.dataset_root) / 'prediction_comparison')
        export_comparison(args.dataset_root, args.output, args.min_pred_conf,
                          args.min_annot_conf, args.max_frames)
    elif args.mode == 'summary':
        print_summary(args.dataset_root, args.min_pred_conf, args.min_annot_conf)


if __name__ == '__main__':
    main()
