#!/usr/bin/env python3
"""Train a 2D pose estimation model (SimpleBaseline / heatmap) on ego dataset annotations.

Stage 1 of the auto-extrinsic pipeline:
  1. Train on annotated ego frames  ->  this script
  2. Predict 2D joints on new frames ->  this script (predict subcommand)
  3. Fit extrinsic delta             ->  fit_extrinsic.py

Usage:
  python train_pose_model.py train <dataset_root> --epochs 100 --batch-size 32 -o model.pth
  python train_pose_model.py predict <ego_dataset_dir> -m model.pth -o predictions/
  python train_pose_model.py eval <dataset_root> -m model.pth
"""

import argparse
import json
import math
import os
import random
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

try:
    import torch
    import torch.nn as nn
    import torch.optim as optim
    from torch.utils.data import DataLoader, Dataset
except ImportError:
    print("ERROR: PyTorch is required. Install with: pip install torch torchvision")
    sys.exit(1)

try:
    import torchvision.models as models
    import torchvision.transforms as T
except ImportError:
    print("ERROR: torchvision is required. Install with: pip install torchvision")
    sys.exit(1)

try:
    import cv2
except ImportError:
    print("ERROR: OpenCV is required. Install with: pip install opencv-python")
    sys.exit(1)

try:
    from tqdm import tqdm
except ImportError:
    # Fallback: no-op wrapper that just passes through the iterable
    def tqdm(iterable, **kwargs):
        return iterable

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

NUM_JOINTS = 32
INPUT_H, INPUT_W = 256, 192       # 4:3 crop
HEATMAP_H, HEATMAP_W = 64, 48     # 1/4 input resolution
HEATMAP_SIGMA = 2.0

JOINT_NAMES = [
    'PELVIS', 'SPINE_NAVEL', 'SPINE_CHEST', 'NECK',
    'CLAVICLE_LEFT', 'SHOULDER_LEFT', 'ELBOW_LEFT', 'WRIST_LEFT',
    'HAND_LEFT', 'HANDTIP_LEFT', 'THUMB_LEFT',
    'CLAVICLE_RIGHT', 'SHOULDER_RIGHT', 'ELBOW_RIGHT', 'WRIST_RIGHT',
    'HAND_RIGHT', 'HANDTIP_RIGHT', 'THUMB_RIGHT',
    'HIP_LEFT', 'KNEE_LEFT', 'ANKLE_LEFT', 'FOOT_LEFT',
    'HIP_RIGHT', 'KNEE_RIGHT', 'ANKLE_RIGHT', 'FOOT_RIGHT',
    'HEAD', 'NOSE', 'EYE_LEFT', 'EAR_LEFT', 'EYE_RIGHT', 'EAR_RIGHT',
]

# Left-right flip pairs (joint indices)
FLIP_PAIRS = [
    (4, 11), (5, 12), (6, 13), (7, 14), (8, 15), (9, 16), (10, 17),   # arms
    (18, 22), (19, 23), (20, 24), (21, 25),                             # legs
    (28, 30), (29, 31),                                                  # eyes/ears
]

# ImageNet normalization
IMAGENET_MEAN = [0.485, 0.456, 0.406]
IMAGENET_STD = [0.229, 0.224, 0.225]

# ---------------------------------------------------------------------------
# Dataset discovery
# ---------------------------------------------------------------------------

def discover_annotations_dirs(dataset_root: str) -> List[Path]:
    """Find all annotations/ dirs under <dataset_root>/<participant>/<session>/ego_dataset/."""
    root = Path(dataset_root)
    dirs = sorted(root.glob("*/*/ego_dataset/annotations"))
    return [d for d in dirs if d.is_dir()]


def build_frame_index(dataset_root: str, min_visible_joints: int = 3) -> List[Dict]:
    """Build flat list of (json_path, image_path, session_key) for all valid frames."""
    ann_dirs = discover_annotations_dirs(dataset_root)
    if not ann_dirs:
        print(f"No annotation directories found under {dataset_root}")
        return []

    frames = []
    skipped = 0
    for ann_dir in tqdm(ann_dirs, desc="Scanning dirs", unit="dir"):
        images_dir = ann_dir.parent / "images"
        if not images_dir.exists():
            continue
        session_key = str(ann_dir.parent.parent)  # <participant>/<session>

        for jf in sorted(ann_dir.glob("frame_*.json")):
            with open(jf) as f:
                data = json.load(f)

            # Count visible joints with confidence > 0
            skel_2d = data.get("skeleton_2d", [])
            n_visible = sum(
                1 for j in skel_2d
                if j.get("confidence", 0) > 0 and j.get("visible", False)
            )
            if n_visible < min_visible_joints:
                skipped += 1
                continue

            image_file = data.get("image_file", "")
            image_path = images_dir / image_file
            if not image_path.exists():
                skipped += 1
                continue

            frames.append({
                "json_path": str(jf),
                "image_path": str(image_path),
                "session_key": session_key,
            })

    print(f"Frame index: {len(frames)} valid frames, {skipped} skipped")
    return frames


def split_train_val(frames: List[Dict], val_ratio: float = 0.2,
                    seed: int = 42) -> Tuple[List[Dict], List[Dict]]:
    """Split frames into train/val at session level to avoid data leakage."""
    # Group by session
    sessions: Dict[str, List[Dict]] = {}
    for f in frames:
        key = f["session_key"]
        sessions.setdefault(key, []).append(f)

    # Deterministic shuffle of sessions
    session_keys = sorted(sessions.keys())
    rng = random.Random(seed)
    rng.shuffle(session_keys)

    # Split sessions
    n_val = max(1, int(len(session_keys) * val_ratio))
    val_keys = set(session_keys[:n_val])

    train_frames, val_frames = [], []
    for key in session_keys:
        if key in val_keys:
            val_frames.extend(sessions[key])
        else:
            train_frames.extend(sessions[key])

    print(f"Split: {len(train_frames)} train, {len(val_frames)} val "
          f"({len(session_keys) - n_val} / {n_val} sessions)")
    return train_frames, val_frames


# ---------------------------------------------------------------------------
# Affine transform + bbox utilities
# ---------------------------------------------------------------------------

def bbox_from_joints(joints_2d: np.ndarray, padding: float = 0.25) -> Tuple[float, float, float, float]:
    """Compute bounding box from visible 2D joints.

    Args:
        joints_2d: Nx4 array [u, v, confidence, visible]
        padding: fractional padding around tight bbox

    Returns:
        (cx, cy, w, h) center + size, adjusted to 4:3 aspect
    """
    visible = (joints_2d[:, 2] > 0) & (joints_2d[:, 3] > 0)
    if visible.sum() < 2:
        # Fallback: use all joints with any confidence
        visible = joints_2d[:, 2] > 0
    if visible.sum() < 2:
        return (0, 0, INPUT_W, INPUT_H)

    pts = joints_2d[visible, :2]
    x_min, y_min = pts.min(axis=0)
    x_max, y_max = pts.max(axis=0)

    w = max(x_max - x_min, 1.0)
    h = max(y_max - y_min, 1.0)
    cx = (x_min + x_max) / 2.0
    cy = (y_min + y_max) / 2.0

    # Add padding
    w *= (1.0 + padding)
    h *= (1.0 + padding)

    # Adjust to 4:3 aspect (h:w = 4:3)
    aspect = INPUT_H / INPUT_W  # 256/192 = 4/3
    if h / w > aspect:
        w = h / aspect
    else:
        h = w * aspect

    return (cx, cy, w, h)


def get_affine_transform(cx: float, cy: float, w: float, h: float,
                         output_size: Tuple[int, int],
                         rot: float = 0.0, scale: float = 1.0) -> np.ndarray:
    """Get 2x3 affine transform mapping bbox region to output_size."""
    src_w = w * scale
    src_h = h * scale

    # Source points: center, center-right, center-bottom
    cos_r = math.cos(math.radians(rot))
    sin_r = math.sin(math.radians(rot))

    def _rotate(pt, c):
        dx, dy = pt[0] - c[0], pt[1] - c[1]
        return [c[0] + dx * cos_r - dy * sin_r,
                c[1] + dx * sin_r + dy * cos_r]

    src = np.float32([
        [cx, cy],
        _rotate([cx + src_w / 2, cy], [cx, cy]),
        _rotate([cx, cy + src_h / 2], [cx, cy]),
    ])

    dst_w, dst_h = output_size
    dst = np.float32([
        [dst_w / 2, dst_h / 2],
        [dst_w, dst_h / 2],
        [dst_w / 2, dst_h],
    ])

    return cv2.getAffineTransform(src, dst)


def affine_transform_pts(pts: np.ndarray, M: np.ndarray) -> np.ndarray:
    """Apply 2x3 affine to Nx2 points."""
    if len(pts) == 0:
        return pts
    ones = np.ones((len(pts), 1), dtype=np.float64)
    pts_h = np.hstack([pts.astype(np.float64), ones])  # Nx3
    return (M @ pts_h.T).T  # Nx2


# ---------------------------------------------------------------------------
# Heatmap generation + peak decoding
# ---------------------------------------------------------------------------

def generate_heatmaps(joints: np.ndarray, heatmap_size: Tuple[int, int] = (HEATMAP_W, HEATMAP_H),
                      sigma: float = HEATMAP_SIGMA) -> Tuple[np.ndarray, np.ndarray]:
    """Generate Gaussian heatmaps and per-joint weights.

    Args:
        joints: Nx4 [u_hm, v_hm, confidence, visible] in heatmap coords
        heatmap_size: (W, H)

    Returns:
        heatmaps: (N, H, W) float32
        weights: (N,) float32  per-joint loss weight
    """
    W, H = heatmap_size
    num_joints = len(joints)
    heatmaps = np.zeros((num_joints, H, W), dtype=np.float32)
    weights = np.zeros(num_joints, dtype=np.float32)

    size = 3 * sigma
    for j in range(num_joints):
        u, v, conf, vis = joints[j]
        if vis < 0.5 or conf < 0.5:
            # invisible or zero confidence -> weight = 0
            weights[j] = 0.0
            continue

        # Weight based on confidence
        if conf >= 2:
            weights[j] = 1.0
        else:
            weights[j] = 0.5

        mu_x, mu_y = u, v
        ul = [int(mu_x - size), int(mu_y - size)]
        br = [int(mu_x + size + 1), int(mu_y + size + 1)]

        if ul[0] >= W or ul[1] >= H or br[0] < 0 or br[1] < 0:
            weights[j] = 0.0
            continue

        # Generate Gaussian
        g_w = br[0] - ul[0]
        g_h = br[1] - ul[1]
        x = np.arange(g_w, dtype=np.float32) + ul[0] - mu_x
        y = np.arange(g_h, dtype=np.float32) + ul[1] - mu_y
        xx, yy = np.meshgrid(x, y)
        g = np.exp(-(xx ** 2 + yy ** 2) / (2 * sigma ** 2))

        # Clip to heatmap bounds
        g_x_start = max(0, -ul[0])
        g_y_start = max(0, -ul[1])
        g_x_end = min(g_w, W - ul[0])
        g_y_end = min(g_h, H - ul[1])
        h_x_start = max(0, ul[0])
        h_y_start = max(0, ul[1])
        h_x_end = h_x_start + (g_x_end - g_x_start)
        h_y_end = h_y_start + (g_y_end - g_y_start)

        heatmaps[j, h_y_start:h_y_end, h_x_start:h_x_end] = g[g_y_start:g_y_end, g_x_start:g_x_end]

    return heatmaps, weights


def decode_heatmaps(heatmaps: np.ndarray) -> np.ndarray:
    """Decode heatmap peaks with sub-pixel refinement.

    Args:
        heatmaps: (N, H, W) float32

    Returns:
        peaks: (N, 3) [x, y, confidence]
    """
    N, H, W = heatmaps.shape
    peaks = np.zeros((N, 3), dtype=np.float32)

    for j in range(N):
        hm = heatmaps[j]
        idx = hm.argmax()
        y, x = divmod(int(idx), W)
        peak_val = float(hm[y, x])

        if peak_val < 0.01:
            peaks[j] = [0, 0, 0]
            continue

        # Sub-pixel refinement: shift by 0.25 in direction of gradient
        px, py = float(x), float(y)
        if 0 < x < W - 1:
            diff = float(hm[y, x + 1]) - float(hm[y, x - 1])
            px += 0.25 * np.sign(diff)
        if 0 < y < H - 1:
            diff = float(hm[y + 1, x]) - float(hm[y - 1, x])
            py += 0.25 * np.sign(diff)

        peaks[j] = [px, py, peak_val]

    return peaks


# ---------------------------------------------------------------------------
# Dataset class
# ---------------------------------------------------------------------------

class EgoPoseDataset(Dataset):
    """PyTorch dataset for ego pose heatmap training."""

    def __init__(self, frame_list: List[Dict], is_train: bool = True,
                 input_size: Tuple[int, int] = (INPUT_W, INPUT_H),
                 heatmap_size: Tuple[int, int] = (HEATMAP_W, HEATMAP_H)):
        self.frames = frame_list
        self.is_train = is_train
        self.input_w, self.input_h = input_size
        self.hm_w, self.hm_h = heatmap_size
        self.normalize = T.Normalize(mean=IMAGENET_MEAN, std=IMAGENET_STD)

    def __len__(self):
        return len(self.frames)

    def _load_joints_2d(self, json_path: str) -> np.ndarray:
        """Load 2D joints as Nx4 array [u, v, confidence, visible]."""
        with open(json_path) as f:
            data = json.load(f)
        arr = np.zeros((NUM_JOINTS, 4), dtype=np.float32)
        for j in data.get("skeleton_2d", []):
            jid = j["joint_id"]
            if jid < NUM_JOINTS:
                arr[jid] = [j["u"], j["v"], j["confidence"],
                            1.0 if j["visible"] else 0.0]
        return arr

    def __getitem__(self, idx: int):
        info = self.frames[idx]
        img = cv2.imread(info["image_path"])
        if img is None:
            # Return zeros if image can't be loaded
            return (torch.zeros(3, self.input_h, self.input_w),
                    torch.zeros(NUM_JOINTS, self.hm_h, self.hm_w),
                    torch.zeros(NUM_JOINTS))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        joints_2d = self._load_joints_2d(info["json_path"])

        # Bounding box from visible joints
        cx, cy, bw, bh = bbox_from_joints(joints_2d)

        # Augmentation params
        rot, scale = 0.0, 1.0
        if self.is_train:
            rot = random.uniform(-30, 30)
            scale = random.uniform(0.75, 1.25)

        # Affine: image bbox -> input crop
        M = get_affine_transform(cx, cy, bw, bh,
                                 (self.input_w, self.input_h),
                                 rot=rot, scale=scale)
        crop = cv2.warpAffine(img, M, (self.input_w, self.input_h),
                              flags=cv2.INTER_LINEAR)

        # Transform joint coords to crop space
        vis_mask = joints_2d[:, 3] > 0
        pts_orig = joints_2d[:, :2].copy()
        pts_crop = affine_transform_pts(pts_orig, M)

        # Scale to heatmap coords
        pts_hm = pts_crop.copy()
        pts_hm[:, 0] *= self.hm_w / self.input_w
        pts_hm[:, 1] *= self.hm_h / self.input_h

        # Build joint array for heatmap generation
        joints_hm = np.zeros((NUM_JOINTS, 4), dtype=np.float32)
        joints_hm[:, 0] = pts_hm[:, 0]
        joints_hm[:, 1] = pts_hm[:, 1]
        joints_hm[:, 2] = joints_2d[:, 2]  # confidence
        joints_hm[:, 3] = joints_2d[:, 3]  # visible

        # Mark joints outside heatmap as invisible
        for j in range(NUM_JOINTS):
            if (pts_hm[j, 0] < 0 or pts_hm[j, 0] >= self.hm_w or
                    pts_hm[j, 1] < 0 or pts_hm[j, 1] >= self.hm_h):
                joints_hm[j, 3] = 0.0

        # Horizontal flip augmentation
        if self.is_train and random.random() < 0.5:
            crop = crop[:, ::-1, :].copy()
            # Flip x coords in heatmap space
            joints_hm[:, 0] = self.hm_w - 1 - joints_hm[:, 0]
            # Swap left-right pairs
            for l, r in FLIP_PAIRS:
                joints_hm[l], joints_hm[r] = joints_hm[r].copy(), joints_hm[l].copy()

        # Color jitter (train only)
        if self.is_train:
            crop = self._color_jitter(crop)

        # Generate heatmaps + weights
        heatmaps, weights = generate_heatmaps(joints_hm, (self.hm_w, self.hm_h))

        # To tensor + normalize
        crop_t = torch.from_numpy(crop.astype(np.float32) / 255.0).permute(2, 0, 1)
        crop_t = self.normalize(crop_t)

        return crop_t, torch.from_numpy(heatmaps), torch.from_numpy(weights)

    @staticmethod
    def _color_jitter(img: np.ndarray) -> np.ndarray:
        """Simple color jitter in numpy (brightness, contrast, saturation)."""
        img = img.astype(np.float32)
        # Brightness
        img += random.uniform(-0.3, 0.3) * 255
        # Contrast
        img = img * random.uniform(0.7, 1.3)
        # Saturation
        gray = np.mean(img, axis=2, keepdims=True)
        sat = random.uniform(0.7, 1.3)
        img = gray + sat * (img - gray)
        return np.clip(img, 0, 255).astype(np.uint8)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

class SimpleBaselineModel(nn.Module):
    """SimpleBaseline pose estimator: ResNet-50 backbone + deconv head."""

    def __init__(self, num_joints: int = NUM_JOINTS, pretrained: bool = True):
        super().__init__()
        resnet = models.resnet50(weights=models.ResNet50_Weights.DEFAULT if pretrained else None)

        # Backbone: everything up to (but not including) avgpool/fc
        self.backbone = nn.Sequential(
            resnet.conv1, resnet.bn1, resnet.relu, resnet.maxpool,
            resnet.layer1, resnet.layer2, resnet.layer3, resnet.layer4,
        )

        # Deconv head: 2048 -> 256 -> 256 -> 256
        self.head = nn.Sequential(
            nn.ConvTranspose2d(2048, 256, kernel_size=4, stride=2, padding=1, bias=False),
            nn.BatchNorm2d(256), nn.ReLU(inplace=True),
            nn.ConvTranspose2d(256, 256, kernel_size=4, stride=2, padding=1, bias=False),
            nn.BatchNorm2d(256), nn.ReLU(inplace=True),
            nn.ConvTranspose2d(256, 256, kernel_size=4, stride=2, padding=1, bias=False),
            nn.BatchNorm2d(256), nn.ReLU(inplace=True),
        )

        # Final 1x1 conv -> heatmaps
        self.final = nn.Conv2d(256, num_joints, kernel_size=1)

        # Initialize head weights
        for m in self.head.modules():
            if isinstance(m, nn.ConvTranspose2d):
                nn.init.kaiming_normal_(m.weight, mode='fan_out', nonlinearity='relu')
            elif isinstance(m, nn.BatchNorm2d):
                nn.init.constant_(m.weight, 1)
                nn.init.constant_(m.bias, 0)
        nn.init.normal_(self.final.weight, std=0.001)
        nn.init.constant_(self.final.bias, 0)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """Input: (B, 3, 256, 192) -> Output: (B, num_joints, 64, 48)"""
        feat = self.backbone(x)    # (B, 2048, 8, 6)
        feat = self.head(feat)     # (B, 256, 64, 48)
        out = self.final(feat)     # (B, num_joints, 64, 48)
        return out


# ---------------------------------------------------------------------------
# Loss + evaluation
# ---------------------------------------------------------------------------

def weighted_mse_loss(pred: torch.Tensor, target: torch.Tensor,
                      weights: torch.Tensor) -> torch.Tensor:
    """Weighted MSE loss on heatmaps.

    Args:
        pred: (B, J, H, W)
        target: (B, J, H, W)
        weights: (B, J) per-joint weights
    """
    diff = (pred - target) ** 2  # (B, J, H, W)
    # Average over spatial dims
    loss_per_joint = diff.mean(dim=(2, 3))  # (B, J)
    # Apply per-joint weights
    weighted = loss_per_joint * weights
    # Mean over batch and joints (only counting non-zero weights)
    n_active = (weights > 0).float().sum().clamp(min=1)
    return weighted.sum() / n_active


def compute_pck(pred_heatmaps: np.ndarray, target_heatmaps: np.ndarray,
                weights: np.ndarray, threshold: float = 5.0) -> Tuple[float, int]:
    """Compute PCK@threshold in heatmap pixel space.

    Returns:
        (pck_value, n_evaluated)
    """
    B, J, H, W = pred_heatmaps.shape
    correct = 0
    total = 0

    for b in range(B):
        for j in range(J):
            if weights[b, j] < 0.5:
                continue
            # Decode peaks
            pred_hm = pred_heatmaps[b, j]
            tgt_hm = target_heatmaps[b, j]

            pred_idx = pred_hm.argmax()
            pred_y, pred_x = divmod(int(pred_idx), W)

            tgt_idx = tgt_hm.argmax()
            tgt_y, tgt_x = divmod(int(tgt_idx), W)

            dist = math.sqrt((pred_x - tgt_x) ** 2 + (pred_y - tgt_y) ** 2)
            if dist <= threshold:
                correct += 1
            total += 1

    if total == 0:
        return 0.0, 0
    return correct / total, total


@torch.no_grad()
def evaluate(model: nn.Module, dataloader: DataLoader,
             device: torch.device) -> Dict[str, float]:
    """Run evaluation, return dict of metrics."""
    model.eval()
    total_loss = 0.0
    total_pck = 0.0
    total_pck_n = 0
    n_batches = 0

    for imgs, targets, weights in tqdm(dataloader, desc="Evaluating",
                                       leave=False, unit="batch"):
        imgs = imgs.to(device)
        targets = targets.to(device)
        weights = weights.to(device)

        preds = model(imgs)
        loss = weighted_mse_loss(preds, targets, weights)
        total_loss += loss.item()
        n_batches += 1

        # PCK
        pck, n = compute_pck(preds.cpu().numpy(), targets.cpu().numpy(),
                             weights.cpu().numpy(), threshold=5.0)
        total_pck += pck * n
        total_pck_n += n

    avg_loss = total_loss / max(n_batches, 1)
    avg_pck = total_pck / max(total_pck_n, 1)

    return {"val_loss": avg_loss, "pck@5": avg_pck}


# ---------------------------------------------------------------------------
# Training
# ---------------------------------------------------------------------------

def train(args):
    """Train the pose estimation model."""
    print(f"Building frame index from {args.dataset_root}...")
    frames = build_frame_index(args.dataset_root, min_visible_joints=3)
    if not frames:
        print("No frames found. Exiting.")
        return

    train_frames, val_frames = split_train_val(frames, val_ratio=0.2)

    train_dataset = EgoPoseDataset(train_frames, is_train=True)
    val_dataset = EgoPoseDataset(val_frames, is_train=False)

    num_workers = min(args.num_workers, os.cpu_count() or 1)
    train_loader = DataLoader(train_dataset, batch_size=args.batch_size,
                              shuffle=True, num_workers=num_workers,
                              pin_memory=True, drop_last=True)
    val_loader = DataLoader(val_dataset, batch_size=args.batch_size,
                            shuffle=False, num_workers=num_workers,
                            pin_memory=True)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}")

    model = SimpleBaselineModel(num_joints=NUM_JOINTS, pretrained=True)
    model = model.to(device)

    # Differential learning rate: backbone lower, head higher
    backbone_params = list(model.backbone.parameters())
    head_params = list(model.head.parameters()) + list(model.final.parameters())
    optimizer = optim.Adam([
        {"params": backbone_params, "lr": args.lr * 0.1},
        {"params": head_params, "lr": args.lr},
    ])
    scheduler = optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=args.epochs)

    start_epoch = 0
    best_val_loss = float("inf")

    # Resume from checkpoint
    if args.resume:
        print(f"Resuming from {args.resume}")
        ckpt = torch.load(args.resume, map_location=device)
        model.load_state_dict(ckpt["model_state_dict"])
        optimizer.load_state_dict(ckpt["optimizer_state_dict"])
        scheduler.load_state_dict(ckpt["scheduler_state_dict"])
        start_epoch = ckpt.get("epoch", 0) + 1
        best_val_loss = ckpt.get("best_val_loss", float("inf"))
        print(f"  Resumed at epoch {start_epoch}, best_val_loss={best_val_loss:.6f}")

    print(f"\nTraining for {args.epochs} epochs (starting from {start_epoch})...")
    print(f"  Train: {len(train_dataset)} samples, Val: {len(val_dataset)} samples")
    print(f"  Batch size: {args.batch_size}, LR: {args.lr}")
    print()

    for epoch in range(start_epoch, args.epochs):
        model.train()
        epoch_loss = 0.0
        n_batches = 0
        t0 = time.time()

        for imgs, targets, weights in tqdm(train_loader,
                                           desc=f"Epoch {epoch+1}/{args.epochs}",
                                           leave=False, unit="batch"):
            imgs = imgs.to(device)
            targets = targets.to(device)
            weights = weights.to(device)

            optimizer.zero_grad()
            preds = model(imgs)
            loss = weighted_mse_loss(preds, targets, weights)
            loss.backward()
            optimizer.step()

            epoch_loss += loss.item()
            n_batches += 1

        scheduler.step()
        avg_train_loss = epoch_loss / max(n_batches, 1)

        # Validate
        metrics = evaluate(model, val_loader, device)
        elapsed = time.time() - t0

        print(f"Epoch {epoch+1:3d}/{args.epochs} | "
              f"train_loss={avg_train_loss:.6f} | "
              f"val_loss={metrics['val_loss']:.6f} | "
              f"PCK@5={metrics['pck@5']:.3f} | "
              f"{elapsed:.1f}s")

        # Save best checkpoint
        if metrics["val_loss"] < best_val_loss:
            best_val_loss = metrics["val_loss"]
            ckpt = {
                "epoch": epoch,
                "model_state_dict": model.state_dict(),
                "optimizer_state_dict": optimizer.state_dict(),
                "scheduler_state_dict": scheduler.state_dict(),
                "best_val_loss": best_val_loss,
                "metrics": metrics,
                "num_joints": NUM_JOINTS,
                "input_size": (INPUT_W, INPUT_H),
                "heatmap_size": (HEATMAP_W, HEATMAP_H),
            }
            torch.save(ckpt, args.output)
            print(f"  -> Saved best model to {args.output} (val_loss={best_val_loss:.6f})")

    print(f"\nTraining complete. Best val_loss={best_val_loss:.6f}")


# ---------------------------------------------------------------------------
# Prediction
# ---------------------------------------------------------------------------

def load_model(model_path: str, device: torch.device) -> nn.Module:
    """Load trained model from checkpoint."""
    ckpt = torch.load(model_path, map_location=device)
    num_joints = ckpt.get("num_joints", NUM_JOINTS)
    model = SimpleBaselineModel(num_joints=num_joints, pretrained=False)
    model.load_state_dict(ckpt["model_state_dict"])
    model = model.to(device)
    model.eval()
    return model


def _predict_one_session(
    model: nn.Module,
    device: torch.device,
    ego_dir: Path,
    out_dir: Path,
) -> int:
    """Run inference on a single ego_dataset directory. Returns frame count."""
    ann_dir = ego_dir / "annotations"
    images_dir = ego_dir / "images"

    if not ann_dir.exists() or not images_dir.exists():
        return 0

    out_dir.mkdir(parents=True, exist_ok=True)
    normalize = T.Normalize(mean=IMAGENET_MEAN, std=IMAGENET_STD)
    json_files = sorted(ann_dir.glob("frame_*.json"))

    for jf in tqdm(json_files, desc="Predicting", leave=False, unit="frame"):
        with open(jf) as f:
            data = json.load(f)

        image_file = data.get("image_file", "")
        image_path = images_dir / image_file
        if not image_path.exists():
            continue

        # Load image
        img = cv2.imread(str(image_path))
        if img is None:
            continue
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # Get bbox from existing 2D annotations
        joints_2d = np.zeros((NUM_JOINTS, 4), dtype=np.float32)
        for j in data.get("skeleton_2d", []):
            jid = j["joint_id"]
            if jid < NUM_JOINTS:
                joints_2d[jid] = [j["u"], j["v"], j["confidence"],
                                  1.0 if j["visible"] else 0.0]

        cx, cy, bw, bh = bbox_from_joints(joints_2d)

        # Affine transform: image -> crop
        M = get_affine_transform(cx, cy, bw, bh, (INPUT_W, INPUT_H))
        M_inv = cv2.invertAffineTransform(M)
        crop = cv2.warpAffine(img_rgb, M, (INPUT_W, INPUT_H), flags=cv2.INTER_LINEAR)

        # To tensor + normalize
        crop_t = torch.from_numpy(crop.astype(np.float32) / 255.0).permute(2, 0, 1)
        crop_t = normalize(crop_t).unsqueeze(0).to(device)

        # Forward pass
        with torch.no_grad():
            heatmaps = model(crop_t)  # (1, 32, 64, 48)

        hm_np = heatmaps[0].cpu().numpy()  # (32, 64, 48)

        # Decode peaks
        peaks = decode_heatmaps(hm_np)  # (32, 3) [x_hm, y_hm, conf]

        # Map heatmap coords -> crop coords -> original image coords
        peaks_crop = peaks[:, :2].copy()
        peaks_crop[:, 0] *= INPUT_W / HEATMAP_W
        peaks_crop[:, 1] *= INPUT_H / HEATMAP_H

        peaks_orig = affine_transform_pts(peaks_crop, M_inv)

        # Build output
        skeleton = []
        for j in range(NUM_JOINTS):
            skeleton.append({
                "joint_id": j,
                "name": JOINT_NAMES[j],
                "u": round(float(peaks_orig[j, 0]), 1),
                "v": round(float(peaks_orig[j, 1]), 1),
                "confidence": round(float(peaks[j, 2]), 4),
            })

        frame_id = data.get("frame_id", 0)
        result = {
            "frame_id": frame_id,
            "image_file": image_file,
            "skeleton_2d_predicted": skeleton,
            "bbox": {
                "cx": round(float(cx), 1),
                "cy": round(float(cy), 1),
                "w": round(float(bw), 1),
                "h": round(float(bh), 1),
            },
        }

        out_path = out_dir / jf.name
        with open(out_path, "w") as f:
            json.dump(result, f, indent=2)

    return len(json_files)


def predict_session(args):
    """Run inference on a single ego_dataset session directory."""
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Loading model from {args.model}...")
    model = load_model(args.model, device)

    ego_dir = Path(args.ego_dataset_dir)
    out_dir = Path(args.output)
    n = _predict_one_session(model, device, ego_dir, out_dir)
    if n == 0:
        print(f"No frames found in {ego_dir}")
    else:
        print(f"Predictions written to {out_dir}/ ({n} frames)")


def predict_all(args):
    """Run inference on all sessions under a dataset root.

    Discovers all ego_dataset/ dirs and writes predictions/ alongside annotations/.
    """
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Loading model from {args.model}...")
    model = load_model(args.model, device)

    root = Path(args.dataset_root)
    ann_dirs = discover_annotations_dirs(str(root))
    if not ann_dirs:
        print(f"No annotation directories found under {root}")
        return

    print(f"Found {len(ann_dirs)} sessions")
    total_frames = 0
    total_sessions = 0

    pbar = tqdm(ann_dirs, desc="Predicting sessions", unit="session")
    for ann_dir in pbar:
        ego_dir = ann_dir.parent  # ego_dataset/
        out_dir = ego_dir / "predictions"

        # Skip if predictions already exist and --resume
        if args.resume and out_dir.exists():
            existing = list(out_dir.glob("frame_*.json"))
            expected = list(ann_dir.glob("frame_*.json"))
            if len(existing) >= len(expected):
                continue

        n = _predict_one_session(model, device, ego_dir, out_dir)
        total_frames += n
        if n > 0:
            total_sessions += 1

        pbar.set_postfix(frames=total_frames, sessions=total_sessions)

    print(f"\nDone: {total_frames} frames across {total_sessions} sessions")


# ---------------------------------------------------------------------------
# Full evaluation with per-joint breakdown
# ---------------------------------------------------------------------------

def eval_full(args):
    """Evaluate model on dataset with per-joint PCK breakdown."""
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Loading model from {args.model}...")
    model = load_model(args.model, device)

    print(f"Building frame index from {args.dataset_root}...")
    frames = build_frame_index(args.dataset_root, min_visible_joints=3)
    if not frames:
        print("No frames found.")
        return

    _, val_frames = split_train_val(frames, val_ratio=0.2)
    val_dataset = EgoPoseDataset(val_frames, is_train=False)
    val_loader = DataLoader(val_dataset, batch_size=args.batch_size,
                            shuffle=False, num_workers=min(args.num_workers, os.cpu_count() or 1),
                            pin_memory=True)

    # Per-joint counters
    joint_correct = np.zeros(NUM_JOINTS, dtype=np.int64)
    joint_total = np.zeros(NUM_JOINTS, dtype=np.int64)
    total_loss = 0.0
    n_batches = 0

    model.eval()
    with torch.no_grad():
        for imgs, targets, weights in val_loader:
            imgs = imgs.to(device)
            targets_d = targets.to(device)
            weights_d = weights.to(device)

            preds = model(imgs)
            loss = weighted_mse_loss(preds, targets_d, weights_d)
            total_loss += loss.item()
            n_batches += 1

            pred_np = preds.cpu().numpy()
            tgt_np = targets.numpy()
            w_np = weights.numpy()
            B, J, H, W = pred_np.shape

            for b in range(B):
                for j in range(J):
                    if w_np[b, j] < 0.5:
                        continue
                    pred_idx = pred_np[b, j].argmax()
                    pred_y, pred_x = divmod(int(pred_idx), W)
                    tgt_idx = tgt_np[b, j].argmax()
                    tgt_y, tgt_x = divmod(int(tgt_idx), W)
                    dist = math.sqrt((pred_x - tgt_x) ** 2 + (pred_y - tgt_y) ** 2)
                    joint_total[j] += 1
                    if dist <= 5.0:
                        joint_correct[j] += 1

    avg_loss = total_loss / max(n_batches, 1)
    overall_pck = joint_correct.sum() / max(joint_total.sum(), 1)

    print(f"\nEvaluation on {len(val_frames)} val frames")
    print(f"  Val loss: {avg_loss:.6f}")
    print(f"  Overall PCK@5: {overall_pck:.3f}")
    print()
    print(f"{'Joint':<25s} {'PCK@5':>8s} {'Count':>8s}")
    print("-" * 43)
    for j in range(NUM_JOINTS):
        if joint_total[j] == 0:
            pck_str = "  N/A"
        else:
            pck_str = f"{joint_correct[j] / joint_total[j]:.3f}"
        print(f"{JOINT_NAMES[j]:<25s} {pck_str:>8s} {joint_total[j]:>8d}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Train/predict with 2D pose estimation model for ego dataset")
    sub = parser.add_subparsers(dest="command", required=True)

    # -- train --
    p_train = sub.add_parser("train", help="Train pose model on annotated ego dataset")
    p_train.add_argument("dataset_root", help="Root dir containing participant/session/ego_dataset/")
    p_train.add_argument("--epochs", type=int, default=100)
    p_train.add_argument("--batch-size", type=int, default=32)
    p_train.add_argument("--lr", type=float, default=1e-3)
    p_train.add_argument("--num-workers", type=int, default=4)
    p_train.add_argument("-o", "--output", default="pose_model.pth",
                         help="Output checkpoint path")
    p_train.add_argument("--resume", default=None, help="Resume from checkpoint")

    # -- predict --
    p_pred = sub.add_parser("predict", help="Run inference on single ego_dataset session")
    p_pred.add_argument("ego_dataset_dir", help="Path to ego_dataset/ directory")
    p_pred.add_argument("-m", "--model", required=True, help="Model checkpoint path")
    p_pred.add_argument("-o", "--output", default="predictions/",
                        help="Output directory for prediction JSONs")

    # -- predict-all --
    p_pall = sub.add_parser("predict-all",
                            help="Run inference on all sessions under dataset root")
    p_pall.add_argument("dataset_root",
                        help="Root dir containing participant/session/ego_dataset/")
    p_pall.add_argument("-m", "--model", required=True, help="Model checkpoint path")
    p_pall.add_argument("--resume", action="store_true",
                        help="Skip sessions that already have predictions/")

    # -- eval --
    p_eval = sub.add_parser("eval", help="Evaluate model with per-joint breakdown")
    p_eval.add_argument("dataset_root", help="Root dir containing participant/session/ego_dataset/")
    p_eval.add_argument("-m", "--model", required=True, help="Model checkpoint path")
    p_eval.add_argument("--batch-size", type=int, default=32)
    p_eval.add_argument("--num-workers", type=int, default=4)

    args = parser.parse_args()

    if args.command == "train":
        train(args)
    elif args.command == "predict":
        predict_session(args)
    elif args.command == "predict-all":
        predict_all(args)
    elif args.command == "eval":
        eval_full(args)


if __name__ == "__main__":
    main()
