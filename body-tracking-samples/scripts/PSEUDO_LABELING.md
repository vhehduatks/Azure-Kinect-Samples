# Pseudo Labeling Pipeline

Auto-extrinsic calibration via 2D pose estimation: train on manually annotated ego frames, predict pseudo labels on unannotated sessions, then fit the extrinsic delta.

## Overview

```
Manual annotations (~40K frames)
        │
        ▼
┌─────────────────────┐
│  train_pose_model.py │  Stage 1: Train heatmap-based 2D pose estimator
│  train               │  (SimpleBaseline: ResNet-50 + 3× deconv head)
└────────┬────────────┘
         │  model.pth
         ▼
┌─────────────────────┐
│  train_pose_model.py │  Generate pseudo labels for all sessions
│  predict-all         │  Writes predictions/ alongside annotations/
└────────┬────────────┘
         │  skeleton_2d_predicted per frame
         ▼
┌─────────────────────┐
│  fit_extrinsic.py    │  Stage 2: Optimize 6DOF extrinsic delta
│  --predictions       │  from 3D (annotation) + 2D (predicted) pairs
└────────┬────────────┘
         │  (rx, ry, rz, tx, ty, tz)
         ▼
┌─────────────────────┐
│  fit_extrinsic.py    │  Apply delta to all annotations
│  --apply             │  (differential 2D projection)
└─────────────────────┘
```

## Step-by-Step

### 1. Train the model

```bash
python train_pose_model.py train <dataset_root> \
    --epochs 100 --batch-size 32 -o model.pth
```

- Input: `<dataset_root>/<participant>/<session>/ego_dataset/annotations/`
- Discovers all annotated frames, splits 80/20 by session
- Trains SimpleBaseline (ResNet-50 backbone, 32-joint heatmaps at 64×48)
- Saves best checkpoint by validation loss

### 2. Generate pseudo labels

```bash
# All sessions at once (writes ego_dataset/predictions/ per session)
python train_pose_model.py predict-all <dataset_root> -m model.pth

# Resume interrupted run
python train_pose_model.py predict-all <dataset_root> -m model.pth --resume

# Single session
python train_pose_model.py predict <session>/ego_dataset -m model.pth -o predictions/
```

Output per frame (`predictions/frame_000000.json`):
```json
{
  "frame_id": 0,
  "image_file": "frame_000000.jpg",
  "skeleton_2d_predicted": [
    {"joint_id": 0, "name": "PELVIS", "u": 823.4, "v": 541.2, "confidence": 0.89}
  ],
  "bbox": {"cx": 960, "cy": 540, "w": 400, "h": 533}
}
```

### 3. Fit extrinsic delta from predictions

```bash
# Dry run (hierarchical, auto-discovers predictions/ dirs)
python fit_extrinsic.py <dataset_root> --predictions

# Dry run (single session)
python fit_extrinsic.py <annotations_dir> --flat --predictions <predictions_dir>

# Apply fitted delta to the same dataset
python fit_extrinsic.py <dataset_root> --predictions --apply

# Apply to a different target dataset
python fit_extrinsic.py <source_root> --predictions --target <target_root> --apply

# Filter by prediction confidence (default 0.5)
python fit_extrinsic.py <dataset_root> --predictions --min-pred-confidence 0.7
```

### 4. Evaluate model quality

```bash
python train_pose_model.py eval <dataset_root> -m model.pth
```

Reports overall PCK@5 and per-joint breakdown on the validation set.

## Directory Layout

After running `predict-all`, each session has:

```
<dataset_root>/
  <participant>/
    <session>/
      ego_dataset/
        images/           # input frames
        annotations/      # original 3D + 2D from offline processor
          frame_000000.json
          frame_000001.json
        predictions/      # pseudo labels from model (auto-created)
          frame_000000.json
          frame_000001.json
```

`fit_extrinsic.py --predictions` auto-discovers `predictions/` as a sibling of `annotations/`.

## Model Architecture

| Component | Detail |
|-----------|--------|
| Backbone | ResNet-50 (ImageNet pretrained), stride 32 |
| Head | 3× deconv (2048→256→256→256, 4×4, stride 2, BN+ReLU) |
| Output | 1×1 conv → 32 heatmaps at 64×48 |
| Input | 256×192 crop (4:3 aspect), ImageNet-normalized |
| Parameters | ~34M |

## Training Details

| Setting | Value |
|---------|-------|
| Loss | Weighted MSE (conf=0→0.0, conf=1→0.5, conf≥2→1.0, invisible→0.0) |
| Optimizer | Adam, backbone LR=1e-4, head LR=1e-3 |
| Scheduler | CosineAnnealingLR |
| Augmentations | Flip (L/R swap), rotation ±30°, scale 0.75–1.25, color jitter |
| Metric | PCK@5px (at heatmap scale ≈ 20px at crop scale) |
| Split | 80/20 by session (no same-session leakage) |

## Bounding Box

The crop bbox is computed from the annotation's existing `skeleton_2d` joints:
1. Tight AABB of visible joints (confidence > 0, visible = True)
2. 25% padding
3. Aspect-lock to 4:3

At prediction time, a single affine transform maps bbox → 256×192 crop. The inverse affine maps heatmap peaks back to original image coordinates. `fit_extrinsic.py` only sees full image-space `(u, v)`.

## Confidence Filtering

`fit_extrinsic.py --min-pred-confidence` (default 0.5) filters which predicted joints become correspondences. Higher thresholds reduce noise at the cost of fewer pairs. The model's confidence is the heatmap peak value (0–1).
