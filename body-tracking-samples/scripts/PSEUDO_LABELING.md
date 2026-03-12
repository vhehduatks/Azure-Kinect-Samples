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
│  fit_extrinsic.py    │  Stage 2: Fit extrinsic delta
│  --predictions       │  Global, per-session, or per-frame
└────────┬────────────┘
         │  6DOF delta(s)
         ▼
┌─────────────────────┐
│  fit_extrinsic.py    │  Apply delta to all annotations
│  --apply             │  (differential 2D projection + visible fix)
└────────┬────────────┘
         │  updated u,v + visible flag
         ▼
┌─────────────────────┐
│  blend_2d_annota... │  Stage 3 (optional): Blend projected 2D
│  --mode blend       │  with model predictions (α = pred_conf)
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

### 3. Fit extrinsic delta

Three fitting granularities, from coarsest to finest:

#### Global (one delta for entire dataset)

```bash
python fit_extrinsic.py <dataset_root> --predictions
python fit_extrinsic.py <dataset_root> --predictions --apply
```

Best when the helmet camera mount is rigid and consistent across sessions.

#### Per-session (one delta per recording session)

```bash
python fit_extrinsic.py <dataset_root> --predictions --per-session
python fit_extrinsic.py <dataset_root> --predictions --per-session --apply
```

Accounts for helmet re-wearing between sessions. Each session's ~40 frames
average out per-frame noise while capturing the session-specific offset.
Reports mean/std/range across sessions to show how much the mount varies.

#### Per-frame (session fit + regularized per-frame refinement)

```bash
python fit_extrinsic.py <dataset_root> --predictions --per-frame
python fit_extrinsic.py <dataset_root> --predictions --per-frame --apply

# Tighter regularization (frames stay closer to session average)
python fit_extrinsic.py <dataset_root> --predictions --per-frame --frame-reg 10.0 --apply

# Looser regularization (more per-frame freedom)
python fit_extrinsic.py <dataset_root> --predictions --per-frame --frame-reg 2.0 --apply
```

Two-pass approach:
1. Fit per-session delta (same as `--per-session`)
2. For each frame, re-fit with L2 regularization pulling toward the session delta

This handles helmet loosening/shifting within a session while preventing
overfitting to per-frame prediction noise. Frames with fewer than 4
correspondences skip refinement and use the session delta directly.

**Regularization weight** (`--frame-reg`, default 5.0):
- Controls how tightly per-frame deltas stay near the session average
- Scale: 1 deg rotation ≈ 10mm translation in cost
- Higher = smoother (less per-frame variation), lower = more responsive
- Typical range: 2.0 (loose) to 15.0 (tight)

#### Other options

```bash
# Single session (flat mode)
python fit_extrinsic.py <annotations_dir> --flat --predictions <predictions_dir>

# Apply to a different target dataset
python fit_extrinsic.py <source_root> --predictions --target <target_root> --apply

# Apply with 3D joint transform
python fit_extrinsic.py <dataset_root> --predictions --per-frame --apply --3d

# Filter by prediction confidence (default 0.5)
python fit_extrinsic.py <dataset_root> --predictions --min-pred-confidence 0.7

# Disable body-part balancing weights (equal weight for all joints)
python fit_extrinsic.py <dataset_root> --predictions --no-joint-weights
```

### 4. Evaluate model quality

```bash
python train_pose_model.py eval <dataset_root> -m model.pth
```

Reports overall PCK@5 and per-joint breakdown on the validation set.

### 5. Visualize predictions

```bash
# Interactive overlay (annotation=muted, prediction=bright)
python visualize_predictions.py <dataset_root>

# Side-by-side comparison
python visualize_predictions.py <dataset_root> --mode side-by-side

# Per-session error summary table
python visualize_predictions.py <dataset_root> --mode summary

# Export comparison images to disk
python visualize_predictions.py <dataset_root> --mode export -o comparison/
```

Keyboard: arrows=frame, PgUp/PgDn=skip 10, N/P=next/prev session.

### 6. Blend 2D annotations (optional)

After `--apply` updates projected u,v, `blend_2d_annotations.py` can blend
the projection-based 2D with model-predicted 2D to produce the final
`skeleton_2d` ground truth.

```bash
# Vis-fix only (recalculate visible flags, no predictions needed)
python blend_2d_annotations.py --dataset-root <root> --mode vis-fix

# Replace with model predictions
python blend_2d_annotations.py --dataset-root <root> --mode prediction

# Weighted blend: u = (1-α)*u_proj + α*u_pred, α = pred_confidence
python blend_2d_annotations.py --dataset-root <root> --mode blend

# Single session
python blend_2d_annotations.py --session-dir <ego_dataset_dir> --mode blend

# Dry run (report changes without writing)
python blend_2d_annotations.py --dataset-root <root> --mode blend --dry-run

# Higher minimum prediction confidence threshold
python blend_2d_annotations.py --dataset-root <root> --mode blend --min-pred-confidence 0.3
```

| Mode | skeleton_2d source | Predictions needed | When to use |
|------|-------------------|--------------------|-------------|
| `vis-fix` | Keep current u,v (only recalculates visible) | No | Standalone visibility repair |
| `projection` | Keep current u,v (recalculates visible) | Yes (dir must exist) | Vis-fix when predictions are present |
| `prediction` | Replace u,v with model predictions | Yes | Model more trusted than projection |
| `blend` | `(1-α)*proj + α*pred`, α = pred_confidence | Yes | Best of both (recommended) |

For joints with no prediction or confidence below `--min-pred-confidence` (default 0.1),
the projection value is kept (α=0).

## Visible Flag Fix

`fit_extrinsic.py --apply`, `apply_per_frame_deltas_to_dir()`, and `apply_avg_offset.py`
all recalculate the `visible` flag after updating u,v coordinates:

- **Behind camera**: `new_pt[2] <= 0` → `visible = False`
- **Out of frame**: `u < 0` or `u >= width` or `v < 0` or `v >= height` → `visible = False`
- **Image dims**: read from `camera_intrinsics.width/height`, falling back to `2*cx / 2*cy`

This prevents joints that moved out of frame from remaining `visible=True`, which
previously corrupted training (visible=False → zero weight in loss function).

## Fitting Modes Comparison

| Mode | Flag | Delta count | Best for |
|------|------|-------------|----------|
| Global | *(default)* | 1 total | Rigid helmet mount, consistent across sessions |
| Per-session | `--per-session` | 1 per session | Helmet re-wearing between sessions |
| Per-frame | `--per-frame` | 1 per frame | Helmet shifting within sessions (e.g. active movement) |

Per-frame implies per-session (session delta is computed first as the prior).

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

## Joint Exclusion

Distal hand joints (HAND, HANDTIP, THUMB — IDs 8-10, 15-17) are excluded by default
because annotation only extends to the wrist. These joints have unreliable 3D tracking
and noisy predictions that corrupt the extrinsic fit — especially inflating tz (forward
stretch to align lower body at the expense of upper body).

```bash
# Default: exclude hand joints
python fit_extrinsic.py <dataset_root> --predictions

# Include all joints (not recommended)
python fit_extrinsic.py <dataset_root> --predictions --exclude-joints none

# Custom exclusion (e.g. also exclude head/face)
python fit_extrinsic.py <dataset_root> --predictions --exclude-joints 8,9,10,15,16,17,26,27,28,29,30,31
```

## Body-Part Balancing (Joint Weights)

In ego-view, the lower body (pelvis, hips, knees, ankles, feet) is visible in nearly every frame, while upper body joints (shoulders, elbows, wrists) frequently extend off-screen. Without correction, the optimizer minimizes total residual, effectively fitting to the lower body and under-fitting the upper body. The symptom is large tz values (>100mm) — the optimizer stretches the skeleton forward to align the always-visible lower body.

**Solution**: Inverse-frequency weighting per body part (enabled by default, disable with `--no-joint-weights`).

### Algorithm

1. Group joints into 6 body parts (hand joints excluded from arm groups)
2. Count correspondences per body part
3. Target count = total / num_parts_present (equal share)
4. Weight for joints in part P = target / count(P)
5. Normalize so mean weight = 1.0
6. Scale each correspondence's residual by sqrt(weight) → least_squares minimizes weighted sum of squares

### Body Part Groups

| Part | Joint IDs | Notes |
|------|-----------|-------|
| spine | 0–3 (PELVIS → NECK) | |
| head | 26–31 (HEAD → EAR_RIGHT) | |
| left_arm | 4–7 (CLAVICLE_LEFT → WRIST_LEFT) | Hand joints excluded |
| right_arm | 11–14 (CLAVICLE_RIGHT → WRIST_RIGHT) | Hand joints excluded |
| left_leg | 18–21 (HIP_LEFT → FOOT_LEFT) | |
| right_leg | 22–25 (HIP_RIGHT → FOOT_RIGHT) | |

### Example output

```
Body-part distribution:
  spine          1800 ( 25.0%)  weight=0.52
  head            300 (  4.2%)  weight=3.12
  left_arm        500 (  6.9%)  weight=1.87
  right_arm       520 (  7.2%)  weight=1.80
  left_leg       2040 ( 28.3%)  weight=0.46
  right_leg      2040 ( 28.3%)  weight=0.46
  Joint balancing: ENABLED
```

Arms and head (rarely visible) get higher weight; legs (almost always visible) get lower weight. The optimizer treats all body parts as equally important.

## Optimization Log

By default, `fit_extrinsic.py` auto-saves a JSON log after each run:

```bash
# Auto-save to <source>/fit_extrinsic_log_<timestamp>.json
python fit_extrinsic.py <dataset_root> --predictions --per-session

# Custom log path
python fit_extrinsic.py <dataset_root> --predictions --log results/my_run.json

# Disable logging
python fit_extrinsic.py <dataset_root> --predictions --no-log
```

Log contents:
- **command**: all CLI arguments used
- **intrinsics**: estimated fx, fy, cx, cy
- **joint_detection**: per-joint count, body part, and applied weight
- **body_part_distribution**: per-part count, percentage, and weight
- **sessions**: per-session 6DOF delta, RMS before/after, n_pairs
- **summary**: mean/std/min/max across sessions for each parameter

## Validation

`validate_fit.py` detects outlier sessions and compares projected 3D joints against model predictions.

```bash
# Basic outlier detection from log file
python validate_fit.py <dataset_root> --log fit_extrinsic_log_xxx.json

# Custom thresholds (tz-specific, tighter IQR)
python validate_fit.py <dataset_root> --log fit.json --tz-max 120 --iqr-factor 1.0

# Include projection vs prediction comparison
python validate_fit.py <dataset_root> --log fit.json --predictions

# Export flagged frames to CSV
python validate_fit.py <dataset_root> --log fit.json --predictions -o flagged.csv

# Full JSON report
python validate_fit.py <dataset_root> --log fit.json --predictions --json report.json
```

### Delta outlier detection

Two methods applied per session:
- **Absolute threshold**: flag if rotation > 5° or translation > 150mm (configurable)
- **IQR-based**: flag if parameter outside [Q1 − 1.5×IQR, Q3 + 1.5×IQR]

### Projection vs prediction comparison (`--predictions`)

For each frame with both annotation (3D) and prediction (2D) data:
1. Project 3D skeleton using the session's fitted delta: `u' = proj(R @ P3d + t)`
2. Compare against model's predicted 2D positions per joint
3. Flag frames where mean distance > 30px or max distance > 90px (configurable)

Output shows worst joints per flagged frame, grouped by session.

## Per-Frame Regularization Details

The per-frame residual function appends 6 regularization terms to the standard reprojection residuals:

```
residuals = [reproj_u_0, reproj_v_0, ..., reproj_u_N, reproj_v_N,
             w * (rx - rx_session),
             w * (ry - ry_session),
             w * (rz - rz_session),
             w * 0.1 * (tx - tx_session),
             w * 0.1 * (ty - ty_session),
             w * 0.1 * (tz - tz_session)]
```

The 0.1 scale on translation normalizes units so that 1 deg ≈ 10mm in cost.
With `--frame-reg 5.0` and ~20 joints per frame:
- A 0.5° deviation costs roughly the same as 2–3px average reprojection improvement
- A 5mm deviation costs the same
- The optimizer only deviates from the session mean when there's strong evidence in the frame data
