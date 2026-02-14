# Data Processing & Visualization Scripts

Post-processing tools for synchronizing skeleton tracking data with HMD/Controller data and visualizing ego-view datasets.

## Overview

When recording body tracking from Orbbec cameras and HMD data from Unity separately, timing offsets exist between the data streams. This script aligns them using **coordinate-system invariant cross-correlation**.

### Key Features

- **Coordinate-system invariant**: Uses inter-point distances instead of raw positions
- **Multi-point correlation**: Combines head + both hands for robust delay estimation
- **Multiple methods**: Distance-based, triangle area, or single-point fallback

## Setup

```bash
pip install -r requirements.txt
```

## Why Inter-Point Distance?

Orbbec cameras and Quest HMD use **different coordinate systems**:

| System | Origin | Orientation |
|--------|--------|-------------|
| Orbbec | Camera position | Camera-relative |
| Quest | Play area center | World-relative |

**Problem**: Raw positions can't be directly compared.

**Solution**: Use distances between body points, which are **invariant** to coordinate system differences.

```
Signal = dist(HEAD, LEFT) + dist(HEAD, RIGHT) + dist(LEFT, RIGHT)
```

This produces the same value regardless of coordinate system!

## Usage

### Basic Usage (Recommended)

```bash
# Inter-point distance method (coordinate-invariant)
python sync_skeleton_hmd.py --skeleton skeleton.csv --hmd hmd.csv --method distance
```

### Alternative Methods

```bash
# Triangle area method (also coordinate-invariant)
python sync_skeleton_hmd.py --skeleton skeleton.csv --hmd hmd.csv --method area

# Head Y position only (legacy, NOT coordinate-invariant)
python sync_skeleton_hmd.py --skeleton skeleton.csv --hmd hmd.csv --method head_y
```

### With Visualization

```bash
python sync_skeleton_hmd.py --skeleton skeleton.csv --hmd hmd.csv --plot
python sync_skeleton_hmd.py --skeleton skeleton.csv --hmd hmd.csv --plot-output sync_plot.png
```

### Manual Delay Override

```bash
python sync_skeleton_hmd.py --skeleton skeleton.csv --hmd hmd.csv --delay 150
```

## Options

| Option | Default | Description |
|--------|---------|-------------|
| `--skeleton` | (required) | Skeleton CSV file path |
| `--hmd` | (required) | HMD CSV file path |
| `--output` | `synced_data.csv` | Output CSV file path |
| `--method` | `distance` | Correlation method: `distance`, `area`, `head_y` |
| `--delay` | (auto) | Manual delay in ms (skips auto-estimation) |
| `--tolerance` | `50.0` | Timestamp matching tolerance in ms |
| `--resample` | `10.0` | Resampling interval in ms |
| `--smooth` | `3.0` | Gaussian smoothing sigma (samples) |
| `--plot` | off | Show visualization plot |
| `--plot-output` | (none) | Save plot to file |

## Correlation Methods

| Method | Option | Coordinate Invariant | Best For |
|--------|--------|---------------------|----------|
| **Inter-point Distance** | `distance` | Yes | General use (recommended) |
| **Triangle Area** | `area` | Yes | Arm spread movements |
| **Head Y** | `head_y` | No | Same coordinate system only |

## CSV Formats

### Skeleton Data (Wide format from multi_device_body_viewer) - Recommended

```csv
timestamp_ms,device_index,body_id,J0_x,J0_y,J0_z,J0_conf,J1_x,J1_y,J1_z,J1_conf,...,J31_x,J31_y,J31_z,J31_conf
1704067200000,0,100,0.12,0.45,0.78,3,0.13,0.48,0.79,3,...
```

| Column | Description |
|--------|-------------|
| `timestamp_ms` | Epoch timestamp in milliseconds |
| `device_index` | Camera index (0, 1, 2, ...) |
| `body_id` | Body ID |
| `J{n}_x/y/z` | Joint n position (mm) |
| `J{n}_conf` | Joint n confidence (0=NONE, 1=LOW, 2=MEDIUM, 3=HIGH) |

### Skeleton Data (Wide format from Unity)

```csv
Frame,Time,P0_posX,P0_posY,P0_posZ,P1_posX,...,Timestamp
0,0.0,0.12,0.45,0.78,0.13,...,01-15 10:30:00.123
```

### HMD Data (from Unity HMDDataRecorder)

```csv
timestamp_ms,frame,unity_time,hmd_pos_x,hmd_pos_y,hmd_pos_z,...,left_pos_x,...,right_pos_x,...
1704067200000,0,0.0,0.1,1.5,0.2,...,0.3,...,0.4,...
```

## Output Example

```
Estimating delay using method: distance
  Skeleton: 1800 frames, 0 - 60000 ms
  HMD: 1800 frames, 0 - 60000 ms
  Using inter-point distance (coordinate-invariant)
  Overlapping range: 0 - 60000 ms (60.0 sec)
  Resampled to 6000 points at 10.0ms interval
  Estimated delay: 45.0 ms (skeleton leads HMD)
  Peak correlation: 0.8234

SUMMARY
==================================================
  Method: distance
  Delay: 45.0 ms
  Correlation: 0.8234
  Output: synced_data.csv
```

## Recording Workflow

1. **Start multi_device_body_viewer** (C++ app)
   ```bash
   multi_device_body_viewer.exe --primary CL3FC3100HN --calibration calibration.json
   ```

2. **Start Unity** with HMDDataRecorder enabled

3. **Press R** in both applications to start recording

4. **Perform movements** (move arms for best correlation signal)

5. **Press R** in both to stop recording

6. **Run synchronization**
   ```bash
   python sync_skeleton_hmd.py --skeleton skeleton_data_*.csv --hmd HMD_*.csv --method distance
   ```

## Algorithm

1. **Extract points**: HEAD, LEFT_WRIST, RIGHT_WRIST from skeleton; HMD, L_CTRL, R_CTRL from Quest
2. **Compute signal**: Sum of inter-point distances (coordinate-invariant)
3. **Resample**: Interpolate to common 10ms time grid
4. **Smooth**: Gaussian filter (sigma=3) to reduce noise
5. **Normalize**: Zero mean, unit variance
6. **Cross-correlate**: Find peak lag using `scipy.signal.correlate`
7. **Output**: Delay in milliseconds

---

## Ego-View Dataset Visualizer

Visualizes the ego-view dataset output from [multi_device_offline_processor](../multi_device_offline_processor/) — overlays 2D skeleton joints onto helmet camera images and shows 3D joints in a separate figure.

### Quick Start

```bash
# Interactive preview (side-by-side 2D overlay + 3D skeleton)
python visualize_ego_dataset.py --input ego_dataset/

# 2D overlay only
python visualize_ego_dataset.py --input ego_dataset/ --view 2d

# 3D skeleton only
python visualize_ego_dataset.py --input ego_dataset/ --view 3d
```

### Output Modes

```bash
# Save overlay images to a directory
python visualize_ego_dataset.py --input ego_dataset/ --output overlays/ --mode images

# Save video (2D overlay, fast OpenCV export)
python visualize_ego_dataset.py --input ego_dataset/ --output overlay.mp4 --mode video --view 2d

# Save video (side-by-side 2D + 3D via matplotlib)
python visualize_ego_dataset.py --input ego_dataset/ --output combined.mp4 --mode video --view both
```

### Options

| Option | Default | Description |
|--------|---------|-------------|
| `--input`, `-i` | (required) | Path to `ego_dataset/` directory |
| `--output`, `-o` | (auto) | Output path (directory for images, file for video) |
| `--mode` | `preview` | `preview`, `images`, or `video` |
| `--view` | `both` | `2d` (overlay only), `3d` (skeleton only), or `both` (side-by-side) |
| `--fps` | `30` | Video frame rate |
| `--min-confidence` | `1` | Minimum joint confidence to display (0-3) |

### Interactive Preview Controls

| Key | Action |
|-----|--------|
| Left / Right | Previous / Next frame |
| PgUp / PgDown | Skip 10 frames |
| Home / End | First / Last frame |
| Slider | Jump to any frame |

### What It Shows

**2D Overlay** — Skeleton bones and joints drawn on the helmet camera image, color-coded by body part:
- Green: spine (pelvis → neck)
- Red: head/face
- Blue: left arm
- Orange: right arm
- Purple: left leg
- Teal: right leg

Joint circle size scales with confidence level. A text overlay shows frame number, timestamp, and whether the checkerboard was detected.

**3D Skeleton** — Interactive matplotlib 3D plot of all 32 joints in the helmet camera coordinate frame (mm). Same color coding. Axes auto-scale to the skeleton bounding box.

### Expected Input

The `ego_dataset/` directory produced by `multi_device_offline_processor --helmet-serial ...`:

```
ego_dataset/
├── images/
│   ├── frame_000000.jpg
│   └── ...
├── annotations/
│   ├── frame_000000.json   ← per-frame: camera_pose, skeleton_3d, skeleton_2d
│   └── ...
└── metadata.json
```

See the [multi_device_offline_processor README](../multi_device_offline_processor/README.md#ego-view-output) for the full JSON schema.

---

## Batch Ego Dataset Generator

Automates running `multi_device_offline_processor.exe` and `sync_skeleton_hmd.py` for all recorded sessions in a directory. Instead of manually invoking ~15 CLI arguments per pose, this script discovers all sessions, groups MKV files by timestamp, and processes them sequentially.

### Input Structure

The input directory should contain MKV recordings and HMD CSVs with matching timestamps:

```
Test/
  recording_cam0_CL8T75400DC_Dancing1_20260214_001511.mkv
  recording_cam1_CL8T75400GD_Dancing1_20260214_001511.mkv   (helmet)
  recording_cam2_CL8T75400KV_Dancing1_20260214_001511.mkv
  recording_cam3_CL8T75400CB_Dancing1_20260214_001511.mkv
  HMD_Test_Dancing1_20260214_001511.csv
  ...
```

Sessions are grouped by matching `{Pose}_{Date}_{Time}`. Camera role is determined by serial number (not cam index).

### Output Structure

```
batch_out/
  Dancing1_20260214_001511/
    ego_dataset/          # images/ + annotations/ + metadata.json
    output.csv            # fused skeleton CSV
    synced_data.csv       # synchronized skeleton + HMD
    processor.log         # stdout/stderr from processor
  Gaming-Boxing_20260214_000909/
    ...
  batch_summary.json      # overall results
```

### Quick Start

```bash
# 1. Dry run — list all discovered sessions without processing
python batch_ego_dataset.py \
    --input-dir Test/ \
    --output-dir batch_out/ \
    --dry-run

# 2. Process all sessions
python batch_ego_dataset.py \
    --input-dir Test/ \
    --output-dir batch_out/

# 3. Resume after interruption (skip completed sessions)
python batch_ego_dataset.py \
    --input-dir Test/ \
    --output-dir batch_out/ \
    --resume
```

### Options

| Option | Default | Description |
|--------|---------|-------------|
| `--input-dir` | (required) | Directory with MKV recordings and HMD CSVs |
| `--output-dir` | (required) | Output root directory |
| `--processor-exe` | (auto-detect) | Path to `multi_device_offline_processor.exe` |
| `--calib-dir` | (same as exe) | Directory with `calibration.json` and `T_checker_to_A.json` |
| `--sensor-orientation` | `ccw90` | Sensor orientation: `default`, `cw90`, `ccw90`, `flip180` |
| `--smoothing` | `0.5` | Temporal smoothing factor (0.0–1.0) |
| `--dry-run` | off | List sessions without processing |
| `--resume` | off | Skip sessions with existing `ego_dataset/metadata.json` |

### Camera Configuration

| Serial | Role |
|--------|------|
| `CL8T75400DC` | Primary (reference camera, identity transform) |
| `CL8T75400GD` | Helmet (head-mounted, ego-view source) |
| `CL8T75400KV` | Secondary (fixed) |
| `CL8T75400CB` | Secondary (fixed) |

### Batch Summary

After processing, `batch_summary.json` contains:

```json
{
  "total": 19,
  "succeeded": 17,
  "failed": 2,
  "skipped": 0,
  "sessions": [
    {"name": "Dancing1_20260214_001511", "status": "success", "time_sec": 120.5},
    {"name": "...", "status": "failed", "error": "..."}
  ]
}
```

---

## Related Projects

- [multi_device_body_viewer](../multi_device_body_viewer/) - C++ skeleton viewer with CSV recording
- [multi_device_offline_processor](../multi_device_offline_processor/) - Offline body tracking with ego-view dataset generation
- [sample_unity_bodytracking](../sample_unity_bodytracking/) - Unity body tracking with HMD recording
- [sample_unity_hmd_recorder](../sample_unity_hmd_recorder/) - Standalone HMD/Controller recorder
