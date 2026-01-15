# Data Synchronization Scripts

Post-processing tools for synchronizing skeleton tracking data with HMD/Controller data.

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

## Related Projects

- [multi_device_body_viewer](../multi_device_body_viewer/) - C++ skeleton viewer with CSV recording
- [sample_unity_bodytracking](../sample_unity_bodytracking/) - Unity body tracking with HMD recording
- [sample_unity_hmd_recorder](../sample_unity_hmd_recorder/) - Standalone HMD/Controller recorder
