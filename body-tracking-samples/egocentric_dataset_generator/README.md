# Egocentric Body Tracking Dataset Generator

Generates ML training datasets by combining first-person view footage from a helmet-mounted camera (A) with skeletons estimated from fixed cameras (B, C).

## Overview

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│ Camera A    │     │ Camera B    │     │ Camera C    │
│ (Helmet)    │     │ (Fixed)     │     │ (Fixed)     │
│ RGB+Depth   │     │ Checkerboard│     │ Checkerboard│
└─────────────┘     │ Detection   │     │ Detection   │
       │            └──────┬──────┘     └──────┬──────┘
       │                   │                   │
       │            ┌──────┴───────────────────┘
       │            │
       ▼            ▼
┌─────────────┐  ┌─────────────┐
│ Color Image │  │ Helmet Pose │
│ (Output)    │  │ Estimation  │
└─────────────┘  └──────┬──────┘
                        │
                        ▼
                 ┌─────────────┐     ┌─────────────┐
                 │ skeleton.csv│────►│ Transform   │
                 │ (World Coord)     │ to Camera A │
                 └─────────────┘     └──────┬──────┘
                                            │
                                            ▼
                                     ┌─────────────┐
                                     │ 3D/2D JSON  │
                                     │ Annotations │
                                     └─────────────┘
```

## Input Files

| File | Description |
|------|-------------|
| `camera_a.mkv` | Helmet camera footage (color + depth + IR) |
| `camera_b.mkv` | Fixed camera B footage (for checkerboard detection) |
| `camera_c.mkv` | Fixed camera C footage (optional, for checkerboard detection) |
| `skeleton.csv` | multi_device_offline_processor output (world coordinates) |
| `calibration.json` | B, C camera extrinsics (multi_device_calibration output) |
| `t_checker_to_a.json` | Checkerboard-to-Camera A fixed transform (pre-measured) |

## Output Structure

```
output/
├── images/
│   ├── frame_000000.jpg
│   ├── frame_000001.jpg
│   └── ...
├── annotations/
│   ├── frame_000000.json
│   ├── frame_000001.json
│   └── ...
└── metadata.json
```

### Annotation JSON Format

```json
{
  "frame_id": 0,
  "timestamp_usec": 1234567890,
  "image_file": "frame_000000.jpg",
  "camera_pose": {
    "R": [[r11,r12,r13], [r21,r22,r23], [r31,r32,r33]],
    "t": [tx, ty, tz]
  },
  "skeleton_3d": [
    {"joint_id": 0, "name": "PELVIS", "x": 100.0, "y": -500.0, "z": 1500.0, "confidence": 2},
    ...
  ],
  "skeleton_2d": [
    {"joint_id": 0, "name": "PELVIS", "u": 640.0, "v": 360.0, "confidence": 2, "visible": true},
    ...
  ],
  "checkerboard_detected": true,
  "checkerboard_cameras": ["B", "C"]
}
```

## Build

1. Open `egocentric_dataset_generator.vcxproj` in Visual Studio
2. Restore NuGet packages (right-click solution → Restore NuGet Packages)
3. Build as Release x64
4. Run `copy_orbbec_dlls.bat` to copy Orbbec DLLs and OpenCV DLLs

## Usage

```bash
egocentric_dataset_generator.exe ^
  --camera-a helmet.mkv ^
  --camera-b fixed_b.mkv ^
  --camera-c fixed_c.mkv ^
  --skeleton skeleton.csv ^
  --calibration calibration.json ^
  --t-checker-to-a t_checker_a.json ^
  --output ./dataset ^
  --checkerboard-rows 6 ^
  --checkerboard-cols 9
```

### Command-Line Options

| Option | Required | Description |
|--------|----------|-------------|
| `--camera-a FILE` | Y | Helmet camera MKV file |
| `--camera-b FILE` | Y | Fixed camera B MKV file |
| `--camera-c FILE` | - | Fixed camera C MKV file (optional) |
| `--skeleton FILE` | Y | Skeleton CSV file |
| `--calibration FILE` | Y | Calibration JSON file |
| `--t-checker-to-a FILE` | Y | Checkerboard-to-Camera A transform JSON |
| `--output DIR` | Y | Output directory |
| `--checkerboard-rows N` | - | Number of checkerboard inner corner rows (default: 6) |
| `--checkerboard-cols N` | - | Number of checkerboard inner corner columns (default: 9) |
| `--max-frames N` | - | Maximum number of frames to process (default: all) |
| `--skip-no-detection` | - | Skip frames where checkerboard is not detected |

## How to Measure T_checker_to_A

The fixed transform between the checkerboard and helmet camera A must be measured in advance.

### Method 1: Direct Measurement
1. Measure the distance between the checkerboard center and camera A lens with a ruler
2. Measure the angle between the checkerboard plane and the camera optical axis
3. Save as a JSON file:

```json
{
  "rotation": [[r11,r12,r13], [r21,r22,r23], [r31,r32,r33]],
  "translation": [tx, ty, tz]
}
```

### Method 2: Calibration (Recommended)
1. Capture footage with the checkerboard visible to camera A
2. Detect the checkerboard with OpenCV and run solvePnP
3. Save the resulting R, t as JSON

## Core Algorithm

### 1. Frame Synchronization
Matches B, C, and skeleton data based on Camera A timestamps:
- Threshold: 10ms (when using Sync Hub)
- Linear interpolation is applied to skeleton data

### 2. Helmet Pose Estimation
The following process is performed for each frame:
1. Detect checkerboard corners in B/C cameras (OpenCV)
2. Convert 2D corners to 3D (using depth)
3. Transform 3D points to world coordinates (applying calibration.json)
4. Compute checkerboard pose (centroid + SVD)
5. Camera A pose = T_checker_world × T_checker_to_A

### 3. Skeleton Coordinate Transformation
Transforms skeletons from world coordinates to Camera A coordinates:
```
P_A = R_A^T × (P_world - t_A)
```

### 4. 2D Projection
Distortion-aware projection using the K4A calibration API:
```cpp
k4a_calibration_3d_to_2d(&calibration, &point3d,
    K4A_CALIBRATION_TYPE_COLOR, K4A_CALIBRATION_TYPE_COLOR,
    &point2d, &valid);
```

## Workflow

Full data collection and processing workflow:

```bash
# 1. Camera calibration (one-time setup)
multi_device_calibration.exe --rows 6 --cols 9 --output calibration

# 2. Record (helmet + fixed cameras)
multi_device_recorder.exe --output ./recordings --session exp01

# 3. Offline skeleton estimation
multi_device_offline_processor.exe ^
    --calibration calibration.json ^
    --output skeleton_exp01.csv ^
    recordings/recording_cam*.mkv

# 4. Dataset generation
egocentric_dataset_generator.exe ^
    --camera-a recordings/helmet_exp01.mkv ^
    --camera-b recordings/fixed_b_exp01.mkv ^
    --skeleton skeleton_exp01.csv ^
    --calibration calibration.json ^
    --t-checker-to-a t_checker_a.json ^
    --output ./dataset_exp01
```

## Output Verification

Verify the generated dataset:

```python
import json
import cv2
import os

dataset_dir = "./dataset_exp01"

# Verify the first frame
with open(os.path.join(dataset_dir, "annotations/frame_000000.json")) as f:
    ann = json.load(f)

img = cv2.imread(os.path.join(dataset_dir, "images", ann["image_file"]))

# 2D skeleton overlay
for joint in ann["skeleton_2d"]:
    if joint["visible"]:
        cv2.circle(img, (int(joint["u"]), int(joint["v"])), 5, (0, 255, 0), -1)

cv2.imshow("Verification", img)
cv2.waitKey(0)
```

## Requirements

| Component | Version |
|-----------|---------|
| OrbbecSDK K4A Wrapper | v1.10.5 |
| OpenCV | 4.12.0 |
| Azure Kinect Body Tracking SDK | 1.1.2 |
| ONNX Runtime | 1.10.0 |

## Limitations

- The checkerboard must always be visible in at least one fixed camera
- When the checkerboard is not detected, the frame is skipped or the previous pose is used
- skeleton.csv must be in the `multi_device_offline_processor` output format

## Troubleshooting

### Checkerboard Detection Failure
- Check lighting conditions (avoid reflections and shadows)
- Check checkerboard size/resolution
- Verify `--checkerboard-rows` and `--checkerboard-cols` values

### Timestamp Synchronization Errors
- Verify all cameras are synchronized via Sync Hub
- Verify MKV files were recorded simultaneously

### 2D Projection Falls Outside Image
- Check Camera A calibration
- Verify T_checker_to_A measurement values
