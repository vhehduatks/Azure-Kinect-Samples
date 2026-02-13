# Multi-Device Body Tracking Viewer

Multi-camera body tracking viewer for Orbbec Femto Bolt cameras with skeleton fusion support. Uses the K4A Wrapper and Azure Kinect Body Tracking SDK.

## Features

- Multi-device body tracking (2+ cameras)
- Hardware sync via Orbbec Sync Hub
- **Skeleton Fusion**: Combine tracking data from multiple cameras to compensate for occlusion
- Two fusion modes: Winner-Takes-All and Weighted Average
- **Camera View Switching**: View individual camera feeds or all cameras combined
- **CSV Recording**: Record skeleton data with epoch timestamps for offline synchronization
- Real-time 3D visualization

## Requirements

- Orbbec Femto Bolt cameras (2+)
- OrbbecSDK K4A Wrapper v1.10.5+
- Azure Kinect Body Tracking SDK 1.1.2
- Visual Studio 2022 (v143 toolset)
- Orbbec Sync Hub (for multi-camera sync)

## Build

1. Open `multi_device_body_viewer.sln` in Visual Studio 2022
2. Restore NuGet packages
3. Build Release x64
4. Copy Orbbec DLLs to output directory:

```bash
# Option 1: Use batch file
copy_orbbec_dlls.bat

# Option 2: Manual PowerShell
$src = "C:\OrbbecSDK_K4A_Wrapper_v1.10.5_windows_202510212040\bin"
$dst = ".\build\bin\Release"
Copy-Item "$src\k4a.dll", "$src\k4arecord.dll", "$src\depthengine_2_0.dll" $dst -Force
Copy-Item "$src\OrbbecSDK.dll", "$src\ob_usb.dll", "$src\live555.dll" $dst -Force
Copy-Item "$src\OrbbecSDKConfig_v1.0.xml" $dst -Force
```

**Verification**: k4a.dll should be ~283KB (Orbbec), not ~651KB (Azure Kinect)

## Usage

```
multi_device_body_viewer.exe [OPTIONS]

Depth Mode:
  NFOV_UNBINNED      Narrow FOV Unbinned (default)
  WFOV_BINNED        Wide FOV Binned

Processing Mode:
  CPU                CPU processing mode
  CUDA               CUDA processing mode
  DIRECTML           DirectML processing mode (default on Windows)
  TENSORRT           TensorRT processing mode

Multi-Camera Sync:
  --primary SERIAL   Serial number of PRIMARY camera (sync hub master port)

Skeleton Fusion:
  --calibration FILE Load calibration file and enable fusion
  --fusion-mode MODE Fusion mode: winner | weighted (default: weighted)

CSV Recording:
  --output FILE      Output CSV file path (default: skeleton_data_YYYYMMDD_HHMMSS.csv)
```

### Examples

```bash
# Basic multi-camera (without fusion)
multi_device_body_viewer.exe --primary CL8T75400DC

# With skeleton fusion
multi_device_body_viewer.exe --primary CL8T75400DC --calibration calibration.json

# With specific fusion mode
multi_device_body_viewer.exe --primary CL8T75400DC --calibration calibration.json --fusion-mode winner
```

## Runtime Controls

| Key | Action |
|-----|--------|
| K | Cycle camera view (All → Cam0 → Cam1 → Cam2 → All) |
| R | Start/stop CSV recording |
| F | Toggle skeleton fusion on/off |
| M | Switch fusion mode (winner/weighted) |
| B | Toggle body visualization mode |
| H | Show help |
| ESC | Quit |

## Skeleton Fusion

### Overview

When using multiple cameras, each camera tracks bodies independently. Skeleton fusion combines these results to create a more complete skeleton by compensating for occlusions.

```
Camera 1 → Tracker 1 → Skeleton 1 ─┐
Camera 2 → Tracker 2 → Skeleton 2 ─┼→ Transform → Match → Fuse → Fused Skeleton
Camera 3 → Tracker 3 → Skeleton 3 ─┘
```

### Calibration Requirement

Skeleton fusion requires extrinsic calibration data from `multi_device_calibration`:

```bash
# Step 1: Run calibration
multi_device_calibration.exe --primary CL8T75400DC --output calibration

# Step 2: Run body viewer with calibration
multi_device_body_viewer.exe --primary CL8T75400DC --calibration calibration.json
```

### Fusion Modes

1. **Winner-Takes-All** (`--fusion-mode winner`)
   - For each joint, select the one with highest confidence
   - Best for: Clear occlusion scenarios where one camera has a better view

2. **Weighted Average** (`--fusion-mode weighted`) [Default]
   - Confidence-weighted average of joint positions
   - Weights: NONE=0, LOW=0.25, MEDIUM=0.6, HIGH=1.0
   - Best for: General use, reduces tracking noise

### Body Matching

Bodies are matched across cameras using pelvis position proximity (threshold: 500mm). The same person seen by multiple cameras is identified and their joint data is fused.

## CSV Recording

Record skeleton data to CSV for offline analysis or synchronization with other data sources (e.g., HMD tracking from Unity).

### Usage

Press `R` to start/stop recording. CSV file is saved with epoch timestamps for synchronization.

```bash
# With custom output path
multi_device_body_viewer.exe --primary CL8T75400DC --output my_recording.csv
```

### CSV Format

```csv
timestamp_ms,device_index,body_id,joint_id,joint_name,pos_x,pos_y,pos_z,rot_w,rot_x,rot_y,rot_z,confidence
1704067200000,0,100,0,PELVIS,0.123,0.456,0.789,1.0,0.0,0.0,0.0,2
...
```

| Column | Description |
|--------|-------------|
| timestamp_ms | System time in epoch milliseconds |
| device_index | Camera index (0, 1, 2, ...) |
| body_id | Body ID (offset by device: device0=0-99, device1=100-199, ...) |
| joint_id | Joint index (0-31) |
| joint_name | Joint name (PELVIS, SPINE_NAVAL, ...) |
| pos_x/y/z | Joint position in mm |
| rot_w/x/y/z | Joint orientation quaternion |
| confidence | Confidence level (0=NONE, 1=LOW, 2=MEDIUM, 3=HIGH) |

### Synchronization with HMD Data

Use the Python script to synchronize with Unity HMD recordings. The script uses **coordinate-system invariant cross-correlation** based on inter-point distances.

```bash
cd ../scripts

# Recommended: Inter-point distance method
python sync_skeleton_hmd.py --skeleton skeleton_data.csv --hmd HMD_Walk_P01.csv --method distance

# With visualization
python sync_skeleton_hmd.py --skeleton skeleton_data.csv --hmd HMD_Walk_P01.csv --method distance --plot
```

See [scripts/README.md](../scripts/README.md) for detailed usage.

## UDP Sync with Unity

Synchronize recording start/stop between this application and Unity via UDP commands.

### How It Works

```
Unity (R key) ──UDP:9000──► multi_device_body_viewer
                           ↓ (toggle recording)
Unity ◄──UDP:9001────────── (confirmation)
```

### Usage

```bash
# With UDP sync enabled (default)
multi_device_body_viewer.exe --primary CL3FC3100HN

# With custom port
multi_device_body_viewer.exe --primary CL3FC3100HN --udp-port 9000

# Disable UDP sync
multi_device_body_viewer.exe --primary CL3FC3100HN --no-udp
```

### Unity Setup

1. Add `RecordingSyncController.cs` to a GameObject
2. Configure ports in Inspector:
   - Send Port: 9000 (must match `--udp-port`)
   - Receive Port: 9001
3. Press R in Unity to toggle recording in both applications

### Protocol

| Command | Description |
|---------|-------------|
| `TOGGLE_RECORD` | Toggle recording on/off |
| `START_RECORD` | Start recording |
| `STOP_RECORD` | Stop recording |
| `CYCLE_CAMERA` | Cycle camera view |

## Helmet Camera Overlay

Project the fused skeleton onto a helmet-mounted camera's live feed for calibration verification. The helmet camera has a checkerboard rigidly attached, and the fixed cameras detect it to compute the helmet's pose in real-time.

### Requirements

- Helmet-mounted Orbbec Femto Bolt camera
- Checkerboard rigidly attached to the helmet
- `T_checker_to_A.json` from `hmd_calibration` (defines checkerboard-to-camera transform)
- Extrinsic calibration file (`calibration.json`) from `multi_device_calibration`

### Usage

```bash
multi_device_body_viewer.exe \
  --primary CL8T75400DC \
  --calibration calibration.json \
  --helmet-serial CL3FC3100AB \
  --t-checker-to-a T_checker_to_A.json \
  --helmet-cb-rows 4 \
  --helmet-cb-cols 5 \
  --helmet-cb-square 30
```

| Option | Default | Description |
|--------|---------|-------------|
| `--helmet-serial SN` | (required) | Serial number of helmet-mounted camera |
| `--t-checker-to-a FILE` | `T_checker_to_A.json` | Checkerboard-to-camera transform |
| `--helmet-cb-rows N` | 4 | Checkerboard inner corner rows |
| `--helmet-cb-cols N` | 5 | Checkerboard inner corner cols |
| `--helmet-cb-square N` | 30 | Square size in mm |

### How It Works

```
Fixed Camera 0 ──► Body Tracker ──► Fused Skeleton (world frame)
Fixed Camera 1 ──► Body Tracker ──┘        │
  ...                                      │
                                           ▼
Fixed Camera N ──► Detect Checkerboard ──► Helmet Pose (world frame)
                   Convert to 3D              │
                   Transform to world         │
                   Compose with T_checker_to_A│
                                              ▼
Helmet Camera ──► Color Image ──► Project skeleton ──► cv::imshow
```

1. **Fixed cameras** detect the checkerboard on the helmet and compute its 3D pose in the world frame
2. **Multi-camera fusion**: When multiple fixed cameras see the checkerboard, their pose estimates are fused using inverse-square-distance weighting
3. **Outlier rejection**: With 3+ detections, candidates >100mm from the median translation are rejected before fusion
4. **EMA smoothing**: Fused pose is temporally smoothed (α=0.75) to reduce frame-to-frame jitter, with a 200ms staleness guard to avoid ghost positions after detection gaps
5. The checkerboard pose is composed with `T_checker_to_A` to get the helmet camera's pose
6. Fused skeleton joints are transformed from world to helmet camera coordinates
7. Joints are projected to 2D using `k4a_calibration_3d_to_2d` and drawn on the color image

### Runtime Controls

Press **V** to toggle the helmet overlay window on/off.

### Status Indicators

- **Green "TRACKING"**: Checkerboard detected, skeleton projected
- **Red "NO CHECKERBOARD"**: No checkerboard visible to any fixed camera
- **Red "CHECKERBOARD LOST"**: Detection stale (>500ms since last detection)

### Build Note

The helmet overlay requires OpenCV 4.12.0. Ensure `opencv_world4120.dll` is in the output directory alongside the Orbbec DLLs:

```powershell
Copy-Item "C:\opencv\build\x64\vc16\bin\opencv_world4120.dll" ".\build\bin\Release" -Force
```

## Sync Hub Configuration

The `--primary` option must match the camera connected to the sync hub's PRIMARY/MASTER port:

1. Connect cameras via Orbbec Sync Hub
2. Configure Primary/Secondary ports in OrbbecViewer
3. Note the serial number of the PRIMARY port camera
4. Use `--primary <serial>` when running

If `--primary` is not specified, device 0 is assumed to be primary.

## Troubleshooting

### No devices found
- Ensure Orbbec DLLs are copied (not Azure Kinect DLLs)
- Check OrbbecSDKConfig_v1.0.xml is in the executable directory
- Verify k4a.dll size is ~283KB (Orbbec) not ~651KB (Azure Kinect)

### Secondary cameras not showing frames
- Verify `--primary` option matches the camera on sync hub's PRIMARY port
- Check sync cable connections
- Ensure sync hub is powered and configured in OrbbecViewer

### Fusion not working
- Ensure calibration.json exists and is valid
- Verify camera serial numbers in calibration match connected devices
- Check console for "Skeleton fusion enabled" message

### Rendering freezes
- This was fixed by minimizing mutex lock time in the fusion pipeline
- If issue persists, try disabling fusion with 'F' key

## Related Projects

- `multi_device_calibration` - Multi-camera extrinsic calibration tool
- `scripts/` - Python synchronization scripts for offline data alignment
- `sample_unity_bodytracking` - Unity body tracking with HMD recording
- `simple_3d_viewer` - Single camera body tracking

## License

MIT License - See repository root for details.
