# Multi-Device Body Tracking Viewer

Multi-camera body tracking viewer for Orbbec Femto Bolt cameras with skeleton fusion support. Uses the K4A Wrapper and Azure Kinect Body Tracking SDK.

## Features

- Multi-device body tracking (2+ cameras)
- Hardware sync via Orbbec Sync Hub
- **Skeleton Fusion**: Combine tracking data from multiple cameras to compensate for occlusion
- Two fusion modes: Winner-Takes-All and Weighted Average
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

```powershell
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
| F | Toggle skeleton fusion on/off |
| M | Switch fusion mode (winner/weighted) |
| B | Toggle body visualization mode |
| K | Change 3D window layout |
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
- `simple_3d_viewer` - Single camera body tracking

## License

MIT License - See repository root for details.
