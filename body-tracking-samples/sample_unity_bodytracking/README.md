# Unity Body Tracking Sample

Unity sample demonstrating Azure Kinect Body Tracking SDK integration with multi-camera skeleton fusion support.

## Features

- Single-camera body tracking with 3D avatar rendering
- **Multi-camera skeleton fusion** for occlusion compensation
- Runtime fusion mode switching
- Support for Azure Kinect DK and Orbbec Femto Bolt cameras

## Requirements

- Unity 2021.3 or later
- Azure Kinect Body Tracking SDK 1.1.2
- Visual C++ Redistributable
- CUDA/cuDNN/TensorRT (for CUDA processing mode)

### For Orbbec Femto Bolt

Replace DLLs in `Assets/Plugins/` with Orbbec K4A Wrapper versions:

| File | Description |
|------|-------------|
| `k4a.dll` | ~283KB (Orbbec) instead of ~651KB (Azure) |
| `k4arecord.dll` | Orbbec version |
| `depthengine_2_0.dll` | Orbbec version |
| `OrbbecSDK.dll` | Add new |
| `ob_usb.dll` | Add new |
| `live555.dll` | Add new |
| `OrbbecSDKConfig_v1.0.xml` | Add new |

Source: `C:\OrbbecSDK_K4A_Wrapper_v1.10.5_windows_202510212040\bin\`

## Setup

1. Open the project in Unity
2. Run `Update-Package -reinstall` in VS Package Manager Console (if needed)
3. Run `MoveLibraryFile.bat` to copy DLLs to Assets/Plugins
4. For Orbbec cameras, replace DLLs as described above

## Multi-Camera Skeleton Fusion

Skeleton fusion combines body tracking data from multiple calibrated cameras to compensate for occlusion.

### Architecture

```
Camera 1 → Tracker 1 → Skeleton 1 ─┐
Camera 2 → Tracker 2 → Skeleton 2 ─┼→ Transform → Match → Fuse → Fused Skeleton
Camera N → Tracker N → Skeleton N ─┘
```

### Calibration

Generate `calibration.json` using the `multi_device_calibration` tool:

```bash
multi_device_calibration.exe --rows 6 --cols 9 --square 25.0 --output calibration
```

Place the `calibration.json` file in one of these locations:
- `Assets/StreamingAssets/calibration.json` (recommended)
- Absolute path specified in Inspector

### Configuration

In Unity Inspector on the `main` GameObject:

| Setting | Description |
|---------|-------------|
| **Enable Fusion** | Enable multi-camera skeleton fusion |
| **Calibration File** | Path to calibration.json |
| **Fusion Mode** | WeightedAverage or WinnerTakesAll |

### Runtime Controls

| Key | Action |
|-----|--------|
| `F` | Toggle fusion on/off |
| `M` | Switch fusion mode |

### Fusion Modes

| Mode | Description |
|------|-------------|
| **WeightedAverage** | Confidence-weighted position averaging (recommended) |
| **WinnerTakesAll** | Use joint from camera with highest confidence |

### Confidence Weights

| Level | Weight |
|-------|--------|
| None | 0.0 |
| Low | 0.25 |
| Medium | 0.6 |
| High | 1.0 |

## Scripts

| File | Description |
|------|-------------|
| `main.cs` | Entry point, fusion configuration |
| `SkeletalTrackingProvider.cs` | Single-camera body tracking |
| `FusedSkeletalTrackingProvider.cs` | Multi-camera fusion provider |
| `CalibrationLoader.cs` | Parses calibration.json |
| `SkeletonFusion.cs` | Transform, match, fuse algorithms |
| `TrackerHandler.cs` | Avatar rendering |
| `Body.cs` | Body data structure |
| `BackgroundData.cs` | Frame data container |

## Troubleshooting

### "No devices connected!"
- For Orbbec cameras: Replace DLLs with Orbbec K4A Wrapper versions
- Check USB connections
- Verify cameras work in OrbbecViewer first

### Fusion not working
- Verify `calibration.json` path is correct
- Check Unity Console for calibration loading messages
- Ensure all cameras are connected and detected

### Poor tracking quality
- Ensure cameras are properly calibrated
- Check camera sync hub configuration
- Try switching fusion modes

## HMD Data Recording & Synchronization

This sample includes HMD and controller recording via `DataRecorder.cs`.

### Recording

1. Attach `DataRecorder` component to a GameObject
2. Configure participant ID and session name in Inspector
3. Press `R` to start/stop recording
4. CSV files are saved to `Assets/HMD_CSV/` and `Assets/JointCSV/`

### Synchronization with External Skeleton Data

When using skeleton data from `multi_device_body_viewer` (C++ app), use the sync script:

```bash
cd ../scripts

# Recommended: Inter-point distance method (coordinate-invariant)
python sync_skeleton_hmd.py --skeleton skeleton_data.csv --hmd HMD_Walk_P01.csv --method distance

# With visualization
python sync_skeleton_hmd.py --skeleton skeleton_data.csv --hmd HMD_Walk_P01.csv --method distance --plot
```

The script uses **inter-point distances** (head-hand, hand-hand) for cross-correlation, which works even though Orbbec and Quest use different coordinate systems.

See [scripts/README.md](../scripts/README.md) for detailed usage.

## See Also

- [multi_device_calibration](../multi_device_calibration/) - Calibration tool
- [multi_device_body_viewer](../multi_device_body_viewer/) - C++ fusion viewer
- [scripts](../scripts/) - Python synchronization scripts
