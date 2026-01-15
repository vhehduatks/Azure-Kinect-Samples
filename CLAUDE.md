# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Overview

Microsoft Azure Kinect DK Samples repository containing code samples demonstrating the Azure Kinect Sensor SDK and Body Tracking SDK. Samples are provided from various sources and may only work on Windows or Linux.

## Build Commands

### Windows
Open the `.sln` file for any sample in Visual Studio and build. Each sample has its own solution file:
- `body-tracking-samples/<sample_name>/<sample_name>.sln`

CMake is not officially supported on Windows.

### Linux
```bash
# Prerequisites
apt install libk4abt1.1-dev libxi-dev

# Build (must be from git clone, not ZIP)
mkdir build && cd build
cmake .. -GNinja
ninja
```

Output binaries go to `build/bin/`.

## Project Structure

### Body Tracking Samples (`body-tracking-samples/`)
| Sample | Description |
|--------|-------------|
| `simple_sample` | Minimal C API example |
| `simple_cpp_sample` | Minimal C++ wrapper example |
| `simple_3d_viewer` | Full 3D visualization (C++) |
| `csharp_3d_viewer` | 3D visualization (C#) |
| `floor_detector_sample` | IMU-based floor plane detection |
| `jump_analysis_sample` | Jump height/kinematics analysis |
| `offline_processor` | Batch process MKV recordings |
| `camera_space_transform_sample` | Depth-to-color space transformation |
| `sample_unity_bodytracking` | Unity integration |
| `multi_device_body_viewer` | Multi-camera body tracking (Orbbec Femto Bolt) |
| `multi_device_calibration` | Multi-camera extrinsic calibration tool |
| `multi_device_recorder` | Multi-camera MKV recording (no body tracking) |
| `multi_device_offline_processor` | Offline body tracking with multi-camera fusion |

### Other Samples
- `opencv-kinfu-samples/` - OpenCV KinectFusion 3D reconstruction
- `pipe-to-python-samples/` - C++ pipe server with Python client for IPC
- `build2019/csharp/` - C# tutorials (image acquisition, depth transform, cognitive services)

### Shared Code (`body-tracking-samples/`)
- `sample_helper_includes/` - Common headers (`BodyTrackingHelpers.h`, `Utilities.h`)
- `sample_helper_libs/window_controller_3d/` - OpenGL 3D rendering utilities
- `extern/` - Git submodules (GLFW, nlohmann_json)
- `sample_recordings/` - Sample MKV files for offline testing

## Key Dependencies

| Component | Version |
|-----------|---------|
| Azure Kinect Sensor SDK | 1.4.1 |
| Azure Kinect Body Tracking SDK | 1.1.2 |
| ONNX Runtime | 1.10.0 |
| C++ Standard | C++17 |
| C Standard | C99 |

GPU execution providers: DirectML (Windows default), CUDA, TensorRT, CPU

## Code Patterns

### Error Handling (C/C++)
Use macros from `Utilities.h`:
```cpp
VERIFY(k4a_device_open(0, &device), "Failed to open device");
EXIT_IF(result != K4A_RESULT_SUCCEEDED, "Operation failed");
```

### Body Tracking Data
- `g_boneList` in `BodyTrackingHelpers.h` defines 31 bone connections between joints
- `g_jointNames` maps `k4abt_joint_id_t` to string names
- `g_bodyColors` provides 20 distinct colors for multi-body visualization

### Sample Runtime Modes
Most viewer samples support:
- Sensor modes: `NFOV_UNBINNED` (default), `WFOV_BINNED`
- Processing modes: GPU (default), `CPU`, `OFFLINE` (for MKV playback)

Example: `simple_3d_viewer.exe WFOV_BINNED CPU`

## Unity Sample Setup

The Unity sample (`sample_unity_bodytracking/`) requires:
1. Run `Update-Package -reinstall` in VS Package Manager Console
2. Run `MoveLibraryFile.bat` to copy DLLs to Assets/Plugins
3. Set `TrackerProcessingMode` in `SkeletalTrackingProvider.cs`
4. Install Visual C++ Redistributable and GPU runtime (CUDA/cuDNN/TensorRT if not using DirectML)

## Architecture Notes

- Samples use either C API (`k4a_*`, `k4abt_*` functions) or C++ wrappers (`k4a::`, `k4abt::` namespaces)
- 3D visualization samples share the `window_controller_3d` library for OpenGL rendering
- Body tracking outputs 32 joints per body with confidence levels
- Coordinate system: depth camera is origin, Y-up, Z-forward (into scene)

## Orbbec Femto Bolt Setup

The `multi_device_body_viewer` and `multi_device_calibration` samples use Orbbec Femto Bolt cameras via the K4A Wrapper.

### Required Components
| Component | Version | Path |
|-----------|---------|------|
| OrbbecSDK K4A Wrapper | v1.10.5 | `C:\OrbbecSDK_K4A_Wrapper_v1.10.5_windows_202510212040` |
| OpenCV | 4.12.0 | `C:\opencv` |
| Azure Kinect Body Tracking SDK | 1.1.2 | NuGet package |

### DLL Replacement (Critical)
After building, NuGet copies Azure Kinect DLLs which must be replaced with Orbbec versions:

```powershell
$src = "C:\OrbbecSDK_K4A_Wrapper_v1.10.5_windows_202510212040\bin"
$dst = ".\build\bin\Release"

# Replace K4A DLLs with Orbbec versions
Copy-Item "$src\k4a.dll", "$src\k4arecord.dll", "$src\depthengine_2_0.dll" $dst -Force
Copy-Item "$src\OrbbecSDK.dll", "$src\ob_usb.dll", "$src\live555.dll" $dst -Force
Copy-Item "$src\OrbbecSDKConfig_v1.0.xml" $dst -Force

# For calibration tool, also copy OpenCV
Copy-Item "C:\opencv\build\x64\vc16\bin\opencv_world4120.dll" $dst -Force
```

**Verification**: k4a.dll should be ~283KB (Orbbec), not ~651KB (Azure Kinect)

### Multi-Device Sync Configuration
1. Connect cameras via Orbbec Sync Hub
2. Configure Primary/Secondary in OrbbecViewer before running samples
3. First device (index 0) = Primary/Master
4. Other devices = Secondary/Subordinate with 160μs delay offset

## Multi-Device Calibration

The calibration tool (`multi_device_calibration`) computes extrinsic parameters using a checkerboard:

### Algorithm (IEEE Paper)
1. Detect checkerboard corners (OpenCV)
2. Convert 2D to 3D via depth-color transformation
3. SVD-based rotation/translation computation
4. Output: R, t for each camera relative to primary

### Usage
```bash
multi_device_calibration.exe --rows 6 --cols 9 --square 25.0 --output calibration
# SPACE: capture, S: save, ESC: quit
```

### Output Files
- `calibration.yml` - OpenCV FileStorage format
- `calibration.json` - JSON format with R (3x3) and t (3x1)

## Detailed Documentation

For detailed usage, see the README files in each project:

| Project | README | Description |
|---------|--------|-------------|
| `multi_device_body_viewer` | [README.md](body-tracking-samples/multi_device_body_viewer/README.md) | Multi-camera body tracking with skeleton fusion, CSV recording, camera view switching |
| `multi_device_calibration` | [README.md](body-tracking-samples/multi_device_calibration/README.md) | Multi-camera extrinsic calibration using checkerboard |
| `multi_device_recorder` | [README.md](body-tracking-samples/multi_device_recorder/README.md) | Zero-latency MKV recording for offline processing |
| `multi_device_offline_processor` | [README.md](body-tracking-samples/multi_device_offline_processor/README.md) | Offline body tracking with multi-camera fusion from MKV files |
| `sample_unity_bodytracking` | [README.md](body-tracking-samples/sample_unity_bodytracking/README.md) | Unity integration with multi-camera fusion and HMD recording |
| `sample_unity_hmd_recorder` | [README.md](body-tracking-samples/sample_unity_hmd_recorder/README.md) | Unity HMD/controller 6DOF recording with UDP sync |
| `scripts` | [README.md](body-tracking-samples/scripts/README.md) | Python synchronization scripts for skeleton/HMD data alignment |

## Ref Notes
1. https://github.com/orbbec/OrbbecSDK-K4A-Wrapper
2. https://doc.orbbec.com/documentation/Camera%20Accessories/Set%20up%20Cameras%20for%20External%20Synchronization
3. C:\OrbbecSDK_C_C++_v1.10.27_20250925_0549823cb_win_x64_release\OrbbecSDK_v1.10.27\Example\cpp

