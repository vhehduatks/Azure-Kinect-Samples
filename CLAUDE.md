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
| `multi_device_body_viewer` | **Orbbec Femto Bolt 멀티 카메라 Body Tracking** |

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

## Orbbec Femto Bolt Multi-Device Setup

Orbbec Femto Bolt 카메라로 Azure Kinect Body Tracking SDK를 사용하려면 K4A Wrapper가 필요합니다.

**상세 문서:** [multi_device_body_viewer/README.md](body-tracking-samples/multi_device_body_viewer/README.md)

### 빌드 후 필수 작업
NuGet 패키지가 Azure Kinect DLL을 덮어쓰므로, 빌드 후 Orbbec DLL을 복사해야 합니다:
```powershell
$src = "C:\OrbbecSDK_K4A_Wrapper_v1.10.5_windows_202510212040\bin"
Copy-Item "$src\k4a.dll", "$src\k4arecord.dll", "$src\depthengine_2_0.dll" .\build\bin\Release\ -Force
Copy-Item "$src\OrbbecSDK.dll", "$src\ob_usb.dll", "$src\live555.dll" .\build\bin\Release\ -Force
Copy-Item "$src\OrbbecSDKConfig_v1.0.xml" .\build\bin\Release\ -Force
```

### 캘리브레이션 내보내기
`multi_device_body_viewer.exe` 실행 중 `C` 키를 누르면:
- `intri.yml`: EasyMocap 형식 내부 파라미터
- `calibration.json`: 상세 캘리브레이션 정보

## Ref Notes
1. https://github.com/orbbec/OrbbecSDK-K4A-Wrapper
2. https://doc.orbbec.com/documentation/Camera%20Accessories/Set%20up%20Cameras%20for%20External%20Synchronization
3. C:\OrbbecSDK_C_C++_v1.10.27_20250925_0549823cb_win_x64_release\OrbbecSDK_v1.10.27\Example\cpp
4. https://orbbec.github.io/docs/OrbbecSDK_K4A_Wrapper/main/index.html
5. https://github.com/vhehduatks/EasyMocap/tree/master/apps/calibration

