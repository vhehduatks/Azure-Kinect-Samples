# Multi-Device Extrinsic Calibration Tool

Multi-camera extrinsic calibration tool for Orbbec Femto Bolt cameras using the K4A Wrapper. Based on the IEEE paper "Accurate Extrinsic Calibration of Multiple Azure Kinect Using a Planar Checkerboard".

## Features

- Automatic multi-device detection
- Real-time checkerboard corner detection using OpenCV
- SVD-based extrinsic calibration algorithm
- Exports calibration in YAML and JSON formats
- Computes relative transformations (secondary cameras to primary)
- **HMD Calibration Mode**: Computes T_checker_to_A for helmet-mounted cameras

## Requirements

- Orbbec Femto Bolt cameras (2+)
- OrbbecSDK K4A Wrapper v1.10.5+
- OpenCV 4.12.0 (installed at `C:\opencv`)
- Visual Studio 2022 (v143 toolset)
- Checkerboard pattern (default: 9x6 inner corners, 25mm squares)

## Build

1. Open `multi_device_calibration.sln` in Visual Studio 2022
2. Restore NuGet packages
3. Build Release x64
4. Copy Orbbec DLLs to output directory:

```powershell
$src = "C:\OrbbecSDK_K4A_Wrapper_v1.10.5_windows_202510212040\bin"
$dst = ".\build\bin\Release"
Copy-Item "$src\k4a.dll", "$src\k4arecord.dll", "$src\depthengine_2_0.dll" $dst -Force
Copy-Item "$src\OrbbecSDK.dll", "$src\ob_usb.dll", "$src\live555.dll" $dst -Force
Copy-Item "$src\OrbbecSDKConfig_v1.0.xml" $dst -Force
Copy-Item "C:\opencv\build\x64\vc16\bin\opencv_world4120.dll" $dst -Force
```

## Usage

```
multi_device_calibration.exe [OPTIONS]

Options:
  --rows N         Checkerboard inner corners (rows), default: 6
  --cols N         Checkerboard inner corners (cols), default: 9
  --square N       Square size in mm, default: 25.0
  --output FILE    Output filename prefix, default: calibration
  --primary SERIAL Serial number of PRIMARY camera (sync hub master port)
  --exclude SERIAL Exclude camera by serial number (can be used multiple times)
  --help           Show help
```

### Examples

```cmd
# Basic usage with primary camera specified
multi_device_calibration.exe --primary CL8T75400DC --rows 4 --cols 5 --square 50

# Exclude a helmet-mounted camera from calibration
multi_device_calibration.exe --primary CL8T75400DC --exclude CL8T75400GD

# Exclude multiple cameras
multi_device_calibration.exe --primary CL8T75400DC --exclude CAM1 --exclude CAM2
```

## Controls

| Key | Action |
|-----|--------|
| SPACE | Capture and calibrate |
| S | Save calibration to file |
| ESC | Quit |

## Calibration Procedure

1. Connect all Femto Bolt cameras via Orbbec Sync Hub
2. Configure sync hub in OrbbecViewer (set PRIMARY/SECONDARY ports)
3. Note the serial number of the camera connected to the PRIMARY port
4. Run the calibration tool with `--primary <serial>` option
5. Hold a checkerboard pattern visible to ALL cameras simultaneously
6. Press SPACE when checkerboard is detected in all views (green indicator)
7. Press S to save calibration results

### Sync Hub Configuration

The `--primary` option must match the camera connected to the sync hub's PRIMARY/MASTER port:
- The PRIMARY camera sends sync signals to SECONDARY cameras
- If `--primary` is not specified or incorrect, secondary cameras won't receive frames
- Use OrbbecViewer to verify which camera is connected to which port

## Output Files

### calibration.yml (OpenCV FileStorage format)
```yaml
num_devices: 2
calibrations:
  - device_index: 0
    serial_number: "ABC123"
    rotation: !!opencv-matrix
      rows: 3
      cols: 3
      data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
    translation: !!opencv-matrix
      rows: 3
      cols: 1
      data: [0, 0, 0]
```

### calibration.json
```json
{
  "num_devices": 2,
  "calibrations": [
    {
      "device_index": 0,
      "serial_number": "ABC123",
      "is_valid": true,
      "rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
      "translation": [0, 0, 0]
    }
  ]
}
```

## Algorithm (IEEE Paper)

The calibration follows the methodology from "Accurate Extrinsic Calibration of Multiple Azure Kinect Using a Planar Checkerboard":

1. **Checkerboard Detection**: OpenCV `findChessboardCorners()` with subpixel refinement
2. **2D to 3D Conversion**:
   - Transform depth image to color camera space using `k4a_transformation_depth_image_to_color_camera()`
   - Convert 2D color + depth to 3D using `k4a_calibration_2d_to_3d()`
3. **SVD-based Extrinsic Computation**:
   - Compute centroid of 3D checkerboard corners
   - Center points: X = points - centroid
   - Correlation matrix: C = X^T * X
   - SVD decomposition: C = U * S * V^T
   - Rotation: R = U^T (principal axes as coordinate frame)
   - Translation: t = -centroid
4. **Relative Transform**: Secondary cameras transformed relative to primary (device 0)

## Coordinate System

- Primary camera (device 0) is the reference frame (identity transform)
- Secondary cameras have R, t that transform their coordinates to primary frame
- To transform a point from camera N to camera 0:
  ```
  P_primary = R_N * P_N + t_N
  ```

## Troubleshooting

### No devices found
- Ensure Orbbec DLLs are copied (not Azure Kinect DLLs)
- Check OrbbecSDKConfig_v1.0.xml is in the executable directory
- Verify k4a.dll size is ~283KB (Orbbec) not ~651KB (Azure Kinect)

### Secondary cameras not showing frames
- Verify `--primary` option matches the camera on sync hub's PRIMARY port
- Check sync cable connections between cameras and sync hub
- Ensure sync hub is powered and configured correctly in OrbbecViewer
- The tool uses parallel capture threads - all cameras should display simultaneously

### Checkerboard not detected
- Ensure adequate lighting
- Hold checkerboard flat, perpendicular to camera view
- Try adjusting distance (0.5m - 2m works best)
- Verify checkerboard dimensions match --rows and --cols arguments

### Invalid depth at corners
- Move checkerboard closer to cameras
- Ensure checkerboard is within depth sensing range
- Avoid reflective checkerboard materials

---

## HMD Bridge Calibration Mode (T_checker_to_A)

This mode computes the fixed transform from a checkerboard (rigidly attached to helmet) to the helmet-mounted camera using the **Bridge Method** (Simultaneous Observation).

### The Problem

The helmet camera cannot see the checkerboard attached to itself. We solve this using two checkerboards:

```
                    Checkerboard A (Ground)
                           ▲
                          / \
                         /   \
            T_A_to_Ext  /     \  T_A_to_Helmet
                       /       \
                      ▼         ▼
            External Camera    Helmet Camera
             (Primary)          (Camera A)
                  │
                  │ T_B_to_Ext
                  ▼
            Checkerboard B (Helmet)
                  │
                  │ T_B_to_Helmet (rigid, UNKNOWN → solve for this)
                  ▼
            Helmet Camera Origin
```

### How It Works

1. **Ground Checkerboard (A)**: Placed on floor, visible to BOTH cameras
2. **Helmet Checkerboard (B)**: Attached to helmet, visible ONLY to external camera

### Bridge Formula Derivation

**Goal**: Find `T_checker_to_A` (helmet board → helmet camera transform)

**What we measure via solvePnP**:

| Transform | Description | Source |
|-----------|-------------|--------|
| `T_ground_to_external` | Ground board in external camera | External sees ground |
| `T_helmet_to_external` | Helmet board in external camera | External sees helmet board |
| `T_ground_to_helmet` | Ground board in helmet camera | Helmet sees ground |

**Visual chain** (ground board as bridge):

```
Helmet Board ──T_helmet_to_ext──► External Cam
     │                                 │
     │ T_checker_to_A                  │ T_ground_to_ext⁻¹
     │ (UNKNOWN)                       │
     ▼                                 ▼
Helmet Cam ◄──T_ground_to_helmet⁻¹── Ground Board
```

**Derivation**:

The helmet board and helmet camera are rigidly attached. Express helmet camera position through two paths:

```
Path via helmet board:  P_ext = T_helmet_to_ext × T_checker_to_A⁻¹ × P_helmet
Path via ground board:  P_ext = T_ground_to_ext × T_ground_to_helmet⁻¹ × P_helmet
```

Setting equal and solving:

```
T_checker_to_A = T_helmet_to_external⁻¹ × T_ground_to_external × T_ground_to_helmet⁻¹
```

**In code**:
```cpp
cv::Mat T_ext_to_helmet = T_helmet_to_external.inv();
cv::Mat T_helmet_to_ground = T_ground_to_helmet.inv();
cv::Mat T_checker_to_A = T_ext_to_helmet * T_ground_to_external * T_helmet_to_ground;
```

### Usage

```cmd
multi_device_calibration.exe --hmd-bridge ^
    --helmet-serial CL8T75400GD ^
    --primary CL8T75400DC ^
    --ground-rows 6 --ground-cols 9 --ground-square 25 ^
    --helmet-rows 4 --helmet-cols 5 --helmet-square 30 ^
    --output T_checker_to_A
```

### Bridge Mode Options

| Option | Description |
|--------|-------------|
| `--hmd-bridge` | Enable HMD bridge calibration mode |
| `--helmet-serial SERIAL` | Helmet camera serial number (required) |
| `--primary SERIAL` | External/primary camera serial (optional) |
| `--ground-rows N` | Ground checkerboard rows (default: 6) |
| `--ground-cols N` | Ground checkerboard cols (default: 9) |
| `--ground-square N` | Ground square size in mm (default: 25) |
| `--helmet-rows N` | Helmet checkerboard rows (default: 4) |
| `--helmet-cols N` | Helmet checkerboard cols (default: 5) |
| `--helmet-square N` | Helmet square size in mm (default: 30) |

**Important**: Use **different checkerboard sizes** for ground and helmet to avoid detection interference!

### Bridge Calibration Controls

| Key | Action |
|-----|--------|
| SPACE | Capture when all boards detected |
| C | Compute calibration from captures |
| S | Save T_checker_to_A.json |
| ESC | Quit |

### Bridge Calibration Procedure

1. **Place GROUND checkerboard** (e.g., 9x6, 25mm) on the floor
2. **Attach HELMET checkerboard** (e.g., 5x4, 30mm) rigidly to the helmet
3. **Position the helmet** so that:
   - External camera sees BOTH ground and helmet boards
   - Helmet camera sees the ground board
4. Press **SPACE** to capture when all three detections are green
5. **Move to different positions** and capture 5-10 times
6. Press **C** to compute the averaged calibration
7. Press **S** to save

### T_checker_to_A.json Output Format

```json
{
  "description": "Checkerboard to Helmet Camera (A) transformation",
  "num_captures": 10,
  "reprojection_error": 0.5,
  "rotation": [
    [r11, r12, r13],
    [r21, r22, r23],
    [r31, r32, r33]
  ],
  "translation": [tx, ty, tz],
  "rvec": [rx, ry, rz]
}
```

- `rotation`: 3x3 rotation matrix (helmet board to helmet camera)
- `translation`: Translation in mm
- `rvec`: Rodrigues rotation vector

### Complete Two-Stage Calibration Workflow

```
Stage 1: External Camera Calibration (checkerboard on ground)
─────────────────────────────────────────────────────────────
multi_device_calibration.exe --primary CAM_B --exclude CAM_A --output calibration
                                  │
                                  ▼
                           calibration.json
                        (B ↔ C extrinsics)


Stage 2: HMD Bridge Calibration (two checkerboards)
─────────────────────────────────────────────────────────────
multi_device_calibration.exe --hmd-bridge \
    --helmet-serial CAM_A \
    --ground-rows 6 --ground-cols 9 --ground-square 25 \
    --helmet-rows 4 --helmet-cols 5 --helmet-square 30 \
    --output T_checker_to_A
                                  │
                                  ▼
                          T_checker_to_A.json
                      (helmet board → helmet cam)


Runtime: Egocentric Dataset Generation
─────────────────────────────────────────────────────────────
egocentric_dataset_generator.exe \
    --calibration calibration.json \
    --t-checker-to-a T_checker_to_A.json \
    ...
```

---

## Related Projects

- `multi_device_body_viewer` - Multi-camera body tracking visualization
- `multi_device_recorder` - Multi-camera MKV recording
- `egocentric_dataset_generator` - ML dataset generator using HMD calibration
- `simple_3d_viewer` - Single camera body tracking

## License

MIT License - See repository root for details.
