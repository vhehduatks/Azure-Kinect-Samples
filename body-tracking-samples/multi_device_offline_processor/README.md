# Multi-Device Offline Body Tracking Processor

Processes MKV recordings from multiple Orbbec Femto Bolt cameras with:

1. **Body tracking** - Extracts skeleton data from each MKV file
2. **Calibration-based fusion** - Transforms skeletons to common coordinate system
3. **Multi-camera fusion** - Combines skeletons from multiple cameras for better accuracy
4. **Ego-view processing** - Projects fused skeletons onto helmet camera images

## Why Use This?

- **No processing latency** - Timestamps in MKV files are capture times, not processing times
- **Better accuracy** - Multiple cameras can see different angles, fusion improves joint confidence
- **Reproducible results** - Same MKV files always produce same output
- **Ego-view datasets** - Generate 2D/3D skeleton annotations for helmet camera footage

## Build

1. Open `multi_device_offline_processor.sln` in Visual Studio
2. Restore NuGet packages (right-click solution → Restore NuGet Packages)
3. Build in Release mode
4. Run `copy_orbbec_dlls.bat` to replace Azure Kinect DLLs with Orbbec versions
5. Copy `opencv_world4120.dll` to the output directory (for ego-view mode)

## Usage

### Standard Mode (Body Tracking + Fusion)

```bash
# Single camera processing
multi_device_offline_processor.exe recording_cam0.mkv --output skeleton.csv

# Multi-camera with fusion
multi_device_offline_processor.exe --calibration calib.json --output skeleton.csv \
    recording_cam0.mkv recording_cam1.mkv

# Specify processing mode
multi_device_offline_processor.exe --mode DirectML --calibration calib.json \
    --output skeleton.csv recording_cam0.mkv recording_cam1.mkv

# With tilted cameras (90° counterclockwise) and temporal smoothing
multi_device_offline_processor.exe --calibration calib.json \
    --sensor-orientation ccw90 --smoothing 0.5 \
    --output skeleton.csv recording_cam0.mkv recording_cam1.mkv
```

### Ego-View Mode

Processes a helmet-mounted camera alongside fixed cameras to generate egocentric skeleton annotations.

```bash
multi_device_offline_processor.exe \
    --calibration calibration.json \
    --helmet-serial CL3FC3100HN \
    --t-checker-to-a T_checker_to_A.json \
    --helmet-cb-rows 4 --helmet-cb-cols 5 --helmet-cb-square 30 \
    --ego-output ego_dataset/ \
    cam0.mkv cam1.mkv cam2.mkv helmet.mkv
```

The helmet camera MKV is identified by its serial number. The processor:
1. Runs body tracking on fixed cameras only (not the helmet camera)
2. Fuses skeletons from fixed cameras into world coordinates
3. Detects the checkerboard (attached to helmet) in fixed camera images
4. Fuses helmet pose from multiple camera detections with outlier rejection and EMA smoothing
5. Transforms fused 3D joints into helmet camera frame
6. Projects 3D joints to 2D on helmet camera images
7. Saves ego-view images and per-frame JSON annotations

### Command Line Options

| Option | Description |
|--------|-------------|
| `--calibration FILE` | Calibration JSON file (required for fusion and ego mode) |
| `--output FILE` | Output CSV file (default: output.csv) |
| `--mode MODE` | Processing mode: CPU, CUDA, DirectML (default), TensorRT |
| `--sync-threshold MS` | Max timestamp difference for sync (default: 33ms) |
| `--sensor-orientation ORI` | Sensor orientation: default, cw90, ccw90, flip180 |
| `--smoothing FACTOR` | Temporal smoothing factor 0.0-1.0 (default: 0.0) |

### Ego-View Options

| Option | Description |
|--------|-------------|
| `--helmet-serial SERIAL` | Serial number of helmet camera (enables ego mode) |
| `--t-checker-to-a PATH` | Path to T_checker_to_A.json (required for ego mode) |
| `--helmet-cb-rows N` | Checkerboard inner corners rows (default: 4) |
| `--helmet-cb-cols N` | Checkerboard inner corners cols (default: 5) |
| `--helmet-cb-square N` | Checkerboard square size in mm (default: 30) |
| `--ego-output DIR` | Output directory for ego-view data (default: ego_output/) |

### Processing Modes

| Mode | Description |
|------|-------------|
| `DirectML` | Windows GPU (default, recommended) |
| `CUDA` | NVIDIA GPU with CUDA |
| `TensorRT` | NVIDIA GPU with TensorRT optimization |
| `CPU` | CPU only (slowest) |

## Calibration File Format

The calibration file is JSON format from `multi_device_calibration`:

```json
{
  "num_devices": 2,
  "calibrations": [
    {
      "device_index": 0,
      "serial_number": "CL2K1234567",
      "is_valid": false,
      "rotation": [[1,0,0],[0,1,0],[0,0,1]],
      "translation": [0,0,0]
    },
    {
      "device_index": 1,
      "serial_number": "CL2K7654321",
      "is_valid": true,
      "rotation": [[r00,r01,r02],[r10,r11,r12],[r20,r21,r22]],
      "translation": [tx,ty,tz]
    }
  ]
}
```

- `is_valid: false` = Primary camera (reference frame)
- `is_valid: true` = Secondary camera with transformation to primary

## Output

### CSV Format (Standard)

Wide format with one row per body per frame:

```csv
timestamp_usec,body_id,J0_x,J0_y,J0_z,J0_conf,J1_x,J1_y,J1_z,J1_conf,...
```

| Column | Description |
|--------|-------------|
| `timestamp_usec` | Device timestamp in microseconds |
| `body_id` | Fused body ID |
| `J{n}_x/y/z` | Joint position (mm) in primary camera space |
| `J{n}_conf` | Confidence level (0=None, 1=Low, 2=Medium, 3=High) |

Joint indices follow Azure Kinect Body Tracking SDK (0-31).

### Ego-View Output

```
ego_output/
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

#### Per-Frame JSON Format

```json
{
  "frame_id": 0,
  "timestamp_usec": 1234567890,
  "image_file": "frame_000000.jpg",
  "checkerboard_detected": true,
  "detection_camera": 1,
  "num_bodies": 1,
  "camera_pose": {
    "R": [[r00,r01,r02], [r10,r11,r12], [r20,r21,r22]],
    "t": [tx, ty, tz]
  },
  "skeleton_3d": [
    {"joint_id": 0, "name": "PELVIS", "x": 100.0, "y": -500.0, "z": 1500.0, "confidence": 2},
    ...
  ],
  "skeleton_2d": [
    {"joint_id": 0, "name": "PELVIS", "u": 640.0, "v": 360.0, "confidence": 2, "visible": true},
    ...
  ]
}
```

| Field | Description |
|-------|-------------|
| `checkerboard_detected` | Whether checkerboard was found this frame |
| `detection_camera` | Index of fixed camera that detected checkerboard (-1 if none) |
| `camera_pose.R` | Helmet camera rotation in world frame (3x3) |
| `camera_pose.t` | Helmet camera translation in world frame (3x1, mm) |
| `skeleton_3d` | 32 joints in helmet camera coordinate frame (mm) |
| `skeleton_2d` | 32 joints projected onto helmet camera image (pixels) |

## Fusion Algorithm

1. **Timestamp sync** - Match frames within sync threshold (default 33ms)
2. **Coordinate transform** - Transform skeletons from each camera to primary camera space
3. **Body matching** - Match bodies across cameras by pelvis distance
4. **Joint fusion** - Weighted average of joint positions based on confidence

```
Camera 0 (Primary)          Camera 1 (Secondary)
     │                           │
     ▼                           ▼
   Body Tracking              Body Tracking
     │                           │
     │                    Transform to Primary
     │                           │
     └───────────┬───────────────┘
                 │
           Body Matching
                 │
           Joint Fusion
                 │
                 ▼
            Fused Skeleton
```

## Ego-View Pipeline

```
Fixed MKV 0 ──► Body Tracker ──► Skeleton ─────────────┐
Fixed MKV 1 ──► Body Tracker ──► Skeleton ─────────────┤
Fixed MKV 2 ──► Body Tracker ──► Skeleton ─────────────┤
                                                        ▼
                                           FuseBodiesAtTimestamp()
                                                        │
Fixed MKV 0 ──► Color+Depth ──► Detect Checkerboard ───┤
Fixed MKV 1 ──► Color+Depth ──► Detect Checkerboard ───┤
Fixed MKV 2 ──► Color+Depth ──► Detect Checkerboard ───┘
                                        │
                          Convert2DTo3D → TransformToWorld → ComputeCheckerboardPose
                                        │
                          candidatePose = checkerPose × T_checker_to_A
                                        │
                          FuseHelmetPoses() ──► Outlier rejection (median, 100mm)
                                        │      Weighted fusion (1/depth²)
                                        │
                          SmoothPose() ──► EMA (α=0.75, 200ms staleness guard)
                                        │
                                        ▼
Helmet MKV ──► Color Frame    3D joints: R^T × (P_world - t) → ego 3D
                   │          2D joints: k4a_calibration_3d_to_2d → ego 2D
                   ▼                    │
              Save image                ▼
                   └──────── Save annotation JSON
```

### Frame Synchronization

MKV files from different cameras have independent timelines. The processor uses **timestamp-interleaved advancement** — at each iteration, only the camera with the oldest unprocessed timestamp advances. This ensures:

- Fixed camera skeletons are fused at closely-matched timestamps
- Helmet frames use the most recent fused skeleton data (not stale data from a different point in time)
- No frames are skipped or duplicated regardless of per-camera frame rate differences

Without interleaved advancement, lock-step processing (advancing all cameras simultaneously) can cause the helmet image to be several frames ahead or behind the skeleton data, producing visibly desynchronized 2D/3D annotations.

### Helmet Pose Processing

When multiple fixed cameras detect the checkerboard simultaneously, their pose estimates are combined for robustness:

1. **Weighted fusion** — Each detection is weighted by `1/depth²` (closer cameras have less depth noise)
2. **Outlier rejection** — With 3+ detections, candidates whose translation is >100mm from the per-axis median are rejected before fusion. If all are rejected, the closest to the median is kept as fallback
3. **EMA temporal smoothing** — The fused pose is blended with the previous frame's pose using an exponential moving average (α=0.75: 75% current, 25% previous). A 200ms staleness guard prevents ghost positions after detection gaps — if more than 200ms has elapsed since the last detection, the new pose snaps directly without blending

### Transform Math

```
World frame = primary fixed camera (device 0, identity in calibration.json)

Fixed camera N → World:
  P_world = R_cam × P_cam + t_cam    (from calibration.json)

Checkerboard 3D corners (detected on fixed camera N):
  P_cam = Convert2DTo3D(corners2D, depth)
  P_world = R_cam × P_cam + t_cam

Checkerboard pose in world:
  checkerR, checkerT = ComputeCheckerboardPose(P_world_corners)

Helmet camera pose in world:
  helmetR = checkerR × T_checker_to_A.R
  helmetT = checkerR × T_checker_to_A.t + checkerT

World → Helmet camera (3D joints):
  P_helmet = helmetR^T × (P_world - helmetT)

Helmet camera 3D → 2D (image projection):
  k4a_calibration_3d_to_2d(&helmetCalib, &P_helmet, COLOR, COLOR, &P_2d)
```

## Workflow

Complete workflow from recording to analysis:

```bash
# 1. Record MKV files (using multi_device_recorder)
multi_device_recorder.exe --output ./recordings --session exp01

# 2. Process offline with fusion
multi_device_offline_processor.exe \
    --calibration calibration.json \
    --output skeleton_exp01.csv \
    recordings/recording_cam0_*.mkv \
    recordings/recording_cam1_*.mkv

# 3. (Optional) Generate ego-view dataset
multi_device_offline_processor.exe \
    --calibration calibration.json \
    --helmet-serial CL3FC3100HN \
    --t-checker-to-a T_checker_to_A.json \
    --helmet-cb-rows 4 --helmet-cb-cols 5 --helmet-cb-square 30 \
    --ego-output ego_dataset/ \
    recordings/recording_cam0_*.mkv \
    recordings/recording_cam1_*.mkv \
    recordings/recording_cam2_*.mkv \
    recordings/recording_helmet_*.mkv

# 4. Sync with HMD data
python sync_skeleton_hmd.py \
    --skeleton skeleton_exp01.csv \
    --hmd hmd_exp01.csv \
    --output synced_exp01.csv
```

## Requirements

- Orbbec Femto Bolt cameras with K4A Wrapper
- OrbbecSDK K4A Wrapper v1.10.5+
- Azure Kinect Body Tracking SDK 1.1.2
- ONNX Runtime (for GPU processing)
- OpenCV 4.12.0 (for ego-view mode)

## Performance

Processing speed depends on:
- Number of MKV files (cameras)
- Recording length
- Processing mode (GPU vs CPU)

Typical performance with DirectML:
- Single camera: ~30 FPS (real-time)
- Dual camera: ~15-20 FPS

With ego-view mode enabled, checkerboard detection adds ~10-20ms per frame.

Note: Processing is sequential, not real-time. A 10-minute recording takes ~10-20 minutes to process.
