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
| `--ego-fusion-mode MODE` | Ego skeleton fusion mode: `world` (default) or `local` |

### Ego Fusion Modes

| Mode | Description |
|------|-------------|
| `world` | Fuse skeletons in world space via extrinsic calibration, then transform to helmet frame (default, backward compatible) |
| `local` | Transform each camera's skeleton to helmet-local coordinates independently (using only camera-local checkerboard detection), then fuse. Eliminates extrinsic calibration errors from the skeleton→helmet projection path. |

#### Why `local` mode?

In `world` mode, the skeleton path traverses two calibrations: camera→world (extrinsic) then world→helmet (checkerboard). Extrinsic calibration errors (typically 5-15mm) accumulate and cause the projected 2D skeleton to drift from the actual body in the helmet image.

In `local` mode, each camera computes its own helmet pose from its own checkerboard detection and transforms its skeleton directly to helmet-local coordinates — the extrinsic calibration is never on the skeleton path. The world-frame pose is still computed for JSON metadata (camera_pose R/t) but does not affect skeleton accuracy.

#### Local-Mode Pipeline

```
For each fixed camera that detects the checkerboard:

  Camera N body tracker
       │
       ▼
  SelectBestBodyLocal()         ← phantom rejection + pelvis continuity (per-camera)
       │
  Per-camera temporal smoothing ← adaptive per-joint EMA in camera space
       │                          body switch detection (200mm threshold)
       │                          confidence carry-forward (5-frame TTL)
       │
  Detect checkerboard corners (2D)
       │
  Convert2DTo3D in CAMERA space (no world transform)
       │
  ComputeCheckerboardPose → checkerPose_cam
       │
  helmetPose_cam = checkerPose_cam ∘ T_checker_to_A
       │
  TransformBodyToHelmetLocal()
       │     P_helmet = R_helmet_cam^T × (P_cam - t_helmet_cam)
       │
       ▼
  CameraHelmetSkeleton { joints[32], weight = 1/depth², cameraIndex }
       │
       └──────────────────► Collect from all cameras
                                    │
                            Pelvis outlier rejection
                                    │  With 2+ candidates: reject if pelvis
                                    │  >150mm from per-axis median
                                    │
                            FuseHelmetLocalSkeletons()
                                    │  Confidence-weighted average per joint:
                                    │  w_total = Σ (camera_weight × conf_weight)
                                    │  P_fused = Σ (P_j × w) / w_total
                                    │  conf_fused = max(conf across cameras)
                                    │
                                    ▼
                            joints3D[32] in helmet-local frame
                                    │
                            ProjectSkeleton() → joints2D[32]
                                    │
                                    ▼
                            Save annotation JSON
```

#### Per-Camera State

Each fixed camera maintains independent smoothing state for local mode:

| State | Description |
|-------|-------------|
| `prevPelvisLocal` | Previous pelvis position (camera space) for body selection continuity |
| `prevSmoothedBodyLocal` | Previous smoothed skeleton (camera space) for EMA |
| `jointCarryCountLocal[32]` | Per-joint carry-forward counter (resets when joint regains confidence) |
| `prevSmoothedTimestampLocal` | Timestamp of last smoothed frame (staleness guard, 100ms) |

This means if one camera temporarily loses its checkerboard detection, the other cameras continue independently without affecting the first camera's smoothing state when it resumes.

#### Confidence Weights

Joint fusion uses confidence-weighted averaging with these weights:

| Confidence Level | Weight |
|-----------------|--------|
| None (0) | 0.0 |
| Low (1) | 0.25 |
| Medium (2) | 0.6 |
| High (3) | 1.0 |

Camera weight is `1/depth²` (inverse-square of average checkerboard depth in mm), with a 2x bonus for the camera used in the previous frame to reduce camera switching.

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
5. **Body selection** - Filter phantom bodies and select best candidate for ego-view

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
      SelectBestBody (confidence filter → multi-cam preference → spatial continuity)
                 │
                 ▼
            Fused Skeleton
```

### Body Selection (SelectBestBody)

When multiple bodies are detected, the selector applies three filters in order:

1. **Confidence filter** — Reject bodies where pelvis confidence = 0 (phantom/hallucination). Falls back to unfiltered list only if ALL bodies have zero confidence
2. **Multi-camera preference** — Prefer bodies seen by 2+ cameras (matchCount ≥ 2) over single-camera detections
3. **Spatial continuity** — Among candidates, select the body closest to the previous frame's pelvis position

This prevents phantom bodies (detected at 6m+ with all-zero confidence) from locking in via spatial continuity and blocking the real body for the entire session.

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
                          SmoothPose() ──► Camera-switch-aware EMA
                                        │    α=0.75 normal, α=0.15 on camera switch
                                        │
                                        ▼
                          SelectBestBody() ──► Confidence filter → multi-cam → spatial
                                        │
                          Skeleton Smoothing ──► Adaptive per-joint EMA
                                        │       + body switch detection (200mm)
                                        │       + confidence carry-forward (5-frame TTL)
                                        │
                                        ▼
Helmet MKV ──► Color Frame    3D joints: R^T × (P_world - t) → ego 3D
                   │          2D joints: k4a_calibration_3d_to_2d → ego 2D
                   ▼                    │      (min depth 50mm, off-screen clamping)
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

1. **Weighted fusion** — Each detection is weighted by `1/depth²` (closer cameras have less depth noise). A **2x same-camera consistency bonus** is applied to the camera used in the previous frame, reducing unnecessary camera switching
2. **Outlier rejection** — With 3+ detections, candidates whose translation is >100mm from the per-axis median are rejected before fusion. If all are rejected, the closest to the median is kept as fallback
3. **Camera-switch-aware EMA** — The fused pose is blended with the previous frame's pose using an exponential moving average. When the detection camera is the same as the previous frame, α=0.75 (75% current, 25% previous). When the detection camera **switches**, α=0.15 (15% current, 85% previous) to suppress the discontinuous pose jump that occurs when different cameras produce slightly different estimates. A 200ms staleness guard prevents ghost positions after detection gaps

### Skeleton Stabilization

After body selection, the ego-view skeleton is stabilized with three mechanisms:

1. **Adaptive per-joint smoothing** — Instead of uniform EMA, each joint group has its own alpha tuned to its expected motion range:

   | Joint Group | Joints | Alpha | Rationale |
   |-------------|--------|-------|-----------|
   | Core | Pelvis, spine, neck, clavicles, hips | 0.30 | Stability anchor — these joints move slowly |
   | Head/face | Head, nose, eyes, ears | 0.50 | Moderate — tracks head turns without lag |
   | Mid-limb | Shoulders, elbows, knees, ankles | 0.65 | Balanced — follows arm/leg motion |
   | Extremities | Wrists, hands, handtips, thumbs, feet | 0.85 | Responsive — fast-motion tracking (boxing, waving) |

2. **Body switch detection** — If the pelvis jumps >200mm between frames (indicating a phantom body or tracker glitch), the previous skeleton is carried forward instead. The 200mm threshold accommodates normal human motion (pelvis moves <150mm/frame at 30fps) while catching phantom switches (typically >300mm)

3. **Confidence carry-forward with TTL** — When a joint drops to confidence=0, its last good position is held for up to 5 frames (~167ms at 30fps). After the TTL expires, the raw position is accepted rather than freezing indefinitely at an increasingly wrong location

### 2D Projection

3D joints in helmet camera frame are projected to 2D using `k4a_calibration_3d_to_2d`. Two guards prevent extreme coordinate values:

- **Minimum depth threshold (50mm)** — Points closer than 50mm to the camera are set to (0, 0) with `visible=false`. This prevents the lens distortion model from producing extreme values for near-camera points
- **Off-screen clamping** — Projected coordinates outside the image are clamped to ±1x image dimensions (e.g., ±1920 for a 1920-wide image). The `visible` flag remains `false` but the stored coordinates stay within a reasonable range for downstream consumers

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
  if P_helmet.z < 50mm → (0, 0, visible=false)
  else → k4a_calibration_3d_to_2d(&helmetCalib, &P_helmet, COLOR, COLOR, &P_2d)
         clamp to ±image_dims if off-screen
```

## Workflow

### Single Session

```bash
# 1. Record MKV files (using multi_device_recorder)
multi_device_recorder.exe --output ./recordings --session exp01

# 2. Process offline with fusion
multi_device_offline_processor.exe \
    --calibration calibration.json \
    --output skeleton_exp01.csv \
    recordings/recording_cam0_*.mkv \
    recordings/recording_cam1_*.mkv

# 3. (Optional) Generate ego-view dataset (local-frame fusion for better accuracy)
multi_device_offline_processor.exe \
    --calibration calibration.json \
    --helmet-serial CL3FC3100HN \
    --t-checker-to-a T_checker_to_A.json \
    --helmet-cb-rows 4 --helmet-cb-cols 5 --helmet-cb-square 30 \
    --ego-fusion-mode local \
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

### Batch Ego Fusion Pipeline

For multi-session recordings (e.g., 20 poses in one capture session), the Python batch scripts automate the full pipeline from raw MKV files to annotated visualization videos.

```
┌──────────────────────────────────────────────────────────────────────────┐
│  RECORDING                                                               │
│                                                                          │
│  multi_device_recorder.exe          Unity HMD Recorder                   │
│   ├─ recording_cam0_DC_Dancing1_20260214_182242.mkv    HMD_Test_         │
│   ├─ recording_cam1_GD_Dancing1_20260214_182242.mkv    Dancing1_         │
│   ├─ recording_cam2_KV_Dancing1_20260214_182242.mkv    20260214_         │
│   └─ recording_cam3_CB_Dancing1_20260214_182242.mkv    182242.csv        │
│   (× N sessions: Dancing1, Boxing, Walking, ...)                         │
└──────────────────────────────┬───────────────────────────────────────────┘
                               │
                               ▼
┌──────────────────────────────────────────────────────────────────────────┐
│  BATCH PROCESSING            batch_ego_dataset.py                        │
│                                                                          │
│  For each session:                                                       │
│   1. multi_device_offline_processor.exe                                  │
│      → ego_dataset/ (images + 3D/2D skeleton annotations)                │
│      → output.csv (fused skeleton timeseries)                            │
│   2. sync_skeleton_hmd.py                                                │
│      → synced_data.csv (skeleton + HMD aligned by cross-correlation)     │
└──────────────────────────────┬───────────────────────────────────────────┘
                               │
                               ▼
┌──────────────────────────────────────────────────────────────────────────┐
│  VISUALIZATION               visualize_batch_ego_dataset.py              │
│                                                                          │
│  --mode preview   Interactive slider (matplotlib)                        │
│  --mode video     Batch MP4 export (OpenCV, ~35-45 fps)                  │
│                                                                          │
│  Output per session: 1280×960 MP4 with 4 panels                          │
│   ┌─────────────────┬─────────────────┐                                  │
│   │ Ego-View 2D     │ 3D Skeleton     │                                  │
│   │ (camera + skel) │ (ortho project) │                                  │
│   ├─────────────────┼─────────────────┤                                  │
│   │ HMD 3D Traj     │ HMD Timeseries  │                                  │
│   │ (trail + arrow) │ (height + speed) │                                  │
│   └─────────────────┴─────────────────┘                                  │
└──────────────────────────────────────────────────────────────────────────┘
```

**Commands:**

```bash
# 1. Batch process all sessions (auto-discovers MKVs + HMD CSVs by timestamp)
python batch_ego_dataset.py \
    --input-dir Test/ \
    --output-dir batch_out/

# 2. Resume after interruption (skips completed sessions)
python batch_ego_dataset.py \
    --input-dir Test/ \
    --output-dir batch_out/ \
    --resume

# 3. Export visualization videos for all sessions (~9s per 300-frame session)
python visualize_batch_ego_dataset.py \
    --batch-dir batch_out/ \
    --mode video

# 4. Interactive preview of one session
python visualize_batch_ego_dataset.py \
    --batch-dir batch_out/ \
    --session Dancing1_20260214_182242
```

**Output structure:**

```
batch_out/
├── batch_summary.json
├── Dancing1_20260214_182242/
│   ├── ego_dataset/                     # images/ + annotations/
│   ├── output.csv                       # fused skeleton CSV
│   ├── synced_data.csv                  # synchronized skeleton + HMD
│   ├── processor.log                    # processor stdout/stderr
│   └── Dancing1_20260214_182242_visualization.mp4
├── Gaming-Boxing_20260214_181615/
│   └── ...
└── Walking_20260214_182220/
    └── ...
```

See [scripts/README.md](../scripts/README.md) for detailed options on batch processing and visualization.

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
