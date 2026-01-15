# Multi-Device Offline Body Tracking Processor

Processes MKV recordings from multiple Orbbec Femto Bolt cameras with:

1. **Body tracking** - Extracts skeleton data from each MKV file
2. **Calibration-based fusion** - Transforms skeletons to common coordinate system
3. **Multi-camera fusion** - Combines skeletons from multiple cameras for better accuracy

## Why Use This?

- **No processing latency** - Timestamps in MKV files are capture times, not processing times
- **Better accuracy** - Multiple cameras can see different angles, fusion improves joint confidence
- **Reproducible results** - Same MKV files always produce same output

## Build

1. Open `multi_device_offline_processor.sln` in Visual Studio
2. Restore NuGet packages (right-click solution → Restore NuGet Packages)
3. Build in Release mode
4. Run `copy_orbbec_dlls.bat` to replace Azure Kinect DLLs with Orbbec versions

## Usage

```bash
# Single camera processing
multi_device_offline_processor.exe recording_cam0.mkv --output skeleton.csv

# Multi-camera with fusion
multi_device_offline_processor.exe --calibration calib.json --output skeleton.csv \
    recording_cam0.mkv recording_cam1.mkv

# Specify processing mode
multi_device_offline_processor.exe --mode DirectML --calibration calib.json \
    --output skeleton.csv recording_cam0.mkv recording_cam1.mkv
```

### Command Line Options

| Option | Description |
|--------|-------------|
| `--calibration FILE` | Calibration JSON file (required for fusion) |
| `--output FILE` | Output CSV file (default: output.csv) |
| `--mode MODE` | Processing mode: CPU, CUDA, DirectML (default), TensorRT |
| `--sync-threshold MS` | Max timestamp difference for sync (default: 33ms) |

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

## Output CSV Format

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

# 3. Sync with HMD data
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

## Performance

Processing speed depends on:
- Number of MKV files (cameras)
- Recording length
- Processing mode (GPU vs CPU)

Typical performance with DirectML:
- Single camera: ~30 FPS (real-time)
- Dual camera: ~15-20 FPS

Note: Processing is sequential, not real-time. A 10-minute recording takes ~10-20 minutes to process.
