# Data Synchronization Scripts

Post-processing tools for synchronizing skeleton tracking data with HMD data.

## Overview

When recording body tracking from Orbbec cameras (external app) and HMD data from Unity separately, there may be timing offsets between the two data streams. This script aligns them using:

1. **Timestamp-based alignment**: Both systems record epoch timestamps (ms)
2. **Cross-correlation**: Automatically estimates delay by correlating head joint position with HMD position

## Setup

```bash
pip install -r requirements.txt
```

## Usage

### Basic Usage

```bash
python sync_skeleton_hmd.py --skeleton skeleton_data.csv --hmd HMD_Walk_P01.csv --output synced_data.csv
```

### With Manual Delay

```bash
python sync_skeleton_hmd.py --skeleton skeleton_data.csv --hmd HMD_Walk_P01.csv --delay 150 --output synced_data.csv
```

### Options

| Option | Description |
|--------|-------------|
| `--skeleton` | Skeleton CSV file from multi_device_body_viewer |
| `--hmd` | HMD CSV file from Unity DataRecorder |
| `--output` | Output synchronized CSV file |
| `--delay` | Manual delay in ms (positive = skeleton leads) |
| `--tolerance` | Timestamp matching tolerance in ms (default: 50) |
| `--no-crosscorr` | Skip automatic delay estimation |

## CSV Formats

### Skeleton Data (from multi_device_body_viewer)

```csv
timestamp_ms,device_index,body_id,joint_id,joint_name,pos_x,pos_y,pos_z,rot_w,rot_x,rot_y,rot_z,confidence
1704067200000,0,100,0,PELVIS,0.123,0.456,0.789,1.0,0.0,0.0,0.0,2
...
```

### HMD Data (from Unity DataRecorder)

```csv
timestamp_ms,frame,unity_time,hmd_pos_x,hmd_pos_y,hmd_pos_z,hmd_rot_x,hmd_rot_y,hmd_rot_z,...
1704067200000,0,0.0000,0.1,1.6,0.2,0.0,90.0,0.0,...
...
```

### Output (Synchronized)

Combined data with all HMD columns plus skeleton joint positions prefixed with `skel_j{joint_id}_{x/y/z}`.

## Recording Workflow

1. **Start multi_device_body_viewer** (external app)
   ```bash
   multi_device_body_viewer.exe --primary CL3FC3100HN --calibration calibration.json
   ```

2. **Start Unity** with DataRecorder enabled

3. **Press R** in both applications to start recording

4. **Perform movements** (ensure head movement for cross-correlation)

5. **Press R** in both applications to stop recording

6. **Run synchronization**
   ```bash
   python sync_skeleton_hmd.py --skeleton skeleton_data_*.csv --hmd HMD_*.csv
   ```

## Delay Estimation

The script uses cross-correlation on the Y-axis (vertical) movement of:
- Skeleton: HEAD joint (joint_id 26) or PELVIS (joint_id 0)
- HMD: `hmd_pos_y` column

A positive delay means the skeleton data is ahead of HMD data.
