# Unity HMD Recorder

Standalone Unity application for recording HMD (Head-Mounted Display) and controller 6DOF data from Meta Quest devices.

## Features

- Record HMD position and rotation (6DOF)
- Record left/right controller position and rotation (6DOF)
- UDP synchronization with external applications
- Configurable recording frame rate
- CSV output with epoch timestamps

## Requirements

- Unity 2021.3 or later
- Meta XR SDK v83+
- Meta Quest 2/3/Pro

## Scripts

| Script | Description |
|--------|-------------|
| `HMDDataRecorder.cs` | Records HMD and controller 6DOF data to CSV |
| `RecordingSyncController.cs` | UDP sync with external applications |
| `RecordingStatusUI.cs` | On-screen recording status display |

## HMDDataRecorder

### Setup

1. Add `HMDDataRecorder` component to a GameObject
2. Ensure `OVRCameraRig` is in the scene
3. Configure in Inspector:
   - Participant ID
   - Session Name
   - Frame Rate (default: 30 Hz)

### Controls

| Key | Action |
|-----|--------|
| R | Start/Stop recording |
| ESC | Quit application |

### Output CSV Format

```csv
timestamp_ms,frame,unity_time,hmd_pos_x,hmd_pos_y,hmd_pos_z,hmd_rot_x,hmd_rot_y,hmd_rot_z,hmd_rot_w,left_pos_x,left_pos_y,left_pos_z,left_rot_x,left_rot_y,left_rot_z,left_rot_w,right_pos_x,right_pos_y,right_pos_z,right_rot_x,right_rot_y,right_rot_z,right_rot_w
```

Output location: `Assets/Recordings/HMD_{participantID}_{sessionName}_{timestamp}.csv`

## UDP Sync with External Applications

Synchronize recording start/stop with external applications (e.g., `multi_device_body_viewer`) via UDP.

### How It Works

```
Unity (R key) ──UDP:9000──► multi_device_body_viewer
                           ↓ (toggle recording)
Unity ◄──UDP:9001────────── (confirmation)
```

### Setup

1. Add `RecordingSyncController` component to a GameObject
2. Configure in Inspector:
   - Target IP: `127.0.0.1` (local) or remote IP
   - Send Port: `9000` (matches multi_device_body_viewer)
   - Receive Port: `9001`
3. Link `HMDDataRecorder` reference (auto-finds if null)

### Protocol

| Command | Direction | Description |
|---------|-----------|-------------|
| `TOGGLE_RECORD` | Unity → C++ | Toggle recording |
| `START_RECORD` | C++ → Unity | Recording started |
| `STOP_RECORD` | C++ → Unity | Recording stopped |
| `CYCLE_CAMERA` | Unity → C++ | Cycle camera view |

### Usage

1. Start `multi_device_body_viewer.exe` (listens on UDP 9000)
2. Start Unity application
3. Press `R` in Unity
   - Sends `TOGGLE_RECORD` to multi_device_body_viewer
   - Both applications start recording simultaneously
4. Press `R` again to stop both

## Data Synchronization

After recording, synchronize HMD data with external skeleton data using the sync script:

```bash
cd ../scripts

# Inter-point distance method (coordinate-invariant)
python sync_skeleton_hmd.py --skeleton skeleton_data.csv --hmd HMD_P01_Session01_*.csv --method distance

# With visualization
python sync_skeleton_hmd.py --skeleton skeleton_data.csv --hmd HMD_*.csv --method distance --plot
```

See [scripts/README.md](../scripts/README.md) for detailed usage.

## Related Projects

- [multi_device_body_viewer](../multi_device_body_viewer/) - C++ multi-camera body tracking
- [sample_unity_bodytracking](../sample_unity_bodytracking/) - Unity body tracking with Orbbec
- [scripts](../scripts/) - Python synchronization scripts
