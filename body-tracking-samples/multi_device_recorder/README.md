# Multi-Device MKV Recorder

Records raw depth data from multiple Orbbec Femto Bolt cameras to MKV files **without body tracking**. This allows for:

1. **Zero-latency recording** - No body tracking processing delay during capture
2. **Offline processing** - Body tracking is done later using `multi_device_offline_processor`
3. **Synchronized timestamps** - MKV files contain accurate device timestamps

## Why Use This?

When doing real-time body tracking, there's a 20-50ms processing delay between frame capture and skeleton output. For applications requiring precise synchronization (e.g., with HMD data), this delay can cause issues.

**Solution:** Record raw depth data without body tracking, then process offline. The MKV timestamps reflect actual capture time, not processing time.

## Build

1. Open `multi_device_recorder.sln` in Visual Studio
2. Restore NuGet packages (right-click solution → Restore NuGet Packages)
3. Build in Release mode
4. Run `copy_orbbec_dlls.bat` to replace Azure Kinect DLLs with Orbbec versions

## Usage

```bash
# Basic usage (records all connected cameras)
multi_device_recorder.exe

# Specify output directory and session name
multi_device_recorder.exe --output ./recordings --session experiment01

# Specify primary camera (sync hub master)
multi_device_recorder.exe --primary CL2K1234567

# Disable UDP listener
multi_device_recorder.exe --no-udp
```

### Command Line Options

| Option | Description |
|--------|-------------|
| `--output DIR` | Output directory for MKV files (default: current) |
| `--session NAME` | Session name prefix for recordings |
| `--primary SERIAL` | Serial number of PRIMARY camera |
| `--udp-port PORT` | UDP listen port (default: 9000) |
| `--no-udp` | Disable UDP listener |

### Runtime Controls

| Key | Action |
|-----|--------|
| `R` | Start/stop recording |
| `Q` / `ESC` | Quit |

### UDP Commands (from Unity)

| Command | Action |
|---------|--------|
| `TOGGLE_RECORD` | Toggle recording on/off |
| `START_RECORD` | Start recording |
| `STOP_RECORD` | Stop recording |

## Output Files

Recordings are saved as:
```
recording_cam0_<serial>_<session>_<timestamp>.mkv
recording_cam1_<serial>_<session>_<timestamp>.mkv
```

Example:
```
recording_cam0_CL2K1234567_experiment01_20240115_143022.mkv
recording_cam1_CL2K7654321_experiment01_20240115_143022.mkv
```

## Workflow

1. **Record**: Use this tool to capture synchronized MKV files from all cameras
2. **Process**: Use `multi_device_offline_processor` to extract body tracking data with fusion
3. **Sync**: Use `sync_skeleton_hmd.py` to align skeleton data with HMD data

```
┌─────────────────────┐     ┌────────────────────────────┐     ┌─────────────────┐
│ multi_device_       │ --> │ multi_device_              │ --> │ sync_skeleton_  │
│ recorder            │     │ offline_processor          │     │ hmd.py          │
│ (MKV files)         │     │ (skeleton CSV)             │     │ (synced CSV)    │
└─────────────────────┘     └────────────────────────────┘     └─────────────────┘
```

## Unity Integration

Start recording from Unity by sending UDP commands:

```csharp
using System.Net.Sockets;
using System.Text;

UdpClient client = new UdpClient();
byte[] data = Encoding.UTF8.GetBytes("TOGGLE_RECORD");
client.Send(data, data.Length, "127.0.0.1", 9000);
```

Or use the `RecordingSyncController.cs` script from `sample_unity_hmd_recorder`.

## Requirements

- Orbbec Femto Bolt cameras with K4A Wrapper
- OrbbecSDK K4A Wrapper v1.10.5+
- Multi-camera sync: Orbbec Sync Hub configured (Primary/Secondary)
