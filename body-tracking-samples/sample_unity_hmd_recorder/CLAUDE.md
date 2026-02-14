# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Unity application for recording HMD (Head-Mounted Display) and controller 6DOF data from Meta Quest devices. Designed to synchronize with external C++ body tracking applications via UDP.

## Requirements

- Unity 2021.3+
- Meta XR SDK v83+
- Meta Quest 2/3/Pro

## Key Scripts

| Script | Purpose |
|--------|---------|
| `HMDDataRecorder.cs` | Records HMD/controller 6DOF to CSV at configurable frame rate |
| `RecordingSyncController.cs` | UDP sync with external apps (port 9000 send, 9001 receive) |
| `RecordingStatusUI.cs` | VR overlay showing recording state and real-time 6DOF data |

## Architecture

### Data Flow
```
OVRCameraRig (Meta SDK)
    ├── centerEyeAnchor → HMD position/rotation
    ├── leftHandAnchor  → Left controller 6DOF
    └── rightHandAnchor → Right controller 6DOF
                ↓
        HMDDataRecorder (CSV output)
                ↓
        RecordingStatusUI (VR overlay)
```

### UDP Sync Protocol
```
Unity ──TOGGLE_RECORD──► multi_device_body_viewer (port 9000)
Unity ◄──START_RECORD─── confirmation (port 9001)
Unity ◄──STOP_RECORD──── confirmation (port 9001)
```

Commands: `TOGGLE_RECORD`, `START_RECORD`, `STOP_RECORD`, `CYCLE_CAMERA`

## Controls

| Key | Action |
|-----|--------|
| R | Start/Stop recording (also sends UDP sync) |
| K | Cycle camera view (UDP command) |
| ESC | Quit application |

## Output Format

CSV location: `Assets/Recordings/HMD_{participantID}_{sessionName}_{timestamp}.csv`

Columns: `timestamp_ms, frame, unity_time, hmd_pos_xyz, hmd_rot_xyzw, left_pos_xyz, left_rot_xyzw, right_pos_xyz, right_rot_xyzw`

## Related Projects

- `../multi_device_body_viewer/` - C++ body tracking (UDP sync target)
- `../scripts/` - Python synchronization scripts for skeleton/HMD alignment
- `../sample_unity_bodytracking/` - Unity body tracking with Orbbec cameras

---

## Vibe Unity Integration

⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄VIBE-UNITY⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄

# Vibe Unity Integration Guide (Auto-generated - v2.0.0)

## Claude-Code Automated Workflow

### Primary Development Pattern
```bash
# 1. Validate compilation after code changes
./claude-compile-check.sh
# Exit codes: 0=success, 1=errors, 2=timeout, 3=script error

# 2. Create scenes via JSON (automatic processing)
echo '{"action":"create-scene","name":"TestScene","path":"Assets/Scenes"}' > .vibe-unity/commands/test.json

# 3. Verify results (check logs after 3 seconds)
sleep 3 && cat .vibe-unity/commands/logs/latest.log
```

### Automated Success/Failure Detection
- ✅ **Success Indicators**: Log contains "Scene created successfully" or "STATUS: SUCCESS"
- ❌ **Failure Indicators**: Log contains "ERROR", "FAILED", or compilation errors
- 🔄 **Claude Action**: On failure, immediately report specific error and stop workflow

### File Locations for Claude-Code
- **Compilation Check**: `./claude-compile-check.sh` (auto-installed)
- **JSON Commands**: Drop files in `.vibe-unity/commands/` directory
- **Log Verification**: Check `.vibe-unity/commands/logs/latest.log`
- **Coverage Reports**: `.vibe-unity/commands/coverage-analysis/`
- **Test Template**: `.vibe-unity/commands/test-scene-creation.json`

### Current Component Support (v2.0.0)
- ✅ **UI**: Canvas, Button, Text, Image, ScrollView, TextMeshPro
- ✅ **3D**: Cube, Sphere, Plane, Cylinder, Capsule, Camera, Light
- ⚠️ **Partial**: Rigidbody, Colliders
- ❌ **Missing**: ParticleSystem, custom scripts, animations

### JSON Command Examples for Claude-Code
```json
// Basic scene creation
{"action":"create-scene","name":"MyScene","path":"Assets/Scenes"}

// Multiple commands in batch file
{
  "commands": [
    {"action":"create-scene","name":"MenuScene","path":"Assets/Scenes/UI"},
    {"action":"add-canvas","name":"MainCanvas"},
    {"action":"add-button","name":"PlayButton","parent":"MainCanvas","text":"Play"}
  ]
}

// Add 3D objects
{"action":"add-cube","name":"TestCube","position":[0,1,0],"scale":[2,2,2]}
```

### Claude-Code Decision Tree
1. **After C# changes**: Run `./claude-compile-check.sh`
   - Exit code 0: Proceed with scene creation
   - Exit code 1: Fix compilation errors immediately, report to user
   - Exit code 2+: Report timeout/system issues to user

2. **For scene operations**: Use JSON commands with automatic verification
   - Success: Continue workflow
   - Failure: Report specific error from logs, ask user for guidance

3. **Error Handling**: 
   - **Compilation errors**: STOP and fix errors
   - **Scene creation failures**: STOP, report error, ask user to check Unity Console
   - **Missing components**: Note in summary, continue with supported components

### Development Workflow Status
- **File Watcher**: ✅ ENABLED (automatic JSON processing)
- **Compilation Check**: ✅ AUTOMATED (`./claude-compile-check.sh`)
- **Log Verification**: ✅ AUTOMATED (structured log parsing)
- **Error Detection**: ✅ AUTOMATED (exit codes + log analysis)

## Automated Claude Instructions
* **ALWAYS** run `./claude-compile-check.sh` after modifying C# scripts
* **ONLY proceed** if compilation check returns exit code 0
* **VERIFY scene creation** by checking `.vibe-unity/commands/logs/latest.log` for success/error messages
* **REPORT failures immediately** with specific error details from logs
* **DO NOT** create .meta files unless explicitly requested
* **ASK USER** for guidance only when encountering system-level failures or unsupported features

## For Detailed Usage
- **Full Documentation**: [Package README](./Packages/com.ricoder.vibe-unity/README.md)
- **JSON Schema Examples**: [Package Test Files](./Packages/com.ricoder.vibe-unity/.vibe-unity/commands/)
- **Coverage Analysis**: Check latest report in `.vibe-unity/commands/coverage-analysis/`

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^VIBE-UNITY^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

