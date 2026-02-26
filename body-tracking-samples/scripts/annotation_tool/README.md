# Ego Dataset Annotation Tool

Visual drag-and-drop editor for correcting 2D skeleton annotations produced by the ego-dataset pipeline (`batch_ego_dataset.py` + `multi_device_offline_processor`).

Manually editing per-frame JSON files is impractical for 300+ frame sessions. This tool provides an interactive viewport with rubber-band multi-joint selection, keyframe interpolation, full undo/redo, and HMD overlay -- reducing manual correction effort by ~90%.

![Layout Diagram](./annotation_tool_layout.png)

```
┌─────────────────────────────────────────────────────────────┐
│ Menu: File | Edit | View | Navigate                         │
├───────────────────────────────────────┬─────────────────────┤
│                                       │  Properties Panel   │
│                                       │  ┌───────────────┐  │
│        QGraphicsView Viewport         │  │ Joint Tree     │  │
│        (image + draggable skeleton)   │  │ Name|Conf|Vis|K│  │
│        - zoom: mouse wheel            │  │ ...32 joints   │  │
│        - pan: middle-drag             │  ├───────────────┤  │
│        - rubber-band: left-drag       │  │ HMD Info      │  │
│        - drag joints: left-click      │  │ pos, rot, spd │  │
│                                       │  └───────────────┘  │
├───────────────────────────────────────┴─────────────────────┤
│ [▓▓▓░░░░░░░░░░░░░░░░░░░░░░░░░░░] Color bar (frame status) │
│ [═══════════●═══════════════════] Frame slider              │
│ Frame 42 / 310                                              │
└─────────────────────────────────────────────────────────────┘
```

## Requirements

| Package | Version | Notes |
|---------|---------|-------|
| Python | >= 3.9 | |
| PySide6 | >= 6.5.0 | `pip install PySide6` |
| scipy | >= 1.7.0 | For cubic spline interpolation (already installed) |
| numpy | any | Already installed |
| opencv-python | any | Already installed |
| pandas | any | Already installed (for HMD data) |

Install the one new dependency:

```bash
pip install PySide6
```

## Usage

```bash
# Open a specific ego_dataset directory
python annotate_ego_dataset.py --input batch_out/Dancing1_20260214_001511/ego_dataset/

# Open the session browser for a batch directory
python annotate_ego_dataset.py --batch-dir batch_out/

# Open a named session from a batch directory
python annotate_ego_dataset.py --batch-dir batch_out/ --session Dancing1_20260214_001511

# Launch with no arguments (use File > Open in the GUI)
python annotate_ego_dataset.py
```

### CLI Arguments

| Argument | Description |
|----------|-------------|
| `--input, -i` | Path to an `ego_dataset/` directory to open immediately |
| `--batch-dir, -b` | Path to a batch output directory (opens session browser) |
| `--session, -s` | Session name within `--batch-dir` to load directly |

## Keyboard Shortcuts

### Navigation

| Key | Action |
|-----|--------|
| `A` / `Left` | Previous frame |
| `D` / `Right` | Next frame |
| `PgUp` | Skip back 10 frames |
| `PgDown` | Skip forward 10 frames |
| `Home` | First frame |
| `End` | Last frame |
| `Space` | Play / pause (~30 fps) |

### Editing

| Key | Action |
|-----|--------|
| `Ctrl+Z` | Undo |
| `Ctrl+Y` | Redo |
| `Delete` | Reset selected joint(s) to original position |
| `K` | Toggle keyframe on selected joint(s) |
| `Ctrl+I` | Apply interpolation between keyframes |
| `Escape` | Deselect all joints |

### View

| Key | Action |
|-----|--------|
| `F` | Fit image to window |
| Mouse wheel | Zoom in/out |
| Middle-drag | Pan viewport |

### File

| Key | Action |
|-----|--------|
| `Ctrl+O` | Open session browser |
| `Ctrl+S` | Save |

## Mouse Interaction

| Action | Result |
|--------|--------|
| Left-click joint | Select it (deselects others) |
| Ctrl+click joint | Add/remove from selection |
| Left-drag on empty space | Rubber-band selection rectangle |
| Left-drag a selected joint | Move all selected joints together |
| Middle-drag | Pan the viewport |
| Mouse wheel | Zoom in/out centered on cursor |

## Workflow

### Basic Joint Correction

1. Open a session (`Ctrl+O` or `--input`)
2. Navigate to a frame with misaligned joints (`A`/`D` or slider)
3. Click a joint to select it
4. Drag it to the correct position
5. Bones update in real-time during the drag
6. `Ctrl+Z` to undo if needed
7. `Ctrl+S` to save

### Multi-Joint Editing

1. Draw a rubber-band rectangle around several joints to select them
2. Drag any one of the selected joints -- all move together by the same offset
3. Press `Delete` to reset all selected joints to their original positions
4. Press `K` to toggle keyframes on all selected joints at once
5. Undo reverts the entire group operation in one step

### Keyframe Interpolation

This is the most powerful feature. Instead of manually adjusting every frame, you set a few keyframes and let the tool fill in the rest:

1. Navigate to a frame and drag a joint to the correct position
2. Press `K` to mark it as a keyframe
3. Navigate to another frame (e.g., 10--20 frames later), correct the joint, press `K`
4. Repeat for a few key positions across the session
5. Select the joint and press `Ctrl+I` to open the interpolation dialog
6. Choose **Linear** or **Cubic Spline** mode and click OK
7. All intermediate frames are filled automatically
8. `Ctrl+Z` reverts the entire interpolation in one step

**Interpolation Modes**:

| Mode | Description | Best For |
|------|-------------|----------|
| Linear | Straight-line interpolation (`numpy.interp`) | Smooth, constant-speed movements |
| Cubic Spline | Smooth curve through keyframes (`scipy.CubicSpline`) | Natural, accelerating/decelerating movements |

Only frames between keyframes are interpolated -- no extrapolation beyond the first/last keyframe.

## Timeline Color Bar

The thin bar above the slider shows per-frame status at a glance:

| Color | Meaning |
|-------|---------|
| Blue | Frame has a keyframe marker |
| Orange | Frame has been edited |
| Green | Checkerboard was detected in this frame |
| Gray | Normal, unedited frame |

Click anywhere on the color bar to jump to that frame.

## Properties Panel

### Joint Tree

Displays all 32 body-tracking joints grouped by body part (Spine, Head, Left Arm, Right Arm, Left Leg, Right Leg). Each joint row shows:

| Column | Description |
|--------|-------------|
| Name | Joint name (e.g., `ELBOW_LEFT`). Excluded distal joints marked with `*` |
| Conf | Confidence level (0--3) from the body tracker |
| Vis | Checkbox: whether the joint is visible in the annotation |
| KF | Checkbox: whether this frame is a keyframe for this joint |
| U | Horizontal pixel coordinate |
| V | Vertical pixel coordinate |

Click a joint in the tree to select it in the viewport. Check/uncheck Vis or KF to toggle visibility or keyframe status (undoable).

### HMD Info

When `synced_data.csv` is available in the session directory, the HMD panel displays:

- **Position**: HMD (x, y, z) in meters
- **Rotation**: Euler angles (pitch, yaw, roll) in degrees
- **Speed**: HMD velocity in m/s

Values are mapped to the current ego frame by linear ratio.

## Save Behavior

- Only frames with actual changes (position or visibility differs from original) are written
- A `.json.bak` backup is created for each modified file before overwriting
- Unsaved changes trigger a confirmation dialog on close or session switch
- The window title shows `*` when there are unsaved edits

## File Structure

```
body-tracking-samples/scripts/
    annotate_ego_dataset.py              # Entry point
    annotation_tool/
        __init__.py                      # Package init
        constants.py                     # Re-exported constants + Qt colors
        data_model.py                    # AnnotationModel (sparse edit overlay)
        undo_commands.py                 # Undo/redo command classes
        interpolation.py                 # Linear + cubic spline engine
        viewport.py                      # QGraphicsView + JointItem + BoneItem
        timeline.py                      # Color bar + slider + frame label
        properties.py                    # Joint tree + HMD info panel
        session_browser.py               # Batch session picker dialog
        app.py                           # Main window (assembles everything)
```

## Architecture

### Data Flow

```
EgoDataset (read-only, loaded from ego_dataset/ directory)
    │
    └──► AnnotationModel (sparse edit overlay)
              │
              ├── get_joint_2d(frame, jid) → checks edits first, falls back to EgoDataset
              ├── set_joint_2d(frame, jid, u, v) → stores edit, emits joint_moved
              ├── keyframe tracking: Dict[joint_id, Set[frame_indices]]
              ├── dirty tracking: Set of modified frame indices
              │
              └── Signals → Viewport, Timeline, Properties all update reactively
```

### Signal Flow (joint drag example)

```
User drags joint(s) → JointItem.mouseReleaseEvent
    → AnnotationViewport.finalize_group_drag()
        → emits group_drag_finished(moves_list)
            → AnnotationMainWindow._on_group_drag()
                → pushes MoveJointCommand or MoveMultipleJointsCommand to QUndoStack
                    → command.redo() calls AnnotationModel.set_joint_2d()
                        → model emits joint_moved(frame, joint_id)
                            → Viewport repositions joint + updates bones
                            → Properties panel refreshes coordinates
                            → Timeline color bar repaints
```

### Undo Commands

| Command | Trigger | Description |
|---------|---------|-------------|
| `MoveJointCommand` | Single joint drag | Stores old/new (u, v) for one joint |
| `MoveMultipleJointsCommand` | Group drag or multi-reset | Stores old/new (u, v) for N joints at one frame |
| `ToggleVisibilityCommand` | Vis checkbox in properties | Stores old/new visibility flag |
| `SetKeyframeCommand` | `K` key or KF checkbox | Stores keyframe on/off |
| `BatchMoveCommand` | `Ctrl+I` interpolation | Stores old/new positions across M frames for one joint |

## Input Data Format

The tool reads `ego_dataset/` directories produced by the offline processor:

```
ego_dataset/
    metadata.json           # Session metadata
    images/
        frame_000000.jpg    # Helmet camera images
        frame_000001.jpg
        ...
    annotations/
        frame_000000.json   # Per-frame skeleton annotations
        frame_000001.json
        ...
```

Each annotation JSON contains `skeleton_2d` (list of `{joint_id, u, v, confidence, visible}`) and `skeleton_3d` entries. The tool modifies the `skeleton_2d` fields (`u`, `v`, `visible`) in-place.
