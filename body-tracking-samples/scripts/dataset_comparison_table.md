# Comparison of Egocentric Body-Pose Datasets

## Full Table - Multimodal & HMD Perspective

| Dataset | Year | Ego View | HMD Device | HMD DOF | Ego Sensor | Ego Depth | Exo Cams | Train Data | Train GT | Test Data | Test GT | Subj. | Acts. | Joints | 2D+ Vis |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| EgoCap | 2016 | Down | Custom rig | ‒ | Stereo fisheye RGB | ‒ | Multi-view studio | Real 75K | Captury MoCap | Real 25K | Captury MoCap | 8 | ‒ | 18 | - |
| Mo²Cap² | 2019 | Down | Custom rig | ‒ | Mono fisheye RGB | ‒ | MoCap studio | Synth 530K | Synthetic render | Real 5.6K | Captury MoCap | ~8 | 8 | 15 | - |
| xR-EgoPose | 2019 | Down | Simulated | ‒ | Mono fisheye RGB | Synth | ‒ | Synth 383K | Synthetic render | Synth (subset) | Synthetic render | 46 | 9 | 22 | - |
| GlobalEgoMocap | 2021 | Down | Custom rig | ‒ | Mono fisheye RGB | ‒ | MoCap studio | Synth (prior) | Synthetic render | Real ~10K | Captury MoCap | ~3 | ‒ | 15 | - |
| EgoPW | 2022 | Down | Custom rig | ‒ | Mono fisheye RGB | ‒ | 1 aux. cam | Real 318K | Pseudo-GT (optim.) | Real (subset) | Pseudo-GT (optim.) | 10 | 20 | 15 | - |
| EgoBody | 2022 | Forward | HoloLens 2 ×1 | 1×6DOF | HoloLens 2 RGB+D | Real | 3–5 Kinect | Real 219K | SMPL-X fit (Kinect) | Real (subset) | SMPL-X fit (Kinect) | 36 | ‒ | 22 | - |
| UnrealEgo | 2022 | Down | Simulated | ‒ | Stereo fisheye RGB | Synth | ‒ | Synth 357K | Synthetic render | Synth 48K | Synthetic render | 17 | 30 | 16 | - |
| SceneEgo | 2023 | Down | Custom rig | ‒ | Mono fisheye RGB | Synth | MoCap studio | Synth 320K + Real 60K | GTA render + pseudo-GT | Real ~28K | Captury MoCap | ~2 | ‒ | 15 | - |
| SLOPER4D | 2023 | Forward | Custom rig | ‒ | LiDAR + RGB | Real (LiDAR) | ‒ | Real 100K | SMPL fit (LiDAR+IMU) | Real (subset) | SMPL fit (LiDAR+IMU) | 12 | ‒ | 24 | - |
| Ego-Exo4D | 2024 | Forward | Aria glasses ×1 | 1×6DOF | Aria RGB+SLAM | ‒ | 4–5 GoPro | Real ~5M | Multi-view triang. | Real (subset) | Multi-view triang. | 740 | 8 | 17 | - |
| Nymeria | 2024 | Forward | Aria glasses ×1 | 1×6DOF | Aria RGB+ET+IMU | ‒ | ‒ | Real 260M | IMU suit (Xsens) | Real (subset) | IMU suit (Xsens) | 264 | 20 | 22 | - |
| EMHI | 2025 | Down | PICO 4 ×1 | 3×6 + 2×3DOF | PICO 4 stereo+IMU | ‒ | 8 Kinect | Real 3.07M | SMPL fit (8-cam) | Real (subset) | SMPL fit (8-cam) | 58 | 39 | 22 | Yes |
| **Ours** | 2026 | Down | Meta Quest 3 ×1 | 3×6DOF | Femto Bolt D+RGB | Real | 3 Femto Bolt | Real 132K | Depth fusion (3-cam) | Real (subset) | Depth fusion (3-cam) | 41 | 20 | 32 | Yes |

## Short Table - Downward Ego-View Datasets

| Dataset | Year | HMD Device | HMD DOF | Ego Depth | GT Source | Train Data | Test Data | Joints | 2D+Vis |
|---|---|---|---|---|---|---|---|---|---|
| EgoCap | 2016 | Custom rig | ‒ | ‒ | Captury MoCap | Real 75K | Real 25K | 18 | - |
| Mo²Cap² | 2019 | Custom rig | ‒ | ‒ | Synthetic render | Synth 530K | Real 5.6K | 15 | - |
| xR-EgoPose | 2019 | Simulated | ‒ | Synth | Synthetic render | Synth 383K | Synth (subset) | 22 | - |
| GlobalEgoMocap | 2021 | Custom rig | ‒ | ‒ | Synthetic render | Synth (prior) | Real ~10K | 15 | - |
| EgoPW | 2022 | Custom rig | ‒ | ‒ | Pseudo-GT (optim.) | Real 318K | Real (subset) | 15 | - |
| UnrealEgo | 2022 | Simulated | ‒ | Synth | Synthetic render | Synth 357K | Synth 48K | 16 | - |
| SceneEgo | 2023 | Custom rig | ‒ | Synth | GTA render + pseudo-GT | Synth 320K + Real 60K | Real ~28K | 15 | - |
| EMHI | 2025 | PICO 4 ×1 | 3×6 + 2×3DOF | ‒ | SMPL fit (8-cam) | Real 3.07M | Real (subset) | 22 | Yes |
| **Ours** | 2026 | Meta Quest 3 ×1 | 3×6DOF | Real | Depth fusion (3-cam) | Real 132K | Real (subset) | 32 | Yes |

## Key Differentiators of Our Dataset

- Real multi-camera depth-fusion GT -- 3 synchronized depth cameras; no SMPL fitting, no pseudo-labels, not synthetic.
- 32 joints (highest count) -- full Azure Kinect Body Tracking skeleton; most datasets provide 15-24 joints.
- Paired 3D + 2D + per-joint visibility -- every frame includes 3D coords, 2D projections, and visibility flags.
- Active ego depth + RGB (not fisheye) -- helmet-mounted Femto Bolt provides metric depth per ego frame.
- Synchronized HMD 6DOF tracking (3x6DOF) -- Meta Quest 3 HMD + 2 controllers recorded at capture rate.
- Helmet-local + world-space + camera pose -- three coordinate frames per frame.
- Real train and test with same GT source -- no domain gap between train and test supervision.
- 20 action classes x 41 participants -- broad activity vocabulary with real subjects.

## Corrections Applied (from paper verification)

| Dataset | Field | Before | After |
|---|---|---|---|
| xR-EgoPose | Joints | 16 | 22 (full skeleton; 16 is eval subset) |
| xR-EgoPose | Test | Synth + Real 3K | Synth (subset) -- no published real test set |
| xR-EgoPose | Subjects | 46+3 | 46 |
| UnrealEgo | Train | Synth 450K+ | Synth 357K (verified split) |
| UnrealEgo | Test | Synth (subset) | Synth 48K (verified split) |
| SceneEgo | Train | Synth 100K + Real 60K | Synth 320K + Real 60K |
| SceneEgo | Test | Real ~10K | Real ~28K |
| Ours | GT Source | Depth fusion (4-cam) | Depth fusion (3-cam) |
