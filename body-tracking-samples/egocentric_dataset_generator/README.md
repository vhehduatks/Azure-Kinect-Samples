# Egocentric Body Tracking Dataset Generator

헬멧에 장착된 카메라(A)의 1인칭 시점 영상과 고정 카메라(B,C)에서 추정한 스켈레톤을 결합하여 ML 학습용 데이터셋을 생성합니다.

## 개요

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│ Camera A    │     │ Camera B    │     │ Camera C    │
│ (Helmet)    │     │ (Fixed)     │     │ (Fixed)     │
│ RGB+Depth   │     │ Checkerboard│     │ Checkerboard│
└─────────────┘     │ Detection   │     │ Detection   │
       │            └──────┬──────┘     └──────┬──────┘
       │                   │                   │
       │            ┌──────┴───────────────────┘
       │            │
       ▼            ▼
┌─────────────┐  ┌─────────────┐
│ Color Image │  │ Helmet Pose │
│ (Output)    │  │ Estimation  │
└─────────────┘  └──────┬──────┘
                        │
                        ▼
                 ┌─────────────┐     ┌─────────────┐
                 │ skeleton.csv│────►│ Transform   │
                 │ (World Coord)     │ to Camera A │
                 └─────────────┘     └──────┬──────┘
                                            │
                                            ▼
                                     ┌─────────────┐
                                     │ 3D/2D JSON  │
                                     │ Annotations │
                                     └─────────────┘
```

## 입력 파일

| 파일 | 설명 |
|------|------|
| `camera_a.mkv` | 헬멧 카메라 영상 (color + depth + IR) |
| `camera_b.mkv` | 고정 카메라 B 영상 (체커보드 검출용) |
| `camera_c.mkv` | 고정 카메라 C 영상 (옵션, 체커보드 검출용) |
| `skeleton.csv` | multi_device_offline_processor 출력 (world 좌표) |
| `calibration.json` | B,C 카메라 extrinsics (multi_device_calibration 출력) |
| `t_checker_to_a.json` | 체커보드→카메라A 고정 변환 (사전 측정) |

## 출력 구조

```
output/
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

### Annotation JSON 형식

```json
{
  "frame_id": 0,
  "timestamp_usec": 1234567890,
  "image_file": "frame_000000.jpg",
  "camera_pose": {
    "R": [[r11,r12,r13], [r21,r22,r23], [r31,r32,r33]],
    "t": [tx, ty, tz]
  },
  "skeleton_3d": [
    {"joint_id": 0, "name": "PELVIS", "x": 100.0, "y": -500.0, "z": 1500.0, "confidence": 2},
    ...
  ],
  "skeleton_2d": [
    {"joint_id": 0, "name": "PELVIS", "u": 640.0, "v": 360.0, "confidence": 2, "visible": true},
    ...
  ],
  "checkerboard_detected": true,
  "checkerboard_cameras": ["B", "C"]
}
```

## 빌드

1. Visual Studio에서 `egocentric_dataset_generator.vcxproj` 열기
2. NuGet 패키지 복원 (솔루션 우클릭 → Restore NuGet Packages)
3. Release x64로 빌드
4. `copy_orbbec_dlls.bat` 실행하여 Orbbec DLL 및 OpenCV DLL 복사

## 사용법

```bash
egocentric_dataset_generator.exe ^
  --camera-a helmet.mkv ^
  --camera-b fixed_b.mkv ^
  --camera-c fixed_c.mkv ^
  --skeleton skeleton.csv ^
  --calibration calibration.json ^
  --t-checker-to-a t_checker_a.json ^
  --output ./dataset ^
  --checkerboard-rows 6 ^
  --checkerboard-cols 9
```

### 명령줄 옵션

| 옵션 | 필수 | 설명 |
|------|------|------|
| `--camera-a FILE` | O | 헬멧 카메라 MKV 파일 |
| `--camera-b FILE` | O | 고정 카메라 B MKV 파일 |
| `--camera-c FILE` | - | 고정 카메라 C MKV 파일 (옵션) |
| `--skeleton FILE` | O | 스켈레톤 CSV 파일 |
| `--calibration FILE` | O | 캘리브레이션 JSON 파일 |
| `--t-checker-to-a FILE` | O | 체커보드→카메라A 변환 JSON |
| `--output DIR` | O | 출력 디렉토리 |
| `--checkerboard-rows N` | - | 체커보드 내부 코너 행 수 (기본: 6) |
| `--checkerboard-cols N` | - | 체커보드 내부 코너 열 수 (기본: 9) |
| `--max-frames N` | - | 최대 처리 프레임 수 (기본: 전체) |
| `--skip-no-detection` | - | 체커보드 미검출 프레임 건너뛰기 |

## T_checker_to_A 측정 방법

체커보드와 헬멧 카메라 A 사이의 고정 변환을 사전에 측정해야 합니다.

### 방법 1: 직접 측정
1. 체커보드 중심과 카메라 A 렌즈 사이의 거리를 자로 측정
2. 체커보드 평면과 카메라 광축 사이의 각도 측정
3. JSON 파일로 저장:

```json
{
  "rotation": [[r11,r12,r13], [r21,r22,r23], [r31,r32,r33]],
  "translation": [tx, ty, tz]
}
```

### 방법 2: 캘리브레이션 (권장)
1. 체커보드가 카메라 A에 보이는 상태에서 촬영
2. OpenCV로 체커보드 검출 및 solvePnP 실행
3. 결과 R, t를 JSON으로 저장

## 핵심 알고리즘

### 1. 프레임 동기화
Camera A의 타임스탬프를 기준으로 B, C, skeleton 매칭:
- 임계값: 10ms (Sync Hub 사용 시)
- Skeleton 데이터는 선형 보간 적용

### 2. 헬멧 포즈 추정
매 프레임마다 다음 과정 수행:
1. B/C 카메라에서 체커보드 코너 검출 (OpenCV)
2. 2D 코너 → 3D 변환 (depth 사용)
3. 3D 포인트 → World 좌표 변환 (calibration.json 적용)
4. 체커보드 pose 계산 (centroid + SVD)
5. Camera A pose = T_checker_world × T_checker_to_A

### 3. 스켈레톤 좌표 변환
World 좌표계의 스켈레톤을 Camera A 좌표계로 변환:
```
P_A = R_A^T × (P_world - t_A)
```

### 4. 2D 투영
K4A calibration API를 사용한 왜곡 적용 투영:
```cpp
k4a_calibration_3d_to_2d(&calibration, &point3d,
    K4A_CALIBRATION_TYPE_COLOR, K4A_CALIBRATION_TYPE_COLOR,
    &point2d, &valid);
```

## 워크플로우

전체 데이터 수집 및 처리 워크플로우:

```bash
# 1. 카메라 캘리브레이션 (최초 1회)
multi_device_calibration.exe --rows 6 --cols 9 --output calibration

# 2. 녹화 (헬멧 + 고정 카메라)
multi_device_recorder.exe --output ./recordings --session exp01

# 3. 오프라인 스켈레톤 추정
multi_device_offline_processor.exe ^
    --calibration calibration.json ^
    --output skeleton_exp01.csv ^
    recordings/recording_cam*.mkv

# 4. 데이터셋 생성
egocentric_dataset_generator.exe ^
    --camera-a recordings/helmet_exp01.mkv ^
    --camera-b recordings/fixed_b_exp01.mkv ^
    --skeleton skeleton_exp01.csv ^
    --calibration calibration.json ^
    --t-checker-to-a t_checker_a.json ^
    --output ./dataset_exp01
```

## 출력 검증

생성된 데이터셋 검증:

```python
import json
import cv2
import os

dataset_dir = "./dataset_exp01"

# 첫 번째 프레임 검증
with open(os.path.join(dataset_dir, "annotations/frame_000000.json")) as f:
    ann = json.load(f)

img = cv2.imread(os.path.join(dataset_dir, "images", ann["image_file"]))

# 2D skeleton overlay
for joint in ann["skeleton_2d"]:
    if joint["visible"]:
        cv2.circle(img, (int(joint["u"]), int(joint["v"])), 5, (0, 255, 0), -1)

cv2.imshow("Verification", img)
cv2.waitKey(0)
```

## 요구사항

| 컴포넌트 | 버전 |
|----------|------|
| OrbbecSDK K4A Wrapper | v1.10.5 |
| OpenCV | 4.12.0 |
| Azure Kinect Body Tracking SDK | 1.1.2 |
| ONNX Runtime | 1.10.0 |

## 제한사항

- 체커보드가 항상 최소 1개 고정 카메라에서 보여야 함
- 체커보드 미검출 시 해당 프레임 건너뛰기 또는 이전 포즈 사용
- skeleton.csv는 `multi_device_offline_processor` 출력 형식이어야 함

## 트러블슈팅

### 체커보드 검출 실패
- 조명 조건 확인 (반사광, 그림자 방지)
- 체커보드 크기/해상도 확인
- `--checkerboard-rows`, `--checkerboard-cols` 값 확인

### 타임스탬프 동기화 오류
- 모든 카메라가 Sync Hub로 동기화되었는지 확인
- MKV 파일이 동시에 녹화되었는지 확인

### 2D 투영이 이미지 밖으로 나감
- Camera A calibration 확인
- T_checker_to_A 측정값 검증
