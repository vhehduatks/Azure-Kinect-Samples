# Multi-Device Body Tracking Viewer

Orbbec Femto Bolt 멀티 카메라 환경에서 Body Tracking을 수행하는 샘플입니다.

## 개요

Azure Kinect Body Tracking SDK를 Orbbec SDK K4A Wrapper와 함께 사용하여 여러 대의 Femto Bolt 카메라에서 동시에 skeleton 추적을 수행합니다.

## 요구사항

### 하드웨어
- Orbbec Femto Bolt 카메라 (1대 이상)
- Multi-Camera Sync Hub (2대 이상 사용 시)
- SH1.2-8P 동기화 케이블

### 소프트웨어
- Windows 10 이상
- Visual Studio 2022 (v143 toolset)
- Azure Kinect Body Tracking SDK 1.1.2
- Orbbec SDK K4A Wrapper v1.10.5+

## 빌드 방법

### 1. NuGet 패키지 복원
```
Visual Studio에서 솔루션 열기 → 솔루션 우클릭 → NuGet 패키지 복원
```

### 2. 빌드
```
Configuration: Release
Platform: x64
```

### 3. Orbbec DLL 복사 (중요!)
빌드 후 NuGet 패키지가 Azure Kinect DLL을 덮어쓰므로, 반드시 Orbbec DLL을 다시 복사해야 합니다:

```powershell
$src = "C:\OrbbecSDK_K4A_Wrapper_v1.10.5_windows_202510212040\bin"
$dst = ".\build\bin\Release"

Copy-Item "$src\k4a.dll" $dst -Force
Copy-Item "$src\k4arecord.dll" $dst -Force
Copy-Item "$src\depthengine_2_0.dll" $dst -Force
Copy-Item "$src\OrbbecSDK.dll" $dst -Force
Copy-Item "$src\ob_usb.dll" $dst -Force
Copy-Item "$src\live555.dll" $dst -Force
Copy-Item "$src\OrbbecSDKConfig_v1.0.xml" $dst -Force
```

## 실행 방법

```bash
# 기본 실행 (NFOV_UNBINNED, DirectML)
multi_device_body_viewer.exe

# Wide FOV 모드
multi_device_body_viewer.exe WFOV_BINNED

# CPU 모드
multi_device_body_viewer.exe CPU

# 옵션 조합
multi_device_body_viewer.exe WFOV_BINNED DIRECTML
```

## 키보드 단축키

| 키 | 기능 |
|----|------|
| ESC | 종료 |
| H | 도움말 표시 |
| B | Body 시각화 모드 전환 |
| K | 3D 레이아웃 전환 |
| C | 캘리브레이션 저장 (intri.yml, calibration.json) |

## 캘리브레이션 내보내기

`C` 키를 누르면 다음 파일이 생성됩니다:

### intri.yml (EasyMocap 형식)
```yaml
%YAML:1.0
---
names:
   - "0"
   - "1"
K_0: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ fx, 0., cx, 0., fy, cy, 0., 0., 1. ]
dist_0: !!opencv-matrix
   rows: 1
   cols: 5
   dt: d
   data: [ k1, k2, p1, p2, k3 ]
H_0: 576
W_0: 640
```

### calibration.json (상세 정보)
```json
{
  "cameras": [
    {
      "id": 0,
      "serial_number": "XXXXXX",
      "is_primary": true,
      "width": 640,
      "height": 576,
      "K": [[fx, 0, cx], [0, fy, cy], [0, 0, 1]],
      "dist": [k1, k2, p1, p2, k3, k4, k5, k6]
    }
  ]
}
```

## 멀티 디바이스 동기화

### 자동 설정
- 첫 번째 디바이스: PRIMARY (Master)
- 나머지 디바이스: SECONDARY (Subordinate)
- IR 간섭 방지를 위해 각 Subordinate에 160μs 지연 적용

### 시작 순서
Secondary 디바이스를 먼저 시작한 후 Primary 디바이스를 시작합니다 (코드에서 자동 처리).

## 아키텍처

```
main.cpp
├── DeviceInfo          # 디바이스 정보 구조체
├── DeviceBodyData      # Body tracking 결과 저장
├── DeviceCaptureThread # 디바이스별 캡처 스레드
├── RenderAllBodies     # 모든 skeleton 렌더링
├── ExportCalibrationToYAML  # EasyMocap 형식 저장
└── ExportCalibrationToJSON  # JSON 형식 저장
```

## 외부 캘리브레이션 (TODO)

멀티 카메라 좌표 통합을 위해서는 외부 캘리브레이션이 필요합니다:

1. **체커보드 이미지 캡처**: 모든 카메라에서 동시에 체커보드가 보이는 컬러 이미지 캡처
2. **EasyMocap 캘리브레이션**: `calib_extri.py` 스크립트로 R, T 계산
3. **좌표 변환**: Secondary 카메라의 skeleton을 Primary 좌표계로 변환

참고: https://github.com/vhehduatks/EasyMocap/tree/master/apps/calibration

## 문제 해결

### 디바이스를 찾을 수 없음
- Orbbec DLL이 올바르게 복사되었는지 확인
- `OrbbecSDKConfig_v1.0.xml` 파일이 exe와 같은 폴더에 있는지 확인
- DLL 크기 확인:
  - k4a.dll: ~283KB (Orbbec) vs ~651KB (Azure Kinect)
  - depthengine_2_0.dll: ~355KB (Orbbec) vs ~423KB (Azure Kinect)

### Body Tracker 생성 실패
- ONNX Runtime DLL이 있는지 확인
- dnn_model_2_0_op11.onnx 파일이 있는지 확인
- GPU 드라이버가 최신인지 확인
