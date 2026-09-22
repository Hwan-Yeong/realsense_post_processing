# RANSAC 튜너 Phase 0~5 결과

2026-09-22. 계획(C++ 코어 단일화 + pybind11) 전 단계 완료. 테스트 30개 전부 통과.

## 구조

```
include/floor_ransac.hpp           알고리즘 단일 소스 (ROS/PCL 비의존, Eigen+std만)
src/floor_ransac.cpp
   ├─> src/realsense_processor_node.cpp   ROS 2 노드 전략 (임베디드 타깃)
   ├─> src/floor_ransac_py.cpp            pybind11 -> Python 튜너
   └─> test/test_floor_ransac.cpp         gtest 합성 데이터
```

Python은 알고리즘을 갖지 않는다. `.db3` 재생, librealsense 필터, OpenCV UI, 지표 집계만 맡는다.

## 게이트 결과

| Phase | 게이트 | 결과 |
|---|---|---|
| 0 | `colcon build` 성공 | 통과 (C++14→17, 누락 include/멤버/의존성, 반환형 등 수정) |
| 1 | 헤더에 rclcpp/pcl 심볼 없을 것 | 통과 (`-I include -I eigen3`만으로 컴파일) |
| 2 | gtest 합성 데이터 (0.35 m + **0.15 m**) | 17/17 통과 |
| 3 | 같은 합성 데이터로 C++/Python 결과 일치 | 11/11 통과, 법선·d·inlier·채택 가설 인덱스까지 **비트 단위 일치** |
| 4 | Python 튜너 | `frame.keep()` 스파이크 선행 확인 후 구현 |
| 5 | ROS 노드 결합 + 실데이터 검증 + 성능평가 | 통과 (아래) |

Phase 3은 `test/dump_reference.cpp`가 C++ 결과를 JSON으로 뱉고
`test/test_floor_ransac_py.py`가 같은 시나리오를 pybind로 재현해 비교하는 방식이다.
시나리오 정의는 C++ 한 곳에만 둔다 (양쪽에 적으면 갈라진다).

## frame.keep() 스파이크 (Phase 4 선행 조건)

`bag/20260922_174035.db3`: 263 프레임 848x480, **적재 2.4초 / maxRSS 270 MB**.
보관한 프레임에 나중에 필터 재적용·점군 계산 모두 정상. 전체 메모리 적재 방식 확정.

## 튜닝 결과 (실데이터)

기준: 실측 마운트 높이 **0.15 m**, 합격 기준 1 cm 이내.

같은 프레임에서 seed만 바꿔 측정한 **순수 추정기 노이즈** (프레임 간 흔들림의 대부분이
카메라 움직임이 아니라 추정기 노이즈였다 — 확인 후 이 값을 최적화 대상으로 삼음):

| 설정 | 높이 std | pitch std | 성공 |
|---|---|---|---|
| 시작값 thresh 20mm / iters 200 / tilt 45° / z≤4.0 | 5.0 mm | 0.91° | 253/263 |
| thresh 10mm / iters 1000 / tilt 10° / z≤4.0 | 5.0 mm | 0.91° | 263/263 |
| + z_max 1.5 | 2.7 mm | 0.56° | 263/263 |
| + eval_n 20000 | 1.9 mm | 0.38° | 263/263 |
| **+ thresh 6mm / iters 4000** | **0.75 mm** | **0.18°** | 263/263 |

핵심 두 가지:
- **`eval_n`이 가장 큰 레버였다** (5000 → 20000에서 노이즈 절반). `iters`보다 효과가 크다.
- **`z_max`를 1.5 m로 자르는 것**이 중요하다. 마운트가 15 cm로 낮아 먼 바닥은
  스침각이라 depth 노이즈가 높이 오차로 크게 증폭된다 — 넣을수록 해가 된다.

pitch 0.18°는 1.5 m에서 약 4.7 mm 높이 오차 → 3 cm 턱 판정에 여유가 있다
(CLAUDE.md: 1° = 1.5 m에서 2.6 cm).

### 카메라 높이 추정값

| 입력 | 평균 높이 | 실측 0.150 m 대비 |
|---|---|---|
| raw (노드가 받는 것과 동일) | 0.1379 m | 12.1 mm |
| decim2 + spatial | 0.1424 m | 7.6 mm |

librealsense 후처리를 거치면 1 cm 이내로 들어온다. 노드 자체는 SDK 필터를 돌리지
않으므로, 실제 시스템에서는 드라이버(realsense2_camera)의 post-processing을 켠 depth를
받는 전제다. 켜지 않으면 1.2 cm로 기준을 살짝 넘는다.

### 노드 ↔ 튜너 일치 확인

같은 raw 입력에 대해:
- 튜너(pybind): h 0.1379 ± 5.6 mm, pitch −1.34°
- ROS 노드 로그: h 0.137~0.138 m, pitch −1.41 ~ −1.49°

값이 일치한다. 코어를 하나로 둔 목적이 실제로 달성됐다는 확인.

## 성능 (x86, Release, 코어 RANSAC 구간만)

| 해상도 | iters / eval_n | mean | p95 |
|---|---|---|---|
| 848x480 | 1000 / 20000 | 9.2 ms | 11.0 ms |
| 848x480 | 1000 / 5000 | 3.0 ms | 3.7 ms |
| 848x480 | 4000 / 20000 | 29.4 ms | 34.3 ms |
| 424x240 | 1000 / 20000 | 7.8 ms | 9.0 ms |
| 424x240 | 200 / 5000 | 0.8 ms | 1.0 ms |

노드 전체(점군 생성 + RANSAC + 발행)는 848x480에서 약 **18 ms**. 15 fps 예산 66.7 ms
안에 충분히 들어간다.

**선택**: 노드 기본값은 `iters 1000 / eval_n 20000` (9 ms, pitch std 0.38°).
가장 정확한 `iters 4000`은 29 ms라 x86에서는 되지만 RK3588 여유를 생각해 뺐다.
정확도가 더 필요하면 `iters`보다 `eval_n`을 먼저 올릴 것.

## 고친 버그

- `setAxis(0,0,1)` = Z축 → **벽을 찾고 있었다.** 광학 좌표계에서 바닥 법선은 ±y다.
  코어는 `|n_y| >= cos(max_tilt)` 조건으로 대체.
- `setDistanceThreshold(0.05)` → 5 cm inlier 밴드는 3 cm 턱을 바닥에 삼킨다. 기본 6 mm.
- 법선 방향 규약 없음 → 코어가 `n[1] < 0` 강제. d 부호로 뒤집지 않는다.
- 재추정 후 tilt 재검사 없음 → 코어가 SVD 재추정 뒤 각도를 다시 본다.
  (실제로 노이즈 있는 벽에서 3점 가설이 tilt 필터를 통과하고 이 재검사에서만 걸리는
   경우가 관측됐다. `WallIsRejected` 테스트가 그 경로다.)
- 빌드 실패 65개 (C++14, include 누락, `node_ptr_`, 반환형, 중복 선언, 의존성 등)
- **`PointCloud2Modifier::resize()`가 organized 구조를 파괴**한다 (height/width가 둘 다
  1이 아니면 height를 1로 누름). 이 탓에 하단 ROI가 적용되지 않고, 원점에 몰린
  무효 픽셀 수십만 개가 '원점을 지나는 평면'으로 뽑혀 카메라 높이가 0으로 나왔다.
  resize 뒤 height/width/row_step을 되돌려 해결.

## 알아 둘 것

- **마운트 15 cm + pitch가 크면 요구 구간(1.0~1.5 m)이 시야에서 사라진다.**
  pitch 40°면 보이는 바닥이 z 0.06~0.81 m뿐이다. 튜닝 전에 pitch부터 확인할 것.
  (`LowMountWithSteepPitchLosesTheRequirementBand` 테스트가 이 사실을 고정한다)
- **Composite에서 `floor_ransac`을 먼저 둘 것.** `passthrough_voxel`이 앞에 오면
  점군이 unorganized가 되어 하단 ROI를 못 쓴다 (후보가 137k → 2k로 급감, 추정 불안정).
  노드가 이 순서를 감지하면 경고한다.
- `.db3`는 `ros2 bag play`로 재생 불가 → 노드 검증에는 `scripts/rs_db3_bridge.py` 필요.
- 이 환경의 `~/.local` anyio 4.13이 시스템 pytest 6.2.5와 안 맞아 pytest 플러그인
  자동 로드가 깨져 있다. CMake가 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`로 우회하지만,
  근본 해결은 `pip install -U pytest` 또는 anyio 제거.

## 사용법

```bash
colcon build --packages-select realsense_post_processing
source install/setup.bash

# 튜너 (트랙바로 실시간 조정, A/B 비교, k=반복성 검사, r=일괄 실행, s=저장)
python3 realsense-post-processing/scripts/rs_ransac_tuner.py bag/20260922_174035.db3

# 기존 raw|filtered 비교
python3 realsense-post-processing/scripts/rs_floor_ransac_test.py bag/... --decim 2 --spatial

# ROS 노드 + .db3 브리지
ros2 launch realsense_post_processing realsense_post_processing.launch.py
python3 realsense-post-processing/scripts/rs_db3_bridge.py --bag bag/20260922_174035.db3

colcon test --packages-select realsense_post_processing && colcon test-result --all
```
