# realsense-post-processing

AMR(ROS 2 / Nav2) 장애물 감지용 RealSense D435i depth 파라미터·post-processing 튜닝 및
바닥(RANSAC) 추출 분석 도구 모음. 녹화 데이터(.db3)를 재생하며 설정별 결과를 비교하는 것이 목적.

## 요구 스펙 (튜닝 판단 기준)
- 감지 거리 0.2 ~ 4 m, 로봇 최고속도 1 m/s, Nav2 costmap 해상도 5 cm
- 낮은 장애물 3 cm 구분 (승월 기준). 실질 목표: 1.0~1.5 m 앞 3 cm 턱을 바닥과 분리
- D435i는 safety-rated 아님 → 주행(회피)용. 안전 기능은 별도 계층 전제

## 환경 / 워크플로
- 녹화: Windows RealSense Viewer 또는 RK3588(Orange Pi 5 Max)에서 `scripts/rs_record_headless.py`
- 분석: WSL (Ubuntu) Python. 결과 창은 OpenCV (WSLg)
- 최신 librealsense는 녹화 포맷이 **.db3 (ROS 2 rosbag2, SQLite)**. legacy .bag 아님
- 녹화한 SDK와 읽는 pyrealsense2 **버전을 맞출 것** (불일치/불완전 파일이면 "SQL logic error")
- 데이터: `bag/*.db3` (git 제외 권장)

## 확정된 설정
- Depth 848x480, Z16, 15 fps, emitter ON, preset = custom (Viewer에서 조정 후 JSON export)
- 848 선택 이유: MinZ = fx·B/126(disparity 탐색 한계) → 848에서 약 18 cm, 1280이면 약 27 cm
- decimation은 호스트 후처리라 MinZ에 영향 없음, 848 depth 정밀도 유지
- Viewer JSON에는 post-processing 설정이 **포함되지 않음** → 코드/launch 파라미터로 별도 관리

## Post-processing 체인 (librealsense 권장 순서)
decimation → threshold → depth→disparity → spatial → temporal → disparity→depth → (hole filling)
- decimation: 2 vs 4 평가 중 (4 이상은 median이 아닌 mean → 엣지 flying pixel 우려)
- threshold: 0.2 ~ 4.0 m
- spatial: 약하게 (mag 2, alpha 0.5, delta 20, holes_fill 0)
- temporal: 주행 중 잔상 위험 → OFF 또는 약하게 (alpha 0.4, persistency 0)
- hole filling: OFF (없는 depth를 만들어 냄)

## 알고리즘 단일화 (중요)
바닥 RANSAC 구현은 **C++ 한 곳뿐**이다: `include/floor_ransac.hpp` + `src/floor_ransac.cpp`
(ROS/PCL 비의존, Eigen+std만). ROS 노드 전략 / pybind11 모듈 / gtest가 전부 이걸 링크한다.
- Python 튜너는 알고리즘을 갖지 않고 pybind11로 '컴파일된 그 코드'를 호출한다
  → 튜너로 찾은 파라미터가 임베디드에서 그대로 재현된다
- `pcl::SACSegmentation`은 쓰지 않는다: 법선 방향 규약 강제, 재추정 후 tilt 재검사,
  가설별 디버그 정보 — 셋 다 PCL API로 불가능
- Python이 맡는 것: .db3 재생, librealsense 필터(SDK 구현), OpenCV UI, 지표 집계
- 결과·튜닝값·함정은 `docs/RESULT_ransac_tuner.md` 참고

## 스크립트
- `scripts/common/` : 공용 모듈 (core=C++ 코어 래퍼, filters, playback, metrics, render)
- `scripts/rs_ransac_tuner.py` : **인터랙티브 튜너**. 전체 프레임 메모리 적재 + 트랙바로
  RANSAC/필터 실시간 조정, A/B 좌우 비교, 반복성 검사(k), 일괄 실행(r), 파라미터 저장(s)
- `scripts/rs_db3_bridge.py` : .db3 → sensor_msgs/Image + CameraInfo 발행 (ROS 노드 검증용).
  `ros2 bag play`로는 이 .db3를 재생할 수 없어서 반드시 필요하다
- `scripts/rs_bag_filter_test.py` : .db3 재생, raw | filtered depth 컬러맵 비교, fill rate, 필터 처리시간
- `scripts/rs_floor_ransac_test.py` : raw | filtered 각각 RANSAC 바닥 추출, 뷰(inlier/residual/depth),
  open3d 3D(p키), 종료 시 거리 구간별 바닥 std / 허상 장애물률 / 평면 안정성 요약
- `scripts/rs_record_headless.py` : RK3588 headless 녹화 (USB 타입/용량 확인, JSON 적용, warmup pause, 드롭 검사)

공통 규칙: 재생은 `set_real_time(False)` (프레임 누락 없이 결정적 재생 → temporal 비교 재현 가능)
전체 프레임 적재는 `frame.keep()` 필수 — 없으면 SDK가 버퍼를 재활용해 나중에 읽은 프레임이 깨진다

## 좌표계 / RANSAC 규약 (변경 금지)
- RealSense 카메라 좌표: x 오른쪽, y 아래, z 전방, 단위 m. 점군은 `rs.pointcloud().calculate()`
- 평면 n·p + d = 0. **법선은 항상 카메라 기준 위쪽(-y, n[1] < 0)으로 방향 고정**
  - d 부호로 뒤집으면 평면이 원점 근처일 때 법선이 180° 튐 (실제로 roll ±90° 발산 버그 있었음)
- d = 카메라 높이, n·p + d = 바닥 기준 높이
- pitch_down = asin(-n_z), roll = atan2(n_x, -n_y)
- RANSAC: 하단 ROI 후보점 → 3점 가설 → |n_y| ≥ cos(max_tilt)만 허용(벽 배제) → 서브샘플로 inlier 투표
  → 전체 inlier SVD 재추정 → **재추정 후 tilt 조건 재검사**
- plane 모드: per_frame(필터가 바닥 추정 자체에 주는 영향) / fixed(첫 raw 프레임 평면을 공통 기준, 필터 비교용)

## 지표 정의
- 거리 구간 z: 0.3-0.5 / 0.5-1.0 / 1.0-1.5 / 1.5-2.0 / 2.0-3.0 / 3.0-4.0 m
- 통계 대상: 바닥 기준 |h| < 10 cm (WALL_CUT) 점만
- 바닥 std: 위 점들의 높이 std. ~58 mm(±10cm 균일분포 std)에 가까우면 그 구간에 실제 바닥이 없다는 뜻
- 허상 장애물률: h > obs(기본 2 cm) 점 비율 (빈 바닥 장면에서만 의미)
- 평면 안정성: per_frame에서 카메라 높이/pitch/roll의 프레임 간 std. 정상 바닥이면 pitch ± 0.5° 이하 기대

## 알려진 함정
- `filter.process()`는 `rs.frame` 반환 → `get_width()` 등 쓰려면 `.as_depth_frame()` 캐스팅 필수
- 틸트 1° 오차 = 1.5 m에서 높이 약 2.6 cm 오차 → 3 cm 판정에는 마운트/평면 정확도가 필터보다 중요
- 장애물이 ROI에 많으면 per_frame 평면이 끌려감 → fixed 모드 사용

## 코드 스타일
- Python 3, argparse CLI, 의존성 최소 (pyrealsense2, numpy, opencv; open3d는 optional import)
- 주석/출력 메시지는 한국어
- 공통 로직은 `scripts/common/`에 있다. 스크립트에 중복 구현하지 말 것
- **RANSAC 등 수치 로직은 C++ 코어에서 고친다.** Python에 같은 로직을 만들면 구현이 갈라진다
- 코어 변경 시 `colcon test`로 합성 데이터 회귀 확인 (0.35 m + 0.15 m 케이스, 법선 규약,
  재추정 후 tilt 재검사, C++/Python 결과 일치까지 검사한다)
