# TASK: RANSAC 바닥 추출 인터랙티브 튜너

## 목적
녹화한 바닥 데이터(.db3)에서 RANSAC 파라미터를 **실시간으로 바꿔가며**
바닥 추출 결과·안정성·지표 변화를 즉시 비교한다. (CLI 재실행 반복 제거)

## 요구사항

### 1. 데이터 로딩 / 프레임 탐색
- .db3 전체 depth 프레임을 시작 시 메모리에 로드 (10초 @15fps 848x480 ≈ 120 MB, 충분)
  - raw depth + 필터 적용 depth를 캐시, 필터 파라미터가 바뀌면 필터 결과만 재계산
  - temporal 필터는 순서 의존 → 켜져 있으면 0번 프레임부터 순차 재계산
- 프레임 슬라이더로 임의 프레임 이동, space로 재생/정지
- 정지 상태에서 파라미터를 바꾸면 **같은 프레임에 즉시 재계산**

### 2. 조정 가능한 파라미터 (OpenCV trackbar 별도 "controls" 창)
RANSAC
- ransac_thresh [mm] (5~50), iters (10~1000), max_tilt [deg] (5~80)
- roi_bottom [%] (10~100), eval_n (1000~20000), z 후보 범위 min/max
- seed + "seed 고정 / 매번 랜덤" 토글
- 입력 소스: raw / filtered
- plane 모드: per_frame / fixed (fixed 기준면 재추출 키 제공)
표시/지표
- obs 임계값 [mm], hmax, hres, 뷰(inlier / residual / depth)
필터 (선택, 2순위)
- decimation 1~4, spatial on/off + alpha/delta, temporal on/off + alpha

### 3. 비교 방식
- **A/B 모드**: 같은 프레임에 파라미터 세트 A | B 좌우 비교 (trackbar는 현재 편집 중인 세트에 적용, 키로 전환)
- **RANSAC 반복성 검사**: 같은 프레임에서 seed만 바꿔 N회(예: 30) 실행 →
  카메라 높이 / pitch / roll / inlier 비율의 std 표시 (파라미터가 불안정한지 판단)
- 오버레이: 카메라 높이, pitch, roll, inlier 비율, 채택된 가설의 inlier 수, RANSAC 시간(ms),
  1.0-1.5 m 구간 바닥 std / 허상 장애물률

### 4. 저장 / 재현성
- 's': 현재 파라미터 세트(A/B)를 YAML/JSON으로 저장
- 시작 시 `--params file.yaml`로 불러오기
- 'r': 현재 설정으로 전체 프레임 일괄 실행 → 기존 rs_floor_ransac_test.py와 같은 요약 표 출력

## 구현 가이드
- 기존 `rs_floor_ransac_test.py`의 RANSAC / 점군 / 지표 / 그리기 로직을 `scripts/common/`으로 분리해
  기존 스크립트와 새 튜너가 공유 (CLAUDE.md의 좌표계·RANSAC 규약 준수)
- RANSAC 함수는 디버깅용 정보도 반환: 가설별 inlier 수 이력, 채택 가설, tilt 조건으로 기각된 가설 수
- trackbar 콜백에서 무거운 계산 금지 → 값만 갱신하고 메인 루프에서 dirty 플래그로 재계산
- 의존성 추가 없이 OpenCV만으로 (open3d 3D 보기는 기존처럼 optional)

## 검증
- 합성 데이터 테스트 (pytest): 높이 0.35 m, pitch 25°, 노이즈 σ≈4 mm, 3 cm 박스
  → 높이 오차 < 5 mm, pitch 오차 < 0.5°, 박스 점 > 95%가 h > 2 cm
- 법선 방향 규약(n_y < 0) / 재추정 후 tilt 재검사 회귀 테스트 포함
- 실제 바닥 녹화 데이터로 실행: 카메라 높이가 실측값과 1 cm 이내인지 확인
