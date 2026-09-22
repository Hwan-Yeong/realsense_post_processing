#pragma once
//
// 바닥 평면 RANSAC 코어 — ROS / PCL 비의존 (Eigen + 표준 라이브러리만).
//
// 이 헤더가 알고리즘의 단일 소스다. 세 곳이 "같은 이 코드"를 호출한다:
//   1) ROS 2 노드 전략 (임베디드 타깃)
//   2) pybind11 모듈  (Python 튜너)
//   3) gtest 합성 데이터 검증
// 튜너에서 찾은 파라미터가 임베디드에서 그대로 재현되려면 구현이 하나여야 한다.
//
// 좌표계 (CLAUDE.md, 변경 금지):
//   RealSense 카메라/광학 좌표 — x 오른쪽, y 아래, z 전방, 단위 m.
//   평면은 n·p + d = 0.
//   법선은 항상 카메라 기준 위쪽(-y)을 향한다: n[1] < 0.
//     ※ d 부호로 방향을 잡으면 평면이 원점 근처일 때 법선이 180° 튄다.
//        실측 마운트 높이가 0.15 m라 이 리그에서는 상시 조건에 가깝다.
//   d = 카메라 높이, n·p + d = 바닥 기준 높이.
//
#include <cstddef>
#include <cstdint>
#include <random>
#include <vector>

#include <Eigen/Core>

namespace floor_ransac
{

// (N, 3) row-major 점군. numpy C-contiguous float64 배열과 메모리 레이아웃이 같다.
using PointsRowMajor = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>;
using PointsRef = Eigen::Ref<const PointsRowMajor>;

struct Params
{
    double thresh = 0.02;        // inlier 판정 거리 [m]
    int iters = 200;             // 3점 가설 반복 수
    double max_tilt_deg = 45.0;  // 바닥 법선이 카메라 y축과 이룰 수 있는 최대 각 (벽 배제)
    double roi_bottom = 0.5;     // 후보로 쓸 화면 하단 비율 (organized 입력 전용)
    double z_min = 0.2;          // 후보 z 범위 [m]
    double z_max = 4.0;
    int eval_n = 5000;           // 가설 평가용 서브샘플 크기 (0 이하 = 전체 사용)
    std::uint64_t seed = 0;
    int min_candidates = 200;    // 이보다 적으면 추정 포기
};

struct Plane
{
    Eigen::Vector3d n{0.0, -1.0, 0.0};  // 항상 n[1] < 0
    double d = 0.0;                     // 카메라 높이 [m]
};

struct Angles
{
    double pitch_down_deg = 0.0;  // 카메라가 아래를 보는 각
    double roll_deg = 0.0;
};

enum class Status
{
    kOk = 0,
    kTooFewCandidates,       // 후보점 < min_candidates
    kNoHypothesis,           // iters 안에서 tilt 조건을 통과한 가설이 없음
    kTooFewInliers,          // 채택 가설의 전체 inlier < 3 (재추정 불가)
    kTiltRejectedAfterRefit  // SVD 재추정 후 tilt 재검사에서 기각
};

// 튜너/회귀 분석용. 임베디드 경로에서도 채워지지만 비용은 무시할 수준이다.
struct Debug
{
    int candidates = 0;             // 입력 점 수
    int eval_points = 0;            // 가설 평가에 쓴 서브샘플 크기
    int hypotheses_tried = 0;       // == iters
    int rejected_degenerate = 0;    // 세 점이 거의 일직선
    int rejected_tilt = 0;          // 가설 단계에서 기울기로 기각 (벽 등)
    int best_hypothesis = -1;       // 채택된 가설의 반복 인덱스
    int best_eval_inliers = 0;      // 채택 가설의 서브샘플 기준 inlier 수
    // 가설별 서브샘플 inlier 수. 기각된 가설은 -1. 크기 = iters.
    std::vector<int> hypothesis_inliers;
    Plane hypothesis_plane;         // SVD 재추정 전, 채택된 3점 가설 평면
};

struct Result
{
    bool ok = false;
    Status status = Status::kTooFewCandidates;
    Plane plane;
    int inliers = 0;  // 재추정에 쓴 전체 후보 기준 inlier 수
    Debug debug;
};

// 결정적 인덱스 생성기.
// stdlib의 uniform_int_distribution은 구현마다 결과가 달라서 직접 rejection sampling 한다.
// (같은 seed면 어디서 빌드하든 같은 가설 순서가 나와야 튜닝 결과가 재현된다)
class Rng
{
public:
    explicit Rng(std::uint64_t seed) : gen_(seed) {}
    std::uint64_t index(std::uint64_t n);
    double uniform01();  // [0, 1)

private:
    std::mt19937_64 gen_;
};

// 메인 진입점. pts는 (N,3) 후보점.
Result fit_plane(const PointsRef & pts, const Params & p);

// pitch_down = asin(-n_z), roll = atan2(n_x, -n_y)  (CLAUDE.md 규약)
Angles plane_angles(const Eigen::Vector3d & n);

// organized 점군 (height*width 행, row-major) -> 하단 ROI + z 범위 후보점.
// z <= 0 (무효 픽셀)은 제외한다.
PointsRowMajor select_candidates(
    const PointsRef & organized, int height, int width, const Params & p);

// 각 점의 바닥 기준 높이 n·p + d.
Eigen::VectorXd point_heights(const PointsRef & pts, const Plane & plane);

const char * status_string(Status s);

}  // namespace floor_ransac
