// floor_ransac 코어 합성 데이터 검증.
//
// 합격 기준은 docs/TASK_ransac_tuner.md의 "검증" 절을 그대로 따른다:
//   높이 오차 < 5 mm, pitch 오차 < 0.5°, 3 cm 턱 점의 95% 이상이 h > 2 cm.
// 0.35 m 케이스만 쓰지 않고 실측 마운트 높이 0.15 m 케이스를 반드시 포함한다
// (평면이 원점 근처일 때 법선이 뒤집히는 버그가 이 리그의 상시 조건이라서).

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "floor_ransac.hpp"
#include "floor_ransac_synthetic.hpp"

using floor_ransac::Params;
using floor_ransac::PointsRowMajor;
using floor_ransac::Result;
using floor_ransac::Status;
namespace syn = floor_ransac::synthetic;

namespace
{

Params tuned_params()
{
    Params p;
    p.thresh = 0.02;      // 5 cm가 아니라 2 cm — 3 cm 턱을 바닥에 삼키지 않는 값
    p.iters = 200;
    p.max_tilt_deg = 45.0;
    p.roi_bottom = 0.5;
    p.z_min = 0.2;
    p.z_max = 4.0;
    p.eval_n = 5000;
    p.seed = 0;
    return p;
}

// 장면 -> 후보점 추출 -> 평면 적합까지 한 번에
Result fit_scene(const syn::Scene & scene, const syn::SceneParams & sp, const Params & p)
{
    const PointsRowMajor cand =
        floor_ransac::select_candidates(scene.points, sp.height, sp.width, p);
    return floor_ransac::fit_plane(cand, p);
}

// 턱 점들 중 바닥 기준 높이가 obs를 넘는 비율
double box_detection_rate(
    const syn::Scene & scene, const floor_ransac::Plane & plane, double obs)
{
    int total = 0;
    int detected = 0;
    for (std::size_t i = 0; i < scene.labels.size(); ++i) {
        if (scene.labels[i] != syn::kBox) {
            continue;
        }
        ++total;
        const Eigen::Vector3d q = scene.points.row(static_cast<Eigen::Index>(i)).transpose();
        if (q.dot(plane.n) + plane.d > obs) {
            ++detected;
        }
    }
    return total == 0 ? 0.0 : static_cast<double>(detected) / total;
}

// 턱을 놓을 거리는 장면마다 다르다. 카메라가 낮고 pitch가 크면 먼 바닥이 아예 안 보인다
// (h=0.15 m, pitch 40°면 보이는 바닥이 z 0.06~0.81 m뿐이라 1.0~1.5 m 밴드가 화면 밖).
void check_scene_at_height(
    double cam_height, double pitch_deg, double box_z_min = 1.0, double box_z_max = 1.5)
{
    syn::SceneParams sp;
    sp.cam_height = cam_height;
    sp.pitch_down_deg = pitch_deg;
    sp.noise_sigma = 0.004;
    sp.box_height = 0.03;
    sp.box_z_min = box_z_min;
    sp.box_z_max = box_z_max;

    const syn::Scene scene = syn::make_scene(sp);
    ASSERT_GT(scene.n_floor, 1000) << "합성 장면에 바닥 점이 너무 적다";
    ASSERT_GT(scene.n_box, 100) << "합성 장면에 턱 점이 너무 적다";

    const Result r = fit_scene(scene, sp, tuned_params());
    ASSERT_TRUE(r.ok) << "바닥 추출 실패: " << floor_ransac::status_string(r.status);

    // 법선 방향 규약
    EXPECT_LT(r.plane.n[1], 0.0) << "법선이 카메라 위쪽(-y)을 향하지 않는다";

    // 높이 오차 < 5 mm
    EXPECT_NEAR(r.plane.d, cam_height, 0.005);

    // pitch 오차 < 0.5°
    const auto ang = floor_ransac::plane_angles(r.plane.n);
    EXPECT_NEAR(ang.pitch_down_deg, pitch_deg, 0.5);
    EXPECT_NEAR(ang.roll_deg, 0.0, 0.5);

    // 3 cm 턱의 95% 이상이 h > 2 cm로 분리되어야 한다
    EXPECT_GT(box_detection_rate(scene, r.plane, 0.02), 0.95);
}

}  // namespace

// ---------------------------------------------------------------- 기본 정확도

TEST(FloorRansac, Height035Pitch25)
{
    check_scene_at_height(0.35, 25.0);
}

// 실측 마운트 높이. 평면이 원점에서 15 cm밖에 안 떨어져 있어 법선 뒤집힘에 가장 취약하다.
TEST(FloorRansac, Height015Pitch25_RealMountHeight)
{
    check_scene_at_height(0.15, 25.0);
}

TEST(FloorRansac, Height015Pitch10)
{
    check_scene_at_height(0.15, 10.0);
}

// pitch 40°에서는 1.0~1.5 m 바닥이 시야 밖이라 턱을 0.4~0.6 m에 둔다.
TEST(FloorRansac, Height015Pitch40)
{
    check_scene_at_height(0.15, 40.0, 0.4, 0.6);
}

// 리그 사실 기록: 마운트 15 cm에서 pitch를 키우면 요구 스펙 구간(1.0~1.5 m)이 시야에서 사라진다.
// 튜닝 전에 pitch부터 확인해야 하는 이유.
TEST(FloorRansac, LowMountWithSteepPitchLosesTheRequirementBand)
{
    syn::SceneParams sp;
    sp.cam_height = 0.15;
    sp.pitch_down_deg = 40.0;
    sp.box_height = 0.0;
    const syn::Scene scene = syn::make_scene(sp);

    double z_max = 0.0;
    for (Eigen::Index i = 0; i < scene.points.rows(); ++i) {
        z_max = std::max(z_max, scene.points(i, 2));
    }
    EXPECT_LT(z_max, 1.0) << "pitch 40°/h 0.15 m에서 1 m 앞 바닥이 보이면 기하 가정이 틀린 것";

    // 같은 높이라도 pitch가 완만하면 1.5 m 바닥이 보인다
    sp.pitch_down_deg = 15.0;
    const syn::Scene shallow = syn::make_scene(sp);
    double z_max_shallow = 0.0;
    for (Eigen::Index i = 0; i < shallow.points.rows(); ++i) {
        z_max_shallow = std::max(z_max_shallow, shallow.points(i, 2));
    }
    EXPECT_GT(z_max_shallow, 1.5);
}

// ---------------------------------------------------------------- 법선 방향 규약

// d 부호로 법선을 뒤집는 구현이었다면, 카메라를 바닥 아래에 두어 d < 0을 만들면 깨진다.
// 여기서는 seed를 바꿔가며 항상 n[1] < 0 인지 본다.
TEST(FloorRansac, NormalAlwaysPointsUpRegardlessOfSeed)
{
    syn::SceneParams sp;
    sp.cam_height = 0.15;
    sp.pitch_down_deg = 25.0;
    const syn::Scene scene = syn::make_scene(sp);

    for (std::uint64_t seed = 0; seed < 20; ++seed) {
        Params p = tuned_params();
        p.seed = seed;
        const Result r = fit_scene(scene, sp, p);
        ASSERT_TRUE(r.ok) << "seed " << seed << ": " << floor_ransac::status_string(r.status);
        EXPECT_LT(r.plane.n[1], 0.0) << "seed " << seed << ": 법선이 아래를 향함";
        EXPECT_GT(r.plane.d, 0.0) << "seed " << seed << ": d가 카메라 높이가 아님";
        EXPECT_NEAR(r.plane.d, 0.15, 0.005) << "seed " << seed;
    }
}

// 카메라가 바닥보다 아래에 있는(=평면이 카메라 위) 뒤집힌 구성에서도 규약은 n[1] < 0.
// 이때 d는 음수가 되어야 하고, 법선이 180° 튀면 안 된다.
TEST(FloorRansac, NormalConventionHoldsWhenPlaneIsAboveCamera)
{
    // 바닥 대신 천장을 본다고 보고 점군을 y 부호 반전시켜 만든다.
    syn::SceneParams sp;
    sp.cam_height = 0.15;
    sp.pitch_down_deg = 25.0;
    sp.box_height = 0.0;
    syn::Scene scene = syn::make_scene(sp);

    PointsRowMajor pts = scene.points;
    for (Eigen::Index i = 0; i < pts.rows(); ++i) {
        pts(i, 1) = -pts(i, 1);  // y 반전 -> 평면이 카메라 위로
    }

    Params p = tuned_params();
    p.roi_bottom = 1.0;  // 반전했으므로 화면 전체를 후보로
    const PointsRowMajor cand = floor_ransac::select_candidates(pts, sp.height, sp.width, p);
    const Result r = floor_ransac::fit_plane(cand, p);

    ASSERT_TRUE(r.ok) << floor_ransac::status_string(r.status);
    EXPECT_LT(r.plane.n[1], 0.0) << "평면이 카메라 위에 있어도 법선은 -y 고정이어야 한다";
    EXPECT_NEAR(r.plane.d, -0.15, 0.005) << "이 경우 d는 음수";
}

// ---------------------------------------------------------------- tilt 재검사 회귀

// 벽만 있는 장면은 절대 '바닥'으로 채택되면 안 된다.
// (예전 setAxis(0,0,1) 버그는 법선이 전방인 평면을 찾았으므로 여기서 벽을 바닥이라고 답했다.)
//
// 주의: 노이즈가 있는 벽에서는 가까운 세 점의 depth 노이즈가 기하를 압도해
// 가설 단계 tilt 필터를 통과하는 법선이 더러 나온다. 그런 가설은 전체 inlier로
// 재추정하면 다시 ±z 법선이 되고, "재추정 후 tilt 재검사"에서 비로소 걸린다.
// 즉 이 테스트는 두 단계 중 어느 쪽이든 기각만 하면 통과다.
TEST(FloorRansac, WallIsRejected)
{
    syn::SceneParams sp;
    const PointsRowMajor wall = syn::make_wall(sp, 1.5);

    Params p = tuned_params();
    p.roi_bottom = 1.0;
    const Result r = floor_ransac::fit_plane(wall, p);

    EXPECT_FALSE(r.ok) << "수직 벽을 바닥으로 채택했다";
    EXPECT_TRUE(
        r.status == Status::kNoHypothesis ||
        r.status == Status::kTiltRejectedAfterRefit)
        << "예상 밖 상태: " << floor_ransac::status_string(r.status);
    EXPECT_GT(r.debug.rejected_tilt, 0) << "tilt 조건으로 기각된 가설이 하나도 없다";
}

// max_tilt를 아주 좁게 주면, 기울어진 바닥은 가설 단계든 재추정 후든 반드시 기각되어야 한다.
// setOptimizeCoefficients(true)처럼 재추정만 하고 각도를 재검사하지 않으면 여기서 통과해버린다.
TEST(FloorRansac, TiltRecheckedAfterRefit)
{
    syn::SceneParams sp;
    sp.cam_height = 0.15;
    sp.pitch_down_deg = 40.0;
    sp.box_height = 0.0;
    const syn::Scene scene = syn::make_scene(sp);

    Params p = tuned_params();
    p.max_tilt_deg = 20.0;  // 실제 기울기 40° > 20° -> 허용 불가
    const Result r = fit_scene(scene, sp, p);

    EXPECT_FALSE(r.ok) << "max_tilt 20°인데 40° 평면을 채택했다";
    EXPECT_TRUE(
        r.status == Status::kNoHypothesis ||
        r.status == Status::kTiltRejectedAfterRefit)
        << "예상 밖 상태: " << floor_ransac::status_string(r.status);
}

// 재추정 tilt 재검사 경로를 결정적으로 태운다.
//
// 장면: 앞쪽 70%는 완전 평평한 바닥, 뒤쪽 30%는 급경사 램프. thresh를 크게 줘서
// 둘 다 inlier가 되게 한다. 평평한 구간에서 뽑힌 3점 가설은 |n_y| = 1 로 tilt 필터를
// 무사히 통과하지만, 전체 inlier 최소제곱 해는 약 50° 기울어진다.
// 재추정 후 각도를 다시 보지 않는 구현(= PCL setOptimizeCoefficients)은 여기서 통과해버린다.
TEST(FloorRansac, RefitTiltRejectionPathIsReachable)
{
    const int n = 6000;
    const double ramp_z0 = 1.9;   // 뒤쪽 30% 시작 (z는 0.5~2.5)
    const double ramp_slope = 4.0;
    PointsRowMajor pts(n, 3);
    floor_ransac::Rng rng(7);
    for (int i = 0; i < n; ++i) {
        const double x = (rng.uniform01() - 0.5) * 2.0;
        const double z = 0.5 + rng.uniform01() * 2.0;
        const double y = (z > ramp_z0) ? 0.15 - ramp_slope * (z - ramp_z0) : 0.15;
        pts.row(i) = Eigen::Vector3d(x, y, z).transpose();
    }

    Params p = tuned_params();
    p.thresh = 5.0;          // 평평면과 램프가 전부 inlier가 되도록 크게
    p.max_tilt_deg = 20.0;   // 평평한 가설(0°)은 통과, 재추정 해(~50°)는 불가
    p.roi_bottom = 1.0;
    p.iters = 400;
    const Result r = floor_ransac::fit_plane(pts, p);

    EXPECT_FALSE(r.ok);
    EXPECT_EQ(r.status, Status::kTiltRejectedAfterRefit)
        << "재추정 후 tilt 재검사 경로를 타지 않았다: "
        << floor_ransac::status_string(r.status);
}

// ---------------------------------------------------------------- 재현성 / 디버그 정보

TEST(FloorRansac, SameSeedGivesIdenticalResult)
{
    syn::SceneParams sp;
    sp.cam_height = 0.15;
    const syn::Scene scene = syn::make_scene(sp);
    Params p = tuned_params();
    p.seed = 42;

    const Result a = fit_scene(scene, sp, p);
    const Result b = fit_scene(scene, sp, p);

    ASSERT_TRUE(a.ok && b.ok);
    EXPECT_EQ(a.plane.n[0], b.plane.n[0]);
    EXPECT_EQ(a.plane.n[1], b.plane.n[1]);
    EXPECT_EQ(a.plane.n[2], b.plane.n[2]);
    EXPECT_EQ(a.plane.d, b.plane.d);
    EXPECT_EQ(a.inliers, b.inliers);
    EXPECT_EQ(a.debug.best_hypothesis, b.debug.best_hypothesis);
}

TEST(FloorRansac, DebugInfoIsPopulated)
{
    syn::SceneParams sp;
    const syn::Scene scene = syn::make_scene(sp);
    Params p = tuned_params();
    const Result r = fit_scene(scene, sp, p);

    ASSERT_TRUE(r.ok);
    EXPECT_EQ(r.debug.hypotheses_tried, p.iters);
    EXPECT_EQ(static_cast<int>(r.debug.hypothesis_inliers.size()), p.iters);
    EXPECT_GE(r.debug.best_hypothesis, 0);
    EXPECT_GT(r.debug.best_eval_inliers, 0);
    EXPECT_EQ(r.debug.eval_points, std::min(p.eval_n, r.debug.candidates));
    // 채택 가설의 inlier 수가 실제로 최댓값인지
    const int best = *std::max_element(
        r.debug.hypothesis_inliers.begin(), r.debug.hypothesis_inliers.end());
    EXPECT_EQ(r.debug.best_eval_inliers, best);
}

// ---------------------------------------------------------------- 입력 방어

TEST(FloorRansac, TooFewCandidates)
{
    PointsRowMajor pts(10, 3);
    pts.setRandom();
    const Result r = floor_ransac::fit_plane(pts, tuned_params());
    EXPECT_FALSE(r.ok);
    EXPECT_EQ(r.status, Status::kTooFewCandidates);
}

TEST(FloorRansac, SelectCandidatesRespectsRoiAndZRange)
{
    syn::SceneParams sp;
    const syn::Scene scene = syn::make_scene(sp);

    Params p = tuned_params();
    p.roi_bottom = 0.5;
    const PointsRowMajor cand =
        floor_ransac::select_candidates(scene.points, sp.height, sp.width, p);

    ASSERT_GT(cand.rows(), 0);
    for (Eigen::Index i = 0; i < cand.rows(); ++i) {
        EXPECT_GT(cand(i, 2), p.z_min);
        EXPECT_LT(cand(i, 2), p.z_max);
    }
    // 하단 절반만 썼으므로 전체 유효 점보다 적어야 한다
    EXPECT_LT(cand.rows(), scene.n_floor + scene.n_box);
}

TEST(FloorRansac, SelectCandidatesRejectsMismatchedShape)
{
    syn::SceneParams sp;
    const syn::Scene scene = syn::make_scene(sp);
    const PointsRowMajor bad =
        floor_ransac::select_candidates(scene.points, sp.height + 1, sp.width, tuned_params());
    EXPECT_EQ(bad.rows(), 0);
}

TEST(FloorRansac, PlaneAnglesRoundTrip)
{
    for (double pitch : {0.0, 10.0, 25.0, 40.0}) {
        for (double roll : {-5.0, 0.0, 5.0}) {
            const Eigen::Vector3d n = syn::normal_from_angles(pitch, roll);
            EXPECT_LT(n[1], 0.0);
            const auto a = floor_ransac::plane_angles(n);
            EXPECT_NEAR(a.pitch_down_deg, pitch, 1e-9);
            EXPECT_NEAR(a.roll_deg, roll, 1e-9);
        }
    }
}

TEST(FloorRansac, PointHeightsMatchPlaneEquation)
{
    syn::SceneParams sp;
    sp.noise_sigma = 0.0;
    sp.box_height = 0.03;
    const syn::Scene scene = syn::make_scene(sp);

    const Eigen::VectorXd h = floor_ransac::point_heights(scene.points, scene.truth);
    for (std::size_t i = 0; i < scene.labels.size(); ++i) {
        const Eigen::Index k = static_cast<Eigen::Index>(i);
        if (scene.labels[i] == syn::kFloor) {
            EXPECT_NEAR(h[k], 0.0, 1e-9);
        } else if (scene.labels[i] == syn::kBox) {
            EXPECT_NEAR(h[k], sp.box_height, 1e-9);
        }
    }
}
