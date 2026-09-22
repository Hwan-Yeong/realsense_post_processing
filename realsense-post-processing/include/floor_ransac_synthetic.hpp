#pragma once
//
// 합성 바닥 장면 생성기 — 검증 전용 (header-only, ROS/PCL 비의존).
//
// gtest와 pybind11 모듈이 둘 다 이 헤더를 쓴다. 같은 seed로 같은 점군이 나오므로
// "C++ 테스트와 Python 튜너가 정말 같은 코드를 돌리는가"를 같은 입력으로 비교할 수 있다.
//
// 핀홀 카메라에서 광선을 쏴 바닥 평면(및 턱 윗면)에 맞히는 방식이라
// 거리에 따른 점 밀도 감소가 실제 depth 카메라와 같다 — RANSAC 거동에 영향이 크다.
//
#include <cmath>
#include <cstdint>
#include <vector>

#include "floor_ransac.hpp"

namespace floor_ransac
{
namespace synthetic
{

struct SceneParams
{
    int width = 424;   // decimation 2 기준 848x480 -> 424x240
    int height = 240;
    double fx = 212.0;
    double fy = 212.0;
    double cx = 212.0;
    double cy = 120.0;

    double cam_height = 0.35;   // 바닥 위 카메라 높이 [m] -> 정답 d
    double pitch_down_deg = 25.0;
    double roll_deg = 0.0;
    double noise_sigma = 0.004;  // 광선 방향(depth) 가우시안 노이즈 [m]

    // 낮은 턱(3 cm 승월 기준). box_height <= 0 이면 생성하지 않는다.
    double box_height = 0.03;
    double box_z_min = 1.0;   // 카메라 전방 거리 [m]
    double box_z_max = 1.5;
    double box_half_width = 0.4;  // 카메라 x축 기준 반폭 [m]

    double z_far = 6.0;  // 이보다 먼 교점은 무효 픽셀로 (지평선 근처 발산 방지)
    std::uint64_t seed = 12345;
};

enum Label : int
{
    kInvalid = 0,
    kFloor = 1,
    kBox = 2
};

struct Scene
{
    PointsRowMajor points;    // (height*width, 3) organized, 무효 픽셀은 (0,0,0)
    std::vector<int> labels;  // 크기 height*width
    Plane truth;              // 정답 바닥 평면 (n[1] < 0, d = cam_height)
    int n_floor = 0;
    int n_box = 0;
};

// pitch_down / roll 규약을 그대로 뒤집어 정답 법선을 만든다.
//   pitch_down = asin(-n_z), roll = atan2(n_x, -n_y)
inline Eigen::Vector3d normal_from_angles(double pitch_down_deg, double roll_deg)
{
    const double deg2rad = 3.14159265358979323846 / 180.0;
    const double t = pitch_down_deg * deg2rad;
    const double r = roll_deg * deg2rad;
    return Eigen::Vector3d(std::cos(t) * std::sin(r), -std::cos(t) * std::cos(r), -std::sin(t));
}

// Box-Muller. std::normal_distribution은 구현마다 결과가 달라 직접 만든다.
inline double gaussian(Rng & rng)
{
    double u1 = rng.uniform01();
    if (u1 < 1e-300) {
        u1 = 1e-300;
    }
    const double u2 = rng.uniform01();
    return std::sqrt(-2.0 * std::log(u1)) *
           std::cos(2.0 * 3.14159265358979323846 * u2);
}

inline Scene make_scene(const SceneParams & sp)
{
    Scene s;
    const Eigen::Index n_px = static_cast<Eigen::Index>(sp.height) * sp.width;
    s.points = PointsRowMajor::Zero(n_px, 3);
    s.labels.assign(static_cast<std::size_t>(n_px), kInvalid);
    s.truth.n = normal_from_angles(sp.pitch_down_deg, sp.roll_deg);
    s.truth.d = sp.cam_height;

    Rng rng(sp.seed);
    const Eigen::Vector3d n = s.truth.n;

    for (int v = 0; v < sp.height; ++v) {
        for (int u = 0; u < sp.width; ++u) {
            const Eigen::Index i = static_cast<Eigen::Index>(v) * sp.width + u;
            // 광학 좌표계 광선 (z = 1 평면 기준)
            const Eigen::Vector3d ray(
                (static_cast<double>(u) - sp.cx) / sp.fx,
                (static_cast<double>(v) - sp.cy) / sp.fy,
                1.0);
            const double denom = n.dot(ray);
            if (denom > -1e-6) {
                continue;  // 광선이 바닥에서 멀어짐(지평선 위) -> 무효 픽셀
            }

            // 바닥 교점: n·(t*ray) + d = 0
            const double t_floor = -s.truth.d / denom;
            Eigen::Vector3d p = t_floor * ray;
            int label = kFloor;

            // 턱 윗면(높이 b)과의 교점: n·p + d = b
            if (sp.box_height > 0.0) {
                const double t_box = -(s.truth.d - sp.box_height) / denom;
                if (t_box > 0.0) {
                    const Eigen::Vector3d pb = t_box * ray;
                    if (pb[2] >= sp.box_z_min && pb[2] <= sp.box_z_max &&
                        std::abs(pb[0]) <= sp.box_half_width)
                    {
                        p = pb;  // 턱이 바닥을 가린다 (윗면이 더 가깝다)
                        label = kBox;
                    }
                }
            }

            if (p[2] <= 0.0 || p[2] > sp.z_far) {
                continue;
            }
            // depth 노이즈: 광선 방향으로 스케일 (실제 stereo depth 오차 방향)
            if (sp.noise_sigma > 0.0) {
                p *= (p[2] + gaussian(rng) * sp.noise_sigma) / p[2];
            }

            s.points.row(i) = p.transpose();
            s.labels[static_cast<std::size_t>(i)] = label;
            if (label == kBox) {
                ++s.n_box;
            } else {
                ++s.n_floor;
            }
        }
    }
    return s;
}

// 수직 벽만 있는 장면 (바닥 없음). tilt 조건이 벽을 실제로 배제하는지 확인용.
// 벽은 z = wall_z 평면이므로 법선이 ±z -> |n_y| = 0.
inline PointsRowMajor make_wall(const SceneParams & sp, double wall_z = 1.5)
{
    PointsRowMajor pts(static_cast<Eigen::Index>(sp.height) * sp.width, 3);
    Rng rng(sp.seed);
    Eigen::Index k = 0;
    for (int v = 0; v < sp.height; ++v) {
        for (int u = 0; u < sp.width; ++u) {
            const double z = wall_z + gaussian(rng) * sp.noise_sigma;
            pts.row(k++) = Eigen::Vector3d(
                (static_cast<double>(u) - sp.cx) * z / sp.fx,
                (static_cast<double>(v) - sp.cy) * z / sp.fy,
                z).transpose();
        }
    }
    pts.conservativeResize(k, 3);
    return pts;
}

}  // namespace synthetic
}  // namespace floor_ransac
