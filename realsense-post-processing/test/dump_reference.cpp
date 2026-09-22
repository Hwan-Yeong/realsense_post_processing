// C++ 쪽 기준값 덤프 (Phase 3 게이트용).
//
// gtest와 같은 합성 데이터/파라미터로 코어를 돌려 결과를 JSON으로 뱉는다.
// test_floor_ransac_py.py가 이 JSON의 시나리오를 pybind11 모듈로 그대로 재현해
// 값이 비트 단위로 같은지 확인한다 — 같으면 "튜너가 정말 임베디드와 같은 코드를 부른다"는 뜻.
//
// 시나리오 정의는 여기 한 곳에만 둔다 (양쪽에 적으면 언젠가 갈라진다).

#include <cstdio>
#include <string>
#include <vector>

#include "floor_ransac.hpp"
#include "floor_ransac_synthetic.hpp"

using namespace floor_ransac;
namespace syn = floor_ransac::synthetic;

namespace
{

struct Scenario
{
    std::string name;
    syn::SceneParams scene;
    Params params;
    bool wall = false;      // 바닥 대신 벽 장면
    double wall_z = 1.5;
    bool use_roi = true;    // false면 organized 전체를 후보로
};

Params base_params()
{
    Params p;
    p.thresh = 0.02;
    p.iters = 200;
    p.max_tilt_deg = 45.0;
    p.roi_bottom = 0.5;
    p.z_min = 0.2;
    p.z_max = 4.0;
    p.eval_n = 5000;
    p.seed = 0;
    return p;
}

std::vector<Scenario> scenarios()
{
    std::vector<Scenario> v;

    {   // 문서 기준 케이스
        Scenario s;
        s.name = "h035_pitch25";
        s.scene.cam_height = 0.35;
        s.scene.pitch_down_deg = 25.0;
        s.params = base_params();
        v.push_back(s);
    }
    {   // 실측 마운트 높이
        Scenario s;
        s.name = "h015_pitch25";
        s.scene.cam_height = 0.15;
        s.scene.pitch_down_deg = 25.0;
        s.params = base_params();
        v.push_back(s);
    }
    {   // roll이 있는 경우
        Scenario s;
        s.name = "h015_pitch15_roll5";
        s.scene.cam_height = 0.15;
        s.scene.pitch_down_deg = 15.0;
        s.scene.roll_deg = 5.0;
        s.scene.seed = 777;
        s.params = base_params();
        s.params.seed = 3;
        v.push_back(s);
    }
    {   // seed/iters를 바꿔도 일치하는지
        Scenario s;
        s.name = "h015_pitch25_seed99_iters500";
        s.scene.cam_height = 0.15;
        s.scene.pitch_down_deg = 25.0;
        s.params = base_params();
        s.params.seed = 99;
        s.params.iters = 500;
        s.params.eval_n = 3000;
        v.push_back(s);
    }
    {   // 실패 경로도 같은 상태값이 나와야 한다
        Scenario s;
        s.name = "wall_rejected";
        s.wall = true;
        s.use_roi = false;
        s.params = base_params();
        s.params.roi_bottom = 1.0;
        v.push_back(s);
    }
    {   // 기울기 기각
        Scenario s;
        s.name = "tilt_too_strict";
        s.scene.cam_height = 0.15;
        s.scene.pitch_down_deg = 40.0;
        s.scene.box_height = 0.0;
        s.params = base_params();
        s.params.max_tilt_deg = 20.0;
        v.push_back(s);
    }
    return v;
}

void print_scene_params(const syn::SceneParams & sp)
{
    std::printf(
        "\"scene\": {\"width\": %d, \"height\": %d, \"fx\": %.17g, \"fy\": %.17g, "
        "\"cx\": %.17g, \"cy\": %.17g, \"cam_height\": %.17g, \"pitch_down_deg\": %.17g, "
        "\"roll_deg\": %.17g, \"noise_sigma\": %.17g, \"box_height\": %.17g, "
        "\"box_z_min\": %.17g, \"box_z_max\": %.17g, \"box_half_width\": %.17g, "
        "\"z_far\": %.17g, \"seed\": %llu}",
        sp.width, sp.height, sp.fx, sp.fy, sp.cx, sp.cy, sp.cam_height,
        sp.pitch_down_deg, sp.roll_deg, sp.noise_sigma, sp.box_height,
        sp.box_z_min, sp.box_z_max, sp.box_half_width, sp.z_far,
        static_cast<unsigned long long>(sp.seed));
}

void print_params(const Params & p)
{
    std::printf(
        "\"params\": {\"thresh\": %.17g, \"iters\": %d, \"max_tilt_deg\": %.17g, "
        "\"roi_bottom\": %.17g, \"z_min\": %.17g, \"z_max\": %.17g, \"eval_n\": %d, "
        "\"seed\": %llu, \"min_candidates\": %d}",
        p.thresh, p.iters, p.max_tilt_deg, p.roi_bottom, p.z_min, p.z_max,
        p.eval_n, static_cast<unsigned long long>(p.seed), p.min_candidates);
}

}  // namespace

int main()
{
    std::printf("[\n");
    const std::vector<Scenario> all = scenarios();
    for (std::size_t k = 0; k < all.size(); ++k) {
        const Scenario & s = all[k];

        PointsRowMajor cand;
        int n_floor = 0;
        int n_box = 0;
        if (s.wall) {
            cand = syn::make_wall(s.scene, s.wall_z);
        } else {
            const syn::Scene scene = syn::make_scene(s.scene);
            n_floor = scene.n_floor;
            n_box = scene.n_box;
            cand = s.use_roi
                ? select_candidates(scene.points, s.scene.height, s.scene.width, s.params)
                : scene.points;
        }
        const Result r = fit_plane(cand, s.params);
        const Angles a = plane_angles(r.plane.n);

        std::printf("  {\"name\": \"%s\", \"wall\": %s, \"use_roi\": %s, \"wall_z\": %.17g, ",
                    s.name.c_str(), s.wall ? "true" : "false",
                    s.use_roi ? "true" : "false", s.wall_z);
        print_scene_params(s.scene);
        std::printf(", ");
        print_params(s.params);
        std::printf(
            ", \"result\": {\"ok\": %s, \"status\": \"%s\", \"candidates\": %d, "
            "\"n_floor\": %d, \"n_box\": %d, \"inliers\": %d, "
            "\"n\": [%.17g, %.17g, %.17g], \"d\": %.17g, "
            "\"pitch_down_deg\": %.17g, \"roll_deg\": %.17g, "
            "\"best_hypothesis\": %d, \"best_eval_inliers\": %d, "
            "\"rejected_tilt\": %d, \"rejected_degenerate\": %d, \"eval_points\": %d}}%s\n",
            r.ok ? "true" : "false", status_string(r.status), r.debug.candidates,
            n_floor, n_box, r.inliers,
            r.plane.n[0], r.plane.n[1], r.plane.n[2], r.plane.d,
            a.pitch_down_deg, a.roll_deg,
            r.debug.best_hypothesis, r.debug.best_eval_inliers,
            r.debug.rejected_tilt, r.debug.rejected_degenerate, r.debug.eval_points,
            (k + 1 < all.size()) ? "," : "");
    }
    std::printf("]\n");
    return 0;
}
