// floor_ransac 코어의 pybind11 바인딩.
//
// Python 튜너는 RANSAC을 따로 구현하지 않고 "임베디드에 들어갈 바로 그 코드"를 호출한다.
// 튜너에서 찾은 파라미터가 ROS 노드에서 그대로 재현되는 이유가 이것이다.
//
// 합성 장면 생성기(synthetic)도 함께 노출한다. gtest와 같은 seed로 같은 점군을 만들 수 있어
// "정말 같은 코드인가"를 양쪽에서 같은 입력으로 비교할 수 있다.

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <sstream>

#include "floor_ransac.hpp"
#include "floor_ransac_synthetic.hpp"

namespace py = pybind11;
using namespace floor_ransac;

PYBIND11_MODULE(floor_ransac_py, m)
{
    m.doc() = "바닥 평면 RANSAC 코어 (C++ 단일 구현) — ROS 노드/튜너/테스트 공용";

    py::enum_<Status>(m, "Status")
        .value("OK", Status::kOk)
        .value("TOO_FEW_CANDIDATES", Status::kTooFewCandidates)
        .value("NO_HYPOTHESIS", Status::kNoHypothesis)
        .value("TOO_FEW_INLIERS", Status::kTooFewInliers)
        .value("TILT_REJECTED_AFTER_REFIT", Status::kTiltRejectedAfterRefit);

    py::class_<Params>(m, "Params")
        .def(py::init<>())
        .def_readwrite("thresh", &Params::thresh)
        .def_readwrite("iters", &Params::iters)
        .def_readwrite("max_tilt_deg", &Params::max_tilt_deg)
        .def_readwrite("roi_bottom", &Params::roi_bottom)
        .def_readwrite("z_min", &Params::z_min)
        .def_readwrite("z_max", &Params::z_max)
        .def_readwrite("eval_n", &Params::eval_n)
        .def_readwrite("seed", &Params::seed)
        .def_readwrite("min_candidates", &Params::min_candidates)
        .def("__repr__", [](const Params & p) {
            std::ostringstream os;
            os << "Params(thresh=" << p.thresh << ", iters=" << p.iters
               << ", max_tilt_deg=" << p.max_tilt_deg << ", roi_bottom=" << p.roi_bottom
               << ", z=[" << p.z_min << ", " << p.z_max << "], eval_n=" << p.eval_n
               << ", seed=" << p.seed << ")";
            return os.str();
        });

    py::class_<Plane>(m, "Plane")
        .def(py::init<>())
        .def_readwrite("n", &Plane::n)
        .def_readwrite("d", &Plane::d)
        .def("__repr__", [](const Plane & pl) {
            std::ostringstream os;
            os << "Plane(n=[" << pl.n[0] << ", " << pl.n[1] << ", " << pl.n[2]
               << "], d=" << pl.d << ")";
            return os.str();
        });

    py::class_<Angles>(m, "Angles")
        .def_readonly("pitch_down_deg", &Angles::pitch_down_deg)
        .def_readonly("roll_deg", &Angles::roll_deg);

    py::class_<Debug>(m, "Debug")
        .def_readonly("candidates", &Debug::candidates)
        .def_readonly("eval_points", &Debug::eval_points)
        .def_readonly("hypotheses_tried", &Debug::hypotheses_tried)
        .def_readonly("rejected_degenerate", &Debug::rejected_degenerate)
        .def_readonly("rejected_tilt", &Debug::rejected_tilt)
        .def_readonly("best_hypothesis", &Debug::best_hypothesis)
        .def_readonly("best_eval_inliers", &Debug::best_eval_inliers)
        .def_readonly("hypothesis_inliers", &Debug::hypothesis_inliers)
        .def_readonly("hypothesis_plane", &Debug::hypothesis_plane);

    py::class_<Result>(m, "Result")
        .def_readonly("ok", &Result::ok)
        .def_readonly("status", &Result::status)
        .def_readonly("plane", &Result::plane)
        .def_readonly("inliers", &Result::inliers)
        .def_readonly("debug", &Result::debug)
        .def_property_readonly(
            "status_string", [](const Result & r) { return status_string(r.status); });

    m.def("fit_plane", &fit_plane, py::arg("points"), py::arg("params"),
          "(N,3) float64 후보점에서 바닥 평면 적합. GIL을 놓지 않는다(입력 배열 참조 중).");

    m.def("plane_angles", &plane_angles, py::arg("n"),
          "pitch_down = asin(-n_z), roll = atan2(n_x, -n_y) [deg]");

    m.def("select_candidates", &select_candidates,
          py::arg("organized"), py::arg("height"), py::arg("width"), py::arg("params"),
          "organized (H*W,3) 점군 -> 하단 ROI + z 범위 후보점");

    m.def("point_heights", &point_heights, py::arg("points"), py::arg("plane"),
          "각 점의 바닥 기준 높이 n·p + d");

    m.def("status_string", &status_string);

    // ---- 검증용 합성 장면 ------------------------------------------------
    py::module_ syn = m.def_submodule("synthetic", "검증용 합성 바닥 장면 (gtest와 동일)");

    py::class_<synthetic::SceneParams>(syn, "SceneParams")
        .def(py::init<>())
        .def_readwrite("width", &synthetic::SceneParams::width)
        .def_readwrite("height", &synthetic::SceneParams::height)
        .def_readwrite("fx", &synthetic::SceneParams::fx)
        .def_readwrite("fy", &synthetic::SceneParams::fy)
        .def_readwrite("cx", &synthetic::SceneParams::cx)
        .def_readwrite("cy", &synthetic::SceneParams::cy)
        .def_readwrite("cam_height", &synthetic::SceneParams::cam_height)
        .def_readwrite("pitch_down_deg", &synthetic::SceneParams::pitch_down_deg)
        .def_readwrite("roll_deg", &synthetic::SceneParams::roll_deg)
        .def_readwrite("noise_sigma", &synthetic::SceneParams::noise_sigma)
        .def_readwrite("box_height", &synthetic::SceneParams::box_height)
        .def_readwrite("box_z_min", &synthetic::SceneParams::box_z_min)
        .def_readwrite("box_z_max", &synthetic::SceneParams::box_z_max)
        .def_readwrite("box_half_width", &synthetic::SceneParams::box_half_width)
        .def_readwrite("z_far", &synthetic::SceneParams::z_far)
        .def_readwrite("seed", &synthetic::SceneParams::seed);

    py::class_<synthetic::Scene>(syn, "Scene")
        .def_readonly("points", &synthetic::Scene::points)
        .def_readonly("labels", &synthetic::Scene::labels)
        .def_readonly("truth", &synthetic::Scene::truth)
        .def_readonly("n_floor", &synthetic::Scene::n_floor)
        .def_readonly("n_box", &synthetic::Scene::n_box);

    syn.attr("INVALID") = static_cast<int>(synthetic::kInvalid);
    syn.attr("FLOOR") = static_cast<int>(synthetic::kFloor);
    syn.attr("BOX") = static_cast<int>(synthetic::kBox);

    syn.def("make_scene", &synthetic::make_scene, py::arg("params"));
    syn.def("make_wall", &synthetic::make_wall, py::arg("params"), py::arg("wall_z") = 1.5);
    syn.def("normal_from_angles", &synthetic::normal_from_angles,
            py::arg("pitch_down_deg"), py::arg("roll_deg"));
}
