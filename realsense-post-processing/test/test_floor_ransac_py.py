#!/usr/bin/env python3
"""Phase 3 게이트: pybind11 모듈이 gtest와 '같은 코드'인지 확인한다.

test/dump_reference.cpp가 C++에서 돌린 결과(JSON)를 읽어,
같은 시나리오를 floor_ransac_py로 재현하고 값이 비트 단위로 같은지 본다.

  1. 합성 장면이 같은가 (같은 seed -> 같은 점군)
  2. 평면 적합 결과가 같은가 (법선/높이/inlier/채택 가설 인덱스까지)
  3. 실패 경로의 status까지 같은가

값이 다르면 튜너로 찾은 파라미터가 임베디드에서 재현되지 않는다는 뜻이므로 실패시킨다.

실행:
  python3 -m pytest test/test_floor_ransac_py.py -v
  (colcon build 후 `source install/setup.bash` 필요)
"""
import json
import os
import subprocess
import sys

import numpy as np
import pytest

try:
    import floor_ransac_py as fr
except ImportError as e:  # pragma: no cover
    pytest.skip(
        f"floor_ransac_py 미설치 ({e}). colcon build 후 source install/setup.bash 필요",
        allow_module_level=True,
    )

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
WS = os.path.dirname(PKG)


def _find_dumper():
    for p in (
        os.path.join(WS, "build", "realsense_post_processing", "dump_reference"),
        os.path.join(PKG, "build", "dump_reference"),
    ):
        if os.path.isfile(p) and os.access(p, os.X_OK):
            return p
    return None


@pytest.fixture(scope="module")
def reference():
    exe = _find_dumper()
    if exe is None:
        pytest.skip("dump_reference 실행 파일 없음. colcon build --cmake-args -DBUILD_TESTING=ON")
    out = subprocess.run([exe], capture_output=True, text=True, check=True).stdout
    return json.loads(out)


def _scene_params(d):
    sp = fr.synthetic.SceneParams()
    for k, v in d.items():
        setattr(sp, k, v)
    return sp


def _params(d):
    p = fr.Params()
    for k, v in d.items():
        setattr(p, k, v)
    return p


def _run(case):
    """C++ dump_reference의 main()과 같은 순서로 Python에서 재현."""
    sp = _scene_params(case["scene"])
    p = _params(case["params"])
    n_floor = n_box = 0
    if case["wall"]:
        cand = fr.synthetic.make_wall(sp, case["wall_z"])
    else:
        scene = fr.synthetic.make_scene(sp)
        n_floor, n_box = scene.n_floor, scene.n_box
        cand = (
            fr.select_candidates(scene.points, sp.height, sp.width, p)
            if case["use_roi"]
            else scene.points
        )
    r = fr.fit_plane(cand, p)
    return r, n_floor, n_box


def test_reference_has_cases(reference):
    assert len(reference) >= 5
    names = [c["name"] for c in reference]
    assert "h015_pitch25" in names, "실측 마운트 높이 케이스가 빠졌다"
    assert "wall_rejected" in names, "실패 경로 케이스가 빠졌다"


@pytest.mark.parametrize("idx", range(6))
def test_matches_cpp_reference(reference, idx):
    if idx >= len(reference):
        pytest.skip("케이스 없음")
    case = reference[idx]
    exp = case["result"]
    r, n_floor, n_box = _run(case)

    # 합성 장면이 먼저 같아야 비교에 의미가 있다
    assert n_floor == exp["n_floor"], f"{case['name']}: 합성 바닥 점 수가 다르다"
    assert n_box == exp["n_box"], f"{case['name']}: 합성 턱 점 수가 다르다"
    assert r.debug.candidates == exp["candidates"], f"{case['name']}: 후보점 수가 다르다"
    assert r.debug.eval_points == exp["eval_points"]

    # 성공/실패와 상태
    assert r.ok == exp["ok"], f"{case['name']}: ok 불일치"
    assert r.status_string == exp["status"], f"{case['name']}: status 불일치"

    # RANSAC 진행 자체가 같아야 한다 (같은 seed -> 같은 가설 순서)
    assert r.debug.best_hypothesis == exp["best_hypothesis"], f"{case['name']}: 채택 가설 다름"
    assert r.debug.best_eval_inliers == exp["best_eval_inliers"]
    assert r.debug.rejected_tilt == exp["rejected_tilt"]
    assert r.debug.rejected_degenerate == exp["rejected_degenerate"]

    if not exp["ok"]:
        return

    assert r.inliers == exp["inliers"], f"{case['name']}: inlier 수 다름"

    # 부동소수 결과는 비트 단위로 같아야 한다 (같은 컴파일된 코드를 부르므로)
    n = np.asarray(r.plane.n).ravel()
    np.testing.assert_array_equal(n, np.array(exp["n"]), err_msg=f"{case['name']}: 법선 불일치")
    assert r.plane.d == exp["d"], f"{case['name']}: d 불일치"

    a = fr.plane_angles(r.plane.n)
    assert a.pitch_down_deg == exp["pitch_down_deg"]
    assert a.roll_deg == exp["roll_deg"]


def test_normal_convention_across_all_cases(reference):
    """어떤 케이스에서도 법선은 n[1] < 0 (카메라 위쪽)."""
    for case in reference:
        r, _, _ = _run(case)
        if r.ok:
            assert np.asarray(r.plane.n).ravel()[1] < 0, f"{case['name']}: 법선이 아래를 향함"


def test_point_heights_matches_plane_equation():
    """point_heights가 n·p + d 그대로인지 (numpy 직접 계산과 비교)."""
    sp = fr.synthetic.SceneParams()
    sp.cam_height = 0.15
    scene = fr.synthetic.make_scene(sp)
    p = fr.Params()
    cand = fr.select_candidates(scene.points, sp.height, sp.width, p)
    r = fr.fit_plane(cand, p)
    assert r.ok

    h_cpp = np.asarray(fr.point_heights(cand, r.plane)).ravel()
    h_np = cand @ np.asarray(r.plane.n).ravel() + r.plane.d
    np.testing.assert_allclose(h_cpp, h_np, rtol=0, atol=1e-15)


def test_same_seed_is_deterministic_in_python():
    sp = fr.synthetic.SceneParams()
    sp.cam_height = 0.15
    scene = fr.synthetic.make_scene(sp)
    p = fr.Params()
    p.seed = 1234
    cand = fr.select_candidates(scene.points, sp.height, sp.width, p)

    a = fr.fit_plane(cand, p)
    b = fr.fit_plane(cand, p)
    assert a.plane.d == b.plane.d
    assert a.inliers == b.inliers
    assert a.debug.best_hypothesis == b.debug.best_hypothesis


def test_different_seed_changes_hypothesis_order():
    """seed가 실제로 먹는지 (안 먹으면 반복성 검사가 무의미해진다)."""
    sp = fr.synthetic.SceneParams()
    scene = fr.synthetic.make_scene(sp)
    p0, p1 = fr.Params(), fr.Params()
    p0.seed, p1.seed = 0, 1
    cand = fr.select_candidates(scene.points, sp.height, sp.width, p0)
    a = fr.fit_plane(cand, p0)
    b = fr.fit_plane(cand, p1)
    assert a.debug.hypothesis_inliers != b.debug.hypothesis_inliers


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
