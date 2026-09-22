"""C++ RANSAC 코어(floor_ransac_py) 로더와 얇은 래퍼.

Python은 RANSAC을 구현하지 않는다. include/floor_ransac.hpp + src/floor_ransac.cpp가
알고리즘의 단일 소스이고, 튜너는 pybind11로 '컴파일된 바로 그 코드'를 호출한다.
(Python에 따로 구현하면 튜너로 찾은 파라미터가 임베디드에서 재현되지 않는다.)
"""
import glob
import os
import sys

import numpy as np

_BUILD_HINT = """
floor_ransac_py 모듈을 찾을 수 없습니다.

  cd <워크스페이스 루트>
  colcon build --packages-select realsense_post_processing
  source install/setup.bash

pybind11이 없으면 빌드가 이 모듈만 조용히 건너뜁니다:
  sudo apt install python3-pybind11
"""


def _candidate_dirs():
    here = os.path.dirname(os.path.abspath(__file__))          # scripts/common
    pkg = os.path.dirname(os.path.dirname(here))               # realsense-post-processing
    ws = os.path.dirname(pkg)                                  # 워크스페이스 루트
    pats = [
        os.path.join(ws, "install", "realsense_post_processing", "lib", "python*",
                     "site-packages"),
        os.path.join(ws, "build", "realsense_post_processing"),
        os.path.join(pkg, "build"),
    ]
    out = []
    for p in pats:
        out.extend(sorted(glob.glob(p)))
    return out


def _load():
    try:
        import floor_ransac_py  # noqa: F401
        return floor_ransac_py
    except ImportError:
        pass
    # source install/setup.bash 를 잊은 경우를 대비해 빌드/설치 트리를 직접 찾아본다
    for d in _candidate_dirs():
        if glob.glob(os.path.join(d, "floor_ransac_py*.so")):
            sys.path.insert(0, d)
            try:
                import floor_ransac_py  # noqa: F401
                return floor_ransac_py
            except ImportError:
                sys.path.pop(0)
    raise ImportError(_BUILD_HINT)


fr = _load()

Status = fr.Status

# Params에서 저장/복원 대상 필드
PARAM_FIELDS = (
    "thresh", "iters", "max_tilt_deg", "roi_bottom",
    "z_min", "z_max", "eval_n", "seed", "min_candidates",
)

# 기본값은 C++ Params의 기본값을 그대로 쓴다 (Python이 따로 기본값을 갖지 않게)
DEFAULTS = {f: getattr(fr.Params(), f) for f in PARAM_FIELDS}


def make_params(**kw):
    """dict/키워드 -> C++ Params. 모르는 키는 에러로 알려준다."""
    p = fr.Params()
    for k, v in kw.items():
        if k not in PARAM_FIELDS:
            raise KeyError(f"알 수 없는 RANSAC 파라미터: {k} (가능: {', '.join(PARAM_FIELDS)})")
        setattr(p, k, type(getattr(p, k))(v))
    return p


def params_to_dict(p):
    return {f: getattr(p, f) for f in PARAM_FIELDS}


def as_core_points(xyz):
    """(H,W,3) 또는 (N,3) 점군 -> C++이 받는 (N,3) float64 C-contiguous.

    to_points()는 float32를 주는데 코어는 float64를 요구한다. 이 변환이 프레임당
    수 MB 복사라서 튜너는 프레임별로 캐시해 두고 재사용한다.
    """
    a = np.asarray(xyz)
    return np.ascontiguousarray(a.reshape(-1, 3), dtype=np.float64)


def select_candidates(points64, height, width, params):
    """하단 ROI + z 범위 후보점 추출 (C++)."""
    return fr.select_candidates(points64, height, width, params)


def fit_plane(candidates64, params):
    """평면 적합 (C++). Result를 그대로 반환."""
    return fr.fit_plane(candidates64, params)


def fit_organized(points64, height, width, params):
    """organized 점군에서 후보 추출 + 적합을 한 번에. (result, candidates) 반환."""
    cand = fr.select_candidates(points64, height, width, params)
    return fr.fit_plane(cand, params), cand


def plane_angles(n):
    """pitch_down = asin(-n_z), roll = atan2(n_x, -n_y) [deg] (C++ 규약 그대로)."""
    a = fr.plane_angles(np.asarray(n, dtype=np.float64).reshape(3))
    return a.pitch_down_deg, a.roll_deg


def plane_nd(result):
    """Result -> (n(3,) float64, d)."""
    return np.asarray(result.plane.n, dtype=np.float64).reshape(3), float(result.plane.d)


def make_plane(n, d):
    """(n, d) -> C++ Plane (fixed 모드에서 기준면을 넘길 때 사용)."""
    pl = fr.Plane()
    pl.n = np.asarray(n, dtype=np.float64).reshape(3)
    pl.d = float(d)
    return pl


def heights(points64, n, d):
    """바닥 기준 높이 n·p + d. numpy로 계산 (코어 호출보다 빠르고 결과는 동일)."""
    return points64 @ np.asarray(n, dtype=np.float64).reshape(3) + float(d)


def module_path():
    return getattr(fr, "__file__", "<unknown>")
