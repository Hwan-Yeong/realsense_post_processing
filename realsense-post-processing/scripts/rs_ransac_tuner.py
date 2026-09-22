#!/usr/bin/env python3
"""RANSAC 바닥 추출 인터랙티브 튜너 (.db3 재생 + 실시간 파라미터 조정)

파라미터를 바꿀 때마다 CLI를 다시 돌리지 않고, 같은 프레임에서 즉시 재계산해 비교한다.

RANSAC 자체는 Python에 구현되어 있지 않다. C++ 코어(floor_ransac_py)를 그대로 호출하므로
여기서 찾은 파라미터가 ROS 노드/임베디드에서 그대로 재현된다.

화면
  왼쪽 = 세트 A, 오른쪽 = 세트 B (같은 프레임, 다른 파라미터)
  controls 창의 트랙바는 '편집 중인 세트'에만 적용된다 (Tab으로 전환)
  흰 가로선 = RANSAC 후보 ROI 상단

키
  space  재생/정지          . , 다음/이전 프레임
  Tab    편집 세트 A<->B    c   편집 세트를 반대편에 복사
  v      뷰 전환            3/p 현재 프레임 3D 보기 (open3d)
  f      fixed 기준 평면을 현재 프레임에서 재추출
  k      반복성 검사 (seed만 바꿔 N회 실행 -> 높이/pitch/roll/inlier std)
  r      현재 설정으로 전체 프레임 일괄 실행 -> 요약 표
  s      현재 파라미터 저장 (YAML, 없으면 JSON)
  q      종료

예:
  python3 rs_ransac_tuner.py bag/20260922_174035.db3
  python3 rs_ransac_tuner.py bag/A.db3 --params tuned.yaml --max_frames 100

좌표계: RealSense 카메라 좌표 (x 오른쪽, y 아래, z 전방), 단위 m
평면 n·p + d = 0, n[1] < 0 고정 -> d = 카메라 높이
"""
import argparse
import copy
import json
import os
import sys
import time
from dataclasses import dataclass, asdict, fields, replace

import cv2
import numpy as np
import pyrealsense2 as rs

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import core, filters as flt, metrics, playback, render  # noqa: E402

try:
    import yaml
except ImportError:
    yaml = None

WIN = "RANSAC tuner: A | B"
CTRL = "controls"
REPEAT_N = 30  # 반복성 검사 실행 횟수

KEYS_HELP = """\
  space 재생/정지   . , 다음/이전 프레임   Tab 편집세트 A<->B   c 편집세트 복사
  v 뷰 전환         3 또는 p 3D 보기       f fixed 기준면 재추출
  k 반복성 검사     r 전체 프레임 일괄 실행  s 파라미터 저장     q 종료"""


# ---------------------------------------------------------------- 파라미터 세트
# 트랙바가 int만 다루므로 세트도 전부 int로 들고 있다가 쓸 때 실수로 바꾼다.
@dataclass
class ParamSet:
    # RANSAC
    thresh_mm: int = 20
    iters: int = 200
    max_tilt_deg: int = 45
    roi_bottom_pct: int = 50
    eval_n_x100: int = 50          # 5000
    z_min_cm: int = 20
    z_max_cm: int = 400
    seed: int = 0
    seed_random: int = 0           # 1이면 매 재계산마다 다른 seed
    # 입력 / 모드
    source: int = 1                # 0 = raw, 1 = filtered
    plane_mode: int = 0            # 0 = per_frame, 1 = fixed
    # 표시 / 지표
    obs_mm: int = 20
    hmax_mm: int = 100
    hres_mm: int = 30
    view: int = 0
    # 필터
    decim: int = 2
    spatial: int = 0
    sp_mag: int = 2
    sp_alpha_x100: int = 50
    sp_delta: int = 20
    temporal: int = 0
    tp_alpha_x100: int = 40
    tp_persist: int = 0
    holefill: int = 0

    # ---- 파생 ----
    def ransac_params(self, seed_override=None):
        return core.make_params(
            thresh=self.thresh_mm / 1000.0,
            iters=self.iters,
            max_tilt_deg=float(self.max_tilt_deg),
            roi_bottom=self.roi_bottom_pct / 100.0,
            z_min=self.z_min_cm / 100.0,
            z_max=self.z_max_cm / 100.0,
            eval_n=self.eval_n_x100 * 100,
            seed=self.seed if seed_override is None else seed_override,
        )

    def filter_config(self):
        # threshold 필터 범위와 RANSAC 후보 z 범위는 같은 값을 쓴다 (기존 스크립트와 동일)
        return flt.FilterConfig(
            decim=self.decim, zmin=self.z_min_cm / 100.0, zmax=self.z_max_cm / 100.0,
            spatial=bool(self.spatial), sp_mag=float(self.sp_mag),
            sp_alpha=self.sp_alpha_x100 / 100.0, sp_delta=float(self.sp_delta),
            temporal=bool(self.temporal), tp_alpha=self.tp_alpha_x100 / 100.0,
            tp_delta=20.0, tp_persist=self.tp_persist, holefill=bool(self.holefill),
        )

    @property
    def thresh(self):
        return self.thresh_mm / 1000.0

    @property
    def obs(self):
        return self.obs_mm / 1000.0

    @property
    def view_name(self):
        return render.VIEWS[self.view % len(render.VIEWS)]

    @property
    def source_name(self):
        return "raw" if self.source == 0 else "filtered"

    @property
    def mode_name(self):
        return "per_frame" if self.plane_mode == 0 else "fixed"


# 트랙바 정의: (표시 이름, 필드, 최소, 최대)
TRACKBARS = [
    ("thresh [mm]",     "thresh_mm",     5, 50),
    ("iters",           "iters",        10, 1000),
    ("max_tilt [deg]",  "max_tilt_deg",  5, 80),
    ("roi_bottom [%]",  "roi_bottom_pct", 10, 100),
    ("eval_n /100",     "eval_n_x100",  10, 200),
    ("z_min [cm]",      "z_min_cm",     10, 400),
    ("z_max [cm]",      "z_max_cm",     20, 600),
    ("seed",            "seed",          0, 99),
    ("seed random",     "seed_random",   0, 1),
    ("src 0raw 1flt",   "source",        0, 1),
    ("plane 0pf 1fix",  "plane_mode",    0, 1),
    ("obs [mm]",        "obs_mm",        0, 100),
    ("hmax [mm]",       "hmax_mm",      20, 300),
    ("hres [mm]",       "hres_mm",       5, 100),
    ("view",            "view",          0, len(render.VIEWS) - 1),
    ("decim",           "decim",         1, 4),
    ("spatial",         "spatial",       0, 1),
    ("sp_mag",          "sp_mag",        1, 5),
    ("sp_alpha x100",   "sp_alpha_x100", 1, 100),
    ("sp_delta",        "sp_delta",      1, 50),
    ("temporal",        "temporal",      0, 1),
    ("tp_alpha x100",   "tp_alpha_x100", 1, 100),
    ("tp_persist",      "tp_persist",    0, 8),
    ("holefill",        "holefill",      0, 1),
]


# ---------------------------------------------------------------- 필터 캐시
class FilterCache:
    """필터 적용 결과 캐시.

    temporal은 이전 프레임 상태를 들고 있어 순서 의존적이다. 켜져 있으면
    임의 프레임으로 점프해도 0번부터 순차로 다시 돌려야 결과가 맞는다.
    (원래 순서대로 통과시키지 않으면 잔상 비교가 재현되지 않는다.)
    """

    def __init__(self, frames):
        self.frames = frames
        self._sig = None
        self._cache = {}
        self._chain = None
        self._max_done = -1

    @staticmethod
    def _signature(cfg):
        return tuple(sorted(cfg.to_dict().items()))

    def _reset(self, cfg):
        self._sig = self._signature(cfg)
        self._cache.clear()
        self._chain = flt.build_filters(cfg)
        self._max_done = -1

    def get(self, idx, cfg):
        if self._signature(cfg) != self._sig:
            self._reset(cfg)

        if idx in self._cache:
            return self._cache[idx]

        if cfg.order_sensitive:
            if idx < self._max_done:      # 뒤로 점프: 체인 상태를 되돌릴 수 없어 처음부터
                self._reset(cfg)
            start = self._max_done + 1
            if start == 0 and idx > 0:
                print(f"[filter] temporal 켜짐 -> 0~{idx} 프레임 순차 재계산 중...")
            for i in range(start, idx + 1):
                out = flt.apply_filters(self._chain, self.frames[i])
                _keep(out)
                self._cache[i] = out
                self._max_done = i
            return self._cache[idx]

        # 순서 무관: 필요한 프레임만
        out = flt.apply_filters(flt.build_filters(cfg), self.frames[idx])
        _keep(out)
        self._cache[idx] = out
        return out


def _keep(frame):
    """필터 출력도 나중에 다시 읽으려면 keep()이 필요하다."""
    try:
        frame.keep()
    except Exception:
        pass


# ---------------------------------------------------------------- 한 세트 계산
class Computed:
    __slots__ = ("xyz", "height", "valid", "result", "stats", "ratio", "ms",
                 "roi_top", "n", "d", "pitch", "roll", "n_cand")

    def __init__(self):
        self.result = None


def compute(pset, idx, frames, cache, pc, fixed_plane, seed_override=None):
    """세트 하나를 현재 프레임에 대해 계산."""
    frame = frames[idx] if pset.source == 0 else cache.get(idx, pset.filter_config())
    xyz = playback.to_points(pc, frame)
    h_px, w_px = xyz.shape[:2]
    pts64 = core.as_core_points(xyz)
    params = pset.ransac_params(seed_override)

    c = Computed()
    c.xyz = xyz
    c.roi_top = metrics.roi_top(h_px, params.roi_bottom)

    t0 = time.perf_counter()
    if pset.plane_mode == 1 and fixed_plane is not None:
        n, d = fixed_plane
        c.result = None
        cand = core.select_candidates(pts64, h_px, w_px, params)
        c.n_cand = int(cand.shape[0])
    else:
        res, cand = core.fit_organized(pts64, h_px, w_px, params)
        c.result = res
        c.n_cand = int(cand.shape[0])
        if not res.ok:
            c.ms = (time.perf_counter() - t0) * 1000
            c.n = c.d = None
            return c
        n, d = core.plane_nd(res)
    c.ms = (time.perf_counter() - t0) * 1000

    c.n, c.d = n, float(d)
    c.pitch, c.roll = core.plane_angles(n)
    c.height, c.valid, c.stats = metrics.analyze(xyz, n, d, pset.obs)

    cm = metrics.candidate_mask(xyz, params.z_min, params.z_max, c.roi_top)
    denom = max(int(np.count_nonzero(cm)), 1)
    c.ratio = float(np.count_nonzero(cm & (np.abs(c.height) < pset.thresh)) / denom)
    return c


def panel(pset, c, idx, size, tag, editing):
    if c.n is None:
        img = render.blank(size, f"{tag}: floor NOT found"
                                 f" ({c.result.status_string if c.result else '-'})")
        render.label(img, f"cand {c.n_cand}  RANSAC {c.ms:.1f} ms", 40)
        return img

    img = render.draw(
        c.xyz, c.height, c.valid, pset.view_name, size, c.roi_top,
        thresh=pset.thresh, hmax=pset.hmax_mm / 1000.0,
        hres=pset.hres_mm / 1000.0, z_range=(pset.z_min_cm / 100.0, pset.z_max_cm / 100.0))

    mark = " <EDIT>" if editing else ""
    render.label(img, f"[{tag}{mark}] {pset.source_name} / {pset.mode_name} / "
                      f"{pset.view_name} {c.xyz.shape[1]}x{c.xyz.shape[0]}", 20)
    render.label(img, f"cam h {c.d:.3f}m  pitch {c.pitch:.1f}  roll {c.roll:.1f}  "
                      f"inlier {c.ratio * 100:.0f}%", 40)
    s15 = c.stats[(1.0, 1.5)]
    std_s = "-" if np.isnan(s15[0]) else f"{s15[0] * 1000:.1f}mm"
    fp_s = "-" if np.isnan(s15[1]) else f"{s15[1] * 100:.1f}%"
    render.label(img, f"z1.0-1.5m: std {std_s}  >{pset.obs * 100:.0f}cm {fp_s}", 60)
    hyp = f"hyp#{c.result.debug.best_hypothesis} inl {c.result.debug.best_eval_inliers}" \
        if c.result else "fixed plane"
    render.label(img, f"thr {pset.thresh_mm}mm it {pset.iters} tilt {pset.max_tilt_deg} "
                      f"roi {pset.roi_bottom_pct}%  cand {c.n_cand}  {hyp}  "
                      f"{c.ms:.1f} ms", 80)
    return img


# ---------------------------------------------------------------- 분석 동작
def repeatability(pset, idx, frames, cache, pc, fixed_plane, n_runs=REPEAT_N):
    """같은 프레임에서 seed만 바꿔 N회 실행. 파라미터가 불안정한지 판단하는 핵심 지표."""
    hs, ps, rs_, ratios, mss, fails = [], [], [], [], [], 0
    for s in range(n_runs):
        c = compute(pset, idx, frames, cache, pc, fixed_plane, seed_override=s)
        if c.n is None:
            fails += 1
            continue
        hs.append(c.d); ps.append(c.pitch); rs_.append(c.roll)
        ratios.append(c.ratio); mss.append(c.ms)
    print(f"\n[반복성 검사] frame #{idx}, seed 0~{n_runs - 1}, {pset.source_name}/"
          f"{pset.mode_name}  실패 {fails}/{n_runs}")
    if not hs:
        print("  전부 실패 — roi_bottom / max_tilt / z 범위를 확인하세요")
        return
    print(f"  cam h  {np.mean(hs):.4f} m   std {np.std(hs) * 1000:.2f} mm   "
          f"(min {min(hs):.4f} / max {max(hs):.4f})")
    print(f"  pitch  {np.mean(ps):.3f} deg std {np.std(ps):.3f} deg")
    print(f"  roll   {np.mean(rs_):.3f} deg std {np.std(rs_):.3f} deg")
    print(f"  inlier {np.mean(ratios) * 100:.1f} %   std {np.std(ratios) * 100:.2f} %p")
    print(f"  RANSAC {np.mean(mss):.1f} ms")
    if np.std(ps) > 0.5:
        print("  ! pitch std > 0.5 deg — iters를 올리거나 roi/thresh를 다시 보세요")
    if np.std(hs) * 1000 > 5:
        print("  ! 높이 std > 5 mm — 이 설정으로는 3 cm 턱 판정이 흔들립니다")


def batch_run(sets, frames, cache, pc, fixed_plane, labels=("A", "B")):
    """전체 프레임 일괄 실행 -> rs_floor_ransac_test.py와 같은 요약 표."""
    log = metrics.new_log(labels)
    fails = {k: 0 for k in labels}
    t0 = time.perf_counter()
    print(f"\n[일괄 실행] {len(frames)} 프레임 ...")
    for i in range(len(frames)):
        for tag, pset in zip(labels, sets):
            c = compute(pset, i, frames, cache, pc, fixed_plane)
            if c.n is None:
                fails[tag] += 1
                continue
            metrics.record(log[tag], c.d, c.pitch, c.roll, c.ratio, c.ms, c.stats)
        if (i + 1) % 50 == 0:
            print(f"  {i + 1}/{len(frames)}")
    el = time.perf_counter() - t0
    print(f"완료 {el:.1f}s")
    for tag, pset in zip(labels, sets):
        print(f"  {tag}: {pset.source_name}/{pset.mode_name} "
              f"thresh {pset.thresh_mm}mm iters {pset.iters} tilt {pset.max_tilt_deg} "
              f"roi {pset.roi_bottom_pct}%  실패 {fails[tag]}/{len(frames)}")
    metrics.print_band_table(log, sets[0].obs, keys=labels, labels=labels)
    metrics.print_stability(log, keys=labels)


def save_params(sets, path):
    data = {
        "_note": "rs_ransac_tuner.py 저장. RANSAC 값은 C++ 코어(floor_ransac)에 그대로 들어간다.",
        "A": asdict(sets[0]),
        "B": asdict(sets[1]),
        "derived": {
            tag: {"ransac": core.params_to_dict(s.ransac_params()),
                  "filters": s.filter_config().to_dict()}
            for tag, s in zip(("A", "B"), sets)
        },
    }
    if yaml is not None and path.endswith((".yaml", ".yml")):
        with open(path, "w") as f:
            yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)
    else:
        if not path.endswith(".json"):
            path += ".json"
        with open(path, "w") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
    print(f"[save] {path}")


def load_params(path):
    with open(path) as f:
        data = yaml.safe_load(f) if (yaml and path.endswith((".yaml", ".yml"))) \
            else json.load(f)
    known = {f.name for f in fields(ParamSet)}
    out = []
    for tag in ("A", "B"):
        d = {k: int(v) for k, v in (data.get(tag) or {}).items() if k in known}
        out.append(ParamSet(**d))
    print(f"[load] {path}")
    return out


# ---------------------------------------------------------------- 트랙바
def build_controls(pset):
    cv2.namedWindow(CTRL, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(CTRL, 420, 800)
    for name, field, lo, hi in TRACKBARS:
        cv2.createTrackbar(name, CTRL, lo, hi, lambda _v: None)
        cv2.setTrackbarMin(name, CTRL, lo)
        cv2.setTrackbarPos(name, CTRL, int(np.clip(getattr(pset, field), lo, hi)))


def push_controls(pset):
    """세트 값을 트랙바에 반영 (A/B 전환 시)."""
    for name, field, lo, hi in TRACKBARS:
        cv2.setTrackbarPos(name, CTRL, int(np.clip(getattr(pset, field), lo, hi)))


def pull_controls(pset):
    """트랙바 -> 세트. 바뀐 게 있으면 True.

    콜백에서 계산하지 않고 메인 루프에서 폴링한다 (콜백 재진입/무거운 계산 방지).
    """
    changed = False
    for name, field, _lo, _hi in TRACKBARS:
        v = cv2.getTrackbarPos(name, CTRL)
        if v != getattr(pset, field):
            setattr(pset, field, v)
            changed = True
    # z_min >= z_max 같은 조합은 후보가 0이 되므로 막는다
    if pset.z_min_cm >= pset.z_max_cm:
        pset.z_min_cm = max(10, pset.z_max_cm - 10)
        cv2.setTrackbarPos("z_min [cm]", CTRL, pset.z_min_cm)
        changed = True
    return changed


# ---------------------------------------------------------------- main
def main():
    p = argparse.ArgumentParser(description="RANSAC 바닥 추출 인터랙티브 튜너")
    p.add_argument("bag")
    p.add_argument("--params", help="시작 시 불러올 파라미터 파일 (YAML/JSON)")
    p.add_argument("--max_frames", type=int, default=None, help="메모리 적재 프레임 수 제한")
    p.add_argument("--out", default="ransac_params.yaml", help="'s' 저장 경로")
    p.add_argument("--scale", type=float, default=1.0, help="패널 표시 배율")
    a = p.parse_args()

    print(f"[core] {core.module_path()}")
    frames, _depth_scale = playback.load_all_depth_frames(a.bag, a.max_frames)
    if not frames:
        raise SystemExit("프레임을 읽지 못했습니다. .db3 경로/SDK 버전을 확인하세요.")

    sets = load_params(a.params) if a.params else [ParamSet(), ParamSet()]
    sets[1] = replace(sets[1], source=0) if not a.params else sets[1]  # 기본 B는 raw 비교
    edit = 0
    cache = FilterCache(frames)
    pc = rs.pointcloud()
    fixed_plane = None

    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    build_controls(sets[edit])
    cv2.createTrackbar("frame", WIN, 0, max(len(frames) - 1, 1), lambda _v: None)

    idx, playing, dirty = 0, False, True
    last_idx = -1
    computed = [None, None]

    print(KEYS_HELP)

    while True:
        slider = cv2.getTrackbarPos("frame", WIN)
        if slider != idx and slider != last_idx:
            idx = slider
            dirty = True

        if pull_controls(sets[edit]):
            dirty = True

        if dirty or idx != last_idx:
            if sets[edit].seed_random:
                sets[edit].seed = int(np.random.default_rng().integers(0, 100))
                cv2.setTrackbarPos("seed", CTRL, sets[edit].seed)
            for i in (0, 1):
                computed[i] = compute(sets[i], idx, frames, cache, pc, fixed_plane)
            last_idx = idx
            dirty = False

        h_px, w_px = computed[0].xyz.shape[:2]
        size = (int(w_px * a.scale), int(h_px * a.scale))
        panels = [panel(sets[i], computed[i], idx, size, "AB"[i], i == edit) for i in (0, 1)]
        view = np.hstack(panels)
        render.label(view, f"#{idx}/{len(frames) - 1}  {'PLAY' if playing else 'PAUSE'}  "
                           f"edit={'AB'[edit]}  fixed={'set' if fixed_plane else 'none'}  "
                           f"[space . , Tab c v 3 f k r s q]", view.shape[0] - 10)
        cv2.imshow(WIN, view)

        key = cv2.waitKey(30 if playing else 20) & 0xFF

        if key == ord("q") or cv2.getWindowProperty(WIN, cv2.WND_PROP_VISIBLE) < 1:
            break
        elif key == ord(" "):
            playing = not playing
        elif key == ord("."):
            idx = min(idx + 1, len(frames) - 1); cv2.setTrackbarPos("frame", WIN, idx)
        elif key == ord(","):
            idx = max(idx - 1, 0); cv2.setTrackbarPos("frame", WIN, idx)
        elif key == 9:  # Tab
            edit = 1 - edit
            push_controls(sets[edit])
            print(f"[edit] 세트 {'AB'[edit]}")
        elif key == ord("c"):
            sets[1 - edit] = copy.deepcopy(sets[edit])
            dirty = True
            print(f"[copy] {'AB'[edit]} -> {'AB'[1 - edit]}")
        elif key == ord("v"):
            sets[edit].view = (sets[edit].view + 1) % len(render.VIEWS)
            cv2.setTrackbarPos("view", CTRL, sets[edit].view)
        elif key in (ord("3"), ord("p")):
            items = [(c.xyz, c.height, c.valid) for c in computed if c.n is not None]
            if items:
                render.show_3d(items, sets[edit].thresh, sets[edit].hmax_mm / 1000.0,
                               names=("A", "B"))
        elif key == ord("f"):
            # fixed 기준면은 A/B가 공유한다 (같은 기준면 대비 비교가 공정하다)
            base = compute(replace(sets[edit], plane_mode=0), idx, frames, cache, pc, None)
            if base.n is None:
                print("[fixed] 현재 프레임에서 바닥 추출 실패 — 기준면 유지")
            else:
                fixed_plane = (base.n, base.d)
                dirty = True
                print(f"[fixed] frame #{idx} 기준면: h {base.d:.3f} m, "
                      f"pitch {base.pitch:.2f}, roll {base.roll:.2f}")
        elif key == ord("k"):
            repeatability(sets[edit], idx, frames, cache, pc, fixed_plane)
        elif key == ord("r"):
            batch_run(sets, frames, cache, pc, fixed_plane)
        elif key == ord("s"):
            save_params(sets, a.out)

        if playing:
            idx = (idx + 1) % len(frames)
            cv2.setTrackbarPos("frame", WIN, idx)

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
