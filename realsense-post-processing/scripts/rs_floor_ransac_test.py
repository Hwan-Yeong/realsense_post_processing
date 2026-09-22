#!/usr/bin/env python3
"""
RealSense 녹화(.db3) 재생 + RANSAC 바닥 추출 비교 (raw vs filtered)

화면 (왼쪽 raw | 오른쪽 filtered), 'v' 키로 뷰 전환
  inlier   : RANSAC 바닥 inlier = 회색, 바닥 위(장애물) = 높이별 컬러, 바닥 아래 = 마젠타
  residual : 바닥 평면 기준 높이를 ±hres 범위 컬러로 (바닥 노이즈 패턴 확인용)
  depth    : 일반 depth 컬러맵
  흰 가로선 = RANSAC 후보 ROI 상단 (이 선 아래 픽셀만 바닥 후보로 사용)

키: space 일시정지 | n 다음 프레임(정지 중) | v 뷰 전환 | p 현재 프레임 3D 보기(open3d) | q 종료

plane 모드
  per_frame : raw/filtered 각각 매 프레임 RANSAC -> 필터가 '바닥 추출 자체'에 주는 영향 비교
  fixed     : 첫 raw 프레임에서 한 번 추출한 평면을 raw/filtered 모두의 기준으로 사용
              -> 같은 기준면 대비 노이즈/허상 장애물 비교 (필터 비교에 더 공정)

RANSAC은 C++ 코어(floor_ransac_py)를 호출한다. Python에 따로 구현하지 않는다 —
여기서 본 결과가 ROS 노드/임베디드에서 그대로 재현되어야 하기 때문.
파라미터를 실시간으로 바꿔가며 보려면 rs_ransac_tuner.py를 쓸 것.

좌표계: RealSense 카메라 좌표 (x 오른쪽, y 아래, z 전방), 단위 m
평면 n·p + d = 0, n은 카메라 쪽(위)을 향하도록 고정 -> d = 카메라 높이, n·p + d = 바닥으로부터 높이

예:
  python3 rs_floor_ransac_test.py A_floor.db3 --decim 2 --spatial
  python3 rs_floor_ransac_test.py B_lowobs.db3 --decim 4 --spatial --plane fixed --obs 0.02

pip install pyrealsense2 opencv-python numpy   (3D 보기: pip install open3d)
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np
import pyrealsense2 as rs

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import core, filters as flt, metrics, playback, render  # noqa: E402


def main():
    p = argparse.ArgumentParser()
    p.add_argument("bag")
    # filters
    p.add_argument("--decim", type=int, default=2)
    p.add_argument("--min", type=float, default=0.2)
    p.add_argument("--max", type=float, default=4.0)
    p.add_argument("--spatial", action="store_true")
    p.add_argument("--sp_mag", type=float, default=2)
    p.add_argument("--sp_alpha", type=float, default=0.5)
    p.add_argument("--sp_delta", type=float, default=20)
    p.add_argument("--temporal", action="store_true")
    p.add_argument("--tp_alpha", type=float, default=0.4)
    p.add_argument("--tp_delta", type=float, default=20)
    p.add_argument("--tp_persist", type=int, default=0)
    p.add_argument("--holefill", action="store_true")
    # RANSAC
    p.add_argument("--plane", choices=["per_frame", "fixed"], default="per_frame")
    p.add_argument("--ransac_thresh", type=float, default=0.02, help="inlier 거리 [m]")
    p.add_argument("--iters", type=int, default=200)
    p.add_argument("--max_tilt", type=float, default=45, help="바닥 법선 허용 기울기 [deg]")
    p.add_argument("--roi_bottom", type=float, default=0.5, help="후보로 쓸 화면 하단 비율")
    p.add_argument("--eval_n", type=int, default=5000, help="가설 평가용 서브샘플 크기")
    p.add_argument("--seed", type=int, default=0)
    # display / metrics
    p.add_argument("--obs", type=float, default=0.02, help="허상 장애물 판정 높이 [m]")
    p.add_argument("--hmax", type=float, default=0.10, help="inlier 뷰 컬러 상한 [m]")
    p.add_argument("--hres", type=float, default=0.03, help="residual 뷰 ±범위 [m]")
    p.add_argument("--range", type=float, nargs=2, default=[0.3, 4.0])
    p.add_argument("--view", choices=render.VIEWS, default="inlier")
    a = p.parse_args()

    cfg = flt.FilterConfig.from_args(a)
    params = core.make_params(
        thresh=a.ransac_thresh, iters=a.iters, max_tilt_deg=a.max_tilt,
        roi_bottom=a.roi_bottom, z_min=a.min, z_max=a.max, eval_n=a.eval_n, seed=a.seed)
    # fixed 기준면은 첫 프레임에서 한 번만 뽑으므로 가설을 넉넉히 준다
    fixed_params = core.make_params(**{**core.params_to_dict(params), "iters": a.iters * 5})

    pipe, _scale = playback.open_playback(a.bag)
    filters = flt.build_filters(cfg)
    pc_raw, pc_flt = rs.pointcloud(), rs.pointcloud()
    print(f"[core] {core.module_path()}")
    print("filter chain:", flt.chain_str(filters))
    print(f"plane mode: {a.plane} | thresh {a.ransac_thresh * 100:.1f} cm | iters {a.iters}")

    view_i = render.VIEWS.index(a.view)
    fixed_plane = None
    log = metrics.new_log(("raw", "flt"))
    ftimes, paused, idx = [], False, 0

    try:
        while True:
            try:
                frames = pipe.wait_for_frames(2000)
            except RuntimeError:
                break
            raw = frames.get_depth_frame()
            if not raw:
                continue
            t0 = time.perf_counter()
            out = flt.apply_filters(filters, raw)
            ftimes.append((time.perf_counter() - t0) * 1000)

            panels, items = [], []
            for key, frame, pc in (("raw", raw, pc_raw), ("flt", out, pc_flt)):
                xyz = playback.to_points(pc, frame)
                h_px, w_px = xyz.shape[:2]
                pts64 = core.as_core_points(xyz)
                top = metrics.roi_top(h_px, a.roi_bottom)

                t1 = time.perf_counter()
                if a.plane == "fixed":
                    if fixed_plane is None:
                        res, _ = core.fit_organized(pts64, h_px, w_px, fixed_params)
                        if not res.ok:
                            raise SystemExit(
                                f"첫 프레임에서 바닥 추출 실패({res.status_string}): "
                                "--roi_bottom/--max_tilt 확인")
                        fixed_plane = core.plane_nd(res)
                    n, d = fixed_plane
                    ok = True
                else:
                    res, _ = core.fit_organized(pts64, h_px, w_px, params)
                    ok = res.ok
                    if ok:
                        n, d = core.plane_nd(res)
                rt = (time.perf_counter() - t1) * 1000

                size = (raw.get_width(), raw.get_height())
                if not ok:
                    panels.append(render.blank(size, f"{key}: floor NOT found "
                                                     f"({res.status_string})"))
                    continue

                height, valid, stats = metrics.analyze(xyz, n, d, a.obs)
                pitch, roll = core.plane_angles(n)
                cm = metrics.candidate_mask(xyz, a.min, a.max, top)
                cand = max(int(np.count_nonzero(cm)), 1)
                ratio = float(np.count_nonzero(cm & (np.abs(height) < a.ransac_thresh)) / cand)
                metrics.record(log[key], d, pitch, roll, ratio, rt, stats)

                img = render.draw(xyz, height, valid, render.VIEWS[view_i], size, top,
                                  thresh=a.ransac_thresh, hmax=a.hmax, hres=a.hres,
                                  z_range=a.range)
                render.label(img, f"{key} [{render.VIEWS[view_i]}] {w_px}x{h_px}", 20)
                render.label(img, f"cam h {d:.3f}m  pitch {pitch:.1f}  roll {roll:.1f}  "
                                  f"inlier {ratio * 100:.0f}%", 40)
                s15 = stats[(1.0, 1.5)]
                render.label(img, f"z1.0-1.5m: floor std {s15[0] * 1000:.1f}mm  "
                                  f">{a.obs * 100:.0f}cm {s15[1] * 100:.1f}%", 60)
                panels.append(img)
                items.append((xyz, height, valid))

            view = np.hstack(panels)
            render.label(view, f"#{idx}  filter {ftimes[-1]:.1f} ms", view.shape[0] - 10)
            cv2.imshow("RANSAC floor: raw | filtered", view)

            key = cv2.waitKey(0 if paused else 1) & 0xFF
            while True:
                if key == ord("v"):
                    view_i = (view_i + 1) % len(render.VIEWS)
                    print("view:", render.VIEWS[view_i], "(다음 프레임부터 적용)")
                elif key == ord("p") and len(items) == 2:
                    render.show_3d(items, a.ransac_thresh, a.hmax)
                if not paused or key in (ord(" "), ord("n"), ord("q")):
                    break
                key = cv2.waitKey(0) & 0xFF
            if key == ord("q"):
                break
            if key == ord(" "):
                paused = not paused
            idx += 1
    finally:
        pipe.stop()
        cv2.destroyAllWindows()

    if not ftimes:
        return
    print(f"\nframes {len(ftimes)} | filter time mean {np.mean(ftimes):.2f} ms, "
          f"p95 {np.percentile(ftimes, 95):.2f} ms")
    metrics.print_band_table(log, a.obs)
    metrics.print_stability(log)


if __name__ == "__main__":
    main()
