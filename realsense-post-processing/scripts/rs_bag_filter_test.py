#!/usr/bin/env python3
"""
RealSense .bag 재생 + post-processing 필터 비교 도구
- 모든 프레임을 빠짐없이 순서대로 처리 (set_real_time(False)) -> temporal 필터 결과가 재현 가능
- 왼쪽: raw depth / 오른쪽: filtered depth (같은 컬러 범위로 표시)
- 콘솔: fill rate, 필터 처리시간(ms)

사용 예:
  python rs_bag_filter_test.py scene_B.bag --decim 2 --spatial --range 0.5 3.0
  python rs_bag_filter_test.py scene_F.bag --decim 4 --spatial --temporal
키: space = 일시정지/재개, n = (정지 중) 다음 프레임, q = 종료

pip install pyrealsense2 opencv-python numpy
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np
import pyrealsense2 as rs

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import filters as flt  # noqa: E402


def colorize(depth_m, rng, size):
    lo, hi = rng
    valid = depth_m > 0
    norm = np.clip((depth_m - lo) / (hi - lo), 0, 1)
    img = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_JET)
    img[~valid] = 0  # 무효 픽셀은 검정
    return cv2.resize(img, size, interpolation=cv2.INTER_NEAREST)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("bag")
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
    p.add_argument("--range", type=float, nargs=2, default=[0.3, 4.0],
                   help="컬러맵 거리 범위(m). 좁게 잡으면 바닥 노이즈가 잘 보임")
    a = p.parse_args()

    pipe, cfg = rs.pipeline(), rs.config()
    rs.config.enable_device_from_file(cfg, a.bag, repeat_playback=False)
    cfg.enable_stream(rs.stream.depth)
    prof = pipe.start(cfg)
    prof.get_device().as_playback().set_real_time(False)
    scale = prof.get_device().first_depth_sensor().get_depth_scale()

    filters = flt.build_filters(flt.FilterConfig.from_args(a))
    print("filter chain:", " -> ".join(n for n, _ in filters))

    paused, idx, times = False, 0, []
    try:
        while True:
            try:
                frames = pipe.wait_for_frames(2000)
            except RuntimeError:
                break  # bag 끝
            raw = frames.get_depth_frame()

            t0 = time.perf_counter()
            out = flt.apply_filters(filters, raw)
            dt = (time.perf_counter() - t0) * 1000
            times.append(dt)

            raw_m = np.asanyarray(raw.get_data()) * scale
            flt_m = np.asanyarray(out.get_data()) * scale
            size = (raw_m.shape[1], raw_m.shape[0])
            view = np.hstack([colorize(raw_m, a.range, size),
                              colorize(flt_m, a.range, size)])
            info = (f"#{idx} fill raw {np.mean(raw_m > 0) * 100:.1f}% | "
                    f"filtered {np.mean(flt_m > 0) * 100:.1f}% "
                    f"({flt_m.shape[1]}x{flt_m.shape[0]}) | {dt:.1f} ms")
            cv2.putText(view, info, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (255, 255, 255), 2)
            cv2.imshow("raw | filtered", view)

            key = cv2.waitKey(0 if paused else 1) & 0xFF
            while paused and key not in (ord(" "), ord("n"), ord("q")):
                key = cv2.waitKey(0) & 0xFF
            if key == ord("q"):
                break
            if key == ord(" "):
                paused = not paused
            idx += 1
    finally:
        pipe.stop()
        cv2.destroyAllWindows()

    if times:
        t = np.array(times)
        print(f"frames: {len(t)} | filter time mean {t.mean():.2f} ms, "
              f"p95 {np.percentile(t, 95):.2f} ms, max {t.max():.2f} ms")


if __name__ == "__main__":
    main()