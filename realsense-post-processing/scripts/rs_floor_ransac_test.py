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

좌표계: RealSense 카메라 좌표 (x 오른쪽, y 아래, z 전방), 단위 m
평면 n·p + d = 0, n은 카메라 쪽(위)을 향하도록 정규화 -> d = 카메라 높이, n·p + d = 바닥으로부터 높이

예:
  python rs_floor_ransac_test.py A_floor.db3 --decim 2 --spatial
  python rs_floor_ransac_test.py B_lowobs.db3 --decim 4 --spatial --plane fixed --obs 0.02

pip install pyrealsense2 opencv-python numpy   (3D 보기: pip install open3d)
"""
import argparse
import time

import cv2
import numpy as np
import pyrealsense2 as rs

try:
    import open3d as o3d
except ImportError:
    o3d = None

BANDS = [(0.3, 0.5), (0.5, 1.0), (1.0, 1.5), (1.5, 2.0), (2.0, 3.0), (3.0, 4.0)]
WALL_CUT = 0.10  # 통계에서 벽/큰 물체 제외: 바닥 기준 |h| < 10cm 인 점만 사용
VIEWS = ["inlier", "residual", "depth"]


# ---------------------------------------------------------------- filters
def build_filters(a):
    f = []
    if a.decim > 1:
        d = rs.decimation_filter()
        d.set_option(rs.option.filter_magnitude, a.decim)
        f.append(("decimation", d))
    f.append(("threshold", rs.threshold_filter(a.min, a.max)))
    if a.spatial or a.temporal:
        f.append(("to_disparity", rs.disparity_transform(True)))
    if a.spatial:
        s = rs.spatial_filter()
        s.set_option(rs.option.filter_magnitude, a.sp_mag)
        s.set_option(rs.option.filter_smooth_alpha, a.sp_alpha)
        s.set_option(rs.option.filter_smooth_delta, a.sp_delta)
        s.set_option(rs.option.holes_fill, 0)
        f.append(("spatial", s))
    if a.temporal:
        t = rs.temporal_filter()
        t.set_option(rs.option.filter_smooth_alpha, a.tp_alpha)
        t.set_option(rs.option.filter_smooth_delta, a.tp_delta)
        t.set_option(rs.option.holes_fill, a.tp_persist)
        f.append(("temporal", t))
    if a.spatial or a.temporal:
        f.append(("to_depth", rs.disparity_transform(False)))
    if a.holefill:
        f.append(("hole_filling", rs.hole_filling_filter()))
    return f


# ---------------------------------------------------------------- geometry
def to_points(pc, depth_frame):
    """depth frame -> (H, W, 3) 카메라 좌표 점군 [m]. 필터 후 intrinsics는 SDK가 반영."""
    pts = pc.calculate(depth_frame)
    v = np.asanyarray(pts.get_vertices()).view(np.float32).reshape(-1, 3).copy()
    return v.reshape(depth_frame.get_height(), depth_frame.get_width(), 3)


def fit_plane_ransac(P, thresh, iters, max_tilt_deg, rng, eval_n=5000):
    """P: (N,3) 후보점. 법선이 카메라 y축과 max_tilt 이내인 평면만 허용(벽 배제).
    반환: (n, d, n_inliers) 또는 None"""
    if len(P) < 200:
        return None
    E = P[rng.choice(len(P), min(eval_n, len(P)), replace=False)]
    cos_tilt = np.cos(np.radians(max_tilt_deg))
    idx = rng.integers(0, len(P), size=(iters, 3))
    best, best_cnt = None, -1
    for i in range(iters):
        p0, p1, p2 = P[idx[i]]
        n = np.cross(p1 - p0, p2 - p0)
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n /= nn
        if abs(n[1]) < cos_tilt:
            continue
        d = -n @ p0
        cnt = np.count_nonzero(np.abs(E @ n + d) < thresh)
        if cnt > best_cnt:
            best_cnt, best = cnt, (n, d)
    if best is None:
        return None
    n, d = best
    inl = P[np.abs(P @ n + d) < thresh]
    if len(inl) < 3:
        return None
    c = inl.mean(0)                                  # 최소제곱(SVD) 재추정
    _, _, vt = np.linalg.svd(inl - c, full_matrices=False)
    n = vt[2]
    if n[1] > 0:                                     # 법선 방향을 '위'(카메라 -y)로 통일
        n = -n                                       # (d 부호로 뒤집으면 평면이 카메라 근처일 때 뒤집힘이 튐)
    if abs(n[1]) < cos_tilt:                         # SVD 재추정 후 기울기 조건 재확인
        return None
    d = -n @ c                                       # 바닥이 카메라 아래면 d > 0 = 카메라 높이
    return n, float(d), len(inl)


def plane_angles(n):
    pitch_down = np.degrees(np.arcsin(np.clip(-n[2], -1, 1)))
    roll = np.degrees(np.arctan2(n[0], -n[1]))
    return pitch_down, roll


def cand_mask(xyz, a, roi_top):
    z = xyz[..., 2]
    m = (z > a.min) & (z < a.max)
    m[:roi_top] = False
    return m


def candidates(xyz, a, roi_top):
    return xyz[cand_mask(xyz, a, roi_top)]


def analyze(xyz, plane, a):
    """바닥 기준 높이맵 + 통계"""
    valid = xyz[..., 2] > 0
    n, d = plane[0], plane[1]
    H = np.where(valid, xyz @ n + d, 0.0)
    z = xyz[..., 2]
    stats = {}
    for lo, hi in BANDS:
        m = valid & (z >= lo) & (z < hi) & (np.abs(H) < WALL_CUT)
        cnt = np.count_nonzero(m)
        if cnt < 50:
            stats[(lo, hi)] = (np.nan, np.nan)
            continue
        h = H[m]
        stats[(lo, hi)] = (float(np.std(h)), float(np.mean(h > a.obs)))
    return H, valid, stats


# ---------------------------------------------------------------- drawing
def draw(xyz, H, valid, view, a, size, roi_top, scale_depth):
    if view == "depth":
        z = xyz[..., 2]
        norm = np.clip((z - a.range[0]) / (a.range[1] - a.range[0]), 0, 1)
        img = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_JET)
    elif view == "residual":
        norm = np.clip((H + a.hres) / (2 * a.hres), 0, 1)
        img = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_JET)
    else:  # inlier
        img = np.zeros(H.shape + (3,), np.uint8)
        t = a.ransac_thresh
        above = valid & (H >= t)
        norm = np.clip((H - t) / (a.hmax - t), 0, 1)
        cm = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
        img[valid & (np.abs(H) < t)] = (90, 90, 90)
        img[above] = cm[above]
        img[valid & (H <= -t)] = (255, 0, 255)
    img[~valid] = 0
    cv2.line(img, (0, roi_top), (img.shape[1] - 1, roi_top), (255, 255, 255), 1)
    return cv2.resize(img, size, interpolation=cv2.INTER_NEAREST)


def label(img, text, y):
    cv2.putText(img, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
    cv2.putText(img, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)


def show_3d(items, a):
    if o3d is None:
        print("[3D] open3d 미설치: pip install open3d")
        return
    geoms, offset = [], 0.0
    for xyz, H, valid in items:
        P, h = xyz[valid], H[valid]
        col = np.full((len(P), 3), 0.6)
        col[np.abs(h) < a.ransac_thresh] = (0.1, 0.8, 0.1)       # 바닥 inlier
        up = h >= a.ransac_thresh
        col[up] = np.c_[np.clip(h[up] / a.hmax, 0.3, 1), np.zeros(up.sum()), np.zeros(up.sum())]
        col[h <= -a.ransac_thresh] = (0.9, 0.1, 0.9)
        pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(P + [offset, 0, 0]))
        pc.colors = o3d.utility.Vector3dVector(col)
        geoms.append(pc)
        offset += 3.0                                            # filtered는 x+3m 옆에
    geoms.append(o3d.geometry.TriangleMesh.create_coordinate_frame(0.3))
    print("[3D] 왼쪽 raw / 오른쪽 filtered (x+3m). 초록=바닥 inlier, 빨강=장애물, 마젠타=바닥 아래")
    o3d.visualization.draw_geometries(geoms, window_name="raw | filtered (side view로 돌려보세요)")


def fmt(vals, k, spec):
    v = np.asarray(vals, float)
    v = v[~np.isnan(v)]
    return format(v.mean() * k, spec) if len(v) else f"{'-':>{spec.split('.')[0]}}"


# ---------------------------------------------------------------- main
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
    p.add_argument("--seed", type=int, default=0)
    # display / metrics
    p.add_argument("--obs", type=float, default=0.02, help="허상 장애물 판정 높이 [m]")
    p.add_argument("--hmax", type=float, default=0.10, help="inlier 뷰 컬러 상한 [m]")
    p.add_argument("--hres", type=float, default=0.03, help="residual 뷰 ±범위 [m]")
    p.add_argument("--range", type=float, nargs=2, default=[0.3, 4.0])
    p.add_argument("--view", choices=VIEWS, default="inlier")
    a = p.parse_args()

    pipe, cfg = rs.pipeline(), rs.config()
    rs.config.enable_device_from_file(cfg, a.bag, repeat_playback=False)
    cfg.enable_stream(rs.stream.depth)
    prof = pipe.start(cfg)
    prof.get_device().as_playback().set_real_time(False)
    scale = prof.get_device().first_depth_sensor().get_depth_scale()

    filters = build_filters(a)
    pc_raw, pc_flt = rs.pointcloud(), rs.pointcloud()
    rng = np.random.default_rng(a.seed)
    print("filter chain:", " -> ".join(n for n, _ in filters))
    print(f"plane mode: {a.plane} | thresh {a.ransac_thresh * 100:.1f} cm | iters {a.iters}")

    view_i = VIEWS.index(a.view)
    fixed_plane = None
    log = {k: {"std": {b: [] for b in BANDS}, "fp": {b: [] for b in BANDS},
               "h": [], "pitch": [], "roll": [], "inl": [], "rt": []} for k in ("raw", "flt")}
    ftimes, paused, idx = [], False, 0

    try:
        while True:
            try:
                frames = pipe.wait_for_frames(2000)
            except RuntimeError:
                break
            raw = frames.get_depth_frame()
            t0 = time.perf_counter()
            out = raw
            for _, f in filters:
                out = f.process(out)
            out = out.as_depth_frame()  # process()는 일반 rs.frame을 반환 -> depth_frame으로 캐스팅
            ftimes.append((time.perf_counter() - t0) * 1000)

            panels, items = [], []
            for key, frame, pc in (("raw", raw, pc_raw), ("flt", out, pc_flt)):
                xyz = to_points(pc, frame)
                roi_top = int(xyz.shape[0] * (1 - a.roi_bottom))
                t1 = time.perf_counter()
                if a.plane == "fixed":
                    if fixed_plane is None:
                        fixed_plane = fit_plane_ransac(candidates(xyz, a, roi_top), a.ransac_thresh,
                                                       a.iters * 5, a.max_tilt, rng)
                        if fixed_plane is None:
                            raise SystemExit("첫 프레임에서 바닥 추출 실패: --roi_bottom/--max_tilt 확인")
                    plane = fixed_plane
                else:
                    plane = fit_plane_ransac(candidates(xyz, a, roi_top), a.ransac_thresh,
                                             a.iters, a.max_tilt, rng)
                rt = (time.perf_counter() - t1) * 1000

                size = (raw.get_width(), raw.get_height())
                if plane is None:
                    img = np.zeros((size[1], size[0], 3), np.uint8)
                    label(img, f"{key}: floor NOT found", 20)
                    panels.append(img)
                    continue

                H, valid, stats = analyze(xyz, plane, a)
                n, d, n_inl = plane
                pitch, roll = plane_angles(n)
                cm = cand_mask(xyz, a, roi_top)
                cand = max(np.count_nonzero(cm), 1)
                L = log[key]
                L["h"].append(d); L["pitch"].append(pitch); L["roll"].append(roll)
                L["inl"].append(np.count_nonzero(cm & (np.abs(H) < a.ransac_thresh)) / cand)
                L["rt"].append(rt)
                for b, (s, fp) in stats.items():
                    L["std"][b].append(s); L["fp"][b].append(fp)

                img = draw(xyz, H, valid, VIEWS[view_i], a, size, roi_top, scale)
                label(img, f"{key} [{VIEWS[view_i]}] {xyz.shape[1]}x{xyz.shape[0]}", 20)
                label(img, f"cam h {d:.3f}m  pitch {pitch:.1f}  roll {roll:.1f}  inlier {L['inl'][-1] * 100:.0f}%", 40)
                s15 = stats[(1.0, 1.5)]
                label(img, f"z1.0-1.5m: floor std {s15[0] * 1000:.1f}mm  >{a.obs * 100:.0f}cm {s15[1] * 100:.1f}%", 60)
                panels.append(img)
                items.append((xyz, H, valid))

            view = np.hstack(panels)
            label(view, f"#{idx}  filter {ftimes[-1]:.1f} ms", view.shape[0] - 10)
            cv2.imshow("RANSAC floor: raw | filtered", view)

            key = cv2.waitKey(0 if paused else 1) & 0xFF
            while True:
                if key == ord("v"):
                    view_i = (view_i + 1) % len(VIEWS)
                    print("view:", VIEWS[view_i], "(다음 프레임부터 적용)")
                elif key == ord("p") and len(items) == 2:
                    show_3d(items, a)
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

    # ------------------------------------------------------------ summary
    if not ftimes:
        return
    print(f"\nframes {len(ftimes)} | filter time mean {np.mean(ftimes):.2f} ms, p95 {np.percentile(ftimes, 95):.2f} ms")
    print(f"\n[거리 구간별 바닥 높이 std (mm) / 높이>{a.obs * 100:.0f}cm 점 비율 (%)]  * 빈 바닥 장면에서 허상 장애물률")
    print(f"{'z band (m)':>12} | {'raw std':>8} {'flt std':>8} | {'raw >obs':>9} {'flt >obs':>9}")
    for b in BANDS:
        r, f = log["raw"], log["flt"]
        print(f"{b[0]:>5.1f}-{b[1]:<5.1f} | {fmt(r['std'][b], 1000, '8.1f')} {fmt(f['std'][b], 1000, '8.1f')} | "
              f"{fmt(r['fp'][b], 100, '8.2f')}% {fmt(f['fp'][b], 100, '8.2f')}%")
    print(f"  * std가 ~58mm(= ±{WALL_CUT * 100:.0f}cm 균일분포의 std)에 가까우면 해당 구간에 실제 바닥이 없다는 뜻")
    print("\n[평면 추정 안정성]  (per_frame 모드에서 의미: 프레임 간 흔들림 = 노이즈가 바닥 추정에 주는 영향)")
    for k in ("raw", "flt"):
        L = log[k]
        if not L["h"]:
            print(f"{k}: floor not found")
            continue
        print(f"{k}: cam h {np.mean(L['h']):.3f} ± {np.std(L['h']) * 1000:.1f} mm | "
              f"pitch {np.mean(L['pitch']):.2f} ± {np.std(L['pitch']):.2f} deg | "
              f"roll {np.mean(L['roll']):.2f} ± {np.std(L['roll']):.2f} deg | "
              f"inlier {np.mean(L['inl']) * 100:.1f}% | RANSAC {np.mean(L['rt']):.1f} ms")


if __name__ == "__main__":
    main()