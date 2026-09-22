"""튜닝 분석용 지표.

의도적 경계: 이 지표들은 C++ 코어로 내리지 않는다. 평면 적합만 공유하고
구간별 std / 허상 장애물률은 튜닝 분석 전용이라 Python에 둔다.

정의 (CLAUDE.md):
- 거리 구간 z: 0.3-0.5 / 0.5-1.0 / 1.0-1.5 / 1.5-2.0 / 2.0-3.0 / 3.0-4.0 m
- 통계 대상은 바닥 기준 |h| < WALL_CUT 점만 (벽/큰 물체 제외)
- 바닥 std: 그 점들의 높이 std. ~58 mm(=±10cm 균일분포의 std)에 가까우면
  그 구간에 실제 바닥이 없다는 뜻
- 허상 장애물률: h > obs 인 점 비율 (빈 바닥 장면에서만 의미)
"""
import numpy as np

BANDS = [(0.3, 0.5), (0.5, 1.0), (1.0, 1.5), (1.5, 2.0), (2.0, 3.0), (3.0, 4.0)]
WALL_CUT = 0.10
MIN_BAND_POINTS = 50
# ±WALL_CUT 균일분포의 std. 이 값에 가까우면 해당 구간에 바닥이 없다는 신호
UNIFORM_STD = 2 * WALL_CUT / np.sqrt(12)


def roi_top(height_px, roi_bottom):
    """후보 ROI 상단 행. 이 아래 픽셀만 바닥 후보로 쓴다 (C++ select_candidates와 같은 식)."""
    return int(height_px * (1.0 - min(max(roi_bottom, 0.0), 1.0)))


def candidate_mask(xyz, z_min, z_max, top):
    """화면 하단 ROI + z 범위 후보 마스크 (inlier 비율 표시용).

    적합 자체는 C++ select_candidates가 하고, 여기는 같은 조건을 (H,W) 마스크로
    다시 만들어 화면에 비율을 띄우기 위한 것이다.
    """
    z = xyz[..., 2]
    m = (z > z_min) & (z < z_max)
    m[:top] = False
    return m


def analyze(xyz, n, d, obs):
    """(H,W,3) 점군 + 평면 -> (높이맵 H, 유효마스크, 구간별 (std, 허상률))."""
    valid = xyz[..., 2] > 0
    n = np.asarray(n, dtype=np.float64).reshape(3)
    height = np.where(valid, xyz @ n + float(d), 0.0)
    z = xyz[..., 2]
    stats = {}
    for lo, hi in BANDS:
        m = valid & (z >= lo) & (z < hi) & (np.abs(height) < WALL_CUT)
        if np.count_nonzero(m) < MIN_BAND_POINTS:
            stats[(lo, hi)] = (np.nan, np.nan)
            continue
        h = height[m]
        stats[(lo, hi)] = (float(np.std(h)), float(np.mean(h > obs)))
    return height, valid, stats


def new_log(keys=("raw", "flt")):
    return {
        k: {
            "std": {b: [] for b in BANDS},
            "fp": {b: [] for b in BANDS},
            "h": [], "pitch": [], "roll": [], "inl": [], "rt": [],
        }
        for k in keys
    }


def record(log_entry, d, pitch, roll, inlier_ratio, ransac_ms, stats):
    log_entry["h"].append(d)
    log_entry["pitch"].append(pitch)
    log_entry["roll"].append(roll)
    log_entry["inl"].append(inlier_ratio)
    log_entry["rt"].append(ransac_ms)
    for b, (s, fp) in stats.items():
        log_entry["std"][b].append(s)
        log_entry["fp"][b].append(fp)


def _fmt(vals, k, spec):
    v = np.asarray(vals, float)
    v = v[~np.isnan(v)]
    return format(v.mean() * k, spec) if len(v) else f"{'-':>{spec.split('.')[0]}}"


def print_band_table(log, obs, keys=("raw", "flt"), labels=("raw", "flt")):
    print(f"\n[거리 구간별 바닥 높이 std (mm) / 높이>{obs * 100:.0f}cm 점 비율 (%)]"
          f"  * 빈 바닥 장면에서 허상 장애물률")
    print(f"{'z band (m)':>12} | {labels[0] + ' std':>8} {labels[1] + ' std':>8} | "
          f"{labels[0] + ' >obs':>9} {labels[1] + ' >obs':>9}")
    a, b = log[keys[0]], log[keys[1]]
    for band in BANDS:
        print(f"{band[0]:>5.1f}-{band[1]:<5.1f} | "
              f"{_fmt(a['std'][band], 1000, '8.1f')} {_fmt(b['std'][band], 1000, '8.1f')} | "
              f"{_fmt(a['fp'][band], 100, '8.2f')}% {_fmt(b['fp'][band], 100, '8.2f')}%")
    print(f"  * std가 ~{UNIFORM_STD * 1000:.0f}mm(= ±{WALL_CUT * 100:.0f}cm 균일분포의 std)에 "
          f"가까우면 해당 구간에 실제 바닥이 없다는 뜻")


def print_stability(log, keys=("raw", "flt")):
    print("\n[평면 추정 안정성]  (per_frame 모드에서 의미: 프레임 간 흔들림 = 노이즈가 바닥 추정에 주는 영향)")
    for k in keys:
        entry = log[k]
        if not entry["h"]:
            print(f"{k}: floor not found")
            continue
        print(f"{k}: cam h {np.mean(entry['h']):.3f} ± {np.std(entry['h']) * 1000:.1f} mm | "
              f"pitch {np.mean(entry['pitch']):.2f} ± {np.std(entry['pitch']):.2f} deg | "
              f"roll {np.mean(entry['roll']):.2f} ± {np.std(entry['roll']):.2f} deg | "
              f"inlier {np.mean(entry['inl']) * 100:.1f}% | RANSAC {np.mean(entry['rt']):.1f} ms")
