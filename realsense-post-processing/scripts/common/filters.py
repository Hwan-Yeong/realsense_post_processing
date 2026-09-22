"""librealsense 후처리 필터 체인.

순서는 librealsense 권장을 따른다 (CLAUDE.md):
  decimation -> threshold -> depth->disparity -> spatial -> temporal -> disparity->depth -> hole filling

필터 자체는 SDK 구현이라 재구현 대상이 아니다. Python이 계속 맡는 부분.
"""
from dataclasses import dataclass, asdict, fields

import pyrealsense2 as rs

FILTER_FIELDS = (
    "decim", "zmin", "zmax",
    "spatial", "sp_mag", "sp_alpha", "sp_delta",
    "temporal", "tp_alpha", "tp_delta", "tp_persist",
    "holefill",
)


@dataclass
class FilterConfig:
    decim: int = 2
    zmin: float = 0.2
    zmax: float = 4.0
    spatial: bool = False
    sp_mag: float = 2.0
    sp_alpha: float = 0.5
    sp_delta: float = 20.0
    temporal: bool = False
    tp_alpha: float = 0.4
    tp_delta: float = 20.0
    tp_persist: int = 0
    holefill: bool = False

    def to_dict(self):
        return asdict(self)

    @classmethod
    def from_dict(cls, d):
        known = {f.name for f in fields(cls)}
        return cls(**{k: v for k, v in d.items() if k in known})

    @classmethod
    def from_args(cls, a):
        """argparse Namespace -> FilterConfig (기존 스크립트 CLI 호환)."""
        return cls(
            decim=a.decim, zmin=a.min, zmax=a.max,
            spatial=a.spatial, sp_mag=a.sp_mag, sp_alpha=a.sp_alpha, sp_delta=a.sp_delta,
            temporal=a.temporal, tp_alpha=a.tp_alpha, tp_delta=a.tp_delta,
            tp_persist=a.tp_persist, holefill=a.holefill,
        )

    @property
    def order_sensitive(self):
        """temporal은 이전 프레임 상태를 들고 있어 순서 의존적이다.
        켜져 있으면 임의 프레임으로 점프할 때 0번부터 순차 재계산해야 한다."""
        return self.temporal


def build_filters(cfg):
    """(이름, 필터) 리스트. 매번 새로 만든다 — temporal이 내부 상태를 갖기 때문."""
    f = []
    if cfg.decim > 1:
        d = rs.decimation_filter()
        d.set_option(rs.option.filter_magnitude, float(cfg.decim))
        f.append(("decimation", d))
    f.append(("threshold", rs.threshold_filter(cfg.zmin, cfg.zmax)))
    if cfg.spatial or cfg.temporal:
        f.append(("to_disparity", rs.disparity_transform(True)))
    if cfg.spatial:
        s = rs.spatial_filter()
        s.set_option(rs.option.filter_magnitude, float(cfg.sp_mag))
        s.set_option(rs.option.filter_smooth_alpha, float(cfg.sp_alpha))
        s.set_option(rs.option.filter_smooth_delta, float(cfg.sp_delta))
        s.set_option(rs.option.holes_fill, 0)
        f.append(("spatial", s))
    if cfg.temporal:
        t = rs.temporal_filter()
        t.set_option(rs.option.filter_smooth_alpha, float(cfg.tp_alpha))
        t.set_option(rs.option.filter_smooth_delta, float(cfg.tp_delta))
        t.set_option(rs.option.holes_fill, int(cfg.tp_persist))
        f.append(("temporal", t))
    if cfg.spatial or cfg.temporal:
        f.append(("to_depth", rs.disparity_transform(False)))
    if cfg.holefill:
        f.append(("hole_filling", rs.hole_filling_filter()))
    return f


def apply_filters(filters, frame):
    """process()는 일반 rs.frame을 반환하므로 depth_frame으로 캐스팅해야
    get_width() 등을 쓸 수 있다 (CLAUDE.md '알려진 함정')."""
    out = frame
    for _, f in filters:
        out = f.process(out)
    return out.as_depth_frame()


def chain_str(filters):
    return " -> ".join(n for n, _ in filters)
