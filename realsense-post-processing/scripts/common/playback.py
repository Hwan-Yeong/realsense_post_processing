""".db3 재생 / 전체 프레임 메모리 적재.

bag/*.db3는 SQLite(rosbag2) 컨테이너를 쓰지만 내용은 librealsense 내부 포맷이라
`ros2 bag play`로는 재생되지 않는다. pyrealsense2의 enable_device_from_file 경로만 가능.

재생은 항상 set_real_time(False) — 프레임 누락 없이 결정적으로 읽어야
temporal 필터 비교가 재현된다 (CLAUDE.md 공통 규칙).
"""
import time

import numpy as np
import pyrealsense2 as rs


def open_playback(path):
    """(pipeline, depth_scale) 반환. 호출자가 pipe.stop() 책임."""
    pipe, cfg = rs.pipeline(), rs.config()
    rs.config.enable_device_from_file(cfg, path, repeat_playback=False)
    cfg.enable_stream(rs.stream.depth)
    prof = pipe.start(cfg)
    prof.get_device().as_playback().set_real_time(False)
    scale = prof.get_device().first_depth_sensor().get_depth_scale()
    return pipe, scale


def iter_depth_frames(path, timeout_ms=2000):
    """스트리밍 방식 (메모리에 쌓지 않음). 기존 비교 스크립트용."""
    pipe, scale = open_playback(path)
    try:
        while True:
            try:
                frames = pipe.wait_for_frames(timeout_ms)
            except RuntimeError:
                break
            d = frames.get_depth_frame()
            if d:
                yield d, scale
    finally:
        pipe.stop()


def load_all_depth_frames(path, max_frames=None, timeout_ms=2000, verbose=True):
    """모든 depth 프레임을 메모리에 적재해서 리스트로 반환.

    frame.keep()이 핵심이다. 이게 없으면 SDK가 프레임 버퍼를 재활용해서
    나중에 읽을 때 내용이 깨진다. 848x480 @15fps 기준 프레임당 약 0.8 MB라
    10초 녹화가 200 MB대 — 슬라이더로 임의 프레임을 오가려면 전부 들고 있는 게 맞다.
    """
    pipe, scale = open_playback(path)
    frames = []
    t0 = time.perf_counter()
    try:
        while True:
            try:
                fs = pipe.wait_for_frames(timeout_ms)
            except RuntimeError:
                break
            d = fs.get_depth_frame()
            if not d:
                continue
            d.keep()  # 버퍼 재활용 방지 — 없으면 나중에 읽은 프레임이 깨진다
            frames.append(d)
            if max_frames and len(frames) >= max_frames:
                break
    finally:
        pipe.stop()
    if verbose:
        el = time.perf_counter() - t0
        if frames:
            w, h = frames[0].get_width(), frames[0].get_height()
            mb = len(frames) * w * h * 2 / 1e6
            print(f"[load] {len(frames)} 프레임 ({w}x{h}) {el:.2f}s, depth 약 {mb:.0f} MB")
        else:
            print(f"[load] 프레임 없음 ({el:.2f}s)")
    return frames, scale


def to_points(pc, depth_frame):
    """depth frame -> (H, W, 3) 카메라 좌표 점군 [m].

    필터 후 intrinsics는 SDK가 반영해 준다 (decimation으로 해상도가 바뀌어도 맞음).
    """
    pts = pc.calculate(depth_frame)
    v = np.asanyarray(pts.get_vertices()).view(np.float32).reshape(-1, 3).copy()
    return v.reshape(depth_frame.get_height(), depth_frame.get_width(), 3)
