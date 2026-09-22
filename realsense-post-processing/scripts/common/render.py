"""OpenCV 화면 구성 + open3d 3D 보기 (optional import).

뷰:
  inlier   : 바닥 inlier = 회색, 바닥 위(장애물) = 높이별 컬러, 바닥 아래 = 마젠타
  residual : 바닥 기준 높이를 ±hres 범위 컬러로 (바닥 노이즈 패턴 확인용)
  depth    : 일반 depth 컬러맵
흰 가로선 = RANSAC 후보 ROI 상단 (이 선 아래 픽셀만 바닥 후보)
"""
import cv2
import numpy as np

try:
    import open3d as o3d
except ImportError:
    o3d = None

VIEWS = ["inlier", "residual", "depth"]


def draw(xyz, height, valid, view, size, roi_top, *,
         thresh, hmax, hres, z_range):
    if view == "depth":
        z = xyz[..., 2]
        norm = np.clip((z - z_range[0]) / (z_range[1] - z_range[0]), 0, 1)
        img = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_JET)
    elif view == "residual":
        norm = np.clip((height + hres) / (2 * hres), 0, 1)
        img = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_JET)
    else:  # inlier
        img = np.zeros(height.shape + (3,), np.uint8)
        above = valid & (height >= thresh)
        norm = np.clip((height - thresh) / max(hmax - thresh, 1e-6), 0, 1)
        cm = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
        img[valid & (np.abs(height) < thresh)] = (90, 90, 90)
        img[above] = cm[above]
        img[valid & (height <= -thresh)] = (255, 0, 255)
    img[~valid] = 0
    if 0 <= roi_top < img.shape[0]:
        cv2.line(img, (0, roi_top), (img.shape[1] - 1, roi_top), (255, 255, 255), 1)
    return cv2.resize(img, size, interpolation=cv2.INTER_NEAREST)


def label(img, text, y, x=8, scale=0.5):
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3)
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), 1)


def blank(size, text):
    img = np.zeros((size[1], size[0], 3), np.uint8)
    label(img, text, 20)
    return img


def show_3d(items, thresh, hmax, names=("raw", "filtered")):
    """items: [(xyz, height, valid), ...] — 두 번째부터 x+3m 옆에 놓는다."""
    if o3d is None:
        print("[3D] open3d 미설치: pip install open3d")
        return
    geoms, offset = [], 0.0
    for xyz, height, valid in items:
        P, h = xyz[valid], height[valid]
        col = np.full((len(P), 3), 0.6)
        col[np.abs(h) < thresh] = (0.1, 0.8, 0.1)               # 바닥 inlier
        up = h >= thresh
        col[up] = np.c_[np.clip(h[up] / hmax, 0.3, 1),
                        np.zeros(up.sum()), np.zeros(up.sum())]  # 장애물
        col[h <= -thresh] = (0.9, 0.1, 0.9)                     # 바닥 아래
        pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(P + [offset, 0, 0]))
        pc.colors = o3d.utility.Vector3dVector(col)
        geoms.append(pc)
        offset += 3.0
    geoms.append(o3d.geometry.TriangleMesh.create_coordinate_frame(0.3))
    print(f"[3D] 왼쪽 {names[0]} / 오른쪽 {names[1]} (x+3m). "
          f"초록=바닥 inlier, 빨강=장애물, 마젠타=바닥 아래")
    o3d.visualization.draw_geometries(
        geoms, window_name=f"{names[0]} | {names[1]} (side view로 돌려보세요)")
