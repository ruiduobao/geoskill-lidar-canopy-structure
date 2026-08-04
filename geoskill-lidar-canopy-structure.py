#!/usr/bin/env python3
"""lidar-canopy-structure — LiDAR 冠层结构分析

从 LiDAR 点云提取冠层结构：先用最低高程面 + 渐进形态学开运算（PMF，
Zhang et al. 2003 简化版）估计裸地 DTM，用最高高程面作为 DSM，二者相减
得到冠层高度模型 CHM（Canopy Height Model）；再在 CHM 上阈值化 + 连通域
标记（简化分水岭）、高斯平滑后用 maximum_filter 确认局部峰值，实现单木
检测，逐木提取树高（峰值 CHM）与冠幅（连通域等效半径）。

数据源：本地点云（.npy / .csv / .txt xyz），或 ``--synthetic`` 生成
平面地形 + 若干高斯冠形树木的模拟点云（离线，局部米制坐标）。

隐私声明 / Privacy：
- 完全离线运行，不访问任何网络服务。
- 所有处理在本地完成，不上传任何用户数据。

Usage:
    python lidar-canopy-structure.py --input cloud.npy --min-height 2
    python lidar-canopy-structure.py --bbox 116 39 117 40 --synthetic --output-dir ./out

License: MIT
"""
from __future__ import annotations

import argparse
import datetime as _dt
import json
import os
import sys
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

VERSION = "1.0.0"
SKILL_NAME = "lidar-canopy-structure"

# ---------------------------------------------------------------------------
# Input validation
# ---------------------------------------------------------------------------
def validate_bbox(bbox) -> None:
    """Validate geographic bbox: W<E, S<N, lat in [-90,90], lon in [-180,180]."""
    if bbox is None or len(bbox) != 4:
        raise ValidationError("bbox must be [W, S, E, N] with 4 floats")
    w, s, e, n = [float(x) for x in bbox]
    if not all(np.isfinite([w, s, e, n])):
        raise ValidationError("bbox contains non-finite values")
    if w < -180.0 or e > 180.0 or s < -90.0 or n > 90.0:
        raise ValidationError(
            f"bbox out of range: lon=[{w},{e}] must be in [-180,180], lat=[{s},{n}] in [-90,90]")
    if w >= e:
        raise ValidationError(
            f"bbox W must be < E; got W={w}, E={e} (likely reversed; this skill does not support wrapping around 180°)")
    if s >= n:
        raise ValidationError(
            f"bbox S must be < N; got S={s}, N={n} (likely reversed)")
    if (e - w) < 1e-9 or (n - s) < 1e-9:
        raise ValidationError(f"bbox has zero area: W={w}, E={e}, S={s}, N={n}")


def validate_params(args) -> None:
    """Validate --min-height / --cell-size / --smooth-sigma values."""
    if not np.isfinite(args.min_height):
        raise ValidationError(f"--min-height must be finite, got {args.min_height}")
    if args.min_height < 0:
        raise ValidationError(
            f"--min-height must be >= 0 (meters), got {args.min_height}")
    if not np.isfinite(args.cell_size):
        raise ValidationError(f"--cell-size must be finite, got {args.cell_size}")
    if args.cell_size <= 0:
        raise ValidationError(
            f"--cell-size must be > 0 (point-cloud units), got {args.cell_size}")
    if hasattr(args, "smooth_sigma") and args.smooth_sigma is not None and args.smooth_sigma < 0:
        raise ValidationError(
            f"--smooth-sigma must be >= 0, got {args.smooth_sigma}")

# ---- 复用共享核心库（本地 vendored，随脚本目录一起分发）----
try:
    from _geoskill_core.errors import (
        GeoSkillError, UsageError, ValidationError, ProcessError, to_exit_code,
    )
    from _geoskill_core.manifest import OutputManifest, OutputFile
except ImportError:  # pragma: no cover - fallback minimal definitions
    class GeoSkillError(Exception):
        def __init__(self, message: str, code: int = 7, kind: str = "EGeo", **kw):
            super().__init__(message)
            self.message, self.code, self.kind = message, code, kind

    class UsageError(GeoSkillError):
        def __init__(self, m, **k): super().__init__(m, code=2, kind="EUsage", **k)

    class ValidationError(GeoSkillError):
        def __init__(self, m, **k): super().__init__(m, code=6, kind="EValidate", **k)

    class ProcessError(GeoSkillError):
        def __init__(self, m, **k): super().__init__(m, code=7, kind="EProcess", **k)

    def to_exit_code(exc):
        return getattr(exc, "code", 7)

    OutputManifest = None
    OutputFile = None


def _utc_now() -> str:
    return _dt.datetime.now(_dt.timezone.utc).isoformat().replace("+00:00", "Z")


# ---------------------------------------------------------------------------
# 栅格化
# ---------------------------------------------------------------------------
def grid_extent(points: np.ndarray, cell_size: float
                ) -> Tuple[float, float, int, int]:
    """返回 (xmin, ymax, width, height)；行 0 = 最北（north-up）。"""
    xmin = float(points[:, 0].min())
    ymin = float(points[:, 1].min())
    xmax = float(points[:, 0].max())
    ymax = float(points[:, 1].max())
    w = max(1, int(np.ceil((xmax - xmin) / cell_size)))
    h = max(1, int(np.ceil((ymax - ymin) / cell_size)))
    return xmin, ymax, w, h


def cell_indices(points: np.ndarray, extent: Tuple[float, float, int, int],
                 cell_size: float) -> Tuple[np.ndarray, np.ndarray]:
    xmin, ymax, w, h = extent
    xi = np.clip(((points[:, 0] - xmin) / cell_size).astype(int), 0, w - 1)
    yi = np.clip(((ymax - points[:, 1]) / cell_size).astype(int), 0, h - 1)
    return xi, yi


def fill_nan_nearest(grid: np.ndarray) -> np.ndarray:
    from scipy.ndimage import distance_transform_edt
    mask = np.isnan(grid)
    if not mask.any():
        return grid.copy()
    if mask.all():
        return np.zeros_like(grid)
    _, idx = distance_transform_edt(mask, return_indices=True)
    return grid[tuple(idx)]


def rasterize_min_surface(points: np.ndarray, extent, cell_size: float) -> np.ndarray:
    xmin, ymax, w, h = extent
    xi, yi = cell_indices(points, extent, cell_size)
    surf = np.full((h, w), np.inf, dtype=np.float64)
    np.minimum.at(surf, (yi, xi), points[:, 2])
    surf[np.isinf(surf)] = np.nan
    return surf


def rasterize_max_surface(points: np.ndarray, extent, cell_size: float) -> np.ndarray:
    xmin, ymax, w, h = extent
    xi, yi = cell_indices(points, extent, cell_size)
    surf = np.full((h, w), -np.inf, dtype=np.float64)
    np.maximum.at(surf, (yi, xi), points[:, 2])
    surf[np.isneginf(surf)] = np.nan
    return surf


def grey_opening(grid: np.ndarray, size: int) -> np.ndarray:
    """灰度开运算（先腐蚀后膨胀）：去除比结构元素小的正向突起。"""
    from scipy.ndimage import grey_erosion, grey_dilation
    return grey_dilation(grey_erosion(grid, size=size), size=size)


def pmf_ground_surface(grid: np.ndarray, cell_size: float, dh_base: float = 0.5,
                       max_slope: float = 0.15,
                       max_window_cells: Optional[int] = None) -> np.ndarray:
    """渐进形态学滤波（PMF）估计裸地面。

    窗口逐级增大（3, 7, 15, 31...）；每级高程差阈值
    dh = dh_base + max_slope × 窗口宽度(米)，随地物尺度放宽，
    既去除小物体（灌木）也去除大物体（乔木）而不破坏平缓地形。
    """
    surface = grid.copy()
    if max_window_cells is None:
        max_window_cells = max(3, (min(grid.shape) // 2) * 2 + 1)
    half = 1
    while 2 * half + 1 <= max_window_cells:
        size = 2 * half + 1
        dh = dh_base + max_slope * size * cell_size
        opened = grey_opening(surface, size=size)
        diff = surface - opened
        mask = diff > dh
        surface[mask] = opened[mask]
        half = 2 * half + 1
    return surface


def estimate_dtm(points: np.ndarray, extent, cell_size: float) -> np.ndarray:
    """最低高程面 + 最近邻填洞 + PMF 渐进开运算 → DTM。

    最低面在树冠/建筑下方会被非地面点抬高；渐进开运算逐级削去这些
    突起，使 DTM 贴近真实裸地。
    """
    min_surf = rasterize_min_surface(points, extent, cell_size)
    return pmf_ground_surface(fill_nan_nearest(min_surf), cell_size)


def compute_chm(points: np.ndarray, extent, cell_size: float
                ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """返回 (chm, dsm, dtm)，CHM 裁剪到 ≥ 0。"""
    dsm = fill_nan_nearest(rasterize_max_surface(points, extent, cell_size))
    dtm = estimate_dtm(points, extent, cell_size)
    chm = np.clip(dsm - dtm, 0.0, None)
    return chm, dsm, dtm


# ---------------------------------------------------------------------------
# 单木检测
# ---------------------------------------------------------------------------
def detect_trees(chm: np.ndarray, extent: Tuple[float, float, int, int],
                 cell_size: float, min_height: float = 2.0,
                 smooth_sigma: float = 1.0,
                 min_area_m2: float = 2.5) -> List[Dict[str, Any]]:
    """阈值化 + 连通域标记 + 局部峰值确认的单木检测。

    先对 CHM 做高斯平滑（ITD 标准预处理，抑制点云噪声造成的伪局部峰，
    见 Persson et al. 2002），在平滑面上用 maximum_filter 找局部峰值；
    阈值掩膜与树高仍在原始 CHM 上量测，保证树高不被平滑衰减。
    小于 min_area_m2 的连通碎片（冠缘采样空洞造成的孤立像元）直接剔除。
    单峰域记 1 棵树，多峰域按峰数拆分、面积均分。
    树高 = 峰值 CHM；冠幅半径 = sqrt(面积/π/峰数)。
    返回按树高降序的树列表（行列下标 + 局部米制坐标 + 树高 + 冠幅）。
    """
    from scipy.ndimage import label, maximum_filter, gaussian_filter
    xmin, ymax = extent[0], extent[1]
    smooth = gaussian_filter(chm, sigma=smooth_sigma) if smooth_sigma > 0 else chm
    mask = chm >= min_height
    lbl, n = label(mask)
    local_max = maximum_filter(smooth, size=3)
    cell_area = cell_size * cell_size
    trees: List[Dict[str, Any]] = []
    for i in range(1, n + 1):
        comp = lbl == i
        if comp.sum() * cell_area < min_area_m2:
            continue
        peaks = comp & (smooth == local_max)
        peak_rc = np.argwhere(peaks)
        if peak_rc.size == 0:
            idx = np.unravel_index(np.argmax(np.where(comp, smooth, -np.inf)), chm.shape)
            peak_rc = np.array([idx])
        # 去重非常近的峰（<2 格）：保留最高
        peak_rc = _dedupe_peaks(peak_rc, smooth, min_dist=2)
        n_peaks = len(peak_rc)
        area_cells = int(comp.sum())
        radius = float(np.sqrt(area_cells / (np.pi * n_peaks)) * cell_size)
        for (r, c) in peak_rc:
            height = float(chm[r, c])
            x_local = xmin + (c + 0.5) * cell_size
            y_local = ymax - (r + 0.5) * cell_size
            trees.append({
                "row": int(r), "col": int(c),
                "x": float(x_local), "y": float(y_local),
                "height": height, "crown_radius": radius,
                "crown_area": float(np.pi * radius * radius),
            })
    trees.sort(key=lambda t: -t["height"])
    for tid, t in enumerate(trees):
        t["tree_id"] = tid
    return trees


def _dedupe_peaks(peak_rc: np.ndarray, chm: np.ndarray,
                  min_dist: int) -> List[Tuple[int, int]]:
    """按 CHM 降序贪心保留间距 ≥ min_dist 的峰。"""
    order = sorted(range(len(peak_rc)), key=lambda k: -chm[peak_rc[k][0], peak_rc[k][1]])
    kept: List[Tuple[int, int]] = []
    for k in order:
        r, c = int(peak_rc[k][0]), int(peak_rc[k][1])
        if all(max(abs(r - kr), abs(c - kc)) >= min_dist for kr, kc in kept):
            kept.append((r, c))
    return kept or [(int(peak_rc[0][0]), int(peak_rc[0][1]))]


# ---------------------------------------------------------------------------
# 合成数据：平面地形 + 高斯冠形树木
# ---------------------------------------------------------------------------
def synthetic_ground(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    return 0.01 * x


def generate_synthetic(bbox: List[float], cell_size: float = 1.0, seed: int = 42,
                       extent_m: float = 64.0, n_trees: int = 10,
                       density: float = 3.0
                       ) -> Tuple[np.ndarray, Dict[str, Any]]:
    """生成 (N, 3) 点云；info['trees'] 为注入真值 [{x, y, height, radius}]。"""
    rng = np.random.default_rng(seed)
    pts: List[np.ndarray] = []

    n_ground = 3000
    gx = rng.uniform(0.0, extent_m, n_ground)
    gy = rng.uniform(0.0, extent_m, n_ground)
    gz = synthetic_ground(gx, gy) + rng.normal(0.0, 0.02, n_ground)
    pts.append(np.column_stack([gx, gy, gz]))

    trees: List[Dict[str, float]] = []
    placed: List[Tuple[float, float, float]] = []
    attempts = 0
    while len(trees) < n_trees and attempts < 500:
        attempts += 1
        R = rng.uniform(2.0, 3.5)
        hgt = rng.uniform(5.0, 15.0)
        cx = rng.uniform(R + 1, extent_m - R - 1)
        cy = rng.uniform(R + 1, extent_m - R - 1)
        if any(np.hypot(cx - px, cy - py) < R + pr + 1.5 for px, py, pr in placed):
            continue
        placed.append((cx, cy, R))
        trees.append({"x": float(cx), "y": float(cy),
                      "height": float(hgt), "radius": float(R)})
        sigma = R / 1.5
        n = max(int(density * np.pi * R * R), 30)
        ang = rng.uniform(0.0, 2.0 * np.pi, n)
        rad = R * np.sqrt(rng.uniform(0.0, 1.0, n))
        px = cx + rad * np.cos(ang)
        py = cy + rad * np.sin(ang)
        pz = synthetic_ground(px, py) + hgt * np.exp(-(rad ** 2) / (2.0 * sigma * sigma)) \
            + rng.normal(0.0, 0.05, n)
        pts.append(np.column_stack([px, py, pz]))
        # 强制顶点，保证树高可精确恢复
        apex = rng.normal(0.0, 0.15, (3, 2))
        pts.append(np.column_stack([
            cx + apex[:, 0], cy + apex[:, 1],
            hgt + synthetic_ground(np.full(3, cx), np.full(3, cy))
            + rng.normal(0.0, 0.03, 3),
        ]))

    points = np.vstack(pts).astype(np.float64)
    info = {
        "bbox": bbox,
        "extent_m": extent_m,
        "cell_size": cell_size,
        "n_points": int(points.shape[0]),
        "n_trees_true": len(trees),
        "trees": trees,
    }
    return points, info


# ---------------------------------------------------------------------------
# 点云 / 栅格 / 矢量 I/O
# ---------------------------------------------------------------------------
def read_points(path: str) -> np.ndarray:
    if not os.path.exists(path):
        raise UsageError(f"input point cloud not found: {path}", path=path)
    if path.lower().endswith(".npy"):
        arr = np.load(path)
    else:
        try:
            arr = np.loadtxt(path, delimiter=",", ndmin=2)
        except ValueError:
            arr = np.loadtxt(path, ndmin=2)
    arr = np.asarray(arr, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[1] < 3:
        raise ValidationError("point cloud must have at least 3 columns (x, y, z)",
                              shape=list(arr.shape))
    return arr[:, :3]


def write_geotiff(path: str, cube: np.ndarray, bbox: List[float],
                  nodata: float = -9999.0) -> None:
    import rasterio
    from rasterio.transform import from_bounds

    if cube.ndim == 2:
        cube = cube[np.newaxis, ...]
    nb, h, w = cube.shape
    transform = from_bounds(bbox[0], bbox[1], bbox[2], bbox[3], w, h)
    profile = {
        "driver": "GTiff", "height": h, "width": w, "count": nb,
        "dtype": "float32", "crs": "EPSG:4326", "transform": transform,
        "nodata": nodata, "compress": "deflate",
    }
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with rasterio.open(path, "w", **profile) as dst:
        for b in range(nb):
            dst.write(cube[b].astype("float32"), b + 1)


def map_to_geo(xy: np.ndarray, work_bbox: List[float],
               geo_bbox: List[float]) -> np.ndarray:
    wx0, wy0, wx1, wy1 = work_bbox
    gx0, gy0, gx1, gy1 = geo_bbox
    sx = (gx1 - gx0) / max(wx1 - wx0, 1e-9)
    sy = (gy1 - gy0) / max(wy1 - wy0, 1e-9)
    return np.column_stack([gx0 + (xy[:, 0] - wx0) * sx,
                            gy0 + (xy[:, 1] - wy0) * sy])


def write_trees_geojson(path: str, trees: List[Dict[str, Any]],
                        xy_geo: np.ndarray) -> None:
    features = []
    for i, t in enumerate(trees):
        features.append({
            "type": "Feature",
            "geometry": {"type": "Point",
                         "coordinates": [round(float(xy_geo[i, 0]), 7),
                                         round(float(xy_geo[i, 1]), 7)]},
            "properties": {
                "tree_id": int(t["tree_id"]),
                "height_m": round(float(t["height"]), 3),
                "crown_radius_m": round(float(t["crown_radius"]), 3),
                "crown_area_m2": round(float(t["crown_area"]), 2),
            },
        })
    doc = {"type": "FeatureCollection", "name": "detected_trees",
           "crs": {"type": "name",
                   "properties": {"name": "urn:ogc:def:crs:OGC:1.3:CRS84"}},
           "features": features}
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(doc, f, ensure_ascii=False, indent=1)


# ---------------------------------------------------------------------------
# 精度评估（合成真值匹配）
# ---------------------------------------------------------------------------
def match_trees(detected: List[Dict[str, Any]], truth: List[Dict[str, Any]],
                max_dist: float = 5.0) -> Tuple[List[Tuple[int, int]], float, float]:
    """最近邻匹配检测树与真值树。返回 (配对, 树高 RMSE, 冠幅 RMSE)。"""
    pairs: List[Tuple[int, int]] = []
    used: set = set()
    for di, d in enumerate(detected):
        best_j, best_d = -1, max_dist
        for tj, t in enumerate(truth):
            if tj in used:
                continue
            dist = float(np.hypot(d["x"] - t["x"], d["y"] - t["y"]))
            if dist < best_d:
                best_j, best_d = tj, dist
        if best_j >= 0:
            pairs.append((di, best_j))
            used.add(best_j)
    if pairs:
        h_err = np.array([detected[di]["height"] - truth[tj]["height"] for di, tj in pairs])
        r_err = np.array([detected[di]["crown_radius"] - truth[tj]["radius"] for di, tj in pairs])
        h_rmse = float(np.sqrt(np.mean(h_err ** 2)))
        r_rmse = float(np.sqrt(np.mean(r_err ** 2)))
    else:
        h_rmse = r_rmse = float("nan")
    return pairs, h_rmse, r_rmse


# ---------------------------------------------------------------------------
# Manifest
# ---------------------------------------------------------------------------
def write_manifest(output_dir: str, args: argparse.Namespace,
                   outputs: List[Dict[str, Any]], qa: Dict[str, Any],
                   started_at: str, exit_code: int, bbox: List[float]) -> Optional[str]:
    if OutputManifest is None:
        return None
    cmd = " ".join([SKILL_NAME] + sys.argv[1:])
    man = OutputManifest(
        skill=SKILL_NAME,
        skill_version=VERSION,
        command=cmd,
        started_at=started_at,
        finished_at=_utc_now(),
        exit_code=exit_code,
        inputs={
            "input": getattr(args, "input", None),
            "min_height": getattr(args, "min_height", None),
            "cell_size": getattr(args, "cell_size", None),
            "synthetic": bool(getattr(args, "synthetic", False)),
        },
        outputs=[OutputFile(**o) for o in outputs],
        qa=qa,
        software={"python": sys.version.split()[0], "skill": SKILL_NAME},
    )
    path = os.path.join(output_dir, "output-manifest.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(man.to_dict(), f, ensure_ascii=False, indent=2, default=str)
    return path


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------
def process(args: argparse.Namespace) -> int:
    started_at = _utc_now()
    output_dir = args.output_dir

    bbox = list(args.bbox) if args.bbox else None
    validate_params(args)

    synth_info: Optional[Dict[str, Any]] = None
    if args.input and not args.synthetic:
        points = read_points(args.input)
        if points.shape[0] == 0:
            raise ValidationError("point cloud is empty (no points)")
        if bbox is None:
            bbox = [float(points[:, 0].min()), float(points[:, 1].min()),
                    float(points[:, 0].max()), float(points[:, 1].max())]
        coord_is_geo = True
        source_note = args.input
    else:
        if bbox is None:
            raise UsageError("provide --bbox (synthetic mode) or --input <point cloud>")
        validate_bbox(bbox)
        points, synth_info = generate_synthetic(bbox, cell_size=args.cell_size)
        coord_is_geo = False
        source_note = "synthetic"

    if points.shape[0] == 0:
        raise ValidationError("point cloud is empty")
    if not np.all(np.isfinite(points)):
        raise ValidationError("point cloud contains non-finite coordinates")

    os.makedirs(output_dir, exist_ok=True)

    extent = grid_extent(points, args.cell_size)
    chm, dsm, dtm = compute_chm(points, extent, args.cell_size)
    trees = detect_trees(chm, extent, args.cell_size, args.min_height)

    heights = [t["height"] for t in trees]
    radii = [t["crown_radius"] for t in trees]
    stats: Dict[str, Any] = {
        "source": source_note,
        "cell_size": args.cell_size,
        "min_height_filter": args.min_height,
        "n_trees_detected": len(trees),
        "mean_height": float(np.mean(heights)) if heights else 0.0,
        "max_height": float(np.max(heights)) if heights else 0.0,
        "height_std": float(np.std(heights)) if heights else 0.0,
        "mean_crown_radius": float(np.mean(radii)) if radii else 0.0,
        "total_canopy_area_m2": float(sum(t["crown_area"] for t in trees)),
    }
    if synth_info is not None:
        pairs, h_rmse, r_rmse = match_trees(trees, synth_info["trees"])
        stats["n_trees_true"] = synth_info["n_trees_true"]
        stats["n_matched"] = len(pairs)
        stats["detection_rate"] = len(pairs) / max(synth_info["n_trees_true"], 1)
        stats["height_rmse_m"] = h_rmse
        stats["crown_radius_rmse_m"] = r_rmse

    # 写出产物
    chm_path = os.path.join(output_dir, "chm.tif")
    write_geotiff(chm_path, chm.astype(np.float32), bbox, nodata=-1.0)

    work_bbox = [extent[0], extent[1] - extent[3] * args.cell_size,
                 extent[0] + extent[2] * args.cell_size, extent[1]]
    if coord_is_geo:
        xy_geo = np.array([[t["x"], t["y"]] for t in trees]).reshape(-1, 2)
    else:
        xy_local = np.array([[t["x"], t["y"]] for t in trees]).reshape(-1, 2)
        xy_geo = map_to_geo(xy_local, work_bbox, bbox) if len(trees) else np.zeros((0, 2))
    trees_path = os.path.join(output_dir, "trees.geojson")
    write_trees_geojson(trees_path, trees, xy_geo)

    stats_path = os.path.join(output_dir, "stats.json")
    with open(stats_path, "w", encoding="utf-8") as f:
        json.dump(stats, f, ensure_ascii=False, indent=2)

    qa: Dict[str, Any] = {
        "source": source_note,
        "n_trees_detected": len(trees),
        "mean_height": stats["mean_height"],
        "max_height": stats["max_height"],
        "detection_rate": stats.get("detection_rate"),
        "height_rmse_m": stats.get("height_rmse_m"),
    }
    outputs = [
        {"path": chm_path, "kind": "raster", "crs_epsg": 4326,
         "bbox_wgs84": bbox, "band_count": 1},
        {"path": trees_path, "kind": "vector", "crs_epsg": 4326,
         "bbox_wgs84": bbox, "feature_count": len(trees)},
        {"path": stats_path, "kind": "json"},
    ]
    man_path = write_manifest(output_dir, args, outputs, qa, started_at, 0, bbox)

    if not args.quiet:
        print(f"[{SKILL_NAME}] source: {source_note}")
        print(f"[{SKILL_NAME}] trees detected: {len(trees)}  (min_height={args.min_height})")
        if heights:
            print(f"[{SKILL_NAME}] height: mean={stats['mean_height']:.2f}  max={stats['max_height']:.2f} m")
        if "detection_rate" in stats:
            print(f"[{SKILL_NAME}] detection rate: {stats['detection_rate']:.2f}  height RMSE: {stats['height_rmse_m']:.3f} m")
        print(f"[{SKILL_NAME}] output: {chm_path}")
        if man_path:
            print(f"[{SKILL_NAME}] manifest: {man_path}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog=SKILL_NAME,
        description="LiDAR canopy structure: CHM, individual tree detection, height & crown.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--bbox", nargs=4, type=float, metavar=("W", "S", "E", "N"),
                   help="geographic extent [minLon minLat maxLon maxLat]")
    p.add_argument("--input", help="input point cloud (.npy or .csv/.txt xyz)")
    p.add_argument("--min-height", type=float, default=2.0,
                   help="minimum tree height to detect, meters (default: 2.0)")
    p.add_argument("--cell-size", type=float, default=1.0,
                   help="rasterization cell size in point-cloud units (default: 1.0)")
    p.add_argument("--synthetic", action="store_true",
                   help="generate a synthetic forest scene (offline)")
    p.add_argument("--output-dir", default="./output", help="output directory")
    p.add_argument("--quiet", action="store_true", help="suppress console output")
    p.add_argument("--version", action="version", version=f"%(prog)s {VERSION}")
    return p


def main(argv: Optional[List[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return process(args)
    except GeoSkillError as exc:
        print(f"[{SKILL_NAME}] ERROR [{exc.kind}] {exc.message}", file=sys.stderr)
        return exc.code
    except KeyboardInterrupt:
        return 130
    except Exception as exc:  # noqa: BLE001
        print(f"[{SKILL_NAME}] ERROR {exc}", file=sys.stderr)
        return to_exit_code(exc)


if __name__ == "__main__":
    sys.exit(main())
