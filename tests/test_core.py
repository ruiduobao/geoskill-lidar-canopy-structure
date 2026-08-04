"""Core algorithm tests for lidar-canopy-structure."""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(__file__))
from conftest import mod as lc


def _two_bump_chm():
    yy, xx = np.mgrid[0:32, 0:32]
    chm = (8.0 * np.exp(-(((xx - 10) ** 2 + (yy - 10) ** 2)) / (2 * 2.5 ** 2))
           + 5.0 * np.exp(-(((xx - 24) ** 2 + (yy - 24) ** 2)) / (2 * 2.0 ** 2)))
    return chm.astype(np.float64)


class TestRasterSurfaces:
    def test_max_surface(self):
        points = np.array([[0.5, 0.5, 2.0],
                           [0.5, 0.5, 7.0],
                           [3.5, 3.5, 1.0]])
        extent = lc.grid_extent(points, 1.0)
        dsm = lc.rasterize_max_surface(points, extent, 1.0)
        assert np.nanmax(dsm) == 7.0

    def test_chm_nonnegative(self):
        points, info = lc.generate_synthetic([116, 39, 117, 40], seed=2)
        extent = lc.grid_extent(points, 1.0)
        chm, dsm, dtm = lc.compute_chm(points, extent, 1.0)
        assert chm.min() >= 0.0
        assert chm.shape == dsm.shape == dtm.shape

    def test_dtm_near_ground(self):
        """合成平面地形上 DTM 应贴近真值地面（0.01x）。"""
        points, info = lc.generate_synthetic([116, 39, 117, 40], seed=3)
        extent = lc.grid_extent(points, 1.0)
        dtm = lc.estimate_dtm(points, extent, 1.0)
        xmin, ymax, w, h = extent
        xs = xmin + (np.arange(w) + 0.5)
        truth_row = 0.01 * xs
        # 取中间行比较（避开边界填充）
        mid = dtm[h // 2]
        assert np.abs(mid - truth_row).mean() < 0.3


class TestDetectTrees:
    def test_two_bumps(self):
        chm = _two_bump_chm()
        extent = (0.0, 32.0, 32, 32)
        trees = lc.detect_trees(chm, extent, 1.0, min_height=2.0)
        assert len(trees) == 2
        assert abs(trees[0]["height"] - 8.0) < 0.5
        assert abs(trees[1]["height"] - 5.0) < 0.5
        assert trees[0]["crown_radius"] > 0

    def test_min_height_filters(self):
        chm = _two_bump_chm()
        trees = lc.detect_trees(chm, (0.0, 32.0, 32, 32), 1.0, min_height=7.0)
        assert len(trees) == 1
        assert abs(trees[0]["height"] - 8.0) < 0.5

    def test_no_trees_on_flat(self):
        chm = np.zeros((16, 16))
        assert lc.detect_trees(chm, (0.0, 16.0, 16, 16), 1.0, 2.0) == []

    def test_local_coordinates(self):
        chm = _two_bump_chm()
        extent = (100.0, 232.0, 32, 32)  # xmin=100, ymax=232
        trees = lc.detect_trees(chm, extent, 1.0, min_height=2.0)
        # 最高树峰值在 (row=10, col=10)
        t = trees[0]
        assert abs(t["x"] - (100.0 + 10.5)) < 0.6
        assert abs(t["y"] - (232.0 - 10.5)) < 0.6


class TestSyntheticPipeline:
    def test_detection_count_and_height(self):
        """端到端：检测树数与注入一致（±2），树高 RMSE < 1.5 m。"""
        points, info = lc.generate_synthetic([116, 39, 117, 40], seed=11, n_trees=8)
        extent = lc.grid_extent(points, 1.0)
        chm, _dsm, _dtm = lc.compute_chm(points, extent, 1.0)
        trees = lc.detect_trees(chm, extent, 1.0, min_height=2.0)
        n_true = info["n_trees_true"]
        assert n_true == 8
        assert abs(len(trees) - n_true) <= 2
        pairs, h_rmse, r_rmse = lc.match_trees(trees, info["trees"])
        assert len(pairs) >= n_true - 2
        assert h_rmse < 1.5, f"h_rmse={h_rmse}"
        assert r_rmse < 1.5, f"r_rmse={r_rmse}"

    def test_trees_info(self):
        points, info = lc.generate_synthetic([116, 39, 117, 40], seed=4)
        assert len(info["trees"]) == 10
        assert points.shape[1] == 3


class TestMatchTrees:
    def test_identity(self):
        truth = [{"x": 5.0, "y": 5.0, "height": 10.0, "radius": 3.0},
                 {"x": 30.0, "y": 30.0, "height": 6.0, "radius": 2.0}]
        detected = [{"x": 5.0, "y": 5.0, "height": 10.0, "crown_radius": 3.0},
                    {"x": 30.0, "y": 30.0, "height": 6.0, "crown_radius": 2.0}]
        pairs, h_rmse, r_rmse = lc.match_trees(detected, truth)
        assert len(pairs) == 2
        assert h_rmse == 0.0 and r_rmse == 0.0

    def test_no_match_far(self):
        truth = [{"x": 0.0, "y": 0.0, "height": 10.0, "radius": 3.0}]
        detected = [{"x": 99.0, "y": 99.0, "height": 10.0, "crown_radius": 3.0}]
        pairs, h_rmse, _ = lc.match_trees(detected, truth)
        assert pairs == []
        assert np.isnan(h_rmse)


class TestReadPoints:
    def test_read_npy(self, tmp_path):
        p = str(tmp_path / "c.npy")
        np.save(p, np.random.rand(15, 3))
        assert lc.read_points(p).shape == (15, 3)

    def test_read_txt(self, tmp_path):
        p = str(tmp_path / "c.txt")
        with open(p, "w", encoding="utf-8") as f:
            f.write("0 0 1\n1 1 2\n")
        assert lc.read_points(p).shape == (2, 3)

    def test_missing_raises(self):
        with pytest.raises(lc.UsageError):
            lc.read_points("/nonexistent/cloud.npy")


class TestGeoOutputs:
    def test_write_geotiff(self, tmp_path):
        p = str(tmp_path / "x.tif")
        lc.write_geotiff(p, np.random.rand(8, 8), [116.0, 39.0, 117.0, 40.0])
        assert os.path.exists(p)

    def test_write_geojson(self, tmp_path):
        import json
        trees = [{"tree_id": 0, "height": 9.0, "crown_radius": 2.5, "crown_area": 19.6}]
        p = str(tmp_path / "t.geojson")
        lc.write_trees_geojson(p, trees, np.array([[116.5, 39.5]]))
        with open(p, encoding="utf-8") as f:
            doc = json.load(f)
        assert doc["features"][0]["properties"]["height_m"] == 9.0

    def test_map_to_geo(self):
        xy = np.array([[0.0, 0.0], [64.0, 64.0]])
        geo = lc.map_to_geo(xy, [0.0, 0.0, 64.0, 64.0], [116.0, 39.0, 117.0, 40.0])
        np.testing.assert_allclose(geo[0], [116.0, 39.0])
        np.testing.assert_allclose(geo[1], [117.0, 40.0])
