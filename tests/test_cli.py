"""CLI argument parsing tests for lidar-canopy-structure."""
import json
import os
import subprocess
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from conftest import SCRIPT, mod as lc


def run_cli(args, timeout=180):
    return subprocess.run(
        [sys.executable, SCRIPT] + args,
        capture_output=True, text=True, timeout=timeout,
    )


class TestCLIBasics:
    def test_help(self):
        r = run_cli(["--help"])
        assert r.returncode == 0
        assert "--bbox" in r.stdout
        assert "--synthetic" in r.stdout
        assert "--min-height" in r.stdout

    def test_version(self):
        r = run_cli(["--version"])
        assert r.returncode == 0
        assert "1.0.0" in r.stdout

    def test_no_args_exit_2(self):
        r = run_cli([])
        assert r.returncode == 2

    def test_bad_min_height_exit_2(self):
        """非法浮点参数 → argparse error → exit 2。"""
        r = run_cli(["--bbox", "116", "39", "117", "40", "--synthetic", "--min-height", "abc"])
        assert r.returncode == 2

    def test_input_file_not_found_exit_2(self):
        r = run_cli(["--input", "nonexistent.npy"])
        assert r.returncode == 2

    def test_synthetic_without_bbox_exit_2(self):
        r = run_cli(["--synthetic"])
        assert r.returncode == 2


class TestCLIRun:
    def test_synthetic_run(self, tmp_path):
        out = str(tmp_path / "out")
        r = run_cli(["--bbox", "116", "39", "117", "40", "--synthetic",
                     "--min-height", "2", "--output-dir", out, "--quiet"])
        assert r.returncode == 0, r.stderr
        assert os.path.exists(os.path.join(out, "chm.tif"))
        assert os.path.exists(os.path.join(out, "trees.geojson"))
        assert os.path.exists(os.path.join(out, "stats.json"))
        assert os.path.exists(os.path.join(out, "output-manifest.json"))

    def test_bbox_only_auto_synthetic(self, tmp_path):
        out = str(tmp_path / "out_auto")
        r = run_cli(["--bbox", "121", "31", "122", "32",
                     "--output-dir", out, "--quiet"])
        assert r.returncode == 0, r.stderr

    def test_detection_quality(self, tmp_path):
        out = str(tmp_path / "out_q")
        r = run_cli(["--bbox", "116", "39", "117", "40", "--synthetic",
                     "--output-dir", out, "--quiet"])
        assert r.returncode == 0, r.stderr
        with open(os.path.join(out, "stats.json"), encoding="utf-8") as f:
            stats = json.load(f)
        assert stats["detection_rate"] >= 0.8
        assert stats["height_rmse_m"] < 1.5

    def test_input_npy(self, tmp_path):
        points, _ = lc.generate_synthetic([116, 39, 117, 40], seed=9)
        p = str(tmp_path / "cloud.npy")
        np.save(p, points)
        out = str(tmp_path / "out_in")
        r = run_cli(["--input", p, "--output-dir", out, "--quiet"])
        assert r.returncode == 0, r.stderr
        assert os.path.exists(os.path.join(out, "output-manifest.json"))
