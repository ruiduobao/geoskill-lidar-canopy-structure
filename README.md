# LiDAR Canopy Structure Analysis (geoskill-lidar-canopy-structure)

> CHM canopy height model + local-maxima single-tree detection with tree height and crown width extraction

---

## 1. Overview

Extracts forest canopy structure parameters from LiDAR point clouds. Workflow: 1. **DTM**: lowest elevation surface + progressive morphological opening (PMF, simplified version of Zhang et al. 2003) that progressively removes canopy/building protrusions to obtain a bare-earth model; 2. **CHM**: DSM (highest elevation surface) minus DTM gives the Canopy Height Model, clipped to ≥ 0; 3. **Individual tree detection (ITD)**: CHM thresholding + connected-component labeling (simplified watershed), with Gaussian smoothing (standard preprocessing per Persson et al. 2002) to suppress noise-induced false peaks before confirming local maxima with maximum_filter, discarding fragments smaller than the minimum crown area; 4. **Parameter extraction**: tree height = original CHM at the peak (without smoothing attenuation), crown radius = equivalent circle radius of the connected component. Synthetic mode generates a simulated point cloud of a flat terrain plus several Gaussian-crown trees (random heights 5–15 m, crown diameters 2–3.5 m), and automatically matches against ground truth to output detection rate, tree height RMSE, and crown width RMSE. Suitable for forest resource surveys, individual tree location, and as the front end of biomass estimation.

## 3. Quick Start

```bash
pip install -r requirements.txt
python geoskill-lidar-canopy-structure.py --bbox 116 39 117 40 --synthetic --output-dir ./out
```

## 4. CLI Parameters

Run `python <skill>.py --help` for the full list. Common parameters:

| Parameter | Type | Description |
|---|---|---|
| `--bbox` | `float[4]` | WGS84 bounding box `min_lon min_lat max_lon max_lat` |
| `--input` | `path` | Local input file (GeoJSON/GeoTIFF/etc.) |
| `--output-dir` | `path` | Output directory (default `./output`) |
| `--synthetic` | `flag` | Use synthetic data instead of real input |
| `--quiet` | `flag` | Suppress non-essential stdout |

## 5. Input / Output

| File | Format | Description |
|---|---|---|
| `chm.tif` | GeoTIFF (float32) | Canopy height model, EPSG:4326 |
| `trees.geojson` | GeoJSON (Point) | Individual tree locations + tree height/crown width/crown area attributes |
| `stats.json` | JSON | Tree count, height statistics, detection rate and RMSE (synthetic mode) |
| `output-manifest.json` | JSON | Run manifest (input/output/QA/software versions) |

## 6. Technical Principle

(See SKILL.md for details)

## 8. License

MIT License. See [`LICENSE`](./LICENSE) for full text.

---

> 中文完整版：[references/SKILL.zh-CN.md](references/SKILL.zh-CN.md)
