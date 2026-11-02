---
name: geoskill-lidar-canopy-structure
description: 'CHM冠层高度模型+局部峰值单木检测与树高冠幅提取'
---

# LiDAR 冠层结构分析 | LiDAR Canopy Structure Analysis

Extracts forest canopy structure parameters from LiDAR point clouds. Workflow:

1. **DTM**: lowest elevation surface + progressive morphological opening (PMF, a simplified version of Zhang et al. 2003) that successively trims canopy/building protrusions to obtain a bare-earth model;
2. **CHM**: subtract the DTM from the highest-elevation DSM to obtain the Canopy Height Model, clipped to ≥ 0;
3. **Individual tree detection (ITD)**: threshold the CHM + connected-component labeling (simplified watershed); apply Gaussian smoothing (the standard preprocessing of Persson et al. 2002) to suppress noise-induced false peaks, then confirm local maxima with maximum_filter and remove fragments smaller than the minimum crown area;
4. **Parameter extraction**: tree height = the raw CHM at the peak (not attenuated by smoothing); crown radius = the equivalent circle radius of the connected component.

Synthetic mode generates a simulated point cloud of flat terrain plus several Gaussian-crown trees (random heights 5–15 m, crown diameters 2–3.5 m), automatically matches them against ground truth, and reports detection rate, tree-height RMSE, and crown-width RMSE. Suited to forest resource inventory, individual tree location, and as a front end for biomass estimation.

## Dependencies / 依赖

```bash
pip install numpy rasterio scipy
```

## Usage / 使用方法

### Basic usage (synthetic data, offline)

```bash
python geoskill-lidar-canopy-structure.py --bbox 116.0 39.0 117.0 40.0 --synthetic --output-dir ./out
```

### Example 1: default 2 m minimum tree height

```bash
python geoskill-lidar-canopy-structure.py \
    --bbox 116.0 39.0 117.0 40.0 \
    --synthetic --min-height 2.0 \
    --output-dir ./itd_2m
```

### Example 2: higher minimum height (detect large trees only)

```bash
python geoskill-lidar-canopy-structure.py \
    --bbox 121.0 31.0 122.0 32.0 \
    --synthetic --min-height 8.0 \
    --output-dir ./itd_8m
```

### Example 3: real point cloud input (.npy / .csv / .txt xyz)

```bash
python geoskill-lidar-canopy-structure.py \
    --input forest_plot.npy --min-height 3.0 --cell-size 0.5 \
    --output-dir ./real_plot
```

### Example 4: coarse grid quick preview

```bash
python geoskill-lidar-canopy-structure.py \
    --input forest_plot.csv --cell-size 2.0 --quiet \
    --output-dir ./coarse
```

## Output / 输出

| File | Format | Description |
|---|---|---|
| `chm.tif` | GeoTIFF (float32) | Canopy height model, EPSG:4326 |
| `trees.geojson` | GeoJSON (Point) | Tree positions + height/crown width/crown area attributes |
| `stats.json` | JSON | Tree count, height statistics, detection rate and RMSE (synthetic mode) |
| `output-manifest.json` | JSON | Run manifest (input/output/QA/software versions) |

## Data Source / 数据源 / Source

- **Real mode**: local point cloud files (.npy / .csv / .txt, at least 3 columns of xyz, projected or local metric coordinates recommended)
- **Synthetic mode**: locally generated point cloud of flat terrain + Gaussian-crown trees; no external data source

## Privacy / 隐私声明 / Privacy

- Runs fully offline; accesses no network services
- All processing is done locally; no user data is uploaded

## License / License

MIT

---

> 中文完整版：需要时再读 [references/SKILL.zh-CN.md](references/SKILL.zh-CN.md)，不要默认全量加载。
