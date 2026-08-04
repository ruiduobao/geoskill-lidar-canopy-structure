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

<!-- ===== 中文原文 (Chinese Original) ===== -->

---
name: geoskill-lidar-canopy-structure
description: 'CHM冠层高度模型+局部峰值单木检测与树高冠幅提取'
---

# LiDAR 冠层结构分析 | LiDAR Canopy Structure Analysis

从 LiDAR 点云提取森林冠层结构参数。流程：

1. **DTM**：最低高程面 + 渐进形态学开运算（PMF，Zhang et al. 2003
   简化版）逐级削去树冠/建筑突起，得到裸地模型；
2. **CHM**：最高高程面 DSM 减 DTM，得冠层高度模型（Canopy Height
   Model），裁剪到 ≥ 0；
3. **单木检测（ITD）**：CHM 阈值化 + 连通域标记（简化分水岭），高斯
   平滑（Persson et al. 2002 标准预处理）抑制噪声伪峰后用
   maximum_filter 确认局部峰值，剔除小于最小冠幅面积的碎片；
4. **参数提取**：树高 = 峰值处原始 CHM（不经平滑衰减），冠幅半径 =
   连通域等效圆半径。

合成模式生成平面地形 + 若干高斯冠形树木（随机高度 5–15 m、冠径
2–3.5 m）的模拟点云，并自动与真值匹配输出检测率、树高 RMSE、冠幅
RMSE。适用于森林资源调查、单木定位、生物量估算前端。

## 依赖

```bash
pip install numpy rasterio scipy
```

## 使用方法

### 基本用法（合成数据，离线）

```bash
python geoskill-lidar-canopy-structure.py --bbox 116.0 39.0 117.0 40.0 --synthetic --output-dir ./out
```

### 示例 1：默认 2 m 起测树高

```bash
python geoskill-lidar-canopy-structure.py \
    --bbox 116.0 39.0 117.0 40.0 \
    --synthetic --min-height 2.0 \
    --output-dir ./itd_2m
```

### 示例 2：提高起测高度（只检大树）

```bash
python geoskill-lidar-canopy-structure.py \
    --bbox 121.0 31.0 122.0 32.0 \
    --synthetic --min-height 8.0 \
    --output-dir ./itd_8m
```

### 示例 3：真实点云输入（.npy / .csv / .txt xyz）

```bash
python geoskill-lidar-canopy-structure.py \
    --input forest_plot.npy --min-height 3.0 --cell-size 0.5 \
    --output-dir ./real_plot
```

### 示例 4：粗格网快速预览

```bash
python geoskill-lidar-canopy-structure.py \
    --input forest_plot.csv --cell-size 2.0 --quiet \
    --output-dir ./coarse
```

## 输出

| 文件 | 格式 | 说明 |
|---|---|---|
| `chm.tif` | GeoTIFF (float32) | 冠层高度模型，EPSG:4326 |
| `trees.geojson` | GeoJSON (Point) | 单木位置 + 树高/冠幅/冠面积属性 |
| `stats.json` | JSON | 株数、树高统计、检测率与 RMSE（合成模式） |
| `output-manifest.json` | JSON | 运行清单（输入/输出/QA/软件版本） |

## 数据源 / Source

- **真实模式**：本地点云文件（.npy / .csv / .txt，至少 3 列 xyz，
  建议投影或局部米制坐标）
- **合成模式**：本地生成平面地形 + 高斯冠形树木点云，无外部数据源

## 隐私声明 / Privacy

- 完全离线运行，不访问任何网络服务
- 所有处理在本地完成，不上传任何用户数据

## License

MIT
