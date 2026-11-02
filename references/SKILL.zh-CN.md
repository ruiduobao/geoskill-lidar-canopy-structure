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
