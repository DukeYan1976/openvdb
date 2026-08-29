# MicroGridLab 算法完整设计文档

## 地位声明

MicroGridLab 是 yggdrasil-midgard 的**未来核心算法模块**。后续所有算法（主流水线 V4.0 的 MacroCut/MicroCut 等）将围绕此模块进行适配和重构。本文档是唯一权威的 MicroGridLab 算法说明。

---

## 目录

1. [设计动机](#1-设计动机)
2. [数据模型](#2-数据模型)
3. [SDF 压缩存储](#3-sdf-压缩存储)
4. [全量切削管线 (executeCut)](#4-全量切削管线-executecut)
5. [增量切削管线 (executeCutIncremental)](#5-增量切削管线-executecutincremental)
6. [八叉树表面提取 (extractSurface)](#6-八叉树表面提取-extractsurface)
7. [核心类型与接口](#7-核心类型与接口)
8. [数学基础](#8-数学基础)
9. [复杂度与性能](#9-复杂度与性能)
10. [集成架构与演进路线](#10-集成架构与演进路线)

---

## 1. 设计动机

### 1.1 主流水线 V4.0 的负担

| 问题 | 根因 |
|------|------|
| 双数据结构同步 | MacroGrid (FloatGrid) 与 MicroGrid (PointDataGrid) 需时刻保持一致性 |
| 每刀全量重建 | Phase 0-4 通跑，即使只有 1 个 voxel 受影响 |
| OpenVDB 依赖重 | PointDataGrid 的 leaf node alloc/free 开销大 |
| 点云走样 | 离散点集难以直接做 SDF 级 bool 运算 |

### 1.2 MicroGridLab 的设计目标

1. **统一 SDF 场表示** — 单层数据，同时承载拓扑和几何
2. **增量切削** — 只处理新刀影响的 CE，复用旧表面点
3. **紧凑内存** — 512-bit 双 mask + int16 量化，CE 粒度比 voxel 更细
4. **自包含** — 除类型定义和 ToolSweptSDF 外不依赖 OpenVDB
5. **可并行** — TBB 三层并行：eval / 旧点验证 / 精修提取

---

## 2. 数据模型

### 2.1 层次结构

```
MicroGridLabState                    ← 全局状态
├── cubeSize, voxelSize, precision  ← 几何参数
├── currentTool, currentSegment     ← 当前刀具/刀路编辑
├── std::vector<MicroGridCell>       ← N³ 个 voxel (N = cubeSize/voxelSize)
│   └── MicroGridCell
│       ├── activeMask[8]           ← 512-bit: CE 是否在窄带 (BOUNDARY)
│       ├── signMask[8]             ← 512-bit: CE 在材料内/外
│       ├── std::vector<int16_t> sdf ← 压缩 SDF (仅存 BOUNDARY CE 的值)
│       └── 方法见 §3
├── std::vector<CutRecord>          ← 切削历史
├── std::vector<SurfaceSample>      ← 最近一刀的表面点
├── 统计字段 (lastDelta, lastCutLog, ...)
└── 方法 (init, reset, rebuild, executeCut, executeCutIncremental)
```

### 2.2 坐标系统

| 层级 | 单位 | 尺寸 | 数量 |
|------|------|------|------|
| World | mm | `cubeSize`³ | 1 个工作立方体 |
| Voxel | mm | `voxelSize`³ | `(cubeSize/voxelSize)`³ |
| CE (Corner Element) | mm | `voxelSize/8`³ | 每个 voxel 固定 512 (8³) |

**CE 的线性索引** (voxel 内 0..511):
$$ceIdx(i_x, i_y, i_z) = i_z \cdot 64 + i_y \cdot 8 + i_x \quad (i_x, i_y, i_z \in [0,7])$$

**CE 中心世界坐标**:
$$\mathbf{p}_{ce} = \mathbf{o}_{voxel} + \left(i_x \cdot ceSize + \frac{ceSize}{2},\; i_y \cdot ceSize + \frac{ceSize}{2},\; i_z \cdot ceSize + \frac{ceSize}{2}\right)$$

其中 $ceSize = voxelSize / 8$。

---

## 3. SDF 压缩存储

### 3.1 核心思想

传统 SDF 网格每个 cell 存一个 float，密集存储。MicroGridLab 利用**切削问题的稀疏性**：

- 绝大多数 CE 要么在材料深处 (内)，要么已被切除 (外) — 不需要精确 SDF 值
- 只有**窄带边界 CE** 需要精确 SDF 做八叉树精修

因此：
- **内部 CE**：`signMask=1, activeMask=0` — SDF 隐式回退为 `+bg`（正=材料内部）
- **外部 CE**：`signMask=0, activeMask=0` — SDF 隐式回退为 `-bg`（负=材料外部）
- **边界 CE**：`activeMask=1, signMask=(sdf>0)` — SDF 存在压缩数组中

### 3.2 512-bit Double Mask

```
CE idx:    0   1   2   ...   63 | 64   ...  127 | ... | 448  ...  511
           ├── slot 0 ───────────┤── slot 1 ──────┤ ... │── slot 7 ────┤
active:    uint64_t              uint64_t         ...   uint64_t
sign:      uint64_t              uint64_t         ...   uint64_t

查询 isActive(i):
  slot = i >> 6           (i / 64)
  bit  = i & 63           (i % 64)
  return (activeMask[slot] >> bit) & 1

查询 isInside(i):
  同上用 signMask
```

### 3.3 压缩 SDF 数组

```cpp
std::vector<int16_t> sdf;  // 变长 — 只存 activeMask=1 的 CE
```

**定位**：要读第 `idx` 个 CE 的 SDF 值，需计算 `popcountBefore(activeMask, idx)` 得到它在 `sdf[]` 中的偏移量。

**量化/反量化**：
$$val_{int16} = \text{clamp}\left(\frac{sdf_{float}}{bg},\; -1,\; 1\right) \cdot 32767$$

$$sdf_{float} = \frac{val_{int16}}{32767} \cdot bg$$

其中 `bg` 为窄带半宽 = `3 × ceSize`。

**getValue 的完整逻辑**：
```
if (!isActive(idx)):
    return isInside(idx) ? +bg : -bg   // 隐式回退
else:
    offset = popcountBefore(activeMask, idx)  // 从掩码推算偏移
    return decodeSDF(sdf[offset], bg)
```

### 3.4 三态 CE 分类

在 `buildFromSDF` 和 `updateCeFromToolSDF` 中：

| 新刀 SDF 范围 | 旧状态 | 新状态 | 操作 |
|---------------|--------|--------|------|
| $> +threshold$ | — | — | 刀外，不影响 |
| $< -threshold$ | SOLID | **AIR** | 内部 CE 被完全切掉 |
| $< -threshold$ | BOUNDARY | **AIR** | 边界 CE 被完全切掉 |
| $\|sdf\| < threshold$ | SOLID | **BOUNDARY** | 插入压缩 SDF |
| $\|sdf\| < threshold$ | BOUNDARY | **BOUNDARY** | $sdf \gets \min(old, tool)$ |

**关键性质**：AIR 状态不可逆 — 已经切除的 CE 不会被新刀恢复。这是布尔差集的本质约束。

---

## 4. 全量切削管线 (executeCut)

### 4.1 接口

```cpp
void MicroGridLabState::executeCut(size_t cutIdx);
```

从 `cutHistory[cutIdx]` 读取刀具和刀路，对所有 voxel **全量**执行切削。

### 4.2 Phase 1/3: 粗筛 (Coarse Screening)

```
for each (voxel, CE):
    计算 CE 中心世界坐标
    toolSDF = ToolSweptSDF::eval(center)

    if CE 是空气 (已切除):
        newSDF = -bg                      // 不复原
    else:
        newSDF = min(oldSDF, toolSDF)     // 取更小的 (更内/更负)

    cell.buildFromSDF(newSDF512, thresh, bg)
```

用 `min(old, tool)` 实现布尔差集的 SDF 表示：被切掉的 CE 从正变为负。

### 4.3 Phase 2/3: 重建 (Build SDF)

`MicroGridCell::buildFromSDF` 做两遍遍历：

1. **分类遍历**：512 个 CE → 判定 AIR / SOLID / BOUNDARY，填充 mask 位
2. **压缩遍历**：BOUNDARY CE 按序填入 `sdf[]`，量化 int16

### 4.4 Phase 3/3: 八叉树精修

```
for each voxel:
    for each boundary CE:
        extractSurface(ceBbox, toolSDF, octCfg)
        → surfacePoints.append(...)
```

见 §6 详细说明。

### 4.5 统计输出

```
CutLog {
    airBefore, solidBefore, bndBefore
    airAfter,  solidAfter,  bndAfter
    elapsedMs
}
```

---

## 5. 增量切削管线 (executeCutIncremental)

**这是 MicroGridLab 的核心创新，也是未来主推的切削策略。**

### 5.1 接口

```cpp
void MicroGridLabState::executeCutIncremental(size_t cutIdx);
```

第一刀 (`cutIdx == 0`) 回退到全量 `executeCut(0)`。

### 5.2 Phase ①: TBB 并行粗筛

```
TBB parallel_for all (voxel, CE) as flat index:
    if CE 是空气: skip (tag=NO_CHANGE)
    else:
        toolSDF = ToolSweptSDF::eval(center)
        预分类:
            SOLID + toolInside   → SOLID→AIR
            SOLID + toolBnd      → SOLID→BND
            BOUNDARY + toolInside → BND→AIR
            BOUNDARY + toolBnd    → BND→BND (update)

    写入 toolVals[flat] 和 updateTags[flat]
```

**关键**：99% 耗时在 `ToolSweptSDF::eval` 上，这里完全并行。仅状态写入（~1%）串行。

### 5.3 串行 Apply

```cpp
for flat = 0..totalCe-1:
    if updateTags[flat] == NO_CHANGE: continue
    cell.updateCeFromToolSDF(ceIdx, toolVals[flat], thresh, bg)
```

`updateCeFromToolSDF` 处理 CE 的位操作和变长 SDF 数组插入/更新/删除。因为涉及变长数组的 insert/erase，必须串行。

**返回枚举**：
```cpp
enum class CeUpdate {
    NO_CHANGE, AIR_TO_BND, AIR_TO_SOLID,
    SOLID_TO_BND, SOLID_TO_AIR,
    BND_TO_AIR, BND_UNCHANGED
};
```

### 5.4 Phase ②: 旧点验证 (TBB 并行)

```
saved = oldPoints (move ownership)

TBB parallel_for all saved oldPoints:
    if toolSDF.eval(oldPoint) < 0:
        local.pruned++          // 点在刀内 → 丢弃
    else:
        local.kept.push(...)    // 点在刀外 → 保留

合并所有线程的 kept → surfacePoints
```

**增量优势**：只需要对旧表面点做一次快速的 SDF 判剔，不需要重新做八叉树提取。

### 5.5 Phase ③: 新/改边界精修 (TBB 并行)

```
TBB parallel_for all refineCandidates (SOLID→BND 和 BND→BND 的 CE):
    extractSurface(ceBbox, toolSDF, octCfg)
    → thread-local points.append(...)

合并所有线程的点 → surfacePoints
```

### 5.6 Phase ④: 弦高误差 (TBB 并行 reduce)

```
TBB parallel_for all surfacePoints:
    err = abs(toolSDF.eval(sp.position))
    local.maxErr = max(local.maxErr, err)

全局 reduce: maxChordalError = max(all thread maxErr)
```

### 5.7 增量 Delta 统计

```cpp
IncrementalDelta {
    solidToAir, solidToBnd, bndToAir, bndUpdated,
    oldPtsKept, oldPtsPruned
};
```

---

## 6. 八叉树表面提取 (extractSurface)

文件：`core/OctreeRefiner.h` / `OctreeRefiner.cpp`

### 6.1 算法概述

在给定的 world-space AABB 内，从 `ToolSweptSDF` 隐式场中自适应提取零等值面点集。

### 6.2 递归细分

```
extractSurface(bbox, toolSDF, config):
    refineOctant(bbox, depth=0, ...)

refineOctant(bbox, depth, toolSDF, config, output, edgeHash, evalCount):
    1. 求 8 角点 SDF → 8 次 eval
    2. classifyCorners(sdf[8], eps):
         ALL_POS  → 停止 (完全在刀外)
         ALL_NEG  → 停止 (完全在刀内)
         MIXED    → 继续
    3. 终止判断:
         bbox 对角线 ≤ chordalTol  → 停止
         depth ≥ maxDepth          → 强制停止
    4. 弦高验证 (leaf cell only):
         若 |SDF(center)| > 1.2×chordalTol → 曲率大，跳过提取，继续细分
    5. extractLeafPoints: 12 条边零交叉 + 去重 → output
    6. 否则: 分裂为 8 个子格元，递归
```

### 6.3 零交叉点提取

对 leaf cell 的 12 条边，检测跨零（edge 两端 SDF 符号相反），二分搜索找到 $SDF \approx 0$ 的点：

```
findZeroCrossing(a, b, toolSDF, zeroTol):
    确保 sdf(lo) < 0, sdf(hi) > 0
    迭代 16 次二分:
        mid = (lo+hi)/2
        if |mid-lo| < zeroTol: return mid
        sdf(mid) < 0 → lo=mid, else hi=mid
```

### 6.4 边哈希去重

相邻 CE 共享边，同一条边会被两个 CE 的 `extractSurface` 重复计算。使用 `edgeHash`（边两端点坐标的整数哈希）去重。

### 6.5 OctreeConfig 参数

```cpp
struct OctreeConfig {
    double chordalTol   = 0.01;   // 弦高容差 (主终止条件)
    int    maxDepth     = 6;      // 安全阀
    double zeroCrossTol = 0.0;    // 0 = auto (chordalTol/10)
    int    evalCount    = 0;      // [out] 内部统计
};
```

**maxDepth 推导**（在 MicroGridLabState 中）：
$$maxDepth = \left\lceil \log_2\left(\frac{ceSize}{precision}\right) \right\rceil$$

其中 $ceSize = voxelSize / 8$。

### 6.6 输出

```cpp
struct SurfaceSample {
    Vec3d position;  // 世界坐标
    Vec3d normal;    // ∇tool_sdf 方向 (指向刀外)
};
```

---

## 7. 核心类型与接口

### 7.1 MicroGridCell

```cpp
struct MicroGridCell {
    uint64_t activeMask[8];      // 512-bit 窄带标记
    uint64_t signMask[8];        // 512-bit 内部/外部标记
    std::vector<int16_t> sdf;    // 压缩 SDF (仅 BOUNDARY CE)

    // 查询
    int  activeCount() const;    // popcount 总和
    bool isActive(int idx) const;
    bool isInside(int idx) const;
    bool isOutside(int idx) const;
    float getValue(int idx, float bg) const;

    // 构造
    Classification buildFromSDF(const float sdf512[512],
                                 float threshold, float bg);
    CeUpdate updateCeFromToolSDF(int ceIdx, float toolSDF,
                                  float threshold, float bg);

    // 整块操作
    void setAllSolid();   // 全部 CE → 材料内部
    void clear();         // 全部 CE → 空气
};
```

### 7.2 MicroGridLabState

```cpp
struct MicroGridLabState {
    double cubeSize  = 1.0;    // 工作立方体边长 (mm)
    double voxelSize = 0.5;    // 体素尺寸 (mm)
    double precision = 0.01;   // 目标精度 (mm)

    ToolDef     currentTool;    // 当前刀具
    MoveSegment currentSegment; // 当前刀路

    std::vector<MicroGridCell> voxels;          // N³ 个体素
    std::vector<CutRecord>     cutHistory;      // 切削历史 (可 replay)
    std::vector<SurfaceSample> surfacePoints;    // 当前帧表面点

    // —— 方法 ——
    void init();       // 重建网格，不清 cutHistory
    void reset();      // = init()
    void rebuild();    // init() + 重放 cutHistory
    void executeCut(size_t cutIdx);             // 全量切削
    void executeCutIncremental(size_t cutIdx);   // 增量切削
};
```

### 7.3 CutRecord

```cpp
struct CutRecord {
    ToolDef tool;          // 刀具定义
    MoveSegment segment;   // 刀路段
    uint32_t seqIndex;     // 序号 (addCutRecord 时填充)
};
```

### 7.4 ToolSweptSDF（依赖）

```cpp
class ToolSweptSDF {
public:
    ToolSweptSDF(const ToolDef& tool, const MoveSegment& seg);

    double eval(const Vec3d& p) const;      // 有符号距离 (< 0 = 刀内)
    Vec3d  gradient(const Vec3d& p) const;  // 解析梯度
    openvdb::BBoxd boundingBox() const;     // 扫掠体 AABB
    Vec3d  evalSurface(double u, double t) const;  // 参数面求值
};
```

---

## 8. 数学基础

### 8.1 SDF 布尔差集

在 SDF 表示下，布尔差集 $A \setminus B$ 为：

$$\phi_{A \setminus B}(\mathbf{x}) = \max\left(\phi_A(\mathbf{x}),\; -\phi_B(\mathbf{x})\right)$$

MicroGridLab 中更具体的应用：
- $\phi_A$ = IPW 的 SDF（cell 中存的值，正=材料内）
- $-\phi_B$ = 刀具 SDF 取负（使得刀具外为正）

实际实现等价于：$sdf_{new} = \min(sdf_{old},\; toolSdf)$。

这是因为在我们的约定中，**正值 = 在 IPW 内部**，**toolSDF 负值 = 刀具内部**。取 min 意味着：
- 刀具内的 CE（toolSDF < 0）→ 结果变负 → 被切除
- 刀具外的 CE（toolSDF > 0）→ 保持原值 → 不变

### 8.2 窄带半宽

$$bg = 3 \cdot ceSize = 3 \cdot \frac{voxelSize}{8}$$

3 个 CE 的窄带保证梯度计算有足够支撑。

### 8.3 边界阈值

$$threshold = ceSize \cdot \sqrt{3}/2$$

CE 对角线半长。用途：当 $|toolSDF| < threshold$，说明此 CE 中心离刀具面足够近，需要进入 BOUNDARY 状态。

### 8.4 弦高误差

八叉树细分终止后，对每个提取的表面点做 SDF 评估：

$$chordalError = \max_i |toolSDF(\mathbf{p}_i)|$$

这是结果精度的直接度量：理想情况下，每个 $\mathbf{p}_i$ 都应在零等值面上（$SDF \approx 0$）。

### 8.5 SDF 组合定值

增量模式中边界 CE 的 SDF 更新使用 composite min：

$$sdf_{new} = \min(sdf_{old},\; toolSDF)$$

**定理**：复合 min 等价于 CSG 布尔差集的 SDF 表示（在阈值以上的精度内）。

---

## 9. 复杂度与性能

### 9.1 时间复杂度

| 阶段 | 全量模式 | 增量模式 |
|------|----------|----------|
| 粗筛 eval | $O(N_{voxel} \cdot 512)$ | $O(N_{solid\_bnd})$ 远小于 $512 \cdot N$ |
| Cell 重建 | $O(N_{voxel} \cdot 512)$ | $O(N_{changed})$ |
| 旧点验证 | — | $O(N_{old\_points})$ |
| 八叉树精修 | $O(N_{bnd\_ce} \cdot E_{depth})$ | $O(N_{new\_bnd\_ce} \cdot E_{depth})$ |
| 弦高误差 | $O(N_{points})$ | $O(N_{points})$ |

其中 $E_{depth}$ 为八叉树的平均 SDF 评估次数，取决于 surface 曲率，典型值 10~100。

### 9.2 空间复杂度

| 数据 | 大小 |
|------|------|
| 全部 MicroGridCell | $N_{voxel} \cdot [128\ \text{bytes (masks)} + N_{active} \cdot 2\ \text{bytes}]$ |
| cutHistory | $\ll$ 网格数据 |
| surfacePoints | $O(N_{points})$，随表面面积增长 |
| 临时缓冲 (toolVals, updateTags) | $512 \cdot N_{voxel} \cdot (4+4)\ \text{bytes}$ |

### 9.3 增量效率

增量模式的核心收益来自：
1. **跳过空气 CE**：已切掉的 CE 不被 eval
2. **复用旧点**：无需重做已有的八叉树提取
3. **局部精修**：只在状态变化的 BOUNDARY CE 上运行 extractSurface

在典型的多刀路径中，每刀影响的 CE 数通常远小于总量（< 10%），增量模式可提速 5-20×。

### 9.4 TBB 并行策略

```
┌────────────────────────────────────────────┐
│ eval 阶段 (TBB parallel_for)               │
│ 512*N CE → toolSDF.eval()                  │
│ 每个 CE 的 eval 是独立的 (只读)             │
│ 写入 toolVals[flat] (无冲突，独立位置)       │
├────────────────────────────────────────────┤
│ apply 阶段 (串行)                           │
│ MicroGridCell 变长数组操作有内部依赖         │
│ 但只占 ~1% 耗时                             │
├────────────────────────────────────────────┤
│ 旧点验证 (TBB parallel_for + thread_local)  │
│ 每线程独立 kept/pruned 容器                 │
├────────────────────────────────────────────┤
│ 精修提取 (TBB parallel_for + thread_local)  │
│ 每线程独立 SurfaceSample 容器 + edgeHash    │
├────────────────────────────────────────────┤
│ 弦高误差 (TBB parallel_for + reduce)        │
│ thread_local max → 全局 max                │
└────────────────────────────────────────────┘
```

---

## 10. 集成架构与演进路线

### 10.1 当前模块关系

```
┌─────────────────────────────────────────────────┐
│                    midgard-app                   │
│  ┌───────────────────────────────────────────┐  │
│  │         MicroGridLabWindow (UI)            │  │
│  │  编辑参数 → MicroGridLabState → 显示结果    │  │
│  └──────────────┬────────────────────────────┘  │
│                 │                                │
│  ┌──────────────▼────────────────────────────┐  │
│  │         MicroGridLabState                  │  │
│  │  executeCut / executeCutIncremental        │  │
│  └──────┬──────────────┬─────────────────────┘  │
│         │              │                         │
│  ┌──────▼──────┐ ┌─────▼──────────┐             │
│  │ MicroGridCell│ │ OctreeRefiner  │             │
│  │ (SDF grid)  │ │ (extractSurface)│             │
│  └──────┬──────┘ └─────┬──────────┘             │
│         │              │                         │
│  ┌──────▼──────────────▼──────────────────────┐ │
│  │           ToolSweptSDF                     │  │
│  │   eval / gradient / surfaceBBox            │  │
│  └────────────────────────────────────────────┘  │
│                                                   │
│  未来路线:                                        │
│  ┌────────────────────────────────────────────┐  │
│  │  ToolSweepSurface  → MicroCut 重写          │  │
│  │  MacroCut (V4.0)   → 适配 MicroGridCell    │  │
│  │  IPWBuilder        → 以 MicroGridCell 输出 │  │
│  │  渲染管线          → MeshExport + LOD      │  │
│  └────────────────────────────────────────────┘  │
└───────────────────────────────────────────────────┘
```

### 10.2 演进阶段

| 阶段 | 内容 | 状态 |
|------|------|------|
| **M1** | MicroGridCell 基础：mask + SDF 压缩 + buildFromSDF | ✅ 完成 |
| **M2** | executeCut 全量切削管线 | ✅ 完成 |
| **M3** | OctreeRefiner 自适应八叉树表面提取 | ✅ 完成 |
| **M4** | 弦高误差验证 | ✅ 完成 |
| **M5** | executeCutIncremental 增量切削 + TBB 三层并行 | ✅ 完成 |
| **M6** | ToolSweepSurface 参数面 → ToolSweptSDF 替代 | 🔲 待适配 |
| **M7** | MeshExport：从 MicroGridCell 直接生成三角网格 | 🔲 待开发 |
| **M8** | 多尺度 LOD：远离刀具区域降低八叉树精度 | 🔲 待设计 |
| **M9** | MicroGridLab → 替代 V4.0 主流水线 | 🔲 计划中 |

### 10.3 与 V4.0 主流水线的融合方向

MicroGridLab 的 `MicroGridCell` 将成为新的核心数据类型，替代 V4.0 的 `PointDataGrid`：

```
V4.0 架构                    →  融合后架构
═══════════════════            ═══════════════════
MacroGrid (FloatGrid)          MacroGrid (FloatGrid, 保留做粗索引)
MicroGrid (PointDataGrid)      MicroGridCell[] (SDF 压缩网格)
MicroCut (参数面四叉树)          OctreeRefiner (空间八叉树, 统一 surface extraction)
Phase 3 非对称点处理             增量 executeCutIncremental (CE 粒度)
Phase 4 无锁坐标注入             保留 (表面点存为 SurfaceSample 缓存)
```

### 10.4 不变性保证

以下性质在整个演进过程中必须保持：

1. **AIR 不可逆**：已切除的 CE 不会因为任何操作重新变为 SOLID 或 BOUNDARY
2. **SDF 单调性**：在增量模式中，任意 CE 的 SDF 值只减不增（$\min(old, tool)$）
3. **保守边界**：BOUNDARY CE 的判定只覆盖真实切削边界，不允许漏切
4. **表面水密**：extractSurface 的边哈希去重保证相邻 CE 的零交叉边只采样一次

---

## 附录 A: 文件清单

```
core/
├── Types.h               # 基础类型 (GeometryDef, ToolDef, MoveSegment, ToleranceConfig, IPWState, VoxelTask...)
├── ToolSweptSDF.h/cpp    # 刀具扫掠体解析 SDF
├── ToolSweepSurface.h/cpp# 刀具扫掠体参数面(遗留，待被 ToolSweptSDF 替代)
├── MicroGridLab.h/cpp    # 核心: MicroGridCell + MicroGridLabState
├── OctreeRefiner.h/cpp   # 自适应八叉树表面提取
├── IPWBuilder.h/cpp      # IPW0 构建 (遗留，待适配)
├── MacroCut.h/cpp        # CSG 粗筛 + VoxelTask 生成 (遗留，待适配)
├── MicroCut.h/cpp        # 四叉树+点云切削 (遗留，待适配)
└── MeshExport.h/cpp      # 表面网格导出

app/
├── AppState.h            # 应用全局状态 (持有 MicroGridLabState 引用?)
├── main.cpp              # 入口
└── windows/
    └── MicroGridLabWindow.h/cpp  # MicroGridLab UI
```

## 附录 B: 术语表

| 术语 | 缩写 | 定义 |
|------|------|------|
| In-Process Workpiece | IPW | 加工中工件，即切削仿真中的工件状态 |
| Constructive Solid Geometry | CSG | 构造实体几何，布尔运算框架 |
| Signed Distance Field | SDF | 有符号距离场 |
| Corner Element | CE | 网格单元角点，MicroGridCell 的最小数据单元 (8×8×8 = 512/voxel) |
| Boundary | BND | 边界 CE，SDF 绝对值 < threshold，需要精确表示 |
| Narrowband | — | 窄带，CE 中 SDF 绝对值 < bg 的集合 |
| Chordal Error | — | 弦高误差，离散采样与连续曲面的最大偏差 |
| Octree | — | 八叉树，3D 自适应空间细分结构 |

---

**文档版本**: v1.0
**最后更新**: 2026-07-01
**作者**: Loop Engineering Algorithm Designer
**状态**: 活跃开发中
