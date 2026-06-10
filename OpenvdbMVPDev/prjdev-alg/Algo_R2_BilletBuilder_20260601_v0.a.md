# R2 算法设计：建立初始毛坯计算模型

**文档编号**：`Algo_R2_BilletBuilder_20260601_v0.a.md`  
**状态**：v0.b  
**作者**：Duke / Kiro  
**日期**：2026-06-01  
**前置依赖**：R1（ResolutionConfig）  
**对应里程碑**：M2  

---

## 1. 问题定义

给定毛坯几何描述（第一阶段仅支持长方体）和 R1 输出的分辨率配置，构建切削计算模型。根据 R1 的模式判定结果，选择不同的构建策略：

| 模式 | 构建内容 | 切削方式 |
|------|----------|----------|
| **SINGLE_TRACK** | 单个 FloatGrid SDF（VoxelSize = d_v） | 直接 SDF 布尔差集 |
| **DUAL_TRACK** | 宏观 FloatGrid(D_v) + 微观 PointDataGrid(d_v) | 宏观过滤 + 微观掩码剔除 |
| **ATLAS_REGION** | 多区域 DUAL_TRACK（第一阶段不实现） | — |

**第一阶段核心目标**：完整实现 SINGLE_TRACK 路径，DUAL_TRACK 作为可选扩展。

---

## 2. 输入

| 参数 | 类型 | 含义 |
|------|------|------|
| `config` | ResolutionConfig | R1 输出（含 mode, d_v, D_v, N） |
| `origin` | Vec3d | 毛坯包围盒最小角点 (世界坐标, mm) |
| `dimensions` | Vec3d | 毛坯尺寸 (Lx, Ly, Lz, mm) |

---

## 3. SINGLE_TRACK 路径（主路径）

### 3.1 原理

当 R1 判定单轨可行时，直接使用 `FloatGrid`（或 `DoubleGrid`）以 `d_v` 为 VoxelSize 构建窄带 SDF。精度由 VoxelSize 天然保证，无需额外的微观轨。

切削时使用 `openvdb::tools::csgDifferenceSDF` 直接执行布尔差集——简单、直接、OpenVDB 原生支持。

### 3.2 构建算法

```
function buildBillet_SingleTrack(config, origin, dimensions) -> FloatGrid:
    voxelSize = config.d_v
    halfWidth = 3  // 窄带半宽（体素数）
    
    // 创建 Transform
    xform = Transform::createLinearTransform(voxelSize)
    
    // 构造世界坐标下的 BBox
    bbox = BBoxd(origin, origin + dimensions)
    
    // 使用 OpenVDB 内置工具生成长方体 Level Set
    grid = tools::createLevelSetBox<FloatGrid>(bbox, *xform, halfWidth)
    
    // 设置网格元数据
    grid->setName("billet")
    grid->setGridClass(GRID_LEVEL_SET)
    
    return grid
```

### 3.3 SDF 语义约定

```
SDF(p) < 0  → 点 p 在毛坯内部（实体材料）
SDF(p) = 0  → 点 p 在毛坯表面
SDF(p) > 0  → 点 p 在毛坯外部（空气）
```

### 3.4 关键属性

| 属性 | 值 |
|------|-----|
| Grid class | `GRID_LEVEL_SET` |
| VoxelSize | d_v (mm) |
| Background value | +halfWidth × d_v |
| 内部 Tile 值 | -halfWidth × d_v |
| 活跃体素 | 仅窄带区域（表面 ± halfWidth 层） |

### 3.5 内存估算

对于长方体毛坯，窄带 SDF 的活跃体素数约为：
```
N_active ≈ 表面积 / d_v² × (2 × halfWidth)
         = [2(LxLy + LyLz + LxLz)] / d_v² × 6
```

**示例**：300×300×100mm, d_v=0.5mm  
→ 表面积 = 2×(90000+30000+30000) = 300,000 mm²  
→ N_active ≈ 300000 / 0.25 × 6 = 7,200,000 体素  
→ 内存 ≈ 7.2M × 4B = ~29MB ✓ 完全可行

---

## 4. DUAL_TRACK 路径（扩展路径）

### 4.1 触发条件

仅当 `config.mode == DUAL_TRACK` 时执行。此时单轨（FloatGrid + d_v）内存不可行，需要用更大的 VoxelSize(D_v) 构建树结构，通过体素内面元点云补偿精度。

### 4.2 核心架构：共享 Transform

**FloatGrid 和 PointDataGrid 共享同一个 Transform（VoxelSize = D_v）。**

两棵树拥有相同的索引空间和相同的叶节点拓扑。同一个索引 (i,j,k)：
- 在 FloatGrid 中 → 返回该体素的 SDF 值（精度 ~D_v）
- 在 PointDataGrid 中 → 返回该体素内的面元点集合（精度可变）

### 4.3 IPW₀ 的设计哲学

毛坯是一个**动态概念**：

```
IPW₀ = 初始毛坯（未经切削）
IPWₙ = 第 n 次切削后的在制品
IPWₙ 就是第 n+1 次切削的"毛坯"
```

**关键洞察**：IPW₀ 的原始表面最终会被切削掉。只有切削后**新暴露的表面**才承载最终零件的精度要求。因此：

| 表面类型 | 来源 | 所需精度 | 面元间距 |
|----------|------|----------|----------|
| 原始表面（IPW₀） | 初始化构建 | 低（会被切掉） | d_v_init ≥ 10 × d_v |
| 切削表面 | 刀具扫掠后新暴露 | 高（仿真精度） | d_v |

这样设计的收益：
- IPW₀ 初始化内存开销降低 100 倍（面元数 ∝ 1/间距²）
- 仿真精度集中在真正需要的地方（切削后表面）

### 4.4 构建算法

```
function buildBillet_DualTrack(config, origin, dimensions) -> BilletModel:
    D_v = config.D_v
    d_v = config.d_v
    halfWidth = 3
    
    // 初始面元间距：粗精度（≥ 10 倍仿真精度）
    d_v_init = max(10 * d_v, D_v)  // 不超过体素尺寸
    N_init = floor(D_v / d_v_init)  // 初始每边面元数（可能仅 1）
    
    // 1. 创建共享 Transform
    sharedXform = Transform::createLinearTransform(D_v)
    
    // 2. 构建 FloatGrid SDF（窄带）
    bbox = BBoxd(origin, origin + dimensions)
    sdfGrid = tools::createLevelSetBox<FloatGrid>(bbox, *sharedXform, halfWidth)
    
    // 3. 构建 PointDataGrid（共享同一 Transform）
    microGrid = PointDataGrid::create()
    microGrid->setTransform(sharedXform)  // 关键：同一个 Transform
    
    // 4. 为窄带表面体素注入粗精度面元（IPW₀ 表面）
    for each leafNode in sdfGrid where |sdfValue| < halfWidth * D_v:
        for each activeVoxel (i,j,k) in leafNode:
            if isOnSurface(sdfGrid, i, j, k):  // SDF 零交叉体素
                injectSurfels_IPW0(microGrid, (i,j,k), d_v_init, sdfGrid)
    
    return { sdfGrid, microGrid, config, origin, dimensions }
```

### 4.5 IPW₀ 粗精度面元注入

```
function injectSurfels_IPW0(microGrid, voxelCoord, d_v_init, sdfGrid):
    D_v = microGrid->transform().voxelSize()
    N_init = max(1, floor(D_v / d_v_init))
    
    // 确定该体素所在的表面法向（从 SDF 梯度估算）
    normal = normalize(gradient(sdfGrid, voxelCoord))
    
    // 在体素表面区域按 d_v_init 间距注入面元
    voxelCenter = microGrid->transform().indexToWorld(voxelCoord)
    
    points = []
    for i in range(N_init):
        for j in range(N_init):
            offset = localSurfaceOffset(i, j, N_init, D_v, normal)
            p = voxelCenter + offset
            points.append({pos: p, normal: normal, active: 1, 
                           precision: COARSE})  // 标记为粗精度
    
    appendPoints(microGrid, voxelCoord, points)
```

### 4.6 切削后精细表面注入（在 R4 中执行）

当刀具切削暴露新表面时，新面元以仿真精度 d_v 注入：

```
function injectSurfels_Cut(microGrid, voxelCoord, config, cutNormal):
    D_v = config.D_v
    d_v = config.d_v
    N = config.N  // 精细面元密度 = D_v / d_v
    
    // 精细采样：每边 N 个面元
    for i in range(N):
        for j in range(N):
            offset = localSurfaceOffset(i, j, N, D_v, cutNormal)
            p = voxelCenter + offset
            points.append({pos: p, normal: cutNormal, active: 1,
                           precision: FINE})  // 标记为精细精度
    
    appendPoints(microGrid, voxelCoord, points)
```

### 4.7 面元属性布局

```
PointDataGrid 每个点的属性：
  - Position (Vec3f): 相对于体素中心的局部偏移
  - Normal (Vec3f): 表面法向量
  - Active (uint8): 1=材料存在, 0=已切除
  - Precision (uint8): COARSE=初始粗面元, FINE=切削后精细面元
```

### 4.8 双轨同步遍历

**遍历规则：以 FloatGrid 为主树。** PointDataGrid 为从属树，按需触发。

```
function cuttingTraversal(sdfGrid, microGrid, toolSDF, config):
    // 主树遍历：FloatGrid 的窄带体素
    for each activeVoxel (i,j,k) in sdfGrid:
        if not toolSDF.getBoundingBox().contains(voxelWorldPos(i,j,k)):
            continue  // 不在刀具范围内，跳过

        // 宏观快速过滤通过，访问从属树
        if not microGrid.hasPoints(i,j,k):
            // 延迟注入触发：该体素首次被刀具接触
            injectSurfels_OnDemand(microGrid, (i,j,k), sdfGrid, config)

        // 微观精确计算
        surfels = microGrid.getPoints(i,j,k)
        for each surfel in surfels:
            if surfel.active == 0:
                continue
            dist = toolSDF.eval(surfel.position)
            if dist <= 0:
                surfel.active = 0  // 位掩码翻转

        // 在切削边界注入精细面元（新暴露表面，N²级采样）
        if hasNewBoundary(surfels, toolSDF):
            injectBoundarySurfels(microGrid, (i,j,k), toolSDF, config)
```

**关键约束**：
- FloatGrid 始终有完整窄带拓扑（初始化时构建），是遍历的唯一入口
- PointDataGrid 可能为空（延迟分配），遍历时检查并按需注入
- 不存在"PointDataGrid 有数据但 FloatGrid 无对应体素"的情况

---

## 5. 统一输出接口

```cpp
struct BilletModel {
    // 单轨模式：sdfGrid 即为唯一计算网格（VoxelSize = d_v）
    // 双轨模式：sdfGrid 为宏观轨（VoxelSize = D_v）
    openvdb::FloatGrid::Ptr sdfGrid;
    
    // 仅 DUAL_TRACK 模式有效，SINGLE_TRACK 为 nullptr
    openvdb::points::PointDataGrid::Ptr microGrid;
    
    ResolutionConfig config;
    openvdb::Vec3d   origin;
    openvdb::Vec3d   dims;
    
    bool isSingleTrack() const { return config.mode == SINGLE_TRACK; }
};
```

### 统一构建入口

```
function buildBillet(config, origin, dimensions) -> BilletModel:
    model.config = config
    model.origin = origin
    model.dims = dimensions
    
    if config.mode == SINGLE_TRACK:
        model.sdfGrid = buildBillet_SingleTrack(config, origin, dimensions)
        model.microGrid = nullptr
    
    else if config.mode == DUAL_TRACK:
        model.sdfGrid = buildMacroSDF(config, origin, dimensions)
        model.microGrid = buildMicroGrid(config, origin)
    
    else:
        error("ATLAS_REGION not implemented in Phase 1")
    
    return model
```

---

## 6. 验收标准

### 6.1 通用验收（所有模式）

| 检查项 | 方法 |
|--------|------|
| SDF 零等值面与毛坯几何一致 | 提取等值面网格，顶点与理论 BBox 偏差 < VoxelSize |
| SDF 符号正确 | 内部采样点 SDF < 0，外部采样点 SDF > 0 |
| 序列化/反序列化 | 写入 .vdb 后重读，数据一致 |
| 可视化 | vdb_view 查看 SDF 切面合理 |

### 6.2 单轨专项

| 检查项 | 方法 |
|--------|------|
| VoxelSize = d_v | `grid->voxelSize()[0] == config.d_v` |
| 内存在预算内 | `grid->memUsage() < MEMORY_BUDGET` |

### 6.3 双轨专项（扩展）

| 检查项 | 方法 |
|--------|------|
| 宏观 VoxelSize = D_v | 验证 Transform |
| 微观 Grid 为空但 Transform 正确 | `microGrid->empty() == true` |
| 对齐关系 | D_v / d_v == N |

---

## 7. 潜在风险点

**【不同见解】**：`createLevelSetBox` 生成的是精确的长方体 SDF，但实际毛坯可能是圆柱体、铸件毛坯（STL 导入）等。第一阶段限定长方体是合理的 MVP 简化，但接口设计应预留扩展点：

```cpp
// 未来扩展：
// - buildBilletFromCylinder(radius, height, ...)
// - buildBilletFromMesh(meshPath, ...)  // 使用 tools::meshToLevelSet
```

建议 `BilletBuilder` 设计为策略模式，几何描述与构建逻辑解耦。

---

## 修订记录

| 版本 | 日期 | 修订内容 |
|------|------|----------|
| v0.a | 2026-06-01 | 初稿创建（以双轨为主路径） |
| v0.b | 2026-06-01 | 重构：SINGLE_TRACK 为主路径；DUAL_TRACK 降为扩展路径 |
| v0.c | 2026-06-01 | 修正双轨架构：共享 Transform(D_v) |
| v0.d | 2026-06-01 | IPW₀ 概念引入：初始表面粗精度，切削后精细 |
| v0.e | 2026-06-01 | Fix-2: 明确双轨遍历规则——以 FloatGrid 为主树，PointDataGrid 延迟注入触发机制 |
