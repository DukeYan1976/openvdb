# 第二阶段详细设计：DualGrid 双轨高精度切削

**文档编号**：`Phase2_DualGrid_Design_20260602_v0.c.md`
**状态**：修订稿
**作者**：Duke / Kiro
**日期**：2026-06-02
**前置**：第一阶段 MVP 完成（单轨验证通过）

---

## 1. 目标

在单轨方案内存不可行的场景下（大件+高精度），实现 DualGrid 双轨架构：
- FloatGrid(D_v) 做宏观快速过滤
- PointDataGrid(D_v) 做微观精确切削（面元精度 d_v）
- 全程监控两个 Grid 的内存使用

**验收标准**：
- 对于 500×500×200mm 毛坯 + t=0.01mm 精度，单轨需要 ~864GB 不可行
- 双轨 D_v=0.64mm 时，FloatGrid 内存 < 100MB，PointDataGrid 按需增长
- 切削后新表面精度达到 d_v=0.005mm 级别

---

## 2. 架构回顾（严格遵循原设计）

### 2.1 共享 Transform

两个 Grid 共享同一个 Transform（VoxelSize = D_v）
同一个索引 (i,j,k)：
  FloatGrid[i,j,k]     → SDF 标量值（精度 ~D_v）
  PointDataGrid[i,j,k] → 该体素内的面元点集合（精度 ~d_v）

### 2.2 面元密度

N = D_v / d_v = 面元密度因子
每个表面体素内约 N² 个面元（表面采样）
IPW₀ 初始面元间距 = d_v_init ≥ 10×d_v（粗精度）
切削后新表面面元间距 = d_v（仿真精度）

### 2.3 遍历规则

以 FloatGrid 为主树遍历
PointDataGrid 为从属树，按需触发延迟注入
不存在"PointDataGrid 有数据但 FloatGrid 无对应体素"的情况

---

## 3. 数据结构设计

### 3.1 PointDataGrid 属性布局

```cpp
struct SurfelAttributes {
    Vec3f position;    // 相对于体素中心的局部偏移（精度 d_v）
    Vec3f normal;      // 表面法向量
    uint8_t active;    // 1=材料存在, 0=已切除
    uint8_t precision; // 0=COARSE(IPW₀), 1=FINE(切削后)
};
```

### 3.2 内存监控结构

```cpp
struct MemoryStats {
    size_t floatGridBytes;
    size_t pointGridBytes;
    size_t totalBytes;
    size_t pointCount;
    size_t activePointCount;
    size_t leafNodeCount;
    void update(const FloatGrid::Ptr& sdf, const PointDataGrid::Ptr& pts);
    void print() const;
};
```

---

## 4. 模块设计

### 4.1 DualGridBuilder（扩展 BilletBuilder）

输入: ResolutionConfig(mode=DUAL_TRACK), origin, dims
输出: BilletModel { sdfGrid(D_v), microGrid(D_v), ... }

流程:
  1. 创建共享 Transform(D_v)
  2. 构建 FloatGrid SDF（长方体窄带，复用第一阶段代码）
  3. 创建 PointDataGrid（共享 Transform）
  4. 注册属性描述符（position, normal, active, precision）
  5. 为窄带表面体素注入 IPW₀ 粗面元（d_v_init = max(10*d_v, D_v)）
  6. 记录初始内存

### 4.2 IPW₀ 面元注入

对每个表面体素 (i,j,k)（|SDF| < halfWidth * D_v 且 SDF 零交叉）:
  1. 确定表面定位策略（见 4.2.1）
  2. 在体素表面区域按 d_v_init 间距生成 N_init² 个面元
     N_init = max(2, floor(D_v / d_v_init))
  3. 面元位置通过精确表面定位获得（见 4.2.1 分类策略）
  4. 标记 precision = COARSE

#### 4.2.1 面元精度保障机制

**核心问题**：FloatGrid 的 SDF 零等值面精度仅为 ~D_v 级别。若面元直接投影到粗 SDF 零等值面，则面元位置误差量级为 D_v，远不满足 d_v 精度要求。

**设计原则**：粗 SDF 仅用于筛选"哪些体素需要注入面元"（拓扑定位），面元的精确位置和法线必须从原始几何获取。

**按几何来源分类处理**：

| 几何来源 | 定位策略 | 法线获取 | 精度 |
|----------|----------|----------|------|
| 解析几何（长方体毛坯） | 解析投影：直接用平面方程求精确交点 | 解析法线（轴对齐±X/±Y/±Z） | 精确（零误差） |
| 解析几何（圆柱/球体毛坯） | 解析投影：用几何方程求精确交点 | 解析法线（径向/球面方向） | 精确（零误差） |
| Mesh 导入（STL/OBJ 自由曲面） | 射线求交：沿粗 SDF 梯度方向发射射线，与原始 Mesh BVH 求交 | 从 Mesh 三角面片插值法线 | d_v 级别 |
| 切削后新表面 | 刀具解析求交：用刀具扫掠面几何方程精确求交 | 刀具几何解析法线 | 精确（零误差） |

**射线求交流程（Mesh 导入场景）**：

```
function locateSurfelOnMesh(voxelCenter, sdfGradient, meshBVH):
    rayOrigin = voxelCenter
    rayDir = -normalize(sdfGradient)
    hit = meshBVH.raycast(rayOrigin, rayDir, maxDist = D_v * 2)
    if hit.valid:
        return (hit.point, hit.interpNormal)
    else:
        fallbackPos = voxelCenter - sdf * normalize(sdfGradient)
        LOG_WARN("Mesh raycast miss, fallback to SDF projection")
        return (fallbackPos, normalize(sdfGradient))
```

**解析投影流程（长方体毛坯）**：

```
function locateSurfelOnBox(candidatePoint, boxOrigin, boxDims):
    closestFace = findClosestFace(candidatePoint, boxOrigin, boxDims)
    surfelPos = projectToFace(candidatePoint, closestFace)
    surfelNormal = closestFace.outwardNormal
    return (surfelPos, surfelNormal)
```

**刀具表面解析投影（切削后新表面）**：

```
function locateSurfelOnToolSurface(candidatePoint, toolGeometry):
    surfelPos = toolGeometry.closestPointOnSurface(candidatePoint)
    surfelNormal = toolGeometry.normalAt(surfelPos)
    return (surfelPos, surfelNormal)
```

#### 4.2.2 几何来源管理

```cpp
struct GeometrySource {
    enum Type { ANALYTIC_BOX, ANALYTIC_CYLINDER, ANALYTIC_SPHERE, MESH };
    Type type;
    struct AnalyticBox { Vec3d origin; Vec3d dims; };
    struct AnalyticCylinder { Vec3d center; double radius; double height; Vec3d axis; };
    struct AnalyticSphere { Vec3d center; double radius; };
    struct MeshGeometry {
        std::shared_ptr<TriangleMesh> mesh;
        std::shared_ptr<BVHAccel> bvh;
    };
    static GeometrySource fromBox(Vec3d origin, Vec3d dims);
    static GeometrySource fromMesh(const std::string& filePath);
};

struct BilletModel {
    FloatGrid::Ptr sdfGrid;
    PointDataGrid::Ptr microGrid;
    GeometrySource geometrySource;  // 保留原始几何引用
};
```

#### 4.2.3 设计决策与权衡

| 决策 | 选择 | 理由 |
|------|------|------|
| Mesh BVH 常驻内存？ | 是（直到 IPW₀ 注入完成后释放） | 批量操作高效；完成后可释放 |
| 延迟注入时 BVH 是否仍需？ | 仅表面体素需要；内部体素用刀具解析几何 | 内部体素新表面来自刀具 |
| 射线 miss 时的回退？ | 降级到粗 SDF 投影 + 日志警告 | 不阻塞流程 |
| 解析几何优先？ | 是 | 零误差、零额外内存、零查询开销 |

### 4.3 DualGridCutter — 布尔减详细算法

#### 4.3.0 概述与物理语义

DualGridCutter 实现**刀具扫掠体与毛坯的布尔减**运算：从毛坯中减去刀具经过的空间，更新残余材料的表面表示。

在 DualGrid 架构下，布尔减分两层协作完成：
- **FloatGrid（宏观层）**：快速判定哪些体素可能受影响（拓扑筛选）
- **PointDataGrid（微观层）**：逐面元精确判定材料剥离（几何精度 d_v）

**DualGrid 不变量**（任何操作后必须满足）：
```
INV-1: 若体素有活跃面元 → FloatGrid[i,j,k] ≤ 0（有材料）
INV-2: 若 FloatGrid[i,j,k] > 0（空气）→ 该体素无活跃面元
INV-3: 不存在 PointDataGrid 有活跃面元但 FloatGrid 标记空气的矛盾
```

**接口定义**：
```
输入:
  - BilletModel(DUAL_TRACK): { sdfGrid(D_v), microGrid(D_v), geometrySource }
  - ToolSweepSDF: 刀具扫掠体的符号距离场（≤0 = 刀具占据区域）
  - toolGeometry: 刀具扫掠面的解析几何描述（用于精确面元定位）

输出:
  - 就地更新 BilletModel（面元剥离/注入、SDF 更新、内存回收）
```

#### 4.3.1 Phase 1 — 宏观候选筛选

**目的**：从数百万体素中快速缩小到只有可能受刀具影响的少量体素。

```
function macroFilter(sdfGrid, toolSweepSDF) → affectedVoxels[]:

    toolBBox = toolSweepSDF.boundingBox()   // 刀具扫掠体的 AABB

    affectedVoxels = []
    for each activeVoxel (i,j,k) in sdfGrid where sdfGrid[i,j,k] <= 0:
        // sdfGrid <= 0 表示"有材料"（表面体素或内部体素）
        voxelWorldPos = transform.indexToWorld(i,j,k)

        if not toolBBox.contains(voxelWorldPos):
            continue  // 快速排除：不在刀具 BBox 内

        // 进一步用 toolSweepSDF 在体素中心采样
        toolDist = toolSweepSDF.eval(voxelWorldPos)
        if toolDist > D_v * 1.5:
            continue  // 保守排除：刀具表面距体素中心超过 1.5 个体素宽度

        affectedVoxels.append( (i,j,k) )

    return affectedVoxels
```

**设计要点**：
- `sdfGrid <= 0` 涵盖表面体素（窄带内）和内部体素（深负值）
- `toolDist > D_v * 1.5` 是保守阈值：体素角点可能比中心更接近刀具
- 此阶段纯粗筛，不修改任何数据

#### 4.3.2 Phase 2 — 体素分类与面元准备

对 affectedVoxels 中的每个体素，判定其输入状态：

```
enum VoxelInputState { HAS_SURFELS, SURFACE_EMPTY_DEFENSIVE, INTERIOR };

function classifyVoxel(voxelCoord, sdfGrid, microGrid):

    hasSurfels = microGrid.hasPointsAt(voxelCoord)
    sdfVal = sdfGrid.getValue(voxelCoord)

    if hasSurfels:
        // ═══ 状态 A：已有面元（正常路径）═══
        // 来源：IPW₀ 初始化注入，或上一次切削产生的 FINE 面元
        return HAS_SURFELS

    elif |sdfVal| < halfWidth * D_v:
        // ═══ 状态 B：表面体素但无面元 ═══
        // ⚠️ 【防御性设计】此分支在正常流程中不应触发。
        //    理由：DualGridBuilder（4.1 步骤 5）保证所有窄带表面体素
        //    在初始化时都已注入 IPW₀ 粗面元。
        //    若执行到此处，说明上游存在 bug（如 IPW₀ 遍历遗漏、
        //    多线程写入丢失、或窄带宽度参数不一致）。
        //    处理策略：补注入 + 报错误日志，不 crash。
        LOG_ERROR("DEFENSIVE: surface voxel (%d,%d,%d) has no surfels. "
                  "IPW0 injection may have a bug. Auto-recovering...",
                  voxelCoord.x, voxelCoord.y, voxelCoord.z)
        injectSurfels_IPW0(microGrid, voxelCoord, geometrySource, config)
        return SURFACE_EMPTY_DEFENSIVE

    else:
        // ═══ 状态 C：内部体素（首次被刀具穿透）═══
        // sdfVal << -halfWidth * D_v，深内部，从未暴露过表面
        return INTERIOR
```

#### 4.3.3 Phase 3 — 微观切削（逐面元布尔减）

对每个有面元的体素（状态 A，或状态 B 防御性补注入后），逐面元判定材料剥离：

```
struct CutResult {
    int totalActive;       // 切削前活跃面元数
    int removedCount;      // 本次被剥离的面元数
    int survivedCount;     // 存活面元数
    float minToolDist;     // 存活面元中距刀具最近距离
    bool hasBoundary;      // 是否产生切削边界（部分切削）
};

function microCut(microGrid, voxelCoord, toolSweepSDF) → CutResult:

    result = CutResult{0, 0, 0, +INF, false}
    surfels = microGrid.getPoints(voxelCoord)

    for each surfel in surfels:
        if surfel.active == 0:
            continue  // 已失活（之前切削的），跳过

        result.totalActive++
        worldPos = voxelCenter(voxelCoord) + surfel.position
        toolDist = toolSweepSDF.eval(worldPos)

        if toolDist <= 0:
            // 面元位于刀具占据区域 → 材料被切除
            surfel.active = 0
            result.removedCount++
        else:
            // 面元存活
            result.survivedCount++
            result.minToolDist = min(result.minToolDist, toolDist)

    // 判定切削边界：有面元被切且有面元存活 → 刀具表面穿过此体素
    result.hasBoundary = (result.removedCount > 0 && result.survivedCount > 0)
    return result
```

#### 4.3.4 Phase 3b — 内部体素快速判定

对 INTERIOR 类型体素（无面元，SDF 深负值），无需逐面元处理，直接判定刀具穿透程度：

```
enum InteriorOutcome { FULLY_CUT, UNTOUCHED, BOUNDARY_NEW_SURFACE };

function handleInteriorVoxel(voxelCoord, toolSweepSDF) → InteriorOutcome:

    // 在体素 8 个角点 + 中心共 9 个采样点评估 toolSweepSDF
    allInside = true   // 全部在刀具内
    allOutside = true  // 全部在刀具外
    for each samplePos in voxelSamples9(voxelCoord):
        d = toolSweepSDF.eval(samplePos)
        if d > 0: allInside = false
        if d <= 0: allOutside = false

    if allInside:
        // 刀具完全吞没此体素 → 材料整体消失，无新表面产生
        return FULLY_CUT
    elif allOutside:
        // 刀具未触及（宏观筛选的保守误报）
        return UNTOUCHED
    else:
        // 刀具部分穿过此体素 → 产生新的切削表面
        return BOUNDARY_NEW_SURFACE
```

#### 4.3.5 Phase 4 — 切削边界面元注入

对所有产生切削边界的体素（Phase 3 的 hasBoundary=true + Phase 3b 的 BOUNDARY_NEW_SURFACE），在刀具零等值面上注入高精度面元：

```
function injectBoundarySurfels(microGrid, voxelCoord, toolSweepSDF,
                                toolGeometry, sdfGrid, config) → int:

    voxelCenter = transform.indexToWorld(voxelCoord)
    d_v = config.fineSpacing            // 精细面元间距
    N_fine = max(2, floor(D_v / d_v))   // 每方向精细采样数

    //以下算法做了重新设计， 这个算法计算量太重，考虑对toolSweepSDF做预计算，构建一个与toolSweepSDF共享生存期的Openvdb  DualGrid结构来代表刀具扫掠体的精确模型。 

    
    // 1. 在体素内生成均匀候选网格（间距 d_v）
    candidates = generateUniformGrid3D(voxelCenter, D_v, N_fine)

    injectedCount = 0
    for each candidatePos in candidates:
        toolDist = toolSweepSDF.eval(candidatePos)

        // 只关注刀具零等值面附近的候选点
        if |toolDist| > d_v * 2:
            continue

        // 2. 精确投影到刀具表面（解析几何，零误差）
        surfelPos = toolGeometry.closestPointOnSurface(candidatePos)
        surfelNormal = toolGeometry.normalAt(surfelPos)

        // 3. 验证投影点仍在当前体素范围内
        if not isInsideVoxel(surfelPos, voxelCoord, D_v):
            continue  // 归属相邻体素，由相邻体素处理

        // 4. 材料侧验证：确认此处确实是"从材料侧暴露"的表面
        //    沿法线反方向（向材料内部）微偏，应仍有材料（SDF ≤ 0）
        checkPos = surfelPos - surfelNormal * d_v * 0.5
        if sdfGrid.eval(checkPos) > 0:
            continue  // 空气侧，不是有效切削面

        // 5. 注入精细面元
        surfel = Surfel {
            position: surfelPos - voxelCenter,   // 局部偏移
            normal:   surfelNormal,               // 刀具表面外法线
            active:   1,
            precision: FINE
        }
        microGrid.addPoint(voxelCoord, surfel)
        injectedCount++

    return injectedCount
```

**关键设计点**：
- 候选网格间距为 d_v → 新表面面元密度达到仿真精度要求
- 投影到刀具解析几何（非粗 SDF）→ 位置精度无损
- 材料侧验证 → 防止在已是空气的区域误注入
- 体素边界检查 → 保证面元归属正确，不重复注入

#### 4.3.6 Phase 5 — DualGrid 对齐与后处理

根据各体素的切削结果，维护 DualGrid 不变量：

```
function postProcess(voxelCoord, inputState, cutResult, interiorOutcome,
                     sdfGrid, microGrid):

    // ─── 结果①：体素完全切除（所有面元被剥离）───
    if inputState != INTERIOR && cutResult.totalActive > 0 && cutResult.survivedCount == 0:
        microGrid.removeAllPoints(voxelCoord)    // 回收面元内存
        sdfGrid.setValue(voxelCoord, +D_v)        // 标记为空气
        return VOXEL_REMOVED
        // 维护 INV-2：空气体素无面元 ✓

    // ─── 结果④：内部体素被刀具完全吞没 ───
    if inputState == INTERIOR && interiorOutcome == FULLY_CUT:
        sdfGrid.setValue(voxelCoord, +D_v)        // 直接标记为空气
        return VOXEL_REMOVED
        // 无面元需要回收（内部体素本就没有面元）

    // ─── 结果②：部分切削，产生新边界 ───
    if cutResult.hasBoundary:
        // Phase 4 已注入边界面元
        // SDF 保持 ≤ 0（体素仍有材料）→ INV-1 满足
        return VOXEL_PARTIAL_CUT

    // ─── 结果②'：内部体素产生新表面 ───
    if inputState == INTERIOR && interiorOutcome == BOUNDARY_NEW_SURFACE:
        // Phase 4 已注入精细面元
        // 更新 SDF：从深内部转为窄带表面
        sdfGrid.setValue(voxelCoord, -halfWidth * D_v * 0.5)
        return VOXEL_NEW_SURFACE
        // INV-1：有面元且 SDF ≤ 0 ✓

    // ─── 结果③：未受影响 ───
    return VOXEL_UNCHANGED
```

#### 4.3.7 Phase 6 — 邻域拓扑修复

体素被完全切除后，其相邻内部体素可能变成新的表面体素：

```
function repairNeighborhood(removedVoxels[], sdfGrid, microGrid):

    for each removed in removedVoxels:
        for each neighbor in get6FaceNeighbors(removed):
            sdfVal = sdfGrid.getValue(neighbor)
            if sdfVal >= 0:
                continue  // 已经是空气

            // 检查：该邻居是否因相邻体素被移除而成为新表面？
            hasAirNeighbor = any(get6FaceNeighbors(neighbor), v -> sdfGrid[v] > 0)
            if hasAirNeighbor && !microGrid.hasPointsAt(neighbor):
                // 新暴露的表面体素，标记为待注入
                // （下次切削时延迟注入，或此处立即注入取决于策略）
                sdfGrid.setValue(neighbor, -halfWidth * D_v * 0.9)
                // 注：此处不立即注入面元，因为该体素可能在后续切削中
                // 被进一步切除。延迟到实际需要时再注入（4.4 延迟注入）
```

#### 4.3.8 Phase 7 — 内存回收与统计

```
function finalizePass(microGrid, sdfGrid, memStats):

    // 1. 紧凑化：释放空叶节点
    microGrid.compactEmptyLeaves()

    // 2. FloatGrid 修剪
    sdfGrid.pruneInactive()

    // 3. 更新内存统计
    memStats.update(sdfGrid, microGrid)
    memStats.printDelta()
```

#### 4.3.9 完整流程总览

```
function DualGridCutter.cut(billetModel, toolSweepSDF, toolGeometry):

    sdfGrid   = billetModel.sdfGrid
    microGrid = billetModel.microGrid
    geoSrc    = billetModel.geometrySource

    // ═══ Phase 1: 宏观筛选 ═══
    affectedVoxels = macroFilter(sdfGrid, toolSweepSDF)
    LOG("[Cut] Phase1: %d candidate voxels", affectedVoxels.size())

    removedVoxels = []
    boundaryCount = 0

    for each voxelCoord in affectedVoxels:

        // ═══ Phase 2: 分类 ═══
        state = classifyVoxel(voxelCoord, sdfGrid, microGrid)

        // ═══ 内部体素路径（Phase 3b）═══
        if state == INTERIOR:
            outcome = handleInteriorVoxel(voxelCoord, toolSweepSDF)
            if outcome == FULLY_CUT:
                sdfGrid.setValue(voxelCoord, +D_v)
                removedVoxels.append(voxelCoord)
            elif outcome == BOUNDARY_NEW_SURFACE:
                injectBoundarySurfels(microGrid, voxelCoord,
                                      toolSweepSDF, toolGeometry, sdfGrid, config)
                sdfGrid.setValue(voxelCoord, -halfWidth * D_v * 0.5)
                boundaryCount++
            // outcome == UNTOUCHED → 什么都不做
            continue

        // ═══ 有面元体素路径（Phase 3 → Phase 4 → Phase 5）═══
        cutResult = microCut(microGrid, voxelCoord, toolSweepSDF)

        if cutResult.hasBoundary:
            injectBoundarySurfels(microGrid, voxelCoord,
                                  toolSweepSDF, toolGeometry, sdfGrid, config)
            boundaryCount++

        outcome = postProcess(voxelCoord, state, cutResult, NONE, sdfGrid, microGrid)
        if outcome == VOXEL_REMOVED:
            removedVoxels.append(voxelCoord)

    // ═══ Phase 6: 邻域修复 ═══
    repairNeighborhood(removedVoxels, sdfGrid, microGrid)

    // ═══ Phase 7: 内存回收 ═══
    finalizePass(microGrid, sdfGrid, billetModel.memStats)

    LOG("[Cut] Done: removed=%d, boundary_injections=%d",
        removedVoxels.size(), boundaryCount)
```

#### 4.3.10 算法正确性论证

| 性质 | 保证机制 |
|------|----------|
| **完备性** | 宏观筛选使用 1.5×D_v 保守阈值；内部体素 9 点采样覆盖全域 |
| **精度** | 面元剥离使用 toolSweepSDF 精确评估；新面元使用刀具解析几何定位（零误差） |
| **一致性** | 每次操作后 DualGrid 不变量（INV-1~3）均被显式维护 |
| **保守性** | 有存活面元则不改 SDF；材料侧验证防止误注入 |
| **防御性** | 状态 B 分支捕获上游 bug 并自愈，不中断流程 |

### 4.4 延迟注入触发

```
function injectOnDemand(microGrid, voxelCoord, sdfGrid, geometrySource, toolGeometry, config):
    // 该体素首次被刀具接触，需要生成面元

    sdfVal = sdfGrid.getValue(voxelCoord)
    if |sdfVal| < halfWidth * D_v:
        // 表面体素：应该在初始化时已注入（IPW₀）
        // ⚠️ 【防御性】若触发此路径，说明 IPW₀ 有遗漏
        LOG_ERROR("DEFENSIVE: delayed injection for surface voxel - IPW0 bug?")
        injectSurfels_IPW0(microGrid, voxelCoord, geometrySource, config)
    else:
        // 内部体素首次暴露：新表面来自刀具，用刀具解析几何注入精细面元
        injectSurfels_Cut(microGrid, voxelCoord, toolGeometry, config)
```

---

## 5. 精度验证方案

### 5.1 面元级精度

```
for each active surfel:
    dist_to_tool_surface = |toolSDF.eval(surfel.position)|
    assert dist_to_tool_surface < d_v  // 面元精度
```

### 5.2 与单轨对比

同一切削场景，分别用:
  - 单轨 FloatGrid(d_v=0.005mm) → 作为 ground truth（如果内存允许小件）
  - 双轨 FloatGrid(D_v=0.64mm) + PointDataGrid
比较两者的切削表面偏差

### 5.3 面元定位精度验证

```
for each COARSE surfel:
    if geometrySource.type == MESH:
        dist = meshBVH.closestDistance(surfel.position)
        assert dist < d_v_init * 0.01
    elif geometrySource.type == ANALYTIC_BOX:
        dist = analyticDistanceToBox(surfel.position, box)
        assert dist < 1e-10  // 浮点精度
```

### 5.4 DualGrid 不变量验证

```
function verifyInvariants(sdfGrid, microGrid):
    for each voxel with active surfels:
        assert sdfGrid[voxel] <= 0    // INV-1
    for each voxel where sdfGrid > 0:
        assert !microGrid.hasActivePointsAt(voxel)  // INV-2
```

---

## 6. 内存监控要求

每次切削操作后输出：

```
[MemStats] FloatGrid: 45.2 MB | PointGrid: 12.8 MB | Total: 58.0 MB
[MemStats] Points: 1,234,567 (active: 987,654) | Leaves: 3,456
[MemStats] Delta: +2.1 MB (inject: +3.5 MB, prune: -1.4 MB)
[MemStats] GeometrySource: BVH 8.3 MB (released: no)
```

---

## 7. 潜在风险点

【风险 1】：PointDataGrid 动态点注入的内存碎片。
每次注入新面元需要重新分配叶节点内存。高频切削时可能产生碎片。

缓解：
  1. 第二阶段先验证正确性，不追求性能
  2. 预分配策略：初始化时为所有窄带体素预留面元容量
  3. 后续引入 RCU 紧凑化

【风险 2】：Mesh BVH 常驻内存开销。
大型 STL 模型（百万三角面）的 BVH 可能占用数十 MB。

缓解：
  1. IPW₀ 批量注入完成后立即释放 BVH
  2. 内部体素新表面来自刀具解析几何，不需要 BVH
  3. 若需重建 BVH，采用惰性重建策略

【风险 3】：边界面元注入的候选网格过密。
N_fine = D_v / d_v 可能很大（如 0.64/0.005 = 128），候选点 128³ ≈ 200万/体素。

缓解：
  1. 先用 toolSweepSDF 快速排除远离零等值面的候选点（|toolDist| > 2*d_v）
  2. 实际注入面元数远小于候选数（仅零等值面附近，约 N² 量级）
  3. 可改为 2D 参数化采样（沿刀具表面的局部 UV 展开），避免 3D 全域搜索

---

## 修订记录

| 版本 | 日期 | 修订内容 |
|------|------|----------|
| v0.a | 2026-06-01 | 初稿创建 |
| v0.b | 2026-06-02 | 补充 4.2.1 面元精度保障机制；新增几何来源管理；设计决策权衡表 |
| v0.c | 2026-06-02 | 重写 4.3 DualGridCutter 为完整 7-Phase 算法；明确体素输入/输出状态分类；状态 B 标注为防御性设计；新增 DualGrid 不变量定义与验证；新增风险 3（候选网格过密） |
