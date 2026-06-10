

#### 4.3.10 完整流程总览

```
function DualGridCutter.cut(billetModel, pathSegment, toolDef, toolTemplate):

    sdfGrid   = billetModel.sdfGrid
    microGrid = billetModel.microGrid
    geoSrc    = billetModel.geometrySource

    // ═══ Phase 0: 构建扫掠体上下文 ═══
    sweepCtx = ToolSweepSDFBuilder.build(toolDef, pathSegment, config)
    LOG("[Cut] Phase0: sweepCtx built (case=%s, time=%dms)",
        sweepCtx.caseType, sweepCtx.buildTimeMs)

    // ═══ Phase 1: 宏观筛选 ═══
    affectedVoxels = macroFilter(sdfGrid, sweepCtx)
    LOG("[Cut] Phase1: %d candidate voxels", affectedVoxels.size())

    removedVoxels = []
    boundaryCount = 0

    for each voxelCoord in affectedVoxels:

        // ═══ Phase 2: 分类 ═══
        state = classifyVoxel(voxelCoord, sdfGrid, microGrid)

        // ═══ 内部体素路径（Phase 3b）═══
        if state == INTERIOR:
            outcome = handleInteriorVoxel(voxelCoord, sweepCtx)
            if outcome == FULLY_CUT:
                sdfGrid.setValue(voxelCoord, +D_v)
                removedVoxels.append(voxelCoord)
            elif outcome == BOUNDARY_NEW_SURFACE:
                // Phase 4: 注入新表面面元
                if pathSegment.is3Axis() || toolDef.isConvex():
                    injectBoundarySurfels_Template(microGrid, [voxelCoord],
                        toolTemplate, pathSegment.endPose, sdfGrid, config)
                else:
                    injectBoundarySurfels_5Axis(microGrid, [voxelCoord],
                        toolTemplate, sweepCtx.poses, sweepCtx, sdfGrid, config)
                sdfGrid.setValue(voxelCoord, -halfWidth * D_v * 0.5)
                boundaryCount++
            continue

        // ═══ 有面元体素路径（Phase 3 → Phase 4 → Phase 5）═══
        cutResult = microCut(microGrid, voxelCoord, sweepCtx)

        if cutResult.hasBoundary:
            if pathSegment.is3Axis() || toolDef.isConvex():
                injectBoundarySurfels_Template(microGrid, [voxelCoord],
                    toolTemplate, pathSegment.endPose, sdfGrid, config)
            else:
                injectBoundarySurfels_5Axis(microGrid, [voxelCoord],
                    toolTemplate, sweepCtx.poses, sweepCtx, sdfGrid, config)
            boundaryCount++

        outcome = postProcess(voxelCoord, state, cutResult, NONE, sdfGrid, microGrid)
        if outcome == VOXEL_REMOVED:
            removedVoxels.append(voxelCoord)

    // ═══ Phase 6: 邻域修复 ═══
    repairNeighborhood(removedVoxels, sdfGrid, microGrid)

    // ═══ Phase 7: 内存回收 ═══
    finalizePass(microGrid, sdfGrid, sweepCtx, billetModel.memStats)

    LOG("[Cut] Done: removed=%d, boundary=%d", removedVoxels.size(), boundaryCount)
```

#### 4.3.11 算法正确性论证

| 性质 | 保证机制 |
|------|----------|
| **完备性** | 宏观筛选 1.5×D_v 保守阈值；内部体素 9 点采样 |
| **精度** | Phase 3 用 preciseEval（解析或 fine Grid）；Phase 4 用模板零误差定位 |
| **一致性** | DualGrid 不变量 INV-1~3 在 Phase 5 显式维护 |
| **保守性** | 有存活面元不改 SDF；材料侧验证防误注入 |
| **防御性** | 状态 B 捕获上游 bug 并自愈 |
| **性能** | Phase 4 用模板方案，加速比 ~2000× vs 原方案 |

### 4.4 延迟注入触发

```
function injectOnDemand(microGrid, voxelCoord, sdfGrid,
                        geometrySource, toolGeometry, config):
    sdfVal = sdfGrid.getValue(voxelCoord)
    if |sdfVal| < halfWidth * D_v:
        // 防御性：表面体素应已有面元
        LOG_ERROR("DEFENSIVE: delayed injection for surface voxel")
        injectSurfels_IPW0(microGrid, voxelCoord, geometrySource, config)
    else:
        // 内部体素首次暴露：用刀具模板注入
        injectSurfels_Cut(microGrid, voxelCoord, toolGeometry, config)
```

### 4.5 ToolSurfelTemplate（刀具面元模板）

**核心思想**：刀具形状在加工过程中不变，其表面面元分布可一次预计算，
运行时通过刚体变换复用。

#### 4.5.1 数据结构

```cpp
struct ToolSurfelTemplate {
    struct LocalSurfel {
        Vec3f localPos;      // 相对于 TCP 的偏移
        Vec3f localNormal;   // 局部外法线
    };
    std::vector<LocalSurfel> surfels;
    float spacing;            // d_v
    AABB localBBox;
    PointDataGrid::Ptr spatialIndex;  // VoxelSize=D_v 空间索引

    std::vector<const LocalSurfel*> queryBox(AABB box) const;
};
```

#### 4.5.2 预计算（刀具加载时，一次性）

```
function buildToolSurfelTemplate(toolDef, d_v, D_v):
    template = ToolSurfelTemplate(spacing = d_v)

    if toolDef.isAnalytic():
        for each (u,v) in toolDef.parameterDomain(step = d_v):
            pos = toolDef.evalUV(u, v)
            nrm = toolDef.normalAtUV(u, v)
            template.surfels.push_back({pos, nrm})
    else:
        toolSDF = buildToolLocalSDF(toolDef, voxelSize = d_v)
        for each narrowBandVoxel in toolSDF:
            grad = toolSDF.gradient(voxel)
            sdfVal = toolSDF.getValue(voxel)
            pos = voxelCenter - sdfVal * normalize(grad)
            nrm = normalize(grad)
            if not template.hasNeighborWithin(pos, d_v * 0.8):
                template.surfels.push_back({pos, nrm})

    template.spatialIndex = buildPointDataGrid(template.surfels, D_v)
    return template
```

#### 4.5.3 生命周期

```
刀具加载 → buildToolSurfelTemplate() → 常驻内存（~30MB）
每个路径段 → 查询 + 变换（只读，不修改模板）
加工结束 → release()
```

#### 4.5.4 性能特征

| 指标 | 数值 |
|------|------|
| 预计算时间 | ~500ms（解析）/ ~2s（非解析） |
| 内存占用 | 30~165MB（取决于有效切削面积） |
| 运行时查询 | O(1) 体素索引 + 线性遍历体素内面元 |
| 每体素面元注入耗时 | ~50us（3轴）/ ~40us（5轴） |

### 4.6 ToolSweepSDF 构建器（摘要）

详细设计见独立文档 `Algo_R5_ToolSweepSDF_Builder_20260602_v0.a.md`。

#### 4.6.1 四种构建策略

| Case | 场景 | 策略 | 构建耗时 | eval 耗时 |
|------|------|------|----------|-----------|
| A | 3轴+解析刀具 | 闭式解析 SDF（胶囊体等） | 0 | ~50ns |
| B | 3轴+非解析刀具 | 预构建 sweepGrid(D_v) | ~300ms | ~50ns |
| C | 5轴+解析刀具 | 离散位姿 + 预构建 Grid | ~12ms | ~50ns |
| D | 5轴+非解析刀具 | 离散位姿 + toolLocalSDF + 预构建 Grid | ~118ms | ~50ns |

#### 4.6.2 双精度分层

- **sweepGrid(D_v)**：用于 Phase 1 宏观筛选（粗精度够用）
- **preciseEval**：用于 Phase 3 面元剥离判定（解析 SDF 或 fine Grid）
- **ToolSurfelTemplate**：用于 Phase 4 面元注入（预计算精度）

三层各司其职，避免用单一 d_v 精度 Grid 表示扫掠体（不可行，内存 ~12GB）。

#### 4.6.3 统一接口

```cpp
class ToolSweepSDFBuilder {
public:
    static ToolSweepContext build(
        const ToolDef& tool,
        const PathSegment& segment,
        const Config& config);

    static ToolSurfelTemplate buildTemplate(
        const ToolDef& tool, float d_v, float D_v);
};

struct ToolSweepContext {
    FloatGrid::Ptr sweepGrid;        // D_v 精度
    IToolSweepSDF::Ptr preciseEval;  // d_v 精度
    std::vector<Pose> poses;         // 5轴位姿序列
    AABB boundingBox;
    float eval(Vec3d p) const;       // 统一查询入口
    void release();
};
```

---

## 5. 精度验证方案

### 5.1 面元级精度

```
for each active FINE surfel:
    dist = |sweepCtx.preciseEval.eval(surfel.worldPosition)|
    assert dist < d_v
```

### 5.2 与单轨对比

小件场景下与单轨 FloatGrid(d_v) 做 ground truth 比较。

### 5.3 面元定位精度验证

```
for each COARSE surfel:
    if geometrySource.type == MESH:
        assert meshBVH.closestDistance(surfel.pos) < d_v_init * 0.01
    elif geometrySource.type == ANALYTIC_BOX:
        assert analyticDist(surfel.pos) < 1e-10
```

### 5.4 DualGrid 不变量验证

```
function verifyInvariants(sdfGrid, microGrid):
    for each voxel with active surfels:
        assert sdfGrid[voxel] <= 0    // INV-1
    for each voxel where sdfGrid > 0:
        assert !microGrid.hasActivePointsAt(voxel)  // INV-2
```

### 5.5 ToolSweepSDF 构建精度验证

```
// 对解析可求的 Case A，验证 Grid 构建（Case B/C/D）的误差
for each sample point p in sweepGrid narrow band:
    gridVal = sweepGrid.eval(p)
    analyticVal = analyticSweepSDF(p)   // ground truth
    assert |gridVal - analyticVal| < D_v * 0.5
```

---

## 6. 内存监控要求

每次切削操作后输出：

```
[MemStats] FloatGrid: 45.2 MB | PointGrid: 12.8 MB | Total: 58.0 MB
[MemStats] Points: 1,234,567 (active: 987,654) | Leaves: 3,456
[MemStats] Delta: +2.1 MB (inject: +3.5 MB, prune: -1.4 MB)
[MemStats] ToolTemplate: 30.2 MB (resident)
[MemStats] SweepCtx: 0.2 MB (released after cut)
```

---

## 7. 潜在风险点

【风险 1】PointDataGrid 动态点注入的内存碎片。

缓解：
  1. 第二阶段先验证正确性
  2. 预分配策略
  3. 后续引入 RCU 紧凑化

【风险 2】Mesh BVH 常驻内存（大型 STL）。

缓解：
  1. IPW0 完成后释放
  2. 内部体素用刀具模板，不需 BVH
  3. 惰性重建

【风险 3】~~边界面元注入候选网格过密（N3=200万/体素）~~

**已解决**：v0.d 引入 ToolSurfelTemplate 模板方案，Phase 4 复杂度从 O(N3)
降至 O(N2)，加速比 ~2000x。

【风险 4】Case D（5轴+非解析）扫掠体构建耗时。

单段 ~118ms，精加工 50000 段串行需 ~100min。

缓解：
  1. TBB 并行（8核 → ~15ms/段）
  2. 增量式构建（相邻段重叠率 >90%，增量加速 ~10x）
  3. 路径段合并（共线段合一）

【风险 5】ToolSurfelTemplate 内存占用（大刀具/小 d_v）。

极端情况 R=25mm, d_v=0.001mm → 面积 ~4000mm2 → 40亿面元（不可行）。

缓解：
  1. 只预计算有效切削区域（ap 深度范围内）
  2. 分层模板：粗模板(10*d_v) 全覆盖 + 细模板(d_v) 按需局部生成
  3. 实际精加工 d_v=0.005mm + ap=1mm → ~30MB，可接受

【风险 6】5轴非凸刀具的包络面判定。

sweepDist 验证（|sweepDist| < d_v）可能误判。

缓解：
  1. 第二阶段限定凸刀具（球头/平底/牛鼻皆为凸）
  2. 非凸刀具后续引入精确包络计算算法

---

## 8. 端到端性能预算

### 8.1 单段切削耗时

| Case | Phase 0 | Phase 1-3 | Phase 4 | Phase 5-7 | 总计 |
|------|---------|-----------|---------|-----------|------|
| A (3轴解析) | 0 | 0.6ms | 0.5ms | 0.1ms | **1.2ms** |
| B (3轴非解析) | 300ms | 0.6ms | 0.5ms | 0.1ms | **301ms** |
| C (5轴解析) | 12ms | 0.6ms | 4ms | 0.1ms | **17ms** |
| D (5轴非解析) | 118ms | 0.6ms | 4ms | 0.1ms | **123ms** |

### 8.2 全程仿真耗时

| Case | 粗加工(2000段) | 精加工(50000段) |
|------|---------------|----------------|
| A | 2.4s | 60s = 1min |
| B | 602s = 10min | 不适用(3轴精加工用A) |
| C | 34s | 850s = 14min |
| D | 246s = 4.1min | 6150s(串行)/768s(8核) = 13min |

### 8.3 内存峰值

| 组件 | 常驻 | 峰值(切削中) |
|------|------|-------------|
| FloatGrid(D_v) | ~50MB | ~50MB |
| PointDataGrid | ~10MB(初始) | ~100MB(精加工后) |
| ToolSurfelTemplate | ~30MB | ~30MB |
| SweepContext(临时) | 0 | ~0.2MB |
| **总计** | ~90MB | ~180MB |

---

## 修订记录

| 版本 | 日期 | 修订内容 |
|------|------|----------|
| v0.a | 2026-06-01 | 初稿创建 |
| v0.b | 2026-06-02 | 补充面元精度保障机制；几何来源管理 |
| v0.c | 2026-06-02 | 重写 4.3 为 7-Phase 算法；防御性设计标注；DualGrid 不变量 |
| v0.d | 2026-06-02 | 新增 Phase 0 扫掠体构建；Phase 4 改为 ToolSurfelTemplate 模板方案（加速2000x）；新增 4.5 ToolSurfelTemplate 模块；新增 4.6 ToolSweepSDF 构建器摘要；端到端性能预算；关联独立算法文档 Algo_R5 |
