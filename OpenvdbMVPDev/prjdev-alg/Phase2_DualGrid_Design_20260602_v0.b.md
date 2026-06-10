# 第二阶段详细设计：DualGrid 双轨高精度切削

**文档编号**：`Phase2_DualGrid_Design_20260602_v0.b.md`
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
// 每个点的属性（SoA 布局由 OpenVDB PointDataGrid 自动管理）
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
    size_t floatGridBytes;      // FloatGrid 内存
    size_t pointGridBytes;      // PointDataGrid 内存
    size_t totalBytes;          // 总计
    size_t pointCount;          // 总面元数
    size_t activePointCount;    // 活跃面元数
    size_t leafNodeCount;       // 叶节点数

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

**核心问题**：FloatGrid 的 SDF 零等值面精度仅为 ~D_v 级别。若面元直接
"投影到粗 SDF 零等值面"，则面元位置误差量级为 D_v，远不满足 d_v 精度
要求。因此，面元定位必须回溯到原始精确几何。

**设计原则**：粗 SDF 仅用于筛选"哪些体素需要注入面元"（拓扑定位），
面元的精确位置和法线必须从原始几何获取。

**按几何来源分类处理**：

| 几何来源 | 定位策略 | 法线获取 | 精度 |
|----------|----------|----------|------|
| 解析几何（长方体毛坯） | 解析投影：直接用平面方程求精确交点 | 解析法线（轴对齐±X/±Y/±Z） | 精确（零误差） |
| 解析几何（圆柱/球体毛坯） | 解析投影：用几何方程求精确交点 | 解析法线（径向/球面方向） | 精确（零误差） |
| Mesh 导入（STL/OBJ 自由曲面） | 射线求交：沿粗 SDF 梯度方向发射射线，与原始 Mesh BVH 求交 | 从 Mesh 三角面片插值法线 | d_v 级别（取决于 Mesh 密度） |
| 切削后新表面 | 刀具解析求交：用刀具扫掠面几何方程（球面/圆柱面）精确求交 | 刀具几何解析法线 | 精确（零误差） |

**射线求交流程（Mesh 导入场景）**：

```
function locateSurfelOnMesh(voxelCenter, sdfGradient, meshBVH):
    // 1. 从体素中心沿 SDF 梯度反方向发射射线（指向表面）
    rayOrigin = voxelCenter
    rayDir = -normalize(sdfGradient)

    // 2. 与原始 Mesh 的 BVH 加速结构求交
    hit = meshBVH.raycast(rayOrigin, rayDir, maxDist = D_v * 2)

    if hit.valid:
        // 3. 获取精确交点和插值法线
        surfelPos = hit.point           // 精确落在原始 Mesh 表面
        surfelNormal = hit.interpNormal  // 从三角面顶点法线插值
        return (surfelPos, surfelNormal)
    else:
        // 4. 回退：用粗 SDF 投影（降级，记录警告）
        fallbackPos = voxelCenter - sdf * normalize(sdfGradient)
        fallbackNormal = normalize(sdfGradient)
        LOG_WARN("Mesh raycast miss at voxel (%d,%d,%d), fallback to SDF projection")
        return (fallbackPos, fallbackNormal)
```

**解析投影流程（长方体毛坯）**：

```
function locateSurfelOnBox(candidatePoint, boxOrigin, boxDims):
    // 长方体六个面都是轴对齐平面，直接解析求最近面
    // 无需 SDF 插值，零误差

    closestFace = findClosestFace(candidatePoint, boxOrigin, boxDims)
    surfelPos = projectToFace(candidatePoint, closestFace)
    surfelNormal = closestFace.outwardNormal  // ±X, ±Y, ±Z
    return (surfelPos, surfelNormal)
```

**刀具表面解析投影（切削后新表面）**：

```
function locateSurfelOnToolSurface(candidatePoint, toolGeometry):
    // 刀具扫掠面是解析几何（球头=球面，平底=圆柱+平面）
    // 直接用几何方程投影

    surfelPos = toolGeometry.closestPointOn Surface(candidatePoint)
    surfelNormal = toolGeometry.normalAt(surfelPos)
    return (surfelPos, surfelNormal)
```

#### 4.2.2 几何来源管理

为支持上述分类策略，BilletModel 需持有原始几何引用：

```cpp
struct GeometrySource {
    enum Type { ANALYTIC_BOX, ANALYTIC_CYLINDER, ANALYTIC_SPHERE, MESH };
    Type type;

    // 解析几何参数（联合体或 variant）
    struct AnalyticBox { Vec3d origin; Vec3d dims; };
    struct AnalyticCylinder { Vec3d center; double radius; double height; Vec3d axis; };
    struct AnalyticSphere { Vec3d center; double radius; };

    // Mesh 几何（带 BVH 加速结构）
    struct MeshGeometry {
        std::shared_ptr<TriangleMesh> mesh;
        std::shared_ptr<BVHAccel> bvh;       // 射线求交加速
    };

    // 工厂方法
    static GeometrySource fromBox(Vec3d origin, Vec3d dims);
    static GeometrySource fromMesh(const std::string& filePath);
};

struct BilletModel {
    FloatGrid::Ptr sdfGrid;
    PointDataGrid::Ptr microGrid;
    GeometrySource geometrySource;  // ← 新增：保留原始几何引用
    // ...
};
```

#### 4.2.3 设计决策与权衡

| 决策 | 选择 | 理由 |
|------|------|------|
| Mesh BVH 常驻内存？ | 是（直到所有 IPW₀ 注入完成后释放） | IPW₀ 注入是批量操作，BVH 查询高效；注入完成后可释放 |
| 延迟注入时 BVH 是否仍需？ | 仅表面体素需要；内部体素用刀具解析几何 | 内部体素从未暴露过原始表面，其新表面来自刀具 |
| 射线 miss 时的回退？ | 降级到粗 SDF 投影 + 日志警告 | 不阻塞流程，但记录精度风险供后续排查 |
| 解析几何优先？ | 是 | 零误差、零额外内存、零查询开销 |

### 4.3 DualGridCutter（扩展 CuttingEngine）

输入: BilletModel(DUAL_TRACK), ToolSweepSDF
流程:
  Phase 1 - 宏观过滤:
    遍历 FloatGrid 窄带体素
    筛选在刀具 BBox 内且 SDF ≤ 0 的体素 → affectedVoxels

  Phase 2 - 微观切削:
    for each voxelCoord in affectedVoxels:
      if PointDataGrid 该体素无数据:
        触发延迟注入（从 SDF 梯度生成面元）
      for each surfel in voxel:
        if surfel.active == 0: continue
        dist = toolSDF.eval(surfel.position_world)
        if dist <= 0:
          surfel.active = 0  // 剥离

  Phase 3 - 边界面元注入（N² 表面采样）:
    对部分切削的体素，在刀具零等值面上注入精细面元
    面元位置通过刀具解析几何精确定位（见 4.2.1 刀具表面解析投影）
    precision = FINE

  Phase 4 - 宏观 SDF 保守更新:
    activeCount == 0 → SDF = +D_v
    activeCount > 0  → 保持原值不变

  Phase 5 - 内存统计:
    更新 MemoryStats

### 4.4 延迟注入触发

```
function injectOnDemand(microGrid, voxelCoord, sdfGrid, geometrySource, config):
    // 该体素首次被刀具接触，需要生成面元
    // 判断是 IPW₀ 表面还是内部首次暴露

    sdfVal = sdfGrid.getValue(voxelCoord)
    if |sdfVal| < halfWidth * D_v:
        // 表面体素：应该在初始化时已注入（IPW₀）
        // 如果没有，补注入粗面元（使用 geometrySource 精确定位）
        injectSurfels_IPW0(microGrid, voxelCoord, geometrySource, config)
    else:
        // 内部体素首次暴露：新表面来自刀具，用刀具解析几何注入精细面元
        injectSurfels_Cut(microGrid, voxelCoord, toolGeometry, config)
```

---

## 5. 精度验证方案

### 5.1 面元级精度

对切削后的活跃面元，计算其到理论切削表面的距离:
  for each active surfel:
    dist_to_tool_surface = |toolSDF.eval(surfel.position)|
    assert dist_to_tool_surface < d_v  // 面元精度

### 5.2 与单轨对比

同一切削场景，分别用:
  - 单轨 FloatGrid(d_v=0.005mm) → 作为 ground truth（如果内存允许小件）
  - 双轨 FloatGrid(D_v=0.64mm) + PointDataGrid
比较两者的切削表面偏差

### 5.3 面元定位精度验证（新增）

验证 IPW₀ 面元是否精确落在原始几何表面：
```
for each COARSE surfel:
    if geometrySource.type == MESH:
        dist = meshBVH.closestDistance(surfel.position)
        assert dist < d_v_init * 0.01  // 面元到原始 Mesh 距离应接近零
    elif geometrySource.type == ANALYTIC_BOX:
        dist = analyticDistanceToBox(surfel.position, box)
        assert dist < 1e-10  // 解析几何应精确为零（浮点误差）
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

【不同见解】：PointDataGrid 的 appendAttribute 和动态点注入在 OpenVDB
中不是零成本操作。每次注入新面元需要重新分配叶节点内存。高频切削时（每段都触发注入），可能产生严重的内存碎片。

缓解：
  1. 第二阶段先验证正确性，不追求性能
  2. 预分配策略：初始化时为所有窄带体素预留面元容量
  3. 后续引入 RCU 紧凑化

【新增风险】：Mesh BVH 常驻内存开销。

对于大型 STL 模型（百万三角面），BVH 可能占用数十 MB 内存。

缓解：
  1. IPW₀ 批量注入完成后立即释放 BVH（geometrySource.releaseBVH()）
  2. 延迟注入场景下，仅表面体素需要 BVH；内部体素新表面来自刀具解析几何，不需要 BVH
  3. 若后续需要重建 BVH（如多次加工间的毛坯重置），采用惰性重建策略

---

## 修订记录

| 版本 | 日期 | 修订内容 |
|------|------|----------|
| v0.a | 2026-06-01 | 初稿创建 |
| v0.b | 2026-06-02 | 补充 4.2.1 面元精度保障机制：区分解析几何/Mesh/刀具表面三种定位策略；新增 4.2.2 几何来源管理结构；新增 4.2.3 设计决策权衡表；新增 5.3 面元定位精度验证；更新延迟注入接口签名 |
