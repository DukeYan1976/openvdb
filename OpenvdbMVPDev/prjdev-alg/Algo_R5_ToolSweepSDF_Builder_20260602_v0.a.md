# 算法设计：ToolSweepSDF 通用构建器

**文档编号**：`Algo_R5_ToolSweepSDF_Builder_20260602_v0.a.md`
**状态**：初稿
**作者**：Duke / Kiro
**日期**：2026-06-02
**关联**：Phase2_DualGrid_Design_20260602_v0.d

---

## 1. 问题定义

### 1.1 什么是 ToolSweepSDF

ToolSweepSDF 表示刀具从位姿 P_start 运动到位姿 P_end 过程中扫过的
全部空间的符号距离场。

```
ToolSweepSDF(x) ≤ 0  ⟹  点 x 被刀具扫掠体占据（需从毛坯中减去）
ToolSweepSDF(x) > 0  ⟹  点 x 在扫掠体外部（材料保留）
ToolSweepSDF(x) = 0  ⟹  扫掠体表面（新切削面的来源）
```

### 1.2 构建难度的两个维度

| | 解析刀具（球头/平底/牛鼻） | 非解析刀具（异形/成形刀） |
|--|--|--|
| **3轴（纯平移）** | 最简单：闭式解析 SDF | 中等：刀具 SDF + 平移包络 |
| **5轴（平移+旋转）** | 中等：离散位姿包络 | 最复杂：离散位姿 + 刀具 SDF |

### 1.3 设计目标

- 统一接口：无论哪种组合，DualGridCutter 都通过同一个 ToolSweepSDF 接口访问
- 精度保证：零等值面误差 < d_v
- 性能可控：构建时间与切削段长度成正比，不随毛坯大小增长
- 内存可控：临时 Grid 在切削完成后立即释放

---

## 2. 统一接口设计

```cpp
// 抽象接口：所有 ToolSweepSDF 实现必须提供
class IToolSweepSDF {
public:
    // 核心查询：给定世界坐标点，返回到扫掠体表面的符号距离
    virtual float eval(const Vec3d& worldPos) const = 0;

    // 包围盒：用于宏观筛选（Phase 1）
    virtual AABB boundingBox() const = 0;

    // 可选：梯度（用于面元法线估算的 fallback）
    virtual Vec3f gradient(const Vec3d& worldPos) const;

    // 生命周期
    virtual size_t memoryUsage() const = 0;
    virtual void release() = 0;
};
```

---

## 3. 分类构建策略

### 3.1 Case A：3轴 + 解析刀具（闭式解析 SDF）

**适用**：球头刀/平底刀/牛鼻刀，刀轴固定（通常为 Z 轴），纯 XYZ 平移。

**物理模型**：刀具沿直线段 L 从 P0 平移到 P1，扫掠体 = 刀具沿 L 的
Minkowski 和。

**解析 SDF 推导**：

```
// 球头刀（半径 R）沿线段 [P0, P1] 平移的扫掠体 SDF
// = 胶囊体（Capsule）SDF
class AnalyticSweep_BallEnd_3Axis : public IToolSweepSDF {
    Vec3d P0, P1;       // 线段端点（刀尖位置）
    double R;           // 球头半径
    double H;           // 圆柱部分高度

    float eval(const Vec3d& p) const override {
        // 球头部分：到线段的距离 - R
        Vec3d closest = closestPointOnSegment(p, P0, P1);
        double distToAxis = length(p - closest);

        // 球头区域（closest 处的刀具截面半径）
        double z_local = dot(p - closest, toolAxis);
        if (z_local <= 0) {
            // 球头区域：到球心的距离 - R
            return length(p - closest) - R;
        } else if (z_local <= H) {
            // 圆柱区域：到轴线的 XY 距离 - R
            return distToAxis - R;
        } else {
            // 超出刀具高度（不应该到达）
            return distToAxis;  // 保守估计
        }
    }
};

// 平底刀（半径 R）沿线段平移 = 圆角长方体
class AnalyticSweep_FlatEnd_3Axis : public IToolSweepSDF {
    Vec3d P0, P1;
    double R, H;

    float eval(const Vec3d& p) const override {
        Vec3d closest = closestPointOnSegment(p, P0, P1);
        double z_local = dot(p - closest, toolAxis);
        double r_local = length(cross(p - closest, toolAxis));

        // 组合：圆柱 SDF（径向） 和 平面 SDF（轴向）的 max
        float sdf_radial = r_local - R;
        float sdf_bottom = -z_local;          // 底面
        float sdf_top = z_local - H;          // 顶面
        return max(sdf_radial, max(sdf_bottom, sdf_top));
    }
};
```

**性能分析**：
- eval() 开销：~20 次浮点运算（1 次 closestPointOnSegment + 距离计算）
- 内存：O(1)，仅存储几个参数
- 构建时间：O(1)，无需任何预计算
- **结论：无性能问题，首选方案**

### 3.2 Case B：3轴 + 非解析刀具（刀具模板 SDF + 平移包络）

**适用**：异形刀、成形刀、磨削轮等不可解析的刀具形状，刀轴固定，纯平移。

**核心思路**：
- 预计算刀具在局部坐标系下的高精度 SDF Grid（ToolLocalSDF）
- 运行时：扫掠体 SDF = min over path { ToolLocalSDF(p - offset(t)) }

```
class GridSweep_NonAnalytic_3Axis : public IToolSweepSDF {
    FloatGrid::Ptr toolLocalSDF;   // 预计算的刀具局部 SDF（voxelSize = d_v）
    Vec3d P0, P1;                   // 路径线段
    int numSamples;                 // 路径离散采样数

    float eval(const Vec3d& p) const override {
        // 沿路径离散采样，取最小 SDF（最深入扫掠体的位置）
        float minDist = +INF;
        for (int i = 0; i <= numSamples; i++) {
            float t = float(i) / numSamples;
            Vec3d offset = lerp(P0, P1, t);
            Vec3d localP = p - offset;  // 变换到刀具局部坐标
            float d = toolLocalSDF.eval(localP);  // 三线性插值
            minDist = min(minDist, d);
        }
        return minDist;
    }
};
```

**路径离散间距选择**：
```
pathLength = length(P1 - P0)
numSamples = ceil(pathLength / d_v)   // 保证采样间距 ≤ d_v
```

**性能分析**：
- 典型切削段长度：1~5 mm
- numSamples = 5mm / 0.005mm = 1000
- 每次 eval = 1000 次 Grid 三线性插值
- 三线性插值 ≈ 8 次 Grid 访问 + 7 次 lerp ≈ ~50ns
- 单次 eval 总耗时 ≈ 1000 × 50ns = **50μs**

**与 Case A 对比**：比解析方案慢 ~1000×。但在 Phase 3 微观切削中：
- 每体素面元数 ≈ 4~16384（COARSE~FINE）
- 每体素耗时 = 面元数 × 50μs = 0.2ms ~ 800ms

**结论：对长路径段有性能压力。需要优化。**

**优化方案：预构建扫掠体 Grid（推荐）**

不在 eval 时做逐采样，而是一次性构建整个扫掠体的 SDF Grid：

```
function buildSweepGrid_3Axis(toolLocalSDF, P0, P1, D_v, d_v) -> FloatGrid::Ptr:

    // 1. 确定扫掠体包围盒
    toolBBox = toolLocalSDF.boundingBox()
    sweepBBox = expand(toolBBox, P0, P1)  // 刀具 BBox 沿路径扫过的总 BBox

    // 2. 创建临时 FloatGrid（voxelSize 选择见下文）
    sweepGrid = FloatGrid::create(voxelSize = D_v)
    // 用 D_v 而非 d_v：因为 Phase 1/3 只需要 D_v 精度的快速判定
    // 面元精确定位由 ToolSurfelTemplate 负责（见 4.5）

    // 3. 遍历 sweepBBox 内的体素，计算 SDF
    for each voxelCoord in sweepBBox:
        worldPos = sweepGrid.indexToWorld(voxelCoord)
        minDist = +INF
        for (int i = 0; i <= numSamples; i++):
            t = float(i) / numSamples
            offset = lerp(P0, P1, t)
            localP = worldPos - offset
            d = toolLocalSDF.eval(localP)
            minDist = min(minDist, d)
        sweepGrid.setValue(voxelCoord, minDist)

    // 4. 窄带裁剪（只保留表面附近的体素，节省内存）
    sweepGrid.pruneByThreshold(halfWidth * D_v)

    return sweepGrid
```

**预构建性能**：
- sweepBBox 体素数 ≈ (刀具直径/D_v + 路径长度/D_v) × (刀具直径/D_v)²
- 例：R=5mm 刀具, 路径 5mm, D_v=0.64mm
  - BBox ≈ (15.64/0.64) × (10/0.64)² ≈ 24 × 244 ≈ 5,900 体素
- 每体素 numSamples=1000 次插值 × 50ns = 50μs
- 总构建时间 ≈ 5,900 × 50μs ≈ **295ms**
- 构建后 eval = O(1) 三线性插值 ≈ 50ns

**结论：预构建 ~300ms，之后所有 eval 变为 O(1)。可接受。**

### 3.3 Case C：5轴 + 解析刀具（离散位姿包络）

**适用**：球头刀/平底刀/牛鼻刀，刀轴方向随路径变化（A/B/C 轴联动）。

**物理模型**：刀具从位姿 (P0, Q0) 运动到 (P1, Q1)，其中 P 是位置，
Q 是方向（四元数或旋转矩阵）。扫掠体是所有中间位姿处刀具的并集。

**关键挑战**：刀轴旋转使得扫掠体不再是简单的 Minkowski 和，没有闭式解。

**构建方案：离散位姿采样 + min 包络**

```
class DiscretePoseSweep_Analytic_5Axis : public IToolSweepSDF {
    struct Pose { Vec3d position; Quatd rotation; };
    std::vector<Pose> poses;        // 离散位姿序列
    ToolGeometry toolGeo;           // 解析刀具几何

    float eval(const Vec3d& p) const override {
        float minDist = +INF;
        for (const auto& pose : poses) {
            // 将查询点变换到当前位姿的刀具局部坐标系
            Vec3d localP = pose.rotation.inverse() * (p - pose.position);
            // 在局部坐标系下用解析公式计算 SDF
            float d = toolGeo.analyticSDF(localP);
            minDist = min(minDist, d);
        }
        return minDist;
    }
};
```

**位姿采样密度**：
```
// 位姿间距需要保证相邻两个位姿之间的刀具表面间隙 < d_v
// 对旋转：角度增量 dTheta 使得刀尖处位移 < d_v
//   R * dTheta < d_v  →  dTheta < d_v / R

// 例：R=5mm, d_v=0.005mm → dTheta < 0.001 rad ≈ 0.057°
// 对平移：间距 < d_v

pathLength = arcLength(P0,Q0 → P1,Q1)
angularSpan = angleBetween(Q0, Q1)
numPoses_translation = ceil(pathLength / d_v)
numPoses_rotation = ceil(angularSpan * R / d_v)
numPoses = max(numPoses_translation, numPoses_rotation)
```

**典型数据**：
- 路径段 2mm，角度变化 2° = 0.035 rad，R=5mm
- numPoses_translation = 2/0.005 = 400
- numPoses_rotation = 0.035 × 5 / 0.005 = 35
- numPoses = 400
- 每次 eval = 400 × analyticSDF（~20 flops）≈ 400 × ~5ns = **2μs**

**对比纯平移方案（Case A 50ns）**：慢 40×，但仍在可接受范围。

**更优方案：预构建 5 轴扫掠体 Grid**

与 Case B 相同思路，将逐位姿采样的结果预写入 Grid：

```
function buildSweepGrid_5Axis_Analytic(toolGeo, poses[], D_v) -> FloatGrid::Ptr:

    // 1. 计算所有位姿下刀具 BBox 的并集
    sweepBBox = AABB()
    for each pose in poses:
        localBBox = toolGeo.boundingBox()
        worldBBox = pose.transformBBox(localBBox)
        sweepBBox.expand(worldBBox)

    // 2. 创建 Grid
    sweepGrid = FloatGrid::create(voxelSize = D_v)

    // 3. 遍历体素，计算 min-over-poses SDF
    for each voxelCoord in sweepBBox:
        worldPos = sweepGrid.indexToWorld(voxelCoord)
        minDist = +INF
        for each pose in poses:
            localP = pose.inverse().transform(worldPos)
            d = toolGeo.analyticSDF(localP)
            minDist = min(minDist, d)
        sweepGrid.setValue(voxelCoord, minDist)

    // 4. 窄带裁剪
    sweepGrid.pruneByThreshold(halfWidth * D_v)
    return sweepGrid
```

**性能**：
- BBox 体素数 ≈ 5,900（与 Case B 同量级）
- 每体素 400 次解析 SDF ≈ 2μs
- 总构建 ≈ 5,900 × 2μs ≈ **12ms**
- **构建后 eval = O(1)，~50ns**

**结论：5 轴解析刀具预构建仅 12ms，极快。推荐预构建。**

### 3.4 Case D：5轴 + 非解析刀具（最复杂情况）

**适用**：异形刀/成形刀/磨削轮，5轴联动运动。

**双重离散**：
- 刀具形状离散化 → ToolLocalSDF Grid（预计算，一次性）
- 运动路径离散化 → 逐位姿采样

```
function buildSweepGrid_5Axis_NonAnalytic(toolLocalSDF, poses[], D_v) -> FloatGrid::Ptr:

    sweepBBox = computeSweepBBox(toolLocalSDF.boundingBox(), poses)
    sweepGrid = FloatGrid::create(voxelSize = D_v)

    for each voxelCoord in sweepBBox:
        worldPos = sweepGrid.indexToWorld(voxelCoord)
        minDist = +INF
        for each pose in poses:
            // 世界坐标 → 刀具局部坐标（含旋转）
            localP = pose.inverse().transform(worldPos)
            // 在刀具 SDF Grid 中三线性插值
            d = toolLocalSDF.eval(localP)
            minDist = min(minDist, d)
        sweepGrid.setValue(voxelCoord, minDist)

    sweepGrid.pruneByThreshold(halfWidth * D_v)
    return sweepGrid
```

**性能**：
- BBox 体素数 ≈ 5,900
- 每体素 400 位姿 × 三线性插值(50ns) = 20μs
- 总构建 ≈ 5,900 × 20μs ≈ **118ms**
- 构建后 eval = O(1)

**结论：118ms 完全可接受。即使路径段更长（10mm, 2000 位姿），也在 600ms 内。**

---

## 4. 预构建扫掠体 Grid 的精度与 VoxelSize 选择

### 4.1 问题

Phase 4（边界面元注入）需要的精度是 d_v，但如果扫掠体 Grid 用 d_v 做
voxelSize，内存会爆炸：

```
扫掠体 BBox ≈ 15mm × 10mm × 10mm
体素数 = (15/0.005) × (10/0.005) × (10/0.005) = 3000 × 2000 × 2000 = 12×10⁹
内存 = 12GB（不可行）
```

### 4.2 双精度分层方案（DualGrid 思想的复用）

**关键洞察**：扫掠体 SDF 在 DualGridCutter 中有两个不同的使用场景，
精度需求不同：

| 使用场景 | 精度需求 | 原因 |
|----------|----------|------|
| Phase 1 宏观筛选 | ~D_v | 只需判定体素是否可能受影响 |
| Phase 3 微观切削（面元剥离判定） | ~d_v | 面元精度要求 |
| Phase 4 边界面元定位 | ~d_v | 新面元必须精确落在零等值面上 |

**方案：扫掠体 Grid 用 D_v 做 voxelSize + 局部高精度补偿**

```
Phase 1/筛选：直接用 sweepGrid(D_v) → 足够
Phase 3/剥离判定：
  - 解析刀具：直接用解析 SDF eval（精度无限）
  - 非解析刀具：用 toolLocalSDF(d_v) 逐位姿插值（精度 d_v）
Phase 4/面元定位：
  - 使用 ToolSurfelTemplate（预计算面元模板，精度 d_v）
```

即：**sweepGrid(D_v) 只用于宏观筛选，微观操作回退到精确计算**。

### 4.3 最终架构

```cpp
struct ToolSweepContext {
    // 宏观层：快速筛选用
    FloatGrid::Ptr sweepGrid;         // voxelSize = D_v，用于 Phase 1
    AABB boundingBox;

    // 微观层：精确 eval 用（Phase 3 面元剥离）
    IToolSweepSDF::Ptr preciseEval;   // 实现取决于 Case A/B/C/D

    // 面元注入层：Phase 4 用
    ToolSurfelTemplate::Ptr template; // 预计算面元模板

    // 工厂方法
    static ToolSweepContext build(ToolDef tool, PathSegment seg, Config cfg);
};
```

---

## 5. ToolSurfelTemplate（刀具面元模板）

### 5.1 概念

刀具形状不变 → 其表面面元分布可一次预计算。运行时只需将模板面元
做刚体变换（旋转+平移）到当前位姿，即可获得切削新表面的精确面元。

### 5.2 数据结构

```cpp
struct ToolSurfelTemplate {
    struct LocalSurfel {
        Vec3f localPos;      // 相对于 TCP（刀尖中心点）的偏移
        Vec3f localNormal;   // 局部外法线
    };

    std::vector<LocalSurfel> surfels;   // 全部面元
    float spacing;                       // 面元间距 = d_v
    AABB localBBox;                      // 模板包围盒

    // 空间索引（用 OpenVDB PointDataGrid，VoxelSize = D_v）
    PointDataGrid::Ptr spatialIndex;

    // 查询：给定局部坐标系下的 BBox，返回其中的面元
    std::vector<const LocalSurfel*> queryBox(AABB localBBox) const;
};
```

### 5.3 预计算（一次性，刀具加载时执行）

```
function buildToolSurfelTemplate(toolDef, d_v, D_v) -> ToolSurfelTemplate:

    template = ToolSurfelTemplate(spacing = d_v)

    if toolDef.isAnalytic():
        // 解析参数化采样
        for each (u, v) in toolDef.parameterDomain(step = d_v):
            pos = toolDef.evalUV(u, v)
            nrm = toolDef.normalAtUV(u, v)
            template.surfels.push_back({pos, nrm})
    else:
        // 从刀具 SDF Grid 零等值面提取
        toolSDF = buildToolLocalSDF(toolDef, voxelSize = d_v)
        for each narrowBandVoxel in toolSDF:
            grad = toolSDF.gradient(voxel)
            sdfVal = toolSDF.getValue(voxel)
            surfacePos = voxelCenter - sdfVal * normalize(grad)
            surfaceNormal = normalize(grad)
            // 去重：确保面元间距 ≥ d_v * 0.8
            if not template.hasNeighborWithin(surfacePos, d_v * 0.8):
                template.surfels.push_back({surfacePos, surfaceNormal})

    // 构建空间索引
    template.spatialIndex = buildPointDataGrid(template.surfels, D_v)
    template.localBBox = computeBBox(template.surfels)

    return template
```

**预计算性能**（球头刀 R=5mm, 有效高度 3mm, d_v=0.005mm）：
```
有效表面积 ≈ 2πR·3 + πR² ≈ 94 + 78 ≈ 172 mm²
面元数 ≈ 172 / (0.005²) ≈ 6,880,000
内存 ≈ 6.88M × 24 bytes ≈ 165 MB
```

**优化：只预计算有效切削区域**：
```
实际切削深度 ap = 1mm → 有效面积 ≈ 2π×5×1 ≈ 31 mm²
面元数 ≈ 31 / 0.000025 ≈ 1,240,000
内存 ≈ 30 MB
```

### 5.4 运行时使用（Phase 4 替代方案）

```
function injectBoundarySurfels_Template(
    microGrid, boundaryVoxels[], toolSurfelTemplate,
    toolPose, sdfGrid, config):

    // toolPose = 刀具终态位姿（切削段终点）
    // 对凸刀具，扫掠体表面在终态处等于刀具表面（包络性质）

    for each voxelCoord in boundaryVoxels:
        voxelBBox_world = getVoxelBBox(voxelCoord)

        // 将体素 BBox 逆变换到刀具局部坐标系
        voxelBBox_local = toolPose.inverse().transformBBox(voxelBBox_world)

        // 在模板空间索引中查找此 BBox 内的面元
        localSurfels = toolSurfelTemplate.queryBox(voxelBBox_local)

        for each ls in localSurfels:
            // 变换到世界坐标
            worldPos = toolPose.transformPoint(ls.localPos)
            worldNormal = toolPose.rotateVector(ls.localNormal)

            // 验证：该面元确实在扫掠体表面上（对非凸扫掠需要额外检查）
            // 对凸刀具 + 终态位姿，这自动满足

            // 材料侧验证
            checkPos = worldPos - worldNormal * d_v * 0.5
            if sdfGrid.eval(checkPos) > 0:
                continue

            // 注入
            microGrid.addPoint(voxelCoord, Surfel{
                position: worldPos - voxelCenter(voxelCoord),
                normal: worldNormal,
                active: 1,
                precision: FINE
            })
```

### 5.5 5轴扫掠体的面元注入

5轴情况下，扫掠体表面 ≠ 终态刀具表面。扫掠体的包络面由多个中间位姿
共同决定。

**处理方案**：多位姿模板叠加 + 扫掠体 SDF 裁剪

```
function injectBoundarySurfels_5Axis(
    microGrid, boundaryVoxels[], toolSurfelTemplate,
    poses[], sweepSDF, sdfGrid, config):

    for each voxelCoord in boundaryVoxels:
        voxelBBox_world = getVoxelBBox(voxelCoord)
        injected = HashSet()  // 去重：避免同一位置重复注入

        for each pose in poses:
            voxelBBox_local = pose.inverse().transformBBox(voxelBBox_world)
            localSurfels = toolSurfelTemplate.queryBox(voxelBBox_local)

            for each ls in localSurfels:
                worldPos = pose.transformPoint(ls.localPos)
                worldNormal = pose.rotateVector(ls.localNormal)

                // 关键验证：该面元位于扫掠体零等值面上
                // （不是所有位姿的刀具表面都构成最终包络）
                sweepDist = sweepSDF.eval(worldPos)
                if |sweepDist| > d_v:
                    continue  // 不在扫掠体表面上，跳过

                // 去重（空间距离 < d_v/2 的视为同一面元）
                gridKey = quantize(worldPos, d_v * 0.5)
                if gridKey in injected:
                    continue
                injected.add(gridKey)

                // 材料侧验证
                checkPos = worldPos - worldNormal * d_v * 0.5
                if sdfGrid.eval(checkPos) > 0:
                    continue

                microGrid.addPoint(voxelCoord, Surfel{...})
```

**5 轴面元注入的额外开销**：
- 每个 boundary 体素需遍历所有位姿的模板面元
- 但通过 BBox 裁剪，每个位姿平均只贡献 1~2 个面元到该体素
- 400 位姿 × ~2 面元 × (1 sweepSDF.eval + 变换) ≈ 400 × 100ns = 40μs/体素
- 100 个 boundary 体素 → 4ms，可接受

---

## 6. 综合性能分析

### 6.1 各 Case 构建性能汇总

| Case | 构建方式 | 构建时间 | eval 耗时 | 内存 |
|------|----------|----------|-----------|------|
| A: 3轴+解析 | 无需构建（闭式） | 0 | ~50ns | O(1) |
| B: 3轴+非解析 | 预构建 sweepGrid(D_v) | ~300ms | ~50ns(Grid) | ~200KB |
| C: 5轴+解析 | 预构建 sweepGrid(D_v) | ~12ms | ~50ns(Grid) | ~200KB |
| D: 5轴+非解析 | 预构建 sweepGrid(D_v) | ~118ms | ~50ns(Grid) | ~200KB |

注：构建时间基于典型参数（路径段 5mm, 刀具 R=5mm, D_v=0.64mm, d_v=0.005mm）。

### 6.2 端到端单次切削段性能预算

以 Case D（最复杂情况）为例：

| 阶段 | 操作 | 耗时 |
|------|------|------|
| 扫掠体 Grid 构建 | 5,900 体素 × 400 位姿 × 插值 | 118ms |
| Phase 1 宏观筛选 | ~10,000 体素判定 | 0.5ms |
| Phase 2 体素分类 | ~200 体素 | 0.01ms |
| Phase 3 微观切削 | ~100 体素 × ~16 面元 × eval | 0.08ms |
| Phase 4 面元注入（模板方案） | ~50 体素 × 400 位姿 × ~2 面元 | 4ms |
| Phase 5-7 后处理 | O(affected) | 0.1ms |
| **总计** | | **~123ms** |

**其中扫掠体构建占 96%**。这是值得的开销，因为：
- 构建一次，Phase 3 所有面元判定都用 O(1) eval
- 如果不预构建（在线逐采样），Phase 3 每次 eval 需 50μs，总耗时反而更高

### 6.3 Case A 端到端性能（最轻量）

| 阶段 | 耗时 |
|------|------|
| 无需构建 | 0 |
| Phase 1-3 | 0.6ms |
| Phase 4（参数化） | 0.5ms |
| **总计** | **~1.1ms** |

### 6.4 吞吐量评估

典型 G 代码段数：
- 粗加工：路径段 ~2000 段
- 精加工：路径段 ~50,000 段

| Case | 单段耗时 | 2000 段 | 50000 段 |
|------|----------|---------|----------|
| A | 1.1ms | 2.2s | 55s |
| D | 123ms | 246s ≈ 4.1min | 6150s ≈ 102min |

Case D 精加工 50000 段需要 ~100 分钟。**需要并行化优化**：
- 扫掠体 Grid 构建可并行（体素间独立）
- OpenVDB 原生支持 TBB 并行
- 8 核并行 → Case D 降至 ~15ms/段 → 精加工 12.5min

---

## 7. 优化策略

### 7.1 增量式扫掠体构建

相邻路径段的扫掠体高度重叠。可以只更新差异部分：

```
// 段 k 的扫掠体 = 段 k-1 的部分体素 + 新增体素
// 仅重新计算刀具 BBox 位移差出的体素
function buildSweepGrid_Incremental(prevGrid, prevPoses, newPoses, toolSDF):
    // 找出新 BBox 相对于旧 BBox 的差集体素
    newVoxels = diff(newBBox, prevBBox)
    // 只计算差集部分
    for each voxel in newVoxels:
        ...
    // 复用重叠部分的 min 值（取 prevGrid 和新计算的 min）
```

典型相邻段重叠率 > 90% → 增量构建加速 ~10×。

### 7.2 ToolLocalSDF 精度分级

```
ToolLocalSDF 可以准备两个分辨率版本：
  - toolSDF_coarse (voxelSize = D_v)：用于 sweepGrid 构建
  - toolSDF_fine (voxelSize = d_v)：用于 Phase 3 精确 eval

sweepGrid 构建用 coarse 版本 → 插值速度更快
Phase 3 面元判定用 fine 版本 → 精度保证
```

### 7.3 路径段合并

对于 3 轴连续共线段，可以合并为一个长段构建一次 sweepGrid：
```
if consecutive segments are collinear (same direction):
    merge into single long segment
    build one sweepGrid for merged segment
```

---

## 8. 通用构建器接口

```cpp
class ToolSweepSDFBuilder {
public:
    struct Config {
        float D_v;              // 宏观体素大小
        float d_v;              // 微观面元精度
        float halfWidth;        // 窄带半宽
        int maxPosesPerSegment; // 最大位姿采样数（限制极端情况）
    };

    // 主入口：根据刀具和路径自动选择最优策略
    static ToolSweepContext build(
        const ToolDef& tool,
        const PathSegment& segment,
        const Config& config
    ) {
        if (tool.isAnalytic() && segment.is3Axis()) {
            return buildCase_A(tool, segment, config);
        } else if (!tool.isAnalytic() && segment.is3Axis()) {
            return buildCase_B(tool, segment, config);
        } else if (tool.isAnalytic() && segment.is5Axis()) {
            return buildCase_C(tool, segment, config);
        } else {
            return buildCase_D(tool, segment, config);
        }
    }

    // 刀具面元模板（一次性预计算，刀具加载时调用）
    static ToolSurfelTemplate buildTemplate(
        const ToolDef& tool,
        float d_v,
        float D_v
    );

private:
    static ToolSweepContext buildCase_A(...);
    static ToolSweepContext buildCase_B(...);
    static ToolSweepContext buildCase_C(...);
    static ToolSweepContext buildCase_D(...);
};
```

---

## 9. 与 DualGridCutter 的集成

### 9.1 更新后的切削流程

```
function DualGridCutter.cut(billetModel, pathSegment, toolDef, toolTemplate):

    // ═══ 新增：构建扫掠体上下文 ═══
    sweepCtx = ToolSweepSDFBuilder.build(toolDef, pathSegment, config)

    sdfGrid   = billetModel.sdfGrid
    microGrid = billetModel.microGrid

    // Phase 1: 用 sweepCtx.sweepGrid 或 sweepCtx.preciseEval 做宏观筛选
    affectedVoxels = macroFilter(sdfGrid, sweepCtx)

    // Phase 2-3: 用 sweepCtx.preciseEval 做面元剥离判定
    ...
    cutResult = microCut(microGrid, voxelCoord, sweepCtx.preciseEval)

    // Phase 4: 用 toolTemplate + 位姿变换 做面元注入
    if pathSegment.is3Axis() || toolDef.isConvex():
        // 凸刀具 + 3轴：终态位姿的模板面元 = 扫掠体表面
        injectBoundarySurfels_Template(
            microGrid, boundaryVoxels, toolTemplate,
            pathSegment.endPose, sdfGrid, config)
    else:
        // 5轴或非凸：多位姿叠加
        injectBoundarySurfels_5Axis(
            microGrid, boundaryVoxels, toolTemplate,
            sweepCtx.poses, sweepCtx.preciseEval, sdfGrid, config)

    // Phase 5-7: 后处理（不变）
    ...

    // ═══ 释放临时扫掠体 Grid ═══
    sweepCtx.release()
```

### 9.2 生命周期管理

```
刀具加载时：
  toolTemplate = ToolSweepSDFBuilder.buildTemplate(toolDef, d_v, D_v)
  // 常驻内存（~30MB），整个加工过程复用

每个路径段：
  sweepCtx = build(...)     // 构建临时 Grid
  ... 执行切削 ...
  sweepCtx.release()        // 立即释放

加工结束时：
  toolTemplate.release()
```

---

## 10. 风险与待决项

| 风险 | 影响 | 缓解 |
|------|------|------|
| Case D 精加工 50000 段总耗时过长 | 仿真实时性 | TBB 并行 + 增量构建 |
| ToolSurfelTemplate 内存（大刀具） | 可能超 100MB | 只预计算有效切削区域 |
| 5 轴非凸刀具的包络面判定 | sweepDist 验证可能漏判 | 后续引入精确包络计算 |
| 相邻路径段交界处面元连续性 | 面元间隙/重叠 | 段间重叠区域去重 |

---

## 修订记录

| 版本 | 日期 | 修订内容 |
|------|------|----------|
| v0.a | 2026-06-02 | 初稿：4 种 Case 分类构建策略；ToolSurfelTemplate 预计算方案；综合性能分析；通用构建器接口 |
