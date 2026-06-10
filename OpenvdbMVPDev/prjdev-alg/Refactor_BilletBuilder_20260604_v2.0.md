# BilletBuilder 重构设计方案 v2：延迟面元化 + 几何原生采样

**文档编号**：`Refactor_BilletBuilder_20260604_v2.0.md`  
**状态**：设计中 (Under Review)  
**作者**：Kiro / Duke  
**日期**：2026-06-04  
**前置文档**：`Refactor_BilletBuilder_20260604_v1.0.md`, `Algo_R2_BilletBuilder_20260603_v0.f.md`

---

## 1. 设计哲学变化

### 1.1 核心洞察

> **绝大多数毛坯表面最终会被切削掉。为不会存活的表面预先生成面元是纯粹的浪费。**

v1.0 方案试图在 build 阶段用几何原生采样快速生成 IPW₀ 面元。但即便优化到极致，仍然是 O(表面积/d_v_init²) 的工作量。本方案的核心转变：

| | v1.0 方案 | v2.0 方案（本文） |
|---|---|---|
| build 阶段 | 生成粗面元 (IPW₀) | **仅构建 SDF，零面元** |
| cut 阶段 | 剥离 + 边界注入 | **按需生成精细面元** |
| 首次切削延迟 | 无 | 微小（单体素采样 ~μs） |
| 总面元生成量 | 整个表面 | **仅切削暴露面** |
| 面元精度 | 粗 → 后续升级为精 | **直接精细** |

### 1.2 为什么延迟面元化更优？

1. **消除精度分级**：不再需要 `COARSE → FINE` 升级逻辑、属性标记、升级时机判断
2. **内存极致**：初始化 O(1)，运行时内存严格正比于已切削面积
3. **代码简化**：BilletBuilder 退化为纯 SDF 构建器，复杂逻辑转移到 CuttingEngine（已有基础）
4. **原始几何保真**：面元直接从刀具解析 SDF 生成，不依赖宏观 SDF 梯度近似

---

## 2. 架构设计

### 2.1 系统分层

```
┌─────────────────────────────────────────────────┐
│                 BilletBuilder (v2)               │
│  职责：构建宏观 SDF + 保留几何定义               │
│  输出：FloatGrid(D_v) + GeometryDef             │
└───────────────────────┬─────────────────────────┘
                        │
                        ▼
┌─────────────────────────────────────────────────┐
│              CuttingEngine (v2)                  │
│  Phase 1: SDF CSG Difference (同现有)           │
│  Phase 2: 脏区标记 (Dirty Region Detection)     │
│  Phase 3: 延迟面元生成 (Lazy Surfel Gen)        │
│  Phase 4: 面元裁剪 (Surfel Clip)               │
└───────────────────────┬─────────────────────────┘
                        │
                        ▼
┌─────────────────────────────────────────────────┐
│           SurfelGenerator (新模块)              │
│  职责：给定体素坐标 + 几何定义 → 精细面元       │
│  策略：刀具面解析采样; Box/Cyl/Mesh 几何采样    │
└─────────────────────────────────────────────────┘
```

### 2.2 数据模型变化

```cpp
struct GeometryDef {
    enum Type { BOX, CYLINDER, MESH };
    Type type;
    Vec3d origin, dims;         // BOX
    double radius, height;      // CYLINDER
    // MeshDef meshDef;         // MESH (Phase 2)
};

struct BilletModel {
    openvdb::FloatGrid::Ptr sdfGrid;                 // 宏观 SDF (VoxelSize = D_v)
    openvdb::points::PointDataGrid::Ptr microGrid;   // 面元（初始为 nullptr）
    openvdb::MaskGrid::Ptr dirtyMask;                // 脏区标记
    GeometryDef geometry;                             // 保留原始几何定义
    ResolutionConfig config;
    Vec3d origin, dims;
};
```

关键变化：
- `microGrid` 初始为 `nullptr`（延迟创建）
- 新增 `dirtyMask`：记录哪些体素经历了切削，需要生成面元
- 新增 `geometry`：保留原始几何定义供后续查询

---

## 3. BilletBuilder v2 实现

### 3.1 职责极简化

```cpp
BilletModel buildBillet(const ResolutionConfig& config,
                        const Vec3d& origin, const Vec3d& dims) {
    BilletModel model;
    model.config = config;
    model.origin = origin;
    model.dims = dims;
    model.geometry = {GeometryDef::BOX, origin, dims, 0, 0};

    if (config.mode == ResolutionConfig::SINGLE_TRACK) {
        // 单轨：直接用 d_v 构建高精度 SDF，无需面元
        model.sdfGrid = tools::createLevelSetBox<FloatGrid>(
            BBoxd(origin, origin+dims),
            *Transform::createLinearTransform(config.d_v));
    } else {
        // 双轨：用 D_v 构建宏观 SDF
        model.sdfGrid = tools::createLevelSetBox<FloatGrid>(
            BBoxd(origin, origin+dims),
            *Transform::createLinearTransform(config.D_v));
        // microGrid 延迟创建
        model.microGrid = nullptr;
        // dirtyMask 共享 Transform
        model.dirtyMask = MaskGrid::create(false);
        model.dirtyMask->setTransform(model.sdfGrid->transformPtr());
    }
    return model;
}
```

**构建复杂度**：O(表面积/D_v²) — 仅构建窄带 SDF，零面元开销。

### 3.2 多几何支持

| 类型 | SDF 构建方式 | 备注 |
|------|-------------|------|
| Box | `createLevelSetBox` | OpenVDB 原生 |
| Cylinder | 程序化网格 → `meshToLevelSet` | OpenVDB 无 `createLevelSetCylinder` |
| Mesh | `meshToLevelSet` | 标准接口 |

---

## 4. CuttingEngine v2：延迟面元化集成

### 4.1 双轨切削修订流程（4-Phase）

```
CuttingEngine::cut(billet, toolSDF):

  Phase 1: SDF CSG Difference + 脏区标记
    for each voxel in toolBBox (±1 dilation):
        oldVal = sdf[ijk]
        newVal = max(oldVal, -toolSDF.eval(worldPos))
        if (newVal != oldVal):
            sdf[ijk] = newVal
            // 脏区判据：从内部变为表面附近
            if (oldVal <= 0 && abs(newVal) < bandWidth):
                dirtyMask[ijk] = ON

  Phase 2: Lazy Surfel Generation
    for each ON voxel in dirtyMask:
        if (microGrid already has surfels in this voxel):
            skip  // 避免重复
        生成 N×N 精细面元（刀具表面解析采样）
        批量注入 microGrid
    clear dirtyMask

  Phase 3: Existing Surfel Clip
    for each surfel in microGrid within toolBBox:
        if toolSDF.eval(surfel.pos) <= 0:
            surfel.active = false

  Phase 4: Prune
    pruneLevelSet(sdfGrid)
```

### 4.2 Phase 2 面元生成细节

```cpp
void generateSurfelsForDirtyVoxels(
    BilletModel& billet, const ToolSweepSDF& tool)
{
    auto& xform = *billet.sdfGrid->transformPtr();
    double D_v = xform.voxelSize()[0];
    double d_v = billet.config.d_v;
    int N = billet.config.N;

    std::vector<Vec3R> newPoints;
    std::vector<Vec3f> newNormals;

    for (auto iter = billet.dirtyMask->cbeginValueOn(); iter; ++iter) {
        Coord voxelCoord = iter.getCoord();
        Vec3d center = xform.indexToWorld(voxelCoord);

        // 刀具表面法向（解析梯度）
        Vec3d grad = tool.gradient(center);
        double gl = grad.length();
        if (gl < 1e-10) continue;
        Vec3d normal = grad / gl;

        // 切平面坐标系
        Vec3d u = (abs(normal.x()) < 0.9) ?
            Vec3d(1,0,0).cross(normal) : Vec3d(0,1,0).cross(normal);
        u.normalize();
        Vec3d v = normal.cross(u);

        // N×N 精细采样 (间距 = d_v)
        double step = d_v;
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                Vec3d offset = ((i+0.5-N/2.0)*step)*u
                             + ((j+0.5-N/2.0)*step)*v;
                Vec3d candidate = center + offset;

                // 牛顿投影到刀具零等值面
                double dist = tool.eval(candidate);
                candidate -= dist * normal;

                // 验证
                if (abs(tool.eval(candidate)) < 0.5*d_v) {
                    newPoints.push_back(candidate);
                    newNormals.push_back(Vec3f(normal));
                }
            }
        }
    }

    // 批量注入
    if (!newPoints.empty()) {
        if (!billet.microGrid) {
            billet.microGrid = points::createPointDataGrid<
                NullCodec, PointDataGrid>(newPoints, xform);
            // 附加 normal + active 属性...
        } else {
            // 追加到现有 Grid...
        }
    }
    billet.dirtyMask->clear();
}
```

---

## 5. SurfelGenerator 模块

独立模块，负责 "给定表面定义 + 采样参数 → 面元点集"。

### 5.1 接口

```cpp
class SurfelGenerator {
public:
    struct SurfelBatch {
        std::vector<Vec3R> positions;
        std::vector<Vec3f> normals;
    };

    // 在刀具零等值面采样（切削暴露面）
    static SurfelBatch sampleToolSurface(
        const ToolSweepSDF& tool,
        const std::vector<Coord>& dirtyVoxels,
        const Transform& xform,
        double d_v, int N);

    // 在原始几何表面采样（如需保留未切削面）
    static SurfelBatch sampleGeometrySurface(
        const GeometryDef& geo,
        const CoordBBox& region,
        const Transform& xform,
        double spacing);
};
```

### 5.2 各场景采样策略

| 场景 | 采样方式 | 法向来源 |
|------|----------|----------|
| 切削暴露面 (主路径) | 切平面 N×N + 牛顿投影 | `tool.gradient()` 解析 |
| Box 未切削面 (可选) | 6 面 2D 均匀网格 | 轴对齐常量 |
| Cylinder 未切削面 | (θ, z) 参数空间 | 解析 (cosθ, sinθ, 0) |
| Mesh 未切削面 | 面积权重 + 重心坐标 | 面片法向插值 |

注：未切削面面元仅在需要计算残余材料轮廓误差时才生成。常规切削仿真中完全不需要。

---

## 6. 性能分析

### 6.1 Build 阶段对比

| 指标 | 现有方案 | v1.0 (几何采样) | **v2.0 (延迟)** |
|------|---------|----------------|-----------------|
| 时间 | O(Surface/D_v² × N²) | O(Surface/d_v_init²) | **O(Surface/D_v²)** |
| 面元 | 全表面粗面元 | 全表面粗面元 | **零** |
| 内存 | SDF + 面元 | SDF + 面元 | **仅 SDF + MaskGrid** |
| 500mm Box 示例 | ~2s, 200MB | ~100ms, 50MB | **<10ms, 3MB** |

### 6.2 Cut 阶段（单次切削，刀具 R=10mm, 行程 400mm）

| 指标 | 现有方案 | **v2.0** |
|------|---------|----------|
| Phase 1 (SDF) | ~5ms | ~5ms (相同) |
| Phase 2 (面元生成) | — | ~8ms (脏区 ~800 体素 × N²) |
| Phase 3 (面元裁剪) | ~15ms (遍历所有面元) | ~2ms (仅脏区内面元) |
| 总计 | ~20ms | **~15ms** |

### 6.3 帧时间均匀性

**潜在问题**：延迟方案首次切削时有 Phase 2 开销。

**缓解措施**：
1. 单次脏区体素数受刀具 BBox 限制（< 1000 体素）
2. 每体素 N² 面元采样是纯计算（~10μs/体素）
3. 总延迟 = 1000 × 10μs = 10ms — 远低于 16ms 帧预算
4. 可选分帧策略：每帧处理 K 个脏体素，将 spike 分摊到多帧

---

## 7. 一致性约束与安全边距

### 7.1 宏观 SDF 筛选加 dilation

```cpp
// 脏区检测及面元裁剪时扩展 1 voxel 安全边距
CoordBBox expandedBBox = toolBBox;
expandedBBox.expand(1);
```

### 7.2 宏微一致性（自然保证）

延迟方案下一致性是天然的：
- 面元仅在"SDF 从负变为零附近"时生成 → 必然在窄带内
- 从未被切削的体素不会有面元 → 无悬浮面元
- 无需 post-hoc 一致性检查

### 7.3 面元去重

```cpp
// 注入前检查：该体素是否已有面元
if (microGrid && pointCountInVoxel(microGrid, voxelCoord) > 0) {
    // 已有面元 → 仅执行 Phase 3 裁剪
    continue;
}
```

---

## 8. 迁移路径

| 步骤 | 改动 | 影响范围 |
|------|------|----------|
| 1 | `YggTypes.h`: 新增 `GeometryDef`, `dirtyMask` 字段 | 类型定义 |
| 2 | `BilletBuilder.cpp`: 删除面元生成代码，仅保留 SDF 构建 | 大幅简化 |
| 3 | 新增 `SurfelGenerator.h/.cpp` | 新文件 |
| 4 | `CuttingEngine::cutDualTrack`: 集成脏区标记 + 调用 SurfelGenerator | 核心修改 |
| 5 | 更新测试 | test_billet.cpp, test_cutting.cpp |

**SINGLE_TRACK 路径完全不变。**

---

## 9. 测试策略

```cpp
TEST(BilletBuilder_v2, DualTrack_BuildProducesNoSurfels) {
    auto cfg = solveResolution(0.01, 50.0, 5.0, {500,500,200});
    ASSERT_EQ(cfg.mode, DUAL_TRACK);
    auto billet = buildBillet(cfg, {0,0,0}, {500,500,200});
    EXPECT_EQ(billet.microGrid, nullptr);
    EXPECT_NE(billet.sdfGrid, nullptr);
    EXPECT_NE(billet.dirtyMask, nullptr);
}

TEST(CuttingEngine_v2, DualTrack_CutGeneratesSurfels) {
    auto cfg = solveResolution(0.01, 50.0, 5.0, {500,500,200});
    auto billet = buildBillet(cfg, {0,0,0}, {500,500,200});

    CuttingEngine engine;
    engine.cut(billet, ToolSweepSDF(BALL_END, 10, 0, 50, {50,250,0}, {450,250,0}));

    EXPECT_NE(billet.microGrid, nullptr);
    EXPECT_GT(pointCount(billet.microGrid->tree()), 0);
}

TEST(CuttingEngine_v2, DualTrack_NoDuplicateSurfels) {
    // 同一区域切两刀，面元数不应翻倍
    auto billet = ...;
    engine.cut(billet, tool1);
    auto count1 = pointCount(billet.microGrid->tree());
    engine.cut(billet, tool1);  // 同位置再切一次
    auto count2 = pointCount(billet.microGrid->tree());
    EXPECT_EQ(count1, count2);  // 无新增
}

TEST(CuttingEngine_v2, DualTrack_UncutRegionNoSurfels) {
    // 远离切削的体素不应有面元
}
```

---

## 10. 风险与缓解

| 风险 | 影响 | 缓解措施 |
|------|------|----------|
| 首次切削 latency spike | 帧时间波动 | 实测 <10ms；可分帧处理 |
| GeometryDef 需持久保留 | 内存 | Box/Cyl 仅几十字节；Mesh 保留引用 |
| 多次切削同区域面元膨胀 | 性能 | 注入前查重（体素级） |
| 刀具曲率极大处采样不均 | 精度 | 自适应细分（曲率阈值加密）|
| `createLevelSetCylinder` 不存在 | Cylinder 支持 | `meshToLevelSet` + 程序化柱面网格 |
| 未切削面无面元导致体积计算不准 | 精度报告 | 体积用 `LevelSetMeasure` 直接从 SDF 算（已实现）|

---

## 修订记录

| 版本 | 日期 | 修订内容 |
|------|------|----------|
| v1.0 | 2026-06-04 | 首次发布：双轨并行 + 几何原生采样 |
| v2.0 | 2026-06-04 | 核心重设计：延迟面元化，废弃 IPW₀ 预生成策略 |
