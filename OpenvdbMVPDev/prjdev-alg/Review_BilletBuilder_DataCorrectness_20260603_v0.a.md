# BilletModel 构建代码审查：数据正确性

**审查范围**：`BilletBuilder.cpp` vs `Algo_R2_BilletBuilder_20260603_v0.f.md`  
**日期**：2026-06-03  
**限定**：仅毛坯构建阶段，不涉及刀具/切削

---

## 摘要

毛坯构建代码存在 **2 个阻塞级数据正确性问题**：
1. `active` 标记用 uint8 属性而非 AttributeGroup，导致面元激活状态不可被 PointDelete/ActiveFilter 消费
2. IPW₀ 面元注入采用全局面采样替代设计要求的逐体素注入，导致部分表面体素可能无面元

---

## 问题 1：active 标记 — uint8 属性 vs AttributeGroup

| | 设计 (§4.7, v0.f) | 代码 (BilletBuilder:173-178) | 匹配 |
|---|---|---|---|
| 存储方式 | `AttributeGroup("active")` | `TypedAttributeArray<uint8_t>` | ❌ |
| PointDelete 消费 | ✅ `deleteFromGroup(tree, "active", true)` | ❌ 不可用 |
| ActiveFilter 消费 | ✅ `pointCount(tree, ActiveFilter())` | ❌ 返回全部点（当作无 group） |
| 内存 | 1 bit/点 (bitmask) | 8 bit/点 | ❌ |

**导致的问题链**：
```
buildBillet 阶段：
  whActive->set(i, 1)  ← uint8 属性写入
  ActiveFilter 找不到 "active" group → 返回 all points
  MemoryStats.activePointCount == pointCount  ← "巧合正确"

切削阶段（后续）：
  PointDelete 无法基于 uint8 属性删除 → 面元永不清理
  每切一刀，面元数单调增长 → 越切越慢
```

**修复方案**：不在 PointDataGrid 中注册 "active" 为属性，改为注册 Group：
```cpp
// Step 4: 不 append "active" 属性
// Step 5: 不写入 whActive->set()

// 改为在 createPointDataGrid 之后：
openvdb::points::appendGroup(microGrid->tree(), "active");
// 新注入的面元默认属于 "active" group（由 createPointDataGrid 行为保证）

// MemoryStats 的 ActiveFilter 会正确识别
// 切削时 PointDelete 可基于 group 执行清理
```

---

## 问题 2：IPW₀ 面元注入 — 全局面采样 vs 逐体素注入

### 设计意图 (§4.4-4.5)

```
for each leaf in sdfGrid:
    for each voxel on surface:
        injectSurfels_IPW0(voxel, d_v_init)
        → N_init² 面元，放置在体素表面切线面上
```

**关键**：每个表面体素独立注入，保证 `N_init²` 面元/体素。

### 当前代码 (BilletBuilder:87-129)

```
generateIPW0Surfels:
    for each of 6 faces:
        按 d_v_init 间距均匀采样整面
        不分体素边界
```

### 后果

对于 `D_v=6.4`, `d_v_init=6.4`（N_init=1，每体素 1 面元）的 30×30×20mm 毛坯：

```
设计（逐体素）：
  Z=0 面 → 约 4×4=16 个表面体素 → 16 面元
  Z=20 面 → 16 面元
  ...6 面总计约 40 面元
  每个表面体素恰好 1 面元 ✅

代码（全局面采样）：
  Z=0 面 30×30mm → nu=ceil(30/6.4)=5, nv=5 → 25 面元
  但 createPointDataGrid 将这 25 个面元分配到 voxel grid
  体素边界与采样网格不对齐 → 某些体素 0 面元，某些 2-3 面元 ❌
```

**精确场景**：
- 面元 (0, 0, 0) → 体素 (0, 0, 0)
- 面元 (6.4, 6.4, 0) → 体素 (1, 1, 0)（恰在体素边界）
- 面元 (12.8, 12.8, 0) → 体素 (2, 2, 0)
- 面元 (19.2, 19.2, 0) → 体素 (3, 3, 0)

这 4 个面元分布到 4 个体素，但面元 (6.4, 6.4, 0) 恰在体素 (1,1,0) 的边界——createPointDataGrid 内部对边界点的体素分配是不确定的。若分配到 (0,0,0) 或 (2,2,0)，则 (1,1,0) 成为空体素。

**空体素对切削的影响**：
```
cuttingTraversal:
  for each surface-voxel in sdfGrid:  ← FloatGrid 遍历所有表面体素
      surfels = microGrid.getPoints(voxel)  ← 空体素返回 0 个面元
      if surfels.empty():
          continue  ← 静默跳过！该体素的材料永不被切削
```

### 修复方案

将 `generateIPW0Surfels` 改为逐体素注入，对齐设计 §4.5：

```cpp
void injectSurfels_IPW0(PointDataGrid& microGrid, const Coord& voxel,
                        double d_v_init, const FloatGrid& sdfGrid) {
    double D_v = microGrid.transform().voxelSize()[0];
    int N_init = std::max(1, static_cast<int>(std::floor(D_v / d_v_init)));
    
    // 从 SDF 梯度估算法向
    Vec3d grad = sdfGradient(sdfGrid, voxel);
    Vec3d normal = grad.normalized();
    
    // 在体素表面切线面上采样 N_init² 点
    Vec3d voxelCenter = microGrid.transform().indexToWorld(voxel);
    Vec3d tangentU, tangentV;
    buildTangentBasis(normal, tangentU, tangentV);
    
    double step = D_v / N_init;
    Vec3d corner = voxelCenter - (D_v/2) * (tangentU + tangentV);
    
    for (int i = 0; i < N_init; ++i)
        for (int j = 0; j < N_init; ++j) {
            Vec3d p = corner + step * (i + 0.5) * tangentU 
                             + step * (j + 0.5) * tangentV;
            // 将 p 投影到 SDF 零等值面
            p = projectToZeroSurface(sdfGrid, p);
            appendSurfel(microGrid, voxel, p, normal, COARSE);
        }
}
```

---

## 问题 3：`d_v_init` 公式偏差

| | 设计 | 代码 |
|---|---|---|
| 公式 | `d_v_init = max(10*d_v, D_v)` | `d_v_init = min(minDim/4, max(10*d_v, D_v))` |
| 含义 | 取 max，保证最粗不超过体素 | 额外 clamp，保证每面至少 4×4 |

**分析**：
- `minDim/4 < max(10*d_v, D_v)` 的情况仅出现在极小毛坯或多边形毛坯的窄面
- 此时代码生成更密的面元（更多），比设计保守但无精度损失
- 属防御性 clamp，非错误

**建议**：文档统一后改为 `d_v_init = max(10 * d_v, D_v)`，不引入额外 clamp。

---

## 已正确的部分

| 检查项 | 状态 |
|--------|------|
| SINGLE_TRACK 路径 | ✅ 完整正确 |
| FloatGrid SDF 值 | ✅ 标准 AABB SDF 公式，内部负/外部正 |
| narrow-band halfWidth=3 | ✅ |
| DUAL_TRACK 共享 Transform | ✅ FloatGrid 和 PointDataGrid 共用 |
| 面元属性注册 (normal, precision) | ✅ Vec3f + uint8 |
| 面元法向方向 | ✅ API 朝外 |
| d_v_init 自适应范围 | ⚠️ 有偏差但不影响精度 |

---

## 修复优先级

| P | 问题 | 工时 | 阻断项 |
|---|------|------|--------|
| **P0** | active: uint8→Group | 30min | PointDelete 可用性 |
| **P0** | IPW₀: 全局采样→逐体素注入 | 2h | 面元分布正确性 |
| P2 | d_v_init 公式统一 | 5min | — |

---

## 修订记录

| 版本 | 日期 | 修订内容 |
|------|------|----------|
| v0.a | 2026-06-03 | 初稿创建 |
