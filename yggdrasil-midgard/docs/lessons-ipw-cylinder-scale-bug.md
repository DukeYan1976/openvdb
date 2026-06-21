# IPW Cylinder Scale Bug: 根因分析与经验总结

## 问题描述

Cylinder 毛坯的 IPW 体素显示比实际 mesh 小一半（直径 = 半径而非直径，高度 = 实际高度的一半）。Box 毛坯正常。

## 根因

`openvdb::tools::meshToVolume` 底层 API 的 `QuadAndTriangleDataAdapter::getIndexSpacePoint()` 要求返回 **index space**（格网索引空间）坐标，不是 world space 坐标。

源码注释明确标注（`MeshToVolume.h:187`）：
```cpp
/// @brief  Returns position @a pos in local grid index space
void getIndexSpacePoint(size_t n, size_t v, Vec3d& pos) const;
```

OpenVDB 的便利 wrapper 函数（接受 `std::vector<Vec3T>` + `Transform`）会自动调用 `worldToIndex()` 转换顶点坐标，但我们的代码直接构造 `QuadAndTriangleDataAdapter` 并传入 **world space** 坐标，跳过了这个转换。

### 缩放路径

```
world vertex (50, 25, 0) → 被 meshToVolume 当作 index (50, 25, 0)
→ indexToWorld(50, 25, 0) = (25, 12.5, 0)   [× 0.5 voxel size]
→ 实际 IPW 体素在世界空间缩小了一半
```

## 为什么 Box 一直正确？

Box 使用 `openvdb::tools::createLevelSetBox(bbox, *transform, halfWidth)`，它接受 **world space** 的 `BBox`，并在内部使用 transform 正确处理坐标系转换。Cylinder 使用 `meshToVolume` 走的是完全不同的代码路径。

**教训**：同一接口的不同实现可能有不同的坐标系约定，不能假设一致性。

## 修复

在构建 `QuadAndTriangleDataAdapter` 之前，用 `xform->worldToIndex()` 将每个顶点从 world space 转换到 index space：

```cpp
auto toIndex = [&](double x, double y, double z) -> openvdb::Vec3s {
    openvdb::Vec3d idx = xform->worldToIndex(openvdb::Vec3d(x, y, z));
    return openvdb::Vec3s(float(idx.x()), float(idx.y()), float(idx.z()));
};
```

## 调试过程中的弯路

### 弯路 1：怀疑 transform 被修改

最初怀疑 `meshToVolume` 修改了 grid 的 transform。添加诊断日志验证后发现 transform 完全正确：
```
voxel=0.5000  idx0=(0.00, 0.00, 0.00)
```
→ `meshToVolume` **不修改** transform。

### 弯路 2：怀疑 mesh 和 IPW 使用了不同的参数

发现 IPW 重建和 mesh 渲染之间存在时序竞态——mesh 在上一帧用旧参数重建，IPW 在当前帧用新参数重建。修复同步问题后参数完全一致，但视觉仍不对。

**教训**：参数匹配 ≠ 几何匹配。即使输入参数完全相同，如果数据转换过程中有不同的坐标系解释，输出仍会不同。

### 弯路 3：在 mesh 生成器中排查

花了大量时间对比 `BilletMeshGenerator::buildCylinder` 和 `IPWBuilder` 的顶点位置、三角形 winding、法线方向等。它们完全一致。

**教训**：当几何输入一致但结果不同时，bug 在数据处理的中间层，不在输入层。

## 关键警示

OpenVDB 的 `meshToVolume` **底层 API** 和 **便利 wrapper** 对坐标系的期望不同：

| API | 坐标空间期望 |
|-----|-------------|
| `meshToVolume(vertices, triangles, xform)` 便利函数 | World space → 内部自动 `worldToIndex` |
| `meshToVolume(QuadAndTriangleDataAdapter(...), xform)` 底层函数 | Index space → **不做** world→index 转换 |

直接使用底层 API 时必须自行完成 world→index 转换。

## 同步修复（额外发现）

时序竞态：`sceneRenderer.render()` 在 `ipwDirty` 块之前调用，导致 mesh 渲染落后 IPW 一帧。

**修复**：将 ipwDirty 块（含 `rebuildBillet` + `build IPW`）移到 `sceneRenderer.render()` 之前，并将 mesh 重建提取为独立的 `SceneRenderer::rebuildBillet(const GeometryDef&)`，在 ipwDirty 块中显式调用。

## 修改文件汇总

| 文件 | 变更 | 性质 |
|------|------|------|
| `core/IPWBuilder.cpp` | CYLINDER case: 顶点加 `worldToIndex()` 转换 | **Bug fix** |
| `core/IPWBuilder.h` | `buildMacroGrid` 移除 `logFn` 参数 | 清理 |
| `app/renderers/SceneRenderer.h` | 新增 `rebuildBillet(const GeometryDef&)` | 同步修复 |
| `app/renderers/SceneRenderer.cpp` | 提取 `rebuildBillet`；`renderBillet` 仅检查 `billetDirty_` | 同步修复 |
| `app/main.cpp` | ipwDirty 块移到 render 之前；调用 `rebuildBillet` | 同步修复 |

## 排查原则

1. **用诊断日志验证假设**：不要依赖推理——transform 日志直接证明了 transform 没问题
2. **消除时序不确定性**：将相关的数据更新聚拢到同一个代码块中
3. **读第三方库源码**：OpenVDB 的 `getIndexSpacePoint` 注释提供了关键线索
4. **"为什么这个能工作，那个不能？"**：Box 能工作是因为用了不同的 API，追踪这个差异找到了根因
