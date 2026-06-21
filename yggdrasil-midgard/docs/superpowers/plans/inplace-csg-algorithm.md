# 自定义 In-Place CSG Difference 算法

> 版本：v0.a | 日期：2026-06-21

## 修订记录

| 版本 | 日期 | 说明 |
|------|------|------|
| 0.a | 06-21 | 初始设计，替代 rasterizeToolGrid + csgDifference + levelSetRebuild |

---

## 1. 动机

当前 Phase 0 流程（~100ms）：
```
rasterizeToolGrid (10ms) → csgDifference (30ms) → levelSetRebuild (60ms)
```

问题：
- rasterizeToolGrid 遍历全 bbox，含大量与 IPW 无关的空区域
- csgDifference 操作两棵树的全量合并
- levelSetRebuild 全局重建窄带（最贵，且对逐段切削过度保守）

目标：**in-place 直接修改 MacroGrid，只处理受影响的 voxel，省去临时 grid 和全局 rebuild**。

---

## 2. 算法设计

### 2.1 CSG Difference 的数学本质

```
new_sdf(P) = max(old_sdf(P), -tool_sdf(P))
```

对于 level set：
- old_sdf > 0: 材料外部（空气）
- old_sdf < 0: 材料内部
- tool_sdf < 0: 刀具内部（要切掉的区域）

`max(old, -tool)` 的效果：在刀具内部（tool<0 → -tool>0），new_sdf 被推正 → 变成空气。

### 2.2 哪些 Voxel 需要处理

受影响的 voxel 分三类：

| 类型 | 原状态 | 条件 | 操作 |
|------|--------|------|------|
| **A: Active boundary** | active, \|sdf\| < bg | tool bbox 内 | 更新 sdf = max(old, -tool) |
| **B: Interior (inactive负)** | inactive, value = -bg | tool 切入内部 | **必须激活为 leaf**，写入新 sdf |
| **C: Exterior (inactive正)** | inactive, value = +bg | 不受影响 | 跳过 |

**关键：类型 B 是 inactive 节点必须被"展开"为 leaf 的情况。**

### 2.3 OpenVDB 树结构与 Inactive 节点

```
Root → InternalNode (5级) → InternalNode (4级) → LeafNode (3级, 8³ voxels)
```

- Active voxel：显式存储在 LeafNode 中
- Inactive voxel：由父节点的 background 值隐式表示
- 当需要写入一个 inactive 位置时，OpenVDB 的 `Accessor::setValue()` 会**自动创建**所需的 InternalNode 和 LeafNode

**回答核心问题**：是的，写入 inactive 区域时，OpenVDB 会自动沿树路径创建节点直到 LeafNode。不需要手动创建——`accessor.setValue(coord, val)` 内部处理了节点分裂。

但需要注意：新创建的 LeafNode 初始状态是全 inactive（值=background）。我们写入后需要手动 `setValueOn(coord)` 标记为 active。

### 2.4 算法伪代码

```cpp
void inPlaceCSGDifference(
    openvdb::FloatGrid::Ptr& macroGrid,
    const ToolSweptSDF& tool)
{
    const auto& xform = macroGrid->transform();
    const double V = xform.voxelSize()[0];
    const double bg = macroGrid->background();  // halfwidth * V
    const int halfwidth = 3;
    
    // ── Step 1: 确定扫掠体 AABB 在 index space 的范围 ──
    openvdb::BBoxd toolBBox = tool.boundingBox();
    openvdb::Coord bboxMin = openvdb::Coord::floor(xform.worldToIndex(toolBBox.min()))
                             - openvdb::Coord(halfwidth);
    openvdb::Coord bboxMax = openvdb::Coord::ceil(xform.worldToIndex(toolBBox.max()))
                             + openvdb::Coord(halfwidth);
    openvdb::CoordBBox toolIdxBox(bboxMin, bboxMax);
    
    auto acc = macroGrid->getAccessor();
    
    // ── Step 2: 处理已有的 active leaf（类型 A）──
    // 遍历所有 leaf，只处理与 tool bbox 重叠的
    for (auto leaf = macroGrid->tree().beginLeaf(); leaf; ++leaf) {
        if (!leaf->getNodeBoundingBox().hasOverlap(toolIdxBox)) continue;
        
        for (auto iter = leaf->beginValueOn(); iter; ++iter) {
            openvdb::Coord coord = iter.getCoord();
            openvdb::Vec3d world = xform.indexToWorld(coord);
            double toolVal = tool.eval(world);
            double negTool = -toolVal;
            
            float oldVal = iter.getValue();
            float newVal = std::max((double)oldVal, negTool);
            
            if (newVal != oldVal) {
                iter.setValue(static_cast<float>(newVal));
            }
        }
    }
    
    // ── Step 3: 激活 interior voxel（类型 B）──
    // 扫掠体内部（tool < 0）如果 MacroGrid 在该处是 inactive 且值为 -bg（材料内部），
    // 说明刀具切入了实心区域，需要激活这些 voxel。
    //
    // 策略：在 tool bbox 范围内，对每个 voxel 检查：
    //   - 当前 inactive 且 value ≈ -bg（材料内部）
    //   - tool.eval(P) 接近零或为负（刀具在此处有影响）
    //   - 新值 = max(-bg, -tool) 落入 narrowband（|new| < bg）
    //
    // 只有满足以上条件才需要激活。

    for (int z = bboxMin.z(); z <= bboxMax.z(); ++z) {
        for (int y = bboxMin.y(); y <= bboxMax.y(); ++y) {
            for (int x = bboxMin.x(); x <= bboxMax.x(); ++x) {
                openvdb::Coord coord(x, y, z);
                
                // 跳过已 active 的 voxel（Step 2 已处理）
                if (acc.isValueOn(coord)) continue;
                
                // 获取 inactive 值
                float oldVal = acc.getValue(coord);
                
                // 只关心材料内部（oldVal ≈ -bg）
                if (oldVal >= 0) continue;  // 外部空气，刀具无法再切
                
                openvdb::Vec3d world = xform.indexToWorld(coord);
                double toolVal = tool.eval(world);
                double negTool = -toolVal;
                float newVal = static_cast<float>(std::max((double)oldVal, negTool));
                
                // 新值进入 narrowband → 需要激活
                if (std::abs(newVal) < bg) {
                    acc.setValue(coord, newVal);    // 自动创建 LeafNode
                    acc.setValueOn(coord);
                }
                // 新值仍为 -bg 级别 → 仍是内部，不激活
                // 新值 > bg → 被完全切掉变成外部（可标记但通常不需要显式存储）
            }
        }
    }
    
    // ── Step 4: 清理远离零面的 active voxel ──
    // 被完全切掉的 voxel（新 sdf > bg）应变为 inactive
    for (auto leaf = macroGrid->tree().beginLeaf(); leaf; ++leaf) {
        if (!leaf->getNodeBoundingBox().hasOverlap(toolIdxBox)) continue;
        
        for (auto iter = leaf->beginValueOn(); iter; ++iter) {
            float val = iter.getValue();
            if (val >= bg) {
                // 完全在外部，deactivate
                iter.setValueOff();
                iter.setValue(static_cast<float>(bg));
            }
        }
    }
    
    // ── Step 5: 剪枝空 leaf ──
    macroGrid->tree().prune();
}
```

### 2.5 Step 3 的性能问题

Step 3 仍然遍历 tool bbox 全范围（与 rasterizeToolGrid 类似）。优化：

**优化 A：只遍历刀具表面附近的 interior voxel**

新 voxel 需要激活的条件：`|max(-bg, -tool)| < bg` → `-bg < -tool < bg` → `|tool| < bg`

即只有 `|tool.eval(P)| < bg`（在刀具 narrowband 内）的位置才可能产生新 boundary voxel。

```cpp
// 优化：先粗筛——只检查可能在 tool narrowband 内的区域
// tool narrowband 厚度 = 2*bg ≈ 2*halfwidth*V = 3mm (for V=0.5, hw=3)
// 这大大缩小搜索范围（从体积搜索变为壳搜索）
```

**优化 B：利用 MacroGrid 的已有 active 区域反推**

新 boundary 只会出现在**已有 boundary 的邻域**——因为材料内部只有在靠近已有表面的地方才会被刀具首先接触到。

```cpp
// 对每个受影响的 active leaf 的邻居坐标检查
// 如果邻居是 inactive 且为 interior → 检查 tool 影响
```

这将搜索量从 O(bbox_volume) 降为 O(boundary_surface_area × 1_voxel_shell)。

### 2.6 与 levelSetRebuild 的对比

| | levelSetRebuild | 自定义 in-place |
|--|-----------------|-----------------|
| 全局操作 | ✓（遍历整棵树） | ✗（只遍历 tool bbox 内的 leaf） |
| 窄带修复 | 自动 | 手动 Step 3+4 |
| 拓扑一致性 | 保证 | 需自行保证（Step 4 清理） |
| 耗时 | ~60ms | 预估 5~15ms |
| 临时内存 | 分配新 grid | 零（in-place） |

### 2.7 正确性保证

| 不变量 | 如何维护 |
|--------|----------|
| Active voxel 的 \|sdf\| < bg | Step 4 deactivate 超出范围的 |
| 零面附近有完整 narrowband | Step 3 激活新 interior→boundary 转换 |
| 无孤岛 leaf | Step 5 prune 清理空 leaf |
| SDF 单调递增远离零面 | max(old, -tool) 保证：old 已满足，-tool 在远处→-bg，不破坏 |

### 2.8 OpenVDB Accessor 自动节点创建

确认：`acc.setValue(coord, val)` 对 inactive 区域的行为：
1. 查找 coord 所在的 LeafNode
2. 若不存在 → 沿树路径创建 InternalNode → 创建 LeafNode（初始值=tile value）
3. 写入 val 到指定 offset
4. `acc.setValueOn(coord)` 将该 voxel 标记为 active

**不需要手动创建节点**。OpenVDB 的 Tree 结构在首次写入时自动展开。

但有个效率细节：如果 tile 是 inactive 的负值（代表材料内部大块），首次写入会触发 tile→leaf 展开，leaf 的其他位置继承 tile 值但仍为 inactive。这是正确的——我们只激活需要的那几个 voxel。

---

## 3. 预估性能

Demo IT-1 (V=0.5, 40mm path, R=5):
- Step 2: ~5000 active boundary voxels eval → ~5ms
- Step 3: tool narrowband shell ≈ ~3000 voxels eval → ~3ms  
- Step 4: ~5000 voxels 值检查 → <1ms
- Step 5: prune → <1ms
- **总计：~10ms（vs 当前 100ms，10x 提升）**

---

## 4. 实施计划

1. 新建 `core/InPlaceCSG.h/.cpp`
2. 在 SimEngine 中替换 classifyVoxels 内的 rasterize+csg+rebuild
3. 测试：同一切削段，对比新旧方法的 MacroGrid 结果（active voxel count 应一致）
4. 性能 benchmark

---

## 5. OpenVDB 5-4-3 树结构

（参考：[OpenVDB Overview](https://www.openvdb.org/documentation/doxygen/overview.html)）

### 5.1 层级

```
RootNode          稀疏哈希表，无大小限制
 └── InternalNode<5>   32³ = 32768 个槽位
      └── InternalNode<4>   16³ = 4096 个槽位
           └── LeafNode<3>   8³ = 512 个 voxel（最底层）
```

### 5.2 每个节点管辖多少 voxels

从底往上累乘（官方文档原文：*"a block of voxels of size 32×16×8 = 4096 on a side"*）：

| 节点 | 每维管辖 voxels | 总管辖 |
|------|----------------|--------|
| LeafNode<3> | 8 | 512 |
| InternalNode<4> | 16 × 8 = 128 | 128³ ≈ 2M |
| InternalNode<5> | 32 × 128 = 4096 | 4096³ ≈ 69G |

### 5.3 槽位 = Child 或 Tile

InternalNode 每个槽位二选一：
- **Child**：指向下级节点
- **Tile**：一个值代表该槽位覆盖的全部 voxels（不展开）

Tile 的意义：如果一片区域值全相同（如材料内部全为 -bg），不创建子节点，一个 float 就够了。

### 5.4 最大 LeafNode 数

一个 InternalNode<5> 下：32768 × 4096 = **1.34 亿** 个 LeafNode（理论极限）。

实际：50×50×20mm @V=0.5mm → index 100×100×40 → 最多 **(100/8)×(100/8)×(40/8) = 845 个 LeafNode**。

### 5.5 Tile 分裂

`accessor.setValue(coord, val)` 写入 tile 覆盖区域时，OpenVDB 自动：
1. 在对应 InternalNode<4> 槽位创建 LeafNode
2. LeafNode 512 voxels 初始化为 tile 值，全 inactive
3. 被写入的 voxel 设新值，标 active

一次分裂产生一个 LeafNode。多级 tile 会触发多级分裂（全自动）。

### 5.6 Active Mask

Active/inactive 是 **voxel 级别**属性（LeafNode 内 512-bit mask），不是节点级别。一个 LeafNode 可以有任意数量的 active voxels（0~512）。

---

*待实现。*
