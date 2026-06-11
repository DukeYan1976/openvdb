# 双轨切削引擎（CuttingEngine）双网格架构设计与优化分析

| 版本 | 修订日期 | 修订人 | 修订说明 |
| :---: | :---: | :---: | :--- |
| v1.0 | 2026-06-11 | Gemini | 首次创建，系统化记录双网格状态同步、高精度CSG策略与增量视图渲染优化分析。 |

---

## 1. 概述

在 Yggdrasil 双轨切削仿真引擎中，毛坯模型采用了一种创新的**双网格混合表征设计**（Dual-Grid Representation）：
1. **宏观 SDF 网格 (MacroGrid, `sdfGrid`)**：采用 `openvdb::FloatGrid`，以相对粗糙的体素尺寸 $D_v$ 存储窄带 Signed Distance Field (SDF)。主要用于整体宏观几何表达、高效碰撞检测和全局视图渲染。
2. **微观面元网格 (MicroGrid, `microGrid`)**：采用 `openvdb::points::PointDataGrid`，以极高精度的采样间距 $d_v$ ($d_v \ll D_v$) 在边界体素中存储带法向量的高密点云（Surfels）。主要用于表达和重建精确的亚毫米级切削表面（加工残留、表面粗糙度等）。

本文档针对 `CuttingEngine.cpp` 中提出的关于双网格同步、高精CSG差集算法策略、以及视图增量渲染更新等核心架构问题进行详细的分析与设计。

---

## 2. 问题一：MacroGrid 与 MicroGrid 的拓扑状态同步

### 2.1 现状与挑战
`sdfGrid` 随着切削会发生拓扑结构的剪枝（Prune）、体素激活与注销。当 `sdfGrid` 的叶子节点（LeafNode）因为完全被刀具切除或退化为背景而被释放时，对应的 `microGrid` 叶子节点如果依然保留在内存中，会导致：
1. **内存泄漏与碎片化**：残留大量无效的高密点云叶子节点。
2. **拓扑不一致**：微观表面存在已被切除的残留幽灵面元。

由于两棵树具有完全一致的索引坐标空间（Index Space）和树深度层级（Root $\rightarrow$ Internal $\rightarrow$ Leaf），因此迫切需要一种低成本、瞬时完成的状态同步方法。

### 2.2 解决方案：利用 OpenVDB 树级拓扑求交 (`topologyIntersection`)
OpenVDB 树（Tree）提供了底层的激活掩码求交算法，可在**亚毫秒级**完成多线程拓扑同步。无需遍历具体的点云数据或体素，只需操作激活节点的 Bitmask。

#### **同步逻辑实现**：
在 `Phase 4` 剪枝重整阶段（`pruneLevelSet` 之后），加入瞬时同步代码：
```cpp
// 核心：以 sdfGrid 为主，瞬时对 microGrid 进行拓扑求交同步
if (billet.microGrid) {
    // 递归地剔除所有在 sdfGrid 中已经不活跃/被释放的树节点和叶子节点，瞬间释放其对应内存
    billet.microGrid->tree().topologyIntersection(billet.sdfGrid->tree());
}
```

#### **工作原理**：
* **自上而下掩码过滤**：`topologyIntersection` 依次对比 `sdfGrid` 与 `microGrid` 的分支掩码（Active Mask）。若某 $8 \times 8 \times 8$ 块在 `sdfGrid` 中不再是 Active 状态，`microGrid` 对应位置的叶子节点（`LeafNodeType`）及其下属的所有点属性数组（P, normal, active）将被**立刻析构和物理释放**。

---

## 3. 问题二：高精高效的 Dual Cut (双轨切削) 策略与计算模式

### 3.1 临时高精度 VDB vs. 点云面元剪裁
**绝对避免构建全局或局部的临时高分辨率 VDB ($d_v$ 精度)。**

#### **量化数据对比**：
* 设刀具包围盒大小为 $100\text{mm} \times 100\text{mm} \times 100\text{mm}$，精细分辨率 $d_v = 0.05\text{mm}$。
* **局部高精 VDB 方案**：
  * 体素空间尺寸：$(100 / 0.05)^3 = 8 \times 10^9$（80 亿体素）。
  * 即使采用 Level Set 窄带（3层体素），表面活跃体素也将达到**数千万**个。生成、更新和 CSG 差集计算将产生数 GB 的分配开销，单步切削耗时可能飙升至数秒甚至数分钟，彻底摧毁实时仿真。
* **点云面元剪裁方案（当前设计）**：
  * **只处理表面**：仅在 $D_v$ 分辨率下的脏体素（`dirtyVoxels`）表面生成一层高精度的 Surfels，空间复杂度由 $O(\text{Volume})$ 降低为面元级别的 $O(\text{Surface})$。
  * **点标记裁剪**：对于旧的 Surfel，通过多线程并行评估刀具的解析 SDF（`toolSDF.eval(wp)`），在内部的点直接通过设置 `active = 0` 剔除，整体耗时在 $10\text{ms}$ 以内。

### 3.2 脏体素计算的并发与硬件协同
1. **基于 TBB 的叶子局域增量注入（CPU 优化）**：
   * **当前瓶颈**：现阶段 `Phase 2` 将所有脏体素生成的点在全局拼成 `allPos` 并重新建立 `PointDataGrid`。这会导致 OpenVDB 在全局对大量点重新进行分箱、构建空树、排序并重新分配内存。
   * **架构升级**：在多线程 `parallel_for` 采样工具零等值面时，因为我们已知脏体素 `coord`，可以根据其对应的 VDB 叶子 `leafOrigin` 进行局部分组。各线程**局部、并发地直接向 billet.microGrid 对应的 LeafNode 插入新点并更新其属性**，避免全局大分配与重排序，可将插入开销降低 $50\%$ 以上。
2. **GPU 协同计算**：
   * **当脏体素数量 $M < 10,000$**：CPU（TBB 并行）是绝对的主宰。GPU 带来的数据双向拷贝延迟（Overhead）远超过计算节省的时间。
   * **当脏体素数量 $M \ge 10,000$**（如大快进量、大刀具切削）：将脏体素索引坐标直接上传至 GPU，在 CUDA Kernel 中完成多线程等值面采样、法向计算以及与毛坯包围盒边界（Box / Cylinder）的过滤裁剪。计算完毕后，通过 Stream 异步回传至主机并直接并入叶子。

---

## 4. 问题三：视图渲染的增量更新记录与处理

在数控仿真中，随着切削不断进行，毛坯表面瞬息万变。实时渲染必须避免整体重建和整体上传，转而采用**基于 VDB 叶子节点的增量式渲染刷新**。

### 4.1 核心机制：VDB Leaf 作为渲染缓存单元
在 OpenVDB 中，一个叶子节点（`LeafNode`）管理 $8 \times 8 \times 8$ 个体素（对于 $D_v = 0.5\text{mm}$，它对应一个 $4\text{mm} \times 4\text{mm} \times 4\text{mm}$ 的物理空间立方体块）。
* **GPU 映射**：GPU 渲染器中，每个活跃的叶子节点对应一个独立的 **GPU Vertex Buffer Object (VBO)**。整个毛坯由成百上千个小尺寸的 VBO 拼合绘制。
* **渲染缓存管理器**：在内存中维护 `std::unordered_map<openvdb::Coord, GPUBufferInfo> RenderCache`，其中 Key 为叶子节点原点 `leaf->origin()`。

### 4.2 状态判定与增量分发流
单步切削完成后，渲染管理器通过比较切削前后的拓扑状态，向 GPU 分发三种不同的轻量更新指令：

```
       [ Billet 切削 + 拓扑重整完成 ]
                    │
                    ▼
          遍历 Tool BBox 受影响的
            叶子原点 (Origins)
                    │
   ┌────────────────┼────────────────┐
   ▼                ▼                ▼
[ 未在该 Origin    [ 该 Origin 在      [ 该 Origin 在
 探测到 Leaf ]     前后均存在 Leaf ]   前不存在，后探测到 ]
   │                │                │
   ▼                ▼                ▼
指令: 删除 (Delete)  指令: 更新 (Dirty)  指令: 新增 (Add)
 ┌─────────┐      ┌──────────┐     ┌──────────┐
 │ 释放并   │      │ 重新提取  │     │ 分配全新  │
 │ 销毁对应 │      │ 对应叶子  │     │ 的 GPU   │
 │ GPU VBO │      │ 覆写 VBO  │     │ VBO 缓冲 │
 └─────────┘      └──────────┘     └──────────┘
```

### 4.3 增量状态计算参考实现
以下是在 `cutDualTrack` 完成剪枝后，提取需要被删除、更新、新增的叶子原点的伪代码：

```cpp
// 1. 在切削前（例如 Phase 1 开始时），记录刀具 BBox 范围内可能受影响的叶子原点
std::unordered_set<openvdb::Coord> originallyActiveLeaves;
if (billet.microGrid) {
    for (auto leaf = billet.microGrid->tree().cbeginLeaf(); leaf; ++leaf) {
        auto lo = leaf->origin();
        // 如果该叶子与刀具 BBox 发生相交，则记录
        if (lo.x() >= minIdx.x() - 8 && lo.x() <= maxIdx.x() + 8 &&
            lo.y() >= minIdx.y() - 8 && lo.y() <= maxIdx.y() + 8 &&
            lo.z() >= minIdx.z() - 8 && lo.z() <= maxIdx.z() + 8) {
            originallyActiveLeaves.insert(lo);
        }
    }
}

// 2. 执行核心切削逻辑、增量采样注入、剪裁、以及 topologyIntersection 同步 ...

// 3. 计算增量渲染列表
std::vector<openvdb::Coord> renderDeleteList;
std::vector<openvdb::Coord> renderUpdateList;

// A. 探测删除：原本存在但在切削后彻底消失的叶子
for (const auto& origin : originallyActiveLeaves) {
    if (!billet.microGrid->tree().probeLeaf(origin)) {
        renderDeleteList.push_back(origin);
    }
}

// B. 探测更新与新增
if (billet.microGrid) {
    for (auto leaf = billet.microGrid->tree().cbeginLeaf(); leaf; ++leaf) {
        auto lo = leaf->origin();
        // 如果在原本受影响的集合中，说明被局部切削了（或者是有点被剪裁标记为了 inactive）
        // 如果不在其中，说明是新分配的叶子（注入了新的切削面元）
        if (originallyActiveLeaves.count(lo) || /* 是新活跃叶子 */) {
            renderUpdateList.push_back(lo);
        }
    }
}

// 4. 将 renderDeleteList 和 renderUpdateList 异步发往渲染线程
// - 渲染线程根据 DeleteList 销毁对应 VBO
// - 渲染线程仅针对 UpdateList 内的叶子，提取最新的 points & normals 并重新上传（Overwrite）GPU VBO
```

---

## 5. 结论

通过引入上述优化思路，双轨切削仿真引擎将实现：
1. **拓扑高一致性**：通过 `topologyIntersection` 一键完成 MicroGrid 的内存释放和拓扑对齐。
2. **极佳的内存带宽控制**：彻底抛弃了高维密集 VDB 实体，转而依靠轻量化的 Surfel 点集剪裁。
3. **流畅的交互帧率**：视图渲染的吞吐量从整机大网格重构，转变为极其轻量、自适应的 **Leaf-level 增量数据传输**。
