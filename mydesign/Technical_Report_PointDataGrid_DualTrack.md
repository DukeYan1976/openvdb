# OpenVDB PointDataGrid 与 DualTrack 算法技术报告

**报告日期**: 2026-06-08
**分支**: feature/ygg-app
**调研范围**: OpenVDB PointDataGrid 全部功能、大规模数据处理能力、DualTrack 算法可行性评估

---

## 1. 执行摘要

本报告对 OpenVDB PointDataGrid 进行了深度技术调研，并对当前项目中的 DualTrack 双轨分辨率切削算法进行了代码级可行性分析。

**核心发现**:
- **PointDataGrid** 是 OpenVDB 专为大规模点云设计的高性能稀疏数据结构，支持十亿级点处理、多属性压缩存储、TBB 并行处理、以及内存映射延迟加载
- **DualTrack 算法** 在概念上正确实现了双轨分辨率分离（宏观 D_v + 微观 d_v），4-phase 流水线设计合理，但存在 **injectSurfels 全量重建导致的 O(n²) 性能退化** 和 **大工件内存爆炸** 两个关键工程问题
- **生产就绪度**: 小工件（<100mm）可用；中工件需监控；大工件高精度切削和实时预览当前不可行

---

## 2. OpenVDB PointDataGrid 功能全景

### 2.1 核心功能架构

PointDataGrid 采用与标准 VDB 相同的 3 层树结构（RootNode → InternalNode → LeafNode），每个叶节点管理 8×8×8 = 512 个体素。其核心设计目标是在保持 VDB 空间稀疏性的同时，高效存储和管理大规模点云的异构属性数据。

#### 属性存储系统

| 特性 | 说明 |
|------|------|
| **多属性支持** | 每个点可同时存储位置(P)、ID、速度、颜色、字符串、组标记等任意属性 |
| **压缩编解码器** | NullCodec（无压缩）、TruncateCodec（float→half）、FixedPointCodec（8/16位定点）、UnitVecCodec（16位向量量化） |
| **统一值优化** | 当属性值全部相同时自动退化为单值存储，内存占用降为 O(1) |
| **组压缩** | 组信息用位图存储，每组占 1 bit，8 个组共用一个 uint8_t |

#### 空间索引与查询

- **体素偏移数组**: 叶节点体素值存储累积点索引偏移量，通过体素坐标可 O(1) 定位点索引范围 `[start, end)`
- **索引迭代器家族**: IndexAllIter（全遍历）、IndexOnIter（活跃体素）、IndexOffIter（非活跃体素）、IndexVoxelIter（指定体素）
- **过滤系统**: NullFilter、GroupFilter、ValueMaskFilter、MultiGroupFilter 等多种过滤器组合

#### 点云处理算法

| 算法 | 功能 |
|------|------|
| `createPointDataGrid` | 从点数组创建 PointDataGrid |
| `pointSample` / `boxSample` / `quadraticSample` | 从 VDB 网格采样到场属性 |
| `rasterizeSdf` | 将点光栅化为 SDF（支持球体/椭球体核） |
| `movePoints` / `advectPoints` | 自定义变形器移动点 / 速度场平流 |
| `evalMinMax` / `evalAverage` / `accumulate` | 并行归约统计 |
| `uniformPointScatter` / `denseUniformPointScatter` | 均匀/稠密点云生成 |
| `pca` | 主成分分析计算邻域各向异性 |

### 2.2 数据模型与内存布局

```
PointDataGrid
  └── PointDataTree
        └── PointDataLeafNode
              ├── BaseLeaf: 体素值缓冲区（累积偏移量）
              ├── AttributeSet
              │     ├── Descriptor: 属性描述符
              │     └── AttributeArray[]: 属性数组列表
              └── ValueMask: 活跃体素掩码
```

**关键设计**:
1. **属性数组分离**: 位置、速度、ID 等属性存储在独立数组中，共享相同索引顺序
2. **位置精度**: 默认使用 FixedPointCodec 将体素空间位置量化为 8/16 位定点数（范围 -0.5~0.5 体素），精度受限于体素大小
3. **延迟加载**: 支持内存映射文件和页式加载（1MB/页），可处理超出物理内存的数据集
4. **Blosc 压缩**: 对属性数组进行块压缩，通常可达 2-10 倍压缩比

### 2.3 并行处理架构

PointDataGrid 全面基于 Intel TBB 实现并行：

| 组件 | 并行模式 | 用途 |
|------|---------|------|
| `LeafManager` | `tbb::parallel_for` | 叶节点级并行遍历 |
| `parallel_reduce` | `tbb::parallel_reduce` | 统计归约（计数、最值、求和） |
| `tbb::spin_mutex` | 细粒度锁 | 属性数组读写保护 |

**线程安全设计**:
- 读操作天然线程安全
- 写操作通过 `tbb::spin_mutex` 保护
- Copy-on-Write: 非 const 访问属性数组时自动深拷贝
- `stealAttributeSet` + `replaceAttributeSet` 模式是标准线程安全做法

---

## 3. PointDataGrid 大规模数据处理能力

### 3.1 已知能力

| 指标 | 能力 |
|------|------|
| **点数量级** | 十亿级（64 位索引，单叶节点可存超过 40 亿点） |
| **内存效率** | 稀疏存储 + Blosc 压缩（2-10x）+ Uniform 值优化 |
| **Out-of-Core** | 内存映射 + 页式延迟加载，可处理超物理内存数据集 |
| **并行扩展** | 接近线性加速比（叶节点级并行） |

### 3.2 已知限制

| 限制 | 影响 |
|------|------|
| **叶节点大小固定** | 8×8×8 体素不可调整，极端密度不均场景可能造成内存碎片 |
| **位置精度** | FixedPointCodec 精度受限于体素大小，亚体素精度需用 NullCodec |
| **组数量** | 每组属性数组 uint8_t，最多 8 个组/数组 |
| **字符串属性** | 字符串存储在全局元数据中，大量唯一字符串导致元数据膨胀 |
| **并行写入** | 同一属性数组的多线程并发修改需外部同步 |
| **构建开销** | 需先生成 PointIndexGrid 再转换，两步构建对动态更新不友好 |

### 3.3 与常规 VDB Grid 对比

| 特性 | PointDataGrid | 常规 VDB Grid |
|------|---------------|---------------|
| 存储内容 | 点属性数组（异构记录） | 标量/向量场值 |
| 叶节点值 | 点索引偏移量 | 实际场值 |
| 属性系统 | 多属性、多类型、可压缩 | 单值类型 |
| 空间查询 | 点迭代器 + 过滤器 | 值访问器 + 插值 |
| 拓扑操作 | 不支持（需先光栅化） | 膨胀/腐蚀/布尔运算 |
| 适用场景 | 粒子、点云、LiDAR | 距离场、密度场、速度场 |

---

## 4. DualTrack 算法可行性分析

### 4.1 算法设计概述

DualTrack 是一种双轨分辨率切削算法，核心思想是在保持微观表面精度的同时降低宏观 SDF 的内存开销。

**三大核心机制**:

1. **CSG 差集原理** (CuttingEngine.cpp:506-507):
   ```cpp
   float csgSdf = std::max(billetSdf, toolSdf);  // billet ∩ toolᶜ
   ```
   标准的水平集布尔差集操作。

2. **增量注入机制** (CuttingEngine.cpp:373-399):
   - 仅在刀具实际切削区域（dirty voxels）生成高分辨率面元
   - 面元延迟创建，`microGrid` 初始为 `nullptr`

3. **Dirty Mask 机制** (CuttingEngine.cpp:361-368):
   - 使用 `MaskGrid` 标记已处理体素
   - Phase 4 清除，为下一次切削做准备

**数据结构**:

| 组件 | 类型 | 精度 | 作用 |
|------|------|------|------|
| `sdfGrid` | FloatGrid | D_v (粗) | 宏观距离场 |
| `microGrid` | PointDataGrid | d_v (细) | 面元点云（精细表面） |
| `dirtyMask` | MaskGrid | D_v | 标记已处理区域 |

分辨率关系: `D_v = N × d_v` (N = 2^n, n≥1)

### 4.2 四阶段实现分析

#### Phase 1: Dirty Voxel 检测 (CuttingEngine.cpp:309-371)

**流程**:
1. 激活刀具 BBox 内 inactive 的内部体素
2. TBB 并行 SDF 更新（LeafManager + enumerable_thread_specific）
3. 去重合并 dirty voxels

**Dirty 判定条件** (第352行):
```cpp
if (toolDist <= 0.0 && std::abs(newVal) < bandWidth)
```

**评估**: ✅ 正确捕获刀具穿透体素；⚠️ `std::abs(newVal) < bandWidth` 可能遗漏深穿透但远离新表面的体素。

#### Phase 2: Surfel 生成与注入 (CuttingEngine.cpp:373-399)

**Chunked 采样策略**: 每批 256 个 dirty voxels，控制峰值内存。

**SurfelGenerator 实现** (SurfelGenerator.cpp:6-53):
1. 计算刀具表面法线（梯度归一化）
2. 构建切平面基底 (u, v)
3. N×N 均匀采样
4. Newton 投影到零等值面
5. 验证: `|tool.eval(candidate)| < 0.5 * d_v`

**injectSurfels 实现** (CuttingEngine.cpp:218-292):
- 若 `microGrid` 已存在，先收集所有现有点
- 追加新点，**重建完整 PointDataGrid**（createPointDataGrid）
- 附加 `normal` 和 `active` 属性

**评估**:
- ✅ Chunked 策略和 Newton 投影设计合理
- 🔴 **关键缺陷**: `injectSurfels` 是**全量重建**，时间复杂度 O(n²)，多次切削后性能严重退化

#### Phase 3: 并行裁剪 (CuttingEngine.cpp:401-445)

**实现**: LeafManager 并行遍历面元叶子节点，`stealAttributeSet` 进行独占访问，对工具内部的点标记 `active = 0`。

**评估**: ✅ 正确的并行裁剪逻辑，标准的 PointDataGrid 线程安全做法；⚠️ `toolSDF.eval(wp) <= 0.0` 是硬截断，可能导致锯齿状边界。

#### Phase 4: Prune + Clear (CuttingEngine.cpp:447-451)

```cpp
openvdb::tools::pruneLevelSet(sdfGrid->tree());
billet.dirtyMask->clear();
```

**评估**: ✅ 正确重置状态，为下一次切削做准备。

### 4.3 并行策略评估

| 位置 | 并行原语 | 用途 | 安全性 |
|------|----------|------|--------|
| Phase 1 | `tbb::parallel_for` + LeafManager | SDF 更新 | ✅ 安全 |
| Phase 1 | `tbb::enumerable_thread_specific` | Dirty 收集 | ✅ 安全 |
| Phase 3 | `tbb::parallel_for` + LeafManager | 面元裁剪 | ✅ 安全 |

**串行瓶颈**:
- Phase 1 激活体素: 三重循环串行（范围有限，影响小）
- Phase 2 注入: **全量重建 PointDataGrid**，主要瓶颈
- Phase 4 prune: 内部可能串行

### 4.4 性能特征

**时间复杂度**:

| 操作 | 复杂度 | 说明 |
|------|--------|------|
| SDF 更新 | O(m × n) | m=刀具 BBox 体素数 |
| Dirty 收集 | O(m) | 与 SDF 更新合并 |
| Surfel 生成 | O(d × N²) | d=dirty voxels 数 |
| Surfel 注入 | O(p) | p=总面元数（全量重建）|
| 裁剪 | O(p) | 并行 |

**内存特征 (DualTrack vs SingleTrack)**:

| 指标 | SingleTrack | DualTrack |
|------|-------------|-----------|
| SDF 精度 | d_v | D_v = N×d_v |
| SDF 内存 | (L/d_v)³ | (L/D_v)³ = (L/d_v)³ / N³ |
| 面元内存 | 0 | ~dirty_voxels × N² × point_size |
| 总内存（稀疏区域） | 高 | 低 |
| 总内存（密集切削） | 低 | 高（面元膨胀）|

**关键观察**: 当 `N=128, d_v=0.005mm` 时，每个 dirty voxel 生成 `128² = 16384` 个面元，PointDataGrid 的 leaf 开销（~2400 bytes/leaf）在大工件中会导致内存爆炸。

### 4.5 潜在缺陷与边界情况

| 缺陷 | 位置 | 严重程度 |
|------|------|----------|
| **全量重建注入** | CuttingEngine.cpp:263 | 🔴 高 — O(n²) |
| **大工件内存爆炸** | 测试用例注释 | 🟡 中 — leaf 开销未摊平 |
| **精度属性缺失** | 禁用测试 | 🟡 中 — FINE/COARSE 未实现 |
| **Newton 投影失败** | SurfelGenerator.cpp:44 | 🟢 低 — 有 fallback |
| **硬截断锯齿** | Phase 3 | 🟢 低 — 可用过渡带优化 |

**边界情况**:
1. **擦切（grazing cut）**: `toolDist <= 0.0` 要求严格内部接触，tangent touch 可能漏切
2. **多次切削重叠**: dirtyMask 防止重复生成，逻辑正确但效率可优化
3. **梯度除零**: 刀具轴线附近梯度为零时跳过，可能出现空洞
4. **空切削**: 刀具完全在工件外时正确处理

### 4.6 测试覆盖评估

| 测试文件 | 测试数 | 覆盖内容 |
|----------|--------|----------|
| `test_cutting.cpp` | 7 | 单轨/双轨基础、体积、多刀 |
| `test_billet.cpp` | 5 | 构建、延迟初始化、性能 |
| `test_e2e.cpp` | 2 | 完整流水线、槽铣 |
| `test_resolution.cpp` | 8 | 分辨率计算、模式判定 |

**覆盖缺口**:
- 多线程并发切削未验证
- 大 N 值（N>64）行为未知
- 非球头刀（FLAT_END/BULL_NOSE）未覆盖
- 边界注入测试被禁用
- microGrid 重建体积与单轨一致性未验证

### 4.7 生产就绪度评估

| 场景 | 状态 |
|------|------|
| 小工件（<100mm）切削 | ✅ 可用 |
| 中工件（100-500mm）切削 | ⚠️ 需监控内存 |
| 大工件（>500mm）高精度切削 | ❌ 不可行（内存爆炸）|
| 多刀连续切削 | ⚠️ 性能退化（O(n²) 注入）|
| 实时预览（>30fps） | ❌ 不可行 |

---

## 5. PointDataGrid 与 DualTrack 的协同潜力

### 5.1 当前 DualTrack 对 PointDataGrid 的使用

当前实现使用 PointDataGrid 存储 surfel（面元），每个 surfel 携带 `position`、`normal`、`active` 属性。这是一种合理的设计选择，因为：

1. **空间稀疏性**: 仅在切削区域存储面元，未切削区域零内存开销
2. **属性灵活性**: 可扩展存储颜色、粗糙度、温度等加工相关属性
3. **并行处理**: Phase 3 裁剪天然利用 LeafManager 并行

### 5.2 可优化的结合点

| 优化方向 | PointDataGrid 能力 | DualTrack 收益 |
|----------|-------------------|----------------|
| **增量注入** | `merge` 或 leaf-level 追加 | 消除 O(n²) 全量重建 |
| **内存压缩** | Blosc 压缩 + Uniform 值优化 | 降低大工件内存开销 |
| **组过滤** | GroupFilter 位图过滤 | 快速区分 FINE/COARSE 面元 |
| **光栅化验证** | `rasterizeSdf` | 验证 microGrid 表面质量 |
| **统计查询** | `evalAverage` / `accumulate` | 快速计算面元密度、法线一致性 |

### 5.3 增量注入的可行性

OpenVDB PointDataGrid 当前不直接支持两个 PointDataGrid 的增量合并，但可通过以下方式实现：

1. **Leaf-level 追加**: 遍历新面元的 leaf，对已有 microGrid 的对应 leaf 执行 `stealAttributeSet` → 追加点索引 → `replaceAttributeSet`
2. **双缓冲策略**: 维护两个 microGrid（读/写），切削完成后原子交换
3. **批量构建**: 积累多批 dirty voxels 后一次性构建，减少重建频率

---

## 6. 结论与建议

### 6.1 PointDataGrid 结论

OpenVDB PointDataGrid 是一个成熟、高性能的大规模点云数据结构，其空间稀疏性、多属性压缩、TBB 并行和延迟加载能力，完全满足 DualTrack 面元存储的需求。PointDataGrid 的十亿级点处理能力和灵活的属性系统，为 DualTrack 的后续扩展（多属性面元、温度场耦合等）提供了坚实基础。

### 6.2 DualTrack 结论

DualTrack 算法在**概念上正确**，4-phase 流水线设计合理，成功实现了双轨分辨率分离。但当前工程实现存在两个关键问题：

1. **injectSurfels 全量重建**（🔴 P0）: 导致多刀切削 O(n²) 性能退化，必须实现增量注入
2. **大工件内存爆炸**（🟡 P1）: PointDataGrid 的 leaf 开销在大规模稀疏面元场景下无法摊平，需实施 batch-per-leaf 策略或降低 N 值

### 6.3 行动建议

| 优先级 | 行动 | 预期收益 |
|--------|------|----------|
| P0 | 重构 `injectSurfels` 为 leaf-level 增量追加 | 消除 O(n²)，支持连续切削 |
| P1 | 添加 `precision` 属性（uint8_t）区分 FINE/COARSE | 验证双轨精度价值 |
| P1 | 实施 batch-per-leaf 注入策略（每 leaf ≥64 点）| 摊平 leaf 开销 |
| P2 | 提供基于 microGrid 的精确体积计算 | 消除粗 SDF 50% 体积误差 |
| P2 | 启用并修复禁用测试（边界注入、大工件内存）| 完善测试覆盖 |

---

**报告结束**
