# Phase 2 设计文档批判性审查（基于现有代码）

**文档编号**：`Review_Phase2_Design_20260603_v0.b.md`  
**状态**：已更新（Spike 完成）  
**作者**：Duke / Jarvas  
**日期**：2026-06-03  
**审查范围**：`TaskPlan_Phase2`, `Phase2_DualGrid_Design`, `YGGDRASIL_DUAL_GRID_ARCH` vs 现有代码 + PointDataGrid Spike 验证

---

## 第一部分：结构级缺陷（🔴 高）

### R1. BilletModel 缺少 microGrid 成员

**位置**: `yggdrasil/types/YggTypes.h`

当前定义：
```cpp
struct BilletModel {
    openvdb::FloatGrid::Ptr sdfGrid;   // 只有一个 Grid
    ResolutionConfig config;
    Vec3d origin, dims;
    bool isSingleTrack() const { return config.mode == ResolutionConfig::SINGLE_TRACK; }
};
```

Phase 2 设计假定：
```cpp
BilletModel {
    sdfGrid(D_v),      // FloatGrid 宏观
    microGrid(D_v),    // PointDataGrid 微观 ← 不存在
    ...
}
```

**严重度**: 🔴 阻断级。不扩展 BilletModel，Phase 2 全部任务无法落地。且设计文档中 4 个不同文档对 microGrid 的类型描述不一致：
- `Phase2_DualGrid_Design` §3.1：`PointDataGrid`
- `YGGDRASIL_DUAL_GRID_ARCH` §1.3：`openvdb::points::PointDataGrid`
- `TaskPlan_Phase2` T6.2：测试代码用 `billet.microGrid->tree()`

需统一为 `openvdb::points::PointDataGrid::Ptr`。

---

### R2. buildBillet 完全不处理 DUAL_TRACK 模式

**位置**: `yggdrasil/core/BilletBuilder.cpp:9-11`

```cpp
double voxelSize = config.d_v;
if (config.mode == ResolutionConfig::DUAL_TRACK)
    voxelSize = config.D_v;  // 仅换了体素尺寸，其他没做
```

当前 `buildBillet` 在 DUAL_TRACK 模式下：
- ✅ 用 D_v 替换体素尺寸
- ❌ 不创建 PointDataGrid
- ❌ 不注册 SurfelAttributes
- ❌ 不注入 IPW₀ 粗面元
- ❌ 不检测 microGrid 创建失败

**严重度**: 🔴 阻断级。Sprint 6 的 T6.2/T6.3/T6.4 全依赖这个函数的扩展。

**修正建议**: 分两步走：
1. T6.2 新增 `createPointDataGrid(sharedTransform)` 辅助函数，返回 `PointDataGrid::Ptr`
2. T6.3 新增 `injectIPW0Surfels(microGrid, sdfGrid, config)` 辅助函数
3. T6.4 在 `buildBillet` 中拼接双轨逻辑

---

### R3. CuttingEngine 零感知 PointerDataGrid

**位置**: `yggdrasil/core/CuttingEngine.cpp`

当前 `cut()` 仅对 FloatGrid 做 `max(billet, -tool)` CSG 布尔。Phase 2 设计要求的 5 阶段流水线：

```
Phase 1: 宏观过滤（遍历 FloatGrid 窄带 + 刀具 BBox）
Phase 2: 微观面元剥离（遍历 microGrid active surfels）
Phase 3: 边界面元注入（N² 采样，写入 microGrid）
Phase 4: 宏观 SDF 保守更新（activeCount==0 → SDF=+D_v）
Phase 5: 内存统计（MemoryStats）
```

当前代码只做到了 Phase 1 的一半（BBox 内遍历 + SDF 更新），其余 **4.5 个阶段代码为 0**。

**严重度**: 🔴 阻断级。这是 Sprint 7 的核心工作，但设计文档将 5 个 Phase 的接口契约、调用顺序、异常处理全部压在伪代码中，代码层面零基础。

---

### R4. MemoryStats 不存在

设计文档多次引用的核心类型，当前代码仓库中无定义、无声明、无测试。

**严重度**: 🔴 阻断级。Sprint 6 第一个任务 T6.1 就依赖它，且 Phase 2 所有测试用例（T6.4, T7.5, T8.3）都用它验证。

---

### R5. ResolutionSolver 降级策略与架构文档冲突

| 来源 | 降级条件 | 行为 |
|------|----------|------|
| 代码 `ResolutionSolver.cpp:20-26` | `N_ideal < 1`（即 D_upper < d_v） | 强制 SINGLE_TRACK |
| 架构文档 §2.2 | `N < 32` | 回退单轨 |
| Phase 2 设计 | 按内存预算判定 | SINGLE_TRACK / DUAL_TRACK / ATLAS_REGION |

代码中**没有 N<32 降级逻辑**，且 `solveResolution` 在 DUAL_TRACK 判定时只看内存预算是否超限。这意味着小件也可能被判定为 DUAL_TRACK（如果内存预算设得极小），或大件在特定参数下错失 DUAL_TRACK。

**严重度**: 🔴 逻辑不一致。需统一为单一真理源。

---

## 第二部分：设计级缺陷（🟡 中）

### R6. PointDataGrid API 假设 — Spike 已验证 ✅

**Spike 位置**: `/tmp/ygg_spike/spike_pointdatagrid.cpp`  
**验证日期**: 2026-06-03  
**编译**: ✅ (OpenVDB 13.0 + TBB 12.17)  
**运行**: ✅ 全部 7 个测试通过

| 测试项 | 结果 | 关键发现 |
|--------|------|----------|
| [1] FloatGrid ↔ PointDataGrid 共享 Transform | ✅ | `createLinearTransform(D_v)` + `setTransform()` 双轨共用，索引对齐零开销 |
| [2] `createPointDataGrid` 创建点云 | ✅ | 惰性分配叶节点，面元可按需注入 |
| [3] 自定义属性注入 (steal/modify/replace) | ✅ | **关键参数**: `replaceAttributeSet(ptr, true)` — 必须传 `allowMismatchingDescriptors=true` |
| [4] `probeLeaf` + 叶遍历 | ✅ | `probeLeaf(Coord)` 探测已填充体素，`beginLeaf()` 遍历所有叶节点 |
| [5] `Grid::memUsage()` | ✅ | 1000 点 ≈ 320KB，可直接用于 MemoryStats |
| [6] `AttributeHandle` 读写 | ✅ | steal/replace 三步走模式稳定可靠 |
| [7] 面元去激活机制 | ⚠️ | PointDelete 基于 Group 机制，不支持 uint8 属性过滤 |

**对设计的直接影响**:

1. **`active` 标记必须改用 `AttributeGroup`**（而非设计文档中的 `uint8_t` 属性）：
   ```cpp
   // ❌ 设计文档原方案（不可行）
   attrSet->appendAttribute("active", uint8_t);
   
   // ✅ Spike 验证的正确方案
   auto groupHandle = leaf->groupWriteHandle("active");
   groupHandle->set(offset, true);  // 激活
   groupHandle->set(offset, false); // 去激活（= PointDelete 可直接消费）
   ```
   
2. **面元注入的 API 模式** — 三步走必须带 `allowMismatchingDescriptors=true`：
   ```cpp
   auto attrSet = leaf->stealAttributeSet();
   attrSet->appendAttribute("normal", TypedAttributeArray<Vec3f>::attributeType(), 1);
   leaf->replaceAttributeSet(attrSet.release(), /*allowMismatchingDescriptors=*/true);
   ```

3. **`MemoryStats` 实现极简** — `Grid::memUsage()` 直出，无需复杂包装。

**严重度**: 🟢 已解决。Spike 已验证全部 API 可行性。

---

### R7. 类型不一致：DoubleGrid vs FloatGrid

| 来源 | 宏观轨类型 |
|------|----------|
| 架构文档 §1.2 | `openvdb::DoubleGrid` |
| 代码全量 | `openvdb::FloatGrid` |
| Phase 2 设计 §2.1 | `FloatGrid(D_v)` |

代码已经用了 FloatGrid（`buildBillet` 返回 `FloatGrid::Ptr`），架构文档却指定 DoubleGrid。如果架构文档是权威源，整个 Phase 1 代码需要类型迁移——成本巨大且无收益（微米级 SDF 不需要 double 精度）。

**严重度**: 🟡 建议。**明确以代码（FloatGrid）为准，修正架构文档**。

---

### R8. 面元注入的延迟触发机制定义模糊

设计文档 §4.4 的 `injectOnDemand` 描述：

```
if |sdfVal| < halfWidth * D_v:
  补注入粗面元
else:
  注入精细面元
```

两个关键边界情况未定义：

1. **首次接触但为外部体素**（`|sdfVal| > halfWidth * D_v`）— 刀具可能触及未在窄带内的区域？如果 BilletBuilder 的窄带宽度足够（3*halfWidth），不应发生。但多段切削后 SDF 更新可能扩大窄带范围——此时新窄带体素可能无 microGrid 数据。

2. **体素已有点数据但部分面元被剥离**——重新注入时是追加还是覆盖？追加会导致重复面元，覆盖会丢失未剥离的面元。

**严重度**: 🟡 逻辑不完整。建议明确"体素首次接触"的严格定义：`microGrid 该体素无任何点数据`。

---

### R9. 宏观 SDF 保守更新会引入过滤误判（二次风险）

设计文档 §4.3 Phase 4：

```
activeCount == 0 → SDF = +D_v
activeCount > 0  → 保持原值不变
```

这是 Phase 1 Review 中 R4-1 修复后的方案（`SDF = max(billet, -tool)`）。但在双轨中新增了一个问题：

**场景**: 一个体素内 1 个面元被剥离，99 个存活 → `activeCount > 0` → SDF 保持原值。但该体素在下一段切削的宏观过滤中被标记为 `affectedVoxels`，重新遍历全部 100 个面元——包括 99 个已确认存活的面元。

**这是正确行为（不会误判），但不是高效行为**。如果 10 段切削后一个体素从未被完全清空，每次都要遍历全部面元。这是设计文档自己承认的"不追求性能"阶段，但应该在 Risk 章节明确标注。

**严重度**: 🟡 性能风险，正确性无问题。

---

### R10. BilletBuilder 窄带宽度硬编码为 3

**位置**: `BilletBuilder.cpp:11` — `float halfWidth = 3.0f;`

窄带宽度 = `halfWidth * voxelSize = 3 * D_v`。对于 D_v=6.4mm（大件粗精度），窄带仅 ~19.2mm，毛坯内部深层体素可能不在窄带内（SDF 被 clamp 到 -bandWidth）。

这**在单轨中无问题**（只关心表面），但在双轨中 **Phase 3 边界面元注入** 需要接触深层体素——这些体素可能不在当前窄带中，导致 `activeCount > 0` 但 SDF 已 clamp 到背景值。

**严重度**: 🟡 需验证。建议在 `buildBillet` DUAL_TRACK 模式下使用更大的 halfWidth（至少 5），或在首次切削时动态扩展窄带。

---

## 第三部分：测试覆盖缺口（🟢 低 / 工程债）

### R11. 零双轨测试覆盖

当前 5 个测试文件的 20+ 测试用例，**0 个使用 DUAL_TRACK 模式**：

```
test_resolution.cpp: 6 tests — 仅 verify 模式判定，不测试 DUAL_TRACK 下的实际行为
test_billet.cpp:     4 tests — 全部 SINGLE_TRACK
test_toolsdf.cpp:    6 tests — 无模式区分
test_cutting.cpp:    4 tests — 全部 SINGLE_TRACK
test_e2e.cpp:        2 tests — 全部 SINGLE_TRACK
```

Sprint 6-8 的 TDD 节奏要求先写 [RED] 测试，但当前没有双轨 scaffold 来支撑这些测试的编译。

**严重度**: 🟢（Phase 2 自然解决，但需先搭 scaffold）

---

### R12. computeVolume 只算单轨

```cpp
double computeVolume(const openvdb::FloatGrid::Ptr& grid);
```

双轨的体积应基于 PointDataGrid 的活跃面元数 × 微体素体积（而非 FloatGrid 的粗体素），需要新增重载或新函数。

**严重度**: 🟢（Sprint 8 才需要）

---

## 第四部分：架构文档与实施文档冲突

### R13. 3 份文档对同一概念的定义不一致

| 概念 | Phase2_DualGrid_Design | TaskPlan_Phase2 | 架构文档 |
|------|----------------------|-----------------|----------|
| 宏观轨类型 | FloatGrid | FloatGrid | DoubleGrid |
| 面元精度标记 | `precision: uint8_t` (0=COARSE,1=FINE) | (无定义，测试中用 `prec == FINE`) | (无) |
| N_min 阈值 | 无 | 无 | N<32 降级 |
| 体积计算 | 未提及 | 未提及 | 未提及 |

**严重度**: 🟡 真理源分裂。建议 Phase 2 实施前做一次文档合并。

---

## 第五部分：可立即执行的工作（无阻断）

### 立即可做

| 优先级 | 工作 | 状态 |
|--------|------|------|
| P0 | 扩展 `BilletModel` 增加 `microGrid` 成员 | 待执行 |
| P0 | 创建 `MemoryStats` 类型（头文件 + 实现） | 待执行 |
| P1 | ~~PointDataGrid API Spike 验证脚本~~ | ✅ 已完成 2026-06-03 |
| P1 | 统一 `Phase2_DualGrid_Design` 与代码的类型声明 | R1 完成前暂缓 |
| P1 | 更新设计文档：`active` 改为 `AttributeGroup`（R6 Spike 结论） | 新增 |

### 需先解决 R1-R5 才能开始

| 依赖阻断 | Sprint |
|-----------|--------|
| R1 + R4 → 解锁 | Sprint 6: T6.1 T6.2 |
| R1 + R6 → 解锁 | Sprint 6: T6.3 T6.4 |
| R3 + R8 + R9 → 解锁 | Sprint 7: 全部 |
| R2 + R10 → 解锁 | Sprint 7: T7.2 T7.3 |

---

## 第六部分：总结

| 等级 | 数量 | 关键项 |
|------|------|--------|
| 🔴 阻断 | 5 | R1 (microGrid), R2 (buildBillet), R3 (CuttingEngine), R4 (MemoryStats), R5 (降级冲突) |
| 🟡 中等 | 4 | R7 (DoubleGrid), R8 (延迟注入), R9 (性能), R10 (窄带) |
| 🟢 低/已解决 | 4 | R6 ✅ (Spike验证), R11 (测试), R12 (体积), R13 (文档) |

**核心判断**: Phase 2 设计在算法层面是自洽的，但**代码基础为零**——BilletModel、buildBillet、CuttingEngine 三个核心入口完全不认识"双轨"。这不是"在 Phase 1 基础上扩展"，而是需要 **重构 Phase 1 的接口契约**（BilletModel 结构、CuttingEngine 方法签名）之后才能开始 Sprint 6。

**建议节奏**:
1. 先做 P0 基础设施（R1 + R4）—— 1 个工作日
2. ~~PointDataGrid Spike（R6）~~ ✅ 已完成，全部 API 验证通过
3. 然后按 Sprint 6 → 7 → 8 推进

**Spike 关键结论（R6）**:
- `active` 标记 → 改用 `AttributeGroup`（非 uint8 属性）
- `replaceAttributeSet` → 必须传 `allowMismatchingDescriptors=true`
- `MemoryStats` → `Grid::memUsage()` 直出
- 面元注入 → `createPointDataGrid` + leaf 逐个迁移

---

## 修订记录

| 版本 | 日期 | 修订内容 |
|------|------|----------|
| v0.a | 2026-06-03 | 初稿创建，基于 feature/ygg-app 代码审查 |
| v0.b | 2026-06-03 | R6 更新：Spike 全部通过，API 验证完成；新增 `active` 改为 `AttributeGroup` 的设计变更建议；修正 summary 等级分布 |
