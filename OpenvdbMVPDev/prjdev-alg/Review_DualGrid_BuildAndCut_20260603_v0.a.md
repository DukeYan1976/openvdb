# Phase 2 DualGrid 毛坯构建 — 代码审查

**文档编号**：`Review_DualGrid_BuildAndCut_20260603_v0.a.md`  
**状态**：初稿  
**作者**：Jarvas  
**日期**：2026-06-03  
**审查范围**：`BilletBuilder.cpp`, `CuttingEngine.cpp`, `YggTypes.h` vs 设计文档

---

## 摘要

**核心洞见**：当前 DUAL_TRACK 实现存在 3 个阻塞级缺陷——`active` 标记用错类型导致 PointDelete 不可用、Phase 3 边界面元注入创建独立 Grid 导致 merge 异常、缺失宏观 SDF 同步更新导致后续切削漏检。这些问题在产品运行中表现为"性能退化"和"精度异常"，而非硬崩溃。

**必须立即修复**：Bug-1 (active→Group), Bug-5 (macroSDF update), Bug-3 (Phase3 merge 策略)

---

## Bug 清单

### 🔴 Bug-1: `active` 标记使用 uint8 属性而非 AttributeGroup

**文件**: `BilletBuilder.cpp:173-178`, `CuttingEngine.cpp:100-101,121,127`

**问题**:
```cpp
// ❌ 当前代码
attrSet->appendAttribute("active", TypedAttributeArray<uint8_t>::attributeType(), ...);
auto activeHandle = AttributeWriteHandle<uint8_t>::create(*activeArr);
if (activeHandle->get(idx) == 0) continue;
activeHandle->set(idx, 0);
```

**正确方案** (Spike R6 验证):
```cpp
// ✅ 应该使用 AttributeGroup
auto groupHandle = leaf->groupWriteHandle("active");
if (!groupHandle->get(offset)) continue;
groupHandle->set(offset, false);
// 面元清理: openvdb::points::deleteFromGroup(tree, "active", /*invert=*/true);
```

**影响**: 
- `PointDelete::deleteFromGroup()` 无法消费 uint8 属性值，只能消费 Group
- 被切除的面元永远无法被删除，内存单调增长
- 所有设计文档已修订为 AttributeGroup 方案，代码与文档不一致

**严重度**: 🔴 阻塞。Spike 已证明 uint8 方案不可行。

---

### 🔴 Bug-2: 无 inactive surfel 清理

**文件**: `CuttingEngine.cpp` — 缺失整个 `deleteInactiveSurfels()` 步骤

**问题**: Phase 2 微观面元剥离后，`active`=0 的面元留在 PointDataGrid 中。后续每次切削：
- 遍历所有面元（包括已切除的），O(总面元数) 增长
- 10 次切削后，切除面元累积量 >> 活跃面元，性能崩塌

**设计意图** (Algo_R4):
> 每次切削后对 PointDataGrid 执行 `deleteFromGroup(tree, "active", true)` 清理被切除面元。

**影响**: 性能退化（非功能错误）。用户感知为"越切越慢"。

**严重度**: 🔴 阻塞（依赖 Bug-1 修复）。

---

### 🔴 Bug-3: Phase 3 边界面元注入 — 独立 Grid + tree.merge 策略错误

**文件**: `CuttingEngine.cpp:231-266`

**问题 1 — 独立 Grid 的 Transform 坐标系不匹配**:
```cpp
newWorldPts.push_back(worldP);  // c 世界坐标
auto newPtGrid = createPointDataGrid<...>(newWorldPts, *microGrid->transformPtr());
```
`createPointDataGrid` 内部会把世界坐标转为索引空间存储。但合并后的新叶节点体素坐标可能与原 Grid 的叶节点不对齐。

**问题 2 — `tree().merge()` 的语义错误**:
```cpp
microGrid->tree().merge(newPtGrid->tree());
```
- `PointDataTree::merge()` 要求两个树的 descriptor 完全一致
- 新 Grid 的 descriptor 通过 `stealAttributeSet → appendAttribute` 构建，与原 Grid 通过 `createPointDataGrid → stealAttributeSet → appendAttribute` 构建的 descriptor 可能版本不同
- 即使 merge 成功，结果是 topology union，新叶节点会创建新的体素索引，导致后续遍历需要重新计算体素-点映射

**问题 3 — 新面元使用 `active` uint8 属性**:
```cpp
tAttr->appendAttribute("active", ...);  // ❌ 又是 uint8
```

**设计意图** (Algo_R4 §4.5):
> 在刀具 SDF 零等值面与体素的交界处直接注入精细面元到现有叶节点，通过 appendPoints 在同一叶节点内追加新点。

**正确方案**:
1. 找到部分切削的叶节点
2. 对该叶节点执行 `stealAttributeSet → resize → 写入新面元 → replaceAttributeSet`
3. 同一叶节点内新旧面元共存，descriptor 不变

**严重度**: 🔴 阻塞。merge 策略在边界情况下会失败或产生数据不一致。

---

### 🟡 Bug-4: 缺失宏观 SDF 更新 (Phase 4)

**文件**: `CuttingEngine.cpp:268`

**当前代码**:
```cpp
// Phase 4: pruneLevelSet  ← 只有这一行
openvdb::tools::pruneLevelSet(sdfGrid->tree());
```

**设计意图** (Algo_R4 §4.4):
```cpp
for each voxel in affectedVoxels:
    if all surfels removed:
        sdfGrid->setValue(voxel, +D_v)  // 标记为空气
    else:
        // 保守策略: 保持 SDF ≤ 0，不修改
```

**影响**:
- 体素内面元全被切除后，SDF 仍为负值
- 后续切削的宏观过滤（Phase 1）会误认为该体素仍有材料，重复计算
- 结果：宏观过滤失去加速效果，退化为全遍历

**严重度**: 🟡 高。功能正确性受影响，但不会crash。

---

### 🟡 Bug-5: Phase 3 面元精度标记错误

**文件**: `CuttingEngine.cpp:249`
```cpp
whP->set(i, 1); // ❌ 标记为 FINE=1，但代码注释说 "FINE"
```

这里 `precision=1` 确实表示 FINE。但检查 BilletBuilder.cpp：
```cpp
outPrecision.push_back(0); // COARSE=0 ✅
```
一致。precision: 0=COARSE, 1=FINE。标记本身是正确的。
**但**新面元的 precision 属性是通过 `createPointDataGrid` 创建的新 Grid 中 append 的，如果 merge 失败（Bug-3），这些面元也不会生效。

**严重度**: ✅ 无独立问题，合并到 Bug-3。

---

### 🟡 Bug-6: Phase 3 采样限制过于保守

**文件**: `CuttingEngine.cpp:198`
```cpp
int sampleN = std::min(N, 8);
```

**问题**: 对于 N=64（高精度），只采样 8×8=64 个点，而非 N²=4096。虽然注释说"避免过密"，但这牺牲了精度。设计文档 Algo_R4 §4.5 明确要求 N×N 采样。

**影响**: 切削边界精度不足。对于 N=64 的情况，新表面面元间距从 0.1mm 变为 0.8mm。

**建议**: 改为 `int sampleN = std::min(N, 16)` 或直接用 `N`，依赖后续去重/合并操作控制密度。

**严重度**: 🟡 中。精度损失，但不会崩溃。

---

### 🟢 Bug-7: `d_v_init` 公式偏离设计

**文件**: `BilletBuilder.cpp:137`
```cpp
// ❌ 代码
double d_v_init = std::min(minDim / 4.0, std::max(10.0 * config.d_v, config.D_v));

// ✅ 设计 (Algo_R2 §4.4)
double d_v_init = std::max(10.0 * config.d_v, config.D_v);
```

**分析**: 
- 对于 minDim=20, max(10*0.1, 6.4)=6.4 → 代码: min(5, 6.4)=5.0, 设计: 6.4。差异小。
- 对于 minDim=200, max(10*0.005, 0.64)=0.64 → 代码: min(50, 0.64)=0.64, 设计: 0.64。一致。
- 仅当 minDim/4 < max(10*d_v, D_v) 时触发，即小毛坯。此时代码会生成更多面元（更密），而非更少。

**影响**: 🟢 低。实际运行效果接近设计意图，但公式应统一。

---

### 🟢 Bug-8: BilletBuilder 两次叶节点遍历（浪费）

**文件**: `BilletBuilder.cpp:151-210`

Step 4 和 Step 5 各遍历一遍所有叶节点。可以合并为一次遍历：
```cpp
for (auto leaf = tree.beginLeaf(); leaf; ++leaf) {
    auto attrSet = leaf->stealAttributeSet();
    attrSet->appendAttribute(...);  // 分配空间
    // 直接写入值（appendAttribute 返回后可立即写）
    auto* nArr = attrSet->get("normal");
    auto whN = ...::create(*nArr);
    for (...) whN->set(i, normals[globalIdx++]);
    leaf->replaceAttributeSet(attrSet.release(), true);
}
```

**影响**: 🟢 低。仅在构建时一次，不影响运行时性能。

---

## 与设计文档的一致性核对

| 设计章节 | 设计要求 | 代码实现 | 状态 |
|----------|----------|----------|------|
| Algo_R2 §4.4 Step 2 | FloatGrid SDF 构建 | ✅ `buildBoxSDF(D_v, ...)` | ✅ |
| Algo_R2 §4.4 Step 3 | PointDataGrid 共享 Transform | ✅ `createPointDataGrid(..., *xform)` | ✅ |
| Algo_R2 §4.5 | IPW₀ 粗面元注入 d_v_init | ⚠️ 公式偏离，见 Bug-7 | ⚠️ |
| Algo_R2 §4.7 | active=AttributeGroup | ❌ 使用 uint8 属性 | ❌ Bug-1 |
| Algo_R4 §4.2 Phase 1 | 宏观过滤 | ⚠️ 遍历全部受影响体素，未做 FloatGrid BBox 加速 | ⚠️ |
| Algo_R4 §4.3 Phase 2 | 微观面元剥离 | ⚠️ 剥离逻辑正确但 active 类型错误 | ❌ Bug-1 |
| Algo_R4 §4.4 Phase 3 | 宏观 SDF 更新 | ❌ 完全缺失 | ❌ Bug-4 |
| Algo_R4 §4.5 Phase 4 | 边界面元注入 | ❌ merge 策略错误 | ❌ Bug-3 |
| 设计文档 | 面元清理 | ❌ 无 deleteInactiveSurfels | ❌ Bug-2 |

---

## 修复优先级

| 优先级 | Bug | 预计工时 | 依赖 |
|--------|-----|----------|------|
| **P0** | Bug-1: active→AttributeGroup | 2h | Spike 代码参考 |
| **P0** | Bug-2: deleteInactiveSurfels | 1h | Bug-1 |
| **P0** | Bug-3: Phase3 注入策略重写 | 4h | Bug-1 |
| **P1** | Bug-4: macroSDF update | 2h | 无 |
| **P1** | Bug-6: sampleN 放宽 | 10min | 无 |
| **P2** | Bug-7: d_v_init 公式统一 | 5min | 无 |
| **P2** | Bug-8: 双遍历合并 | 30min | 无 |

---

## 修订记录

| 版本 | 日期 | 修订内容 |
|------|------|----------|
| v0.a | 2026-06-03 | 初稿创建 |
