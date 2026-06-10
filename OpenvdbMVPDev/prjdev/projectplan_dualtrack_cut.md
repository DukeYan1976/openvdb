# DualTrack 切削算法专项计划

**文档编号**：`projectplan_dualtrack_cut.md`  
**状态**：初稿  
**作者**：Duke / Kiro  
**日期**：2026-06-10  

---

## 1. 问题域

双轨切削的核心计算：给定毛坯（MacroGrid + MicroGrid）和一个刀具扫掠体，计算切削后的几何状态。

**输入**：
- `sdfGrid` (FloatGrid, D_v精度)：毛坯的宏观有符号距离场
- `microGrid` (PointDataGrid, D_v框架内存储d_v精度面元)：切削面的精确点集
- `toolSDF`：刀具扫掠体的解析SDF函数

**输出**：
- 更新后的 `sdfGrid`：内部体素被切除，窄带更新
- 更新后的 `microGrid`：旧面元被裁剪 + 新切削界面面元被注入

---

## 2. 算法总体流程

```
对于每个毛坯中的 voxel，其状态由 toolSDF.eval(voxel中心) 确定：

  toolDist >> 0   → 完全在刀具外部 → 不受影响
  toolDist << 0   → 完全在刀具内部 → 被完全切除
  |toolDist| < D_v → 刀具表面穿过此 voxel → 产生新切削界面
```

对应三种处理：
1. **外部 voxel**：保持不变（SDF 不变，面元不变）
2. **内部 voxel**：SDF 置为正值（切除），已有面元标记 inactive
3. **边界 voxel**：SDF 更新窄带值，在刀具零面上注入精确面元

---

## 3. 当前实现的 4-Phase 架构

### Phase 1: SDF CSG 差集 + Dirty Region 标识

**输入**：sdfGrid, toolSDF  
**输出**：更新后的 sdfGrid, dirtyVoxels 列表

步骤：
1. 激活 tool BBox 范围内的 inactive 负值 voxel（将内部 tile 展开）
2. 并行遍历所有 active voxel：
   - 计算 `toolDist = toolSDF.eval(wp)`
   - CSG 差集：`newVal = max(oldVal, -toolDist)`
   - 标记 dirty：`toolDist < D_v && |newVal| < bandWidth`

**已知问题**：
- [x] dirty 条件修正：从 `toolDist <= 0` 改为 `toolDist < D_v`
- [x] inactive 激活范围：从 ±1 改为 ±3（对齐 bandWidth）
- [ ] **性能问题**：Phase 1a 的三重循环激活是 O(BBox体积)，大 BBox 时慢
- [ ] **正确性问题**：对于大的 inactive tile 区域（毛坯深处），三重循环可能遍历大量无意义 voxel

### Phase 2: 面元生成（Surfel Generation）+ 注入

**输入**：dirtyVoxels, toolSDF, xform  
**输出**：新面元注入到 microGrid

步骤：
1. 分块（256/chunk）调用 SurfelGenerator
2. 对每个 dirty voxel：在刀具零面上采样 N×N 个面元
3. 将所有新面元注入到 microGrid（首次创建或增量合并）

**已知问题**：
- [x] 毛坯边界过滤：超出 billet geometry 的面元被丢弃
- [ ] **面元投影精度**：单次法线投影 `candidate -= dist * normal` 对曲率大的刀具表面可能不精确
- [ ] **面元与旧面元重叠**：重注入时没有去重，同一位置可能产生多个面元
- [ ] **注入效率**：`injectSurfels` 对 dirty leaf 做提取+重建，频繁调用时有碎片

### Phase 3: 面元裁剪（Surfel Clipping）

**输入**：microGrid, toolSDF  
**输出**：刀具内部的面元标记为 inactive

步骤：
1. 遍历 tool BBox 范围内 microGrid 的 leaf nodes
2. 对每个 active 面元计算其世界坐标
3. 如果 `toolSDF.eval(wp) <= 0`：标记 `active = 0`

**已知问题**：
- [ ] **执行顺序**：Phase 2（注入）在 Phase 3（裁剪）之前。新注入的面元理论上在 tool 零面上（eval≈0），不应被裁剪。但如果投影不精确导致 eval < 0，新面元会被误杀。
- [ ] **阈值问题**：`eval <= 0.0` 应改为 `eval < -epsilon` 避免浮点误判

### Phase 4: 清理

- `pruneLevelSet`：还原远离表面的 voxel 为 tile
- `dirtyMask->clear()`

---

## 4. 需要解决的问题清单

### 4.1 正确性问题（P0）

| ID | 问题 | 影响 | 解决方案 |
|----|------|------|----------|
| C1 | Phase 3 裁剪阈值 `<= 0.0` 可能误杀刚注入的面元 | 新切削面出现空洞 | 改为 `< -0.5*d_v`，给零面面元容差 |
| C2 | 面元投影仅一步牛顿迭代，曲率大时不精确 | 面元偏离真实零面 | 改为多步迭代（2-3步），或验证收敛 |
| C3 | 重注入不去重，多次切削同区域面元堆积 | 内存增长、渲染重叠 | 注入前清除 dirty voxel 内的旧面元 |
| C4 | Phase 1a inactive 激活对大毛坯是全量三重循环 | 功能正确但效率极低 | 用 `tree.fill()` 或 `dilateActiveValues` 替代 |

### 4.2 设计问题（P1）

| ID | 问题 | 影响 | 解决方案 |
|----|------|------|----------|
| D1 | Phase 2 → Phase 3 顺序导致新面元需容差保护 | 逻辑脆弱 | 考虑调换为 Phase 3（先裁剪旧面元）→ Phase 2（再注入新面元） |
| D2 | dirtyVoxels 无去重，同一 coord 可能多次出现 | 重复采样浪费 | 用 MaskGrid 或 set 去重 |
| D3 | injectSurfels 中 leaf-local splice 对已有叶先提取再重建 | 频繁内存分配 | 区分"新 leaf"和"已有 leaf 追加"两种路径 |
| D4 | microGrid 的 PointDataGrid 属性只有 position/normal/active | 缺少 precision 标记 | 后续需区分 IPW₀ 粗面元和切削精细面元 |

### 4.3 性能问题（P2）

| ID | 问题 | 影响 | 解决方案 |
|----|------|------|----------|
| P1 | Phase 1a 三重循环（激活 inactive voxel） | 大 BBox 时 O(n³) 慢 | 改用 `sdfGrid->tree().fill(bbox, background, active=true)` 仅激活 tile |
| P2 | SurfelGenerator 每 voxel 独立计算 gradient | 相邻 voxel 重复计算 | 可缓存或批量化 |
| P3 | Phase 3 遍历所有可能 leaf origin（包括不存在的） | 空遍历浪费 | 直接用 leafManager 遍历存在的 leaf |

---

## 5. 重构方案：修正版 4-Phase

### 核心变更：**调换 Phase 2 和 Phase 3 的顺序**

```
Phase 1: SDF 更新 + Dirty 标识（不变）
Phase 2: 裁剪旧面元（原 Phase 3 → 前移）
Phase 3: 注入新面元（原 Phase 2 → 后移）
Phase 4: 清理（不变）
```

**理由**：
- 先裁剪再注入，新面元不需要容差保护，逻辑更清晰
- 裁剪后 dirty voxel 内的旧面元已被标记 inactive
- 注入时可以安全地清除 inactive 旧面元（或直接覆盖）

### 修正版流程详述

#### Phase 1: SDF CSG Diff + Dirty Region（同现有，修 C4/D2）

```
1a. 激活 tool BBox ∩ sdfGrid 中的 inactive 负值区域
    → 改用 tree.fill() + voxelizeActiveTiles() 替代三重循环
1b. TBB 并行 SDF 更新：newVal = max(oldVal, -toolDist)
    dirty 条件：toolDist < D_v && |newVal| < bandWidth
1c. 合并 dirty（用 dirtyMask 去重）
```

#### Phase 2: 裁剪旧面元（Clip）

```
对 tool BBox 范围内 microGrid 的所有 leaf:
    对每个 active 面元:
        如果 toolSDF.eval(wp) < -0.5*d_v:   // 容差: 在刀具内部超过半个面元间距
            active = 0
```

注意：阈值从 `<= 0.0` 改为 `< -0.5*d_v`，避免零面附近的面元被误杀。

#### Phase 3: 注入新面元（Inject）

```
对每个 dirty voxel:
    清除该 voxel 内已有的所有面元（active + inactive 全部清除）
    在刀具零面上采样 N×N 新面元（多步迭代保证精度）
    注入到 microGrid
```

**关键变更**：注入前**清除 dirty voxel 内的旧面元**，解决 C3（重叠堆积）问题。

#### Phase 4: 清理

```
pruneLevelSet(sdfGrid)
dirtyMask->clear()
```

---

## 6. 面元投影精度改进（C2）

当前：
```cpp
candidate -= dist * normal;  // 单步
```

改进：
```cpp
for (int iter = 0; iter < 3; ++iter) {
    double dist = tool.eval(candidate);
    Vec3d grad = tool.gradient(candidate);
    double gl = grad.length();
    if (gl < 1e-10) break;
    candidate -= (dist / gl) * grad;  // 沿当前梯度方向投影
    if (std::abs(tool.eval(candidate)) < 0.1 * d_v) break;  // 收敛
}
```

使用实际梯度（而非固定 normal）进行牛顿迭代，对曲率大的表面更精确。

---

## 7. 实施计划

| 步骤 | 内容 | 预计工时 |
|------|------|----------|
| S1 | 调换 Phase 2/3 顺序，裁剪阈值改为 `-0.5*d_v` | 0.5天 |
| S2 | 注入前清除 dirty voxel 旧面元 | 0.5天 |
| S3 | 面元投影改为多步牛顿迭代 | 0.5天 |
| S4 | Phase 1a 改用 `fill()` 替代三重循环 | 0.5天 |
| S5 | dirtyVoxels 去重（用 dirtyMask 判断） | 0.5天 |
| S6 | Phase 3 裁剪改用 leafManager 遍历 | 0.5天 |
| S7 | 单元测试验证 | 1天 |
| S8 | 端到端精度验证（面元到刀具零面距离 < d_v） | 0.5天 |

**总计**：~4.5 天

---

## 8. 验收标准

| 指标 | 标准 |
|------|------|
| 面元精度 | 所有 active surfel 到 tool 零面距离 < d_v |
| 无误杀 | 新注入面元 100% 存活（不被 Phase 2 裁剪） |
| 无堆积 | dirty voxel 内面元数恒定 ≤ N×N（不随切削次数增长） |
| 无空洞 | 切削面完整覆盖所有 dirty voxel |
| 性能 | 30mm 球头刀路径 (R=3, D_v=0.5)：< 50ms |

---

## 修订记录

| 版本 | 日期 | 修订内容 |
|------|------|----------|
| v0.a | 2026-06-10 | 初稿 |
