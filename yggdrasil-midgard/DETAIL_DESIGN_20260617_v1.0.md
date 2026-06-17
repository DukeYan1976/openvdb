# Yggdrasil-Midgard 详细设计文档 v1.0

## 修订记录

| 版本 | 日期 | 说明 |
|------|------|------|
| 1.0 | 2026-06-17 | 基于原型实现的完整设计文档，反映已验证的代码实现 |

---

## 1. 系统概述

三轴CNC铣削仿真引擎，基于 DualTrack 架构：
- **MacroGrid** (FloatGrid): 宏观 level set，CSG 差集维护拓扑
- **MicroGrid** (PointDataGrid): 微观点法式数据，精确表面表示

### 1.1 已实现模块

| 模块 | 文件 | 职责 |
|------|------|------|
| Types | `core/Types.h` | 公共类型、ToleranceConfig |
| IPWBuilder | `core/IPWBuilder.h/.cpp` | 毛坯初始化（MacroGrid + MicroGrid） |
| ToolSweepSDF | `core/ToolSweepSDF.h/.cpp` | 刀具扫掠体解析SDF + 梯度 |
| ToolSweepSurface | `core/ToolSweepSurface.h/.cpp` | 刀具扫掠体参数面（双补丁模型） |
| MacroCut | `core/MacroCut.h/.cpp` | Phase 0（CSG+分类）+ Phase 1（任务生成） |
| MicroCut | `core/MicroCut.h/.cpp` | Phase 2（采样）+ Phase 3+4（剔除+重建） |
| RtDebugSys | `debug/RtDebugSys.h/.cpp` | 运行时调试（零开销关闭） |

### 1.2 验证状态

- **59 个单元测试全部通过**
- 端到端精度验证：Max penetration = 5.3e-7mm ≤ t=0.05mm ✓

---

## 2. ToleranceConfig — 单参数派生

用户唯一输入 `t`（目标加工公差），系统自动派生：

```cpp
V_macro     = clamp(K * t, 0.02, 5.0)   // K=30, 宏观体素尺寸
reprojBand  = t                           // 重投影带宽（可选模块）
baseStep    = 10 * t                      // 流形采样基础步长
chordalLimit = t                          // 四叉树弦高终止条件
```

V_macro 不参与精度保证。精度由 `chordalLimit = t` 严格保证。

---

## 3. 管线数据流

```
IPWBuilder.build(geom, config)
    → IPWState { macroGrid, microGrid }
        │
        ▼
Phase 0: MacroCut.classifyVoxels(ipw, sdf)
    CSG差集 + 三态分类 (deleted / cut / newBoundary)
    分类条件: toolSdf ∈ [-threshold, threshold] AND |macroSdf| < V
        │
        ▼
Phase 1: MacroCut.buildTaskList(cls, surface, config, xform)
    参数面统一分块 → 反向查询匹配 voxel → VoxelTask列表
    动态 N_U/N_V + 二次最近点搜索兜底 (零全域fallback)
        │
        ▼
Phase 2: MicroCut.sampleNewSurface(tasks, surface, sdf, config)
    TBB并行四叉树自适应采样 → PointBuffer per-voxel
    终止: 弦高≤t 或 depth≥12
        │
        ▼
Phase 3+4: MicroCut.rebuildLeaves(ipw, cls, buffers, sdf, config)
    Per-leaf就地操作（只处理受影响leaf）:
    - deleted voxels: 点全部丢弃
    - cut voxels: SDF≥t 旧点保留 + 新点加入
    - newBoundary: 新点加入
    - unaffected voxels: 不动
```

---

## 4. 关键设计决策

### 4.1 Phase 0: CSG差集 + 零等值面过滤

MacroGrid 必须执行 CSG 才能正确激活新边界。分类条件增加 `|macroSdf| < V` 过滤，只收集真正的零等值面 voxel。

光栅化范围裁剪: `toolBBox ∩ workpieceActiveBBox`（排除刀杆等非接触区域）。

### 4.2 Phase 1: 统一参数面 + 动态分块

使用 `ToolSweepSurface` 的统一接口 `eval(u,v)`。不分 mid/cap 分段——统一在 [0,1]² 上分块。分块数动态适配 voxelSize：`N = clamp(extent/V, 4, 16)`。

保守性验证: 50×50 密集采样 0 violations。

### 4.3 Phase 2: 3D 拒止 + 弦高终止

四叉树在参数域内递归。拒止条件：子域中心距 voxel 中心过远时跳过。输出条件：弦高 ≤ t 且中心在 voxel 内。

所有输出点精确在 SDF=0 面上（float 精度 ≈ 5e-7mm）。

### 4.4 Phase 3+4: Per-leaf 就地重建

不整体重建 MicroGrid。采用 **“按 Leaf 流式更新”** 策略：
1. 收集受影响的 coords，按 leaf origin 分组。
2. 对每个受影响 Leaf 进行局部重建：
   - 提取存活旧点（SDF ≥ t）+ 收集新采样点。
   - 为该 Leaf 创建极小的临时 Grid（避免全局 Binning 开销）。
   - 使用 `stealNode` 从临时 Grid 获取新 Leaf，并用 `addLeaf` 手动替换主 Grid 节点。
3. 不受影响的 leaf 零开销。复杂度 O(N_affected_leaves * N_pts_per_leaf)。

这种方案实现了最优内存占用（峰值从全局点数降低到单 Leaf 点数）和缓存友好的属性注入。

### 4.5 DualTrack 一致性原则

- MacroGrid 负责拓扑（宽 narrowband = 3 层）
- MicroGrid 负责精度（只在有表面数据处有 leaf）
- **不要求 leaf count 相等**
- 一致性: MicroGrid 有点的 voxel ⊆ MacroGrid active voxel
- MacroGrid deleted 区域 MicroGrid 无点

### 4.6 ToolSweepSurface 双补丁模型

- S_mid(u,t): 包络面（侧面沿路径扫掠，包络理论）
- S_cap(u,w): 前向端帽（终点处面向进给的半球）
- 统一接口: `eval(u,v)` 跨越 vSplit 拼接两补丁
- 静态退化(L=0): S_mid 失效，S_cap 扩展为全球面
- 验证: 100×100 采样 |SDF| < 1e-9

---

## 5. 性能实测 (Mac mini M4, 20mm Box, R=5 球头刀, t=0.05mm)

| Phase | 耗时 | 输出 |
|-------|------|------|
| IPW0 build | 4.0ms | 6029 active voxels, 2234 points |
| Phase 0 | 0.6ms | d=316, c=96, n=364 |
| Phase 1 | 1.0ms | 460 tasks, 256 blocks |
| Phase 2 | 0.2ms | 419 new points, 93 voxels |
| Phase 3+4 | 0.2ms | 6 affected leaves |
| **总计** | **~6ms** | |

---

## 6. 测试覆盖

| 类别 | 测试数 | 关键验证 |
|------|--------|----------|
| Types | 2 | ToleranceConfig 派生 + clamp |
| ToolSweepSDF | 20 | 球头/平底 SDF + 梯度(有限差分<1e-6) |
| ToolSweepSurface | 13 | 静态/线性 SDF交叉验证 + bbox保守 |
| MacroCut Phase 0 | 6 | CSG + 三态分类 + 精确验证 |
| MacroCut Phase 1 | 4 | 任务生成 + 参数域保守性(0 violations) |
| IPWBuilder | 5 | MacroGrid/MicroGrid 创建 |
| MicroCut Phase 2 | 3 | SDF=0精度 + 点在voxel内 + 法线单位 |
| Phase 3+4 | 3 | 点数变化 + deleted无点 + cut剔除正确 |
| DualTrack一致性 | 3 | Transform一致 + 无inactive区点 + 无穿透 |
| **总计** | **59** | |

---

## 7. 已知限制与后续方向

| # | 限制 | 影响 | 后续方向 |
|---|------|------|----------|
| 1 | Phase 2 覆盖率 ~20% | 部分边界voxel无新点 | 正常行为（保守分类过宽） |
| 2 | 只实现球头刀+平底刀 | 牛鼻刀未实现 | M2 迭代 |
| 3 | 重投影未启用 | Phase 3 仅硬删除 | 如需再启用 |
| 4 | 无渲染管线 | 无法可视化 | 下一步实现 |
| 5 | 单段刀路 | 未实现多段连续切削循环 | M8+ |
