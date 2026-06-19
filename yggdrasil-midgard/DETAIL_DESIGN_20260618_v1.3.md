# Yggdrasil-Midgard 详细设计文档 v1.3

## 修订记录

| 版本 | 日期 | 说明 |
|------|------|------|
| 1.0 | 2026-06-17 | 基于原型实现的完整设计文档，反映已验证的代码实现 |
| 1.1 | 2026-06-18 | **架构升级：引入 6-Patch 迎水面参数面模型。** 实现全刀型（球头/平底/牛鼻）准弧长参数化，增加平底刀数值稳定性圆角。 |
| 1.2 | 2026-06-18 | **全量重构：ToolSweepSDF 重命名为 ToolSweptSDF。** 统一类名、文件名及工程引用；通过 `ToolSweepSurface` 进行跨模块 SDF 一致性验证。 |
| 1.3 | 2026-06-18 | **算法攻坚：引入 Phase 1.5 动态毛坯补全与特征感知局部 SDF。** 解决 IPW0 边界采样缺失及 QuadTree 相交线“空气中采样”问题。 |

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
| ToolSweptSDF | `core/ToolSweptSDF.h/.cpp` | 扫掠体解析SDF（支持球头/平底/牛鼻，含数值平滑） |
| ToolSweepSurface | `core/ToolSweepSurface.h/.cpp` | 6-Patch 迎水面参数面（双向弧长比例映射） |
| MacroCut | `core/MacroCut.h/.cpp` | Phase 0（CSG+分类）+ Phase 1（任务生成） |
| LocalSurfaceEngine | `core/MicroCut.cpp (Internal)` | **新增**：基于 Voxel 既有点云的特征感知局部隐式场 |
| MicroCut | `core/MicroCut.h/.cpp` | Phase 1.5（毛坯补全）+ Phase 2（相交采样）+ Phase 3+4（重建） |
| RtDebugSys | `debug/RtDebugSys.h/.cpp` | 运行时调试（零开销关闭） |

### 1.2 验证状态

- **核心单元测试覆盖全面**，已通过 `ToolSweepSurface` 与 `ToolSweptSDF` 的交叉验证（误差 < 1e-9）
- 端到端精度验证：Max penetration ≈ 5.8e-7mm ≤ t=0.05mm ✓
- 支持球头、牛鼻、平底（含 0.01mm 数值圆角）三种常用刀具

---

## 2. ToleranceConfig — 单参数派生

用户唯一输入 `t`（目标加工公差），系统自动派生：

```cpp
V_macro     = clamp(K * t, 0.02, 5.0)   // K=30, 宏观体素尺寸
reprojBand  = t                         // 重投影带宽
baseStep    = 10 * t                    // 流形采样基础步长
chordalLimit = t                        // 四叉树弦高终止条件
```

---

## 3. 管线数据流

```text
IPWBuilder.build(geom, config)
    → IPWState { macroGrid, microGrid }
        │
        ▼
Phase 0: MacroCut.classifyVoxels(ipw, sdf)
    CSG差集 + 三态分类 (deleted / cut / newBoundary)
        │
        ▼
Phase 1: MacroCut.buildTaskList(...)
    6-Patch参数面统一分块 → 反向查询匹配 voxel → VoxelTask列表
        │
        ▼
Phase 1.5: MicroCut.primeBilletBoundaries(...) [新增]
    对 NEW_BOUNDARY 且缺少点云的体素，动态进行毛坯表面自适应采样。
    解析毛坯降级为单点平面近似，Mesh毛坯提取三角面片特征。
        │
        ▼
Phase 2: MicroCut.sampleNewSurface(...) [增强]
    TBB并行四叉树自适应采样。
    引入 LocalSurfaceEngine 计算 Voxel 局部多法向约束 SDF，精准捕捉相交线，消除“空气采样”。
        │
        ▼
Phase 3+4: MicroCut.rebuildLeaves(...)
    Per-leaf就地重建 + 旧点 SDF 剔除
```

---

## 4. 关键设计决策

### 4.1 Phase 1.5: 动态毛坯补全 (Dynamic Billet Priming)
解决 IPW0 状态下 `MicroGrid` 为空导致的第一刀边界精度退化问题。
- **触发时机**：在 `MacroCut` 生成分类任务后，`MicroCut` 采样前。
- **并行执行**：对标记为 `NEW_BOUNDARY` 且需与毛坯求交的 Voxel 提取点集。
- **极简采样策略**：
  - **解析毛坯 (Analytical)**：不需要过度采样。若判定为平面，确保 Voxel 点集内至少包含一个有效点与该面的法向即可代表局部半空间。
  - **Mesh 毛坯**：直接提取落在 Voxel 内部的三角面片重心及法向，低开销完成冷启动。

### 4.2 LocalSurfaceEngine: 特征感知局部隐式场
用于 Phase 2 QuadTree 中精确判断刀具采样点是否位于旧材料内部 ($SDF_{old} < -t$)。
- **k-NN 半空间约束**：不再盲目使用最近单点投影，而是获取邻域内的多个点集。
- **脊线保护**：当邻域法向发散（例如夹角 > 30°），构建多平面交集约束 `SDF = max( (P - P_i)*N_i )`。这确保了在刀具越过尖锐残高脊线时，只有真正侵入**所有**相邻面的点才会被保留，完美防止过切。

### 4.3 Phase 0: CSG差集 + 零等值面过滤
MacroGrid 执行 CSG 差集更新。分类逻辑在 CSG 之前进行，确保被完全删除的 voxel 被正确收集。增加 `levelSetRebuild` 确保窄带拓扑稳健。

### 4.4 Phase 1: 6-Patch 迎水面分块
使用 `ToolSweepSurface` 的 6 分段接口。参数域 $[0,1]^2$ 按 $u$ 向（轮廓弧长）和 $v$ 向（扫掠宽度）比例划分，分块大小适配 VoxelSize。

### 4.5 Phase 2: 准弧长采样 + 3D 拒止
四叉树在参数域内递归。通过 $u$ 向纬度角映射解决极点奇异性，通过 $v$ 向物理权重分配保证采样均匀。弦高终止条件保证几何精度。

### 4.6 Phase 3+4: Per-leaf 流式重建
不整体重建 MicroGrid。收集受影响 Leaf，局部重建（存活旧点 + 新采样点），利用 OpenVDB `stealNode` 实现零拷贝节点替换。

### 4.7 ToolSweepSurface: 6-Patch "迎水面"模型
为了物理上的等弧长采样并解决极点奇异性，参数面采用 **6 分段构造** ($2 \times 3$ 拓扑)：
- **$u$ 向（纵向剖面）**：底平段 $\to$ 圆角段（纬度角映射）$\to$ 圆柱段。
- **$v$ 向（扫掠宽度）**：左侧包络 $\to$ 前向端帽 $\to$ 右侧包络。
- **数值稳定性**：平底刀自动应用 $0.01$mm 极小圆角，消除 QuadTree 递归爆炸。

### 4.8 ToolSweptSDF: 统一圆角模型
采用 `roundedCylinderSDF` 统一处理平底和牛鼻刀，确保显式点云与隐式距离场高度统一（SDF误差 $< 10^{-9}$）。

---

## 5. 性能实测 (Mac mini M4, 20mm Path, R=5 球头刀, t=0.05mm)

| Phase | 耗时 | 输出 |
|-------|------|------|
| IPW0 build | ~8ms | 47726 active voxels |
| MacroCut P0 | ~18ms | d=2911, n=1350 |
| Phase 1 | ~1ms | 1350 tasks |
| Phase 2+3+4 | ~1ms | 100 new points (adaptive) |
| **总计** | **~28ms** | |

---

## 6. 已知限制与后续方向
1. **梯度精度偏差风险**：当前 `ToolSweptSDF::gradient` 使用固定步长 $h=10^{-4}$，在测试中发现与有限差分（$h=10^{-7}$）存在约 $5 \times 10^{-6}$ 的偏差。对于极高精度要求（$t < 0.001$）的场景，需推导全解析梯度公式或引入自适应步长。
2. 多段连续切削循环支持 (M8+)
3. GPU 加速四叉树细分 (未来规划)
4. 五轴非线性进给支持
