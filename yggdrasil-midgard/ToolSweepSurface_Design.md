# 刀具扫掠参数面构建 — 技术专题

## 修订记录

| 版本 | 日期 | 说明 |
|------|------|------|
| 0.a | 2026-06-17 | 初始版本，双补丁统一参数面框架设计 |

---

## 1. 问题描述

### 1.1 背景

在 CNC 铣削仿真的 Phase 1-2 中，需要将刀具沿刀路产生的切削界面降维为 2D 参数域，以便：
- Phase 1：参数域分块 → 3D AABB → 反向查询匹配 voxel
- Phase 2：在参数域内自适应四叉树细分 → 精确采样切削面点

### 1.2 复杂性来源

**刀具几何分区**：
- 刀杆（非切削，仅碰撞检测）
- 圆柱侧刃（切削长度 H_cut）
- 底部形态（球头/平底/牛鼻圆角）

**运动方向分区**（沿进给方向）：
- 前向端帽：迎着材料，发生新切削
- 扫掠中间面：侧面沿路径扫过的包络面
- 后向端帽：离开材料，已被前段完成，**不参与当前段计算**

**与 IPW 的接触约束**：
- 即使是切削部分，悬在空气中的区域也不需要计算
- Phase 0 的 CutClassification 提供了实际接触范围

### 1.3 目标

构建一个统一的参数面框架，满足：
1. 覆盖所有切削面（且仅覆盖切削面）
2. 三种刀型（球头/平底/牛鼻）统一接口
3. 退化情况（路径=0、纯端面切削、纯侧面切削）自然处理
4. 参数域为规整矩形 [0,1]²，适配四叉树细分

---

## 2. 方案：双补丁统一参数面框架 (Two-Patch Model)

### 2.1 核心思想

将扫掠体切削面分解为两个独立的 2D 参数补丁：

| 补丁 | 参数域 | 物理含义 |
|------|--------|----------|
| $S_{mid}(u, t)$ | $[0,1] \times [0,1]$ | 扫掠中间面（包络面） |
| $S_{cap}(u, w)$ | $[0,1] \times [0,1]$ | 前向端帽（终点处迎料面） |

后向端帽被**数学上直接抛弃**——不生成参数补丁，从根源消灭冗余计算。

### 2.2 统一 1D 刀具轮廓 C(u)

封装刀型差异为统一的 1D 母线参数化。参数 $u \in [0,1]$ 表示从刀具底部极点到切削部分上界的归一化位置。

刀具局部坐标系（刀轴沿 Z，刀尖朝 -Z）：

```
P_tool(u, φ) = (r(u)·cos(φ), r(u)·sin(φ), z(u))
N_tool(u, φ) = (n_r(u)·cos(φ), n_r(u)·sin(φ), n_z(u))
```

各刀型的 r(u), z(u) 定义：

**球头刀** (R = 刀具半径)：
```
u ∈ [0, u_equator]: 底部半球
  角度 α = (π/2) · (u / u_equator)    // 从南极到赤道
  r(u) = R · cos(α)... 
  
简化：用 u 直接映射球面纬度
  z(u) = -R + u·(R + H_cut)          // -R(刀尖) → H_cut(侧刃顶)
  对于 z ∈ [-R, 0]: r = √(R² - (z+R-R)²)... 

实用定义（分段）:
  u ∈ [0, u_eq]:  半球段
    z = -R·(1 - u/u_eq)             // z从-R到0
    r = √(R² - z²) = √(R² - R²(1-u/u_eq)²) = R·√(1-(1-u/u_eq)²)
  u ∈ [u_eq, 1]:  圆柱段
    z = (u - u_eq)/(1 - u_eq) · H_cut  // z从0到H_cut
    r = R
  其中 u_eq = R / (R + H_cut)
```

**平底刀**：
```
  u ∈ [0, u_bot]:  底面半径段
    z = 0
    r = u/u_bot · R
  u ∈ [u_bot, 1]:  圆柱段
    z = (u - u_bot)/(1 - u_bot) · H_cut
    r = R
  其中 u_bot = R / (R + H_cut)  (或用较小比例，底面扫描快)
```

**牛鼻刀** (R = 刀具半径, r_c = 圆角半径)：
```
  u ∈ [0, u_flat]: 底面段 (半径 R - r_c)
    z = 0
    r = u/u_flat · (R - r_c)
  u ∈ [u_flat, u_arc]: 圆角段
    角度 α = (u - u_flat)/(u_arc - u_flat) · (π/2)
    z = r_c · (1 - cos(α))
    r = (R - r_c) + r_c · sin(α)
  u ∈ [u_arc, 1]: 圆柱段
    z = r_c + (u - u_arc)/(1 - u_arc) · (H_cut - r_c)
    r = R
```

### 2.3 补丁 1: 扫掠中间面 S_mid(u, t)

基于包络理论：刀具表面上的点能留在扫掠体表面 ⟺ 该点法线与运动速度垂直。

**包络方程**：$N_{tool}(u, φ) · v(t) = 0$

对于三轴线性进给（速度方向 $v$ = 常向量 $(v_x, v_y, 0)$）：
```
n_r(u)·cos(φ)·v_x + n_r(u)·sin(φ)·v_y = 0
→ tan(φ) = -v_x / v_y
→ φ_grazing = atan2(-v_x, v_y)  和  φ_grazing + π
```

取面向进给方向的解（前向擦掠线）：
```
φ_grazing = atan2(-v_x, v_y) + π   // 面对材料的那侧
```

**补丁映射**：
```
S_mid(u, t) = P_tool(u, φ_grazing) + A + t·(B - A)

u ∈ [0, 1]: 沿刀具轮廓从底到顶
t ∈ [0, 1]: 沿刀路从A到B
```

**法线**：直接从 N_tool(u, φ_grazing) 获取。

**关键性质**：对于三轴直线进给，φ_grazing 是常数（不依赖 u 或 t）——中间面退化为刀具单侧轮廓沿路径的平移。

### 2.4 补丁 2: 前向端帽 S_cap(u, w)

刀具在终点 B 处，面向进给方向的半个"面罩"。

```
φ_start = φ_grazing - π/2
φ_end = φ_grazing + π/2
φ(w) = φ_start + w · (φ_end - φ_start)    // w ∈ [0,1] → 前向180°

S_cap(u, w) = P_tool(u, φ(w)) + B

u ∈ [0, 1]: 沿刀具轮廓
w ∈ [0, 1]: 绕刀轴的前向半圆
```

### 2.5 退化处理

| 场景 | S_mid | S_cap |
|------|-------|-------|
| **路径=0（静态）** | 退化：v=0，无擦掠线，不生成 | 角度扩展为 360°（w 覆盖全圆） |
| **纯端面切削（v⊥底面）** | 生成圆柱侧面包络 | 底部形态全部由 cap 负责 |
| **纯侧面切削** | 生成侧面包络 | cap 的底部区域被 Phase 1 bbox 剪枝 |

---

## 3. 与 V4.0 架构的对接

### 3.1 Phase 1 流程

```
输入: CutClassification (cut + newBoundary voxels), 双补丁定义

1. 对 S_mid 参数域 [0,1]² 均匀分块 (N_u × N_t)
   对每个块 [u_i, u_i+1] × [t_j, t_j+1]:
     计算 3D AABB (采样 evalSurface 多点取包围)
     与 voxel 列表做 overlap 匹配

2. 对 S_cap 参数域 [0,1]² 均匀分块 (N_u × N_w)
   同样计算 AABB → 匹配 voxel

3. 每个 voxel 的任务携带:
   - 来自 S_mid 的参数子域 [u_min, u_max, t_min, t_max]
   - 来自 S_cap 的参数子域 [u_min, u_max, w_min, w_max]
   - 可能两者都有，也可能只有一个
```

### 3.2 Phase 2 流程

```
对每个 VoxelTask:
  if has S_mid range:
    quadtreeSubdivide(S_mid, u_range, t_range, voxelAABB, chordalLimit)
  if has S_cap range:
    quadtreeSubdivide(S_cap, u_range, w_range, voxelAABB, chordalLimit)
  
  核内 3D 拒止: if (!voxelAABB.contains(point)) skip
```

Phase 2 不关心当前处理的是中间面还是端帽——只看到一个 2D 矩形域和一个 evalSurface 函数。

### 3.3 接口设计

```cpp
class ToolSweepSurface {
public:
    ToolSweepSurface(const ToolDef& tool, const MoveSegment& seg);

    // ─── 统一接口 ──────────────────────────────────────────────────
    // v ∈ [0,1] 横跨 mid + cap 两个补丁（以 vSplit() 为分界）
    // v ∈ [0, vSplit]: mid 补丁, v ∈ [vSplit, 1]: cap 补丁
    Vec3d eval(double u, double v) const;
    Vec3d normal(double u, double v) const;
    openvdb::BBoxd bbox(double u0, double u1, double v0, double v1) const;

    /// mid/cap 的分界位置
    double vSplit() const;

    // ─── 分补丁接口（精细控制时使用）───────────────────────────────
    bool hasMidPatch() const;  // 路径=0时为false

    Vec3d evalMid(double u, double t) const;
    Vec3d normalMid(double u, double t) const;
    openvdb::BBoxd bboxMid(double u0, double u1, double t0, double t1) const;

    Vec3d evalCap(double u, double w) const;
    Vec3d normalCap(double u, double w) const;
    openvdb::BBoxd bboxCap(double u0, double u1, double w0, double w1) const;

private:
    /// 1D 刀具轮廓
    double r(double u) const;
    double z(double u) const;
    Vec3d  n(double u) const;  // (n_r, n_z)

    double mPhiGrazing;  // 包络角（三轴常数）
    double mVSplit;      // 统一参数分界点
    // ...
};
```

**统一接口 eval(u,v) 的局限说明**：

统一接口在 v = vSplit 处存在 C¹ 不连续（位置 C⁰ 连续，切向量方向突变）。
这是因为 v < vSplit 时偏导 ∂S/∂v 方向沿刀路，v > vSplit 时 ∂S/∂v 方向沿角度。

**对 Phase 2 的影响**：接缝处弦高计算可能偏大（因为双线性插值跨越了方向突变），
导致四叉树在该区域多细分 1-2 层。但这不影响精度——最终采样点仍然落在正确的
曲面上，只是多了少量冗余采样。弦高终止条件保证最终精度 ≤ t。

**实践建议**：Phase 1 分块时将块边界对齐 vSplit（不跨越），可完全消除此开销。

---

## 4. 独立分析与风险评估

### 4.1 方案优势

1. **严格降维**：3D 布尔 → 2D 参数域，计算量与表面积成正比（非体积）
2. **解析可导**：每个补丁的法线可解析计算，无需有限差分
3. **自然裁剪**：后向端帽不生成 → 零冗余；Phase 1 的 AABB 匹配自动剪枝空气区域
4. **刀型统一**：差异封装在 C(u) 内部，Phase 1/2 看到的都是 [0,1]² 域

### 4.2 潜在风险

| 风险 | 分析 | 应对 |
|------|------|------|
| 包络方程在 n_r=0 时退化 | 球头刀南极点处 n_r→0，φ_grazing 无定义 | u=0 处退化为单点，被 Phase 2 的弦高终止自然处理 |
| S_mid 与 S_cap 拼接缝 | 两补丁在 t=1, φ=φ_grazing±π/2 处相接 | 裕量外扩保证拼接处不漏采样 |
| 三轴非水平路径 | v 有 Z 分量时，φ_grazing 仍为常数（只依赖 v_x, v_y）但轮廓投影改变 | 三轴固定刀轴Z，v 的 Z 分量不影响包络角计算 |
| surfaceBBox 精度 | 采样点数不足时 AABB 可能非保守 | 加密采样（6×6 网格）+ 外扩 margin |

### 4.3 计算量评估

Phase 1 的参数域分块：
- S_mid: N_u × N_t 块，每块 6×6=36 次 evalMid 求 AABB → 总计 N_u × N_t × 36
- S_cap: N_u × N_w 块，同上
- 典型值：N_u=8, N_t=16, N_w=8 → (8×16 + 8×8) × 36 = 6912 次求值
- 单次求值 ≈ 10ns → Phase 1 参数域计算 ≈ 0.07ms（可忽略）

---

## 5. 设计决策记录

### 5.1 ToolSweepSurface 与 ToolSweepSDF 的关系

两者描述同一几何体的不同表示：

| | ToolSweepSDF | ToolSweepSurface |
|--|---|---|
| 表示类型 | 隐式（点→距离） | 显式参数化（参数→曲面点） |
| 用途 | Phase 0/3: inside/outside 判断 | Phase 1/2: 空间定位+采样 |
| 求值方式 | 直接 O(1) | 直接 O(1) |
| 采样密度控制 | ❌ 无法控制 | ✅ 参数域细分 = 空间精度 |
| 空间索引支持 | ❌ 需逐点判断 | ✅ 参数块 → 3D AABB 预分块 |

**交叉验证原则**：
```
∀ (u,v): |ToolSweepSDF.eval(ToolSweepSurface.eval(u,v))| ≈ 0
```
参数面上的任意点，其 SDF 值应为零——这是最强力的正确性检验。

### 5.2 为什么不直接参数化 SDF=0 等值面

理论上可以通过在 SDF=0 面上做数值 tracing 来获得参数化，但：
- 每次采样需要 Newton 迭代求根（不如解析正向求值 O(1)）
- 无法预先知道"哪部分表面对应哪个空间区域"（丧失 Phase 1 预分块能力）
- 采样密度无法从参数域控制（盲目搜索）

双补丁解析参数面是精度、效率、可控性的最优均衡。

### 5.3 独立性

ToolSweepSurface 是纯几何模块，不依赖 OpenVDB/IPW。可独立开发、独立测试、独立复用（碰撞检测、可视化等场景均适用）。

---

## 6. 实现优先级

| 阶段 | 内容 | 验证方式 |
|------|------|----------|
| M4.1 | 球头刀 C(u) + 静态退化 (S_cap = 全球面) | SDF交叉验证: eval点SDF≈0 |
| M4.2 | 球头刀线性刀路 S_mid + S_cap 双补丁 | 同上 + bbox保守性验证 |
| M4.3 | buildTaskList 集成双补丁 | Phase 1 测试通过 |
| M4.4 | 平底刀/牛鼻刀 C(u) 扩展 | 后续迭代 |
