# 刀具扫掠参数面构建 — 技术专题 v0.b

## 修订记录

| 版本 | 日期 | 说明 |
|------|------|------|
| 0.a | 2026-06-17 | 初始版本，双补丁统一参数面框架设计 |
| 0.b | 2026-06-18 | 审查优化：添加摘要、修正球头刀C(u)定义、补充符号约定、强化退化处理、增加边界裕量公式与性能对比、新增测试策略附录 |

---

## 摘要

**核心洞见**：将刀具扫掠体切削面分解为两个独立的 2D 参数补丁（扫掠中间面 + 前向端帽），可将 3D 布尔运算降维为 2D 参数域操作，计算复杂度从体积级降至表面积级。

**关键技术决策**：
1. 统一 1D 刀具轮廓 C(u) 封装三种刀型差异，Phase 1/2 只看到 [0,1]² 参数域
2. 后向端帽数学上直接抛弃，从根源消除冗余计算
3. 三轴线性进给下包络角 φ_grazing 为常数，中间面退化为单侧轮廓平移
4. 统一接口 eval(u,v) 在 vSplit 处 C¹ 不连续，但 Phase 1 分块对齐边界即可消除开销

**适用场景**：CNC 铣削仿真 Phase 1-2 的切削界面降维、自适应四叉树细分、精确采样。

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
| S_mid(u, t) | [0,1] × [0,1] | 扫掠中间面（包络面） |
| S_cap(u, w) | [0,1] × [0,1] | 前向端帽（终点处迎料面） |

后向端帽被**数学上直接抛弃**——不生成参数补丁，从根源消灭冗余计算。

### 2.2 符号约定与坐标系

| 符号 | 含义 |
|------|------|
| u ∈ [0,1] | 沿刀具轮廓从底部极点到切削部分上界的归一化位置 |
| t ∈ [0,1] | 沿刀路从起点 A 到终点 B 的归一化弧长 |
| w ∈ [0,1] | 前向端帽的半圆角度参数 |
| φ ∈ [0,2π] | 刀具横截面内的极角 |
| O_tool | 刀具局部坐标系原点（刀尖底部极点） |
| Z_tool | 刀具局部坐标系：刀轴沿 +Z，刀尖朝 -Z |
| X_global | 全局坐标系，A→B 刀路在此坐标系下描述 |

刀具局部坐标系下的参数方程：
```
P_tool(u, φ) = (r(u)·cos(φ), r(u)·sin(φ), z(u))
N_tool(u, φ) = (n_r(u)·cos(φ), n_r(u)·sin(φ), n_z(u))   // 单位外法线（指向刀具外部）
```

### 2.3 统一 1D 刀具轮廓 C(u)

三种刀型的 r(u), z(u) 定义如下。总切削高度 H = R + H_cut（球头/平底）或 H = R + H_cut（牛鼻），其中 H_cut 为侧刃有效切削长度。

**球头刀**（球半径 R，总切削高度 H = R + H_cut）：

设 u_eq = R / H（赤道点归一化位置），则：

```
u ∈ [0, u_eq]:  半球段
    纬度角 α = (π/2) · (u / u_eq)        // 从南极(α=0)到赤道(α=π/2)
    z(u) = -R · cos(α) = -R · cos((π/2)·u/u_eq)
    r(u) =  R · sin(α) =  R · sin((π/2)·u/u_eq)
    
u ∈ [u_eq, 1]:  圆柱段
    z(u) = (u - u_eq)/(1 - u_eq) · H_cut
    r(u) = R
```

**平底刀**（半径 R，总切削高度 H = R + H_cut）：

设 u_bot = R / H（底面半径段与圆柱段分界），则：

```
u ∈ [0, u_bot]:  底面圆盘段
    z(u) = 0
    r(u) = (u / u_bot) · R
    
u ∈ [u_bot, 1]:  圆柱段
    z(u) = (u - u_bot)/(1 - u_bot) · H_cut
    r(u) = R
```

**牛鼻刀**（半径 R，圆角半径 r_c，总切削高度 H = R + H_cut）：

设 u_flat = (R - r_c) / H（底平面段），u_arc = u_flat + (π/2)·r_c / H（圆角段终点），则：

```
u ∈ [0, u_flat]:  底平面段（半径 R - r_c）
    z(u) = 0
    r(u) = (u / u_flat) · (R - r_c)
    
u ∈ [u_flat, u_arc]:  圆角段
    角度 α = (u - u_flat)/(u_arc - u_flat) · (π/2)
    z(u) = r_c · (1 - cos(α))
    r(u) = (R - r_c) + r_c · sin(α)
    
u ∈ [u_arc, 1]:  圆柱段
    z(u) = r_c + (u - u_arc)/(1 - u_arc) · (H_cut - r_c)
    r(u) = R
```

### 2.4 补丁 1: 扫掠中间面 S_mid(u, t)

**包络理论**：刀具表面上的点能留在扫掠体表面 ⟺ 该点法线与运动速度垂直。

**包络方程**：N_tool(u, φ) · v(t) = 0

对于三轴线性进给（刀轴固定为 Z，速度 v = (v_x, v_y, 0) 为常向量）：

```
n_r(u)·cos(φ)·v_x + n_r(u)·sin(φ)·v_y = 0
→ cos(φ)·v_x + sin(φ)·v_y = 0          (n_r ≠ 0 时)
→ tan(φ) = -v_x / v_y
→ φ = atan2(-v_x, v_y)  或  φ + π
```

**擦掠线方向判定**：
擦掠线有两条（相差 π），分别位于刀具两侧。需要选择"面向进给方向"的那条——即该侧法线方向与速度方向夹角为锐角（材料在刀具前方）。

```
φ_grazing = atan2(-v_x, v_y) + π·δ

其中 δ = 1  if  N_tool(u, φ_0)·v < 0  (法线指向刀具内部，面向材料)
          0  otherwise
且 φ_0 = atan2(-v_x, v_y)
```

**关键性质**：对于三轴直线进给，v 为常向量，因此 φ_grazing 是常数（不依赖 u 或 t）。中间面退化为刀具单侧轮廓沿路径的平移。

**补丁映射**：
```
S_mid(u, t) = P_tool(u, φ_grazing) + A + t·(B - A)

u ∈ [0, 1]: 沿刀具轮廓从底到顶
t ∈ [0, 1]: 沿刀路从 A 到 B
```

**法线**（指向扫掠体外部，即切削界面朝向被切除材料）：
```
N_mid(u, t) = N_tool(u, φ_grazing)   // 与刀具外法线一致
```

### 2.5 补丁 2: 前向端帽 S_cap(u, w)

刀具在终点 B 处，面向进给方向的半个"面罩"。

```
φ_start = φ_grazing - π/2
φ_end   = φ_grazing + π/2
φ(w) = φ_start + w · (φ_end - φ_start)    // w ∈ [0,1] → 前向 180°

S_cap(u, w) = P_tool(u, φ(w)) + B

u ∈ [0, 1]: 沿刀具轮廓
w ∈ [0, 1]: 绕刀轴的前向半圆
```

**法线**：
```
N_cap(u, w) = N_tool(u, φ(w))   // 端帽法线与刀具外法线一致
```

### 2.6 退化处理

| 场景 | S_mid | S_cap | 备注 |
|------|-------|-------|------|
| **路径=0（静态）** | 不生成，hasMidPatch() = false | 角度扩展为 360°（w 覆盖全圆） | 纯端面接触 |
| **纯端面切削（v ⊥ 底面）** | 生成圆柱侧面包络 | 底部形态全部由 cap 负责 | 此时 v_z = 0, v_xy ⊥ 底面 |
| **纯侧面切削** | 生成侧面包络 | cap 的底部区域被 Phase 1 bbox 剪枝 | 无新底面暴露 |
| **纯径向进给（v ⊥ 刀轴）** | 生成完整侧面包络 | 前向半圆帽 | 标准双补丁 |
| **轴向进给（v ∥ 刀轴）** | 包络面退化为圆柱侧面（无新侧刃暴露） | cap 退化为全圆端面 | 需特殊处理 |

**路径=0 的端帽扩展**：
当路径长度 |B - A| = 0 时，刀具没有运动。此时所有切削发生在端面，S_cap 的 w 覆盖 [0, 2π]（全圆），而非半圆。

### 2.7 统一接口与分补丁接口

```cpp
class ToolSweepSurface {
public:
    ToolSweepSurface(const ToolDef& tool, const MoveSegment& seg);

    // ─── 统一接口 ──────────────────────────────────────────────────
    // v ∈ [0,1] 横跨 mid + cap 两个补丁（以 vSplit() 为分界）
    // v ∈ [0, vSplit):   mid 补丁
    // v ∈ [vSplit, 1]:   cap 补丁
    Vec3d eval(double u, double v) const;
    Vec3d normal(double u, double v) const;
    openvdb::BBoxd bbox(double u0, double u1, double v0, double v1) const;

    /// mid/cap 的分界位置（固定值，取决于刀型与刀路参数）
    double vSplit() const;

    // ─── 分补丁接口（精细控制时使用）───────────────────────────────
    bool hasMidPatch() const;  // 路径=0 或纯轴向进给时为 false

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
    double mVSplit;      // 统一参数分界点 = H_mid / (H_mid + H_cap)
    // ...
};
```

**统一接口 eval(u,v) 的连续性说明**：

统一接口在 v = vSplit 处存在 C¹ 不连续：
- v < vSplit 时，∂S/∂v 方向沿刀路（B - A）
- v > vSplit 时，∂S/∂v 方向沿角度（φ 方向）

位置 C⁰ 连续（接缝处 S_mid(u, 1) = S_cap(u, 0.5)，即 t=1 与 w=0.5 对应同一空间点），但切向突变。

**对 Phase 2 的影响**：
接缝处弦高计算可能偏大（双线性插值跨越了方向突变），导致四叉树在该区域多细分 1-2 层。但这不影响精度——最终采样点仍然落在正确的曲面上，只是多了少量冗余采样。弦高终止条件保证最终精度 ≤ t。

**优化策略**：Phase 1 分块时块边界对齐 vSplit（不跨越），可完全消除此开销。

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

### 3.3 边界裕量计算

为确保 Phase 1 的 AABB 匹配不漏掉边界 voxel，需对参数块边界外扩裕量：

```
margin = chordalLimit + ε
        = max_{块内} |S(u,v) - bilinear_interp(S(u,v))| + ε

保守估计：
- 线性刀路：margin = 0（S_mid 是直纹面，双线性插值精确）
- 端帽：margin = max_r(u) · (Δφ)² / 8  （圆弧线二阶近似）
  其中 Δφ = π / N_w（半圆分块角度步长）

实际取 margin = chordalLimit + voxelSize 即可保证保守性。
```

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
| 包络方程在 n_r=0 时退化 | 球头刀南极点（u=0）处 n_r → 0，φ_grazing 无定义 | u=0 处退化为单点，被 Phase 2 的弦高终止自然处理；Phase 1 该块 AABB 仍保守 |
| S_mid 与 S_cap 拼接缝 | 两补丁在 t=1, w=0.5 处相接 | Phase 1 分块对齐 vSplit；Phase 2 弦高终止保证采样精度 |
| 三轴非水平路径 | v 有 Z 分量时，φ_grazing 仍为常数（仅依赖 v_x, v_y）但轮廓投影改变 | 三轴固定刀轴 Z，v_z 不改变包络角，仅影响 S_mid 的 3D 位置（已含于 A+B） |
| 轴向进给（v ∥ Z） | 包络条件 n_r·cos(φ)·v_x + n_r·sin(φ)·v_y = 0 恒成立（v_x=v_y=0） | 此时无擦掠线，S_mid 不生成，全部切削由 S_cap（全圆）覆盖 |
| surfaceBBox 精度 | 采样点数不足时 AABB 可能非保守 | 加密采样（6×6 网格）+ 外扩 margin = chordalLimit + voxelSize |

### 4.3 计算量评估与对比

| 阶段 | 双补丁参数面 | 逐点 SDF 查询 |
|------|-------------|--------------|
| Phase 1 | (N_u × N_t + N_u × N_w) × 36 eval | 遍历所有 voxel，每个 voxel 需多次 SDF 求值 |
| Phase 2 | 自适应细分，采样密度与曲率匹配 | 均匀采样，大量冗余点 |
| 典型值 | 8×16 + 8×8 = 192 块，6912 次 eval | 10⁵~10⁶ 次 SDF 求值 |
| 单次耗时 | eval ≈ 10ns | SDF ≈ 50~100ns |
| 总计 | Phase 1 ≈ 0.07ms | 10~100ms |

**结论**：双补丁参数面在 Phase 1 的预分块开销可忽略，但为 Phase 2 节省了 2~3 个数量级的采样点。

---

## 5. 设计决策记录

### 5.1 ToolSweepSurface 与 ToolSweptSDF 的关系

两者描述同一几何体的不同表示：

| | ToolSweptSDF | ToolSweepSurface |
|--|---|---|
| 表示类型 | 隐式（点→距离） | 显式参数化（参数→曲面点） |
| 用途 | Phase 0/3: inside/outside 判断 | Phase 1/2: 空间定位+采样 |
| 求值方式 | 直接 O(1) | 直接 O(1) |
| 采样密度控制 | ❌ 无法自适应 | ✅ 参数域细分 = 空间精度 |
| 空间索引支持 | ❌ 需逐点判断 | ✅ 参数块 → 3D AABB 预分块 |

**交叉验证原则**：
```
∀ (u,v): |ToolSweptSDF.eval(ToolSweepSurface.eval(u,v))| ≤ ε
```
参数面上的任意点，其 SDF 绝对值应 ≤ ε（几何精度容差）——这是最强力的正确性检验。

### 5.2 为什么不直接参数化 SDF=0 等值面

理论上可以通过在 SDF=0 面上做数值 tracing 来获得参数化，但：
- 每次采样需要 Newton 迭代求根（不如解析正向求值 O(1)）
- 无法预先知道"哪部分表面对应哪个空间区域"（丧失 Phase 1 预分块能力）
- 采样密度无法从参数域控制（盲目搜索）

双补丁解析参数面是精度、效率、可控性的最优均衡。

### 5.3 独立性

ToolSweepSurface 是纯几何模块，不依赖 OpenVDB/IPW。可独立开发、独立测试、独立复用（碰撞检测、可视化等场景均适用）。

---

## 6. 实现优先级与验证策略

| 阶段 | 内容 | 验证方式 | 优先级 |
|------|------|----------|--------|
| M4.1 | 球头刀 C(u) + 静态退化 (S_cap = 全球面) | SDF 交叉验证：eval 点 SDF ≤ ε | P0 |
| M4.2 | 球头刀线性刀路 S_mid + S_cap 双补丁 | 同上 + bbox 保守性验证（AABB 包含所有 eval 点） | P0 |
| M4.3 | buildTaskList 集成双补丁 | Phase 1 测试：无漏 voxel、无多余 voxel | P1 |
| M4.4 | 平底刀/牛鼻刀 C(u) 扩展 | 三种刀型统一接口验证 | P2 |
| M4.5 | 性能基准测试 | 与逐点 SDF 查询对比，确认 2~3 个数量级提升 | P2 |

---

## 附录 A：测试策略

### A.1 单元测试矩阵

| 测试项 | 输入 | 预期 | 通过条件 |
|--------|------|------|----------|
| 球头刀轮廓 | u = 0, u_eq, 1 | z(0)=-R, z(u_eq)=0, z(1)=H_cut | 数值误差 < 1e-10 |
| 包络角 | v = (1,0,0) | φ_grazing = π/2 或 3π/2 | 面向材料侧判定正确 |
| 静态退化 | 路径长度 = 0 | hasMidPatch() = false, S_cap 覆盖全圆 | 角度范围 = 2π |
| 统一接口连续性 | v = vSplit | evalMid(u,1) == evalCap(u,0.5) | 位置误差 < 1e-10 |
| 法线一致性 | 任意 (u,v) | \|N(u,v)\| = 1 | 单位化误差 < 1e-10 |

### A.2 集成测试场景

1. **简单槽铣**：直槽，平底刀，验证 S_mid 为矩形平面
2. **型腔铣削**：球头刀，直线刀路，验证双补丁无重叠无遗漏
3. **静态钻孔**：Z 轴进给，验证 S_mid 不生成，仅 S_cap（全圆）
4. **边缘接触**：刀具部分悬空，验证 Phase 1 bbox 自动剪枝
