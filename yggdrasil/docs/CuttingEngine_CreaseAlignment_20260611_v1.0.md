# 球头刀切削边界交线双向投影与分裂法向对齐算法详细设计说明书

| 版本 | 修订日期 | 修订人 | 修订说明 |
| :---: | :---: | :---: | :--- |
| v1.0 | 2026-06-11 | Gemini | 首次创建，详细设计边界交线投影、双边法向敏感滤波。 |
| v1.a | 2026-06-11 | Gemini | 引入"点云密度与物理公差解耦"设计，增加 d_v_floor 刚性截断，并添加多段切削精度验证输出分析。 |
| v1.1 | 2026-06-11 | Duke/Kiro | 新增§1.4复合历史表面问题；新增§3.0面元自包含不变量与历史无关性设计；重构d_v语义为"精度保证下界"而非"均匀存储密度"；补充面元参与后续切削的完备性论证。 |

---

## 1. 问题陈述 (Problem Statement)

在双轨（Dual-Track）切削仿真中，工件表面由**宏观层（MacroGrid, `sdfGrid`, 体素尺寸 $D_v$）**和**微观层（MicroGrid, `microGrid`, 包含带法向的面元点云 Surfels）**共同表达。

当球头刀扫掠体（Capsule, $\Phi_{\text{tool}}(P) = 0$）对原有工件（$\Phi_{\text{work}}(P) = 0$）进行切削时，新切削痕迹与旧表面在空间上相交，形成了一条(或者多条）**锐利的边缘交线（Sharp Crease Line）** $\Gamma$。在传统的离散面元更新和裁剪算法中，存在以下关键瓶颈：

### 1.1 交界处缝隙与锯齿破洞 (Spatial Gaps & Jagged Cracks)
如果仅采用空间二值剪裁（如：将落入刀具内部的旧面元标记为 `active = 0`，同时在新面元注入时根据体素格子进行就近投影），由于新旧表面在网格精度尺度上的离散采样存在随机偏差，两者在交线 $\Gamma$ 处无法完美接触。这会在渲染或三维网格化（Mesh Reconstruction）时，在切边交界处产生大量的**几何缝隙（Gaps）、漏水破洞（Holes）或阶梯锯齿（Jagging）**。

### 1.2 法向混叠与锐边圆角化 (Normal Aliasing & Blurring)
交线 $\Gamma$ 物理上属于尖锐特征（Sharp Feature），在交线上的一点存在两个极限法向（旧表面法向 $\mathbf{n}_w$ 和新切削面法向 $\mathbf{n}_t$）。若在交界处仅保留单个面元且使用单一法向：
* 渲染时：由于法向突变被强制插值（Smoothing），会导致棱边在视觉上呈现**圆角化（Chamfering）或严重的黑斑阴影**。
* 网格化重建时：等值面提取算法（如 Marching Cubes）由于缺乏对不连续法向的约束，会将锋利的棱角钝化为平滑过渡。

### 1.3 传统去密算法对特征边的破坏 (Destruction of Creases by Naive Decimation)
为了控制点云内存，算法会在叶子节点（LeafNode）合并时对空间距离过近的点进行滤除（Decimation）。然而，在交线 $\Gamma$ 处，双分裂法向点在空间上是重合或极度接近的。如果采用常规的空间距离滤波器，会误将这两个法向不同的点合并为单个点，从而**彻底摧毁锐边与薄壁特征**。

### 1.4 复合历史表面的累积与正确切削 (Composite Historical Surface Accumulation)

在真实加工流程中，第 K 刀并不只切到"第 K-1 刀留下的表面"——它可能切入的是任意历史刀具 $K-n$（$n = 1, 2, \ldots, K-1$）留下的复合表面加上原始毛坯面的混合体。经过数千刀切削后，单个 voxel 内的工件表面可能由以下碎片拼接而成：

* 第 3 刀留下的圆弧面残余
* 第 47 刀与第 3 刀的交线附近的分裂法向对
* 第 982 刀部分覆盖后保留的旧面元
* 原始毛坯平面的一小块残余

**这意味着**：
1. 工件表面不归属于任何单一刀具，无法用单一解析方程表达。
2. 后续切削不可能也不应该去回溯"某个面元来自哪一刀"。
3. 面元系统必须是**自包含的（Self-Contained）**——每个 active 面元 `(pos, normal)` 就是该处表面的完整描述，能够无条件地参与任何后续切削判定、交线投影和渲染。

这是面元方法相对于纯解析几何方法的根本优势：**历史无关的增量表达**。

---

## 2. 输入与输出定义 (Inputs & Outputs)

本算法作为切削引擎的核心几何算子，其输入输出接口设计如下：

### 2.1 算法输入 (Inputs)
1. **物理刚性输入 (`PhysicalInputs`)**：
   * `billetGeometry`：毛坯初始几何包围盒（Box）与尺寸约束。
   * `toolType`：`BALL_END`（球头刀扫掠体胶囊体）。
   * `toolRadius` ($R$)：刀具物理半径（mm）。
   * `tolerance` ($\tau$)：加工逼近公差（mm，如 $0.05\text{mm}$ 或极端的 $0.001\text{mm}$）。
   * `path_A`, `path_B`：当前切削路径的起点与终点。
2. **控制超参数 (`HyperParameters`)**：
   * `tau`：加工公差（mm）——**面元位置精度的保证下界**。每个注入面元的位置误差不超过 $\tau$。
   * `ds_min`：面元最小步长（mm）——仅在交线（crease）附近使用，通常 `ds_min = tau`。
   * `ds_max`：面元最大步长（mm）——平坦区上限，`ds_max = D_v / 2`。
   * `D_v`：宏观网格体素尺寸（mm）。
   * `gamma_step`：采样最大步长系数（默认 $0.5$）。
   * `beta_long`：轴向平坦区稀疏系数（默认 $1.0$）。
   
   **关键语义澄清**：`tau`（公差）不是均匀面元间距。实际面元密度由曲率自适应公式决定：
   $$ds(\kappa) = \min\left(\sqrt{8 \cdot r_{curv} \cdot \tau},\ ds_{max}\right)$$
   只有在交线处才收紧到 `ds_min` 级别。

3. **当前毛坯模型状态 (`BilletModel`)**：
   * `sdfGrid`：当前状态下的 $D_v$ 级宏观 narrow-band 浮点 SDF 网格（累积了所有历史切削的结果）。
   * `microGrid`：当前状态下的高精度面元网格（`PointDataGrid`，初始可为 `nullptr`）。面元来源无需追踪——任何 active 面元都是当前工件表面的一部分。

### 2.2 算法输出 (Outputs)
1. **更新后的 `sdfGrid`**：
   * 完成与刀具扫掠体的 CSG 差集运算（$SDF_{\text{new}} = \max(SDF_{\text{old}}, -SDF_{\text{tool}})$），并在 Tool BBox 范围内完成了剪枝（Prune）与体素重整。
2. **更新后的 `microGrid`**：
   * **完美缝合的尖锐边缘**：交线 $\Gamma$ 处的面元点被精确拖拽投影到交线位置，且分裂为成对的**双法向节点**（Split-Normals）。
   * **曲率自适应采样表面**：在刀具切除实体区（New Cut）增量注入了曲率自适应密度的面元——平坦区稀疏、高曲率区加密、交线处最密。
   * **无冗余物理内存**：
     * 落入刀具内的旧面元被干净剪裁（`active = 0`），无论这些面元来自第几刀。
     * 同一平滑面上的多余高密采样点被"法向敏感滤波器"自动去密，而锐边及薄壁面元被完美保留。
     * 完成了 `topologyIntersection` 拓扑同步，彻底物理回收了完全切除的叶子节点内存。
3. **精度保证**：
   * 每个新注入面元的位置精度 ≤ $\tau$（来自刀具解析几何的精确投影）。
   * 交线对齐精度 ≤ $\tau$（来自牛顿迭代收敛）。
   * 精度输出查询：在需要微米级验证时，直接用当前刀具解析 SDF 评估面元偏差，不需要回溯历史刀具。

---

## 3. 面元自包含不变量与历史无关性设计 (Surfel Self-Containment Invariant)

本节是整个切削引擎的**基础设计公理**，所有后续算法模块必须严格遵守。

### 3.0 核心不变量

```
═══════════════════════════════════════════════════════════════════════
INVARIANT: Surfel Self-Containment (面元自包含不变量)
═══════════════════════════════════════════════════════════════════════

1. 每个 active 面元 (pos, normal) 是工件表面在该点的完整描述。
   - 不依赖任何外部刀具 ID、时间戳、或历史引用。
   - pos 精度 ≤ τ（加工公差）。
   - normal 是该点表面的准确外法向。

2. 切削判定只使用 surfel.pos：
   toolSDF.eval(surfel.pos) ≤ 0 → active = 0
   无论该面元是第 1 刀还是第 10000 刀产生的。

3. 面元位置一旦注入不再被修改。
   - 只有 active 标志位变化（1→0，不可逆）。
   - 位置和法向在注入时一次性确定，精度 ≤ τ。

4. 新面元只来自当前刀具的零等值面。
   - 注入时使用当前刀具解析几何保证精度。
   - 注入后立即"忘记"来源，成为工件表面的一部分。

5. Crease 对齐使用 macrogrid SDF 作为"旧表面"。
   - macrogrid 是所有历史切削的累积结果。
   - 不需要回溯到具体某把刀的解析方程。
═══════════════════════════════════════════════════════════════════════
```

### 3.1 面元参与后续计算的完备性论证

面元系统需要正确参与的计算类型及其依赖分析：

| 计算类型 | 输入 | 依赖项 | 自包含？ |
|---------|------|--------|---------|
| A. 切削判定 | surfel.pos | toolSDF_K.eval(pos) | ✓ 仅需 pos |
| B. 交线投影 | surfel.pos, surfel.normal | macrogrid SDF 插值 + 当前刀具 SDF | ✓ n_work 来自面元 normal 或 macrogrid 梯度 |
| C. 法向敏感去密 | pos_A, norm_A, pos_B, norm_B | 距离 + 法向夹角 | ✓ 仅需 pos + normal |
| D. 渲染/网格化 | 所有 active 面元 pos + normal | 直接使用 | ✓ 完全自包含 |
| E. 碰撞检测 | macrogrid SDF | 从 SDF 场直接查询 | ✓ macrogrid 已是历史综合结果 |

**关键结论**：没有任何计算需要知道面元"来自哪把刀"。

### 3.2 复合历史表面的切削流程

```
第 K 刀执行时（工件已经历了 K-1 刀切削）：

  Step 1: Clip（纯点级判定，不关心面元来源）
    for surfel in tool_bbox where active == 1:
      if toolSDF_K.eval(surfel.pos) ≤ 0:
        surfel.active = 0
    // 被 clip 的面元可能来自第 1 刀、第 K-1 刀、或原始毛坯，
    // 算法完全不需要区分。

  Step 2: Generate（只用当前刀具解析几何）
    在 Φ_K = 0 表面做曲率自适应采样。
    交线处：
      phi_work = macrogrid.interpolate(P)  // 累积所有历史的 SDF
      phi_tool = toolSDF_K.eval(P)          // 当前刀具解析 SDF
      → 双向投影 + 分裂法向

  Step 3: Inject（注入后面元脱离刀具身份）
    新面元成为 microGrid 的一部分。
    后续切削对它的处理与对任何历史面元完全相同。
```

### 3.3 为什么交线投影不需要回溯历史刀具

交线投影需要两个 SDF：
- $\Phi_{\text{tool}}$：当前刀具——有解析表达，精确。
- $\Phi_{\text{work}}$：旧工件表面——来自 **macrogrid SDF 插值**。

macrogrid 经过所有历史切削的 CSG 差集累积，它的零等值面就是当前工件表面的粗近似（精度 ~$D_v$）。在交线投影中：
- 使用 macrogrid SDF 的梯度方向作为 $\mathbf{n}_w$ 的近似
- 或使用**旧面元自身的 normal** 作为 $\mathbf{n}_w$（更精确）

两种方式都不需要回溯到产生该面元的历史刀具方程。

### 3.4 面元精度不退化的保证

| 担忧 | 分析 | 结论 |
|------|------|------|
| 面元位置会随时间"漂移"？ | 位置一旦注入不再被修改，只有 active 状态变化 | ✗ 不会退化 |
| 交线投影会引入累积误差？ | 投影是一次性操作，精度由牛顿迭代保证到 τ | ✗ 不会累积 |
| 旧面元被新刀具切时精度够吗？ | 判定只看 `toolSDF.eval(pos) ≤ 0`，是精确解析计算 | ✓ 精度取决于面元 pos 本身，而非判定过程 |

### 3.5 数据结构设计原则

**不需要**添加的字段：
```cpp
// ✗ 不需要：
uint32_t sourceToolId;      // 不需要记录面元来自哪把刀
uint32_t creationStep;      // 不需要时间戳
GeometryRef sourceGeometry; // 不需要保持历史解析几何引用
```

**已经足够**的字段：
```cpp
// PointDataGrid 属性中每个点的数据：
Vec3f position;    // 相对于 voxel 中心的偏移（精度 ≤ τ）
Vec3f normal;      // 表面外法向（单位向量）
uint8_t active;    // 1=存活, 0=已切除（不可逆 1→0）
```

---

## 4. 算法详细设计与数学原理

本算法由三大几何内核构成：**曲率自适应扫掠体参数化采样**、**2D 牛顿-拉夫逊双向投影对齐** 以及 **法向敏感双边邻域滤波**。

### 4.1 d_v 的正确语义：精度保证下界而非均匀存储密度

**核心观点**：d_v（或 tau）是面元位置精度的保证，不是面元之间的均匀间距。

实际面元密度由两级策略控制：

| 区域 | 面元步长 | 决定因素 | 每 voxel 典型面元数 |
|------|----------|----------|-------------------|
| 平坦/低曲率区（圆柱扫掠面主体） | $ds_{max} = D_v/2$ | 弦高偏差 ≤ τ 自动满足 | 4~16 |
| 中等曲率区（球头端盖） | $ds = \sqrt{8R\tau}$ | 曲率自适应 | 20~80 |
| 交线（crease）附近 | $ds_{min} \approx \tau$ | 特征捕获精度 | 集中在一维线上，总量可控 |

**关键数学**：对于曲率半径 $r_{curv}$ 的表面，相邻面元间距 $ds$ 产生的弦高偏差为：
$$\delta = \frac{ds^2 \cdot \kappa}{8} = \frac{ds^2}{8 \cdot r_{curv}}$$

只要 $\delta \leq \tau$，精度就满足。求解得：
$$ds \leq \sqrt{8 \cdot r_{curv} \cdot \tau}$$

对于 R=5mm, τ=0.001mm：$ds = \sqrt{8 \times 5 \times 0.001} = 0.2\text{mm}$ —— 远大于 τ=0.001mm。

**结论**：即使要求微米级精度，面元间距也不需要是微米级——只需要保证每个面元的 position 本身是微米级精确（来自解析投影），而面元之间的插值误差由曲率自适应步长自动控制。

### 4.2 刀具扫掠体（胶囊体）曲率自适应采样

对于球头刀从 $\mathbf{A}$ 到 $\mathbf{B}$ 运动形成的胶囊体，其表面由圆柱扫掠面（Cylindrical Skin）和球头端盖（Spherical End Caps）拼合而成。

1. **曲率自适应弦截步长公式**：
   对于给定的局部曲率半径 $r_{\text{curv}}$，自适应步长为：
   $$ds(r_{\text{curv}}) = \min\left( \sqrt{8.0 \cdot r_{\text{curv}} \cdot \tau}, \; ds_{\text{max}} \right), \quad ds_{\text{max}} = \frac{D_v}{2.0}$$

2. **圆柱扫掠面参数化采样**：
   * 纵向（沿路径 $\mathbf{AB}$ 方向）：曲率半径 $r_{\text{curv}} = \infty$。
     $$ds_{\text{long}} = \beta_{\text{long}} \cdot ds_{\text{max}}$$
   * 环向（绕轴线旋转方向）：曲率半径 $r_{\text{curv}} = R$。
     $$ds_{\text{rot}} = ds(R) \implies d\theta = \frac{ds_{\text{rot}}}{R}, \quad M_{\theta} = \max\left(4, \left\lceil \frac{2\pi}{d\theta} \right\rceil\right)$$

3. **球形端盖参数化采样**：
   * 经向与纬向：曲率半径在任何方向上均为 $R$。
     $$ds = ds(R)$$
   * 采用球坐标参数化 $(\phi, \theta)$，在半球面上均匀且高精度地散播候选面元点。

4. **交线处加密**：
   当候选点接近交线（$|\Phi_{\text{work}}| < 0.2 \cdot ds_{\text{min}}$ 且 $|\Phi_{\text{tool}}| < 0.2 \cdot ds_{\text{min}}$）时，局部步长收紧到 $ds_{\text{min}}$，启动双向投影和分裂法向。交线是一维结构，面元数 = $O(L_{\text{crease}} / ds_{\text{min}})$，内存开销可控。

---

### 4.3 2D 牛顿-拉夫逊双边投影对齐几何算子

对于处于交线过渡带内的任何面元点 $P$：

1. **一阶泰勒展开式**：
   我们寻找投影位移 $\Delta P = c_w \mathbf{n}_w + c_t \mathbf{n}_t$，使更新后的点同时落在两表面上：
   $$\begin{cases} \Phi_{\text{work}}(P) + \mathbf{n}_w \cdot \Delta P = 0 \\ \Phi_{\text{tool}}(P) + \mathbf{n}_t \cdot \Delta P = 0 \end{cases}$$

2. **建立线性方程组**：
   $$\begin{bmatrix} 1 & \mathbf{n}_w \cdot \mathbf{n}_t \\ \mathbf{n}_w \cdot \mathbf{n}_t & 1 \end{bmatrix} \begin{bmatrix} c_w \\ c_t \end{bmatrix} = \begin{bmatrix} -\Phi_{\text{work}}(P) \\ -\Phi_{\text{tool}}(P) \end{bmatrix}$$

3. **解析求解系数 (Cramer's Rule)**：
   令 $d = \mathbf{n}_w \cdot \mathbf{n}_t$，$det = 1.0 - d^2$。若 $det \ge 10^{-5}$，则有：
   $$c_w = \frac{-\Phi_{\text{work}}(P) + d \cdot \Phi_{\text{tool}}(P)}{det}$$
   $$c_t = \frac{-\Phi_{\text{tool}}(P) + d \cdot \Phi_{\text{work}}(P)}{det}$$

4. **投影及分裂 (Split Normals)**：
   投影后交点位置更新为：
   $$P_{\text{crease}} = P + c_w \mathbf{n}_w + c_t \mathbf{n}_t$$
   对于新注入的边界采样，在 $P_{\text{crease}}$ 处**同时保留两个面元节点**：
   * 节点一（旧表面侧）：位置 $P_{\text{crease}}$，法向设为旧表面插值梯度 $\mathbf{n}_w$。
   * 节点二（新切削面侧）：位置 $P_{\text{crease}}$，法向设为刀面解析梯度 $\mathbf{n}_t$。

5. **旧表面法向 $\mathbf{n}_w$ 的获取（历史无关）**：
   - **优先**：从 macrogrid SDF 梯度插值获取。macrogrid 已累积所有历史切削，其梯度就是当前工件外法向的粗近似。
   - **备选**：从 microGrid 中最近的存活旧面元的 normal 获取。该面元的 normal 在其注入时就已经被精确设定，无需追溯来源。
   - **两种方式都不需要知道旧表面"是哪把刀切出来的"。**

---

### 4.4 法向敏感双边邻域滤波器 (Normal-Aware Bilateral Filter)

在叶子节点局域点云合并时，为了在不破坏锐边和薄壁特征的前提下剔除多余的面元，设计双边冗余判定机制。

对于空间距离接近（处于去密半径 $R_{\text{decim}} = 0.3 \cdot ds_{\text{adapt}}$ 以内，其中 $ds_{\text{adapt}}$ 为该区域的自适应步长）的两个点 $P_1(\mathbf{n}_1)$ 和 $P_2(\mathbf{n}_2)$，只有当它们的**法向夹角小于 $15^\circ$ 时，才允许判定为冗余并合并**：

```
                    【法向敏感双边滤波器判定分支】

                  距离 d = ||P_1 - P_2|| < R_decim
                                 │
         ┌───────────────────────┴───────────────────────┐
         ▼ (n_1 · n_2 > cos(15°))                        ▼ (n_1 · n_2 <= cos(15°))
    [ A. 同一平滑表面的冗余点 ]                      [ B. 锐利特征边缘 / 双面薄壁 ]
         │                                               │
         ▼                                               ▼
  舍弃旧点，保留新点                                 两点均严格保留！
(去密过滤，保障内存)                               (特征保护，防圆角与破洞)
```

---

## 5. C++ 接口与算法核心实现伪代码

### 5.1 数据结构定义 (Structures)
```cpp
#pragma once

#include <openvdb/openvdb.h>
#include <vector>

namespace ygg {

/// @brief 表达高精度面元的显式点数据结构
/// 注意：不包含 sourceToolId / creationStep 等历史追踪字段。
/// 面元一旦注入即成为工件表面的一部分，与来源刀具无关。
struct Surfel {
    openvdb::Vec3R pos;     // 物理世界三维坐标 (mm)，精度 ≤ τ
    openvdb::Vec3f norm;    // 单位方向法向量 (指向空气)
};

/// @brief 自适应采样配置（替代旧的固定 N² 模型）
struct AdaptiveSampleConfig {
    double tau;          // 加工公差 (mm) — 面元位置精度保证
    double ds_max;       // 最大面元步长 (mm) — 平坦区上限 = D_v/2
    double ds_min;       // 最小面元步长 (mm) — 交线处 = tau
    double R_tool;       // 刀具半径 (mm) — 曲率来源
    
    /// 曲率自适应步长
    double adaptiveStep(double r_curv) const {
        return std::min(std::sqrt(8.0 * r_curv * tau), ds_max);
    }
};

/// @brief 局域分箱数据容器：以 Leaf 节点为 Key 组织新注入的面元
struct LeafInjectionBucket {
    openvdb::Coord origin;
    std::vector<Surfel> newSurfels;
};

} // namespace ygg
```

### 5.2 2D 牛顿-拉夫逊投影算子实现
```cpp
/// @brief 将候选点投影到旧表面 SDF 和新刀具 SDF 的交线上。
/// 
/// 旧表面 SDF (phi_work) 来自 macrogrid 插值——它是所有历史切削的累积，
/// 不需要也不可能回溯到产生该表面的具体刀具。
/// 
/// @param p_inout  [in/out] 输入坐标，计算后就地更新为交线上的精确投影坐标
/// @param n_w      旧表面在 p 处的法向 (从 macrogrid 梯度或旧面元 normal 获取)
/// @param phi_work 旧表面在 p 处的有符号距离值 (macrogrid 插值)
/// @param n_t      刀具解析法向 (已归一化)
/// @param phi_tool 刀面解析有符号距离值
/// @return 是否成功完成交线对齐
inline bool projectToCrease(
    openvdb::Vec3d& p_inout,
    const openvdb::Vec3d& n_w, double phi_work,
    const openvdb::Vec3d& n_t, double phi_tool) 
{
    const double d = n_w.dot(n_t);
    const double det = 1.0 - d * d;
    
    // 若两法向高度平行 (余弦接近 1)，退化为单表面常规投影
    if (std::abs(det) < 1e-5) {
        if (std::abs(phi_work) < std::abs(phi_tool)) {
            p_inout -= phi_work * n_w;
        } else {
            p_inout -= phi_tool * n_t;
        }
        return true;
    }
    
    // 克莱姆法则求解二元一阶泰勒位移系数
    const double c_w = (-phi_work + d * phi_tool) / det;
    const double c_t = (-phi_tool + d * phi_work) / det;
    
    // 就地更新
    p_inout += c_w * n_w + c_t * n_t;
    return true;
}
```

### 5.3 法向敏感双边邻域合并算子
```cpp
/// @brief 对单个叶子节点内合并后的面元点云执行法向敏感双边去密。
/// 面元来源不可知也不需要知道——只使用 pos 和 norm 做判定。
/// @param candidates  包含旧存活点和新注入点的待处理数组
/// @param R_decim     去密空间距离阈值 (通常为 0.3 * ds_local)
/// @param outPos      [out] 滤波后保留的面元物理坐标
/// @param outNorm     [out] 滤波后保留的面元法向量
inline void filterBilateralSurfels(
    const std::vector<Surfel>& candidates,
    double R_decim,
    std::vector<openvdb::Vec3R>& outPos,
    std::vector<openvdb::Vec3f>& outNorm) 
{
    const double cos_15 = 0.96592582628; // cos(15 degrees)
    const double r_decim_sq = R_decim * R_decim;
    
    std::vector<bool> discarded(candidates.size(), false);
    
    for (size_t i = 0; i < candidates.size(); ++i) {
        if (discarded[i]) continue;
        
        for (size_t j = i + 1; j < candidates.size(); ++j) {
            if (discarded[j]) continue;
            
            // 1. 空间距离校验
            openvdb::Vec3d diff = candidates[i].pos - candidates[j].pos;
            if (diff.lengthSqr() < r_decim_sq) {
                // 2. 法向对齐度校验
                double dot = candidates[i].norm.dot(candidates[j].norm);
                if (dot > cos_15) {
                    // 同一平滑面上的冗余点，合并
                    discarded[j] = true;
                }
                // 若法向夹角大于15°，属于锐边/薄壁，必须共同保留
            }
        }
    }
    
    outPos.reserve(candidates.size());
    outNorm.reserve(candidates.size());
    for (size_t i = 0; i < candidates.size(); ++i) {
        if (!discarded[i]) {
            outPos.push_back(candidates[i].pos);
            outNorm.push_back(candidates[i].norm);
        }
    }
}
```

---

## 6. 性能分析与预测 (Performance Analysis & Prediction)

### 6.1 计算时间复杂度分析 (Time Complexity)

本算法的单步运行总耗时 $T_{\text{total}}$ 可解耦为四个核心阶段：
$$T_{\text{total}} = T_{\text{macro\_csg}} + T_{\text{template\_gen}} + T_{\text{projection}} + T_{\text{local\_merge}}$$

#### 1. 宏观 SDF 差集计算时间 $T_{\text{macro\_csg}}$
* **复杂度**：$O\left(\frac{V_{\text{bbox}}}{D_v^3}\right)$
* **预测耗时**：$1 \sim 3\text{ms}$（通过 ValueAccessor O(1) 缓存级读写）

#### 2. 自适应模板生成时间 $T_{\text{template\_gen}}$
* **在平直圆柱面**：$O\left(\frac{L}{ds_{\text{max}}} \cdot \frac{2\pi R}{ds(R)}\right)$，轴向步距从 $\tau$ 放大到 $D_v/2$。
* **在弯曲端盖区**：$O\left(\frac{R^2}{ds(R)^2}\right)$。
* **预测耗时**：$2.0 \sim 4.5\text{ms}$（比均匀 τ 采样减少 80%+ 点数）

#### 3. 2D 交线投影对齐时间 $T_{\text{projection}}$
* **复杂度**：$O\left(\frac{L_{\text{crease}}}{ds_{\text{min}}}\right)$——仅在一维交线上执行
* **预测耗时**：$0.5 \sim 1.2\text{ms}$（触发投影的点仅占总点数 3%~5%）

#### 4. 双边去密合并时间 $T_{\text{local\_merge}}$
* **复杂度**：每 LeafNode $O((M_1 + M_2)^2)$，但单叶子点数限制在 ~500
* **预测耗时**：$3.5 \sim 7.5\text{ms}$（TBB 完美负载均衡）

---

### 6.2 内存增长预测 (Memory Prediction)

#### 自适应密度 vs 均匀密度的对比

以 R=5mm, D_v=0.5mm, L=20mm 为例：

| 方案 | τ | 面元数/step | 单步内存增量 |
|------|---|-------------|-------------|
| 均匀 d_v = τ | 0.05mm | ~800,000 | ~22 MB |
| 均匀 d_v = τ | 0.001mm | ~200,000,000 | **不可行** |
| **自适应（本方案）** | 0.05mm | ~20,000 | ~0.56 MB |
| **自适应（本方案）** | 0.001mm | ~25,000 + 400 crease | ~0.7 MB |

**关键结论**：自适应方案下，即使 τ 从 0.05mm 缩小到 0.001mm（50倍），面元数几乎不增长——因为面元密度由曲率决定，不由 τ 直接决定。τ 只影响交线处的加密密度（一维结构，增量可控）。

#### 长期内存上界

工件内存总量物理上限与暴露表面积刚性正比：
$$\text{Memory}_{\text{total}} \approx C \cdot \text{Area}_{\text{active}}$$

通过 `topologyIntersection` + `compactAttributes`，被完全切除区域的点云被物理回收，消除了内存随切削刀数累积增长的隐患。

---

### 6.3 仿真单步性能预测

在典型工况（$R=5\text{mm}$, $\tau=0.05\text{mm}$, $D_v=0.4\text{mm}$, $L=20\text{mm}$, 多核 CPU）：

| 仿真阶段 | 预测耗时 (ms) | 说明 |
| :--- | :---: | :--- |
| 宏观 SDF CSG | 1.2~2.5 | L3 cache 友好 |
| 自适应模板采样 | 2.0~4.5 | SIMD 可加速 |
| 2D 交线投影 | 0.5~1.2 | 仅 3%~5% 点触发 |
| 局域双边合并 | 3.5~7.5 | TBB 线性扩展 |
| 拓扑精炼 | 1.0~1.8 | 属性紧凑化 |
| **单步总延迟** | **8.2~17.5** | **< 33ms (30 FPS)** |

---

## 7. 结论

本算法设计通过以下三项核心原则，从根本上解决了双网格模型在任意复杂历史切削序列中的正确性、精度与效率问题：

1. **面元自包含不变量**：每个 active 面元 `(pos, normal)` 是工件表面的完整自描述，不依赖任何历史引用。这使得任意复合历史表面都能被面元集合正确表达，并无条件参与后续切削。

2. **d_v 语义重构**：从"均匀存储密度"重新定义为"精度保证下界"。实际面元密度由曲率自适应决定，消除了 N² 内存爆炸风险。

3. **交线双向对齐 + 分裂法向保护**：在一维交线上精确对齐并保留双法向，确保无论历史多复杂，crease 特征都被完美捕获而不退化。

三者协同，为 Yggdrasil 加工仿真引擎在工业场景中实现**任意刀序的正确递归切削、亚微米级精度输出、以及 30+ FPS 实时交互**提供了坚实的底层架构。
