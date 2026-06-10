# BilletBuilder 重构设计方案：双轨并行与高性能几何采样

**文档编号**：`Refactor_BilletBuilder_20260604_v1.0.md`  
**状态**：设计中 (Under Review)  
**作者**：Gemini CLI / Duke  
**日期**：2026-06-04  
**前置文档**：`Algo_R2_BilletBuilder_20260603_v0.f.md`

---

## 1. 重构动机与目标

现有的 `BilletBuilder` 采用“先构建 SDF 场，再反向搜索采样点”的逻辑。在大规模高精度需求下，这种方式存在严重的计算冗余。本次重构旨在通过**几何原生采样 (Analytic/Geometric Sampling)** 路径，压榨算法潜能，实现秒级构建千万级面元。

### 核心目标：
1.  **计算性能极致化**：利用参数化方程和几何拓扑直接生成点云，规避全局体素遍历。
2.  **多模型支持**：原生支持长方体 (Box)、圆柱 (Cylinder) 及任意闭合网格 (Mesh)。
3.  **内存管控**：通过属性量化和延迟分配，确保 IPW 链条在长时间仿真中的内存可控。
4.  **精度补偿**：确立“微观精度 (Micro) 补偿宏观数值 (Macro)”的策略。

---

## 2. 架构设计：双轨并行构建 (Dual-Path Construction)

我们将构建逻辑拆分为两个独立但同步的流水线：

### 2.1 Macro-Path (体积轨)
*   **目标**：构建 `FloatGrid` (VoxelSize = $D_v$)。
*   **策略**：**数值近似 (Numerical Approximation)**。
    *   对于 Box/Cylinder：使用 OpenVDB 内置解析生成工具。
    *   对于 Mesh：使用 `meshToLevelSet`。
    *   **关键准则**：宏观 SDF 不需要绝对精确的距离值，只需确保符号（内部 < 0，外部 > 0）正确，作为切削过滤的快查索引。

### 2.2 Micro-Path (表面轨)
*   **目标**：构建 `PointDataGrid` (共享 $D_v$ Transform)。
*   **策略**：**解析采样 (Analytic Sampling)**。
    *   直接从几何定义出发，按照 $k \cdot d_v$ (毛坯精度) 间距生成面元。
    *   点云生成后直接批量注入 (Bulk Insert) 到 VDB 树结构中。

---

## 3. 详细算法实现

### 3.1 长方体 (Box) 构建器
*   **SDF**：调用 `tools::createLevelSetBox`。
*   **Surfel**：
    1.  遍历 6 个面。
    2.  对每个矩形面按 $d_{v\_init}$ 步长进行 2D 循环采样。
    3.  直接赋予面法向 $\vec{n} \in \{ \pm x, \pm y, \pm z \}$。

### 3.2 圆柱体 (Cylinder) 构建器
*   **SDF**：调用 `tools::createLevelSetCylinder`。
*   **Surfel**：
    1.  **侧面**：在 $(\theta, z)$ 参数空间采样。$d\theta = d_{v\_init} / R$。
    2.  **端面**：在极坐标 $(r, \theta)$ 采样。
    3.  法向根据参数方程 $\vec{n} = (\cos\theta, \sin\theta, 0)$ 解析计算。

### 3.3 Mesh 构建器
*   **SDF**：使用 `tools::meshToLevelSet` 生成 $D_v$ 精度的窄带网格。
*   **Surfel**：**面积权重重要性采样 (Area-Weighted Sampling)**。
    1.  计算所有三角面片面积，构建 CDF。
    2.  并行在各面片上撒点，点数 $N_{points} = \text{TotalArea} / d_{v\_init}^2$。
    3.  **高性能优化**：直接继承面片法向，避免后期的最近邻查找。

---

## 4. 性能压榨路径 (Optimization Path)

### 4.1 算法级优化 (第一优先级)
*   **Batch Transformation**：利用 OpenVDB Points 的底层接口，将生成的所有点作为连续内存块一次性注入。避免逐点调用 `appendPoint` 导致的树结构频繁重构。
*   **Zero-Copy Normal Assignment**：在采样阶段直接关联法向，不再进行昂贵的 SDF 梯度运算。
*   **Spatial Partitioning Pre-sort**：在注入前对采样点按 $D_v$ 体素坐标进行 Z-order 排序，提升 CPU 缓存命中率。

### 4.2 数据布局优化 (内存管控)
*   **Attribute Quantization (属性量化)**：
    *   `Position`: 存储为 `Vec3f` (局部体素偏移)。
    *   `Normal`: 存储为 `Vec3f` (或量化为 16-bit 极坐标)。
    *   `ActiveGroup`: 必须使用 `AttributeGroup`。
*   **Lazy Pruning**：构建完成后执行 `tools::prune`，剔除不包含任何活跃点的 Tile 和 LeafNode。

---

## 5. 精度与数据结构准则

### 5.1 共享 Transform 约束
*   `FloatGrid` 与 `PointDataGrid` **必须**共享同一个 `Transform`。
*   $D_v$ 必须是 $d_v$ 的整数倍。

### 5.2 精度等级 (Precision Tiers)
| 类型 | 间距规格 (Default) | 灵活范围 | 属性标记 |
| :--- | :--- | :--- | :--- |
| **IPW₀ (Blank)** | $k \cdot d_v$ (通常 $k=10 \sim 20$) | $1.0 \cdot d_v \sim 20 \cdot d_v$ | `Precision = 0` (COARSE/INIT) |
| **Cutting Surface** | $d_v$ | 固定为 $d_v$ | `Precision = 1` (FINE) |

**注：自适应初始精度策略**  
重构后的 `BilletBuilder` 将 $d_{v\_init}$ 作为一个开放配置项。若毛坯规模较小或硬件资源充足，用户可以将 $k$ 设为 $1.0$，使初始毛坯表面直接达到最终切削精度。系统默认推荐 $k \ge 10$ 仅作为大规模超实时仿真的性能压榨手段。

### 5.3 宏微对齐一致性
*   **准则**：若某体素 $(i, j, k)$ 的 `FloatGrid` 值为正（空气），则该体素在 `PointDataGrid` 中理论上不应包含 `active` 的面元。
*   **实现**：构建时以几何边界为准，允许 SDF 在边界附近有 $\pm 0.5 D_v$ 的波动，但微观面元必须严格落在几何定义的表面上。

---

## 6. 接口预览 (C++ Strategy)

```cpp
class BilletBuilder {
public:
    struct Params {
        GeometryType type;
        BoxParams box;
        CylParams cyl;
        MeshParams mesh;
        ResolutionConfig config;
    };

    // 重构后的统一入口
    BilletModel build(const Params& p);

private:
    // 各几何类型的专用采样器 (Analytic/Geometric)
    void sampleBox(const BoxParams& p, std::vector<Vec3f>& pts, std::vector<Vec3f>& normals);
    void sampleCylinder(const CylParams& p, std::vector<Vec3f>& pts, std::vector<Vec3f>& normals);
    void sampleMesh(const MeshParams& p, std::vector<Vec3f>& pts, std::vector<Vec3f>& normals);
};
```

---

## 7. 方案评估 (Evaluation)

*   **构建速度**：预计对于 500mm 规模的毛坯，构建时间从 2s+ 降至 < 100ms。
*   **内存开销**：由于 IPW₀ 采用了 20x 的稀疏采样，初始内存占用将减少约 400 倍。
*   **健壮性**：解耦后，即使复杂的 Mesh 导致 SDF 产生拓扑缺陷（不闭合等），只要面元采样是连续的，切削计算依然能得到正确的结果。

---

## 修订记录

| 版本 | 日期 | 修订内容 |
| :--- | :--- | :--- |
| v1.0 | 2026-06-04 | 首次发布重构方案，确定双轨并行与几何原生采样策略。 |
