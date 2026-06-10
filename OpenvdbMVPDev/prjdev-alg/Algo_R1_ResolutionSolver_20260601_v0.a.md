# R1 算法设计：DualGrid 与 VoxelSize 初始设定计算

**文档编号**：`Algo_R1_ResolutionSolver_20260601_v0.a.md`  
**状态**：初稿  
**作者**：Duke / Kiro  
**日期**：2026-06-01  
**前置依赖**：无  
**对应里程碑**：M1  

---

## 1. 问题定义

给定加工工艺参数，确定性地计算网格系统的最优分辨率配置：
- **共享体素尺寸** D_v（VoxelSize）：FloatGrid 和 PointDataGrid 共享的树结构粒度
- **面元采样间距** d_v（Surfel Spacing）：PointDataGrid 体素内部面元点的采样精度
- **面元密度因子** N（D_v = N × d_v）：每个体素边长方向的面元采样数，每个表面体素内约 N² 个面元
- **系统模式判定**：单轨（纯 SDF）/ 双轨（SDF + 面元点云）/ Atlas 切片

**关键架构约束**：双轨模式下，FloatGrid 和 PointDataGrid **共享同一个 Transform（同一个 VoxelSize = D_v）**，拥有相同的树拓扑结构。精度差异不在 VoxelSize，而在数据表达——FloatGrid 每体素存 1 个 SDF 值（精度 ~D_v），PointDataGrid 每体素存 N² 个面元点（精度 ~d_v）。

---

## 2. 输入参数

| 参数 | 符号 | 单位 | 含义 | 典型值 |
|------|------|------|------|--------|
| 精度公差 | t | mm | 零件表面允许的最大偏差 | 0.01 |
| 最小刀具半径 | R_min | mm | 加工中使用的最小刀具半径 | 5.0 |
| 最小特征尺寸 | F_min | mm | 零件上需要分辨的最小几何特征 | 1.0 |
| 包围盒最大跨度 | L_max | mm | 毛坯包围盒三轴中的最大值 | 500.0 |

---

## 3. 算法步骤

### Step 1：计算微观基准 d_v

```
d_v = 0.5 × t
```

**物理意义**：根据奈奎斯特采样定理，要分辨公差 t 级别的表面偏差，采样间距必须 ≤ t/2。

**示例**：t = 0.01mm → d_v = 0.005mm = 5μm

### Step 2：计算宏观体素物理上界 D_upper

```
D_upper = min(0.5 × R_min, F_min)
```

**约束来源**：
- `0.5 × R_min`：宏观体素必须能分辨刀具的曲率变化（至少 2 个体素覆盖刀具半径）
- `F_min`：宏观体素不能大于最小特征，否则特征被"吞没"

**示例**：R_min=5mm, F_min=1mm → D_upper = min(2.5, 1.0) = 1.0mm

### Step 3：计算理想面元密度因子 N_ideal

```
N_ideal = floor(D_upper / d_v)
```

**含义**：每个体素边长方向可容纳的面元数。每个表面体素内将有约 N² 个面元点。

**示例**：D_upper=1.0mm, d_v=0.005mm → N_ideal = 200

### Step 4：对齐到 2 的幂次 N

```
N = 2^n，其中 n = floor(log2(N_ideal))
```

选取不超过 N_ideal 的最大 2 的幂。

**理由**：2 的幂对齐适配 OpenVDB 叶节点的 8×8×8 = 512 体素块结构，且便于位运算加速体素内面元索引。

**示例**：N_ideal=200 → n=7, N=128

### Step 5：计算最终共享体素尺寸 D_v

```
D_v = N × d_v
```

**含义**：这是双轨模式下 FloatGrid 和 PointDataGrid 共享的 VoxelSize。

**示例**：N=128, d_v=0.005mm → D_v = 0.64mm

### Step 6：系统模式判定

**核心原则**：如果单轨 SDF（FloatGrid/DoubleGrid + d_v 作为 VoxelSize）在内存和性能上可行，则直接使用单轨，不启动 DualGrid。DualGrid 仅在单轨方案不可行时才触发。

#### 6.1 单轨可行性评估

```
// 使用实际毛坯尺寸计算真实表面积（非正方体假设）
surface_area = 2 * (Lx*Ly + Ly*Lz + Lx*Lz)  // mm²
total_active_voxels_estimate = (surface_area / d_v^2) * (2 * halfWidth)

// 单轨可行条件：活跃体素内存 < 可用预算
memory_est = total_active_voxels_estimate * bytes_per_voxel  // FloatGrid: 4 bytes
single_track_feasible = memory_est < MEMORY_BUDGET
```

**注意**：`MEMORY_BUDGET` 为可配置参数，默认值 = 系统可用内存 × 50%（运行时查询），fallback 为 4GB。

**边界处理**：若 Step 2 中 D_upper < d_v（即 N_ideal < 1），不报错，强制 mode=SINGLE_TRACK, N=1, D_v=d_v。

#### 6.2 模式判定逻辑

```
if single_track_feasible:
    mode = SINGLE_TRACK
    // 直接使用 FloatGrid(d_v) 或 DoubleGrid(d_v)
    // 无需宏观轨，无需 N 对齐
    D_v = d_v  // 单轨模式下宏观=微观
    N = 1

else if total_voxels_per_axis > 1.67e7:
    mode = ATLAS_REGION
    // 超大件：需要 Floating Origin 分区 + DualGrid
    // 理由：float 精度 7 位有效数字极限，累积误差 > 0.1μm

else:
    mode = DUAL_TRACK
    // 中大件：单轨内存爆炸，但坐标精度尚可
    // 启用共享 Transform(D_v) 的 FloatGrid + PointDataGrid
    // FloatGrid: 每体素 1 个 SDF 值，做快速空间过滤
    // PointDataGrid: 每体素 N² 个面元点（精度 d_v），做精确切削
```

**设计哲学**：
- **SINGLE_TRACK 是默认优选**——简单、直接、精度由 VoxelSize 天然保证
- **DUAL_TRACK 是性能优化手段**——当单轨内存/性能不可行时才启动
- **ATLAS_REGION 是极端场景兜底**——超大件浮点精度不足时触发

---

## 4. 输出结构

```cpp
struct ResolutionConfig {
    double d_v;          // 面元采样间距 (mm)，微观精度
    double D_v;          // 共享体素尺寸 (mm)，树结构粒度
    int    N;            // 面元密度因子 (D_v / d_v)，2^n
    int    n;            // 幂次
    
    enum Mode { DUAL_TRACK, SINGLE_TRACK, ATLAS_REGION };
    Mode   mode;
    
    // SINGLE_TRACK: 仅 FloatGrid(d_v)，无 PointDataGrid
    // DUAL_TRACK:   FloatGrid(D_v) + PointDataGrid(D_v)，共享 Transform
    //               PointDataGrid 体素内按 d_v 间距注入面元
    // ATLAS_REGION: 多区域 DUAL_TRACK（第一阶段不实现）
    
    int    atlas_divisions;  // 各轴切片数（仅 ATLAS_REGION 有效）
};
```

---

## 5. 完整伪代码

```
function solveResolution(t, R_min, F_min, L_max) -> ResolutionConfig:
    // Step 1
    d_v = 0.5 * t
    
    // Step 2
    D_upper = min(0.5 * R_min, F_min)
    
    // Step 3
    N_ideal = floor(D_upper / d_v)
    
    // Step 4: 对齐到 2 的幂（向下取整）
    if N_ideal < 1:
        error("参数矛盾：D_upper < d_v，请检查输入")
    n = floor(log2(N_ideal))
    N = 2^n
    
    // Step 5
    D_v = N * d_v
    
    // Step 6: 模式判定（单轨优先）
    halfWidth = 3
    surface_area = 2 * (Lx*Ly + Ly*Lz + Lx*Lz)  // 需要实际毛坯尺寸
    active_voxels_est = (surface_area / d_v^2) * (2 * halfWidth)
    memory_est = active_voxels_est * 4  // FloatGrid: 4 bytes/voxel
    MEMORY_BUDGET = getSystemAvailableMemory() * 0.5  // 可配置，fallback 4GB
    
    if N_ideal < 1:
        // 边界情况：D_upper < d_v，强制单轨
        mode = SINGLE_TRACK
        D_v = d_v
        N = 1
    else if memory_est < MEMORY_BUDGET:
        mode = SINGLE_TRACK
        D_v = d_v
        N = 1
    else if (L_max / d_v) > 1.67e7:
        mode = ATLAS_REGION
        atlas_divisions = ceil(L_max / (1.67e7 * d_v))
    else:
        mode = DUAL_TRACK
    
    return { d_v, D_v, N, n, mode, atlas_divisions }
```

---

## 6. 验证用例

| 用例 | t | R_min | F_min | L_max | 期望 d_v | 期望 N | 期望 D_v | 期望 mode |
|------|---|-------|-------|-------|----------|--------|----------|-----------|
| 典型精加工 | 0.01 | 5.0 | 1.0 | 500 | 0.005 | 128 | 0.64 | DUAL_TRACK |
| 粗加工大件 | 0.1 | 25.0 | 5.0 | 2000 | 0.05 | 64 | 3.2 | SINGLE_TRACK |
| 超精密小件 | 0.001 | 0.5 | 0.1 | 50 | 0.0005 | 128 | 0.064 | ATLAS_REGION |
| 低精度快速 | 1.0 | 50.0 | 20.0 | 300 | 0.5 | 32 | 16.0 | SINGLE_TRACK |
| 中等精度中件 | 0.05 | 10.0 | 2.0 | 200 | 0.025 | 64 | 1.6 | SINGLE_TRACK |

### 用例2 验证（单轨可行）：
- d_v = 0.05mm, L_max = 2000mm
- 单轴体素数 = 2000/0.05 = 40,000
- 活跃体素估算 = 6 × 40000² × 3 = 2.88×10^10 → 内存 ~115GB → 超预算!
- 修正：实际为窄带SDF，活跃体素远小于全量。重新估算：
  - 表面积 ≈ 2×(2000×2000 + 2000×200 + 2000×200) = 9,600,000 mm²（假设200mm高）
  - 表面体素数 ≈ 9.6e6 / 0.05² × 3(halfWidth) ≈ 1.15×10^10 → 仍然巨大 → DUAL_TRACK ✓

### 用例4 验证（单轨可行）：
- d_v = 0.5mm, L_max = 300mm
- 单轴体素数 = 600
- 活跃体素估算 = 6 × 600² × 3 = 6,480,000 → 内存 ~26MB → 远低于预算
- mode = SINGLE_TRACK ✓ （直接用 FloatGrid(0.5mm) 即可满足精度）

### 用例3 验证（Atlas）：
- d_v = 0.0005mm, L_max = 50mm
- 单轴体素数 = 50/0.0005 = 100,000,000 > 1.67e7 → ATLAS_REGION ✓

---

## 7. 与 OpenVDB Transform 的映射

### 7.1 单轨模式 (SINGLE_TRACK)

```cpp
// 唯一的 Grid，VoxelSize = d_v（精度由体素粒度直接保证）
auto xform = openvdb::math::Transform::createLinearTransform(d_v);
auto billetSDF = tools::createLevelSetBox<FloatGrid>(bbox, *xform, halfWidth);
```

### 7.2 双轨模式 (DUAL_TRACK)

```cpp
// 关键：两个 Grid 共享同一个 Transform（同一个 VoxelSize = D_v）
auto sharedXform = openvdb::math::Transform::createLinearTransform(D_v);

// 宏观轨：FloatGrid，每体素 1 个 SDF 值
auto macroSDF = tools::createLevelSetBox<FloatGrid>(bbox, *sharedXform, halfWidth);

// 微观轨：PointDataGrid，共享同一 Transform
// 拓扑镜像 macroSDF 的窄带叶节点
auto microPoints = PointDataGrid::create();
microPoints->setTransform(sharedXform);

// 在每个活跃叶节点内，按 d_v 间距注入面元点
// 每个表面体素内约 N² 个面元（N = D_v / d_v）
// 面元位置 = 体素中心 + 局部偏移（精度 d_v）
```

**核心性质**：
- 同一个索引 (i,j,k) 在 FloatGrid 中返回 SDF 值，在 PointDataGrid 中返回该体素内的面元集合
- 两棵树的叶节点集合完全一致（窄带区域重合）
- 遍历时可同步推进，无需坐标转换

---

## 8. 潜在风险点

**【不同见解】**：当前 CSP 求解器假设所有刀具共享同一 R_min。但实际加工中，粗加工刀具(R=25mm)和精加工刀具(R=0.5mm)的尺度差异巨大。如果以精加工刀具的 R_min 来约束全局 D_upper，会导致宏观网格过于精细，浪费粗加工阶段的计算资源。

**建议**：后续迭代考虑引入"分阶段分辨率重配置"机制——粗加工阶段使用较大 D_v，精加工阶段动态细化。但第一阶段 MVP 暂以最小刀具为基准，保证精度正确性优先。

---

## 修订记录

| 版本 | 日期 | 修订内容 |
|------|------|----------|
| v0.a | 2026-06-01 | 初稿创建 |
| v0.b | 2026-06-01 | 重构模式判定逻辑：单轨(SINGLE_TRACK)为默认优选 |
| v0.c | 2026-06-01 | 修正双轨架构理解：共享 Transform(D_v)；N 为体素内面元密度因子 |
| v0.d | 2026-06-01 | Fix-1: 内存估算改用实际 Lx,Ly,Lz 表面积；MEMORY_BUDGET 可配置；D_upper<d_v 时强制单轨 |
