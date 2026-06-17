# Yggdrasil-Midgard 详细设计文档

## 修订记录

| 版本 | 日期 | 说明 |
|------|------|------|
| 0.a | 2026-06-16 | 初始版本，基于 initialdesign0_0.a.md 审查后细化 |
| 0.b | 2026-06-16 | 引入 ToleranceConfig 单参数派生模型；重投影标记为可选模块 |
| 0.c | 2026-06-16 | Phase 0 从CSG差集改为SDF点查询分类；Phase 3删除判据改为SDF<t；边界钳位降级为后备 |
| 0.d | 2026-06-17 | Phase 0 恢复CSG差集（纯SDF查询无法发现新边界）；CutClassification改为vector<Coord>；加入性能实测数据 |
| 0.e | 2026-06-17 | 新增 IPWBuilder 模块（§3.3, §4.0）；GeometryDef; 毛坯采样精度 t_billet=2t；三态分类改用MicroGrid查询 |

---

## 1. 设计审查结论与修正决策

基于对主设计文档的第一性原理审查，以下为识别的关键风险及本详细设计的应对决策：

### 1.1 Critical 风险修正

| # | 风险 | 修正决策 |
|---|------|----------|
| C1 | halfwidth=1 导致 CSG 拓扑错误 | MacroGrid halfwidth 固定为 3；Phase 0 执行 CSG 差集确保新边界正确激活 |
| C2 | 单步重投影在高曲率区发散 | **[可选模块]** 若启用则改为阻尼 Newton 迭代；见§4.4备忘 |
| C3 | Phase 1 O(N·M) 复杂度爆炸 | 反转查询方向：per-param_block 查 voxel（利用 OpenVDB 空间索引），O(M·log N) |
| C4 | 牛鼻刀 SDF 梯度奇异 | 分段构造（圆柱+环面+底盘取 min），环面段数值 Newton + 脊线 ε-正则化 |

### 1.2 High 风险修正

| # | 风险 | 修正决策 |
|---|------|----------|
| H1 | "无锁注入"不可实现 | 改为 thread-local buffer + batch merge（map-reduce 模式） |
| H2 | 跨 voxel 水密性 | 引入边界钳位协议：voxel_c 在与 voxel_d 相邻面生成钳位点 |
| H3 | ε→V 映射缺失 | 引入 ToleranceConfig 单参数派生模型（见§1.3） |
| H4 | 30FPS 预算紧张 | 分帧摊还 + 交互/最终双模式 |

### 1.3 ToleranceConfig — 单参数派生模型

**设计哲学**：用户只输入一个全局目标加工误差 `t`（即 ε）。系统内部自动将 `t` 解耦派生为驱动各模块的内部阈值，实现算力、内存与精度的动态平衡。

**核心洞察**：DualTrack 架构的本质优势是 MacroGrid 只负责拓扑分类（哪些voxel受影响），精度完全由 MicroGrid 的相对坐标点保证。因此 `V_macro` 应尽可能大——每个 voxel 越大，Phase 2 的切削计算单元越少，系统越快。`V_macro` 的上限约束不是精度（精度由 MicroGrid 保证），而是**单 voxel 内切削界面计算的效率**：voxel 越大，内部四叉树细分层数越多，单任务计算时间越长。

**派生公式**：

| 内部参数 | 符号 | 派生规则 | 约束来源 |
|----------|------|----------|----------|
| 宏观体素尺寸 | `V_macro` | `clamp(K·t, MIN_VOXEL, MAX_VOXEL)` | 单 voxel 计算效率 |
| 重投影带宽 | `ε_band` | `t` | 过渡区判定（如启用） |
| 流形基础采样步长 | `Δ_base` | `10·t` | 平坦区节省内存 |
| 弦高细分阈值 | `ε_chordal` | `t` | 几何保真度底线 |

**V_macro 的 K 因子设计**：

K 取值范围 **20~50**（默认 K=30），含义是每个 voxel 边长为目标精度的 30 倍。

| 场景 | t | V_macro (K=30) | 意义 |
|------|---|----------------|------|
| 粗加工 | 0.1mm | 3.0mm | 极稀疏，性能最优 |
| 半精加工 | 0.05mm | 1.5mm | 中等 |
| 精加工 | 0.01mm | 0.3mm | 合理密度 |
| 高精 | 0.001mm | 0.03mm | 仍可接受 |

**K 的优化依据**：单 voxel 内四叉树细分深度 ≈ log₂(V_macro / t) = log₂(K)。
- K=30 → 约 5 层细分，单 voxel 计算量 ≈ 4⁵ = 1024 次 S(u,t) 求值（最坏）
- K=50 → 约 6 层，4⁶ = 4096 次（接近上限）
- K=20 → 约 4.3 层，典型 ~500 次（快速但 voxel 数多）

**最优 K 取决于**：`单voxel计算时间 × 受影响voxel总数` 的全局最小值。这是一个运行时可 profile 的超参数，初始取 K=30，后期基于 benchmark 调优。

**边界约束**：
- `MIN_VOXEL_SIZE = 0.02mm`（防止极小 t 导致 tree 过深）
- `MAX_VOXEL_SIZE = 5.0mm`（防止单 voxel 内计算量爆炸）

**误差收敛保证**：
V_macro 不参与精度保证链。精度完全由以下保证：
```
ε_total ≤ ε_chordal = t
```
四叉树细分在每个 voxel 内独立收敛至弦高 ≤ t，与 V_macro 无关。V_macro 只影响性能（voxel 数量 vs 单 voxel 计算量的 trade-off）。

---

## 2. 系统架构总览

### 2.1 数据模型

```
┌─────────────────────────────────────────────┐
│                 IPW (工件状态)                │
├─────────────────────┬───────────────────────┤
│    MacroGrid        │      MicroGrid        │
│  (FloatGrid)        │  (PointDataGrid)      │
│  - halfwidth=3      │  - 位置: FixedPointCodec<false> (相对坐标16-bit) │
│  - voxel size = V   │  - 法线: QuantizedUnitVec (16-bit编码)          │
│  - 拓扑+SDF值       │  - 存活掩码 bit       │
└─────────────────────┴───────────────────────┘
```

**MicroGrid 属性编码**：
- **位置**：`FixedPointCodec<false>`（16-bit定点，[-0.5, 0.5]范围，精度 1/65536 ≈ 1.5e-5 voxel）
- **法线**：`QuantizedUnitVec`（OpenVDB内置，16-bit → 单位向量，角度误差 < 0.5°，内存仅 2 bytes/法线 vs 12 bytes/Vec3f）

### 2.2 管线数据流

```
输入: IPW_{n-1} + ToolPath segment + ε
        │
        ▼
┌─ Phase 0 (CPU) ─────────────────────────────┐
│ SDF点查询分类 → voxel三态标记(d/c/n)        │
│ 输出: dirty_mask, voxel_classification      │
└──────────────────────────────────────────────┘
        │
        ▼
┌─ Phase 1 (CPU) ─────────────────────────────┐
│ 参数面分块 → 3D AABB → 反向voxel查询         │
│ 输出: vector<VoxelTask> task_list           │
└──────────────────────────────────────────────┘
        │
        ▼
┌─ Phase 2 (TBB) ─────────────────────────────┐
│ per-task 四叉树细分 → 新点采样               │
│ 输出: per-leaf thread-local point buffers   │
└──────────────────────────────────────────────┘
        │
        ▼
┌─ Phase 3 (TBB) ─────────────────────────────┐
│ 旧点 SDF<t 判断 → 硬删除                    │
│ (过渡区旧点由新采样覆盖，安全删除)           │
│ 输出: updated survival masks                │
└──────────────────────────────────────────────┘
        │
        ▼
┌─ Phase 4 (TBB) ─────────────────────────────┐
│ batch merge buffers → MicroGrid             │
│ 输出: IPW_n (MacroGrid + MicroGrid 一致)    │
└──────────────────────────────────────────────┘
        │
        ▼
┌─ Compaction (条件触发) ─────────────────────┐
│ fragmentation > 30% 时执行内存规整           │
└──────────────────────────────────────────────┘
```

---

## 3. 模块接口定义

### 3.1 Types.h — 公共类型

```cpp
namespace midgard {

using Vec3d = openvdb::Vec3d;
using Vec3f = openvdb::Vec3f;

// === 刀具定义 ===
enum class ToolType : uint8_t { BALL_END, FLAT_END, BULL_NOSE };

struct ToolDef {
    ToolType type;
    double R;    // 刀具半径 (mm)
    double r;    // 圆角半径 (mm), 仅 BULL_NOSE 有效
    double H;    // 刀具高度 (mm)
};

// === 刀路段 ===
struct MoveSegment {
    Vec3d start;      // 起点 (mm)
    Vec3d end;        // 终点 (mm)
    Vec3d axis;       // 刀轴方向 (三轴固定Z)
    double feedRate;  // 进给速度 (mm/min), 仅用于时间估算
};

// === 仿真配置：单参数派生 ===
struct ToleranceConfig {
    double user_t;          // 用户唯一输入：目标加工公差 (mm)

    // 内部派生参数 (构造时自动计算)
    double voxelMacro;      // 宏观体素尺寸 = K * t
    double reprojBand;      // 重投影带宽 (如启用)
    double baseStep;        // 流形采样基础步长
    double chordalLimit;    // 弦高细分阈值

    int    halfwidth = 3;   // narrowband 半宽 (固定)
    double K = 30.0;        // V_macro / t 比例因子 (可调优, 20~50)
    enum Mode { INTERACTIVE, FINAL } mode = INTERACTIVE;

    static constexpr double MIN_VOXEL_SIZE = 0.02;  // mm
    static constexpr double MAX_VOXEL_SIZE = 5.0;   // mm

    explicit ToleranceConfig(double t, Mode m = INTERACTIVE, double k = 30.0)
        : user_t(t), K(k), mode(m)
    {
        voxelMacro   = std::clamp(K * t, MIN_VOXEL_SIZE, MAX_VOXEL_SIZE);
        reprojBand   = t;
        baseStep     = 10.0 * t;
        chordalLimit = t;
    }
};

// === Voxel 分类 ===
enum class VoxelClass : uint8_t {
    UNCHANGED = 0,  // 不受影响
    DELETED,        // voxel_d: 完全切除
    CUT,            // voxel_c: 部分切除
    NEW_BOUNDARY    // voxel_n: 新生边界
};

// === 任务负载 ===
struct VoxelTask {
    openvdb::Coord origin;     // leaf node 原点坐标
    openvdb::BBoxd aabb;       // voxel 物理包围盒
    double u_min, u_max;       // 保守参数域 U
    double t_min, t_max;       // 保守参数域 T
    VoxelClass classification;
};

// === 毛坯几何定义 (定义在 IPWBuilder.h) ===
struct GeometryDef {
    enum Type { BOX, CYLINDER, SPHERE, MESH };
    Type type = BOX;
    Vec3d origin{0, 0, 0};
    Vec3d dims{0, 0, 0};       // BOX: length/width/height
    double radius = 0;          // CYLINDER/SPHERE
    double height = 0;          // CYLINDER
    // MESH: vertices + triangles (外部传入)
};

// === IPW 状态 ===
struct IPWState {
    openvdb::FloatGrid::Ptr macroGrid;
    openvdb::points::PointDataGrid::Ptr microGrid;
    ToleranceConfig config;

    explicit IPWState(double tolerance) : config(tolerance) {}
};

} // namespace midgard
```

### 3.2 ToolSweepSDF — 刀具扫掠体解析 SDF

```cpp
namespace midgard {

class ToolSweepSDF {
public:
    ToolSweepSDF(const ToolDef& tool, const MoveSegment& seg);

    /// 有符号距离 (< 0 = 内部)
    double eval(const Vec3d& p) const;

    /// 解析梯度 (牛鼻刀环面段用数值Newton)
    Vec3d gradient(const Vec3d& p) const;

    /// 扫掠体AABB
    openvdb::BBoxd boundingBox() const;

    /// 参数面求值 S(u,t) → R³
    /// u ∈ [0,1]: 截面参数, t ∈ [0,1]: 沿刀路参数
    Vec3d evalSurface(double u, double t) const;

    /// 参数子域的3D AABB (用于Phase 1分块匹配)
    openvdb::BBoxd surfaceBBox(double u0, double u1,
                               double t0, double t1) const;

private:
    // 分段SDF: 取三段min
    double evalCylinder(const Vec3d& p) const;
    double evalTorus(const Vec3d& p) const;    // 仅BULL_NOSE
    double evalDisc(const Vec3d& p) const;

    ToolDef mTool;
    MoveSegment mSeg;
    Vec3d mDir;       // 归一化刀路方向
    double mLength;   // 刀路段长度
};

} // namespace midgard
```

### 3.3 IPWBuilder — 毛坯初始化

```cpp
namespace midgard {

class IPWBuilder {
public:
    /// 构建完整 IPW0 (MacroGrid + MicroGrid)
    /// 毛坯采样精度 t_billet = 2 * config.user_t
    IPWState build(const GeometryDef& geom, const ToleranceConfig& config);

private:
    openvdb::FloatGrid::Ptr buildMacroGrid(
        const GeometryDef& geom, const ToleranceConfig& config);

    /// 在 MacroGrid 零等值面附近生成 MicroGrid 点集
    /// 采样密度由局部曲率和 t_billet 控制
    openvdb::points::PointDataGrid::Ptr buildMicroGrid(
        const openvdb::FloatGrid::Ptr& macroGrid,
        const GeometryDef& geom,
        double t_billet);
};

} // namespace midgard
```

### 3.4 MacroCut — Phase 0 & 1

```cpp
namespace midgard {

struct CutClassification {
    std::vector<openvdb::Coord> deleted;      // voxel_d: 完全在扫掠体内，需删除
    std::vector<openvdb::Coord> cut;          // voxel_c: 边界voxel，MicroGrid有既有点集
    std::vector<openvdb::Coord> newBoundary;  // voxel_n: 边界voxel，MicroGrid无数据
};

class MacroCut {
public:
    /// Phase 0: CSG差集 + SDF分类
    /// 1. 光栅化刀具SDF为同分辨率FloatGrid
    /// 2. csgDifference更新MacroGrid拓扑（激活新边界）
    /// 3. 对CSG后的active voxels做SDF阈值分类
    CutClassification classifyVoxels(
        IPWState& ipw,
        const ToolSweepSDF& tool);

    /// Phase 1: 生成任务列表
    /// 反向查询: per-param_block → 匹配 voxels
    std::vector<VoxelTask> buildTaskList(
        const CutClassification& classification,
        const ToolSweepSDF& tool,
        const ToleranceConfig& config);

private:
    // 参数面均匀分块数 (自适应)
    int computeBlockCount(const ToolSweepSDF& tool,
                          double voxelSize) const;
};

} // namespace midgard
```

### 3.5 MicroCut — Phase 2 & 3

```cpp
namespace midgard {

/// Phase 2 输出: 每个leaf的新点缓冲
struct PointBuffer {
    std::vector<Vec3f> positions;  // 相对坐标 (计算时double，存储时编码为FixedPoint16)
    std::vector<Vec3f> normals;    // 单位法线 (存储时编码为QuantizedUnitVec 16-bit)
};

class MicroCut {
public:
    /// Phase 2: 四叉树自适应采样, 生成新切削面点
    /// 返回 per-leaf 的 thread-local buffers
    std::unordered_map<openvdb::Coord, PointBuffer>
    sampleNewSurface(
        const std::vector<VoxelTask>& tasks,
        const ToolSweepSDF& tool,
        const ToleranceConfig& config);

    /// Phase 3: 旧点剔除 (SDF < t → 删除)
    /// 过渡区 [0, t] 的旧点由新采样覆盖，安全删除
    void cullOldPoints(
        IPWState& ipw,
        const CutClassification& classification,
        const ToolSweepSDF& tool,
        const ToleranceConfig& config);

private:
    /// 四叉树递归求值核心
    void quadtreeEval(
        double u0, double u1, double t0, double t1,
        const openvdb::BBoxd& voxelAABB,
        const ToolSweepSDF& tool,
        double epsilon,
        int depth,
        PointBuffer& output);

    /// 阻尼Newton重投影 (2-3步)
    Vec3d reproject(const Vec3d& p,
                    const ToolSweepSDF& tool,
                    double epsilon) const;

    static constexpr int MAX_QUADTREE_DEPTH = 12;
};

} // namespace midgard
```

### 3.6 Compaction — Phase 4 & 内存规整

```cpp
namespace midgard {

class Compaction {
public:
    /// Phase 4: 批量合并点缓冲到 MicroGrid
    /// (map-reduce: thread-local buffers → PointDataGrid)
    void mergeBuffers(
        IPWState& ipw,
        const std::unordered_map<openvdb::Coord, PointBuffer>& buffers);

    /// 条件触发内存规整
    /// 返回 true 如果执行了规整
    bool compactIfNeeded(IPWState& ipw);

    /// 碎片率查询
    double fragmentationRatio(const IPWState& ipw) const;

    static constexpr double FRAGMENTATION_THRESHOLD = 0.3;
};

} // namespace midgard
```

---

## 4. 关键算法细节

### 4.0 IPWBuilder — 毛坯初始化算法

**精度参数**：`t_billet = 2 * config.user_t`（毛坯采样精度，切削精度的2倍）

**Step 1: MacroGrid 创建**

| 几何类型 | 方法 | 备注 |
|----------|------|------|
| Box | `createLevelSetBox` | OpenVDB 原生 |
| Cylinder | 自定义光栅化 SDF | `sdf = max(|xy|-R, |z|-H/2)` |
| Sphere | `createLevelSetSphere` | OpenVDB 原生 |
| Mesh | `meshToLevelSet` | 输入三角网格 |

voxelSize = `config.voxelMacro`, halfwidth = 3。

**Step 2: MicroGrid 采样**

对 MacroGrid 中每个表面 voxel（`|SDF(center)| < V`）：

```
1. 投影到零等值面: p = center - SDF(center) * ∇SDF(center)
2. 法线 = normalize(∇SDF(center))
3. 判断需要的采样密度:
   ρ = 局部曲率半径 (Box→∞, Cylinder侧面→R, Sphere→R, Mesh→估算)
   step = 2 * sqrt(2 * ρ * t_billet)
   n_per_edge = ceil(V / step)
   n_per_edge = max(1, n_per_edge)  // 至少1个点

4. 在 voxel 面片范围内生成 n_per_edge² 个点:
   - 平面 (ρ=∞): 1个点 (投影点即可)
   - 曲面: 在 voxel 内沿切面方向均匀撒点，各自投影到SDF=0
```

**法线来源**：

| 类型 | 法线计算 |
|------|----------|
| Box | 解析：最近面外法线 `±(1,0,0)/(0,1,0)/(0,0,1)` |
| Cylinder | 解析：侧面 `(x,y,0)/r`，端面 `(0,0,±1)` |
| Sphere | 解析：`p/|p|` |
| Mesh | 有限差分梯度 `∇SDF`（6点中心差分） |

**典型开销**（20mm Box, V=1mm）：
- 表面 voxel 数 ≈ 6×20² / 1² = 2400
- 平面 → 1点/voxel → 2400 点总量
- 预计耗时 < 1ms

**设计 Trade-off：采样来源选择（MacroGrid SDF vs 原始几何）**

当前实现统一从 MacroGrid SDF 有限差分梯度投影到零等值面。未走解析几何路径。

| 区域 | SDF方案精度 | 影响 |
|------|-------------|------|
| Box 平面区 | 精确 | 无 |
| Box 棱角/顶角 | ~V/2 偏差（level set 圆角化） | 首次切削后被替代，无实际影响 |
| Cylinder/Sphere | SDF 本身精确 | 无 |
| Mesh | 取决于 meshToLevelSet 精度 | 统一管道，无需 BVH |

**保留理由**：
1. 统一管道——与后续切削产生的 MicroGrid 点（来自参数面采样）流程一致
2. Mesh 输入天然只能走 SDF 路径（解析投影需要 BVH，复杂度不对等）
3. 棱角误差无实际影响（切削后被新点替代）

**后续优化路径**（如有硬需求）：对解析几何（Box/Cylinder/Sphere）增加可选的解析投影快速路径。

### 4.1 Phase 0：CSG 差集 + SDF 分类

**设计决策**：MacroGrid 必须执行 CSG 差集才能正确激活切削产生的新边界 voxel。

**算法**（四步）：
```
Input: IPWState (macroGrid + microGrid), ToolSweepSDF
Output: CutClassification { deleted, cut, newBoundary }

Step 1 - 光栅化范围裁剪:
    cutBBox = tool.boundingBox() ∩ macroGrid.activeVoxelBoundingBox()
    // 只在刀具与工件拓扑实际重叠的区域光栅化
    // 自然排除刀杆（在工件上方）和非接触区域

Step 2 - 光栅化刀具SDF (裁剪后范围):
    toolGrid = 在 cutBBox±halfwidth 范围内逐voxel求tool.eval()
    仅存储 |sdf| < background 的narrowband

Step 3 - CSG差集:
    csgDifference(macroGrid, toolGrid)  // in-place
    → 正确更新narrowband：原内部区域被切削后激活为新边界

Step 4 - SDF阈值分类 (仅tool bbox内leaf nodes):
    threshold = V * sqrt(3) / 2
    for each leaf in macroGrid where leaf.bbox overlaps toolIdxBox:
        for each active_voxel in leaf:
            sdf = tool.eval(voxel.worldCenter())
            if sdf < -threshold: → deleted
            elif sdf <= threshold:
                query microGrid → has data? → cut : newBoundary
```

**光栅化范围裁剪的收益**：
- 刀具 AABB ∩ 工件 AABB 自然排除刀杆（刀杆不接触工件则不光栅化）
- 球头刀 R=5, H=30, 切深2mm → 光栅化高度从30mm缩减为~7mm（4× 体积缩减）
- 平底刀深切型腔时收益较小（切削部分本身大）

**碰撞检测**（独立可选步骤）：
```
shankBBox = tool.fullBoundingBox() - cutBBox  // 非切削部分
if macroGrid.activeTopology overlaps shankBBox:
    → 碰撞警告 (不做CSG, 报告给上层)
```

**性能特征**（实测 100mm工件 + R=5球头刀 + V=0.5mm）：
- 优化前: 14ms（全量光栅化+全量遍历）
- 优化后: 3.9ms（bbox裁剪光栅化 + leaf过滤分类）

**cut/newBoundary 向量的意义**：
- cut: 原有表面 voxel（MicroGrid 有既有点集），后续需剔除旧点+补新点
- newBoundary: 新暴露的内部 voxel（MicroGrid 无数据），后续需从零生成切削面点
- 总边界数 ∝ 刀路长度 × 切削截面周长 / V²

### 4.2 牛鼻刀 SDF 分段构造

牛鼻刀截面由三部分组成：圆柱壁面 + 底部环面过渡 + 底面圆盘。

```
     |←─ R ─→|
     ┌────────┐  ← 顶部 (高度H)
     │  cyl   │  ← 圆柱段: SDF_cyl
     │        │
     └──╮  ╭──┘  ← 环面过渡段: SDF_torus (圆角半径r)
        ╰──╯     ← 底盘: SDF_disc (半径 R-r)
```

扫掠体 SDF：
```
SDF_sweep(p) = min(SDF_cyl_sweep(p), SDF_torus_sweep(p), SDF_disc_sweep(p))
```

环面段 SDF 需要求解：
```
min_{λ∈[0,1]} dist(p, torus_center + λ·d)
```
展开为关于 λ 的四次方程。采用：
1. 先用 clamp(dot(p-A, d)/|d|², 0, 1) 得到初始 λ₀
2. 2-3 步 Newton 迭代细化
3. 脊线区域（|p_radial - (R-r)| < r/10）加 smoothmin 正则化

### 4.3 Phase 1 反向查询算法

```
Input: param_blocks[M], MacroGrid topology
Output: task_list

for each block_i in param_blocks:
    aabb_i = tool.surfaceBBox(block_i.u_range, block_i.t_range)
    aabb_i.expand(SAFE_MARGIN)  // 裕量外扩

    // 利用 OpenVDB CoordBBox 快速遍历
    coord_bbox = world_to_index(aabb_i)
    for each active_voxel in MacroGrid.intersect(coord_bbox):
        task_list[voxel].expand_param_range(block_i)

// 复杂度: O(M · K), K = 每个block覆盖的平均voxel数
// 典型: M=50, K=100 → 5000次操作 (vs 原方案 10⁸)
```

### 4.4 四叉树终止判据 (Phase 2)

双重条件，满足任一则终止：

1. **弦高精度**: 子域内弦高误差 `≤ config.chordalLimit`（即 t）
2. **最大深度**: `depth ≥ MAX_QUADTREE_DEPTH (12)`

初始采样步长 = `config.baseStep`（= 10·t），平坦区域大步长直接输出；
仅当局部弦高偏差 > t 或检测到 SDF 符号翻转（相交线）时触发细分。

终止后：在子域中心采样 `S(u_mid, t_mid)` 并计算法线，加入 PointBuffer。

额外优化 — 提前终止（3D AABB 拒止）：
```
if (!voxelAABB.hasOverlap(surfaceBBox(u_sub, t_sub))):
    return  // 子域完全在voxel外，跳过
```

### 4.5 阻尼 Newton 重投影 (Phase 3) — ⚠️ 可选模块

> **设计备忘 (2026-06-16)**：重投影可能非必要。理由：Phase 2 的相交线计算
> 已经在参数域精确定位了切削边界，生成的新点本身就在解析曲面上（精度由
> 四叉树终止条件保证）。重投影的原始目的是将"距边界 ε 以内的旧点"拉回
> 解析面，但如果 Phase 2 的边界采样已经完备覆盖，旧点只需做 inside/outside
> 硬判断即可——处于过渡带的旧点直接删除，由新采样点替代。
>
> **当前决策**：保留重投影算法设计作为后备方案，但实现优先级降至 M6 之后。
> 初始实现中 Phase 3 仅做硬删除（SDF < 0 → 删除），不做重投影。
> 如果集成测试发现边界锯齿问题，再启用重投影路径。

如果启用，算法如下：

```
Input: p_old (旧点坐标), tool (SDF), ε
Output: p_new (投影到切削面的点)

p = p_old
for i in 1..3:
    sdf = tool.eval(p)
    if |sdf| < ε/10: break  // 已收敛

    grad = tool.gradient(p)
    ρ = 估算局部曲率半径 (= 1/|∇²SDF| 近似为 tool.r 或 tool.R)
    α = min(1.0, ρ / (|sdf| + ρ))  // 阻尼因子
    p = p - α · sdf · grad

return p
```

收敛保证：当 |SDF|/ρ < 1 时，阻尼因子确保每步至少缩减50%的误差。

### 4.5b MicroGrid 点数据更新策略 (Phase 3+4)

**PointDataGrid 存储约束**：
- 同一 leaf node 内所有 voxel 的点共享一个连续 AttributeArray
- voxel values 存储累积偏移量（非 SDF 值），voxel i 的点范围 = `[offset[i-1], offset[i])`
- per-voxel 增删不能原地操作——必须重建整个 leaf 的数组和偏移表

**选定方案：per-leaf 整体重建**

对每个受影响的 leaf node（包含 deleted/cut/newBoundary voxels），执行一次性重建：

```
for each affected_leaf (按 leaf origin 分组 CutClassification 的 coords):

    1. 收集存活旧点:
       for each voxel in leaf:
         if voxel in deleted: 丢弃该voxel所有点
         elif voxel in cut: 逐点检查 tool.eval(p)
           SDF >= t → 保留 (不在刀具内)
           SDF < t  → 丢弃 (被切削)
         else: 保留该voxel所有点 (不受影响)

    2. 收集新点:
       从 Phase 2 的 PointBuffer 中获取属于该 leaf 的新采样点+法线

    3. 重建 leaf:
       new_points = surviving_old + new_samples
       重新计算 offset array (per-voxel 累积)
       创建新 AttributeArray (P + N)
       替换该 leaf 的 attributeSet

    4. updateValueMask():
       有点的 voxel → active, 无点 → inactive
```

**选择此方案的理由**：
1. 受影响 leaf 数有限（tool bbox 内 ~几十个），重建开销可控
2. 一次操作完成，无"标记-compact"两阶段状态管理
3. 新分配 leaf 数据天然兼容 COW（不影响渲染线程持有的旧引用）
4. Cache 友好——连续写入新数组

**并行粒度**：以 leaf 为 TBB 任务单位，各 leaf 独立重建，无锁。

### 4.6 边界钳位协议 (跨 voxel 水密性) — ⚠️ 后备方案

> **设计备忘 (2026-06-16)**：初始实现不启用边界钳位。理由：Phase 1 的裕量
> 外扩（SAFE_MARGIN）已保证新点采样会略微越过 voxel 边界，这些越界点在
> Phase 4 merge 时按归属 voxel 分配，自然填充边界区域。加之渲染采用点云
> splatting 时，微小 gap 不可见。仅当隐式曲面重建出现可见缝隙时再启用。

如果启用：

当 voxel_c 与 voxel_d 共享一个面时：
1. 在共享面上额外采样一排边界点（坐标钳位为 ±0.5 在对应轴）
2. 这些钳位点的法线取切削面在该位置的法线
3. 相邻两个 voxel_c 在共享面上的钳位点自动匹配（同一坐标）

这确保了渲染重建时在 voxel 边界处无 gap。

---

## 5. 实现优先级与里程碑

### 5.1 Phase 依赖图

```
ToolSweepSDF ──┬──→ MacroCut (Phase 0-1) ──→ MicroCut (Phase 2-3) ──→ Compaction (Phase 4)
               │
Types.h ───────┘
```

### 5.2 迭代计划

| 里程碑 | 内容 | 验收标准 |
|--------|------|----------|
| M0 | 工程骨架 + Types + 编译通过 | CMake build 成功，空 test 运行 |
| M1 | ToolSweepSDF (球头+平底) | TDD 通过，解析梯度误差 < 1e-10 |
| M2 | ToolSweepSDF (牛鼻刀) | 环面段 Newton 收敛，脊线处 smooth |
| M3 | MacroCut Phase 0 | SDF 点查询分类正确，voxel 三态标记准确 |
| M4 | MacroCut Phase 1 | 反向查询正确，任务覆盖保守完备 |
| M5 | MicroCut Phase 2 | 四叉树采样精度 ≤ ε/2 |
| M6 | MicroCut Phase 3 | 旧点 SDF<t 硬删除正确，无过渡区重叠 |
| M7 | Compaction Phase 4 | batch merge 正确，碎片规整有效 |
| M8 | 端到端集成 | Hausdorff ≤ ε，帧时间可测 |

---

## 6. 验收规格 (Acceptance Criteria)

### 6.1 精度验收

| 测试场景 | 输入 | 验收条件 |
|----------|------|----------|
| AC-P1: 球头刀切平面 | R=5mm球头刀，水平切入10mm方块毛坯，深度2mm，ε=0.01mm | Hausdorff(结果, 解析真值) ≤ 0.01mm |
| AC-P2: 平底刀切型腔 | R=3mm平底刀，Z向下切5mm，ε=0.05mm | 底面平面度偏差 ≤ 0.05mm，侧壁垂直度 ≤ 0.05mm |
| AC-P3: 牛鼻刀圆角 | R=5mm, r=1mm 牛鼻刀，沿X切削，ε=0.01mm | 圆角过渡区曲率连续，Hausdorff ≤ 0.01mm |
| AC-P4: 极端精度 | 任意刀具，ε=0.001mm | 结果点集存在且精度达标（允许性能降级） |
| AC-P5: 零切削 | 刀具路径完全在毛坯外 | IPW 不变（bit-identical） |
| AC-P6: 完全切除 | 刀具路径完全穿透毛坯 | MacroGrid 对应区域全部 inactive |

### 6.2 鲁棒性验收

| 测试场景 | 输入 | 验收条件 |
|----------|------|----------|
| AC-R1: 零长度路径 | start == end | 不崩溃，等效为静态刀具布尔减 |
| AC-R2: 擦边切削 | 刀具边缘刚好接触工件表面 | 不产生孤立点或拓扑碎片 |
| AC-R3: 高曲率重投影 | 牛鼻刀 r=0.5mm，voxel=0.5mm | 重投影 3 步内收敛，无发散 |
| AC-R4: 大工件坐标 | 工件原点偏移 (500, 500, 500)mm | 相对坐标精度不退化 |

### 6.3 性能验收

| 测试场景 | 条件 | 验收条件 |
|----------|------|----------|
| AC-F1: 交互模式帧时间 | 100mm³毛坯，ε=0.05mm，单段切削 | Phase 0-4 总时间 ≤ 50ms (Mac mini M4) |
| AC-F2: 最终模式无帧率要求 | 同上，ε=0.001mm | 正确完成，不限时间 |
| AC-F3: 内存上限 | 200mm³毛坯，ε=0.01mm | 峰值内存 ≤ 4GB |
| AC-F4: 碎片规整 | 连续1000次切削后 | 碎片率 < 30% |

### 6.4 模块级 TDD 验收

| 模块 | 必须通过的 TDD 测试 |
|------|---------------------|
| ToolSweepSDF | 球体/平面/环面已知解析解对比，梯度有限差分验证（误差<1e-6） |
| MacroCut | SDF分类正确（球切方块→DELETED/CUT/UNCHANGED分布符合几何预期） |
| MicroCut.quadtreeEval | 平面→单次终止，球面→多层细分，采样密度与ε关系 |
| MicroCut.reproject | 收敛步数统计，发散检测，边界case |
| Compaction | merge前后点数一致，碎片率计算正确 |

---

## 7. 运行时调试策略 (RtDebugSys 集成)

各 Phase 实现时同步埋入 RtDebugSys 标签。标签随代码编写，不事后补。

### 7.1 标签规划

| 标签 | 输出 | 级别 |
|------|------|------|
| `Phase0_Classify` | d/c/n 计数、扫掠体 AABB | Normal |
| `Phase1_TaskList` | 任务数、耗时 | Normal |
| `Phase2_Quadtree` | 每个 task 的最大深度、采样点数 | Normal |
| `Phase2_Detail` | 单次细分的 u/t 范围、AABB、弦高 | Verbose |
| `Phase3_Cull` | 删除/保留点数 | Normal |
| `Phase4_Merge` | 新增点数、耗时 | Normal |
| `Compaction` | 碎片率、是否触发 | Normal |

### 7.2 使用原则

- **Normal 标签**：每个 Phase 一条摘要日志（聚合统计），开发期常开
- **Verbose 标签**：per-voxel / per-subdivision 细节，仅定点排查时开启
- 精度验证由测试用例（Hausdorff 断言）负责，不在运行时调试中做
- 标签内禁止副作用和重量级计算

---

## 8. 开放问题 (待后续迭代解决)

| # | 问题 | 当前决策 | 后续方向 |
|---|------|----------|----------|
| O1 | 多段折线刀路是否合并为单次CSG | 暂不合并，逐段执行 | M8后评估batch union性能 |
| O2 | 渲染管线同步机制 | 单帧延迟 + COW SharedPtr | 集成渲染时细化 |
| O3 | GPU (CUDA) 迁移 | TBB first，API预留GPU接口 | 性能瓶颈确认后迁移 |
| O4 | 五轴扩展 | 接口预留刀轴方向参数 | 三轴稳定后设计 |
| O5 | 错误恢复/Checkpoint | 每100步自动保存.vdb快照 | 实现时细化 |
