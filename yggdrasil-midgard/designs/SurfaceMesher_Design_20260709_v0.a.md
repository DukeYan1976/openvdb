# SurfaceMesher 设计文档

**文件**: `SurfaceMesher_Design_20260709_v0.a.md`  
**模块**: `core/SurfaceMesher.h/cpp`  
**目的**: 从切削界面点法式数据重建三角形网格，用于视觉渲染

---

## 1. 需求概述

| 项目 | 说明 |
|------|------|
| 输入 | `std::vector<SurfaceSample>` — position + normal |
| 输出 | `TriMesh` — 顶点(pos+normal) + 三角形索引 |
| 精度 | 视觉级（~0.01mm），高精度数据保留在点法集中 |
| 性能 | 10k点 < 15ms，50k点 < 80ms |
| 独立性 | 零耦合，不依赖 OpenVDB / TBB，纯 STL + 数学 |
| 鲁棒性 | 处理非均匀密度、多连通面、退化区域 |

---

## 2. 算法选型

**选定方案**: 轻量体素化 + Marching Cubes (MC)

**理由**:
1. 点法式 → MLS 隐式场天然处理非均匀分布和拓扑缝隙
2. MC 对多连通/复杂拓扑天然鲁棒
3. 输出网格质量均匀，适合 GPU 渲染
4. 实现紧凑（~500行），无外部依赖
5. 视觉精度足够，无需迭代优化

---

## 3. 算法流程

```
┌─────────────────────────────────────────────────────┐
│ Step 0: AABB + 参数计算                              │
│   - 计算点集包围盒，膨胀 2 个 voxel                   │
│   - voxelSize = max_extent / resolution              │
└──────────────────────┬──────────────────────────────┘
                       │
┌──────────────────────▼──────────────────────────────┐
│ Step 1: KD-tree 构建                                 │
│   - 3D KD-tree (median split, depth ~log₂N)         │
│   - 用于 K 近邻查询                                  │
└──────────────────────┬──────────────────────────────┘
                       │
┌──────────────────────▼──────────────────────────────┐
│ Step 2: 隐式场评估（稀疏体素）                        │
│   - 只评估距点集 < searchRadius 的体素角点            │
│   - MLS 带符号距离:                                  │
│       φ(x) = Σ wᵢ·(x-pᵢ)·nᵢ / Σ wᵢ              │
│       wᵢ = Wendland(‖x-pᵢ‖/h)                     │
│   - 角点无近邻 → 标记为 +∞ (外部)                    │
└──────────────────────┬──────────────────────────────┘
                       │
┌──────────────────────▼──────────────────────────────┐
│ Step 3: Marching Cubes                               │
│   - 标准 256 项查找表                                │
│   - 对每个体素8角点检查符号变化                       │
│   - 边上线性插值得顶点位置                            │
│   - 顶点法线 = 隐式场梯度（由 MLS 解析）             │
└──────────────────────┬──────────────────────────────┘
                       │
┌──────────────────────▼──────────────────────────────┐
│ Step 4: 法线平滑                                     │
│   - 每顶点 = 相邻三角形面法线面积加权平均             │
│   - 归一化                                           │
└──────────────────────┬──────────────────────────────┘
                       │
┌──────────────────────▼──────────────────────────────┐
│ 输出: TriMesh {vertices[], indices[]}                │
└─────────────────────────────────────────────────────┘
```

---

## 4. 关键设计决策

### 4.1 体素分辨率

```
resolution = 128 (默认)
voxelSize = max(AABB.extent) / resolution
```

128³ 的理论最大体素数 = 2M，但稀疏评估只处理距点集 2*voxelSize 内的体素，实际 ~5% 被激活。

### 4.2 MLS 权重函数

使用 Wendland C2 紧支撑核:

```
W(r) = (1 - r)⁴ · (4r + 1),  r = ‖x-p‖/h, r ∈ [0,1]
W(r) = 0,                      r > 1
```

支撑半径 `h = 3.0 * 局部平均点间距`（由 KD-tree K=8 近邻估计）。

### 4.3 稀疏评估策略

不构建完整 resolution³ 数组。使用 hash map 存储被激活的体素角点:
- 遍历所有点，标记其所在体素及 26-邻域为活跃
- 只评估活跃体素的角点
- 内存占用 = O(活跃体素数) ≈ O(N · 常数)

### 4.4 KD-tree 实现

自包含、无外部依赖:
- 构建: median-of-3 pivot
- 查询: K近邻 (K=8) + 半径限制 (maxR=h)
- 期望性能: 构建 O(N log N), 查询 O(log N + K)

---

## 5. 接口定义

```cpp
namespace midgard {

/// 三角形网格（渲染用）
struct TriMesh {
    std::vector<float> vertices;     // [x,y,z, nx,ny,nz] × nVerts
    std::vector<uint32_t> indices;   // 三角形索引 × nTris*3
    int vertexCount() const { return (int)vertices.size() / 6; }
    int triangleCount() const { return (int)indices.size() / 3; }
    bool empty() const { return indices.empty(); }
};

/// 从点法式集合重建视觉三角形网格
/// @param points     位置数组
/// @param normals    法线数组（等长）
/// @param resolution 每轴最大体素数（默认128，越大越精细但越慢）
/// @return 三角形网格（可能为空，如输入 <3 个点）
TriMesh buildSurfaceMesh(
    const Vec3d* points,
    const Vec3d* normals,
    size_t count,
    int resolution = 128);

} // namespace midgard
```

---

## 6. 集成方式

`MicroGridLabWindow::drawActions()` 中新增按钮:

```cpp
if (ImGui::Button("Cut Surface Mesh")) {
    // 1. 提取点法数据
    // 2. 调用 buildSurfaceMesh()
    // 3. 通过 g_debugDisplay->drawTriangles() 渲染
}
```

使用现有的 `drawTriangles` 接口，颜色使用半透明橙色（与点集一致但区分）。

---

## 7. 性能预算

| 步骤 | 10k 点 | 50k 点 |
|------|--------|--------|
| KD-tree 构建 | 0.5ms | 2ms |
| 活跃体素标记 | 0.3ms | 1.5ms |
| MLS 隐式场 | 5ms | 25ms |
| Marching Cubes | 2ms | 10ms |
| 法线平滑 | 0.5ms | 2ms |
| **总计** | **~8ms** | **~40ms** |

---

## 8. 修订记录

| 版本 | 日期 | 变更 |
|------|------|------|
| 0.a | 2026-07-09 | 初始设计 |
