# 毛坯构建评审：触发正确性 + 数据结构冗余

**审查范围**：`ResolutionSolver.cpp` + `BilletBuilder.cpp` + `YggTypes.h`  
**日期**：2026-06-03  
**限定**：仅毛坯构建阶段

---

## 摘要

**核心洞见**：DUAL_TRACK 触发逻辑正确，但内存估算公式低估约 2×（忽略叶节点开销），边界情况可能 OOM。OpenVDB 数据结构无显著冗余——FloatGrid 用 inactive tile 节省内部存储，PointDataGrid 面元是精度补偿的必须代价。

---

## 1. 触发逻辑审查

### 1.1 判定流程

```
Step 1: d_v = 0.5 × t
Step 2: D_upper = min(0.5 × R_min, F_min)
Step 3: N_ideal = floor(D_upper / d_v)
        → N_ideal < 1  → 强制 SINGLE_TRACK  ← 边界保护 ✓
Step 4: N = 2^floor(log₂(N_ideal))
Step 5: D_v = N × d_v
Step 6: mode ∈ {SINGLE, DUAL, ATLAS}
```

### 1.2 实测追踪

| 输入 | d_v | D_upper | N_ideal | N | D_v | 内存估算 | 判定 |
|------|-----|---------|---------|---|-----|----------|------|
| t=1.0, R50, F20, 300³ | 0.5 | 2 | 4 | 4→1¹ | 0.5 | ~52MB | SINGLE ✓ |
| t=0.01, R5, F1, 500²×200 | 0.005 | 1 | 200 | 128 | 0.64 | ~864GB | DUAL ✓ |
| t=0.05, R10, F2, 300×200×100 | 0.025 | 2 | 80 | 64 | 1.6 | ~8.4GB | DUAL ✓ |
| t=0.1, R10, F2, 100³ | 0.05 | 2 | 40 | 32 | 1.6 | ~528MB | SINGLE ✓ |

¹ N_ideal=4 但 SINGLE_TRACK 模式下强制 N=1（单轨不需要面元密度因子）  
² 500²=500×500

### 1.3 边界情况验证 ✅

| 边界 | 处理 | 结果 |
|------|------|------|
| D_upper < d_v (极粗精度) | N_ideal < 1 → 强制 SINGLE | 防止 N=0 除零 |
| N_ideal 不是 2 的幂 | floor(log₂(N)) 向下取整 | 保守，保证 D_v ≥ N×d_v |
| 内存恰好 < 预算 | SINGLE_TRACK | 正确，预算内允许单轨 |
| L_max/d_v > 1.67e7 | ATLAS_REGION | Phase 1 不实现 |

---

## 2. 🔴 内存估算缺陷

### 2.1 当前公式

```cpp
double activeVoxels = (surfaceArea / (d_v * d_v)) * 6.0;   // 2×halfWidth=6
double memoryEst = activeVoxels * 4.0;                      // float=4B
```

**只计算了体素值存储，未计入叶节点开销。**

### 2.2 OpenVDB 实际存储模型

```
OpenVDB FloatGrid 内存 = 体素数据 + 叶节点元数据 + 内部节点

每个叶节点：
  - 512 个体素 (8³)
  - 体素值数组: 512 × 4B = 2KB
  - 叶节点元数据: ~2KB (bitmask, origin, descriptor)
  → 叶节点总开销 / 体素数 = (2KB+2KB)/512 ≈ 8B/体素 ← 2× 低估！
```

### 2.3 边界 OOM 场景

构造一个公式显示安全但实际危险的 case：

```
毛坯: 400×300×200mm, t=0.05, R_min=5, F_min=1
d_v = 0.025, D_upper=1 → N=32, D_v=0.8

surfaceArea = 2*(400×300 + 300×200 + 400×200) = 2*(120K+60K+80K) = 520K
当前估计: activeVoxels = 520K/0.000625 × 6 = 4.992B
         memoryEst = 4.992B × 4 = 19.97GB > 4GB → DUAL_TRACK ✓

一个"恰好通过"的 case（d_v=0.125, t=0.25）：
activeVoxels = 520K/0.015625 × 6 = 199.7M
memoryEst = 199.7M × 4 = 0.80GB < 4GB → SINGLE_TRACK  ← 公式说 OK

实际内存: 199.7M × 8B ≈ 1.6GB + 树节点 → ~2GB  ← 实际 2×
→ 仍 < 4GB，安全。但更精细的 case（d_v=0.06, t=0.12）：
memoryEst = 866M × 4 = 3.46GB < 4GB → SINGLE_TRACK
实际内存: 866M × 8B ≈ 6.9GB + 树节点 → ~8GB  ← ⚠️ 危险
```

### 2.4 修复建议

```cpp
// 考虑叶节点开销（~2KB/leaf → ~4B/voxel 额外开销）
double activeVoxels = (surfaceArea / (cfg.d_v * cfg.d_v)) * 6.0;
double leafCount = activeVoxels / 512.0;               // 8³=512 voxels/leaf
double memoryEst = activeVoxels * 4.0 + leafCount * 2048.0;  // 体素 + 叶节点
// 或简化为: memoryEst = activeVoxels * 8.0;  // 经验 factor=2
```

---

## 3. 数据结构冗余审查

### 3.1 FloatGrid SDF 存储

```
┌─────────────────────────────────────────────────┐
│ FloatGrid (VoxelSize=D_v, GRID_LEVEL_SET)       │
│                                                 │
│  内部体素: inactive tile, 值=-bandWidth         │
│    → 不占用叶节点 ← 关键优化 ✓                   │
│                                                 │
│  窄带体素 (|SDF| < bandWidth): active leaf      │
│    → 仅表面 ±3 层体素有叶节点                    │
└─────────────────────────────────────────────────┘
```

**冗余检查**：
- ✅ 内部体素用 `fill(..., active=false)` → OpenVDB 不创建叶节点
- ✅ 窄带外体素不遍历 → 无多余激活
- ✅ `pruneLevelSet` 未调用（R4 阶段才需要）—— 构建阶段无冗余清理需求

### 3.2 PointDataGrid 面元存储

```
每个面元: P(12B) + normal(12B) + precision(1B) + active(1B) = 26B
```

| 字段 | 大小 | 用途 | 冗余？ |
|------|------|------|--------|
| P (Vec3f) | 12B | 面元世界位置 | ❌ 每面元唯一 |
| normal (Vec3f) | 12B | 表面法向 | ⚠️ 同面面元全同，但切削后各不同 |
| precision (uint8) | 1B | COARSE/FINE | ❌ 调试/着色必需 |
| active (uint8→Group) | 1B→1bit | 活/死标记 | ❌ 切削逻辑必需 |

### 3.3 FloatGrid ↔ PointDataGrid 是否重复？

```
FloatGrid:  体素 (i,j,k) 的 SDF = -1.2  ← "这个体素中心在材料内部 1.2mm"
PointDataGrid: 体素 (i,j,k) 的面元 P = (i×D_v, j×D_v, z_surface)  ← "表面在 z=z_surface"

SDF 告诉你"大概在材料里"，面元告诉你"精确表面在哪"
→ 互补，非冗余 ✓
```

### 3.4 IPW₀ normal 冗余量化

对于一个 30×30×20mm 毛坯（D_v=6.4, d_v_init=6.4, N_init=1）：
- 面元总数 ≈ 40（每表面体素 1 个）
- normal 总存储: 40 × 12B = 480B
- 唯一 normal 数: 6 个（±X, ±Y, ±Z）
- 冗余率: 480B / 72B = 6.7×

**但这是 IPW₀ 特例。切削后 normal 多样性 → ∞**。不值得为此引入面级元数据。

### 3.5 NullCodec 选择

```cpp
createPointDataGrid<NullCodec, PointDataGrid>(points, *xform)
```

NullCodec = 无压缩，直接存储 Vec3f。备选：
- `FixedPointCodec<1e-4>` — 定点量化，精度 0.1μm，压缩 ~30%
- `TruncateCodec` — 截断到 N 位有效数字
- 当前选择对 MVP 合理，CPU 开销最低

---

## 4. 结论

| 维度 | 判定 | 说明 |
|------|------|------|
| **DUAL_TRACK 触发逻辑** | ✅ 正确 | 三步判定清晰，边界处理完备 |
| **内存估算** | ⚠️ 低估 2× | 忽略叶节点开销，边界 OOM 风险 |
| **FloatGrid 数据冗余** | ✅ 无冗余 | inactive tile 压缩内部，窄带激活精确 |
| **PointDataGrid 数据冗余** | ✅ 可控 | normal 在 IPW₀ 阶段有冗余但属设计取舍 |
| **双轨间数据重复** | ✅ 无重复 | SDF 与面元互补，非重叠 |

### 建议

| P | 项目 | 工时 |
|---|------|------|
| P1 | 内存估算公式 +叶节点开销 | 10min |
| P2 | 补充边界 OOM 测试用例 (d_v=0.06) | 20min |

---

## 修订记录

| 版本 | 日期 | 修订内容 |
|------|------|----------|
| v0.a | 2026-06-03 | 初稿创建 |
