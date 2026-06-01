# 第一阶段算法文档评判性审查与修改计划

**文档编号**：`Review_Phase1_20260601_v0.a.md`  
**状态**：初稿  
**作者**：Duke / Kiro  
**日期**：2026-06-01  

---

## 第一部分：评判性审查

### 1. Phase1_DevPlan

| # | 风险项 | 严重度 | 说明 |
|---|--------|--------|------|
| P1 | M4 验收标准"体积误差<5%"无理论依据 | 中 | 应用 `误差 < 表面积×d_v` 替代 |
| P2 | 未定义性能瓶颈量化阈值 | 低 | — |
| P3 | GTest 集成方式未说明 | 低 | — |

### 2. Algo_R1

| # | 风险项 | 严重度 | 修复状态 |
|---|--------|--------|----------|
| R1-1 | 内存估算假设正方体 | **高** | ✅ Fix-1 已修复 |
| R1-2 | MEMORY_BUDGET 硬编码 | 中 | ✅ Fix-1 已修复 |
| R1-3 | 单轨判定后 Step 2~5 白算 | 低 | 编码时优化 |
| R1-4 | D_upper < d_v 无降级策略 | 中 | ✅ Fix-1 已修复 |

### 3. Algo_R2

| # | 风险项 | 严重度 | 修复状态 |
|---|--------|--------|----------|
| R2-1 | IPW₀ 法向量棱边不连续 | 中 | 编码时用解析法向 |
| R2-2 | 每体素仅 1 面元 | 中 | 编码时强制 N_init≥2 |
| R2-3 | 双轨拓扑不一致 | **高** | ✅ Fix-2 已修复 |
| R2-4 | isOnSurface 未定义 | 低 | 编码时定义 |

### 4. Algo_R3

| # | 风险项 | 严重度 | 修复状态 |
|---|--------|--------|----------|
| R3-1 | 牛鼻刀 C1 不连续 | 中 | 第一阶段暂不实现牛鼻刀 |
| R3-2 | 刀轴=Z 无断言 | 低 | 编码时加 assert |
| R3-3 | 光栅化遍历效率低 | 中 | 编码时用种子扩展 |
| R3-4 | evalBatch 布局未定义 | 低 | 编码时定义 SoA |

### 5. Algo_R4

| # | 风险项 | 严重度 | 修复状态 |
|---|--------|--------|----------|
| R4-1 | SDF 近似更新导致过滤误判 | **高** | ✅ Fix-3 已修复 |
| R4-2 | N³ 遍历性能爆炸 | **高** | ✅ Fix-4 已修复 |
| R4-3 | gradient 未定义 | 中 | 编码时实现有限差分 |
| R4-4 | 多段切削顺序依赖 | 中 | SDF max 天然顺序无关 |
| R4-5 | CSG 后窄带不完整 | 中 | 编码时加 pruneLevelSet |

---

## 第二部分：性能记录方案

### 指标结构

```cpp
struct CutPerformanceRecord {
    double voxelSize;
    int halfWidth, N;
    std::string method, toolType;
    double toolRadius, segmentLength;
    size_t billetActiveVoxels, billetMemoryBytes;
    double time_total, time_rasterize, time_csg, time_eval, time_prune, time_inject;
    size_t eval_count, voxels_modified, surfels_deactivated;
    double volume_error_percent, surface_error_max_mm;
    size_t peak_memory_bytes, delta_memory_bytes;
};
```

### 基准矩阵

| 维度 | 取值 |
|------|------|
| voxelSize | 0.5, 0.1, 0.05, 0.01 mm |
| halfWidth | 2, 3, 5 |
| tool | BALL×5mm, FLAT×10mm |
| segmentLength | 1, 5, 10, 50 mm |
| 毛坯 | 20³, 50³, 100³ mm |
| method | CSG, DirectEval |

### 评价

```
Score = w1/time + w2/memory + w3/error
均衡权重: (0.4, 0.3, 0.3)
```

输出 CSV → 帕累托前沿分析 → 选最优超参数。

---

## 第三部分：修改执行总结

| Fix | 严重度 | 状态 | 文件 |
|-----|--------|------|------|
| Fix-1 | 高 | ✅ 完成 | R1 §6.1 + 伪代码 |
| Fix-2 | 高 | ✅ 完成 | R2 §4.8 |
| Fix-3 | 高 | ✅ 完成 | R4 §4.4 |
| Fix-4 | 高 | ✅ 完成 | R4 §4.5 |

---

## 修订记录

| 版本 | 日期 | 修订内容 |
|------|------|----------|
| v0.a | 2026-06-01 | 初稿创建 |
