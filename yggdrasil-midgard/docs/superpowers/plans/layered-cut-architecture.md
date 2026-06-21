# 分层切削计算方案

> 版本：v0.a | 日期：2026-06-20

## 修订记录

| 版本 | 日期 | 说明 |
|------|------|------|
| 0.a | 06-20 | 初始方案记录 |

---

## 1. 核心思想

**快慢分离**：切削时只做快速拓扑计算（实时可交互），精度计算延迟到需要时按 voxel 局部执行。

## 2. 刀路预处理：精度驱动分段

任意长度刀路 → 按弦高/精度要求自适应分段 → 每段是一个**最小扫掠体单元**。

```
长刀路 → [seg₀, seg₁, seg₂, ..., segₙ]
每段满足: 弦高偏差 < t (用户公差)
```

分段结果线性增长：N 段 → N 个 ToolSweptSDF 实例（参数化存储，不占大内存）。

## 3. 三层架构

```
┌──────────────────────────────────────────────────┐
│ Layer 0: MacroGrid (即时 CSG)                    │
│   • FloatGrid, 粗 SDF                           │
│   • 每段切削做 CSG 差集 → 更新拓扑              │
│   • 决定: boundary / air / solid                 │
│   • 性能: ~100ms/段 (可视化可接受)              │
├──────────────────────────────────────────────────┤
│ Layer 1: CutHistory (追加记录)                   │
│   • 每个 boundary voxel 维护一个段编号列表       │
│   • 切削时 O(1) 追加                            │
│   • 数据: std::vector<uint16_t> per voxel       │
│   • 意义: "哪些段的扫掠体影响过我"              │
├──────────────────────────────────────────────────┤
│ Layer 2: SurfaceCache (按需精确计算)             │
│   • 精确表面点 (position + normal)              │
│   • 通过回放 CutHistory 的合成 SDF 生成         │
│   • 密度满足弦高 < t                            │
│   • 过期标记: epoch != current → 需重算         │
└──────────────────────────────────────────────────┘
```

## 4. 操作流程

### 4.1 切削（实时）

```
对每段 seg_i:
  ① MacroGrid CSG 差集 (拓扑快速更新)
  ② 对所有受影响的 boundary voxel:
       voxel.cutHistory.push_back(i)
  ③ 标记受影响 voxel 的 SurfaceCache 过期
```

### 4.2 精确计算（按需）

```
对某个 voxel V (需要高精度数据时):
  ① 合成 SDF:
       sdf(P) = max(billet(P), -tool[h₀](P), -tool[h₁](P), ...)
       其中 h₀,h₁,... = V.cutHistory
  ② 在合成 SDF 的零等值面上自适应采样
       (四叉树/MC, 弦高 < t)
  ③ 结果写入 SurfaceCache, 标记 epoch
```

### 4.3 压实（可选优化）

当 `cutHistory.size() > K` (如 8~16):
- 执行一次精确计算
- 结果固化为"等效单一 SDF"或直接保留点集
- 清空 history

## 5. 内存估算

以 50×50×20mm 毛坯, V=0.5mm, boundary ~5000 voxels 为例:

| 层 | 数据量 |
|----|--------|
| L0 MacroGrid | ~2MB (OpenVDB FloatGrid) |
| L1 CutHistory | 5000 × avg(4 entries) × 2B = 40KB |
| L2 SurfaceCache | 5000 × avg(20 pts) × 24B = 2.4MB |
| 段参数存储 | N_segs × 80B (ToolDef+start+end) ≈ 几十KB |
| **总计** | **< 5MB** |

## 6. 性能特征

| 操作 | 复杂度 | 延迟 |
|------|--------|------|
| 切削(每段) | O(affected_voxels) | ~100ms (交互可接受) |
| History 追加 | O(1) per voxel | 微秒级 |
| 精确求值(单voxel) | O(history_len × sampling_pts) | 1~10ms |
| 全局精确重算 | O(dirty_voxels × above) | 按需触发 |

## 7. 精度保证

- **实时层 (L0)**：精度 = voxelSize/2 ≈ 0.25mm (视觉可接受,拓扑正确)
- **精确层 (L2)**：精度 = t (用户公差, 如 0.001mm)
- **一致性**：L2 通过回放 L1 的解析 SDF 计算，精度由解析公式保证，不受 L0 离散化影响

## 8. 关键接口

```cpp
// 段参数存储（全局池）
struct SegmentRecord {
    ToolDef tool;
    MoveSegment seg;
    uint32_t seqIndex;   // 全局切削序号（单调递增）
    // 可按需构造 ToolSweptSDF
};
std::vector<SegmentRecord> g_segmentPool;

// Voxel 附加数据
struct VoxelCutState {
    std::vector<uint16_t> cutHistory;  // 引用 g_segmentPool 索引（已按时序排列）
    uint32_t cacheEpoch = 0;           // L2 缓存有效性标记
};

// 精确求值（可指定时间截止点，实现任意时刻回溯）
float evalComposite(const Vec3d& P, const VoxelCutState& state,
                    const GeometryDef& billet,
                    uint32_t upToSeq = UINT32_MAX) {
    float sdf = billetSDF(P, billet);
    for (uint16_t idx : state.cutHistory) {
        if (g_segmentPool[idx].seqIndex > upToSeq) break;  // 时间截止
        ToolSweptSDF tool(g_segmentPool[idx].tool, g_segmentPool[idx].seg);
        sdf = std::max(sdf, -(float)tool.eval(P));
    }
    return sdf;
}
```

## 9. 与现有架构的关系

| 现有模块 | 在新方案中的角色 |
|----------|-----------------|
| MacroGrid + MacroCut | Layer 0, 不变 |
| ToolSweptSDF | 解析 SDF 提供者, 不变 |
| MicroCut Phase 2 (四叉树采样) | Layer 2 的按需触发器 |
| MicroCut Phase 3+4 (重建) | Layer 2 写入 |
| Phase 1 (buildTaskList) | 简化为 closestLambda 反投影 |
| Phase 1.5 (毛坯补全) | 仍需, 作为首次切削时的 L2 冷启动 |

## 10. 并行化与 GPU 策略

### 10.1 并行适配性

核心优势：**voxel 间零依赖**，天然 data-parallel。

| 操作 | 粒度 | 依赖关系 |
|------|------|----------|
| L0 CSG | per-voxel | 无（每个 voxel 独立 eval） |
| L1 History 追加 | per-voxel | 无（原子 append） |
| L2 精确求值 | per-voxel | 无（history 回放自洽） |
| L2 表面采样 | per-voxel | 无（局部四叉树/MC） |

### 10.2 实施路径

| 阶段 | 方案 | 收益 | 迁移成本 |
|------|------|------|----------|
| 近期 | TBB `parallel_for` per-voxel | 4~8x (M4 10核) | 极低 |
| 中期 | Metal Compute Shader (L2 批量精确) | 10~50x | 中 |
| 远期 | NanoVDB 全 GPU 管线 | 100x+ | 高 |

### 10.3 TBB 方案（近期）

```cpp
// L0: 并行分类
tbb::parallel_for(tbb::blocked_range<size_t>(0, leaves.size()),
    [&](const auto& range) {
        for (size_t i = range.begin(); i < range.end(); ++i)
            classifyLeaf(leaves[i], toolSDF);
    });

// L2: 并行精确计算
tbb::parallel_for(tbb::blocked_range<size_t>(0, dirtyVoxels.size()),
    [&](const auto& range) {
        for (size_t i = range.begin(); i < range.end(); ++i)
            rebuildSurfaceCache(dirtyVoxels[i]);
    });
```

### 10.4 GPU 方案（中期，Metal Compute）

M4 统一内存架构 → CPU/GPU 共享物理内存，`segmentPool` 和 `cutHistory` 无需拷贝。

```
Kernel: batchPreciseEval
  threads = dirty_voxels.count
  每个 thread:
    读取 cutHistory[]
    对采样点网格逐点 eval composite SDF
    stream compaction 输出零交叉点
```

**GPU 挑战与解法**：

| 问题 | 解法 |
|------|------|
| cutHistory 长度不一致 → thread divergence | 按 history 长度分桶调度 |
| ToolSweptSDF 有刀型分支 | 单次切削同刀型无分支；混合时按刀型分批 |
| 动态点集输出 | 两遍法（count → write） |

### 10.5 M4 硬件优势

- 统一内存：无 PCIe 瓶颈，数据零拷贝
- 10 CPU 核 + 10 GPU 核：混合调度（L0 用 CPU/TBB，L2 用 GPU）
- Metal 3：mesh shader 可直接从 compute 输出渲染，跳过 CPU 回读

---

*待实现。当前 SimEngine 继续使用 Phase 0 only 模式进行测试验证。*
