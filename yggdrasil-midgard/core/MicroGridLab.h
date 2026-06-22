#pragma once

#include "core/Types.h"
#include "core/ToolSweptSDF.h"
#include "core/OctreeRefiner.h"
#include <openvdb/math/Coord.h>
#include <vector>
#include <cstdint>
#include <cmath>

namespace midgard {

// ─────────────────────────────────────────────────────────────
// 单个 MicroGrid cell (8³=512 ce)
// 借鉴 OpenVDB LeafNode：只存窄带，其余用状态位表示
// ─────────────────────────────────────────────────────────────
struct MicroGridCell {
    uint64_t activeMask[8] = {};   // 512-bit: ce 在窄带内
    uint64_t signMask[8]   = {};   // 512-bit: 0=外部/空气, 1=内部/实心
    std::vector<int16_t> sdf;      // 窄带 ce 的量化 SDF（变长）

    // ── 位操作 ──
    int  activeCount() const;
    bool isActive(int idx) const;
    bool isInside(int idx) const;
    bool isOutside(int idx) const;
    void setActiveBit(int idx, bool v);
    void setSignBit(int idx, bool v);

    // ── 批量操作 ──
    int  solidCount() const;
    void setAllSolid();
    void clear();

    // ── SDF 读写（含隐式 bg 回退）──
    /// 读 ce 的 SDF 值。非 active ce 返回 ±bg（正=材料/内部）。
    float getValue(int idx, float bg) const;

    // ── M2: 从 SDF 数组构建 cell ──
    /// 给定 512 个 ce 中心处的 tool_sdf，分类并构建 mask + 压缩 SDF
    /// @param sdf512    512 个 ce 中心的 tool_sdf 值
    /// @param threshold 边界判定阈值（|sdf| < threshold → BOUNDARY）
    /// @param bg        窄带半宽，用于量化编码
    /// @return {airCount, solidCount, boundaryCount} 三态 ce 计数
    struct Classification {
        int airCount      = 0;
        int solidCount    = 0;
        int boundaryCount = 0;
    };
    Classification buildFromSDF(const float sdf512[512],
                                float threshold, float bg);

    // ── M5: 增量 ce 更新 ──
    /// 用新刀 SDF 更新单个 ce 的状态
    /// @param ceIdx     ce 索引 (0..511)
    /// @param toolSDF   新刀的 ce 中心 SDF 值
    /// @param threshold BOUNDARY 判定阈值
    /// @param bg        窄带半宽
    /// @return 旧状态 → 新状态的变化
    enum class CeUpdate { NO_CHANGE, AIR_TO_BND, AIR_TO_SOLID,
                          SOLID_TO_BND, SOLID_TO_AIR,
                          BND_TO_AIR, BND_UNCHANGED };
    CeUpdate updateCeFromToolSDF(int ceIdx, float toolSDF,
                                  float threshold, float bg);

};

// ─────────────────────────────────────────────────────────────
// 切削记录
// ─────────────────────────────────────────────────────────────
struct CutRecord {
    ToolDef tool;
    MoveSegment segment;
    uint32_t seqIndex = 0;
};

// ─────────────────────────────────────────────────────────────
// MicroGrid Lab 全局状态
// ─────────────────────────────────────────────────────────────
struct MicroGridLabState {
    // ── 参数 ──
    double cubeSize   = 1.0;   // mm
    double voxelSize  = 0.5;   // mm
    double precision  = 0.01;  // mm (t)

    // ── 刀具/路径（当前编辑用）──
    ToolDef     currentTool{ToolType::BALL_END, 2.0, 0.0, 10.0};
    MoveSegment currentSegment{Vec3d(0, 0.5, 0.8), Vec3d(1, 0.5, 0.8), Vec3d(0, 0, 1), 0};

    // ── 数据 ──
    std::vector<MicroGridCell> voxels;     // 2×2×2 = 8（默认）
    std::vector<CutRecord> cutHistory;
    std::vector<SurfaceSample> surfacePoints;  // 最近一次切削产生的表面点

    // ── 统计 ──
    int    totalSurfacePoints = 0;
    double lastCutMs          = 0.0;
    double maxChordalError    = 0.0;
    int    totalRefineEvals   = 0;       // 精修阶段 SDF 评估总次数
    int    deepestRefineDepth = 0;       // 精修到达的最大深度

    // ── 日志（最近一次切削的详细信息）──
    struct CutLog {
        int airCeBefore   = 0;
        int solidCeBefore = 0;
        int bndCeBefore   = 0;
        int airCeAfter    = 0;
        int solidCeAfter  = 0;
        int bndCeAfter    = 0;
        double elapsedMs  = 0.0;
    };
    CutLog lastCutLog;

    // ── M6: 执行日志（逐阶段记录）──
    std::string executeLog;   // executeCut 内部填充

    /// 表面点钳位：确保所有点都在立方体 [0, cubeSize]³ 范围内
    void clampSurfacePoints();
    void ensureInit();

    // ── 方法 ──
    void init();                          // 重建网格（保留 cutHistory）
    void reset();                         // 重置网格（保留 cutHistory）= init()
    void rebuild();                       // init() + 重放所有 cutHistory
    int  voxelCount() const;              // (cubeSize/voxelSize)³
    int  perDim() const;                  // cubeSize/voxelSize (每维度体素数)
    int  maxOctreeDepth() const;          // ceil(log2(voxelSize/8 / precision))
    void addCutRecord(const CutRecord& rec);

    // ── M2: 切削执行 ──
    /// 对 cutHistory[cutIdx] 执行粗筛分类
    void executeCut(size_t cutIdx);

    // ── M5: 增量切削 ──
    /// 增量切削：只处理新刀 SDF，不重建整棵树
    /// @param cutIdx cutHistory 中要执行的新刀索引
    void executeCutIncremental(size_t cutIdx);

    // ── 增量统计（最近一刀的 delta）──
    struct IncrementalDelta {
        int solidToAir   = 0;
        int solidToBnd   = 0;
        int bndToAir     = 0;
        int bndUpdated   = 0;
        int oldPtsKept   = 0;
        int oldPtsPruned = 0;
    };
    IncrementalDelta lastDelta;

    /// CE 在世界坐标系的中心
    Vec3d ceCenterWorld(int voxelIdx, int ceIdx) const;
    /// Voxel 在世界坐标系的原点
    Vec3d voxelOrigin(int voxelIdx) const;

private:
    /// 阈值：|sdf| < 此值的 ce 判定为 BOUNDARY
    double boundaryThreshold() const;
    /// 窄带半宽（SDF 量化用）
    double narrowBandHalfWidth() const;
};

} // namespace midgard
