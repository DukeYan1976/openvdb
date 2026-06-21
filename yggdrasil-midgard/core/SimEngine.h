#pragma once

#include "core/Types.h"

namespace midgard {

struct CutResult {
    bool success = false;
    int  deletedVoxels = 0;
    int  cutVoxels = 0;        // 边界voxel（已有点集）
    int  newBoundaryVoxels = 0;// 边界voxel（新建）
    double elapsedMs = 0.0;
    double pathLength = 0.0;
    int  affectedVoxels = 0;   // deleted + cut + newBoundary
};

/// 单段切削管线封装
/// 将 MacroCut + MicroCut 五阶段管线封装为一次调用 cutSegment()。
/// 无内部状态 — 不缓存任何 grid，可复用。
class SimEngine {
public:
    CutResult cutSegment(IPWState& ipw, const MoveSegment& seg,
                         const ToolDef& tool, const GeometryDef& billet,
                         const ToleranceConfig& config);
};

} // namespace midgard
