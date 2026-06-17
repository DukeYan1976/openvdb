#pragma once

#include "core/Types.h"
#include "core/ToolSweepSDF.h"
#include "core/ToolSweepSurface.h"
#include "core/MacroCut.h"
#include "debug/RtDebugSys.h"
#include <openvdb/openvdb.h>
#include <unordered_map>
#include <vector>

namespace midgard {

/// Phase 2 输出: 每个voxel的新采样点
struct PointBuffer {
    std::vector<Vec3f> positions;  // 世界坐标 (Phase 4转相对坐标)
    std::vector<Vec3f> normals;    // 单位法线
};

class MicroCut {
public:
    /// Phase 2: TBB并行四叉树自适应采样
    std::unordered_map<openvdb::Coord, PointBuffer>
    sampleNewSurface(
        const std::vector<VoxelTask>& tasks,
        const ToolSweepSurface& surface,
        const ToolSweepSDF& sdf,
        const ToleranceConfig& config);

    /// Phase 3+4: 旧点剔除 + per-leaf 重建 MicroGrid
    /// deleted voxels: 全部点删除
    /// cut voxels: SDF < t 的旧点删除，保留存活点 + 加入新点
    /// newBoundary voxels: 加入新点
    void rebuildLeaves(
        IPWState& ipw,
        const CutClassification& cls,
        const std::unordered_map<openvdb::Coord, PointBuffer>& newBuffers,
        const ToolSweepSDF& sdf,
        const ToleranceConfig& config);

private:
    /// 四叉树递归核心
    void quadtreeEval(
        double u0, double u1, double v0, double v1,
        const openvdb::BBoxd& voxelAABB,
        const ToolSweepSurface& surface,
        double chordalLimit,
        int depth,
        PointBuffer& output);

    static constexpr int MAX_DEPTH = 12;
};

} // namespace midgard
