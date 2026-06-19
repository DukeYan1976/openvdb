#pragma once

#include "core/Types.h"
#include "core/ToolSweptSDF.h"
#include "core/ToolSweepSurface.h"
#include "core/MacroCut.h"
#include "debug/RtDebugSys.h"
#include <openvdb/openvdb.h>
#include <unordered_map>
#include <vector>

namespace midgard {

class MicroCut {
public:
    /// Phase 2: TBB并行四叉树自适应采样
    std::unordered_map<openvdb::Coord, PointBuffer>
    sampleNewSurface(
        const std::vector<VoxelTask>& tasks,
        const ToolSweepSurface& surface,
        const ToolSweptSDF& sdf,
        const ToleranceConfig& config,
        const IPWState& ipw);

    /// Phase 1.5: 对缺失点云的 NEW_BOUNDARY 体素，动态进行毛坯表面采样（冷启动）
    std::unordered_map<openvdb::Coord, PointBuffer>
    primeBilletBoundaries(
        const std::vector<VoxelTask>& tasks,
        const GeometryDef& billetDef,
        const ToleranceConfig& config);

    /// Phase 3+4: 旧点剔除 + per-leaf 重建 MicroGrid
    /// deleted voxels: 全部点删除
    /// cut voxels: SDF < t 的旧点删除，保留存活点 + 加入新点
    /// newBoundary voxels: 加入新点
    void rebuildLeaves(
        IPWState& ipw,
        const CutClassification& cls,
        const std::unordered_map<openvdb::Coord, PointBuffer>& newBuffers,
        const ToolSweptSDF& sdf,
        const ToleranceConfig& config);

    /// 四叉树递归核心
    void quadtreeEval(
        double u0, double u1, double v0, double v1,
        const openvdb::BBoxd& voxelAABB,
        const ToolSweepSurface& surface,
        double chordalLimit,
        int depth,
        PointBuffer& output,
        const openvdb::FloatGrid::ConstAccessor* billetAcc = nullptr,
        const openvdb::math::Transform* billetXform = nullptr,
        const PointBuffer* existingData = nullptr,
        double cullThreshold = 0.01);

private:
    static constexpr int MAX_DEPTH = 12;
};

} // namespace midgard
