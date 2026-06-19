#pragma once

#include "core/Types.h"
#include <openvdb/points/PointDataGrid.h>

namespace midgard {

class IPWBuilder {
public:
    /// 构建完整 IPW0
    IPWState build(const GeometryDef& geom, const ToleranceConfig& config);

    /**
     * @brief [标准功能] 对给定物理区域采样毛坯边界特征。
     * 
     * 从原始几何定义 (Analytical/Mesh) 提取高精度点集与法向。
     * 用于解决 NEW_BOUNDARY 体素的“冷启动”问题。
     */
    static PointBuffer sampleBoundary(const GeometryDef& geom, const openvdb::BBoxd& region);

private:
    openvdb::FloatGrid::Ptr buildMacroGrid(
        const GeometryDef& geom, const ToleranceConfig& config);

    openvdb::points::PointDataGrid::Ptr buildMicroGrid(
        const openvdb::FloatGrid::Ptr& macroGrid,
        const GeometryDef& geom,
        double t_billet);
};

} // namespace midgard
