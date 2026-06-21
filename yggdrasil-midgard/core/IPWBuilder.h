#pragma once

#include "core/Types.h"
#include <openvdb/points/PointDataGrid.h>

#include <functional>

namespace midgard {

/// 调试日志回调 (core 层不持有输出端，通过回调解耦)
using DebugLogFn = std::function<void(const char* level, const char* message)>;

class IPWBuilder {
public:
    /// 构建完整 IPW0（含 Debug Section）
    /// @param showBoundary 是否绘制 boundary leaf bbox（受 ipwShowMacroGrid 控制）
    IPWState build(const GeometryDef& geom, const ToleranceConfig& config,
                   DebugLogFn logFn = nullptr,
                   bool showBoundary = false);

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
