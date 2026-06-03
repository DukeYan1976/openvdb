#pragma once

#include <openvdb/openvdb.h>
#include <openvdb/tools/LevelSetSphere.h>
#include <openvdb/points/PointDataGrid.h>
#include <cstdint>

namespace ygg {

using Vec3d = openvdb::Vec3d;

enum class ToolType : uint8_t {
    BALL_END,
    FLAT_END,
    BULL_NOSE
};

struct ResolutionConfig {
    double d_v = 0.0;   // 面元采样间距 (mm)
    double D_v = 0.0;   // 共享体素尺寸 (mm)
    int    N   = 1;     // 面元密度因子
    int    n   = 0;     // 幂次 (N = 2^n)

    enum Mode { SINGLE_TRACK, DUAL_TRACK, ATLAS_REGION };
    Mode mode = SINGLE_TRACK;

    int atlas_divisions = 0;
};

struct BilletModel {
    openvdb::FloatGrid::Ptr sdfGrid;
    openvdb::points::PointDataGrid::Ptr microGrid;  // 面元网格（与 sdfGrid 共享 Transform）

    ResolutionConfig config;
    Vec3d origin;
    Vec3d dims;

    bool isSingleTrack() const { return config.mode == ResolutionConfig::SINGLE_TRACK; }
    bool isDualTrack() const { return config.mode == ResolutionConfig::DUAL_TRACK; }
};

} // namespace ygg
