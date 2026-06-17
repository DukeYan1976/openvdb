#pragma once

#include <openvdb/openvdb.h>
#include <openvdb/points/PointDataGrid.h>
#include <algorithm>
#include <cstdint>

namespace midgard {

using Vec3d = openvdb::Vec3d;
using Vec3f = openvdb::Vec3f;

// === 刀具定义 ===
enum class ToolType : uint8_t { BALL_END, FLAT_END, BULL_NOSE };

struct ToolDef {
    ToolType type;
    double R;    // 刀具半径 (mm)
    double r;    // 圆角半径 (mm), 仅 BULL_NOSE
    double H;    // 刀具高度 (mm)
};

// === 刀路段 ===
struct MoveSegment {
    Vec3d start;
    Vec3d end;
    Vec3d axis{0, 0, 1};   // 三轴固定Z
    double feedRate = 0.0;  // mm/min, 仅用于时间估算
};

// === 仿真配置：单参数派生 ===
struct ToleranceConfig {
    double user_t;          // 用户唯一输入：目标加工公差 (mm)

    double voxelMacro;      // 宏观体素尺寸 = K * t
    double reprojBand;      // 重投影带宽
    double baseStep;        // 流形采样基础步长
    double chordalLimit;    // 弦高细分阈值

    int    halfwidth = 3;
    double K = 30.0;        // V_macro / t 比例因子 (20~50)

    enum Mode { INTERACTIVE, FINAL };
    Mode mode = INTERACTIVE;

    static constexpr double MIN_VOXEL_SIZE = 0.02;
    static constexpr double MAX_VOXEL_SIZE = 5.0;

    explicit ToleranceConfig(double t, Mode m = INTERACTIVE, double k = 30.0)
        : user_t(t), K(k), mode(m)
    {
        voxelMacro   = std::clamp(K * t, MIN_VOXEL_SIZE, MAX_VOXEL_SIZE);
        reprojBand   = t;
        baseStep     = 10.0 * t;
        chordalLimit = t;
    }
};

// === Voxel 分类 ===
enum class VoxelClass : uint8_t {
    UNCHANGED = 0,
    DELETED,
    CUT,
    NEW_BOUNDARY
};

// === 任务负载 ===
struct VoxelTask {
    openvdb::Coord origin;
    openvdb::BBoxd aabb;
    double u_min, u_max;
    double t_min, t_max;
    VoxelClass classification;
};

// === IPW 状态 ===
struct IPWState {
    openvdb::FloatGrid::Ptr macroGrid;
    openvdb::points::PointDataGrid::Ptr microGrid;
    ToleranceConfig config;

    IPWState(double tolerance) : config(tolerance) {}
};

} // namespace midgard
