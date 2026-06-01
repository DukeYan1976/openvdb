#pragma once

#include <cstdint>
#include <algorithm>
#include <cmath>
#include <array>

namespace ygg {

/// 简单3D向量（第一阶段不强制依赖OpenVDB头文件）
struct Vec3d {
    double v[3] = {0, 0, 0};
    Vec3d() = default;
    Vec3d(double x, double y, double z) : v{x, y, z} {}
    double x() const { return v[0]; }
    double y() const { return v[1]; }
    double z() const { return v[2]; }
};

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

} // namespace ygg
