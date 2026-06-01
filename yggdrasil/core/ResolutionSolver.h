#pragma once

#include "types/YggTypes.h"

namespace ygg {

/// 求解最优分辨率配置
/// @param t         精度公差 (mm)
/// @param R_min     最小刀具半径 (mm)
/// @param F_min     最小特征尺寸 (mm)
/// @param dims      毛坯尺寸 Vec3d(Lx, Ly, Lz) (mm)
/// @param memBudget 内存预算 (bytes), 默认 4GB
ResolutionConfig solveResolution(double t, double R_min, double F_min,
                                 const Vec3d& dims,
                                 size_t memBudget = 4ULL * 1024 * 1024 * 1024);

} // namespace ygg
