#include "core/ResolutionSolver.h"
#include <algorithm>
#include <cmath>

namespace ygg {

ResolutionConfig solveResolution(double t, double R_min, double F_min,
                                 const Vec3d& dims, size_t memBudget) {
    ResolutionConfig cfg;

    // Step 1: 微观基准
    cfg.d_v = 0.5 * t;

    // Step 2: 宏观体素物理上界
    double D_upper = std::min(0.5 * R_min, F_min);

    // Step 3: 理想面元密度因子
    int N_ideal = static_cast<int>(std::floor(D_upper / cfg.d_v));

    // 边界情况：D_upper < d_v
    if (N_ideal < 1) {
        cfg.mode = ResolutionConfig::SINGLE_TRACK;
        cfg.D_v = cfg.d_v;
        cfg.N = 1;
        cfg.n = 0;
        return cfg;
    }

    // Step 4: 对齐到 2 的幂
    cfg.n = static_cast<int>(std::floor(std::log2(static_cast<double>(N_ideal))));
    cfg.N = 1 << cfg.n;

    // Step 5: 最终共享体素尺寸
    cfg.D_v = cfg.N * cfg.d_v;

    // Step 6: 模式判定
    double Lx = dims.x(), Ly = dims.y(), Lz = dims.z();
    double L_max = std::max({Lx, Ly, Lz});

    // 单轨可行性：用实际表面积估算内存
    double surfaceArea = 2.0 * (Lx * Ly + Ly * Lz + Lx * Lz);
    double activeVoxels = (surfaceArea / (cfg.d_v * cfg.d_v)) * 6.0; // 2*halfWidth=6
    double memoryEst = activeVoxels * 4.0; // float = 4 bytes

    if (memoryEst < static_cast<double>(memBudget)) {
        cfg.mode = ResolutionConfig::SINGLE_TRACK;
        cfg.D_v = cfg.d_v;
        cfg.N = 1;
        cfg.n = 0;
    } else if ((L_max / cfg.d_v) > 1.67e7) {
        cfg.mode = ResolutionConfig::ATLAS_REGION;
        cfg.atlas_divisions = static_cast<int>(
            std::ceil(L_max / (1.67e7 * cfg.d_v)));
    } else {
        cfg.mode = ResolutionConfig::DUAL_TRACK;
    }

    return cfg;
}

} // namespace ygg
