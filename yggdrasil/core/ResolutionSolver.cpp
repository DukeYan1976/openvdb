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

    // 边界情况：D_upper < d_v → 不降级为 SINGLE_TRACK（避免全分辨率灾难）
    // 而是使用最小 DUAL_TRACK (N=2)，保持双轨分离的内存/计算优势
    if (N_ideal < 2) {
        cfg.mode = ResolutionConfig::DUAL_TRACK;
        cfg.n = 1;  // N = 2^1 = 2
        cfg.N = 2;
        cfg.D_v = cfg.N * cfg.d_v;
        return cfg;
    }

    // Step 4: 对齐到 2 的幂，并限制上限防止内存/计算爆炸
    const int N_MAX = 64; // 上限：每 dirty voxel 最多 4096 surfel
    cfg.n = static_cast<int>(std::floor(std::log2(static_cast<double>(N_ideal))));
    if (cfg.n > 6) cfg.n = 6; // log2(64) = 6
    cfg.N = 1 << cfg.n;
    if (cfg.N > N_MAX) cfg.N = N_MAX;

    // Step 5: 最终共享体素尺寸
    cfg.D_v = cfg.N * cfg.d_v;

    // Step 6: 模式判定 — DUAL_TRACK 是默认主路径
    double L_max = std::max({dims.x(), dims.y(), dims.z()});

    if ((L_max / cfg.d_v) > 1.67e7) {
        cfg.mode = ResolutionConfig::ATLAS_REGION;
        cfg.atlas_divisions = static_cast<int>(
            std::ceil(L_max / (1.67e7 * cfg.d_v)));
    } else {
        cfg.mode = ResolutionConfig::DUAL_TRACK;
    }

    return cfg;
}

} // namespace ygg
