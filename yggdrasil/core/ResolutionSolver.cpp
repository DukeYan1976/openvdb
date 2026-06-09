#include "core/ResolutionSolver.h"
#include <algorithm>
#include <cmath>

namespace ygg {

ResolutionConfig solveResolution(double t, double R_min, double F_min,
                                 const Vec3d& dims, size_t memBudget) {
    ResolutionConfig cfg;

    // ═══ Step 1: Micro precision (non-negotiable) ═══
    cfg.d_v = 0.5 * t;

    // ═══ Step 2: D_v Nash equilibrium ═══
    // Target: driven by tool radius (optimal for cut-zone filtering)
    double D_v_target = std::min(0.5 * R_min, F_min);

    // Floor: minimum N=4 to maintain meaningful macro partitioning
    const int N_MIN = 4;
    const int N_MAX = 16;
    double D_v_floor = N_MIN * cfg.d_v;   // 2t
    double D_v_ceil  = N_MAX * cfg.d_v;   // 8t

    // Clamp: balance tool-driven target with precision/performance bounds
    double D_v_clamped = std::clamp(D_v_target, D_v_floor, D_v_ceil);

    // ═══ Step 3: Align N to power-of-2 ═══
    int N_ideal = static_cast<int>(std::floor(D_v_clamped / cfg.d_v));
    if (N_ideal < N_MIN) {
        // Cannot form meaningful macro partition → SINGLE_TRACK
        cfg.mode = ResolutionConfig::SINGLE_TRACK;
        cfg.D_v = cfg.d_v;
        cfg.N = 1;
        cfg.n = 0;
        return cfg;
    }

    cfg.n = static_cast<int>(std::floor(std::log2(static_cast<double>(N_ideal))));
    cfg.n = std::max(cfg.n, 2); // enforce N >= 4
    cfg.N = 1 << cfg.n;

    // ═══ Step 4: Final D_v ═══
    cfg.D_v = cfg.N * cfg.d_v;

    // ═══ Step 5: Mode selection ═══
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
