#include <gtest/gtest.h>
#include "core/ResolutionSolver.h"

using namespace ygg;

TEST(ResolutionSolver, SingleTrack_LowPrecisionSmallPart) {
    // t=1.0, R_min=50, F_min=20, 300x300x300
    // d_v=0.5, surface=2*(90000+90000+90000)=540000mm²
    // active≈540000/0.25*6=12.96M → ~52MB < 4GB → SINGLE_TRACK
    auto cfg = solveResolution(1.0, 50.0, 20.0, {300, 300, 300});
    EXPECT_EQ(cfg.mode, ResolutionConfig::SINGLE_TRACK);
    EXPECT_DOUBLE_EQ(cfg.d_v, 0.5);
    EXPECT_EQ(cfg.N, 1);
    EXPECT_DOUBLE_EQ(cfg.D_v, 0.5);
}

TEST(ResolutionSolver, DualTrack_HighPrecisionLargePart) {
    // t=0.01, R_min=5, F_min=1, 500x500x200
    // d_v=0.005, surface=2*(250000+100000+100000)=900000mm²
    // active≈900000/0.000025*6=2.16e11 → ~864GB >> budget → DUAL_TRACK
    auto cfg = solveResolution(0.01, 5.0, 1.0, {500, 500, 200});
    EXPECT_EQ(cfg.mode, ResolutionConfig::DUAL_TRACK);
    EXPECT_DOUBLE_EQ(cfg.d_v, 0.005);
    EXPECT_EQ(cfg.N, 128);
    EXPECT_DOUBLE_EQ(cfg.D_v, 0.64);
}

TEST(ResolutionSolver, Atlas_UltraPrecision) {
    // t=0.001, d_v=0.0005, 需要 L_max/d_v > 1.67e7
    // L_max > 1.67e7 * 0.0005 = 8350mm → 用 10000mm
    auto cfg = solveResolution(0.001, 0.5, 0.1, {10000, 100, 100});
    EXPECT_EQ(cfg.mode, ResolutionConfig::ATLAS_REGION);
    EXPECT_DOUBLE_EQ(cfg.d_v, 0.0005);
}

TEST(ResolutionSolver, BoundaryCase_DupperLessThanDv) {
    // t=10 → d_v=5, R_min=1 → D_upper=min(0.5,...)=0.5 < d_v=5
    // N_ideal < 1 → 强制 SINGLE_TRACK
    auto cfg = solveResolution(10.0, 1.0, 0.5, {100, 100, 100});
    EXPECT_EQ(cfg.mode, ResolutionConfig::SINGLE_TRACK);
    EXPECT_EQ(cfg.N, 1);
    EXPECT_DOUBLE_EQ(cfg.D_v, cfg.d_v);
}

TEST(ResolutionSolver, NisPowerOfTwo) {
    // N_ideal=200 → N=128 (2^7)
    auto cfg = solveResolution(0.01, 5.0, 1.0, {500, 500, 200});
    EXPECT_EQ(cfg.N, 128);
    EXPECT_EQ(cfg.n, 7);
    // Verify N is power of 2
    EXPECT_EQ(cfg.N & (cfg.N - 1), 0);
}

TEST(ResolutionSolver, DvEqualsNTimesDv) {
    auto cfg = solveResolution(0.01, 5.0, 1.0, {500, 500, 200});
    EXPECT_DOUBLE_EQ(cfg.D_v, cfg.N * cfg.d_v);
}
