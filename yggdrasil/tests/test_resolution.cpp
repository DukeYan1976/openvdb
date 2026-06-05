#include <gtest/gtest.h>
#include "core/ResolutionSolver.h"

using namespace ygg;

TEST(ResolutionSolver, SingleTrack_WhenNLessThan2) {
    // t=10 → d_v=5, R_min=1 → D_upper=min(0.5,20)=0.5 < d_v=5 → N_ideal=0 → SINGLE_TRACK
    auto cfg = solveResolution(10.0, 1.0, 20.0, {100, 100, 100});
    EXPECT_EQ(cfg.mode, ResolutionConfig::SINGLE_TRACK);
    EXPECT_EQ(cfg.N, 1);
    EXPECT_DOUBLE_EQ(cfg.D_v, cfg.d_v);
}

TEST(ResolutionSolver, DualTrack_DefaultForN2OrMore) {
    // t=1.0, R_min=50, F_min=20: d_v=0.5, D_upper=min(25,20)=20, N_ideal=40 → N=32 → DUAL_TRACK
    auto cfg = solveResolution(1.0, 50.0, 20.0, {300, 300, 300});
    EXPECT_EQ(cfg.mode, ResolutionConfig::DUAL_TRACK);
    EXPECT_DOUBLE_EQ(cfg.d_v, 0.5);
    EXPECT_EQ(cfg.N, 32);
}

TEST(ResolutionSolver, DualTrack_SmallPart) {
    // Small scale also gets DUAL_TRACK when N>=2
    auto cfg = solveResolution(0.5, 10.0, 2.0, {30, 30, 15});
    EXPECT_EQ(cfg.mode, ResolutionConfig::DUAL_TRACK);
    EXPECT_DOUBLE_EQ(cfg.d_v, 0.25);
    EXPECT_GE(cfg.N, 2);
}

TEST(ResolutionSolver, DualTrack_HighPrecisionLargePart) {
    auto cfg = solveResolution(0.01, 5.0, 1.0, {500, 500, 200});
    EXPECT_EQ(cfg.mode, ResolutionConfig::DUAL_TRACK);
    EXPECT_DOUBLE_EQ(cfg.d_v, 0.005);
    EXPECT_EQ(cfg.N, 128);
    EXPECT_DOUBLE_EQ(cfg.D_v, 0.64);
}

TEST(ResolutionSolver, Atlas_UltraPrecision) {
    // t=0.001, d_v=0.0005, L_max/d_v > 1.67e7 needs L_max > 8350mm
    auto cfg = solveResolution(0.001, 0.5, 0.1, {10000, 100, 100});
    EXPECT_EQ(cfg.mode, ResolutionConfig::ATLAS_REGION);
    EXPECT_DOUBLE_EQ(cfg.d_v, 0.0005);
}

TEST(ResolutionSolver, BoundaryCase_DupperLessThanDv) {
    // t=10 → d_v=5, D_upper=0.5 < d_v → N_ideal=0 < 2 → SINGLE_TRACK
    auto cfg = solveResolution(10.0, 1.0, 0.5, {100, 100, 100});
    EXPECT_EQ(cfg.mode, ResolutionConfig::SINGLE_TRACK);
    EXPECT_EQ(cfg.N, 1);
    EXPECT_DOUBLE_EQ(cfg.D_v, cfg.d_v);
}

TEST(ResolutionSolver, NisPowerOfTwo) {
    auto cfg = solveResolution(0.01, 5.0, 1.0, {500, 500, 200});
    EXPECT_EQ(cfg.N, 128);
    EXPECT_EQ(cfg.n, 7);
    EXPECT_EQ(cfg.N & (cfg.N - 1), 0);
}

TEST(ResolutionSolver, DvEqualsNTimesDv) {
    auto cfg = solveResolution(0.01, 5.0, 1.0, {500, 500, 200});
    EXPECT_DOUBLE_EQ(cfg.D_v, cfg.N * cfg.d_v);
}
