#include <gtest/gtest.h>
#include "core/CuttingEngine.h"
#include "core/BilletBuilder.h"
#include "core/ResolutionSolver.h"
#include <openvdb/points/PointCount.h>
#include <cmath>

using namespace ygg;

class CuttingEngineTest : public ::testing::Test {
protected:
    void SetUp() override { openvdb::initialize(); }

    // Force single track config for legacy single-track tests
    static ResolutionConfig singleTrackCfg(double voxelSize = 1.0) {
        ResolutionConfig cfg;
        cfg.mode = ResolutionConfig::SINGLE_TRACK;
        cfg.d_v = voxelSize; cfg.D_v = voxelSize; cfg.N = 1;
        return cfg;
    }

    // Dual track config
    static ResolutionConfig dualTrackCfg(double d_v = 0.25, int N = 4) {
        ResolutionConfig cfg;
        cfg.mode = ResolutionConfig::DUAL_TRACK;
        cfg.d_v = d_v; cfg.N = N; cfg.D_v = d_v * N;
        return cfg;
    }
};

TEST_F(CuttingEngineTest, SingleTrack_BasicCut) {
    auto billet = buildBillet(singleTrackCfg(1.0), {0,0,0}, {20, 20, 10});
    ToolSweepSDF tool(ToolType::BALL_END, 3.0, 0, 15, {5,10,0}, {15,10,0});
    CuttingEngine engine;
    engine.cut(billet, tool);
    auto& xform = billet.sdfGrid->transform();
    auto idx = xform.worldToIndexCellCentered({10, 10, 0});
    EXPECT_GT(billet.sdfGrid->getConstAccessor().getValue(idx), 0.0f);
}

TEST_F(CuttingEngineTest, SingleTrack_VolumeDecreases) {
    auto billet = buildBillet(singleTrackCfg(1.0), {0,0,0}, {20, 20, 10});
    double volBefore = computeVolume(billet.sdfGrid);
    CuttingEngine engine;
    engine.cut(billet, ToolSweepSDF(ToolType::BALL_END, 3.0, 0, 15, {5,10,0}, {15,10,0}));
    EXPECT_LT(computeVolume(billet.sdfGrid), volBefore);
}

TEST_F(CuttingEngineTest, SingleTrack_UncutRegionUnchanged) {
    auto billet = buildBillet(singleTrackCfg(1.0), {0,0,0}, {20, 20, 10});
    auto& xform = billet.sdfGrid->transform();
    auto farIdx = xform.worldToIndexCellCentered({2, 2, 5});
    float before = billet.sdfGrid->getConstAccessor().getValue(farIdx);
    CuttingEngine engine;
    engine.cut(billet, ToolSweepSDF(ToolType::BALL_END, 2.0, 0, 15, {15,15,0}, {18,15,0}));
    EXPECT_FLOAT_EQ(before, billet.sdfGrid->getConstAccessor().getValue(farIdx));
}

// ── DUAL_TRACK tests ──

TEST_F(CuttingEngineTest, DualTrack_CutGeneratesSurfels) {
    auto billet = buildBillet(dualTrackCfg(0.25, 4), {0,0,0}, {20, 20, 10});
    EXPECT_TRUE(billet.microGrid == nullptr);  // no surfels before cut

    CuttingEngine engine;
    engine.cut(billet, ToolSweepSDF(ToolType::BALL_END, 3.0, 0, 15, {5,10,0}, {15,10,0}));

    ASSERT_TRUE(billet.microGrid != nullptr);
    EXPECT_GT(openvdb::points::pointCount(billet.microGrid->tree()), 0u);
}

TEST_F(CuttingEngineTest, DualTrack_VolumeDecreases) {
    auto billet = buildBillet(dualTrackCfg(0.25, 4), {0,0,0}, {20, 20, 10});
    double volBefore = computeVolume(billet.sdfGrid);
    CuttingEngine engine;
    engine.cut(billet, ToolSweepSDF(ToolType::BALL_END, 3.0, 0, 15, {5,10,0}, {15,10,0}));
    EXPECT_LT(computeVolume(billet.sdfGrid), volBefore);
}

TEST_F(CuttingEngineTest, DualTrack_DirtyMaskClearedAfterCut) {
    auto billet = buildBillet(dualTrackCfg(0.25, 4), {0,0,0}, {20, 20, 10});
    CuttingEngine engine;
    engine.cut(billet, ToolSweepSDF(ToolType::BALL_END, 3.0, 0, 15, {5,10,0}, {15,10,0}));
    ASSERT_TRUE(billet.dirtyMask != nullptr);
    EXPECT_EQ(billet.dirtyMask->activeVoxelCount(), 0u);  // cleared after cut
}

TEST_F(CuttingEngineTest, DualTrack_MultipleCutsAccumulateSurfels) {
    auto billet = buildBillet(dualTrackCfg(0.25, 4), {0,0,0}, {20, 20, 10});
    CuttingEngine engine;
    engine.cut(billet, ToolSweepSDF(ToolType::BALL_END, 2.0, 0, 15, {5,10,0}, {10,10,0}));
    size_t count1 = openvdb::points::pointCount(billet.microGrid->tree());
    engine.cut(billet, ToolSweepSDF(ToolType::BALL_END, 2.0, 0, 15, {10,10,0}, {15,10,0}));
    size_t count2 = openvdb::points::pointCount(billet.microGrid->tree());
    EXPECT_GE(count2, count1);  // second cut adds surfels in new area
}
