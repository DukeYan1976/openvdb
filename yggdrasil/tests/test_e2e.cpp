#include <gtest/gtest.h>
#include "core/ResolutionSolver.h"
#include "core/BilletBuilder.h"
#include "core/ToolSweepSDF.h"
#include "core/CuttingEngine.h"
#include <openvdb/points/PointCount.h>
#include <cmath>

using namespace ygg;

class EndToEndTest : public ::testing::Test {
protected:
    void SetUp() override { openvdb::initialize(); }
};

TEST_F(EndToEndTest, FullPipeline_DualTrack) {
    // Small part now gets DUAL_TRACK (N>=2)
    auto cfg = solveResolution(0.5, 10.0, 2.0, {30, 30, 15});
    ASSERT_EQ(cfg.mode, ResolutionConfig::DUAL_TRACK);
    EXPECT_DOUBLE_EQ(cfg.d_v, 0.25);

    auto billet = buildBillet(cfg, {0,0,0}, {30, 30, 15});
    ASSERT_TRUE(billet.sdfGrid != nullptr);
    EXPECT_TRUE(billet.microGrid == nullptr);  // lazy: no surfels yet

    double volOriginal = computeVolume(billet.sdfGrid);
    EXPECT_NEAR(volOriginal, 13500.0, volOriginal * 0.05);

    // Execute cut
    CuttingEngine engine;
    engine.cut(billet, ToolSweepSDF(ToolType::BALL_END, 3.0, 0, 20, {5,15,0}, {25,15,0}));

    double volAfter = computeVolume(billet.sdfGrid);
    EXPECT_LT(volAfter, volOriginal);
    EXPECT_GT(volAfter, 0.0);

    // Surfels generated after cut
    ASSERT_TRUE(billet.microGrid != nullptr);
    EXPECT_GT(openvdb::points::pointCount(billet.microGrid->tree()), 0u);

    // Cut path center should be air
    auto& xform = billet.sdfGrid->transform();
    auto idx = xform.worldToIndexCellCentered({15, 15, 0});
    EXPECT_GT(billet.sdfGrid->getConstAccessor().getValue(idx), 0.0f);

    // Far region unchanged
    auto farIdx = xform.worldToIndexCellCentered({5, 5, 7});
    EXPECT_LT(billet.sdfGrid->getConstAccessor().getValue(farIdx), 0.0f);
}

TEST_F(EndToEndTest, MultipleCuts_SlotMilling) {
    auto cfg = solveResolution(0.5, 10.0, 2.0, {40, 40, 10});
    auto billet = buildBillet(cfg, {0,0,0}, {40, 40, 10});
    double volBefore = computeVolume(billet.sdfGrid);

    CuttingEngine engine;
    for (int i = 0; i < 5; ++i) {
        double y = 10.0 + i * 4.0;
        engine.cut(billet, ToolSweepSDF(ToolType::BALL_END, 2.0, 0, 15,
                                        {5, y, 0}, {35, y, 0}));
    }

    EXPECT_LT(computeVolume(billet.sdfGrid), volBefore);

    if (billet.isDualTrack()) {
        ASSERT_TRUE(billet.microGrid != nullptr);
        EXPECT_GT(openvdb::points::pointCount(billet.microGrid->tree()), 0u);
    }
}
