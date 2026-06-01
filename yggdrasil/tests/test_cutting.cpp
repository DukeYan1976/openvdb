#include <gtest/gtest.h>
#include "core/CuttingEngine.h"
#include "core/BilletBuilder.h"
#include "core/ResolutionSolver.h"
#include <cmath>

using namespace ygg;

class CuttingEngineTest : public ::testing::Test {
protected:
    void SetUp() override { openvdb::initialize(); }
};

TEST_F(CuttingEngineTest, SingleTrack_BasicCut) {
    auto cfg = solveResolution(1.0, 50.0, 20.0, {20, 20, 10});
    auto billet = buildBillet(cfg, {0,0,0}, {20, 20, 10});

    ToolSweepSDF tool(ToolType::BALL_END, 3.0, 0, 15, {5,10,0}, {15,10,0});

    CuttingEngine engine;
    engine.cut(billet, tool);

    // 切削路径中心应变为空气
    auto& xform = billet.sdfGrid->transform();
    auto idx = xform.worldToIndexCellCentered({10, 10, 0});
    EXPECT_GT(billet.sdfGrid->getConstAccessor().getValue(idx), 0.0f);
}

TEST_F(CuttingEngineTest, SingleTrack_UncutRegionUnchanged) {
    auto cfg = solveResolution(1.0, 50.0, 20.0, {20, 20, 10});
    auto billet = buildBillet(cfg, {0,0,0}, {20, 20, 10});

    auto& xform = billet.sdfGrid->transform();
    auto farIdx = xform.worldToIndexCellCentered({2, 2, 5});
    float before = billet.sdfGrid->getConstAccessor().getValue(farIdx);

    ToolSweepSDF tool(ToolType::BALL_END, 2.0, 0, 15, {15,15,0}, {18,15,0});
    CuttingEngine engine;
    engine.cut(billet, tool);

    float after = billet.sdfGrid->getConstAccessor().getValue(farIdx);
    EXPECT_FLOAT_EQ(before, after);
}

TEST_F(CuttingEngineTest, SingleTrack_VolumeDecreases) {
    auto cfg = solveResolution(1.0, 50.0, 20.0, {20, 20, 10});
    auto billet = buildBillet(cfg, {0,0,0}, {20, 20, 10});

    double volBefore = computeVolume(billet.sdfGrid);

    ToolSweepSDF tool(ToolType::BALL_END, 3.0, 0, 15, {5,10,0}, {15,10,0});
    CuttingEngine engine;
    engine.cut(billet, tool);

    double volAfter = computeVolume(billet.sdfGrid);
    EXPECT_LT(volAfter, volBefore);
    EXPECT_GT(volAfter, 0.0);
}

TEST_F(CuttingEngineTest, SingleTrack_MultiSegmentConsistent) {
    auto cfg = solveResolution(1.0, 50.0, 20.0, {20, 20, 10});

    // 3段切削
    auto billet1 = buildBillet(cfg, {0,0,0}, {20, 20, 10});
    CuttingEngine engine;
    engine.cut(billet1, ToolSweepSDF(ToolType::BALL_END, 2.0, 0, 15, {3,10,0}, {8,10,0}));
    engine.cut(billet1, ToolSweepSDF(ToolType::BALL_END, 2.0, 0, 15, {8,10,0}, {13,10,0}));
    engine.cut(billet1, ToolSweepSDF(ToolType::BALL_END, 2.0, 0, 15, {13,10,0}, {17,10,0}));

    // 等价长切削
    auto billet2 = buildBillet(cfg, {0,0,0}, {20, 20, 10});
    engine.cut(billet2, ToolSweepSDF(ToolType::BALL_END, 2.0, 0, 15, {3,10,0}, {17,10,0}));

    double vol1 = computeVolume(billet1.sdfGrid);
    double vol2 = computeVolume(billet2.sdfGrid);
    // 允许 1 个体素层的差异
    double tolerance = 6.0 * cfg.d_v * cfg.d_v * cfg.d_v * 1000;
    EXPECT_NEAR(vol1, vol2, tolerance);
}
