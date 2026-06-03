#include <gtest/gtest.h>
#include "core/BilletBuilder.h"
#include "core/ToolSweepSDF.h"
#include "core/CuttingEngine.h"
#include "types/MemoryStats.h"
#include <openvdb/points/PointCount.h>
#include <openvdb/points/PointAttribute.h>

using namespace ygg;

class DualGridCutTest : public ::testing::Test {
protected:
    void SetUp() override { openvdb::initialize(); }

    BilletModel makeDualBillet() {
        ResolutionConfig cfg;
        cfg.mode = ResolutionConfig::DUAL_TRACK;
        cfg.d_v = 0.5; cfg.D_v = 4.0; cfg.N = 8;
        return buildBillet(cfg, {0,0,0}, {30, 30, 20});
    }
};

TEST_F(DualGridCutTest, MicroCut_SurfelsDeactivated) {
    auto billet = makeDualBillet();
    ASSERT_NE(billet.microGrid, nullptr);

    // Count active before
    size_t activeBefore = 0;
    for (auto leaf = billet.microGrid->tree().beginLeaf(); leaf; ++leaf) {
        auto& attrSet = leaf->attributeSet();
        auto* arr = attrSet.get("active");
        auto rh = openvdb::points::AttributeHandle<uint8_t>::create(*arr);
        for (size_t i = 0; i < rh->size(); ++i)
            if (rh->get(i) == 1) activeBefore++;
    }
    ASSERT_GT(activeBefore, 0u);

    // Cut: ball-end R=5 through top surface (Z=20, sphere center at Z=22)
    ToolSweepSDF tool(ToolType::BALL_END, 5.0, 0, 30, {15,15,22}, {15,15,22});
    CuttingEngine engine;
    engine.cut(billet, tool);

    // Count active after
    size_t activeAfter = 0;
    for (auto leaf = billet.microGrid->tree().beginLeaf(); leaf; ++leaf) {
        auto& attrSet = leaf->attributeSet();
        auto* arr = attrSet.get("active");
        auto rh = openvdb::points::AttributeHandle<uint8_t>::create(*arr);
        for (size_t i = 0; i < rh->size(); ++i)
            if (rh->get(i) == 1) activeAfter++;
    }

    EXPECT_LT(activeAfter, activeBefore);
}

TEST_F(DualGridCutTest, MacroSDF_UpdatedAfterCut) {
    auto billet = makeDualBillet();

    // Before cut: center top voxel SDF should be ≤ 0
    auto& xform = billet.sdfGrid->transform();
    auto idx = xform.worldToIndexCellCentered({15, 15, 20});
    float sdfBefore = billet.sdfGrid->getConstAccessor().getValue(idx);
    EXPECT_LE(sdfBefore, 0.0f);

    // Deep cut through center
    ToolSweepSDF tool(ToolType::BALL_END, 5.0, 0, 30, {5,15,22}, {25,15,22});
    CuttingEngine engine;
    engine.cut(billet, tool);

    // After cut: SDF at center should become positive (air)
    float sdfAfter = billet.sdfGrid->getConstAccessor().getValue(idx);
    EXPECT_GT(sdfAfter, 0.0f);
}

TEST_F(DualGridCutTest, MemoryGrowth_Monitored) {
    auto billet = makeDualBillet();

    MemoryStats before, after;
    before.update(billet.sdfGrid, billet.microGrid);

    ToolSweepSDF tool(ToolType::BALL_END, 5.0, 0, 30, {5,15,22}, {25,15,22});
    CuttingEngine engine;
    engine.cut(billet, tool);

    after.update(billet.sdfGrid, billet.microGrid);

    printf("Memory before: %.2f MB, after: %.2f MB, delta: %.2f MB\n",
           before.totalBytes/1e6, after.totalBytes/1e6,
           (after.totalBytes - before.totalBytes)/1e6);

    // FloatGrid memory should not grow significantly
    EXPECT_NEAR(static_cast<double>(after.floatGridBytes),
                static_cast<double>(before.floatGridBytes),
                before.floatGridBytes * 0.2);
}
