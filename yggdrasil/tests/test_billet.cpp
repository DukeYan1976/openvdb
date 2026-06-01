#include <gtest/gtest.h>
#include "core/BilletBuilder.h"

using namespace ygg;

class BilletBuilderTest : public ::testing::Test {
protected:
    void SetUp() override { openvdb::initialize(); }
};

TEST_F(BilletBuilderTest, SingleTrack_CreatesValidLevelSet) {
    ResolutionConfig cfg;
    cfg.mode = ResolutionConfig::SINGLE_TRACK;
    cfg.d_v = 0.5;  cfg.D_v = 0.5;  cfg.N = 1;

    auto billet = buildBillet(cfg, {0,0,0}, {20, 20, 10});
    ASSERT_NE(billet.sdfGrid, nullptr);
    EXPECT_EQ(billet.sdfGrid->getGridClass(), openvdb::GRID_LEVEL_SET);
    EXPECT_NEAR(billet.sdfGrid->voxelSize()[0], 0.5, 1e-10);
}

TEST_F(BilletBuilderTest, SingleTrack_SDFNegativeInside) {
    ResolutionConfig cfg;
    cfg.mode = ResolutionConfig::SINGLE_TRACK;
    cfg.d_v = 0.5;  cfg.D_v = 0.5;  cfg.N = 1;

    auto billet = buildBillet(cfg, {0,0,0}, {20, 20, 10});
    auto accessor = billet.sdfGrid->getConstAccessor();
    auto& xform = billet.sdfGrid->transform();

    // 中心点应在内部 (SDF < 0)
    auto idx = xform.worldToIndexCellCentered({10, 10, 5});
    EXPECT_LT(accessor.getValue(idx), 0.0f);
}

TEST_F(BilletBuilderTest, SingleTrack_SDFPositiveOutside) {
    ResolutionConfig cfg;
    cfg.mode = ResolutionConfig::SINGLE_TRACK;
    cfg.d_v = 0.5;  cfg.D_v = 0.5;  cfg.N = 1;

    auto billet = buildBillet(cfg, {0,0,0}, {20, 20, 10});
    auto accessor = billet.sdfGrid->getConstAccessor();
    auto& xform = billet.sdfGrid->transform();

    // 外部点 SDF > 0
    auto idx = xform.worldToIndexCellCentered({-5, 10, 5});
    EXPECT_GT(accessor.getValue(idx), 0.0f);
}

TEST_F(BilletBuilderTest, SingleTrack_ActiveVoxelsReasonable) {
    ResolutionConfig cfg;
    cfg.mode = ResolutionConfig::SINGLE_TRACK;
    cfg.d_v = 1.0;  cfg.D_v = 1.0;  cfg.N = 1;

    auto billet = buildBillet(cfg, {0,0,0}, {10, 10, 10});
    // 10mm cube with voxelSize=1mm → ~6 faces × 100 voxels × 6 halfwidth layers
    // Should have some active voxels but not millions
    auto count = billet.sdfGrid->activeVoxelCount();
    EXPECT_GT(count, 0u);
    EXPECT_LT(count, 100000u);
}
