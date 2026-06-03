#include <gtest/gtest.h>
#include "core/BilletBuilder.h"
#include "types/MemoryStats.h"
#include <openvdb/points/PointCount.h>

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

// ===== T6.4: DualGrid Integration Tests =====

TEST_F(BilletBuilderTest, DualGrid_CreatePointDataGrid_SharedTransform) {
    ResolutionConfig cfg;
    cfg.mode = ResolutionConfig::DUAL_TRACK;
    cfg.d_v = 0.1; cfg.D_v = 6.4; cfg.N = 64;

    auto billet = buildBillet(cfg, {0,0,0}, {30, 30, 20});
    ASSERT_NE(billet.sdfGrid, nullptr);
    ASSERT_NE(billet.microGrid, nullptr);
    EXPECT_TRUE(billet.isDualTrack());

    // 共享 Transform 验证
    EXPECT_DOUBLE_EQ(billet.sdfGrid->voxelSize()[0], cfg.D_v);
    EXPECT_DOUBLE_EQ(billet.microGrid->voxelSize()[0], cfg.D_v);
}

TEST_F(BilletBuilderTest, DualGrid_IPW0_SurfelsInjected) {
    ResolutionConfig cfg;
    cfg.mode = ResolutionConfig::DUAL_TRACK;
    cfg.d_v = 0.1; cfg.D_v = 6.4; cfg.N = 64;

    auto billet = buildBillet(cfg, {0,0,0}, {30, 30, 20});

    size_t ptCount = openvdb::points::pointCount(billet.microGrid->tree());
    EXPECT_GT(ptCount, 0u);

    // 粗精度：平均每叶节点面元不多
    size_t leafCount = billet.microGrid->tree().leafCount();
    ASSERT_GT(leafCount, 0u);
    double avgPerLeaf = static_cast<double>(ptCount) / leafCount;
    EXPECT_LT(avgPerLeaf, 2000.0);  // 粗精度 N_init²=4 per voxel, ~240 voxels/leaf → ~960
}

TEST_F(BilletBuilderTest, DualGrid_MemoryStats_Reported) {
    ResolutionConfig cfg;
    cfg.mode = ResolutionConfig::DUAL_TRACK;
    cfg.d_v = 0.1; cfg.D_v = 6.4; cfg.N = 64;

    auto billet = buildBillet(cfg, {0,0,0}, {30, 30, 20});

    ygg::MemoryStats stats;
    stats.update(billet.sdfGrid, billet.microGrid);

    EXPECT_GT(stats.floatGridBytes, 0u);
    EXPECT_GT(stats.pointGridBytes, 0u);
    EXPECT_GT(stats.pointCount, 0u);
    EXPECT_EQ(stats.activePointCount, stats.pointCount); // 初始全活跃
    EXPECT_GT(stats.leafNodeCount, 0u);
    stats.print();
}
