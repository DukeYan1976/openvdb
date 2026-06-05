#include <gtest/gtest.h>
#include "core/BilletBuilder.h"
#include "types/MemoryStats.h"
#include <openvdb/points/PointCount.h>
#include <chrono>

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
    ASSERT_TRUE(billet.sdfGrid != nullptr);
    EXPECT_EQ(billet.sdfGrid->getGridClass(), openvdb::GRID_LEVEL_SET);
    EXPECT_NEAR(billet.sdfGrid->voxelSize()[0], 0.5, 1e-10);
    EXPECT_TRUE(billet.microGrid == nullptr);
    EXPECT_TRUE(billet.dirtyMask == nullptr);
}

TEST_F(BilletBuilderTest, SingleTrack_SDFNegativeInside) {
    ResolutionConfig cfg;
    cfg.mode = ResolutionConfig::SINGLE_TRACK;
    cfg.d_v = 0.5;  cfg.D_v = 0.5;  cfg.N = 1;

    auto billet = buildBillet(cfg, {0,0,0}, {20, 20, 10});
    auto& xform = billet.sdfGrid->transform();
    auto idx = xform.worldToIndexCellCentered({10, 10, 5});
    EXPECT_LT(billet.sdfGrid->getConstAccessor().getValue(idx), 0.0f);
}

TEST_F(BilletBuilderTest, DualTrack_BuildProducesNoSurfels) {
    ResolutionConfig cfg;
    cfg.mode = ResolutionConfig::DUAL_TRACK;
    cfg.d_v = 0.1; cfg.D_v = 6.4; cfg.N = 64;

    auto billet = buildBillet(cfg, {0,0,0}, {30, 30, 20});

    // Lazy surfels: microGrid must be null after build
    EXPECT_TRUE(billet.microGrid == nullptr);
    // SDF exists with D_v voxel size
    ASSERT_TRUE(billet.sdfGrid != nullptr);
    EXPECT_NEAR(billet.sdfGrid->voxelSize()[0], cfg.D_v, 1e-10);
    // DirtyMask exists and shares Transform
    ASSERT_TRUE(billet.dirtyMask != nullptr);
    EXPECT_NEAR(billet.dirtyMask->voxelSize()[0], cfg.D_v, 1e-10);
    EXPECT_EQ(billet.dirtyMask->activeVoxelCount(), 0u);
}

TEST_F(BilletBuilderTest, DualTrack_GeometryDef_Set) {
    ResolutionConfig cfg;
    cfg.mode = ResolutionConfig::DUAL_TRACK;
    cfg.d_v = 0.1; cfg.D_v = 6.4; cfg.N = 64;

    auto billet = buildBillet(cfg, {1,2,3}, {30, 30, 20});
    EXPECT_EQ(billet.geometry.type, GeometryDef::BOX);
    EXPECT_DOUBLE_EQ(billet.geometry.origin.x(), 1.0);
    EXPECT_DOUBLE_EQ(billet.geometry.dims.z(), 20.0);
}

TEST_F(BilletBuilderTest, DualTrack_BuildIsFast) {
    // With lazy surfel generation, build should be very fast
    ResolutionConfig cfg;
    cfg.mode = ResolutionConfig::DUAL_TRACK;
    cfg.d_v = 0.01; cfg.D_v = 5.0; cfg.N = 500;

    auto t0 = std::chrono::high_resolution_clock::now();
    auto billet = buildBillet(cfg, {0,0,0}, {500, 500, 200});
    auto t1 = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1-t0).count();

    EXPECT_TRUE(billet.microGrid == nullptr);  // zero surfels
    EXPECT_LT(ms, 500.0);  // build in < 500ms (only SDF construction)
    printf("[BilletBuilder] DualTrack build: %.1f ms\n", ms);
}
