#include <gtest/gtest.h>
#include <chrono>
#include <openvdb/openvdb.h>
#include "core/SimEngine.h"
#include "core/IPWBuilder.h"
#include "core/Types.h"

using namespace midgard;

class SimEngineTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        openvdb::initialize();
    }
    void SetUp() override {
        geom.type = GeometryDef::BOX;
        geom.origin = Vec3d(0, 0, 0);
        geom.dims = Vec3d(50, 50, 20);

        tool.type = ToolType::BALL_END;
        tool.R = 5.0;
        tool.r = 0.0;
        tool.H = 20.0;

        config = ToleranceConfig(0.05);

        IPWBuilder builder;
        ipw = builder.build(geom, config);
    }

    GeometryDef geom;
    ToolDef tool{ToolType::BALL_END, 5.0, 0.0, 20.0};
    ToleranceConfig config{0.05};
    IPWState ipw{0.05};
};

TEST_F(SimEngineTest, SingleSegmentChangesTopology) {
    int activeBefore = static_cast<int>(ipw.macroGrid->activeVoxelCount());

    MoveSegment seg{Vec3d(5, 5, 14), Vec3d(45, 5, 14), Vec3d(0, 0, 1), 0};

    SimEngine engine;
    auto result = engine.cutSegment(ipw, seg, tool, geom, config);

    EXPECT_TRUE(result.success);
    EXPECT_GT(result.deletedVoxels, 0);
    EXPECT_LT(static_cast<int>(ipw.macroGrid->activeVoxelCount()), activeBefore);
}

TEST_F(SimEngineTest, SingleSegmentProducesNewBoundary) {
    MoveSegment seg{Vec3d(5,5,14), Vec3d(45,5,14), Vec3d(0,0,1), 0};

    SimEngine engine;
    auto result = engine.cutSegment(ipw, seg, tool, geom, config);

    EXPECT_TRUE(result.success);
    EXPECT_GT(result.newBoundaryVoxels, 0);  // newBoundary voxels created
}

TEST_F(SimEngineTest, TwoConsecutiveSegments) {
    SimEngine engine;

    MoveSegment seg1{Vec3d(5, 5, 14), Vec3d(45, 5, 14), Vec3d(0, 0, 1), 0};
    auto r1 = engine.cutSegment(ipw, seg1, tool, geom, config);
    EXPECT_TRUE(r1.success);

    MoveSegment seg2{Vec3d(45, 5, 14), Vec3d(45, 45, 14), Vec3d(0, 0, 1), 0};
    auto r2 = engine.cutSegment(ipw, seg2, tool, geom, config);
    EXPECT_TRUE(r2.success);
}

TEST_F(SimEngineTest, PerformanceBound) {
    MoveSegment seg{Vec3d(5, 5, 14), Vec3d(45, 5, 14), Vec3d(0, 0, 1), 0};

    SimEngine engine;
    auto start = std::chrono::high_resolution_clock::now();
    engine.cutSegment(ipw, seg, tool, geom, config);
    auto elapsed = std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - start).count();

    EXPECT_LT(elapsed, 15000.0) << "Single segment took " << elapsed << "ms";
}
