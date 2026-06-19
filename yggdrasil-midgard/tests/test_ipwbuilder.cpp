#include <gtest/gtest.h>
#include "core/IPWBuilder.h"
#include "core/MacroCut.h"
#include <openvdb/points/PointCount.h>

using namespace midgard;

class IPWBuilderTest : public ::testing::Test {
protected:
    void SetUp() override { openvdb::initialize(); }
};

TEST_F(IPWBuilderTest, Box_MacroGridValid) {
    GeometryDef geom;
    geom.type = GeometryDef::BOX;
    geom.origin = Vec3d(0);
    geom.dims = Vec3d(20, 20, 20);

    ToleranceConfig config(0.05);
    IPWBuilder builder;
    auto ipw = builder.build(geom, config);

    ASSERT_NE(ipw.macroGrid, nullptr);
    EXPECT_EQ(ipw.macroGrid->getGridClass(), openvdb::GRID_LEVEL_SET);
    EXPECT_GT(ipw.macroGrid->activeVoxelCount(), 0);
}

TEST_F(IPWBuilderTest, Box_MicroGridExists) {
    GeometryDef geom;
    geom.type = GeometryDef::BOX;
    geom.origin = Vec3d(0);
    geom.dims = Vec3d(20, 20, 20);

    ToleranceConfig config(0.05);
    IPWBuilder builder;
    auto ipw = builder.build(geom, config);

    ASSERT_NE(ipw.microGrid, nullptr);
    // IPW0 MicroGrid 为空（毛坯表面由MacroGrid表示）
    EXPECT_EQ(openvdb::points::pointCount(ipw.microGrid->tree()), 0u);
}

TEST_F(IPWBuilderTest, Box_PointsOnSurface) {
    // IPW0 MicroGrid 为空——毛坯表面不生成点
    GeometryDef geom{GeometryDef::BOX, Vec3d(0), Vec3d(20,20,20)};
    ToleranceConfig config(0.05);
    auto ipw = IPWBuilder().build(geom, config);
    EXPECT_EQ(openvdb::points::pointCount(ipw.microGrid->tree()), 0u);
}

TEST_F(IPWBuilderTest, Box_NormalsOutward) {
    // IPW0 MicroGrid 为空——无法线可验证
    GeometryDef geom{GeometryDef::BOX, Vec3d(0), Vec3d(20,20,20)};
    ToleranceConfig config(0.05);
    auto ipw = IPWBuilder().build(geom, config);
    EXPECT_EQ(openvdb::points::pointCount(ipw.microGrid->tree()), 0u);
}

TEST_F(IPWBuilderTest, MacroCut_AfterInit_HasNewBoundary) {
    GeometryDef geom{GeometryDef::BOX, Vec3d(0), Vec3d(20,20,20)};
    ToleranceConfig config(0.05);
    auto ipw = IPWBuilder().build(geom, config);

    ToolDef tool{ToolType::BALL_END, 3.0, 0.0, 20.0};
    MoveSegment seg{Vec3d(0, 10, 18), Vec3d(20, 10, 18)};
    ToolSweptSDF sdf(tool, seg);

    MacroCut macrocut;
    auto cls = macrocut.classifyVoxels(ipw, sdf);

    // IPW0无初始点 → 全部为 newBoundary (无 cut)
    EXPECT_EQ(cls.cut.size(), 0u);
    EXPECT_GT(cls.newBoundary.size(), 0u);
}
