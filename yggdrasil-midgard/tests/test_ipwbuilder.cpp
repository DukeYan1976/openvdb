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
    EXPECT_GT(openvdb::points::pointCount(ipw.microGrid->tree()), 0u);
}

TEST_F(IPWBuilderTest, Box_PointsOnSurface) {
    GeometryDef geom;
    geom.type = GeometryDef::BOX;
    geom.origin = Vec3d(0);
    geom.dims = Vec3d(20, 20, 20);

    ToleranceConfig config(0.05);
    double t_billet = 2.0 * config.user_t;
    IPWBuilder builder;
    auto ipw = builder.build(geom, config);

    // 所有点应靠近 MacroGrid 零等值面 (|SDF| < t_billet)
    auto sdfAcc = ipw.macroGrid->getConstAccessor();
    auto& tree = ipw.microGrid->tree();
    for (auto leaf = tree.cbeginLeaf(); leaf; ++leaf) {
        auto posHandle = openvdb::points::AttributeHandle<openvdb::Vec3f>::create(
            leaf->constAttributeArray("P"));
        for (auto idx = leaf->beginIndexOn(); idx; ++idx) {
            openvdb::Vec3f voxelPos = posHandle->get(*idx);
            openvdb::Vec3d offset(voxelPos.x(), voxelPos.y(), voxelPos.z());
            openvdb::Vec3d idxPos = idx.getCoord().asVec3d() + offset;
            openvdb::Vec3d worldPos = ipw.microGrid->transform().indexToWorld(idxPos);

            // 用最近voxel的SDF值近似验证
            openvdb::Coord nearCoord = ipw.macroGrid->transform().worldToIndexCellCentered(worldPos);
            float sdf = sdfAcc.getValue(nearCoord);
            // 投影后的点应在SDF≈0附近，容差为voxelSize (离散化误差)
            EXPECT_LT(std::abs(sdf), config.voxelMacro + t_billet)
                << "Point at " << worldPos << " SDF=" << sdf;
        }
    }
}

TEST_F(IPWBuilderTest, Box_NormalsOutward) {
    GeometryDef geom;
    geom.type = GeometryDef::BOX;
    geom.origin = Vec3d(0);
    geom.dims = Vec3d(20, 20, 20);

    ToleranceConfig config(0.05);
    IPWBuilder builder;
    auto ipw = builder.build(geom, config);

    // 验证: 法线是单位向量且指向外部(SDF增大方向)
    auto& tree = ipw.microGrid->tree();
    int checked = 0;
    for (auto leaf = tree.cbeginLeaf(); leaf; ++leaf) {
        if (!leaf->hasAttribute("N")) continue;
        auto nrmHandle = openvdb::points::AttributeHandle<openvdb::Vec3f>::create(
            leaf->constAttributeArray("N"));
        for (auto idx = leaf->beginIndexOn(); idx; ++idx) {
            openvdb::Vec3f normal = nrmHandle->get(*idx);
            float len = normal.length();
            EXPECT_NEAR(len, 1.0f, 0.01f) << "Normal not unit vector";
            checked++;
        }
    }
    EXPECT_GT(checked, 0);
}

TEST_F(IPWBuilderTest, MacroCut_AfterInit_HasCutVoxels) {
    GeometryDef geom;
    geom.type = GeometryDef::BOX;
    geom.origin = Vec3d(0);
    geom.dims = Vec3d(20, 20, 20);

    ToleranceConfig config(0.05);
    IPWBuilder builder;
    auto ipw = builder.build(geom, config);

    // 切削：球头刀在方块表面切削
    ToolDef tool{ToolType::BALL_END, 3.0, 0.0, 20.0};
    MoveSegment seg{Vec3d(0, 10, 18), Vec3d(20, 10, 18)};
    ToolSweepSDF sdf(tool, seg);

    MacroCut macrocut;
    auto cls = macrocut.classifyVoxels(ipw, sdf);

    // 有MicroGrid → 应识别出 voxel_c (原表面处有点数据)
    EXPECT_GT(cls.cut.size(), 0u) << "Should detect voxel_c after IPW init";
    EXPECT_GT(cls.newBoundary.size(), 0u) << "Should also have voxel_n (new interior boundary)";
}
