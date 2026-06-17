#include <gtest/gtest.h>
#include "core/MacroCut.h"
#include <openvdb/tools/MeshToVolume.h>
#include <openvdb/points/PointConversion.h>

using namespace midgard;

class MacroCutTest : public ::testing::Test {
protected:
    void SetUp() override { openvdb::initialize(); }

    openvdb::FloatGrid::Ptr makeBox(double size, double voxelSize, int halfwidth = 3) {
        openvdb::math::Transform::Ptr xform =
            openvdb::math::Transform::createLinearTransform(voxelSize);
        openvdb::math::BBox<openvdb::Vec3d> bbox(
            openvdb::Vec3d(0), openvdb::Vec3d(size));
        return openvdb::tools::createLevelSetBox<openvdb::FloatGrid>(bbox, *xform, halfwidth);
    }
};

TEST_F(MacroCutTest, NoMicroGrid_AllBoundaryIsNew) {
    auto grid = makeBox(20.0, 1.0);
    ToleranceConfig config(0.033);
    IPWState ipw(config.user_t);
    ipw.macroGrid = grid;
    // microGrid = nullptr → 所有边界都是 newBoundary

    ToolDef tool{ToolType::BALL_END, 5.0, 0.0, 30.0};
    MoveSegment seg{Vec3d(10, 10, 15), Vec3d(10, 10, 15)};
    ToolSweepSDF sdf(tool, seg);

    MacroCut macrocut;
    auto cls = macrocut.classifyVoxels(ipw, sdf);

    EXPECT_GT(cls.deleted.size(), 0u);
    EXPECT_EQ(cls.cut.size(), 0u);       // 无MicroGrid → 无cut
    EXPECT_GT(cls.newBoundary.size(), 0u);// 全部为new
}

TEST_F(MacroCutTest, NoContact_AllEmpty) {
    auto grid = makeBox(20.0, 1.0);
    ToleranceConfig config(0.033);
    IPWState ipw(config.user_t);
    ipw.macroGrid = grid;

    ToolDef tool{ToolType::BALL_END, 5.0, 0.0, 30.0};
    MoveSegment seg{Vec3d(10, 10, 30), Vec3d(10, 10, 30)};
    ToolSweepSDF sdf(tool, seg);

    MacroCut macrocut;
    auto cls = macrocut.classifyVoxels(ipw, sdf);

    EXPECT_EQ(cls.deleted.size(), 0u);
    EXPECT_EQ(cls.cut.size(), 0u);
    EXPECT_EQ(cls.newBoundary.size(), 0u);
}

TEST_F(MacroCutTest, FullPenetration_MostlyDeleted) {
    auto grid = makeBox(4.0, 1.0);
    ToleranceConfig config(0.033);
    IPWState ipw(config.user_t);
    ipw.macroGrid = grid;

    ToolDef tool{ToolType::BALL_END, 10.0, 0.0, 30.0};
    MoveSegment seg{Vec3d(2, 2, 2), Vec3d(2, 2, 2)};
    ToolSweepSDF sdf(tool, seg);

    MacroCut macrocut;
    auto cls = macrocut.classifyVoxels(ipw, sdf);

    EXPECT_GT(cls.deleted.size(), cls.cut.size() + cls.newBoundary.size());
}

TEST_F(MacroCutTest, LinearPath_BoundaryOnThreshold) {
    auto grid = makeBox(20.0, 1.0);
    ToleranceConfig config(0.033);
    IPWState ipw(config.user_t);
    ipw.macroGrid = grid;

    ToolDef tool{ToolType::BALL_END, 3.0, 0.0, 20.0};
    MoveSegment seg{Vec3d(0, 10, 18), Vec3d(20, 10, 18)};
    ToolSweepSDF sdf(tool, seg);

    MacroCut macrocut;
    auto cls = macrocut.classifyVoxels(ipw, sdf);

    EXPECT_GT(cls.deleted.size(), 0u);
    EXPECT_GT(cls.newBoundary.size(), 0u);

    // 所有边界voxel的|SDF| <= threshold
    double V = grid->voxelSize()[0];
    double threshold = V * std::sqrt(3.0) / 2.0;
    for (const auto& coord : cls.newBoundary) {
        Vec3d wp = grid->indexToWorld(coord);
        EXPECT_LE(std::abs(sdf.eval(wp)), threshold + 1e-10);
    }
}

TEST_F(MacroCutTest, CSG_ActivatesNewVoxels) {
    auto grid = makeBox(20.0, 1.0);
    int initialActive = grid->activeVoxelCount();

    ToleranceConfig config(0.033);
    IPWState ipw(config.user_t);
    ipw.macroGrid = grid;

    ToolDef tool{ToolType::BALL_END, 5.0, 0.0, 30.0};
    MoveSegment seg{Vec3d(10, 10, 15), Vec3d(10, 10, 15)};
    ToolSweepSDF sdf(tool, seg);

    MacroCut macrocut;
    auto cls = macrocut.classifyVoxels(ipw, sdf);

    EXPECT_GT(cls.newBoundary.size(), 0u);
    EXPECT_NE((int)ipw.macroGrid->activeVoxelCount(), initialActive);
}

// === 精确验证: 单点球头刀切削 ===
// 设计: Box 10×10×10mm, voxelSize=1mm, 球头刀R=3mm静态置于顶面中心
// 刀具中心(5,5,9), R=3 → 球底Z=6, 球顶Z=12
// Box顶面Z=10 → 球与顶面相交圆半径 = sqrt(R²-(10-9)²) = sqrt(9-1) = sqrt(8) ≈ 2.83mm
// 预期: 顶面附近voxel中有MicroGrid数据的(表面点)与球重叠 → cut
//       球深入内部暴露的新边界 → newBoundary  
//       球完全包住的内部voxel → deleted
#include "core/IPWBuilder.h"

TEST_F(MacroCutTest, SinglePointBallEnd_Precise) {
    // voxelSize=1mm 方便手工计算
    GeometryDef geom;
    geom.type = GeometryDef::BOX;
    geom.origin = Vec3d(0);
    geom.dims = Vec3d(10, 10, 10);

    // t=0.033 → V=max(30*0.033, 0.02)=0.99→clamp=0.99, 太小
    // 直接用K使得V=1: t=1/30=0.0333
    ToleranceConfig config(1.0/30.0);  // V_macro = 1.0mm
    ASSERT_NEAR(config.voxelMacro, 1.0, 0.01);

    IPWBuilder builder;
    auto ipw = builder.build(geom, config);
    ASSERT_NE(ipw.microGrid, nullptr);

    // 静态球头刀: 中心(5,5,9), R=3
    // 球底Z=6, 与顶面(Z=10)相交处: Z=10, 相交圆半径=sqrt(9-1)=2.83mm
    ToolDef tool{ToolType::BALL_END, 3.0, 0.0, 20.0};
    MoveSegment seg{Vec3d(5, 5, 9), Vec3d(5, 5, 9)};  // 静态
    ToolSweepSDF sdf(tool, seg);

    MacroCut macrocut;
    auto cls = macrocut.classifyVoxels(ipw, sdf);

    double V = config.voxelMacro;
    double threshold = V * std::sqrt(3.0) / 2.0;  // ≈ 0.866mm

    // --- 验证 deleted: 这些voxel中心距球心 < R - threshold ---
    // 球心(5,5,9), 完全在球内的voxel中心: dist_to_center < 3 - 0.866 = 2.134mm
    for (const auto& coord : cls.deleted) {
        Vec3d wp = ipw.macroGrid->indexToWorld(coord);
        double dist = (wp - Vec3d(5,5,9)).length();
        // SDF = dist - R, 要求 SDF < -threshold → dist < R - threshold = 2.134
        EXPECT_LT(dist, 3.0 - threshold + 1e-6)
            << "deleted voxel at " << coord << " dist=" << dist;
    }

    // --- 验证 cut+newBoundary: 这些voxel中心距球心在 [R-threshold, R+threshold] ---
    for (const auto& coord : cls.cut) {
        Vec3d wp = ipw.macroGrid->indexToWorld(coord);
        double dist = (wp - Vec3d(5,5,9)).length();
        EXPECT_GE(dist, 3.0 - threshold - 1e-6);
        EXPECT_LE(dist, 3.0 + threshold + 1e-6);
    }
    for (const auto& coord : cls.newBoundary) {
        Vec3d wp = ipw.macroGrid->indexToWorld(coord);
        double dist = (wp - Vec3d(5,5,9)).length();
        EXPECT_GE(dist, 3.0 - threshold - 1e-6);
        EXPECT_LE(dist, 3.0 + threshold + 1e-6);
    }

    // --- 验证 cut vs newBoundary 的区分 ---
    // cut: 表面voxel(Z≈10附近)与球重叠 → MicroGrid有数据
    // newBoundary: 工件内部(Z<10)与球重叠 → MicroGrid无数据
    auto microAcc = ipw.microGrid->getConstAccessor();
    for (const auto& coord : cls.cut) {
        auto* mleaf = microAcc.probeConstLeaf(coord);
        ASSERT_NE(mleaf, nullptr);
        const auto off = mleaf->coordToOffset(coord);
        const auto end = mleaf->getValue(off);
        const decltype(end) start = (off == 0) ? decltype(end)(0) : mleaf->getValue(off-1);
        EXPECT_GT(end, start) << "cut at " << coord << " should have points";
    }
    for (const auto& coord : cls.newBoundary) {
        auto* mleaf = microAcc.probeConstLeaf(coord);
        if (mleaf) {
            const auto off = mleaf->coordToOffset(coord);
            const auto end = mleaf->getValue(off);
            const decltype(end) start = (off == 0) ? decltype(end)(0) : mleaf->getValue(off-1);
            EXPECT_EQ(end, start) << "newBoundary at " << coord << " should have no points";
        }
    }

    // 三态都应存在
    EXPECT_GT(cls.deleted.size(), 0u);
    EXPECT_GT(cls.cut.size(), 0u);
    EXPECT_GT(cls.newBoundary.size(), 0u);
}
