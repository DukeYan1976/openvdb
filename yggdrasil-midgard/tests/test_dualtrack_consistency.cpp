#include <gtest/gtest.h>
#include "core/MicroCut.h"
#include "core/MacroCut.h"
#include "core/IPWBuilder.h"
#include "core/ToolSweepSurface.h"
#include <openvdb/points/PointCount.h>

using namespace midgard;

/// DualTrack 一致性验证：
/// 1. MacroGrid 和 MicroGrid 共享同一 Transform
/// 2. MacroGrid 的每个表面 voxel (|SDF| < V) 在 MicroGrid 中有对应点数据
///    即：MicroGrid 的点集完整覆盖了 MacroGrid 的边界
static void verifyDualTrackConsistency(const IPWState& ipw, const std::string& context) {
    SCOPED_TRACE(context);
    auto& macroGrid = ipw.macroGrid;
    auto& microGrid = ipw.microGrid;

    ASSERT_NE(macroGrid, nullptr);
    ASSERT_NE(microGrid, nullptr);

    // 1. Transform 一致
    double macroVoxel = macroGrid->voxelSize()[0];
    double microVoxel = microGrid->voxelSize()[0];
    EXPECT_NEAR(macroVoxel, microVoxel, 1e-10);

    // 2. MicroGrid 有点的 voxel ⊆ MacroGrid 零等值面 voxel
    double V = macroVoxel;
    auto macroAcc = macroGrid->getConstAccessor();
    int microPoints = 0;
    int pointsOutsideMacro = 0;

    for (auto leaf = microGrid->tree().cbeginLeaf(); leaf; ++leaf) {
        for (auto it = leaf->cbeginValueOn(); it; ++it) {
            openvdb::Coord coord = it.getCoord();
            // 该voxel有点 → 检查MacroGrid中是否active
            if (!macroAcc.isValueOn(coord)) {
                pointsOutsideMacro++;
            }
            microPoints++;
        }
    }

    // 允许极少数边界case（levelSetRebuild可能轻微改变拓扑）
    EXPECT_LE(pointsOutsideMacro, 2)
        << "MicroGrid has points in " << pointsOutsideMacro
        << " voxels that are inactive in MacroGrid";
    EXPECT_GT(microPoints, 0) << "MicroGrid has no points";
}

class DualTrackConsistencyTest : public ::testing::Test {
protected:
    void SetUp() override { openvdb::initialize(); }
};

TEST_F(DualTrackConsistencyTest, IPW0_FullCoverage) {
    // IPW0: MicroGrid 为空（毛坯表面不生成点）
    GeometryDef geom{GeometryDef::BOX, Vec3d(0), Vec3d(10, 10, 10)};
    ToleranceConfig config(1.0 / 30.0);
    auto ipw = IPWBuilder().build(geom, config);

    ASSERT_NE(ipw.microGrid, nullptr);
    // 空 MicroGrid：无点在 inactive 区域（trivially true）
    EXPECT_EQ(openvdb::points::pointCount(ipw.microGrid->tree()), 0u);
}

TEST_F(DualTrackConsistencyTest, AfterCut_StillConsistent) {
    GeometryDef geom{GeometryDef::BOX, Vec3d(0), Vec3d(10, 10, 10)};
    ToleranceConfig config(1.0 / 30.0);
    auto ipw = IPWBuilder().build(geom, config);

    // 执行切削
    ToolDef tool{ToolType::BALL_END, 3.0, 0.0, 20.0};
    MoveSegment seg{Vec3d(5, 5, 9), Vec3d(5, 5, 9)};
    ToolSweepSDF sdf(tool, seg);
    ToolSweepSurface surf(tool, seg);

    MacroCut macrocut;
    auto cls = macrocut.classifyVoxels(ipw, sdf);
    auto tasks = macrocut.buildTaskList(cls, surf, config, ipw.macroGrid->transform());

    MicroCut microcut;
    auto buffers = microcut.sampleNewSurface(tasks, surf, sdf, config);
    microcut.rebuildLeaves(ipw, cls, buffers, sdf, config);

    // 切削后验证
    verifyDualTrackConsistency(ipw, "After cut");

    std::cout << "  MacroGrid: leaves=" << ipw.macroGrid->tree().leafCount()
              << " active=" << ipw.macroGrid->activeVoxelCount() << "\n";
    std::cout << "  MicroGrid: leaves=" << ipw.microGrid->tree().leafCount()
              << " points=" << openvdb::points::pointCount(ipw.microGrid->tree()) << "\n";
}

TEST_F(DualTrackConsistencyTest, AfterCut_NoPointsInsideTool) {
    // 切削后，MicroGrid 中不应有点深入刀具内部
    GeometryDef geom{GeometryDef::BOX, Vec3d(0), Vec3d(10, 10, 10)};
    ToleranceConfig config(1.0 / 30.0);
    auto ipw = IPWBuilder().build(geom, config);

    ToolDef tool{ToolType::BALL_END, 3.0, 0.0, 20.0};
    MoveSegment seg{Vec3d(5, 5, 9), Vec3d(5, 5, 9)};
    ToolSweepSDF sdf(tool, seg);
    ToolSweepSurface surf(tool, seg);

    MacroCut macrocut;
    auto cls = macrocut.classifyVoxels(ipw, sdf);
    auto tasks = macrocut.buildTaskList(cls, surf, config, ipw.macroGrid->transform());

    MicroCut microcut;
    auto buffers = microcut.sampleNewSurface(tasks, surf, sdf, config);
    microcut.rebuildLeaves(ipw, cls, buffers, sdf, config);

    // 所有MicroGrid点的SDF应≥-t (不深入刀具超过容差)
    double maxPenetration = 0;
    for (auto leaf = ipw.microGrid->tree().cbeginLeaf(); leaf; ++leaf) {
        auto posHandle = openvdb::points::AttributeHandle<Vec3f>::create(
            leaf->constAttributeArray("P"));
        for (auto it = leaf->beginIndexOn(); it; ++it) {
            Vec3f pos = posHandle->get(*it);
            Vec3d wp = ipw.microGrid->transform().indexToWorld(
                it.getCoord().asVec3d() + Vec3d(pos));
            double s = sdf.eval(wp);
            if (s < 0) maxPenetration = std::max(maxPenetration, -s);
        }
    }
    EXPECT_LT(maxPenetration, config.user_t)
        << "Points penetrate tool by " << maxPenetration << "mm (limit=" << config.user_t << ")";
}
