#include <gtest/gtest.h>
#include "core/MicroCut.h"
#include "core/MacroCut.h"
#include "core/IPWBuilder.h"
#include "core/ToolSweepSurface.h"
#include <openvdb/points/PointCount.h>

using namespace midgard;

class Phase34Test : public ::testing::Test {
protected:
    void SetUp() override { openvdb::initialize(); }
};

TEST_F(Phase34Test, RebuildLeaves_PointCountChanges) {
    // IPW0 + 切削
    GeometryDef geom{GeometryDef::BOX, Vec3d(0), Vec3d(10, 10, 10)};
    ToleranceConfig config(1.0 / 30.0);
    auto ipw = IPWBuilder().build(geom, config);
    auto pointsBefore = openvdb::points::pointCount(ipw.microGrid->tree());

    ToolDef tool{ToolType::BALL_END, 3.0, 0.0, 20.0};
    MoveSegment seg{Vec3d(5, 5, 9), Vec3d(5, 5, 9)};
    ToolSweepSDF sdf(tool, seg);
    ToolSweepSurface surf(tool, seg);

    MacroCut macrocut;
    auto cls = macrocut.classifyVoxels(ipw, sdf);
    auto tasks = macrocut.buildTaskList(cls, surf, config, ipw.macroGrid->transform());

    MicroCut microcut;
    auto newBuffers = microcut.sampleNewSurface(tasks, surf, sdf, config);

    // Phase 3+4: 重建
    microcut.rebuildLeaves(ipw, cls, newBuffers, sdf, config);

    auto pointsAfter = openvdb::points::pointCount(ipw.microGrid->tree());

    // 切削后点数应该变化（旧点被删 + 新点加入）
    EXPECT_NE(pointsBefore, pointsAfter);
    EXPECT_GT(pointsAfter, 0u);
}

TEST_F(Phase34Test, DeletedVoxels_NoPoints) {
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
    auto newBuffers = microcut.sampleNewSurface(tasks, surf, sdf, config);
    microcut.rebuildLeaves(ipw, cls, newBuffers, sdf, config);

    // deleted voxels 不应有点
    auto acc = ipw.microGrid->getConstAccessor();
    for (const auto& coord : cls.deleted) {
        auto* leaf = acc.probeConstLeaf(coord);
        if (leaf) {
            auto off = leaf->coordToOffset(coord);
            auto end = leaf->getValue(off);
            decltype(end) start = (off == 0) ? decltype(end)(0) : leaf->getValue(off - 1);
            EXPECT_EQ(end, start) << "Deleted voxel " << coord << " still has points";
        }
    }
}

TEST_F(Phase34Test, CutVoxels_OldPointsCulled) {
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
    auto newBuffers = microcut.sampleNewSurface(tasks, surf, sdf, config);
    microcut.rebuildLeaves(ipw, cls, newBuffers, sdf, config);

    // cut voxels中剩余的点：旧点SDF≥t应保留，SDF<t应被删
    // 所有剩余点的SDF应≥t（在刀具外部）
    auto acc = ipw.microGrid->getConstAccessor();
    for (const auto& coord : cls.cut) {
        auto* leaf = acc.probeConstLeaf(coord);
        if (!leaf) continue;
        auto posHandle = openvdb::points::AttributeHandle<Vec3f>::create(
            leaf->constAttributeArray("P"));
        for (auto it = leaf->beginIndexVoxel(coord); it; ++it) {
            Vec3f pos = posHandle->get(*it);
            Vec3d wp = ipw.microGrid->transform().indexToWorld(
                coord.asVec3d() + Vec3d(pos));
            // 旧存活点应在刀具外(SDF≥t) 或是新采样点(SDF≈0)
            double s = sdf.eval(wp);
            EXPECT_GE(s, -config.user_t * 0.1)
                << "Point in cut voxel should be outside tool. SDF=" << s;
        }
    }
}
