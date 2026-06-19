#include <gtest/gtest.h>
#include "core/MacroCut.h"
#include "core/ToolSweepSurface.h"
#include "core/IPWBuilder.h"
#include <cmath>

using namespace midgard;

class MacroCutPhase1Test : public ::testing::Test {
protected:
    void SetUp() override { openvdb::initialize(); }
};

TEST_F(MacroCutPhase1Test, SinglePoint_GeneratesTasks) {
    GeometryDef geom{GeometryDef::BOX, Vec3d(0), Vec3d(10,10,10)};
    ToleranceConfig config(1.0 / 30.0, ToleranceConfig::INTERACTIVE, 30.0);
    auto ipw = IPWBuilder().build(geom, config);

    ToolDef tool{ToolType::BALL_END, 3.0, 0.0, 20.0};
    MoveSegment seg{Vec3d(5,5,9), Vec3d(5,5,9)};
    ToolSweptSDF sdf(tool, seg);
    ToolSweepSurface surf(tool, seg);

    MacroCut macrocut;
    auto cls = macrocut.classifyVoxels(ipw, sdf);
    ASSERT_GT(cls.cut.size() + cls.newBoundary.size(), 0u);

    auto tasks = macrocut.buildTaskList(cls, surf, config, ipw.macroGrid->transform());

    EXPECT_EQ(tasks.size(), cls.cut.size() + cls.newBoundary.size());
    for (const auto& task : tasks) {
        EXPECT_LE(task.u_min, task.u_max);
        EXPECT_LE(task.t_min, task.t_max);
        EXPECT_GE(task.u_min, -0.1);
        EXPECT_LE(task.u_max, 1.1);
        EXPECT_GE(task.t_min, -0.1);
        EXPECT_LE(task.t_max, 1.1);
        EXPECT_LT(task.aabb.min().x(), task.aabb.max().x());
    }
}

TEST_F(MacroCutPhase1Test, SinglePoint_TaskAABBCoversVoxel) {
    GeometryDef geom{GeometryDef::BOX, Vec3d(0), Vec3d(10,10,10)};
    ToleranceConfig config(1.0 / 30.0, ToleranceConfig::INTERACTIVE, 30.0);
    auto ipw = IPWBuilder().build(geom, config);

    ToolDef tool{ToolType::BALL_END, 3.0, 0.0, 20.0};
    MoveSegment seg{Vec3d(5,5,9), Vec3d(5,5,9)};
    ToolSweptSDF sdf(tool, seg);
    ToolSweepSurface surf(tool, seg);

    MacroCut macrocut;
    auto cls = macrocut.classifyVoxels(ipw, sdf);
    auto tasks = macrocut.buildTaskList(cls, surf, config, ipw.macroGrid->transform());

    double V = config.voxelMacro;
    for (const auto& task : tasks) {
        Vec3d extent = task.aabb.max() - task.aabb.min();
        EXPECT_NEAR(extent.x(), V, 1e-10);
        EXPECT_NEAR(extent.y(), V, 1e-10);
        EXPECT_NEAR(extent.z(), V, 1e-10);
    }
}

TEST_F(MacroCutPhase1Test, SinglePoint_ParamDomainCoversVoxel) {
    GeometryDef geom{GeometryDef::BOX, Vec3d(0), Vec3d(10,10,10)};
    ToleranceConfig config(1.0 / 30.0, ToleranceConfig::INTERACTIVE, 30.0);
    auto ipw = IPWBuilder().build(geom, config);

    ToolDef tool{ToolType::BALL_END, 3.0, 0.0, 20.0};
    MoveSegment seg{Vec3d(5,5,9), Vec3d(5,5,9)};
    ToolSweptSDF sdf(tool, seg);
    ToolSweepSurface surf(tool, seg);

    MacroCut macrocut;
    auto cls = macrocut.classifyVoxels(ipw, sdf);
    auto tasks = macrocut.buildTaskList(cls, surf, config, ipw.macroGrid->transform());

    // 验证保守性: 参数域映射的3D AABB与voxel AABB应有大部分重叠
    // 少量task因margin松匹配可能不精确重叠(由Phase 2的3D拒止处理)
    int overlapping = 0;
    for (const auto& task : tasks) {
        openvdb::BBoxd surfBBox = surf.bbox(task.u_min, task.u_max,
                                             task.t_min, task.t_max);
        if (task.aabb.hasOverlap(surfBBox)) overlapping++;
    }
    // 至少70%的task应有精确覆盖（其余由Phase 2的3D拒止处理）
    double ratio = (double)overlapping / tasks.size();
    EXPECT_GT(ratio, 0.7) << "Only " << overlapping << "/" << tasks.size()
                          << " tasks have param bbox overlap";
}

TEST_F(MacroCutPhase1Test, SinglePoint_ParamDomainIsConservative) {
    GeometryDef geom{GeometryDef::BOX, Vec3d(0), Vec3d(10,10,10)};
    ToleranceConfig config(1.0 / 30.0, ToleranceConfig::INTERACTIVE, 30.0);
    auto ipw = IPWBuilder().build(geom, config);

    ToolDef tool{ToolType::BALL_END, 3.0, 0.0, 20.0};
    MoveSegment seg{Vec3d(5,5,9), Vec3d(5,5,9)};
    ToolSweptSDF sdf(tool, seg);
    ToolSweepSurface surf(tool, seg);

    MacroCut macrocut;
    auto cls = macrocut.classifyVoxels(ipw, sdf);
    auto tasks = macrocut.buildTaskList(cls, surf, config, ipw.macroGrid->transform());

    // 对每个task: 在全域密集采样找落在voxel内的参数面点
    // 验证这些点的参数(u,v)是否在task的[u_min,u_max]×[t_min,t_max]内
    int violations = 0;
    int totalChecked = 0;
    for (const auto& task : tasks) {
        for (int i = 0; i <= 50; ++i) {
            for (int j = 0; j <= 50; ++j) {
                double u = i / 50.0;
                double v = j / 50.0;
                Vec3d p = surf.eval(u, v);
                // 点是否落在这个voxel内?
                if (task.aabb.isInside(p)) {
                    totalChecked++;
                    // 参数(u,v)必须在task的参数域内
                    if (u < task.u_min || u > task.u_max ||
                        v < task.t_min || v > task.t_max) {
                        violations++;
                    }
                }
            }
        }
    }
    // 保守性要求: 0 violations (所有落在voxel内的参数面点都被参数域覆盖)
    EXPECT_EQ(violations, 0)
        << violations << " violations out of " << totalChecked << " checked points";
}
