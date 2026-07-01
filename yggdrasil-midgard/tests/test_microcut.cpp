#include <gtest/gtest.h>
#include "core/MicroCut.h"
#include "core/MacroCut.h"
#include "core/IPWBuilder.h"
#include "core/ToolSweepSurface.h"
#include <cmath>

using namespace midgard;

class MicroCutTest : public ::testing::Test {
protected:
    void SetUp() override { openvdb::initialize(); }
};

TEST_F(MicroCutTest, SinglePoint_AllPointsOnSDF_Zero) {
    // 构建IPW + 切削
    GeometryDef geom{GeometryDef::BOX, Vec3d(0), Vec3d(10, 10, 10)};
    ToleranceConfig config(1.0 / 30.0, ToleranceConfig::INTERACTIVE, 30.0);  // V=1mm, t=0.033mm
    auto ipw = IPWBuilder().build(geom, config);

    ToolDef tool{ToolType::BALL_END, 3.0, 0.0, 20.0};
    MoveSegment seg{Vec3d(5, 5, 9), Vec3d(5, 5, 9)};
    ToolSweptSDF sdf(tool, seg);
    ToolSweepSurface surf(tool, seg);

    // Phase 0 + 1
    MacroCut macrocut;
    auto cls = macrocut.classifyVoxels(ipw, sdf);
    auto tasks = macrocut.buildTaskList(cls, surf, config, ipw.macroGrid->transform());

    // Phase 2: 采样
    MicroCut microcut;
    auto buffers = microcut.sampleNewSurface(tasks, surf, sdf, config, ipw, geom);

    // 验证: 所有采样点的SDF值应≈0
    double maxAbsSDF = 0;
    int totalPoints = 0;
    for (const auto& [coord, buf] : buffers) {
        for (const auto& pos : buf.positions) {
            double s = std::abs(sdf.eval(Vec3d(pos)));
            maxAbsSDF = std::max(maxAbsSDF, s);
            totalPoints++;
        }
    }
    EXPECT_GT(totalPoints, 0) << "Should generate points";
    // 所有点精确在扫掠面上（误差仅为float存储精度 ≈ R*2^-23 ≈ 1e-6）
    EXPECT_LT(maxAbsSDF, 1e-5)
        << "All points must be on sweep surface. Max |SDF| = " << maxAbsSDF;
}

TEST_F(MicroCutTest, SinglePoint_PointsInsideVoxel) {
    GeometryDef geom{GeometryDef::BOX, Vec3d(0), Vec3d(10, 10, 10)};
    ToleranceConfig config(1.0 / 30.0, ToleranceConfig::INTERACTIVE, 30.0);
    auto ipw = IPWBuilder().build(geom, config);

    ToolDef tool{ToolType::BALL_END, 3.0, 0.0, 20.0};
    MoveSegment seg{Vec3d(5, 5, 9), Vec3d(5, 5, 9)};
    ToolSweptSDF sdf(tool, seg);
    ToolSweepSurface surf(tool, seg);

    MacroCut macrocut;
    auto cls = macrocut.classifyVoxels(ipw, sdf);
    auto tasks = macrocut.buildTaskList(cls, surf, config, ipw.macroGrid->transform());

    MicroCut microcut;
    auto buffers = microcut.sampleNewSurface(tasks, surf, sdf, config, ipw, geom);

    // 验证: 每个buffer的点都在对应task的voxel AABB附近
    // 四叉树精确采样的点在AABB内(浮点误差)
    // SDF投影fallback点可能在相邻体素(1V范围内)
    for (const auto& task : tasks) {
        auto it = buffers.find(task.origin);
        if (it == buffers.end()) continue;
        for (const auto& pos : it->second.positions) {
            openvdb::BBoxd expandedBox = task.aabb;
            expandedBox.expand(config.voxelMacro);
            EXPECT_TRUE(expandedBox.isInside(pos))
                << "Point " << pos << " too far from voxel at " << task.origin;
        }
    }
}

TEST_F(MicroCutTest, SinglePoint_NormalsAreUnit) {
    GeometryDef geom{GeometryDef::BOX, Vec3d(0), Vec3d(10, 10, 10)};
    ToleranceConfig config(1.0 / 30.0, ToleranceConfig::INTERACTIVE, 30.0);
    auto ipw = IPWBuilder().build(geom, config);

    ToolDef tool{ToolType::BALL_END, 3.0, 0.0, 20.0};
    MoveSegment seg{Vec3d(5, 5, 9), Vec3d(5, 5, 9)};
    ToolSweptSDF sdf(tool, seg);
    ToolSweepSurface surf(tool, seg);

    MacroCut macrocut;
    auto cls = macrocut.classifyVoxels(ipw, sdf);
    auto tasks = macrocut.buildTaskList(cls, surf, config, ipw.macroGrid->transform());

    MicroCut microcut;
    auto buffers = microcut.sampleNewSurface(tasks, surf, sdf, config, ipw, geom);

    for (const auto& [coord, buf] : buffers) {
        for (const auto& n : buf.normals) {
            double len = Vec3d(n).length();
            EXPECT_NEAR(len, 1.0, 0.01);
        }
    }
}
