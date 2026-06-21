#include <gtest/gtest.h>
#include "core/MicroCut.h"
#include "core/IPWBuilder.h"
#include "core/LocalSurfaceEngine.h"
#include "core/ToolSweptSDF.h"
#include "core/ToolSweepSurface.h"
#include <openvdb/openvdb.h>

using namespace midgard;

class MicroCutRobustTest : public ::testing::Test {
protected:
    void SetUp() override {
        openvdb::initialize();
    }
};

// CASE 1: 动态毛坯边界缝合 (Billet Boundary)
// 验证 Phase 1.5 补全点云后，Phase 2 能够识别“材料内部”并生成新点
TEST_F(MicroCutRobustTest, CASE1_BilletBoundaryStitching) {
    ToleranceConfig config(0.05, ToleranceConfig::INTERACTIVE, 10.0); // V=0.5
    
    // 1. 模拟毛坯: Z=9.75 以下为金属 (为了让体素中心[9.75]正好在边界上)
    // 注意：当前 IPWBuilder 只能做简单的 BOX，我们通过 Voxel 范围来模拟
    GeometryDef geom{GeometryDef::BOX, Vec3d(0,0,0), Vec3d(20,20,9.75)};
    auto ipw = IPWBuilder().build(geom, config);
    
    // 2. 目标体素: [10, 10.5] x [10, 10.5] x [9.5, 10]
    openvdb::Coord origin(20, 20, 19); // 0.5mm voxel size -> world [10, 10, 9.5]
    VoxelTask task;
    task.origin = origin;
    task.aabb = openvdb::BBoxd(Vec3d(10,10,9.5), Vec3d(10.5,10.5,10));
    task.classification = VoxelClass::NEW_BOUNDARY;
    
    // 3. 刀具: 球头刀 R=2, 刀尖在 (10.25, 10.25, 9.25)
    // 刀尖在毛坯(Z=9.75)下方 0.5mm，属于有效切削
    ToolDef tool{ToolType::BALL_END, 2.0, 0.0, 10.0};
    MoveSegment seg{Vec3d(10.25, 10.25, 9.25), Vec3d(10.25, 10.25, 9.25)};
    ToolSweptSDF sdf(tool, seg);
    ToolSweepSurface surface(tool, seg);
    
    MicroCut microcut;
    
    // ---------------------------------------------------------
    // 执行 Phase 1.5: 动态补全 (显式调用)
    // ---------------------------------------------------------
    std::vector<VoxelTask> tasks = { task };
    // TODO: billetDef removed from IPWState
    std::unordered_map<openvdb::Coord, PointBuffer> primedBuffers;
    ASSERT_TRUE(primedBuffers.count(origin) > 0) << "Phase 1.5 should prime the billet boundary";

    // 验证采样点的准确性 (解析采样)
    bool foundSurface = false;
    for (const auto& p : primedBuffers[origin].positions) {
        if (std::abs(p.z() - 9.75) < 1e-6) foundSurface = true;
    }
    EXPECT_TRUE(foundSurface) << "Should find a point exactly on the Z=9.75 plane";

    // ---------------------------------------------------------
    // 执行 Phase 2: 采样 (目前尚未修改 sampleNewSurface 接口)
    // ---------------------------------------------------------
    auto buffers = microcut.sampleNewSurface(tasks, surface, sdf, config, ipw);
    // ...
    }

// CASE 2: 特征脊线保护 (Convex Ridge Protection)
// 验证 LocalSurfaceEngine 的 max(dot) 逻辑能够阻止在旧脊线外的空气中采样
TEST_F(MicroCutRobustTest, CASE2_ConvexRidgeProtection) {
    ToleranceConfig config(0.05, ToleranceConfig::INTERACTIVE, 10.0);
    
    // 模拟一个已有凸角的体素点云 (X=10, Y=10 为空气侧)
    // P1: (10, 10.25, 10), N=(1, 0, 0) -> 指向 X+ 空气
    // P2: (10.25, 10, 10), N=(0, 1, 0) -> 指向 Y+ 空气
    PointBuffer existing;
    existing.positions.push_back(Vec3f(10.0f, 10.25f, 10.0f));
    existing.normals.push_back(Vec3f(1.0f, 0.0f, 0.0f));
    existing.positions.push_back(Vec3f(10.25f, 10.0f, 10.0f));
    existing.normals.push_back(Vec3f(0.0f, 1.0f, 0.0f));
    
    openvdb::BBoxd voxelBox(Vec3d(10,10,10), Vec3d(10.5,10.5,10.5));
    
    // ---------------------------------------------------------
    // 补充断言: 直接验证 LocalSurfaceEngine 对空气侧点返回正值
    // ---------------------------------------------------------
    {
        LocalSurfaceEngine engine(existing.positions, existing.normals, voxelBox);
        // 凸脊空气侧测试点
        EXPECT_GT(engine.eval(Vec3d(10.4, 10.4, 10.0)), 0.0)
            << "Air-side point must have positive SDF";
        EXPECT_GT(engine.eval(Vec3d(10.3, 10.3, 10.25)), 0.0)
            << "Corner air-side point must have positive SDF";
        // 材料内部测试点 (X<10 且 Y<10 方向)
        EXPECT_LT(engine.eval(Vec3d(9.8, 9.8, 10.0)), 0.0)
            << "Material-side point must have negative SDF";
    }
    
    // ---------------------------------------------------------
    // 集成断言: quadtreeEval 端到端拒绝空气侧采样
    // ---------------------------------------------------------
    ToolDef tool{ToolType::BALL_END, 5.0, 0.0, 20.0};
    MoveSegment seg{Vec3d(10.4, 10.4, 5.0), Vec3d(10.4, 10.4, 5.0)}; // 刀尖在Z=5, R=5 -> 表面在Z=10
    ToolSweepSurface surface(tool, seg);
    
    PointBuffer output;
    MicroCut microcut;
    
    microcut.quadtreeEval(0, 1, 0, 1, voxelBox, surface, config.chordalLimit, 0, output, 
                          nullptr, nullptr, &existing, config.user_t);
    
    EXPECT_EQ(output.positions.size(), 0u) << "Should NOT sample in the air side of a convex ridge";
}

// CASE 3: 解析毛坯平面简化 (Planar Billet)
// 验证针对简单平面不进行过度采样
TEST_F(MicroCutRobustTest, CASE3_PlanarBilletOptimization) {
    ToleranceConfig config(0.05, ToleranceConfig::INTERACTIVE, 10.0);
    
    // 毛坯: 一个大长方体, 顶面在 Z=10
    GeometryDef geom{GeometryDef::BOX, Vec3d(0,0,0), Vec3d(100,100,10)};
    
    // 体素: 跨越顶面
    // 体素中心 (5.25, 5.25, 10.0), AABB: [5, 10.0] -> [5.5, 10.5]
    openvdb::BBoxd region(Vec3d(5, 5, 9.75), Vec3d(5.5, 5.5, 10.25));
    
    // 直接调用 IPWBuilder 的标准边界采样服务
    auto buf = IPWBuilder::sampleBoundary(geom, region);
    
    // 预期: 因为只与顶面 (Z=10) 相交，应该只生成 1 个解析点
    ASSERT_EQ(buf.positions.size(), 1u) << "Should generate exactly 1 point for a flat analytical face within the voxel";
    
    // 验证点的坐标: Z必须完美等于10，XY应该是体素在相交面的中心
    EXPECT_NEAR(buf.positions[0].x(), 5.25, 1e-10);
    EXPECT_NEAR(buf.positions[0].y(), 5.25, 1e-10);
    EXPECT_NEAR(buf.positions[0].z(), 10.0, 1e-10);
    
    // 验证法向: 顶面法向应指向外 (0,0,1)
    EXPECT_NEAR(buf.normals[0].z(), 1.0, 1e-10);
}
