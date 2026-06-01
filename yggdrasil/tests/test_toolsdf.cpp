#include <gtest/gtest.h>
#include "core/ToolSweepSDF.h"
#include <cmath>

using namespace ygg;

TEST(ToolSweepSDF, BallEnd_StaticSphere) {
    ToolSweepSDF tool(ToolType::BALL_END, 5.0, 0, 20, {0,0,0}, {0,0,0});
    EXPECT_NEAR(tool.eval({5, 0, 0}), 0.0, 1e-10);
    EXPECT_NEAR(tool.eval({0, 5, 0}), 0.0, 1e-10);
    EXPECT_NEAR(tool.eval({0, 0, 5}), 0.0, 1e-10);
    EXPECT_LT(tool.eval({0, 0, 0}), 0.0);
    EXPECT_GT(tool.eval({10, 0, 0}), 0.0);
}

TEST(ToolSweepSDF, BallEnd_Capsule) {
    ToolSweepSDF tool(ToolType::BALL_END, 5.0, 0, 20, {0,0,0}, {20,0,0});
    // 中点处球面
    EXPECT_NEAR(tool.eval({10, 5, 0}), 0.0, 1e-10);
    // 起点球面
    EXPECT_NEAR(tool.eval({-5, 0, 0}), 0.0, 1e-10);
    // 终点球面
    EXPECT_NEAR(tool.eval({25, 0, 0}), 0.0, 1e-10);
    // 内部
    EXPECT_LT(tool.eval({10, 0, 0}), 0.0);
    EXPECT_NEAR(tool.eval({10, 0, 0}), -5.0, 1e-10);
}

TEST(ToolSweepSDF, BallEnd_BoundingBox) {
    ToolSweepSDF tool(ToolType::BALL_END, 5.0, 0, 20, {0,0,0}, {20,0,0});
    auto bbox = tool.getBoundingBox();
    EXPECT_LE(bbox.min().x(), -5.0);
    EXPECT_GE(bbox.max().x(), 25.0);
    EXPECT_LE(bbox.min().y(), -5.0);
    EXPECT_GE(bbox.max().y(), 5.0);
}

TEST(ToolSweepSDF, FlatEnd_StaticCylinder) {
    // 平底刀静止：圆柱体 R=5, H=20, 底面在 z=0
    ToolSweepSDF tool(ToolType::FLAT_END, 5.0, 0, 20, {0,0,0}, {0,0,0});
    // 底面圆周上（表面）
    EXPECT_NEAR(tool.eval({5, 0, 0}), 0.0, 1e-10);
    // 内部（圆柱中心，z=10）
    EXPECT_LT(tool.eval({0, 0, 10}), 0.0);
    // 底面下方（外部）
    EXPECT_GT(tool.eval({0, 0, -1}), 0.0);
    // 侧面外部
    EXPECT_GT(tool.eval({10, 0, 10}), 0.0);
}

TEST(ToolSweepSDF, FlatEnd_Sweep) {
    // 平底刀从(0,0,0)到(10,0,0)
    ToolSweepSDF tool(ToolType::FLAT_END, 3.0, 0, 15, {0,0,0}, {10,0,0});
    // 路径中点，圆柱面上
    EXPECT_NEAR(tool.eval({5, 3, 0}), 0.0, 1e-10);
    // 内部
    EXPECT_LT(tool.eval({5, 0, 5}), 0.0);
}

TEST(ToolSweepSDF, Gradient_BallEnd) {
    ToolSweepSDF tool(ToolType::BALL_END, 5.0, 0, 20, {0,0,0}, {20,0,0});
    auto grad = tool.gradient({10, 5, 0});
    double len = std::sqrt(grad.x()*grad.x() + grad.y()*grad.y() + grad.z()*grad.z());
    // 法向量应指向 +Y
    EXPECT_NEAR(grad.y() / len, 1.0, 0.01);
}
