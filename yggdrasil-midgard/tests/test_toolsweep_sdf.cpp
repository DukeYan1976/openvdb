#include <gtest/gtest.h>
#include "core/ToolSweptSDF.h"
#include <cmath>

using namespace midgard;

// === 辅助：有限差分梯度验证 ===
static Vec3d numericalGradient(const ToolSweptSDF& sdf, const Vec3d& p, double h = 1e-7) {
    return Vec3d(
        (sdf.eval(p + Vec3d(h,0,0)) - sdf.eval(p - Vec3d(h,0,0))) / (2*h),
        (sdf.eval(p + Vec3d(0,h,0)) - sdf.eval(p - Vec3d(0,h,0))) / (2*h),
        (sdf.eval(p + Vec3d(0,0,h)) - sdf.eval(p - Vec3d(0,0,h))) / (2*h)
    );
}

static void verifyGradient(const ToolSweptSDF& sdf, const Vec3d& p, double tol = 1e-6) {
    Vec3d ag = sdf.gradient(p);
    Vec3d ng = numericalGradient(sdf, p);
    EXPECT_NEAR(ag.x(), ng.x(), tol) << "at " << p;
    EXPECT_NEAR(ag.y(), ng.y(), tol) << "at " << p;
    EXPECT_NEAR(ag.z(), ng.z(), tol) << "at " << p;
}

// ============================================================
// 球头刀测试 (Tip-Relative Model)
// 刀尖在 A->B, 球心在 A+Rz -> B+Rz
// ============================================================

class BallEndTest : public ::testing::Test {
protected:
    // 球头刀 R=5mm, 刀路沿X: (0,0,0)->(10,0,0)
    // 刀尖路径: (0,0,0)->(10,0,0)
    // 球心路径: (0,0,5)->(10,0,5)
    ToolDef tool{ToolType::BALL_END, 5.0, 0.0, 30.0};
    MoveSegment seg{Vec3d(0,0,0), Vec3d(10,0,0)};
    ToolSweptSDF sdf{tool, seg};
};

TEST_F(BallEndTest, TipPath_Inside) {
    // 刀尖路径 (5,0,0), 球心 (5,0,5), 距球心=5=R, SDF=0
    EXPECT_NEAR(sdf.eval(Vec3d(5, 0, 0)), 0.0, 1e-10);
}

TEST_F(BallEndTest, CenterPath_DeepInside) {
    // 球心路径 (5,0,5), 距球心=0, SDF = -R = -5
    EXPECT_NEAR(sdf.eval(Vec3d(5, 0, 5)), -5.0, 1e-10);
}

TEST_F(BallEndTest, OnSurface) {
    // (5, 5, 5): 距球心路径=5=R, SDF=0 (equator)
    EXPECT_NEAR(sdf.eval(Vec3d(5, 5, 5)), 0.0, 1e-10);
    // (0, 0, 0): 距起点刀尖=0, SDF=0
    EXPECT_NEAR(sdf.eval(Vec3d(0, 0, 0)), 0.0, 1e-10);
    // (10, 0, 0): 距终点刀尖=0, SDF=0
    EXPECT_NEAR(sdf.eval(Vec3d(10, 0, 0)), 0.0, 1e-10);
}

TEST_F(BallEndTest, Outside) {
    // (5, 8, 5): 距球心路径=8, SDF = 8-5 = 3
    EXPECT_NEAR(sdf.eval(Vec3d(5, 8, 5)), 3.0, 1e-10);
}

TEST_F(BallEndTest, BeyondEndCap) {
    // (15, 0, 5): 距终点球心=5=R, SDF=0
    EXPECT_NEAR(sdf.eval(Vec3d(15, 0, 5)), 0.0, 1e-10);
}

TEST_F(BallEndTest, GradientOnSurface) {
    verifyGradient(sdf, Vec3d(5, 5, 5));
    verifyGradient(sdf, Vec3d(0, 0, 0));
    verifyGradient(sdf, Vec3d(15, 0, 5));
}

TEST_F(BallEndTest, ZeroLengthPath) {
    // 静态球, 刀尖(0,0,0), 球心(0,0,5)
    MoveSegment zero{Vec3d(0,0,0), Vec3d(0,0,0)};
    ToolSweptSDF sphere(tool, zero);
    EXPECT_NEAR(sphere.eval(Vec3d(0, 0, 0)), 0.0, 1e-10); // Tip
    EXPECT_NEAR(sphere.eval(Vec3d(0, 0, 5)), -5.0, 1e-10); // Center
    EXPECT_NEAR(sphere.eval(Vec3d(5, 0, 5)), 0.0, 1e-10); // Equator
    verifyGradient(sphere, Vec3d(3, 4, 5));
}

// ============================================================
// 平底刀测试 (保持不变, 因为之前已经是对的)
// ============================================================

class FlatEndTest : public ::testing::Test {
protected:
    ToolDef tool{ToolType::FLAT_END, 3.0, 0.0, 20.0};
    MoveSegment seg{Vec3d(0,0,10), Vec3d(20,0,10)};
    ToolSweptSDF sdf{tool, seg};
};

TEST_F(FlatEndTest, CenterInside) {
    EXPECT_LT(sdf.eval(Vec3d(10, 0, 20)), 0.0);
}

TEST_F(FlatEndTest, BelowBottom) {
    EXPECT_NEAR(sdf.eval(Vec3d(10, 0, 9)), 1.0, 1e-10);
}

TEST_F(FlatEndTest, OnSideSurface) {
    EXPECT_NEAR(sdf.eval(Vec3d(10, 3, 20)), 0.0, 1e-10);
}

TEST_F(FlatEndTest, OutsideSide) {
    EXPECT_NEAR(sdf.eval(Vec3d(10, 5, 20)), 2.0, 1e-10);
}

TEST_F(FlatEndTest, OnBottomSurface) {
    EXPECT_NEAR(sdf.eval(Vec3d(10, 0, 10)), 0.0, 1e-10);
}

TEST_F(FlatEndTest, AboveTop) {
    EXPECT_NEAR(sdf.eval(Vec3d(10, 0, 31)), 1.0, 1e-10);
}

TEST_F(FlatEndTest, BottomEdge) {
    // 由于使用了 0.01mm 数值圆角，边缘处的 SDF 并非精确为 0
    EXPECT_NEAR(sdf.eval(Vec3d(10, 3, 10)), 0.0, 0.005);
}

TEST_F(FlatEndTest, CornerOutside) {
    double expected = std::sqrt(2.0);
    // 同样受数值圆角影响
    EXPECT_NEAR(sdf.eval(Vec3d(10, 4, 9)), expected, 0.005);
}

TEST_F(FlatEndTest, GradientSide) {
    verifyGradient(sdf, Vec3d(10, 5, 20));
    verifyGradient(sdf, Vec3d(5, 4, 15));
}

TEST_F(FlatEndTest, GradientBottom) {
    verifyGradient(sdf, Vec3d(10, 0, 8));
}

TEST_F(FlatEndTest, GradientCorner) {
    verifyGradient(sdf, Vec3d(10, 5, 8));
}

TEST_F(FlatEndTest, BeyondEndCap) {
    EXPECT_LT(sdf.eval(Vec3d(-2, 0, 20)), 0.0);
    EXPECT_GT(sdf.eval(Vec3d(-5, 0, 20)), 0.0);
}
