#include <gtest/gtest.h>
#include "core/ToolSweepSDF.h"
#include <cmath>

using namespace midgard;

// === 辅助：有限差分梯度验证 ===
static Vec3d numericalGradient(const ToolSweepSDF& sdf, const Vec3d& p, double h = 1e-7) {
    return Vec3d(
        (sdf.eval(p + Vec3d(h,0,0)) - sdf.eval(p - Vec3d(h,0,0))) / (2*h),
        (sdf.eval(p + Vec3d(0,h,0)) - sdf.eval(p - Vec3d(0,h,0))) / (2*h),
        (sdf.eval(p + Vec3d(0,0,h)) - sdf.eval(p - Vec3d(0,0,h))) / (2*h)
    );
}

static void verifyGradient(const ToolSweepSDF& sdf, const Vec3d& p, double tol = 1e-6) {
    Vec3d ag = sdf.gradient(p);
    Vec3d ng = numericalGradient(sdf, p);
    EXPECT_NEAR(ag.x(), ng.x(), tol) << "at " << p;
    EXPECT_NEAR(ag.y(), ng.y(), tol) << "at " << p;
    EXPECT_NEAR(ag.z(), ng.z(), tol) << "at " << p;
}

// ============================================================
// 球头刀测试
// 扫掠体 = 胶囊体: 球心从A沿AB移动到B，半径R
// SDF(p) = dist(p, segment(A,B)) - R
// ============================================================

class BallEndTest : public ::testing::Test {
protected:
    // 球头刀 R=5mm, 刀路沿X: (0,0,0)->(10,0,0)
    ToolDef tool{ToolType::BALL_END, 5.0, 0.0, 30.0};
    MoveSegment seg{Vec3d(0,0,0), Vec3d(10,0,0)};
    ToolSweepSDF sdf{tool, seg};
};

TEST_F(BallEndTest, CenterOfSegment_Inside) {
    // 线段中点 (5,0,0), 距线段=0, SDF = -R = -5
    EXPECT_NEAR(sdf.eval(Vec3d(5, 0, 0)), -5.0, 1e-10);
}

TEST_F(BallEndTest, OnSurface) {
    // (5, 5, 0): 距线段中点=5=R, SDF=0
    EXPECT_NEAR(sdf.eval(Vec3d(5, 5, 0)), 0.0, 1e-10);
    // (0, 0, 5): 距起点=5=R, SDF=0
    EXPECT_NEAR(sdf.eval(Vec3d(0, 0, 5)), 0.0, 1e-10);
}

TEST_F(BallEndTest, Outside) {
    // (5, 8, 0): 距线段=8, SDF = 8-5 = 3
    EXPECT_NEAR(sdf.eval(Vec3d(5, 8, 0)), 3.0, 1e-10);
}

TEST_F(BallEndTest, BeyondEndCap) {
    // (-3, 0, 0): 距起点=3, SDF = 3-5 = -2 (inside)
    EXPECT_NEAR(sdf.eval(Vec3d(-3, 0, 0)), -2.0, 1e-10);
    // (15, 0, 0): 距终点=5, SDF = 5-5 = 0 (on surface)
    EXPECT_NEAR(sdf.eval(Vec3d(15, 0, 0)), 0.0, 1e-10);
}

TEST_F(BallEndTest, GradientOnSurface) {
    verifyGradient(sdf, Vec3d(5, 5, 0));
    verifyGradient(sdf, Vec3d(0, 0, 5));
    verifyGradient(sdf, Vec3d(15, 0, 0));
}

TEST_F(BallEndTest, GradientInside) {
    verifyGradient(sdf, Vec3d(5, 2, 0));
    verifyGradient(sdf, Vec3d(3, 0, 1));
}

TEST_F(BallEndTest, GradientOutside) {
    verifyGradient(sdf, Vec3d(5, 8, 0));
    verifyGradient(sdf, Vec3d(-5, 3, 4));
}

TEST_F(BallEndTest, ZeroLengthPath) {
    // 退化: start==end → 静态球, SDF = |p - center| - R
    MoveSegment zero{Vec3d(0,0,0), Vec3d(0,0,0)};
    ToolSweepSDF sphere(tool, zero);
    EXPECT_NEAR(sphere.eval(Vec3d(5, 0, 0)), 0.0, 1e-10);
    EXPECT_NEAR(sphere.eval(Vec3d(0, 0, 0)), -5.0, 1e-10);
    EXPECT_NEAR(sphere.eval(Vec3d(7, 0, 0)), 2.0, 1e-10);
    verifyGradient(sphere, Vec3d(3, 4, 0));
}

// ============================================================
// 平底刀测试
// 扫掠体 = 圆柱(半径R,高H)沿线段平移的Minkowski和
// 三轴固定Z轴方向: 圆柱轴沿Z
// 对于XY平面: SDF_xy = dist_to_segment_xy - R
// 对于Z轴: SDF_z = 需要考虑上下限
// 最终: SDF = max(SDF_xy, SDF_z_bottom, SDF_z_top) 的某种组合
// ============================================================

class FlatEndTest : public ::testing::Test {
protected:
    // 平底刀 R=3mm, H=20mm, 刀路沿X: (0,0,10)->(20,0,10)
    // 刀轴沿Z, 刀尖在Z=10(底面), 刀顶在Z=30
    ToolDef tool{ToolType::FLAT_END, 3.0, 0.0, 20.0};
    MoveSegment seg{Vec3d(0,0,10), Vec3d(20,0,10)};
    ToolSweepSDF sdf{tool, seg};
};

TEST_F(FlatEndTest, CenterInside) {
    // (10, 0, 20): 在扫掠体中心, 深度内部
    EXPECT_LT(sdf.eval(Vec3d(10, 0, 20)), 0.0);
}

TEST_F(FlatEndTest, BelowBottom) {
    // (10, 0, 9): 在底面下方1mm
    EXPECT_NEAR(sdf.eval(Vec3d(10, 0, 9)), 1.0, 1e-10);
}

TEST_F(FlatEndTest, OnSideSurface) {
    // (10, 3, 20): 在侧面上(Y距轴心=R=3)
    EXPECT_NEAR(sdf.eval(Vec3d(10, 3, 20)), 0.0, 1e-10);
}

TEST_F(FlatEndTest, OutsideSide) {
    // (10, 5, 20): Y=5, 距侧面=2
    EXPECT_NEAR(sdf.eval(Vec3d(10, 5, 20)), 2.0, 1e-10);
}

TEST_F(FlatEndTest, OnBottomSurface) {
    // (10, 0, 10): 在底面中心(正好在底面上)
    EXPECT_NEAR(sdf.eval(Vec3d(10, 0, 10)), 0.0, 1e-10);
}

TEST_F(FlatEndTest, AboveTop) {
    // (10, 0, 31): 在顶面上方1mm
    EXPECT_NEAR(sdf.eval(Vec3d(10, 0, 31)), 1.0, 1e-10);
}

TEST_F(FlatEndTest, BottomEdge) {
    // (10, 3, 10): 底面边缘 (侧面和底面的交线), SDF=0
    EXPECT_NEAR(sdf.eval(Vec3d(10, 3, 10)), 0.0, 1e-10);
}

TEST_F(FlatEndTest, CornerOutside) {
    // (10, 4, 9): 底面棱角外, dist = sqrt(1² + 1²) = sqrt(2) ≈ 1.414
    // 距侧面1mm(Y方向), 距底面1mm(Z方向)
    double expected = std::sqrt(2.0);
    EXPECT_NEAR(sdf.eval(Vec3d(10, 4, 9)), expected, 1e-10);
}

TEST_F(FlatEndTest, GradientSide) {
    verifyGradient(sdf, Vec3d(10, 5, 20));
    verifyGradient(sdf, Vec3d(5, 4, 15));
}

TEST_F(FlatEndTest, GradientBottom) {
    verifyGradient(sdf, Vec3d(10, 0, 8));
    verifyGradient(sdf, Vec3d(10, 2, 8));
}

TEST_F(FlatEndTest, GradientCorner) {
    // 底面棱角外(非光滑点附近但稍远处)
    verifyGradient(sdf, Vec3d(10, 5, 8));
}

TEST_F(FlatEndTest, BeyondEndCap) {
    // (-2, 0, 20): X超出线段起点2mm, 距线段投影到X轴的起点=2
    // 圆柱侧面距离 = sqrt(2² + 0²) - 3 = -1 (inside in radial)
    // 但端部需要考虑: 距起点在XY面投影的距离=2, radial=0, 
    // 所以SDF_radial = 2 - 3 = -1 (inside), SDF_z: 在范围内
    EXPECT_LT(sdf.eval(Vec3d(-2, 0, 20)), 0.0);
    // (-5, 0, 20): 距起点5 > R=3, outside
    EXPECT_GT(sdf.eval(Vec3d(-5, 0, 20)), 0.0);
}
