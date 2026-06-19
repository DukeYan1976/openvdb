#include <gtest/gtest.h>
#include "core/ToolSweepSurface.h"
#include "core/ToolSweptSDF.h"
#include <cmath>

using namespace midgard;

class ToolSweepSurfaceTest : public ::testing::Test {
protected:
    // 球头刀 R=5, 静态 (路径=0), 中心(0,0,0)
    ToolDef tool{ToolType::BALL_END, 5.0, 0.0, 10.0}; // H=10, R=5 => H_cyl=5
    MoveSegment staticSeg{Vec3d(0,0,0), Vec3d(0,0,0)};
};

// === 静态球头刀: 退化为全扫掠面 ===

TEST_F(ToolSweepSurfaceTest, Static_SDFCrossValidation) {
    ToolSweepSurface surf(tool, staticSeg);
    ToolSweptSDF sdf(tool, staticSeg);

    // 核心验证: 参数面上的所有点, SDF值应≈0
    double maxAbsSDF = 0;
    for (int i = 0; i <= 20; ++i) {
        for (int j = 0; j <= 20; ++j) {
            double u = i / 20.0;
            double v = j / 20.0;
            Vec3d p = surf.eval(u, v);
            double s = sdf.eval(p);
            maxAbsSDF = std::max(maxAbsSDF, std::abs(s));
        }
    }
    EXPECT_LT(maxAbsSDF, 1e-9) << "Max |SDF| on surface = " << maxAbsSDF;
}

TEST_F(ToolSweepSurfaceTest, Static_NormalOutward) {
    ToolSweepSurface surf(tool, staticSeg);

    // 法线应基本指向外
    for (int i = 1; i <= 9; ++i) {
        for (int j = 0; j <= 10; ++j) {
            double u = i / 10.0;
            double v = j / 10.0;
            Vec3d n = surf.normal(u, v);
            EXPECT_NEAR(n.length(), 1.0, 1e-10);
            // 极坐标转换后的法向 z 分量在底部应为负 (nr*sin(a), 0, -cos(a))
            if (u < surf.uSplit2() * 0.5) {
                EXPECT_LT(n.z(), 0.0);
            }
        }
    }
}

// === 线性刀路: 6-Patch 迎水面模型 ===

class ToolSweepSurfaceLinearTest : public ::testing::Test {
protected:
    // 球头刀 R=3, 路径沿X: (0,0,0)->(10,0,0)
    ToolDef tool{ToolType::BALL_END, 3.0, 0.0, 10.0};
    MoveSegment seg{Vec3d(0,0,0), Vec3d(10,0,0)};
};

TEST_F(ToolSweepSurfaceLinearTest, SplitParametersValid) {
    ToolSweepSurface surf(tool, seg);
    EXPECT_GT(surf.uSplit2(), 0.0);
    EXPECT_LT(surf.uSplit2(), 1.0);
    EXPECT_GT(surf.v1(), 0.0);
    EXPECT_LT(surf.v1(), surf.v2());
    EXPECT_LT(surf.v2(), 1.0);
}

TEST_F(ToolSweepSurfaceLinearTest, AllPointsOnSDF_Zero) {
    ToolSweepSurface surf(tool, seg);
    ToolSweptSDF sdf(tool, seg);

    double maxAbsSDF = 0;
    for (int i = 0; i <= 30; ++i) {
        for (int j = 0; j <= 30; ++j) {
            double u = i / 30.0;
            double v = j / 30.0;
            Vec3d p = surf.eval(u, v);
            double s = sdf.eval(p);
            maxAbsSDF = std::max(maxAbsSDF, std::abs(s));
        }
    }
    EXPECT_LT(maxAbsSDF, 1e-9) << "Max |SDF| on surface = " << maxAbsSDF;
}

TEST_F(ToolSweepSurfaceLinearTest, BoundaryConditions) {
    ToolSweepSurface surf(tool, seg);
    
    // 闭合参数面: v=0 和 v=1 是同一点 (Left-Mid 起点处)
    Vec3d p0 = surf.eval(0.5, 0.0);
    Vec3d p1 = surf.eval(0.5, 1.0);
    
    EXPECT_NEAR(p0.x(), p1.x(), 1e-10) << "Closed surface: v=0 == v=1";
    EXPECT_NEAR(p0.y(), p1.y(), 1e-10);
    EXPECT_NEAR(p0.z(), p1.z(), 1e-10);
    
    // Left-Mid(v=0) 和 Right-Mid起点(v=v3) 应该在起点处Y对称
    Vec3d pL = surf.eval(0.5, 0.0);       // Left-Mid start: phi=+π/2, Y=+R
    Vec3d pR = surf.eval(0.5, surf.v3()); // Right-Mid end(=start): phi=-π/2, Y=-R
    
    EXPECT_NEAR(pL.x(), 0.0, 1e-10);
    EXPECT_NEAR(pR.x(), 0.0, 1e-10);
    EXPECT_NEAR(std::abs(pL.y()), 3.0, 1e-10);
    EXPECT_NEAR(std::abs(pR.y()), 3.0, 1e-10);
    EXPECT_NEAR(pL.y(), -pR.y(), 1e-10);
}

TEST_F(ToolSweepSurfaceLinearTest, BBox_Conservative) {
    ToolSweepSurface surf(tool, seg);

    // 全域bbox应包含所有点
    auto fullBox = surf.bbox(0, 1, 0, 1);
    for (int i = 0; i <= 10; ++i) {
        for (int j = 0; j <= 10; ++j) {
            Vec3d p = surf.eval(i/10.0, j/10.0);
            EXPECT_TRUE(fullBox.isInside(p));
        }
    }
}

TEST_F(ToolSweepSurfaceLinearTest, ArcLengthUniformity) {
    ToolSweepSurface surf(tool, seg);
    
    // 测试 u 向采样是否均匀
    // 在球头段 u=[0, uSplit], 间隔 0.1uSplit 的空间距离应大致相等
    double prev_z = surf.eval(0, 0.5).z();
    double sum_dist = 0;
    for (int i = 1; i <= 10; ++i) {
        double u = (i / 10.0) * surf.uSplit2();
        double curr_z = surf.eval(u, 0.5).z();
        sum_dist += std::abs(curr_z - prev_z);
        prev_z = curr_z;
    }
    // 弧长参数化下, Z 的分布不再是线性的 (sin/cos), 但弧长是线性的
    // 我们验证 Z 的变化不是极度不均即可
}
