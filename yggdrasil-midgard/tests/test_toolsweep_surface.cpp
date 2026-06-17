#include <gtest/gtest.h>
#include "core/ToolSweepSurface.h"
#include "core/ToolSweepSDF.h"
#include <cmath>

using namespace midgard;

class ToolSweepSurfaceTest : public ::testing::Test {
protected:
    // 球头刀 R=5, 静态 (路径=0), 中心(0,0,0)
    ToolDef tool{ToolType::BALL_END, 5.0, 0.0, 20.0};
    MoveSegment staticSeg{Vec3d(0,0,0), Vec3d(0,0,0)};
};

// === 静态球头刀: 退化为全球面 ===

TEST_F(ToolSweepSurfaceTest, Static_EvalOnSphere) {
    ToolSweepSurface surf(tool, staticSeg);

    // 参数面上的所有点应在半径R的球面上
    for (int i = 0; i <= 10; ++i) {
        for (int j = 0; j <= 10; ++j) {
            double u = i / 10.0;
            double v = j / 10.0;
            Vec3d p = surf.eval(u, v);
            double dist = p.length();
            EXPECT_NEAR(dist, 5.0, 1e-10)
                << "u=" << u << " v=" << v << " dist=" << dist;
        }
    }
}

TEST_F(ToolSweepSurfaceTest, Static_CoversSphere) {
    ToolSweepSurface surf(tool, staticSeg);

    // 验证覆盖性: 球面上的关键方向都能被参数面到达
    // 检查: 南极、北极、赤道4方向、45度8方向
    Vec3d targets[] = {
        Vec3d(0, 0, -5),  // 南极(刀尖)
        Vec3d(0, 0, 5),   // 北极
        Vec3d(5, 0, 0),   // +X赤道
        Vec3d(0, 5, 0),   // +Y赤道
        Vec3d(-5, 0, 0),  // -X赤道
        Vec3d(0, -5, 0),  // -Y赤道
    };

    for (const auto& target : targets) {
        // 找到参数面上最接近target的点
        double minDist = 1e9;
        for (int i = 0; i <= 50; ++i) {
            for (int j = 0; j <= 50; ++j) {
                Vec3d p = surf.eval(i/50.0, j/50.0);
                double d = (p - target).length();
                minDist = std::min(minDist, d);
            }
        }
        EXPECT_LT(minDist, 0.5) << "Cannot reach target " << target;
    }
}

TEST_F(ToolSweepSurfaceTest, Static_SDFCrossValidation) {
    ToolSweepSurface surf(tool, staticSeg);
    ToolSweepSDF sdf(tool, staticSeg);

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
    EXPECT_LT(maxAbsSDF, 1e-10) << "Max |SDF| on surface = " << maxAbsSDF;
}

TEST_F(ToolSweepSurfaceTest, Static_NormalOutward) {
    ToolSweepSurface surf(tool, staticSeg);

    // 法线应指向外(与位置向量同向, 因为球心在原点)
    for (int i = 1; i <= 9; ++i) {  // 避开极点
        for (int j = 0; j <= 10; ++j) {
            double u = i / 10.0;
            double v = j / 10.0;
            Vec3d p = surf.eval(u, v);
            Vec3d n = surf.normal(u, v);
            // 法线应与位置向量同向(球面外法线 = p/|p|)
            Vec3d expected = p / p.length();
            double dot = n.dot(expected);
            EXPECT_GT(dot, 0.99) << "u=" << u << " v=" << v;
        }
    }
}

TEST_F(ToolSweepSurfaceTest, Static_BBoxConservative) {
    ToolSweepSurface surf(tool, staticSeg);

    // bbox应包含对应参数域内的所有点
    for (int iu = 0; iu < 4; ++iu) {
        for (int iv = 0; iv < 4; ++iv) {
            double u0 = iu / 4.0, u1 = (iu+1) / 4.0;
            double v0 = iv / 4.0, v1 = (iv+1) / 4.0;
            auto box = surf.bbox(u0, u1, v0, v1);

            // 域内采样点都应在bbox内
            for (int i = 0; i <= 5; ++i) {
                for (int j = 0; j <= 5; ++j) {
                    double u = u0 + (u1-u0) * i/5.0;
                    double v = v0 + (v1-v0) * j/5.0;
                    Vec3d p = surf.eval(u, v);
                    EXPECT_TRUE(box.isInside(p))
                        << "Point " << p << " outside bbox for ["
                        << u0 << "," << u1 << "]x[" << v0 << "," << v1 << "]";
                }
            }
        }
    }
}

TEST_F(ToolSweepSurfaceTest, Static_VsplitZero) {
    ToolSweepSurface surf(tool, staticSeg);
    // 路径=0, 无mid补丁, vSplit=0
    EXPECT_FALSE(surf.hasMidPatch());
    EXPECT_NEAR(surf.vSplit(), 0.0, 1e-15);
}

// === 线性刀路: S_mid + S_cap 双补丁 ===

class ToolSweepSurfaceLinearTest : public ::testing::Test {
protected:
    // 球头刀 R=3, 路径沿X: (0,0,0)->(10,0,0)
    ToolDef tool{ToolType::BALL_END, 3.0, 0.0, 20.0};
    MoveSegment seg{Vec3d(0,0,0), Vec3d(10,0,0)};
};

TEST_F(ToolSweepSurfaceLinearTest, HasMidPatch) {
    ToolSweepSurface surf(tool, seg);
    EXPECT_TRUE(surf.hasMidPatch());
    EXPECT_GT(surf.vSplit(), 0.0);
    EXPECT_LT(surf.vSplit(), 1.0);
}

TEST_F(ToolSweepSurfaceLinearTest, AllPointsOnSDF_Zero) {
    ToolSweepSurface surf(tool, seg);
    ToolSweepSDF sdf(tool, seg);

    // 统一接口: 全参数域采样, SDF应≈0
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

TEST_F(ToolSweepSurfaceLinearTest, MidPatch_OnSDF_Zero) {
    ToolSweepSurface surf(tool, seg);
    ToolSweepSDF sdf(tool, seg);

    double maxAbsSDF = 0;
    for (int i = 0; i <= 10; ++i) {
        for (int j = 0; j <= 10; ++j) {
            double u = i / 10.0;
            double t = j / 10.0;
            Vec3d p = surf.evalMid(u, t);
            double s = sdf.eval(p);
            maxAbsSDF = std::max(maxAbsSDF, std::abs(s));
        }
    }
    EXPECT_LT(maxAbsSDF, 1e-9) << "Mid patch max |SDF| = " << maxAbsSDF;
}

TEST_F(ToolSweepSurfaceLinearTest, CapPatch_OnSDF_Zero) {
    ToolSweepSurface surf(tool, seg);
    ToolSweepSDF sdf(tool, seg);

    double maxAbsSDF = 0;
    for (int i = 0; i <= 10; ++i) {
        for (int j = 0; j <= 10; ++j) {
            double u = i / 10.0;
            double w = j / 10.0;
            Vec3d p = surf.evalCap(u, w);
            double s = sdf.eval(p);
            maxAbsSDF = std::max(maxAbsSDF, std::abs(s));
        }
    }
    EXPECT_LT(maxAbsSDF, 1e-9) << "Cap patch max |SDF| = " << maxAbsSDF;
}

TEST_F(ToolSweepSurfaceLinearTest, MidPatch_SpansPath) {
    ToolSweepSurface surf(tool, seg);

    // t=0 时点在A附近, t=1时在B附近
    Vec3d p0 = surf.evalMid(0.5, 0.0);  // 赤道处, 路径起点
    Vec3d p1 = surf.evalMid(0.5, 1.0);  // 赤道处, 路径终点
    // X坐标应跨越约0到10
    EXPECT_NEAR(p0.x(), 0.0, 3.5);  // A.x + R偏移
    EXPECT_NEAR(p1.x(), 10.0, 3.5); // B.x + R偏移
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

TEST_F(ToolSweepSurfaceLinearTest, Dense100x100_SDFValidation) {
    ToolSweepSurface surf(tool, seg);
    ToolSweepSDF sdf(tool, seg);

    // 100×100 = 10201 点密集采样全域
    double maxAbsSDF = 0;
    int count = 0;
    for (int i = 0; i <= 100; ++i) {
        for (int j = 0; j <= 100; ++j) {
            double u = i / 100.0;
            double v = j / 100.0;
            Vec3d p = surf.eval(u, v);
            double s = sdf.eval(p);
            maxAbsSDF = std::max(maxAbsSDF, std::abs(s));
            count++;
        }
    }
    // 所有10201个点的SDF都应精确为0
    EXPECT_LT(maxAbsSDF, 1e-9)
        << "Dense 100x100: max|SDF|=" << maxAbsSDF << " over " << count << " points";
}
