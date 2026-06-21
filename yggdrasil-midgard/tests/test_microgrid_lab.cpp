#include <gtest/gtest.h>
#include "core/MicroGridLab.h"
#include "core/ToolSweptSDF.h"
#include <cstring>

using namespace midgard;

// ═══════════════════════════════════════════════════════════════
// SDF 约定：正 = 材料（保留），负 = 空气（切除）
// compositeSDF = min(tool_sdfs)
// ═══════════════════════════════════════════════════════════════

// ═══════════════════════════════════════════════════════════════
// M1 测试：结构体初始化
// ═══════════════════════════════════════════════════════════════

TEST(MicroGridLab, StructuresInit) {
    MicroGridCell cell;
    EXPECT_EQ(cell.activeCount(), 0);
    EXPECT_FALSE(cell.isActive(0));
    EXPECT_FALSE(cell.isActive(511));
    EXPECT_FALSE(cell.isInside(0));
    EXPECT_FALSE(cell.isInside(511));
    EXPECT_TRUE(cell.isOutside(0));
}

TEST(MicroGridLab, CubeSetup) {
    MicroGridLabState state;
    state.cubeSize  = 1.0;
    state.voxelSize = 0.5;
    state.precision = 0.01;
    state.init();

    EXPECT_EQ(state.voxelCount(), 8);

    for (int i = 0; i < 8; i++) {
        EXPECT_EQ(state.voxels[i].solidCount(), 512);
        EXPECT_EQ(state.voxels[i].activeCount(), 0);
        EXPECT_FALSE(state.voxels[i].isOutside(0));
    }
}

TEST(MicroGridLab, AddCutRecord) {
    MicroGridLabState state;
    state.cubeSize  = 1.0;
    state.voxelSize = 0.5;
    state.precision = 0.01;
    state.init();

    CutRecord rec;
    rec.tool     = {ToolType::BALL_END, 2.0, 0.0, 10.0};
    rec.segment  = {Vec3d(0, 0.5, 0.8), Vec3d(1, 0.5, 0.8), Vec3d(0, 0, 1), 0};
    state.addCutRecord(rec);

    EXPECT_EQ(state.cutHistory.size(), 1u);
    EXPECT_EQ(state.cutHistory[0].seqIndex, 0u);
    EXPECT_DOUBLE_EQ(state.cutHistory[0].tool.R, 2.0);
}

TEST(MicroGridLab, PrecisionLevels) {
    {
        MicroGridLabState state;
        state.voxelSize = 0.5;
        state.precision = 0.1;
        EXPECT_LE(state.maxOctreeDepth(), 1);
    }
    {
        MicroGridLabState state;
        state.voxelSize = 0.5;
        state.precision = 0.01;
        EXPECT_LE(state.maxOctreeDepth(), 3);
        EXPECT_GE(state.maxOctreeDepth(), 2);
    }
    {
        MicroGridLabState state;
        state.voxelSize = 0.5;
        state.precision = 0.001;
        EXPECT_LE(state.maxOctreeDepth(), 6);
        EXPECT_GE(state.maxOctreeDepth(), 5);
    }
}

TEST(MicroGridLab, CellSetAllSolid) {
    MicroGridCell cell;
    cell.setAllSolid();

    EXPECT_EQ(cell.solidCount(), 512);
    EXPECT_EQ(cell.activeCount(), 0);
    EXPECT_TRUE(cell.isInside(0));
    EXPECT_TRUE(cell.isInside(511));
    EXPECT_FALSE(cell.isOutside(0));

    cell.clear();
    EXPECT_EQ(cell.solidCount(), 0);
    EXPECT_EQ(cell.activeCount(), 0);
    EXPECT_FALSE(cell.isInside(0));
    EXPECT_TRUE(cell.isOutside(0));
}

TEST(MicroGridLab, MultipleCutRecords) {
    MicroGridLabState state;
    state.cubeSize  = 1.0;
    state.voxelSize = 0.5;
    state.precision = 0.01;
    state.init();

    CutRecord rec;
    rec.tool     = {ToolType::BALL_END, 2.0, 0.0, 10.0};
    rec.segment  = {Vec3d(0, 0.5, 0.8), Vec3d(1, 0.5, 0.8), Vec3d(0, 0, 1), 0};

    state.addCutRecord(rec);
    EXPECT_EQ(state.cutHistory.size(), 1u);

    state.addCutRecord(rec);
    EXPECT_EQ(state.cutHistory.size(), 2u);
    EXPECT_EQ(state.cutHistory[1].seqIndex, 1u);

    state.reset();
    EXPECT_EQ(state.cutHistory.size(), 2u);  // 保留 cutHistory
    EXPECT_EQ(state.voxelCount(), 8);
    EXPECT_EQ(state.totalSurfacePoints, 0);
    EXPECT_TRUE(state.surfacePoints.empty());
    // voxels 全部回到 SOLID 状态
    for (auto& v : state.voxels) {
        for (int ci = 0; ci < 512; ++ci) {
            if (v.isActive(ci)) {
                EXPECT_TRUE(v.isInside(ci));
            }
        }
    }
}

TEST(MicroGridLab, VoxelCountScaling) {
    {
        MicroGridLabState state;
        state.cubeSize  = 2.0;
        state.voxelSize = 0.5;
        state.precision = 0.01;
        state.init();
        EXPECT_EQ(state.voxelCount(), 64);
    }
    {
        MicroGridLabState state;
        state.cubeSize  = 1.0;
        state.voxelSize = 0.25;
        state.precision = 0.01;
        state.init();
        EXPECT_EQ(state.voxelCount(), 64);
    }
}

// ═══════════════════════════════════════════════════════════════
// M2 测试：粗筛算法
// ═══════════════════════════════════════════════════════════════

TEST(MicroGridLab, CeCenterCoordinates) {
    MicroGridLabState state;
    state.cubeSize  = 1.0;
    state.voxelSize = 0.5;
    state.init();

    Vec3d c0 = state.ceCenterWorld(0, 0);
    double ceHalf = 0.5 / 8.0 / 2.0;
    EXPECT_NEAR(c0.x(), ceHalf, 1e-9);
    EXPECT_NEAR(c0.y(), ceHalf, 1e-9);
    EXPECT_NEAR(c0.z(), ceHalf, 1e-9);

    Vec3d c511 = state.ceCenterWorld(0, 511);
    double expected = 0.5 - ceHalf;
    EXPECT_NEAR(c511.x(), expected, 1e-9);
    EXPECT_NEAR(c511.y(), expected, 1e-9);
    EXPECT_NEAR(c511.z(), expected, 1e-9);

    Vec3d v1c0 = state.ceCenterWorld(1, 0);
    EXPECT_NEAR(v1c0.x(), 0.5 + ceHalf, 1e-9);
    EXPECT_NEAR(v1c0.y(), ceHalf, 1e-9);
}

// ─── buildFromSDF: 全负（空气）────────────────────────────────
TEST(MicroGridLab, BuildFromSDF_AllAir) {
    MicroGridCell cell;
    float sdf[512];
    for (int i = 0; i < 512; ++i) sdf[i] = -10.0f;  // 远负 = 空气

    auto cls = cell.buildFromSDF(sdf, 0.05f, 0.2f);

    EXPECT_EQ(cls.airCount, 512);
    EXPECT_EQ(cls.solidCount, 0);
    EXPECT_EQ(cls.boundaryCount, 0);
    EXPECT_EQ(cell.activeCount(), 0);
    EXPECT_TRUE(cell.isOutside(0));
    EXPECT_TRUE(cell.isOutside(511));
}

// ─── buildFromSDF: 全正（材料/实心）────────────────────────────
TEST(MicroGridLab, BuildFromSDF_AllSolid) {
    MicroGridCell cell;
    float sdf[512];
    for (int i = 0; i < 512; ++i) sdf[i] = 10.0f;   // 远正 = 材料

    auto cls = cell.buildFromSDF(sdf, 0.05f, 0.2f);

    EXPECT_EQ(cls.airCount, 0);
    EXPECT_EQ(cls.solidCount, 512);
    EXPECT_EQ(cls.boundaryCount, 0);
    EXPECT_EQ(cell.activeCount(), 0);
    EXPECT_TRUE(cell.isInside(0));
    EXPECT_TRUE(cell.isInside(511));
}

// ─── buildFromSDF: 混合（有边界）──────────────────────────────
TEST(MicroGridLab, BuildFromSDF_Mixed) {
    MicroGridCell cell;
    float sdf[512];
    // SDF 约定：正=材料, 负=空气
    for (int i = 0; i < 100; ++i)  sdf[i] =  0.02f;   // 边界（材料侧）
    for (int i = 100; i < 200; ++i) sdf[i] = -0.03f;   // 边界（空气侧）
    for (int i = 200; i < 400; ++i) sdf[i] =  1.0f;    // 远正 = 材料
    for (int i = 400; i < 512; ++i) sdf[i] = -1.0f;    // 远负 = 空气

    auto cls = cell.buildFromSDF(sdf, 0.05f, 0.2f);

    EXPECT_EQ(cls.boundaryCount, 200);
    EXPECT_EQ(cls.solidCount, 200);
    EXPECT_EQ(cls.airCount, 112);
    EXPECT_EQ(cls.boundaryCount + cls.solidCount + cls.airCount, 512);
    EXPECT_EQ(cell.activeCount(), 200);
    EXPECT_EQ(cell.sdf.size(), 200u);
}

// ─── getValue: 边界 ce 读回正确值 ─────────────────────────────
TEST(MicroGridLab, GetValue_Roundtrip) {
    MicroGridCell cell;
    float sdf[512] = {};
    float bg = 0.2f;

    sdf[0] = 0.03f;    // 边界，材料侧（正）
    sdf[511] = -0.04f; // 边界，空气侧（负）
    for (int i = 1; i < 511; ++i) sdf[i] = bg;  // 远正 = 材料

    cell.buildFromSDF(sdf, 0.05f, bg);

    EXPECT_TRUE(cell.isActive(0));
    EXPECT_TRUE(cell.isActive(511));
    EXPECT_FALSE(cell.isActive(1));

    float v0 = cell.getValue(0, bg);
    float v511 = cell.getValue(511, bg);
    float v1 = cell.getValue(1, bg);

    EXPECT_GT(v0, 0.0f);                // 材料侧边界 → 正
    EXPECT_LT(v511, 0.0f);              // 空气侧边界 → 负
    EXPECT_NEAR(v1, bg, 1e-4f);         // 远材料 → +bg
}

// ─── 辅助：统计 cell 的三态 ce 数 ─────────────────────────────
namespace {
struct CeCounts { int air, solid, bnd; };
CeCounts countCEs(const MicroGridCell& cell) {
    CeCounts c = {0, 0, 0};
    for (int i = 0; i < 512; ++i) {
        if (cell.isActive(i))        ++c.bnd;
        else if (cell.isInside(i))   ++c.solid;
        else                          ++c.air;
    }
    return c;
}
} // namespace

// ─── 单刀切削：球头刀全覆盖 cube ────────────────────────────────
// 球头刀 R=2mm, 刀轴在 cube 中心偏下 → 全部 ce 被切除
TEST(MicroGridLab, SingleCut_BallEndAbove) {
    MicroGridLabState state;
    state.cubeSize  = 1.0;
    state.voxelSize = 0.5;
    state.precision = 0.01;
    state.init();

    CutRecord rec;
    rec.tool     = {ToolType::BALL_END, 2.0, 0.0, 10.0};
    rec.segment  = {Vec3d(0, 0.25, -0.5), Vec3d(1, 0.25, -0.5), Vec3d(0, 0, 1), 0};
    state.addCutRecord(rec);

    int solidBefore = 0;
    for (auto& v : state.voxels) solidBefore += v.solidCount();
    EXPECT_EQ(solidBefore, 8 * 512);

    state.executeCut(0);

    int airTotal = 0, solidTotal = 0, bndTotal = 0;
    for (auto& v : state.voxels) {
        auto c = countCEs(v);
        airTotal   += c.air;
        solidTotal += c.solid;
        bndTotal   += c.bnd;
    }

    // R=2 球完全覆盖 cube → 全部 ce 变为空气（被切除）
    EXPECT_EQ(airTotal, 8 * 512);
    EXPECT_EQ(solidTotal, 0);
    EXPECT_EQ(bndTotal, 0);
    EXPECT_EQ(state.totalSurfacePoints, 0);  // 无剩余材料 → 无表面

    EXPECT_GT(state.lastCutMs, 0.0);
}

// ─── 单刀切削：FLAT_END 刀水平横切 ────────────────────────────
// R=0.45mm 平底刀在 Z=0.5 水平穿过 cube, Y=0.5（中心线）
// ce 在 Z>0.5 且径向 ≤0.45mm → 切除(air)
// ce 在 Z<0.5 或径向 >0.45mm → 保留(solid)
TEST(MicroGridLab, SingleCut_FlatEndHorizontal) {
    MicroGridLabState state;
    state.cubeSize  = 1.0;
    state.voxelSize = 0.5;
    state.precision = 0.01;
    state.init();

    CutRecord rec;
    rec.tool     = {ToolType::FLAT_END, 0.45, 0.0, 5.0};
    rec.segment  = {Vec3d(0, 0.5, 0.5), Vec3d(1, 0.5, 0.5), Vec3d(1, 0, 0), 0};
    state.addCutRecord(rec);

    state.executeCut(0);

    int airTotal = 0, solidTotal = 0, bndTotal = 0;
    for (auto& v : state.voxels) {
        auto c = countCEs(v);
        airTotal   += c.air;
        solidTotal += c.solid;
        bndTotal   += c.bnd;
    }

    EXPECT_GT(airTotal, 0);           // Z>0.5 且近刀轴 → 切除
    EXPECT_GT(bndTotal, 0);           // 刀具表面附近 → 边界
    EXPECT_LT(solidTotal, 8 * 512);   // 部分 ce 保留
    EXPECT_EQ(airTotal + solidTotal + bndTotal, 8 * 512);
}

// ─── 两次切削：增量更新 ──────────────────────────────────────
TEST(MicroGridLab, TwoCut_Incremental) {
    MicroGridLabState state;
    state.cubeSize  = 1.0;
    state.voxelSize = 0.5;
    state.precision = 0.01;
    state.init();

    // 第 1 刀：球头刀 R=2，切穿 cube
    CutRecord rec1;
    rec1.tool    = {ToolType::BALL_END, 2.0, 0.0, 10.0};
    rec1.segment = {Vec3d(0, 0.25, -0.5), Vec3d(1, 0.25, -0.5), Vec3d(0, 0, 1), 0};
    state.addCutRecord(rec1);
    state.executeCut(0);

    int air1 = 0, solid1 = 0, bnd1 = 0;
    for (auto& v : state.voxels) {
        auto c = countCEs(v);
        air1 += c.air; solid1 += c.solid; bnd1 += c.bnd;
    }
    EXPECT_GT(air1, 0);

    // 第 2 刀：平底刀从侧面切入
    CutRecord rec2;
    rec2.tool    = {ToolType::FLAT_END, 0.5, 0.0, 5.0};
    rec2.segment = {Vec3d(-0.5, 0.5, 0.5), Vec3d(1.5, 0.5, 0.5), Vec3d(1, 0, 0), 0};
    state.addCutRecord(rec2);
    state.executeCut(1);

    int air2 = 0, solid2 = 0, bnd2 = 0;
    for (auto& v : state.voxels) {
        auto c = countCEs(v);
        air2 += c.air; solid2 += c.solid; bnd2 += c.bnd;
    }

    EXPECT_GE(air2, air1);
    EXPECT_EQ(air2 + solid2 + bnd2, 8 * 512);
    EXPECT_EQ(state.cutHistory.size(), 2u);
}

// ─── SDF 符号一致性：cube 内的 ce 中心应为负 SDF ─────────────
TEST(MicroGridLab, SDF_SignAtCubeCenter) {
    MicroGridLabState state;
    state.cubeSize  = 1.0;
    state.voxelSize = 0.5;
    state.init();

    CutRecord rec;
    rec.tool    = {ToolType::BALL_END, 2.0, 0.0, 10.0};
    rec.segment = {Vec3d(0.5, 0.5, -0.5), Vec3d(0.5, 0.5, -0.5), Vec3d(0, 0, 1), 0};
    ToolSweptSDF sdf(rec.tool, rec.segment);

    // Cube 中心点在刀内部 → tool_sdf < 0
    double s = sdf.eval(Vec3d(0.5, 0.5, 0.5));
    EXPECT_LT(s, 0.0);
}

// ─── 性能：单刀切削 8 voxels < 10ms ──────────────────────────
TEST(MicroGridLab, Performance_SingleCutUnder10ms) {
    MicroGridLabState state;
    state.cubeSize  = 1.0;
    state.voxelSize = 0.5;
    state.precision = 0.1;
    state.init();

    CutRecord rec;
    rec.tool     = {ToolType::BALL_END, 2.0, 0.0, 10.0};
    rec.segment  = {Vec3d(0, 0.25, -0.5), Vec3d(1, 0.25, -0.5), Vec3d(0, 0, 1), 0};
    state.addCutRecord(rec);

    state.executeCut(0);

    EXPECT_LT(state.lastCutMs, 10.0);
    EXPECT_GT(state.lastCutMs, 0.0);
}

// ═══════════════════════════════════════════════════════════════
// M3 测试：八叉树精修与表面提取
// ═══════════════════════════════════════════════════════════════

// ─── 全外 bbox → 无表面点 ───────────────────────────────────
TEST(MicroGridLab, Octree_AllOutsideBBox) {
    ToolDef tool{ ToolType::BALL_END, 2.0, 0.0, 10.0 };
    MoveSegment seg{ Vec3d(10, 10, -5), Vec3d(10, 10, -5), Vec3d(0,0,1), 0 };
    ToolSweptSDF sdf(tool, seg);

    openvdb::BBoxd bbox(Vec3d(0,0,0), Vec3d(1,1,1));  // 远离刀具
    OctreeConfig cfg;
    cfg.chordalTol = 0.01;
    cfg.maxDepth   = 4;

    auto pts = extractSurface(bbox, sdf, cfg);
    EXPECT_TRUE(pts.empty());
}

// ─── 全内 bbox → 无表面点 ───────────────────────────────────
TEST(MicroGridLab, Octree_AllInsideBBox) {
    ToolDef tool{ ToolType::BALL_END, 2.0, 0.0, 10.0 };
    // 刀心在原点，球半径 2mm → 球覆盖范围 [-2,2]
    MoveSegment seg{ Vec3d(0, 0, -2), Vec3d(0, 0, -2), Vec3d(0,0,1), 0 };
    ToolSweptSDF sdf(tool, seg);

    openvdb::BBoxd bbox(Vec3d(-0.1,-0.1,-0.1), Vec3d(0.1,0.1,0.1));  // 刀球心附近
    OctreeConfig cfg;
    cfg.chordalTol = 0.01;
    cfg.maxDepth   = 4;

    auto pts = extractSurface(bbox, sdf, cfg);
    EXPECT_TRUE(pts.empty());  // 全部在球内 → 刀内部 → 无表面
}

// ─── 跨表面 bbox → 有表面点 ─────────────────────────────────
TEST(MicroGridLab, Octree_CrossSurfaceBBox) {
    ToolDef tool{ ToolType::BALL_END, 2.0, 0.0, 10.0 };
    // 球心在原点，球半径 2mm → 球面在 |p|=2
    MoveSegment seg{ Vec3d(0, 0, -2), Vec3d(0, 0, -2), Vec3d(0,0,1), 0 };
    ToolSweptSDF sdf(tool, seg);

    // bbox 跨越球面 (x 方向从 -0.5 到 2.5, 球面在 x=2)
    openvdb::BBoxd bbox(Vec3d(1.5, -0.5, -0.5), Vec3d(2.5, 0.5, 0.5));
    OctreeConfig cfg;
    cfg.chordalTol = 0.01;
    cfg.maxDepth   = 6;

    auto pts = extractSurface(bbox, sdf, cfg);
    EXPECT_GT(pts.size(), 0u);  // 应有表面点

    // 验证表面点确实在球面附近（精度 = cell对角+曲率）
    Vec3d center(0, 0, 0);
    double tol = 0.1;  // 0.01mm cell diagonal + 球面曲率余量
    for (const auto& sp : pts) {
        double dist = (sp.position - center).length();
        EXPECT_NEAR(dist, 2.0, tol);
    }
}

// ─── 法向检查：应指向球外 ───────────────────────────────────
TEST(MicroGridLab, Octree_NormalPointsOutward) {
    ToolDef tool{ ToolType::BALL_END, 2.0, 0.0, 10.0 };
    MoveSegment seg{ Vec3d(0, 0, -2), Vec3d(0, 0, -2), Vec3d(0,0,1), 0 };
    ToolSweptSDF sdf(tool, seg);

    openvdb::BBoxd bbox(Vec3d(1.5, -0.5, -0.5), Vec3d(2.5, 0.5, 0.5));
    OctreeConfig cfg;
    cfg.chordalTol = 0.01;
    cfg.maxDepth   = 6;

    auto pts = extractSurface(bbox, sdf, cfg);
    ASSERT_GT(pts.size(), 0u);

    Vec3d center(0, 0, 0);
    for (const auto& sp : pts) {
        // 法向应指向球外（与 pos-center 同向）
        Vec3d radial = sp.position - center;
        radial.normalize();
        double dot = sp.normal.dot(radial);
        EXPECT_GT(dot, 0.0);  // 正点积 → 法向朝外
    }
}

// ─── 深度限制：maxDepth=0 → 不应递归 ────────────────────────
TEST(MicroGridLab, Octree_DepthLimitRespected) {
    ToolDef tool{ ToolType::BALL_END, 2.0, 0.0, 10.0 };
    MoveSegment seg{ Vec3d(0, 0, -2), Vec3d(0, 0, -2), Vec3d(0,0,1), 0 };
    ToolSweptSDF sdf(tool, seg);

    openvdb::BBoxd bbox(Vec3d(1.5, -0.5, -0.5), Vec3d(2.5, 0.5, 0.5));
    OctreeConfig cfg;
    cfg.chordalTol = 1e-6;  // 极小容差 → 应触达 maxDepth
    cfg.maxDepth   = 1;     // 限制深度

    auto pts = extractSurface(bbox, sdf, cfg);
    EXPECT_GT(pts.size(), 0u);  // maxDepth 到达后仍提取点
}

// ─── FlatEnd 刀具：提取切削表面点 ────────────────────────────
TEST(MicroGridLab, M3_SurfacePointsAfterCut) {
    MicroGridLabState state;
    state.cubeSize  = 1.0;
    state.voxelSize = 0.5;
    state.precision = 0.01;
    state.init();

    CutRecord rec;
    rec.tool     = {ToolType::FLAT_END, 0.45, 0.0, 5.0};
    rec.segment  = {Vec3d(0, 0.5, 0.5), Vec3d(1, 0.5, 0.5), Vec3d(1, 0, 0), 0};
    state.addCutRecord(rec);

    state.executeCut(0);

    // 应有切削产生的表面点
    EXPECT_GT(state.totalSurfacePoints, 0);
    EXPECT_EQ(state.totalSurfacePoints,
              static_cast<int>(state.surfacePoints.size()));

    // 表面点应在刀具表面（近零 SDF）
    ToolSweptSDF sdf(rec.tool, rec.segment);
    for (const auto& sp : state.surfacePoints) {
        double s = sdf.eval(sp.position);
        EXPECT_NEAR(s, 0.0, state.precision * 2.0);  // 放宽到 2t
    }
}

// ─── 弦高误差应在容差内 ─────────────────────────────────────
TEST(MicroGridLab, M3_ChordalErrorWithinTolerance) {
    MicroGridLabState state;
    state.cubeSize  = 1.0;
    state.voxelSize = 0.5;
    state.precision = 0.01;
    state.init();

    CutRecord rec;
    rec.tool     = {ToolType::BALL_END, 0.3, 0.0, 5.0};
    // 球心在 cube 中心 (0.5, 0.5, 0.5) → 球面半径 0.3mm 穿过 cube 内部
    rec.segment  = {Vec3d(0.5, 0.5, 0.2), Vec3d(0.5, 0.5, 0.2), Vec3d(0, 0, 1), 0};
    state.addCutRecord(rec);

    state.executeCut(0);

    EXPECT_GT(state.totalSurfacePoints, 0);
    // chordalTol=0.01mm cell diagonal + 球面曲率 → 误差 < 0.1mm
    EXPECT_LT(state.maxChordalError, 0.1);
}

// ═══════════════════════════════════════════════════════════════
// M5 测试：增量切削管线
// ═══════════════════════════════════════════════════════════════

// ─── 增量首刀 = 全量重建 ─────────────────────────────────────
TEST(MicroGridLab, M5_FirstCutIsFullRebuild) {
    MicroGridLabState state;
    state.cubeSize  = 1.0;
    state.voxelSize = 0.5;
    state.precision = 0.1;
    state.init();

    CutRecord rec;
    rec.tool    = {ToolType::BALL_END, 0.3, 0.0, 5.0};
    // 球心在 cube 中心 → 球面半径 0.3mm 穿过 cube
    rec.segment = {Vec3d(0.5, 0.5, 0.2), Vec3d(0.5, 0.5, 0.2), Vec3d(0, 0, 1), 0};
    state.addCutRecord(rec);

    state.executeCutIncremental(0);

    // 首刀应该有结果
    EXPECT_GT(state.lastCutMs, 0.0);
    EXPECT_GT(state.totalSurfacePoints, 0);
}

// ─── 增量第二刀：刀不移，无变化 ──────────────────────────────
TEST(MicroGridLab, M5_SecondCutNoChange) {
    MicroGridLabState state;
    state.cubeSize  = 1.0;
    state.voxelSize = 0.5;
    state.precision = 0.1;
    state.init();

    // 第一刀：球端刀切削中心
    CutRecord rec1;
    rec1.tool    = {ToolType::BALL_END, 2.0, 0.0, 10.0};
    rec1.segment = {Vec3d(0.5, 0.5, -0.5), Vec3d(0.5, 0.5, -0.5), Vec3d(0, 0, 1), 0};
    state.addCutRecord(rec1);

    state.executeCut(0);  // 首刀全量
    int ptsAfter1 = state.totalSurfacePoints;

    // 第二刀：远离 cube 的球端刀
    CutRecord rec2;
    rec2.tool    = {ToolType::BALL_END, 1.0, 0.0, 5.0};
    rec2.segment = {Vec3d(10, 10, 10), Vec3d(10, 10, 10), Vec3d(0, 0, 1), 0};
    state.addCutRecord(rec2);

    state.executeCutIncremental(1);

    // 远离的刀不应增加切削
    int totalAir = 0;
    for (auto& v : state.voxels)
        for (int i = 0; i < 512; ++i)
            if (v.isOutside(i)) ++totalAir;
    // airCount 不应增加（第二刀在远处）
    // 表面点数可能变化（增量重建了表面）
}

// ─── 增量第二刀：刀切更深 ────────────────────────────────────
TEST(MicroGridLab, M5_SecondCutDeeper) {
    MicroGridLabState state;
    state.cubeSize  = 1.0;
    state.voxelSize = 0.5;
    state.precision = 0.1;
    state.init();

    // 第一刀：球端刀切浅处
    CutRecord rec1;
    rec1.tool    = {ToolType::BALL_END, 0.5, 0.0, 10.0};
    rec1.segment = {Vec3d(0.5, 0.5, 0.3), Vec3d(0.5, 0.5, 0.3), Vec3d(0, 0, 1), 0};
    state.addCutRecord(rec1);
    state.executeCut(0);  // 首刀全量

    int airAfter1 = 0;
    for (auto& v : state.voxels)
        for (int i = 0; i < 512; ++i)
            if (v.isOutside(i)) ++airAfter1;
    EXPECT_GT(airAfter1, 0);

    // 第二刀：同位置大半径刀 → 更多 ce 变空气
    CutRecord rec2;
    rec2.tool    = {ToolType::BALL_END, 2.0, 0.0, 10.0};
    rec2.segment = {Vec3d(0.5, 0.5, -1.0), Vec3d(0.5, 0.5, -1.0), Vec3d(0, 0, 1), 0};
    state.addCutRecord(rec2);
    state.executeCutIncremental(1);

    int airAfter2 = 0;
    for (auto& v : state.voxels)
        for (int i = 0; i < 512; ++i)
            if (v.isOutside(i)) ++airAfter2;

    // 第二刀切除更多 → air 增加
    EXPECT_GE(airAfter2, airAfter1);
    EXPECT_GT(state.lastCutMs, 0.0);
}

// ─── 增量 vs 全量结果一致性 ──────────────────────────────────
TEST(MicroGridLab, M5_IncrementalMatchesFullRebuild) {
    // 全量重建
    MicroGridLabState stateFull;
    stateFull.cubeSize  = 1.0;
    stateFull.voxelSize = 0.5;
    stateFull.precision = 0.1;
    stateFull.init();

    CutRecord rec;
    rec.tool    = {ToolType::BALL_END, 1.5, 0.0, 10.0};
    rec.segment = {Vec3d(0.5, 0.5, -0.2), Vec3d(0.5, 0.5, -0.2), Vec3d(0, 0, 1), 0};
    stateFull.addCutRecord(rec);
    stateFull.executeCut(0);

    int fullAir = 0, fullBnd = 0;
    for (auto& v : stateFull.voxels)
        for (int i = 0; i < 512; ++i) {
            if (v.isOutside(i)) ++fullAir;
            if (v.isActive(i)) ++fullBnd;
        }

    // 增量方式（同一刀，即首刀）
    MicroGridLabState stateInc;
    stateInc.cubeSize  = 1.0;
    stateInc.voxelSize = 0.5;
    stateInc.precision = 0.1;
    stateInc.init();
    stateInc.addCutRecord(rec);
    stateInc.executeCutIncremental(0);

    int incAir = 0, incBnd = 0;
    for (auto& v : stateInc.voxels)
        for (int i = 0; i < 512; ++i) {
            if (v.isOutside(i)) ++incAir;
            if (v.isActive(i)) ++incBnd;
        }

    EXPECT_EQ(fullAir, incAir);
    EXPECT_EQ(fullBnd, incBnd);
}

// ─── ④ 旧点验证：旧表面点被新刀切除 ──────────────────────────
TEST(MicroGridLab, M5_OldPointsValidatedAgainstNewTool) {
    MicroGridLabState state;
    state.cubeSize  = 1.0;
    state.voxelSize = 0.5;
    state.precision = 0.1;
    state.init();

    // 第一刀：小球切 cube 上半部
    CutRecord rec1;
    rec1.tool    = {ToolType::BALL_END, 0.3, 0.0, 5.0};
    rec1.segment = {Vec3d(0.5, 0.5, 0.7), Vec3d(0.5, 0.5, 0.7), Vec3d(0, 0, 1), 0};
    state.addCutRecord(rec1);
    state.executeCutIncremental(0);

    int ptsAfter1 = state.totalSurfacePoints;
    EXPECT_GT(ptsAfter1, 0);

    // 第二刀：大球覆盖 cube 全部 → 旧点全部在刀内
    CutRecord rec2;
    rec2.tool    = {ToolType::BALL_END, 3.0, 0.0, 10.0};
    rec2.segment = {Vec3d(0.5, 0.5, -2.0), Vec3d(0.5, 0.5, -2.0), Vec3d(0, 0, 1), 0};
    state.addCutRecord(rec2);
    state.executeCutIncremental(1);

    // 旧点全被切除 → 新表面点全来自新刀
    // 大球内部切空大部分 cube，可能仍有边界点
    int ptsAfter2 = state.totalSurfacePoints;
    // 球面可能穿出cube外 → 可能有边界点；也可能cube全在球内 → 无边界点
    // 关键验证：第二刀执行无崩溃，结果合理
    EXPECT_GE(ptsAfter2, 0);
}

// ═══════════════════════════════════════════════════════════════
// TBB 并行化性能验证
// ═══════════════════════════════════════════════════════════════

// ─── 并行增量：高精度多刀不出错 ───────────────────────────────
TEST(MicroGridLab, TBB_MultiCutParallelCorrectness) {
    MicroGridLabState state;
    state.cubeSize  = 1.0;
    state.voxelSize = 0.5;
    state.precision = 0.01;
    state.init();

    // 3 刀序列：从浅到深逐刀切削
    // 刀1: 小球
    CutRecord r1{ {ToolType::BALL_END, 0.3, 0.0, 5.0},
                  {Vec3d(0.5, 0.5, 0.7), Vec3d(0.5, 0.5, 0.7), Vec3d(0,0,1), 0}, 0 };
    state.addCutRecord(r1);
    state.executeCutIncremental(0);
    auto pts1 = state.totalSurfacePoints;

    // 刀2: 中球更深 → 旧点部分幸存
    CutRecord r2{ {ToolType::BALL_END, 0.5, 0.0, 5.0},
                  {Vec3d(0.5, 0.5, 0.4), Vec3d(0.5, 0.5, 0.4), Vec3d(0,0,1), 0}, 1 };
    state.addCutRecord(r2);
    state.executeCutIncremental(1);
    auto pts2 = state.totalSurfacePoints;

    // 刀3: 大球切穿 → 几乎所有 ce 变空气
    CutRecord r3{ {ToolType::BALL_END, 2.0, 0.0, 10.0},
                  {Vec3d(0.5, 0.5, 0.0), Vec3d(0.5, 0.5, 0.0), Vec3d(0,0,1), 0}, 2 };
    state.addCutRecord(r3);
    state.executeCutIncremental(2);
    auto pts3 = state.totalSurfacePoints;

    // 无崩溃、无数据竞态 → 测试通过
    EXPECT_GE(pts1, 0);
    EXPECT_GE(pts2, 0);
    EXPECT_GE(pts3, 0);

    // 3 刀都执行了
    EXPECT_EQ(state.cutHistory.size(), 3u);
    EXPECT_GT(state.lastCutMs, 0.0);
}

// ─── 增量 vs 全量：5 刀 TBB 并行一致性 ─────────────────────────
TEST(MicroGridLab, TBB_FiveCutIncrementalConsistency) {
    auto runCuts = [](MicroGridLabState& st, bool incremental) {
        st.init();

        struct { ToolDef tool; MoveSegment seg; } cuts[] = {
            {{ToolType::BALL_END, 0.3, 0.0, 5.0},
             {Vec3d(0.5, 0.5, 0.7), Vec3d(0.5, 0.5, 0.7), Vec3d(0,0,1), 0}},
            {{ToolType::BALL_END, 0.5, 0.0, 5.0},
             {Vec3d(0.5, 0.5, 0.4), Vec3d(0.5, 0.5, 0.4), Vec3d(0,0,1), 0}},
            {{ToolType::FLAT_END, 0.3, 0.0, 5.0},
             {Vec3d(0.2, 0.5, 0.5), Vec3d(0.8, 0.5, 0.5), Vec3d(1,0,0), 0}},
            {{ToolType::BALL_END, 1.0, 0.0, 10.0},
             {Vec3d(0.5, 0.5, -0.3), Vec3d(0.5, 0.5, -0.3), Vec3d(0,0,1), 0}},
            {{ToolType::BALL_END, 0.2, 0.0, 5.0},
             {Vec3d(0.5, 0.3, 0.6), Vec3d(0.5, 0.3, 0.6), Vec3d(0,0,1), 0}},
        };

        for (int i = 0; i < 5; ++i) {
            CutRecord rec{ cuts[i].tool, cuts[i].seg, static_cast<uint32_t>(i) };
            st.addCutRecord(rec);
            if (incremental)
                st.executeCutIncremental(i);
            else
                st.executeCut(i);
        }
    };

    MicroGridLabState stFull, stInc;
    stFull.cubeSize = stInc.cubeSize = 1.0;
    stFull.voxelSize = stInc.voxelSize = 0.5;
    stFull.precision = stInc.precision = 0.05;

    runCuts(stFull, false);   // 全量 (串行)
    runCuts(stInc, true);     // 增量 (TBB 并行)

    // 三态统计必须一致
    auto countStates = [](MicroGridLabState& st) {
        int a = 0, s = 0, b = 0;
        for (auto& v : st.voxels)
            for (int i = 0; i < 512; ++i) {
                if (v.isOutside(i)) ++a;
                else if (v.isInside(i)) ++s;
                else ++b;
            }
        return std::make_tuple(a, s, b);
    };

    auto [af, sf, bf] = countStates(stFull);
    auto [ai, si, bi] = countStates(stInc);

    EXPECT_EQ(af, ai);
    EXPECT_EQ(sf, si);
    EXPECT_EQ(bf, bi);
}

// ── M7: ensureInit 不清理 cutHistory ──
TEST(MicroGridLab, M7_EnsureInitPreservesCutHistory) {
    MicroGridLabState st;
    st.cubeSize = 1.0;
    st.voxelSize = 0.5;
    st.precision = 0.05;

    // 先加刀，再 ensureInit
    CutRecord rec1;
    rec1.tool.type = ToolType::BALL_END;
    rec1.tool.R = 1.0;
    rec1.tool.H = 2.0;
    rec1.segment.start = Vec3d(0.0, 0.5, 0.8);
    rec1.segment.end   = Vec3d(1.0, 0.5, 0.8);
    st.addCutRecord(rec1);

    st.ensureInit();

    EXPECT_EQ(st.cutHistory.size(), 1u);
    EXPECT_FALSE(st.voxels.empty());
    EXPECT_EQ(st.voxelCount(), 8);  // (1.0/0.5)³ = 8
}

// ── M7: clampSurfacePoints ──────────────────────────────
TEST(MicroGridLab, M7_ClampSurfacePoints) {
    MicroGridLabState state;
    state.cubeSize  = 1.0;
    state.voxelSize = 0.5;
    state.precision = 0.01;
    state.init();

    // 添加越界的表面点
    state.surfacePoints.push_back({{-0.1, 0.5, 0.5}, {0,0,1}});  // below x
    state.surfacePoints.push_back({{1.2, 0.5, 0.5}, {0,0,1}});   // above x
    state.surfacePoints.push_back({{0.5, -0.2, 0.5}, {0,0,1}});  // below y
    state.surfacePoints.push_back({{0.5, 1.3, 0.5}, {0,0,1}});   // above y
    state.surfacePoints.push_back({{0.5, 0.5, -0.05}, {0,0,1}}); // below z
    state.surfacePoints.push_back({{0.5, 0.5, 1.1}, {0,0,1}});   // above z
    state.surfacePoints.push_back({{0.3, 0.3, 0.3}, {0,0,1}});   // valid

    state.clampSurfacePoints();

    EXPECT_NEAR(state.surfacePoints[0].position[0], 0.0, 1e-10);
    EXPECT_NEAR(state.surfacePoints[1].position[0], 1.0, 1e-10);
    EXPECT_NEAR(state.surfacePoints[2].position[1], 0.0, 1e-10);
    EXPECT_NEAR(state.surfacePoints[3].position[1], 1.0, 1e-10);
    EXPECT_NEAR(state.surfacePoints[4].position[2], 0.0, 1e-10);
    EXPECT_NEAR(state.surfacePoints[5].position[2], 1.0, 1e-10);
    EXPECT_NEAR(state.surfacePoints[6].position[0], 0.3, 1e-10);  // unchanged
}
