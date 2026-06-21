#include "core/Types.h"
#include "core/IPWBuilder.h"
#include "core/IDebugDisplay.h"
#include <cstdio>
#include <cassert>
#include <cmath>

using namespace midgard;

// g_debugDisplay provided by midgard-core library (core/IDebugDisplay.cpp)

static int failures = 0;

#define CHECK(cond, msg) do { \
    if (!(cond)) { fprintf(stderr, "FAIL: %s\n", msg); ++failures; } \
    else { fprintf(stdout, "  PASS: %s\n", msg); } \
} while(0)

// ─── TC1: Box 毛坯构建 IPWState ────────────────────────────
static void tc1_box_ipw() {
    fprintf(stdout, "\nTC1: Box 50x50x20, tol=0.5\n");

    GeometryDef geom;
    geom.type = GeometryDef::BOX;
    geom.origin = Vec3d(0, 0, 0);
    geom.dims = Vec3d(50, 50, 20);

    ToleranceConfig config(0.5);

    IPWBuilder builder;
    IPWState ipw = builder.build(geom, config);

    CHECK(ipw.macroGrid != nullptr,              "macroGrid 非空");
    CHECK(ipw.microGrid != nullptr,              "microGrid 非空");
    CHECK(ipw.config.user_t == 0.5,               "config.user_t == 0.5");

    // Voxel 尺寸应 = K * t = 10 * 0.5 = 5.0 mm
    double vs = ipw.macroGrid->voxelSize()[0];
    CHECK(std::abs(vs - 5.0) < 1e-6,             "voxelSize == 5.0mm");

    // BBox 应覆盖毛坯 + halfwidth 带
    auto bbox = ipw.macroGrid->evalActiveVoxelBoundingBox();
    CHECK(!bbox.empty(),                          "active bbox 非空");

    // active voxel 数量应 > 0
    CHECK(ipw.macroGrid->activeVoxelCount() > 0,  "activeVoxelCount > 0");

    fprintf(stdout, "  active voxels: %llu\n", (unsigned long long)ipw.macroGrid->activeVoxelCount());
}

// ─── TC2: 不同公差 → 不同体素尺寸 ──────────────────────────
static void tc2_tolerance_scaling() {
    fprintf(stdout, "\nTC2: 公差 0.1 vs 1.0 → voxel 尺寸缩放\n");

    GeometryDef geom;
    geom.type = GeometryDef::BOX;
    geom.origin = Vec3d(0, 0, 0);
    geom.dims = Vec3d(50, 50, 20);

    IPWBuilder builder;

    {
        ToleranceConfig fine(0.1);
        IPWState ipw = builder.build(geom, fine);
        double vs = ipw.macroGrid->voxelSize()[0];
        CHECK(std::abs(vs - 1.0) < 1e-6,         "tol=0.1 → voxelSize=1.0mm");
    }

    {
        ToleranceConfig coarse(1.0);
        IPWState ipw = builder.build(geom, coarse);
        double vs = ipw.macroGrid->voxelSize()[0];
        // K=10 * 1.0 = 10.0, 但被 MAX_VOXEL_SIZE=5.0 钳制
        CHECK(std::abs(vs - 5.0) < 1e-6,        "tol=1.0 → voxelSize=5.0mm (clamped by MAX)");
    }
}

// ─── TC3: 重建 (修改 GeometryDef → 新 IPWState) ─────────────
static void tc3_rebuild() {
    fprintf(stdout, "\nTC3: 修改 GeometryDef 重建 IPWState\n");

    IPWBuilder builder;
    ToleranceConfig config(0.5);

    // 第一次：小毛坯
    GeometryDef small;
    small.type = GeometryDef::BOX;
    small.origin = Vec3d(0, 0, 0);
    small.dims = Vec3d(10, 10, 10);
    IPWState ipw1 = builder.build(small, config);
    size_t n1 = ipw1.macroGrid->activeVoxelCount();

    // 第二次：大毛坯（覆盖旧数据）
    GeometryDef large;
    large.type = GeometryDef::BOX;
    large.origin = Vec3d(0, 0, 0);
    large.dims = Vec3d(50, 50, 20);
    IPWState ipw2 = builder.build(large, config);
    size_t n2 = ipw2.macroGrid->activeVoxelCount();

    CHECK(n2 > n1,                                "大毛坯 activeVoxelCount > 小毛坯");

    fprintf(stdout, "  small: %zu voxels, large: %zu voxels\n", n1, n2);
}

// ─── TC4: sampleBoundary 箱体边界采样 ───────────────────────
static void tc4_sample_boundary() {
    fprintf(stdout, "\nTC4: sampleBoundary 箱体边界采样\n");

    GeometryDef geom;
    geom.type = GeometryDef::BOX;
    geom.origin = Vec3d(0, 0, 0);
    geom.dims = Vec3d(10, 10, 10);

    // 完整 BBox 应采样 6 个面
    openvdb::BBoxd region(Vec3d(0,0,0), Vec3d(10,10,10));
    PointBuffer buf = IPWBuilder::sampleBoundary(geom, region);
    CHECK(buf.positions.size() == 6,              "完整 region 采样 6 面");
    CHECK(buf.normals.size() == 6,                "法线数量匹配");

    // 子区域：只相交 3 个面
    openvdb::BBoxd sub(Vec3d(0,0,0), Vec3d(5,5,5));
    PointBuffer buf2 = IPWBuilder::sampleBoundary(geom, sub);
    CHECK(buf2.positions.size() == 3,             "子区域采样 3 面");
}

// ─── main ──────────────────────────────────────────────────
int main() {
    openvdb::initialize();

    tc1_box_ipw();
    tc2_tolerance_scaling();
    tc3_rebuild();
    tc4_sample_boundary();

    fprintf(stdout, "\n%s: %d failures\n", failures ? "FAILED" : "PASSED", failures);
    return failures ? 1 : 0;
}
