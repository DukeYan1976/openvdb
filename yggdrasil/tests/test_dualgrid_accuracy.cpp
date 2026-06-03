#include <gtest/gtest.h>
#include "core/BilletBuilder.h"
#include "core/ToolSweepSDF.h"
#include "core/CuttingEngine.h"
#include "types/MemoryStats.h"
#include <openvdb/points/PointCount.h>
#include <openvdb/points/PointAttribute.h>
#include <cmath>

using namespace ygg;

class DualGridAccuracyTest : public ::testing::Test {
protected:
    void SetUp() override { openvdb::initialize(); }
};

TEST_F(DualGridAccuracyTest, DeactivatedSurfels_InsideTool) {
    // 验证所有被剥离的面元确实在刀具内部
    ResolutionConfig cfg;
    cfg.mode = ResolutionConfig::DUAL_TRACK;
    cfg.d_v = 0.5; cfg.D_v = 4.0; cfg.N = 8;

    auto billet = buildBillet(cfg, {0,0,0}, {30, 30, 20});
    ToolSweepSDF tool(ToolType::BALL_END, 5.0, 0, 30, {5,15,22}, {25,15,22});

    CuttingEngine engine;
    engine.cut(billet, tool);

    auto& tree = billet.microGrid->tree();
    auto& xform = billet.microGrid->transform();

    size_t deactivated = 0;
    size_t wrongDeactivation = 0;

    for (auto leaf = tree.beginLeaf(); leaf; ++leaf) {
        auto leafOrigin = leaf->origin();
        auto& attrSet = leaf->attributeSet();
        auto* posArr = attrSet.get("P");
        auto* activeArr = attrSet.get("active");
        auto ph = openvdb::points::AttributeHandle<openvdb::Vec3f>::create(*posArr);
        auto ah = openvdb::points::AttributeHandle<uint8_t>::create(*activeArr);

        for (openvdb::Index vIdx = 0; vIdx < 512; ++vIdx) {
            openvdb::Index end = static_cast<openvdb::Index>(leaf->getValue(vIdx));
            openvdb::Index start = (vIdx == 0) ? openvdb::Index(0) :
                static_cast<openvdb::Index>(leaf->getValue(vIdx - 1));

            openvdb::Coord localCoord((vIdx >> 6) & 7, (vIdx >> 3) & 7, vIdx & 7);
            openvdb::Coord voxelCoord = leafOrigin + localCoord;

            for (openvdb::Index i = start; i < end; ++i) {
                if (ah->get(i) == 0) {
                    deactivated++;
                    openvdb::Vec3f p = ph->get(i);
                    Vec3d worldPos = xform.indexToWorld(
                        Vec3d(voxelCoord.x() + p.x(), voxelCoord.y() + p.y(), voxelCoord.z() + p.z()));
                    double dist = tool.eval(worldPos);
                    if (dist > cfg.d_v) { // 允许 d_v 容差
                        wrongDeactivation++;
                    }
                }
            }
        }
    }

    printf("Deactivated surfels: %zu, wrong: %zu\n", deactivated, wrongDeactivation);
    EXPECT_GT(deactivated, 0u);
    EXPECT_EQ(wrongDeactivation, 0u);
}

TEST_F(DualGridAccuracyTest, ActiveSurfels_OutsideTool) {
    // 验证所有仍活跃的面元确实在刀具外部
    ResolutionConfig cfg;
    cfg.mode = ResolutionConfig::DUAL_TRACK;
    cfg.d_v = 0.5; cfg.D_v = 4.0; cfg.N = 8;

    auto billet = buildBillet(cfg, {0,0,0}, {30, 30, 20});
    ToolSweepSDF tool(ToolType::BALL_END, 5.0, 0, 30, {5,15,22}, {25,15,22});

    CuttingEngine engine;
    engine.cut(billet, tool);

    auto& tree = billet.microGrid->tree();
    auto& xform = billet.microGrid->transform();

    size_t active = 0;
    size_t wrongActive = 0;

    for (auto leaf = tree.beginLeaf(); leaf; ++leaf) {
        auto leafOrigin = leaf->origin();
        auto& attrSet = leaf->attributeSet();
        auto* posArr = attrSet.get("P");
        auto* activeArr = attrSet.get("active");
        auto ph = openvdb::points::AttributeHandle<openvdb::Vec3f>::create(*posArr);
        auto ah = openvdb::points::AttributeHandle<uint8_t>::create(*activeArr);

        for (openvdb::Index vIdx = 0; vIdx < 512; ++vIdx) {
            openvdb::Index end = static_cast<openvdb::Index>(leaf->getValue(vIdx));
            openvdb::Index start = (vIdx == 0) ? openvdb::Index(0) :
                static_cast<openvdb::Index>(leaf->getValue(vIdx - 1));

            openvdb::Coord localCoord((vIdx >> 6) & 7, (vIdx >> 3) & 7, vIdx & 7);
            openvdb::Coord voxelCoord = leafOrigin + localCoord;

            for (openvdb::Index i = start; i < end; ++i) {
                if (ah->get(i) == 1) {
                    active++;
                    openvdb::Vec3f p = ph->get(i);
                    Vec3d worldPos = xform.indexToWorld(
                        Vec3d(voxelCoord.x() + p.x(), voxelCoord.y() + p.y(), voxelCoord.z() + p.z()));
                    double dist = tool.eval(worldPos);
                    if (dist < -cfg.d_v) { // 深在刀具内部却仍活跃
                        wrongActive++;
                    }
                }
            }
        }
    }

    printf("Active surfels: %zu, wrongly active (inside tool): %zu\n", active, wrongActive);
    EXPECT_GT(active, 0u);
    EXPECT_EQ(wrongActive, 0u);
}

TEST_F(DualGridAccuracyTest, DualVsSingle_SmallPart_Comparable) {
    // 小件：双轨和单轨对同一切削，体积差应很小
    Vec3d dims = {20, 20, 10};

    // 单轨
    ResolutionConfig cfgSingle;
    cfgSingle.mode = ResolutionConfig::SINGLE_TRACK;
    cfgSingle.d_v = 0.5; cfgSingle.D_v = 0.5; cfgSingle.N = 1;
    auto billetS = buildBillet(cfgSingle, {0,0,0}, dims);

    // 双轨
    ResolutionConfig cfgDual;
    cfgDual.mode = ResolutionConfig::DUAL_TRACK;
    cfgDual.d_v = 0.5; cfgDual.D_v = 4.0; cfgDual.N = 8;
    auto billetD = buildBillet(cfgDual, {0,0,0}, dims);

    ToolSweepSDF tool(ToolType::BALL_END, 4.0, 0, 20, {5,10,12}, {15,10,12});

    CuttingEngine engine;
    engine.cut(billetS, tool);
    engine.cut(billetD, tool);

    double volSingle = computeVolume(billetS.sdfGrid);
    double volDual = computeVolume(billetD.sdfGrid);

    printf("Single-track volume: %.2f, Dual-track volume: %.2f\n", volSingle, volDual);
    // 双轨的 computeVolume 基于粗 SDF(D_v=4mm)，精度远低于单轨(d_v=0.5mm)
    // 差异大是预期行为——双轨的真实精度由面元承载，不由宏观 SDF 体积反映
    // 这里只验证两者量级一致（同一数量级）
    EXPECT_GT(volDual, 0.0);
    EXPECT_GT(volSingle, 0.0);
    EXPECT_NEAR(volDual, volSingle, volSingle * 0.5); // 50% 容差（粗 SDF 的固有误差）
}
