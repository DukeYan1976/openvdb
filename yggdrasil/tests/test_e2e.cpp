#include <gtest/gtest.h>
#include "core/ResolutionSolver.h"
#include "core/BilletBuilder.h"
#include "core/ToolSweepSDF.h"
#include "core/CuttingEngine.h"
#include <openvdb/io/File.h>
#include <cmath>

using namespace ygg;

class EndToEndTest : public ::testing::Test {
protected:
    void SetUp() override { openvdb::initialize(); }
};

TEST_F(EndToEndTest, FullPipeline_SingleTrack) {
    // 1. 分辨率求解
    auto cfg = solveResolution(0.5, 10.0, 2.0, {30, 30, 15});
    ASSERT_EQ(cfg.mode, ResolutionConfig::SINGLE_TRACK);
    EXPECT_DOUBLE_EQ(cfg.d_v, 0.25);

    // 2. 构建毛坯
    auto billet = buildBillet(cfg, {0,0,0}, {30, 30, 15});
    ASSERT_NE(billet.sdfGrid, nullptr);
    double volOriginal = computeVolume(billet.sdfGrid);
    // 理论体积 30×30×15 = 13500 mm³
    EXPECT_NEAR(volOriginal, 13500.0, volOriginal * 0.05);

    // 3. 执行多段切削（模拟一条槽）
    CuttingEngine engine;
    engine.cut(billet, ToolSweepSDF(ToolType::BALL_END, 3.0, 0, 20, {5,15,0}, {25,15,0}));

    // 4. 验证
    double volAfter = computeVolume(billet.sdfGrid);
    EXPECT_LT(volAfter, volOriginal);
    EXPECT_GT(volAfter, 0.0);

    // 切削路径中心应为空气
    auto& xform = billet.sdfGrid->transform();
    auto idx = xform.worldToIndexCellCentered({15, 15, 0});
    EXPECT_GT(billet.sdfGrid->getConstAccessor().getValue(idx), 0.0f);

    // 远离切削区域应不变
    auto farIdx = xform.worldToIndexCellCentered({5, 5, 7});
    EXPECT_LT(billet.sdfGrid->getConstAccessor().getValue(farIdx), 0.0f);

    // 5. 输出 .vdb 文件
    std::string path = "/tmp/ygg_e2e_result.vdb";
    openvdb::io::File file(path);
    openvdb::GridPtrVec grids;
    grids.push_back(billet.sdfGrid);
    file.write(grids);
    file.close();

    // 验证文件可读
    openvdb::io::File readFile(path);
    readFile.open();
    auto readGrids = readFile.getGrids();
    EXPECT_EQ(readGrids->size(), 1u);
    readFile.close();
    std::remove(path.c_str());
}

TEST_F(EndToEndTest, MultipleCuts_SlotMilling) {
    // 模拟开槽：多次平行切削
    auto cfg = solveResolution(0.5, 10.0, 2.0, {40, 40, 10});
    auto billet = buildBillet(cfg, {0,0,0}, {40, 40, 10});
    double volBefore = computeVolume(billet.sdfGrid);

    CuttingEngine engine;
    // 5条平行刀路，间距4mm
    for (int i = 0; i < 5; ++i) {
        double y = 10.0 + i * 4.0;
        engine.cut(billet, ToolSweepSDF(ToolType::BALL_END, 2.0, 0, 15,
                                        {5, y, 0}, {35, y, 0}));
    }

    double volAfter = computeVolume(billet.sdfGrid);
    EXPECT_LT(volAfter, volBefore);

    // 输出可视化
    std::string path = "/tmp/ygg_slot_milling.vdb";
    openvdb::io::File file(path);
    openvdb::GridPtrVec grids;
    grids.push_back(billet.sdfGrid);
    file.write(grids);
    file.close();
    // 保留文件供手动查看（不删除）
}
