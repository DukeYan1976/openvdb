#include <gtest/gtest.h>
#include "types/MemoryStats.h"

using namespace ygg;

class MemoryStatsTest : public ::testing::Test {
protected:
    void SetUp() override { openvdb::initialize(); }
};

TEST_F(MemoryStatsTest, NullGrids_AllZero) {
    MemoryStats stats;
    stats.update(nullptr, nullptr);
    EXPECT_EQ(stats.floatGridBytes, 0u);
    EXPECT_EQ(stats.pointGridBytes, 0u);
    EXPECT_EQ(stats.totalBytes, 0u);
    EXPECT_EQ(stats.pointCount, 0u);
    EXPECT_EQ(stats.activePointCount, 0u);
    EXPECT_EQ(stats.leafNodeCount, 0u);
}

TEST_F(MemoryStatsTest, FloatGridOnly_ReportsMemory) {
    auto grid = openvdb::FloatGrid::create(3.0f);
    grid->setGridClass(openvdb::GRID_LEVEL_SET);
    // 写入一些值使其有内存占用
    auto acc = grid->getAccessor();
    for (int i = 0; i < 10; ++i)
        acc.setValue(openvdb::Coord(i, 0, 0), float(i) * 0.1f);

    MemoryStats stats;
    stats.update(grid, nullptr);
    EXPECT_GT(stats.floatGridBytes, 0u);
    EXPECT_EQ(stats.pointGridBytes, 0u);
    EXPECT_EQ(stats.totalBytes, stats.floatGridBytes);
}

TEST_F(MemoryStatsTest, EmptyPointGrid_ReportsStructure) {
    auto ptGrid = openvdb::points::PointDataGrid::create();
    ptGrid->setTransform(openvdb::math::Transform::createLinearTransform(1.0));

    auto sdfGrid = openvdb::FloatGrid::create(3.0f);

    MemoryStats stats;
    stats.update(sdfGrid, ptGrid);
    EXPECT_GT(stats.pointGridBytes, 0u);  // 至少有树结构开销
    EXPECT_EQ(stats.pointCount, 0u);
    EXPECT_EQ(stats.leafNodeCount, 0u);
}
