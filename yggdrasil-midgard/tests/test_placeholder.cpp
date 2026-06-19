#include <gtest/gtest.h>
#include "core/Types.h"

TEST(TypesTest, ToleranceConfigDerivation) {
    midgard::ToleranceConfig cfg(0.01);
    EXPECT_DOUBLE_EQ(cfg.user_t, 0.01);
    EXPECT_DOUBLE_EQ(cfg.voxelMacro, 0.1);  // K=10, 10*0.01=0.1
    EXPECT_DOUBLE_EQ(cfg.baseStep, 0.1);     // 10*0.01
    EXPECT_DOUBLE_EQ(cfg.chordalLimit, 0.01);
}

TEST(TypesTest, ToleranceConfigClamp) {
    // MIN_VOXEL_SIZE = 0.02
    midgard::ToleranceConfig tiny(0.0001);
    EXPECT_GE(tiny.voxelMacro, midgard::ToleranceConfig::MIN_VOXEL_SIZE);

    // MAX_VOXEL_SIZE = 5.0
    midgard::ToleranceConfig huge(1.0);
    EXPECT_LE(huge.voxelMacro, midgard::ToleranceConfig::MAX_VOXEL_SIZE);
}
