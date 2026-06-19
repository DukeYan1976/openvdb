#include <gtest/gtest.h>
#include "core/LocalSurfaceEngine.h"

using namespace midgard;

TEST(LocalSurfaceEngineTest, MultiNormalSDF) {
    // 模拟一个 90 度凸脊线 (Corner)
    std::vector<Vec3f> pos = { Vec3f(0,0,0), Vec3f(0,0,0) };
    std::vector<Vec3f> nrm = { Vec3f(1,0,0), Vec3f(0,1,0) }; // 90-degree ridge
    openvdb::BBoxd bbox(Vec3d(-1,-1,-1), Vec3d(1,1,1));
    
    LocalSurfaceEngine engine(pos, nrm, bbox);
    
    // 内部: ( -1, -1 ) -> SDF_x = -1, SDF_y = -1 -> max = -1 (Inside)
    EXPECT_LT(engine.eval(Vec3d(-1, -1, 0)), 0.0);
    
    // 外部 (空气):
    // ( 1, -1 ) -> SDF_x = 1, SDF_y = -1 -> max = 1 (Air)
    EXPECT_GT(engine.eval(Vec3d(1, -1, 0)), 0.0);
    // ( -1, 1 ) -> SDF_x = -1, SDF_y = 1 -> max = 1 (Air)
    EXPECT_GT(engine.eval(Vec3d(-1, 1, 0)), 0.0);
    // ( 1, 1 ) -> max = 1 (Air)
    EXPECT_GT(engine.eval(Vec3d(1, 1, 0)), 0.0);
}

TEST(LocalSurfaceEngineTest, EmptyCloud) {
    openvdb::BBoxd bbox(Vec3d(-1,-1,-1), Vec3d(1,1,1));
    LocalSurfaceEngine engine({}, {}, bbox);
    EXPECT_GT(engine.eval(Vec3d(0,0,0)), 1e6); // Empty should be treated as air
}

TEST(LocalSurfaceEngineTest, AdaptiveGridAccuracy) {
    // 构造一个包含足够多点的平面 (Z=0, N=(0,0,1)) 来触发格栅
    std::vector<Vec3f> pos;
    std::vector<Vec3f> nrm;
    for (int i = 0; i < 10; ++i) {
        for (int j = 0; j < 10; ++j) {
            pos.push_back(Vec3f(i*0.1f, j*0.1f, 0.0f));
            nrm.push_back(Vec3f(0.0f, 0.0f, 1.0f));
        }
    }
    openvdb::BBoxd bbox(Vec3d(0,0,-1), Vec3d(1,1,1));
    
    // N=100 > 64, 应该触发 4x4x4 格栅
    LocalSurfaceEngine engine(pos, nrm, bbox);
    
    // 验证内部点 (Z < 0)
    EXPECT_NEAR(engine.eval(Vec3d(0.5, 0.5, -0.2)), -0.2, 1e-6);
    // 验证外部点 (Z > 0)
    EXPECT_NEAR(engine.eval(Vec3d(0.5, 0.5, 0.3)), 0.3, 1e-6);
}
