// 切削策略A/B/C性能对比测试
// 编译: cd yggdrasil/build && c++ -std=c++17 -O2 -I.. -I../../install/include -I/opt/homebrew/include
//   ../tests/bench_strategies.cpp -L. -lyggdrasil -L../../install/lib -lopenvdb
//   -L/opt/homebrew/lib -ltbb -Wl,-rpath,../../install/lib -Wl,-rpath,/opt/homebrew/lib
//   -o bench_strategies

#include "core/ResolutionSolver.h"
#include "core/BilletBuilder.h"
#include "core/ToolSweepSDF.h"
#include "core/CuttingEngine.h"
#include <openvdb/tools/Composite.h>
#include <openvdb/tools/Prune.h>
#include <openvdb/tools/LevelSetMeasure.h>
#include <chrono>
#include <cstdio>

using namespace ygg;
using Clock = std::chrono::high_resolution_clock;

// 独立实现三种策略（不依赖宏切换）
static void stratA(openvdb::FloatGrid::Ptr grid, const ToolSweepSDF& tool) {
    auto& xform = grid->transform();
    double vs = xform.voxelSize()[0];
    float bw = 3.0f * (float)vs;
    auto bbox = tool.getBoundingBox();
    auto mn = xform.worldToIndexCellCentered(bbox.min());
    auto mx = xform.worldToIndexCellCentered(bbox.max());
    auto acc = grid->getAccessor();
    openvdb::Coord ijk;
    for (ijk[0]=mn[0]; ijk[0]<=mx[0]; ++ijk[0])
      for (ijk[1]=mn[1]; ijk[1]<=mx[1]; ++ijk[1])
        for (ijk[2]=mn[2]; ijk[2]<=mx[2]; ++ijk[2]) {
            auto wp = xform.indexToWorld(ijk);
            float td = (float)tool.eval(wp);
            if (td < bw) { float v=acc.getValue(ijk); float nv=std::max(v,-td); if(nv!=v) acc.setValue(ijk,nv); }
        }
    openvdb::tools::pruneLevelSet(grid->tree());
}

static void stratB(openvdb::FloatGrid::Ptr grid, const ToolSweepSDF& tool) {
    auto& xform = grid->transform();
    double vs = xform.voxelSize()[0];
    float bw = 3.0f * (float)vs;
    auto bbox = tool.getBoundingBox();
    auto mn = xform.worldToIndexCellCentered(bbox.min());
    auto mx = xform.worldToIndexCellCentered(bbox.max());
    for (auto it = grid->beginValueOn(); it; ++it) {
        auto c = it.getCoord();
        if (c.x()<mn.x()||c.x()>mx.x()||c.y()<mn.y()||c.y()>mx.y()||c.z()<mn.z()||c.z()>mx.z()) continue;
        float td = (float)tool.eval(xform.indexToWorld(c));
        if (td < bw) { float v=it.getValue(); float nv=std::max(v,-td); if(nv!=v) it.setValue(nv); }
    }
    openvdb::tools::pruneLevelSet(grid->tree());
}

static void stratC(openvdb::FloatGrid::Ptr grid, const ToolSweepSDF& tool) {
    auto& xform = grid->transform();
    double vs = xform.voxelSize()[0];
    float bw = 3.0f * (float)vs;
    auto bbox = tool.getBoundingBox();
    auto mn = xform.worldToIndexCellCentered(bbox.min());
    auto mx = xform.worldToIndexCellCentered(bbox.max());
    auto tg = openvdb::FloatGrid::create(bw);
    tg->setTransform(grid->transformPtr());
    tg->setGridClass(openvdb::GRID_LEVEL_SET);
    auto acc = tg->getAccessor();
    openvdb::Coord ijk;
    for (ijk[0]=mn[0]; ijk[0]<=mx[0]; ++ijk[0])
      for (ijk[1]=mn[1]; ijk[1]<=mx[1]; ++ijk[1])
        for (ijk[2]=mn[2]; ijk[2]<=mx[2]; ++ijk[2]) {
            float d = (float)tool.eval(xform.indexToWorld(ijk));
            if (std::abs(d) < bw) acc.setValue(ijk, d);
        }
    openvdb::tools::csgDifference(*grid, *tg);
}

int main() {
    openvdb::initialize();

    struct TestCase { double vs; double dim; double toolR; double segLen; };
    TestCase cases[] = {
        {0.5, 30, 3, 20},   // 粗精度小件
        {0.25, 30, 3, 20},  // 中精度小件
        {0.1, 30, 5, 20},   // 高精度小件
        {0.5, 100, 5, 50},  // 粗精度中件
        {0.25, 50, 5, 30},  // 中精度中件
    };

    printf("%-6s %-5s %-5s %-5s | %-10s %-10s %-10s | %-8s %-8s\n",
           "vs","dim","R","seg", "A(ms)","B(ms)","C(ms)", "C_mem","winner");
    printf("-----------------------------------------------------------------------\n");

    for (auto& tc : cases) {
        ResolutionConfig cfg;
        cfg.mode = ResolutionConfig::SINGLE_TRACK;
        cfg.d_v = tc.vs; cfg.D_v = tc.vs; cfg.N = 1;

        ToolSweepSDF tool(ToolType::BALL_END, tc.toolR, 0, 20,
            {tc.dim*0.2, tc.dim*0.5, tc.dim*0.4},
            {tc.dim*0.2 + tc.segLen, tc.dim*0.5, tc.dim*0.4});

        double times[3]; size_t memC = 0;
        for (int s = 0; s < 3; s++) {
            auto billet = buildBillet(cfg, {0,0,0}, {tc.dim, tc.dim, tc.dim*0.5});
            auto t0 = Clock::now();
            if (s==0) stratA(billet.sdfGrid, tool);
            else if (s==1) stratB(billet.sdfGrid, tool);
            else { stratC(billet.sdfGrid, tool); memC = billet.sdfGrid->memUsage(); }
            times[s] = std::chrono::duration<double,std::milli>(Clock::now()-t0).count();
        }

        int winner = (times[1] <= times[0] && times[1] <= times[2]) ? 1 :
                     (times[2] <= times[0]) ? 2 : 0;
        const char* names[] = {"A","B","C"};
        printf("%-6.2f %-5.0f %-5.1f %-5.0f | %-10.1f %-10.1f %-10.1f | %-8.1f %s\n",
               tc.vs, tc.dim, tc.toolR, tc.segLen,
               times[0], times[1], times[2], memC/1e6, names[winner]);
    }

    printf("\nA=brute bbox | B=active-only | C=rasterize+csgDiff\n");
    return 0;
}
