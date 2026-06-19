#include "debug/RtDebugSys.h"
#include "core/Types.h"
#include "core/ToolSweptSDF.h"
#include "core/ToolSweepSurface.h"
#include "core/MacroCut.h"
#include "core/MicroCut.h"
#include "core/IPWBuilder.h"
#include <openvdb/openvdb.h>
#include <openvdb/points/PointCount.h>
#include <filesystem>
#include <iostream>
#include <vector>
#include <chrono>
#include <cmath>
#include <iomanip>

using namespace midgard;

// ═══════════════════════════════════════════════════════════════════════
// 1000段微刀路压力测试 (Profiling Phase 1)
// ═══════════════════════════════════════════════════════════════════════

int main(int argc, char* argv[]) {
    openvdb::initialize();

    // 1. 初始化 Debug 系统（静默模式）
    RtDebugSys::Debugger::GetInstance().Activate(0);

    // 2. 参数配置
    double t = 0.05;
    ToleranceConfig config(t);
    
    // 3. 构建大毛坯 (100x100x20 mm)
    GeometryDef geom{GeometryDef::BOX, Vec3d(0), Vec3d(100, 100, 20)};
    IPWBuilder builder;
    auto ipw = builder.build(geom, config);

    std::cout << "════════════════════════════════════════════════════════════\n"
              << "  1000-Segment Stress Test (Profiling Phase 1)\n"
              << "════════════════════════════════════════════════════════════\n"
              << "  Billet Size:  " << geom.dims << " mm\n"
              << "  Macro Voxel:  " << config.voxelMacro << " mm\n"
              << "  Tolerance t:  " << config.user_t << " mm\n\n";

    // 4. 生成 1000 段微刀路 (螺旋线)
    std::vector<MoveSegment> toolpath;
    const int numSegments = 1000;
    double centerX = 50, centerY = 50;
    double radius = 40.0;
    double currentZ = 18.0;

    for (int i = 0; i < numSegments; ++i) {
        double theta1 = (2.0 * M_PI * i * 5.0) / numSegments; // 5圈
        double theta2 = (2.0 * M_PI * (i + 1) * 5.0) / numSegments;
        double r1 = radius * (1.0 - (double)i / numSegments);
        double r2 = radius * (1.0 - (double)(i + 1) / numSegments);

        MoveSegment seg;
        seg.start = Vec3d(centerX + r1 * std::cos(theta1), centerY + r1 * std::sin(theta1), currentZ);
        seg.end = Vec3d(centerX + r2 * std::cos(theta2), centerY + r2 * std::sin(theta2), currentZ);
        toolpath.push_back(seg);
    }

    // 5. 循环执行并计时
    MacroCut macrocut;
    MicroCut microcut;
    ToolDef tool{ToolType::BALL_END, 3.0, 0.0, 20.0};

    double totalMsP0 = 0, totalMsP1 = 0, totalMsP2 = 0, totalMsP34 = 0;
    int totalTasks = 0;

    std::cout << std::left << std::setw(8) << "Seg" 
              << std::setw(10) << "P0(ms)" 
              << std::setw(10) << "P1(ms)" 
              << std::setw(10) << "P2(ms)" 
              << std::setw(10) << "Tasks" << "\n"
              << "------------------------------------------------------------\n";

    for (int i = 0; i < numSegments; ++i) {
        const auto& seg = toolpath[i];
        ToolSweptSDF sdf(tool, seg);
        ToolSweepSurface surface(tool, seg);

        // Phase 0: Classify
        auto t0 = std::chrono::high_resolution_clock::now();
        auto cls = macrocut.classifyVoxels(ipw, sdf);
        auto t1 = std::chrono::high_resolution_clock::now();

        // Phase 1: Task List (Focus)
        auto t2 = std::chrono::high_resolution_clock::now();
        auto tasks = macrocut.buildTaskList(cls, surface, config, ipw.macroGrid->transform());
        auto t3 = std::chrono::high_resolution_clock::now();

        // Phase 2: Sample
        auto t4 = std::chrono::high_resolution_clock::now();
        auto buffers = microcut.sampleNewSurface(tasks, surface, sdf, config, ipw.macroGrid);
        auto t5 = std::chrono::high_resolution_clock::now();

        // Phase 3+4: Rebuild
        auto t6 = std::chrono::high_resolution_clock::now();
        microcut.rebuildLeaves(ipw, cls, buffers, sdf, config);
        auto t7 = std::chrono::high_resolution_clock::now();

        double ms0 = std::chrono::duration<double, std::milli>(t1 - t0).count();
        double ms1 = std::chrono::duration<double, std::milli>(t3 - t2).count();
        double ms2 = std::chrono::duration<double, std::milli>(t5 - t4).count();
        double ms34 = std::chrono::duration<double, std::milli>(t7 - t6).count();

        totalMsP0 += ms0;
        totalMsP1 += ms1;
        totalMsP2 += ms2;
        totalMsP34 += ms34;
        totalTasks += tasks.size();

        if (i % 100 == 0 || i == numSegments - 1) {
            std::cout << std::setw(8) << i 
                      << std::setw(10) << std::fixed << std::setprecision(2) << ms0
                      << std::setw(10) << ms1
                      << std::setw(10) << ms2
                      << std::setw(10) << tasks.size() << std::endl;
        }
    }

    std::cout << "------------------------------------------------------------\n"
              << "  TOTAL (1000 segments):\n"
              << "  Phase 0: " << totalMsP0 << " ms (Avg: " << totalMsP0/numSegments << ")\n"
              << "  Phase 1: " << totalMsP1 << " ms (Avg: " << totalMsP1/numSegments << ")\n"
              << "  Phase 2: " << totalMsP2 << " ms (Avg: " << totalMsP2/numSegments << ")\n"
              << "  Phase 3+4: " << totalMsP34 << " ms (Avg: " << totalMsP34/numSegments << ")\n"
              << "  Avg Tasks per Segment: " << (double)totalTasks/numSegments << "\n"
              << "════════════════════════════════════════════════════════════\n";

    return 0;
}
