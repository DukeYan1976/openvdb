#include "debug/RtDebugSys.h"
#include "core/Types.h"
#include "core/ToolSweptSDF.h"
#include "core/MacroCut.h"
#include "core/IPWBuilder.h"
#include <openvdb/openvdb.h>
#include <openvdb/points/PointCount.h>
#include <filesystem>
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>

using namespace midgard;

// ═══════════════════════════════════════════════════════════════════════
// 分页输出辅助
// ═══════════════════════════════════════════════════════════════════════
static void pageBreak(const std::string& title) {
    std::cout << "\n════════════════════════════════════════════════════════════\n"
              << "  " << title << "\n"
              << "════════════════════════════════════════════════════════════\n\n";
}

// ═══════════════════════════════════════════════════════════════════════
// 集成测试: 球头刀沿X方向切削方块毛坯
//
// 配置:
//   毛坯: 20×20×20mm Box
//   刀具: 球头刀 R=5mm
//   刀路: 单段线性 (0,10,18)->(20,10,18), 切深2mm
//   容差: t=0.05mm → V_macro=1.5mm
// ═══════════════════════════════════════════════════════════════════════
int main(int argc, char* argv[]) {
    openvdb::initialize();

    // ─── Debug系统初始化 ─────────────────────────────────────────────
    std::filesystem::path exePath = std::filesystem::canonical(argv[0]).parent_path();
    std::filesystem::path debugDir = exePath / "debug_output";
    std::filesystem::create_directories(debugDir);
    RtDebugSys::Debugger::GetInstance().SetWorkspace(debugDir.string());
    RtDebugSys::Debugger::GetInstance().Activate(1);

    pageBreak("Phase -1: 参数配置");

    double t = 0.05;  // 用户容差 mm
    ToleranceConfig config(t);
    std::cout << "  user_t      = " << config.user_t << " mm\n"
              << "  V_macro     = " << config.voxelMacro << " mm\n"
              << "  K factor    = " << config.K << "\n"
              << "  halfwidth   = " << config.halfwidth << "\n"
              << "  baseStep    = " << config.baseStep << " mm\n"
              << "  chordalLimit= " << config.chordalLimit << " mm\n";

    // ─── IPW0 构建 ──────────────────────────────────────────────────
    pageBreak("Phase -1: IPW0 毛坯构建");

    GeometryDef geom;
    geom.type = GeometryDef::BOX;
    geom.origin = Vec3d(0);
    geom.dims = Vec3d(20, 20, 20);

    auto tBuild0 = std::chrono::high_resolution_clock::now();
    IPWBuilder builder;
    auto ipw = builder.build(geom, config);
    auto tBuild1 = std::chrono::high_resolution_clock::now();

    double msBuild = std::chrono::duration<double, std::milli>(tBuild1 - tBuild0).count();
    openvdb::Index64 microPoints = openvdb::points::pointCount(ipw.microGrid->tree());

    std::cout << "  Box: " << geom.dims << " mm\n"
              << "  MacroGrid active voxels: " << ipw.macroGrid->activeVoxelCount() << "\n"
              << "  MacroGrid leaf nodes:    " << ipw.macroGrid->tree().leafCount() << "\n"
              << "  MicroGrid points:        " << microPoints << "\n"
              << "  MicroGrid leaf nodes:    " << ipw.microGrid->tree().leafCount() << "\n"
              << "  Build time:              " << msBuild << " ms\n";

    // ─── 刀具定义 ───────────────────────────────────────────────────
    pageBreak("Phase 0: 刀具 & 刀路");

    ToolDef tool{ToolType::BALL_END, 5.0, 0.0, 30.0};
    MoveSegment seg{Vec3d(0, 10, 18), Vec3d(20, 10, 18)};
    ToolSweptSDF sdf(tool, seg);

    double pathLen = (seg.end - seg.start).length();
    auto bbox = sdf.boundingBox();

    std::cout << "  Tool: BallEnd R=" << tool.R << "mm\n"
              << "  Path: " << seg.start << " -> " << seg.end << "\n"
              << "  Path length: " << pathLen << " mm\n"
              << "  Tool BBox: " << bbox.min() << " -> " << bbox.max() << "\n"
              << "  Tip Z (lowest): " << (seg.start.z() - tool.R) << " mm\n"
              << "  Cut depth from top(Z=20): " << (20.0 - (seg.start.z() - tool.R)) << " mm\n";

    // ─── MacroCut (Phase 0) ─────────────────────────────────────────
    pageBreak("Phase 0: MacroCut 执行");

    int activeBefore = ipw.macroGrid->activeVoxelCount();

    auto tCut0 = std::chrono::high_resolution_clock::now();
    MacroCut macrocut;
    auto cls = macrocut.classifyVoxels(ipw, sdf);
    auto tCut1 = std::chrono::high_resolution_clock::now();

    double msCut = std::chrono::duration<double, std::milli>(tCut1 - tCut0).count();
    int activeAfter = ipw.macroGrid->activeVoxelCount();
    double V = config.voxelMacro;
    double threshold = V * std::sqrt(3.0) / 2.0;

    std::cout << "  Active voxels: " << activeBefore << " -> " << activeAfter
              << " (delta=" << (activeAfter - activeBefore) << ")\n"
              << "  Classification:\n"
              << "    deleted (d):      " << cls.deleted.size() << "\n"
              << "    cut (c):          " << cls.cut.size() << "\n"
              << "    newBoundary (n):  " << cls.newBoundary.size() << "\n"
              << "  Total boundary:     " << (cls.cut.size() + cls.newBoundary.size()) << "\n"
              << "  Boundary/path_len:  " << (cls.cut.size() + cls.newBoundary.size()) / pathLen
              << " voxels/mm\n"
              << "  Threshold:          " << threshold << " mm\n"
              << "  MacroCut time:      " << msCut << " ms\n";

    // ─── 分类精度验证 ───────────────────────────────────────────────
    pageBreak("Phase 0: 分类精度验证");

    // deleted: SDF应 < -threshold
    double maxDeletedSDF = -1e9;
    for (const auto& co : cls.deleted) {
        Vec3d wp = ipw.macroGrid->indexToWorld(co);
        maxDeletedSDF = std::max(maxDeletedSDF, sdf.eval(wp));
    }

    // boundary: |SDF| 应 <= threshold
    double maxBoundarySDF = 0;
    for (const auto& co : cls.cut) {
        Vec3d wp = ipw.macroGrid->indexToWorld(co);
        maxBoundarySDF = std::max(maxBoundarySDF, std::abs(sdf.eval(wp)));
    }
    for (const auto& co : cls.newBoundary) {
        Vec3d wp = ipw.macroGrid->indexToWorld(co);
        maxBoundarySDF = std::max(maxBoundarySDF, std::abs(sdf.eval(wp)));
    }

    bool deletedOK = cls.deleted.empty() || (maxDeletedSDF < -threshold + 1e-10);
    bool boundaryOK = maxBoundarySDF <= threshold + 1e-10;

    std::cout << "  deleted max SDF:   " << maxDeletedSDF
              << (deletedOK ? " [PASS]" : " [FAIL]") << "\n"
              << "  boundary max|SDF|: " << maxBoundarySDF
              << (boundaryOK ? " [PASS]" : " [FAIL]") << "\n"
              << "  threshold:         " << threshold << "\n";

    // ─── Phase 2/3/4 占位 ───────────────────────────────────────────
    pageBreak("Phase 2-4: 待实现 (MicroCut + Compaction)");

    std::cout << "  [TODO] Phase 2: 四叉树采样生成新切削面点\n"
              << "         - 需处理 " << cls.newBoundary.size() << " 个 voxel_n (从零生成)\n"
              << "         - 需处理 " << cls.cut.size() << " 个 voxel_c (补新点)\n"
              << "  [TODO] Phase 3: 旧点剔除 (SDF < t)\n"
              << "         - 需处理 " << cls.cut.size() << " 个 voxel_c 中的旧点\n"
              << "         - 需删除 " << cls.deleted.size() << " 个 voxel_d 的全部点\n"
              << "  [TODO] Phase 4: per-leaf 重建 MicroGrid\n";

    // ─── 总结 ────────────────────────────────────────────────────────
    pageBreak("Summary");

    std::cout << "  IPW0 build:   " << msBuild << " ms\n"
              << "  MacroCut:     " << msCut << " ms\n"
              << "  Total:        " << (msBuild + msCut) << " ms\n"
              << "  Status:       Phase 0 " << (deletedOK && boundaryOK ? "PASS" : "FAIL") << "\n"
              << "\n  Debug log: " << (debugDir / "DebugInfo.txt").string() << "\n";

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return (deletedOK && boundaryOK) ? 0 : 1;
}
