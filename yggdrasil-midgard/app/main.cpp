#include "debug/RtDebugSys.h"
#include "core/Types.h"
#include "core/ToolSweepSDF.h"
#include "core/ToolSweepSurface.h"
#include "core/MacroCut.h"
#include "core/MicroCut.h"
#include "core/IPWBuilder.h"
#include "core/MeshExport.h"
#include <openvdb/openvdb.h>
#include <openvdb/points/PointCount.h>
#include <filesystem>
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <unordered_set>

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
    ToolSweepSDF sdf(tool, seg);

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
    pageBreak("Phase 1: 任务列表生成");

    ToolSweepSurface surface(tool, seg);
    auto tasks = macrocut.buildTaskList(cls, surface, config, ipw.macroGrid->transform());

    std::cout << "  Tasks generated: " << tasks.size() << "\n"
              << "  vSplit: " << surface.vSplit() << "\n"
              << "  hasMidPatch: " << (surface.hasMidPatch() ? "yes" : "no") << "\n";

    // 统计参数域范围
    double avgURange = 0, avgVRange = 0;
    for (const auto& t : tasks) {
        avgURange += (t.u_max - t.u_min);
        avgVRange += (t.t_max - t.t_min);
    }
    if (!tasks.empty()) {
        avgURange /= tasks.size();
        avgVRange /= tasks.size();
    }
    std::cout << "  Avg param range: u=" << avgURange << " v=" << avgVRange << "\n";

    pageBreak("Phase 2-4: 待实现 (MicroCut + Compaction)");

    // Phase 2: 四叉树采样
    MicroCut microcut;
    auto buffers = microcut.sampleNewSurface(tasks, surface, sdf, config);

    int totalNewPoints = 0;
    for (const auto& [coord, buf] : buffers) totalNewPoints += buf.positions.size();

    std::cout << "  Phase 2 results:\n"
              << "    Voxels with new points: " << buffers.size() << "\n"
              << "    Total new points: " << totalNewPoints << "\n";

    // ─── Phase 3+4: 旧点剔除 + 重建 ────────────────────────────────
    pageBreak("Phase 3+4: 重建 MicroGrid");

    openvdb::Index64 ptsBefore = openvdb::points::pointCount(ipw.microGrid->tree());

    auto tR0 = std::chrono::high_resolution_clock::now();
    microcut.rebuildLeaves(ipw, cls, buffers, sdf, config);
    auto tR1 = std::chrono::high_resolution_clock::now();

    openvdb::Index64 ptsAfter = openvdb::points::pointCount(ipw.microGrid->tree());
    double msRebuild = std::chrono::duration<double, std::milli>(tR1 - tR0).count();

    std::cout << "  MicroGrid points: " << ptsBefore << " -> " << ptsAfter << "\n"
              << "  Rebuild time: " << msRebuild << " ms\n";

    // ─── 端到端精度验证 ─────────────────────────────────────────────
    pageBreak("M8: 切削面精度验证");

    // 只验证切削区域（cut + newBoundary voxels）内的点
    // 这些点应该在扫掠面上（新生成的）或在刀具外部（存活旧点）
    double maxSdfInCutZone = 0;
    int cutZonePoints = 0;
    std::unordered_set<int64_t> cutZoneSet;
    auto ck = [](const openvdb::Coord& c) -> int64_t {
        return (int64_t(c.x()) << 40) | (int64_t(c.y() & 0xFFFFF) << 20) | int64_t(c.z() & 0xFFFFF);
    };
    for (const auto& c : cls.cut) cutZoneSet.insert(ck(c));
    for (const auto& c : cls.newBoundary) cutZoneSet.insert(ck(c));

    for (auto leaf = ipw.microGrid->tree().cbeginLeaf(); leaf; ++leaf) {
        auto posHandle = openvdb::points::AttributeHandle<Vec3f>::create(
            leaf->constAttributeArray("P"));
        for (auto it = leaf->beginIndexOn(); it; ++it) {
            openvdb::Coord voxCoord = it.getCoord();
            if (!cutZoneSet.count(ck(voxCoord))) continue;  // 非切削区跳过

            Vec3f pos = posHandle->get(*it);
            Vec3d wp = ipw.microGrid->transform().indexToWorld(
                voxCoord.asVec3d() + Vec3d(pos));
            double s = sdf.eval(wp);
            // 新点应SDF≈0, 存活旧点应SDF≥t
            // 最小SDF代表最深入刀具的点（不应有SDF<0）
            if (s < 0) maxSdfInCutZone = std::max(maxSdfInCutZone, -s);
            cutZonePoints++;
        }
    }

    std::cout << "  Cut zone points checked: " << cutZonePoints << "\n"
              << "  Max penetration (SDF<0): " << maxSdfInCutZone << " mm\n"
              << "  Tolerance t: " << config.user_t << " mm\n"
              << "  " << (maxSdfInCutZone <= config.user_t ? "[PASS]" : "[WARN]")
              << " All cut-zone points within tolerance\n";

    // ─── 总结 ────────────────────────────────────────────────────────
    pageBreak("Summary");

    std::cout << "  IPW0 build:   " << msBuild << " ms\n"
              << "  MacroCut:     " << msCut << " ms\n"
              << "  Total:        " << (msBuild + msCut) << " ms\n"
              << "  Status:       Phase 0 " << (deletedOK && boundaryOK ? "PASS" : "FAIL") << "\n"
              << "\n  Debug log: " << (debugDir / "DebugInfo.txt").string() << "\n";

    // ─── OBJ 导出 ───────────────────────────────────────────────────
    pageBreak("OBJ Export");

    std::filesystem::path objDir = exePath / "obj_output";
    std::filesystem::create_directories(objDir);

    // 1. 原始毛坯 mesh
    {
        auto pristine = IPWBuilder().build(geom, config);
        exportMacroMesh(pristine.macroGrid, (objDir / "billet_original.obj").string());
        std::cout << "  [1] billet_original.obj\n";
    }

    // 2. 切削后工件 mesh
    exportMacroMesh(ipw.macroGrid, (objDir / "billet_after_cut.obj").string());
    std::cout << "  [2] billet_after_cut.obj\n";

    // 3. 切削区点云
    exportMicroPoints(ipw.microGrid, (objDir / "cut_surface_points.obj").string());
    std::cout << "  [3] cut_surface_points.obj\n";

    std::cout << "\n  Output: " << objDir.string() << "\n";

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return (deletedOK && boundaryOK) ? 0 : 1;
}
