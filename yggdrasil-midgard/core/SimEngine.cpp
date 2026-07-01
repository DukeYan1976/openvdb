#include "core/SimEngine.h"
#include "core/MacroCut.h"
#include "core/MicroCut.h"
#include "core/ToolSweptSDF.h"
#include "core/ToolSweepSurface.h"
#include <chrono>

namespace midgard {

CutResult SimEngine::cutSegment(IPWState& ipw, const MoveSegment& seg,
                                 const ToolDef& tool, const GeometryDef& billet,
                                 const ToleranceConfig& config)
{
    CutResult result;
    auto t0 = std::chrono::high_resolution_clock::now();

    // ── 构建刀具扫掠体 SDF ──────────────────────────────────────
    ToolSweptSDF sdf(tool, seg);

    // ── Phase 0: CSG 差集 + 三态分类 ────────────────────────────
    MacroCut macroCut;
    auto cls = macroCut.classifyVoxels(ipw, sdf);

    result.deletedVoxels = static_cast<int>(cls.deleted.size());
    result.cutVoxels = static_cast<int>(cls.cut.size());
    result.newBoundaryVoxels = static_cast<int>(cls.newBoundary.size());
    result.affectedVoxels = static_cast<int>(cls.deleted.size() + cls.cut.size() + cls.newBoundary.size());
    result.pathLength = (seg.end - seg.start).length();

    // ── Phase 1: 生成 VoxelTask 列表 ───────────────────────────
    ToolSweepSurface sweepSurface(tool, seg, true);
    auto tasks = macroCut.buildTaskList(cls, sweepSurface, config, ipw.macroGrid->transform());

    // ── Phase 1.5: 对缺失点云的 NEW_BOUNDARY 体素进行毛坯表面采样（冷启动） ────────
    MicroCut microCut;
    auto billetBoundaries = microCut.primeBilletBoundaries(tasks, billet, config, ipw.microGrid);

    // ── Phase 2: TBB 并行自适应四叉树采样 ─────────────────────────
    auto newSurfacePts = microCut.sampleNewSurface(tasks, sweepSurface, sdf, config, ipw, billet);

    // 合并新点缓冲区与毛坯原始表面采样缓冲区
    std::unordered_map<openvdb::Coord, PointBuffer> combinedBuffers = newSurfacePts;
    for (const auto& [coord, buf] : billetBoundaries) {
        if (combinedBuffers.count(coord)) {
            combinedBuffers[coord].positions.insert(combinedBuffers[coord].positions.end(), buf.positions.begin(), buf.positions.end());
            combinedBuffers[coord].normals.insert(combinedBuffers[coord].normals.end(), buf.normals.begin(), buf.normals.end());
        } else {
            combinedBuffers[coord] = buf;
        }
    }

    // ── Phase 3+4: 旧点剔除 + per-leaf 重建 MicroGrid ─────────────────
    microCut.rebuildLeaves(ipw, cls, combinedBuffers, sdf, config);

    result.success = true;
    result.elapsedMs = std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - t0).count();

    return result;
}

} // namespace midgard
