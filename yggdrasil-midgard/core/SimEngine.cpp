#include "core/SimEngine.h"
#include "core/MacroCut.h"
#include "core/ToolSweptSDF.h"
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
    result.success = true;
    result.elapsedMs = std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - t0).count();

    return result;
}

} // namespace midgard
