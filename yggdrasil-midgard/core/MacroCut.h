#pragma once

#include "core/Types.h"
#include "core/ToolSweepSDF.h"
#include "debug/RtDebugSys.h"
#include <openvdb/Grid.h>
#include <vector>

namespace midgard {

struct CutClassification {
    std::vector<openvdb::Coord> deleted;    // voxel_d: 完全在扫掠体内，需删除
    std::vector<openvdb::Coord> cut;        // voxel_c: 边界voxel，CSG前已active（MicroGrid有既有点集）
    std::vector<openvdb::Coord> newBoundary;// voxel_n: 边界voxel，CSG前inactive（MicroGrid无数据，需新建）
};

class MacroCut {
public:
    /// Phase 0: CSG差集 + 三态分类
    CutClassification classifyVoxels(IPWState& ipw, const ToolSweepSDF& tool);
};

} // namespace midgard
