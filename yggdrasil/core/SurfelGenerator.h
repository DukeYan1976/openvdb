#pragma once

#include "types/YggTypes.h"
#include "core/ToolSweepSDF.h"
#include <vector>

namespace ygg {

struct SurfelBatch {
    std::vector<openvdb::Vec3R> positions;
    std::vector<openvdb::Vec3f> normals;
};

class SurfelGenerator {
public:
    /// Sample surfels on the tool zero-isosurface at dirty voxel locations.
    /// Each dirty voxel produces up to N×N surfels at d_v spacing.
    /// If billetGeo is non-null, surfels outside the billet boundary are discarded.
    static SurfelBatch sampleToolSurface(
        const ToolSweepSDF& tool,
        const std::vector<openvdb::Coord>& dirtyVoxels,
        const openvdb::math::Transform& xform,
        double d_v, int N,
        const GeometryDef* billetGeo = nullptr);
};

} // namespace ygg
