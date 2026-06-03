#pragma once

#include <vector>
#include <openvdb/openvdb.h>

namespace ygg {

struct MeshData {
    std::vector<float> vertices;  // x,y,z,nx,ny,nz interleaved
    std::vector<uint32_t> indices;
};

/// Extract triangle mesh from OpenVDB level set grid
MeshData vdbToMesh(const openvdb::FloatGrid::Ptr& grid,
                   double isovalue = 0.0, double adaptivity = 0.0);

} // namespace ygg
