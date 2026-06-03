#include "VDBToMesh.h"
#include <openvdb/tools/VolumeToMesh.h>

namespace ygg {

MeshData vdbToMesh(const openvdb::FloatGrid::Ptr& grid,
                   double isovalue, double adaptivity) {
    MeshData mesh;

    std::vector<openvdb::Vec3s> points;
    std::vector<openvdb::Vec3I> triangles;
    std::vector<openvdb::Vec4I> quads;

    openvdb::tools::volumeToMesh(*grid, points, triangles, quads,
                                  isovalue, adaptivity);

    // Reserve: each quad becomes 2 triangles
    size_t triCount = triangles.size() + quads.size() * 2;
    mesh.indices.reserve(triCount * 3);
    mesh.vertices.resize(points.size() * 6); // pos + normal (zero init)

    // Copy positions (normals computed later)
    for (size_t i = 0; i < points.size(); ++i) {
        mesh.vertices[i * 6 + 0] = points[i].x();
        mesh.vertices[i * 6 + 1] = points[i].y();
        mesh.vertices[i * 6 + 2] = points[i].z();
    }

    // Triangles
    for (auto& tri : triangles) {
        mesh.indices.push_back(tri[0]);
        mesh.indices.push_back(tri[1]);
        mesh.indices.push_back(tri[2]);
    }

    // Quads → 2 triangles each
    for (auto& q : quads) {
        mesh.indices.push_back(q[0]);
        mesh.indices.push_back(q[1]);
        mesh.indices.push_back(q[2]);
        mesh.indices.push_back(q[0]);
        mesh.indices.push_back(q[2]);
        mesh.indices.push_back(q[3]);
    }

    // Compute per-vertex normals (area-weighted face normals)
    for (size_t i = 0; i < mesh.indices.size(); i += 3) {
        uint32_t i0 = mesh.indices[i], i1 = mesh.indices[i+1], i2 = mesh.indices[i+2];
        float* v0 = &mesh.vertices[i0 * 6];
        float* v1 = &mesh.vertices[i1 * 6];
        float* v2 = &mesh.vertices[i2 * 6];

        float ex = v1[0]-v0[0], ey = v1[1]-v0[1], ez = v1[2]-v0[2];
        float fx = v2[0]-v0[0], fy = v2[1]-v0[1], fz = v2[2]-v0[2];
        float nx = ey*fz - ez*fy, ny = ez*fx - ex*fz, nz = ex*fy - ey*fx;

        for (uint32_t idx : {i0, i1, i2}) {
            mesh.vertices[idx*6+3] += nx;
            mesh.vertices[idx*6+4] += ny;
            mesh.vertices[idx*6+5] += nz;
        }
    }

    // Normalize
    for (size_t i = 0; i < points.size(); ++i) {
        float* n = &mesh.vertices[i*6+3];
        float len = std::sqrt(n[0]*n[0] + n[1]*n[1] + n[2]*n[2]);
        if (len > 1e-8f) { n[0]/=len; n[1]/=len; n[2]/=len; }
    }

    return mesh;
}

} // namespace ygg
