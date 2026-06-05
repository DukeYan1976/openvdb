#include "core/SurfelGenerator.h"
#include <cmath>

namespace ygg {

SurfelBatch SurfelGenerator::sampleToolSurface(
    const ToolSweepSDF& tool,
    const std::vector<openvdb::Coord>& dirtyVoxels,
    const openvdb::math::Transform& xform,
    double d_v, int N)
{
    SurfelBatch batch;
    batch.positions.reserve(dirtyVoxels.size() * N * N);
    batch.normals.reserve(dirtyVoxels.size() * N * N);

    for (auto& coord : dirtyVoxels) {
        Vec3d center = xform.indexToWorld(coord);

        // Tool surface normal via gradient
        Vec3d grad = tool.gradient(center);
        double gl = grad.length();
        if (gl < 1e-10) continue;
        Vec3d normal = grad / gl;

        // Tangent plane basis
        Vec3d u = (std::abs(normal.x()) < 0.9)
            ? Vec3d(1, 0, 0).cross(normal)
            : Vec3d(0, 1, 0).cross(normal);
        u.normalize();
        Vec3d v = normal.cross(u);

        // N×N sampling at d_v spacing
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                Vec3d offset = ((i + 0.5 - N * 0.5) * d_v) * u
                             + ((j + 0.5 - N * 0.5) * d_v) * v;
                Vec3d candidate = center + offset;

                // Newton projection onto tool zero-isosurface
                double dist = tool.eval(candidate);
                candidate -= dist * normal;

                // Validate: close to zero-isosurface
                if (std::abs(tool.eval(candidate)) < 0.5 * d_v) {
                    batch.positions.push_back(candidate);
                    batch.normals.emplace_back(
                        float(normal.x()), float(normal.y()), float(normal.z()));
                }
            }
        }
    }
    return batch;
}

} // namespace ygg
