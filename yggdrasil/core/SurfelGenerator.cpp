#include "core/SurfelGenerator.h"
#include <tbb/parallel_for.h>
#include <tbb/enumerable_thread_specific.h>
#include <cmath>

namespace ygg {

SurfelBatch SurfelGenerator::sampleToolSurface(
    const ToolSweepSDF& tool,
    const std::vector<openvdb::Coord>& dirtyVoxels,
    const openvdb::math::Transform& xform,
    double d_v, int N)
{
    using BatchTLS = tbb::enumerable_thread_specific<SurfelBatch>;
    BatchTLS tlsBatches;

    tbb::parallel_for(tbb::blocked_range<size_t>(0, dirtyVoxels.size()),
        [&](const tbb::blocked_range<size_t>& r) {
            auto& batch = tlsBatches.local();
            for (size_t idx = r.begin(); idx != r.end(); ++idx) {
                auto& coord = dirtyVoxels[idx];
                Vec3d center = xform.indexToWorld(coord);

                Vec3d grad = tool.gradient(center);
                double gl = grad.length();
                if (gl < 1e-10) continue;
                Vec3d normal = grad / gl;

                Vec3d u = (std::abs(normal.x()) < 0.9)
                    ? Vec3d(1, 0, 0).cross(normal)
                    : Vec3d(0, 1, 0).cross(normal);
                u.normalize();
                Vec3d v = normal.cross(u);

                for (int i = 0; i < N; ++i) {
                    for (int j = 0; j < N; ++j) {
                        Vec3d offset = ((i + 0.5 - N * 0.5) * d_v) * u
                                     + ((j + 0.5 - N * 0.5) * d_v) * v;
                        Vec3d candidate = center + offset;

                        double dist = tool.eval(candidate);
                        candidate -= dist * normal;

                        if (std::abs(tool.eval(candidate)) < 0.5 * d_v) {
                            batch.positions.push_back(candidate);
                            batch.normals.emplace_back(
                                float(normal.x()), float(normal.y()), float(normal.z()));
                        }
                    }
                }
            }
        });

    SurfelBatch result;
    size_t totalPos = 0, totalNorm = 0;
    for (auto& b : tlsBatches) {
        totalPos += b.positions.size();
        totalNorm += b.normals.size();
    }
    result.positions.reserve(totalPos);
    result.normals.reserve(totalNorm);
    for (auto& b : tlsBatches) {
        result.positions.insert(result.positions.end(), b.positions.begin(), b.positions.end());
        result.normals.insert(result.normals.end(), b.normals.begin(), b.normals.end());
    }
    return result;
}

} // namespace ygg
