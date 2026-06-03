#pragma once

#include <openvdb/openvdb.h>
#include <openvdb/points/PointDataGrid.h>
#include <openvdb/points/PointCount.h>
#include <cstdio>

namespace ygg {

struct MemoryStats {
    size_t floatGridBytes = 0;
    size_t pointGridBytes = 0;
    size_t totalBytes = 0;
    size_t pointCount = 0;
    size_t activePointCount = 0;
    size_t leafNodeCount = 0;

    void update(const openvdb::FloatGrid::Ptr& sdf,
                const openvdb::points::PointDataGrid::Ptr& pts) {
        floatGridBytes = sdf ? sdf->memUsage() : 0;
        pointGridBytes = pts ? pts->memUsage() : 0;
        totalBytes = floatGridBytes + pointGridBytes;
        if (pts) {
            pointCount = openvdb::points::pointCount(pts->tree());
            activePointCount = openvdb::points::pointCount(pts->tree(),
                openvdb::points::ActiveFilter());
            leafNodeCount = pts->tree().leafCount();
        } else {
            pointCount = activePointCount = leafNodeCount = 0;
        }
    }

    void print() const {
        std::printf("[MemStats] FloatGrid: %.2f MB | PointGrid: %.2f MB | Total: %.2f MB\n",
                    floatGridBytes / 1e6, pointGridBytes / 1e6, totalBytes / 1e6);
        std::printf("[MemStats] Points: %zu (active: %zu) | Leaves: %zu\n",
                    pointCount, activePointCount, leafNodeCount);
    }
};

} // namespace ygg
