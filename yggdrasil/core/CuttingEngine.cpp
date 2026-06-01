#include "core/CuttingEngine.h"
#include <openvdb/tools/Prune.h>
#include <cmath>

namespace ygg {

void CuttingEngine::cut(BilletModel& billet, const ToolSweepSDF& toolSDF) {
    auto& grid = billet.sdfGrid;
    auto& xform = grid->transform();
    double voxelSize = xform.voxelSize()[0];
    float bandWidth = 3.0f * static_cast<float>(voxelSize);

    // 计算刀具包围盒在索引空间的范围
    auto bbox = toolSDF.getBoundingBox();
    auto minIdx = xform.worldToIndexCellCentered(bbox.min());
    auto maxIdx = xform.worldToIndexCellCentered(bbox.max());

    auto accessor = grid->getAccessor();

    // 遍历包围盒内的体素，执行 SDF 布尔差集: max(billet, -tool)
    openvdb::Coord ijk;
    for (ijk[0] = minIdx[0]; ijk[0] <= maxIdx[0]; ++ijk[0]) {
        for (ijk[1] = minIdx[1]; ijk[1] <= maxIdx[1]; ++ijk[1]) {
            for (ijk[2] = minIdx[2]; ijk[2] <= maxIdx[2]; ++ijk[2]) {
                Vec3d worldPos = xform.indexToWorld(ijk);
                double toolDist = toolSDF.eval(worldPos);

                // 仅处理刀具内部或表面附近的体素
                if (toolDist < bandWidth) {
                    float billetVal = accessor.getValue(ijk);
                    float newVal = std::max(billetVal, static_cast<float>(-toolDist));

                    if (newVal != billetVal) {
                        accessor.setValue(ijk, newVal);
                    }
                }
            }
        }
    }

    // 剪枝：移除远离表面的体素
    openvdb::tools::pruneLevelSet(grid->tree());
}

double computeVolume(const openvdb::FloatGrid::Ptr& grid) {
    double voxelVol = std::pow(grid->voxelSize()[0], 3);
    size_t count = 0;
    for (auto iter = grid->cbeginValueOn(); iter; ++iter) {
        if (*iter < 0.0f) ++count;
    }
    return static_cast<double>(count) * voxelVol;
}

} // namespace ygg
