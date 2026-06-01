#include "core/BilletBuilder.h"
#include <openvdb/tools/LevelSetUtil.h>

namespace ygg {

BilletModel buildBillet(const ResolutionConfig& config,
                        const Vec3d& origin, const Vec3d& dims) {
    BilletModel model;
    model.config = config;
    model.origin = origin;
    model.dims = dims;

    double voxelSize = config.d_v;  // 单轨模式用 d_v
    if (config.mode == ResolutionConfig::DUAL_TRACK)
        voxelSize = config.D_v;

    float halfWidth = 3.0f;

    // 创建 Transform
    auto xform = openvdb::math::Transform::createLinearTransform(voxelSize);

    // 构建长方体 Level Set：
    // OpenVDB 没有 createLevelSetBox，手动用 dense fill + 距离计算
    auto grid = openvdb::FloatGrid::create(halfWidth * static_cast<float>(voxelSize));
    grid->setTransform(xform);
    grid->setGridClass(openvdb::GRID_LEVEL_SET);
    grid->setName("billet");

    // 计算索引空间的包围盒（含窄带扩展）
    auto minIdx = xform->worldToIndexCellCentered(origin - Vec3d(halfWidth * voxelSize));
    auto maxIdx = xform->worldToIndexCellCentered(origin + dims + Vec3d(halfWidth * voxelSize));

    auto accessor = grid->getAccessor();
    openvdb::Coord ijk;

    for (ijk[0] = minIdx[0]; ijk[0] <= maxIdx[0]; ++ijk[0]) {
        for (ijk[1] = minIdx[1]; ijk[1] <= maxIdx[1]; ++ijk[1]) {
            for (ijk[2] = minIdx[2]; ijk[2] <= maxIdx[2]; ++ijk[2]) {
                Vec3d world = xform->indexToWorld(ijk);

                // 计算点到长方体的有符号距离
                Vec3d boxMin = origin;
                Vec3d boxMax = origin + dims;

                // 距离各面的距离（负值=内部）
                double dx = std::max(boxMin.x() - world.x(), world.x() - boxMax.x());
                double dy = std::max(boxMin.y() - world.y(), world.y() - boxMax.y());
                double dz = std::max(boxMin.z() - world.z(), world.z() - boxMax.z());

                float dist;
                if (dx <= 0 && dy <= 0 && dz <= 0) {
                    // 内部：距离 = 到最近面的距离（负值）
                    dist = static_cast<float>(std::max({dx, dy, dz}));
                } else {
                    // 外部：欧氏距离到最近角/边/面
                    double ex = std::max(dx, 0.0);
                    double ey = std::max(dy, 0.0);
                    double ez = std::max(dz, 0.0);
                    dist = static_cast<float>(std::sqrt(ex*ex + ey*ey + ez*ez));
                }

                // 仅存储窄带内的值
                float bandWidth = halfWidth * static_cast<float>(voxelSize);
                if (std::abs(dist) < bandWidth) {
                    accessor.setValue(ijk, dist);
                } else if (dist < 0) {
                    // 内部深处：设为负背景值
                    accessor.setValue(ijk, -bandWidth);
                }
            }
        }
    }

    model.sdfGrid = grid;
    return model;
}

} // namespace ygg
