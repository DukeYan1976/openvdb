#include "core/BilletBuilder.h"
#include <openvdb/tools/LevelSetUtil.h>
#include <openvdb/points/PointConversion.h>
#include <openvdb/points/PointCount.h>
#include <openvdb/points/PointAttribute.h>
#include <cmath>
#include <vector>

namespace ygg {

// 手动长方体 SDF 构建（共用）
static openvdb::FloatGrid::Ptr buildBoxSDF(double voxelSize, const Vec3d& origin, const Vec3d& dims) {
    float halfWidth = 3.0f;
    auto xform = openvdb::math::Transform::createLinearTransform(voxelSize);
    auto grid = openvdb::FloatGrid::create(halfWidth * static_cast<float>(voxelSize));
    grid->setTransform(xform);
    grid->setGridClass(openvdb::GRID_LEVEL_SET);
    grid->setName("billet");

    auto minIdx = xform->worldToIndexCellCentered(origin - Vec3d(halfWidth * voxelSize));
    auto maxIdx = xform->worldToIndexCellCentered(origin + dims + Vec3d(halfWidth * voxelSize));

    auto accessor = grid->getAccessor();
    openvdb::Coord ijk;
    for (ijk[0] = minIdx[0]; ijk[0] <= maxIdx[0]; ++ijk[0]) {
        for (ijk[1] = minIdx[1]; ijk[1] <= maxIdx[1]; ++ijk[1]) {
            for (ijk[2] = minIdx[2]; ijk[2] <= maxIdx[2]; ++ijk[2]) {
                Vec3d world = xform->indexToWorld(ijk);
                Vec3d boxMin = origin, boxMax = origin + dims;
                double dx = std::max(boxMin.x() - world.x(), world.x() - boxMax.x());
                double dy = std::max(boxMin.y() - world.y(), world.y() - boxMax.y());
                double dz = std::max(boxMin.z() - world.z(), world.z() - boxMax.z());

                float dist;
                if (dx <= 0 && dy <= 0 && dz <= 0) {
                    dist = static_cast<float>(std::max({dx, dy, dz}));
                } else {
                    double ex = std::max(dx, 0.0), ey = std::max(dy, 0.0), ez = std::max(dz, 0.0);
                    dist = static_cast<float>(std::sqrt(ex*ex + ey*ey + ez*ez));
                }

                float bandWidth = halfWidth * static_cast<float>(voxelSize);
                if (std::abs(dist) < bandWidth) {
                    accessor.setValue(ijk, dist);
                } else if (dist < 0) {
                    accessor.setValue(ijk, -bandWidth);
                }
            }
        }
    }
    return grid;
}

// 为长方体的6个面生成粗面元点
static void generateIPW0Surfels(
    const openvdb::FloatGrid::Ptr& sdfGrid,
    const Vec3d& origin, const Vec3d& dims,
    double D_v, double d_v_init,
    std::vector<openvdb::Vec3R>& outPoints,
    std::vector<openvdb::Vec3f>& outNormals,
    std::vector<uint8_t>& outPrecision)
{
    // 对长方体6面直接解析采样（无需梯度计算）
    struct Face { int axis; double pos; double sign; double w, h; Vec3d corner; };
    std::vector<Face> faces = {
        {2, origin.z(),          -1, dims.x(), dims.y(), origin},                          // Z_MIN
        {2, origin.z()+dims.z(), +1, dims.x(), dims.y(), {origin.x(), origin.y(), origin.z()+dims.z()}}, // Z_MAX
        {1, origin.y(),          -1, dims.x(), dims.z(), origin},                          // Y_MIN
        {1, origin.y()+dims.y(), +1, dims.x(), dims.z(), {origin.x(), origin.y()+dims.y(), origin.z()}}, // Y_MAX
        {0, origin.x(),          -1, dims.y(), dims.z(), origin},                          // X_MIN
        {0, origin.x()+dims.x(), +1, dims.y(), dims.z(), {origin.x()+dims.x(), origin.y(), origin.z()}}, // X_MAX
    };

    for (auto& f : faces) {
        int nu = static_cast<int>(std::ceil(f.w / d_v_init));
        int nv = static_cast<int>(std::ceil(f.h / d_v_init));
        double du = f.w / nu, dv = f.h / nv;

        for (int i = 0; i < nu; ++i) {
            for (int j = 0; j < nv; ++j) {
                openvdb::Vec3R p;
                double u = (i + 0.5) * du;
                double v = (j + 0.5) * dv;

                if (f.axis == 2) {       // Z面
                    p = {origin.x() + u, origin.y() + v, f.pos};
                } else if (f.axis == 1) { // Y面
                    p = {origin.x() + u, f.pos, origin.z() + v};
                } else {                  // X面
                    p = {f.pos, origin.y() + u, origin.z() + v};
                }

                outPoints.push_back(p);
                openvdb::Vec3f normal(0);
                normal[f.axis] = static_cast<float>(f.sign);
                outNormals.push_back(normal);
                outPrecision.push_back(0); // COARSE
            }
        }
    }
}

BilletModel buildBillet(const ResolutionConfig& config,
                        const Vec3d& origin, const Vec3d& dims) {
    BilletModel model;
    model.config = config;
    model.origin = origin;
    model.dims = dims;

    if (config.mode == ResolutionConfig::SINGLE_TRACK) {
        model.sdfGrid = buildBoxSDF(config.d_v, origin, dims);
        model.microGrid = nullptr;
    }
    else if (config.mode == ResolutionConfig::DUAL_TRACK) {
        // Step 1: 构建 FloatGrid SDF（共享 Transform 的 VoxelSize = D_v）
        model.sdfGrid = buildBoxSDF(config.D_v, origin, dims);

        // Step 2: 生成 IPW₀ 粗面元
        double d_v_init = std::max(10.0 * config.d_v, config.D_v);
        std::vector<openvdb::Vec3R> points;
        std::vector<openvdb::Vec3f> normals;
        std::vector<uint8_t> precision;
        generateIPW0Surfels(model.sdfGrid, origin, dims, config.D_v, d_v_init,
                            points, normals, precision);

        // Step 3: 创建 PointDataGrid（共享 Transform）
        auto xform = model.sdfGrid->transformPtr();
        model.microGrid = openvdb::points::createPointDataGrid<openvdb::points::NullCodec,
            openvdb::points::PointDataGrid>(points, *xform);
        model.microGrid->setName("micro_surfels");

        // Step 4: 注册并附加 normal + precision + active 属性
        openvdb::points::TypedAttributeArray<openvdb::Vec3f>::registerType();
        openvdb::points::TypedAttributeArray<uint8_t>::registerType();

        auto& tree = model.microGrid->tree();
        for (auto leaf = tree.beginLeaf(); leaf; ++leaf) {
            auto attrSet = leaf->stealAttributeSet();

            // Append normal (Vec3f)
            if (attrSet->descriptor().find("normal") == openvdb::points::AttributeSet::INVALID_POS) {
                attrSet->appendAttribute("normal",
                    openvdb::points::TypedAttributeArray<openvdb::Vec3f>::attributeType(),
                    static_cast<openvdb::Index>(leaf->pointCount()));
            }
            // Append precision (uint8)
            if (attrSet->descriptor().find("precision") == openvdb::points::AttributeSet::INVALID_POS) {
                attrSet->appendAttribute("precision",
                    openvdb::points::TypedAttributeArray<uint8_t>::attributeType(),
                    static_cast<openvdb::Index>(leaf->pointCount()));
            }
            // Append active (uint8)
            if (attrSet->descriptor().find("active") == openvdb::points::AttributeSet::INVALID_POS) {
                attrSet->appendAttribute("active",
                    openvdb::points::TypedAttributeArray<uint8_t>::attributeType(),
                    static_cast<openvdb::Index>(leaf->pointCount()));
            }

            leaf->replaceAttributeSet(attrSet.release(), true);
        }

        // Step 5: 写入属性值
        size_t globalIdx = 0;
        for (auto leaf = tree.beginLeaf(); leaf; ++leaf) {
            auto attrSetPtr = leaf->stealAttributeSet();

            auto* normalArr = attrSetPtr->get("normal");
            auto* precArr = attrSetPtr->get("precision");
            auto* activeArr = attrSetPtr->get("active");

            auto whNormal = openvdb::points::AttributeWriteHandle<openvdb::Vec3f>::create(*normalArr);
            auto whPrec = openvdb::points::AttributeWriteHandle<uint8_t>::create(*precArr);
            auto whActive = openvdb::points::AttributeWriteHandle<uint8_t>::create(*activeArr);

            for (size_t i = 0; i < whNormal->size(); ++i) {
                if (globalIdx < normals.size()) {
                    whNormal->set(i, normals[globalIdx]);
                    whPrec->set(i, precision[globalIdx]);
                    whActive->set(i, 1); // all active
                }
                globalIdx++;
            }

            leaf->replaceAttributeSet(attrSetPtr.release(), true);
        }
    }

    return model;
}

} // namespace ygg
