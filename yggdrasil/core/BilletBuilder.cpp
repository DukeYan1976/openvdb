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
    float bandWidth = halfWidth * static_cast<float>(voxelSize);
    auto xform = openvdb::math::Transform::createLinearTransform(voxelSize);
    auto grid = openvdb::FloatGrid::create(bandWidth);
    grid->setTransform(xform);
    grid->setGridClass(openvdb::GRID_LEVEL_SET);
    grid->setName("billet");

    auto accessor = grid->getAccessor();
    Vec3d boxMin = origin, boxMax = origin + dims;

    // 只遍历窄带区域：距六面 ±halfWidth 范围内的体素
    // 外扩 halfWidth 层，内缩 halfWidth 层
    auto outerMin = xform->worldToIndexCellCentered(boxMin - Vec3d(bandWidth));
    auto outerMax = xform->worldToIndexCellCentered(boxMax + Vec3d(bandWidth));
    auto innerMin = xform->worldToIndexCellCentered(boxMin + Vec3d(bandWidth));
    auto innerMax = xform->worldToIndexCellCentered(boxMax - Vec3d(bandWidth));

    // 先填充内部为负背景值（用 fill 整块操作）
    if (innerMin.x() < innerMax.x() && innerMin.y() < innerMax.y() && innerMin.z() < innerMax.z()) {
        grid->fill(openvdb::CoordBBox(innerMin, innerMax), -bandWidth, /*active=*/false);
    }

    // 只遍历窄带层（6 个面的薄壳区域）
    openvdb::Coord ijk;
    for (ijk[0] = outerMin[0]; ijk[0] <= outerMax[0]; ++ijk[0]) {
        for (ijk[1] = outerMin[1]; ijk[1] <= outerMax[1]; ++ijk[1]) {
            for (ijk[2] = outerMin[2]; ijk[2] <= outerMax[2]; ++ijk[2]) {
                // 跳过深内部（已经 fill 了）
                if (ijk[0] > innerMin[0] && ijk[0] < innerMax[0] &&
                    ijk[1] > innerMin[1] && ijk[1] < innerMax[1] &&
                    ijk[2] > innerMin[2] && ijk[2] < innerMax[2])
                    continue;

                Vec3d world = xform->indexToWorld(ijk);
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

                if (std::abs(dist) < bandWidth) {
                    accessor.setValue(ijk, dist);
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
        // Step 1: 构建 FloatGrid SDF（共享 Transform，VoxelSize = D_v）
        model.sdfGrid = buildBoxSDF(config.D_v, origin, dims);

        // Step 2: 逐体素注入 IPW₀ 粗面元
        double D_v = config.D_v;
        double minDim = std::min({dims.x(), dims.y(), dims.z()});
        double d_v_init = std::min(minDim / 4.0, std::max(10.0 * config.d_v, D_v));
        float bandWidth = 3.0f * static_cast<float>(D_v);

        // 收集点（按体素分组，确保每个点落在正确的体素中）
        std::vector<openvdb::Vec3R> allPoints;
        std::vector<openvdb::Vec3f> allNormals;
        std::vector<uint8_t> allPrecision;

        auto& xform = *model.sdfGrid->transformPtr();
        auto sdfAcc = model.sdfGrid->getConstAccessor();

        // 遍历 FloatGrid 窄带表面体素
        for (auto iter = model.sdfGrid->cbeginValueOn(); iter; ++iter) {
            float sdfVal = *iter;
            // 表面体素判定：SDF 接近零（-D_v < SDF < D_v）
            if (sdfVal >= -static_cast<float>(D_v) && sdfVal <= static_cast<float>(D_v)) {
                openvdb::Coord voxelCoord = iter.getCoord();
                Vec3d voxelCenter = xform.indexToWorld(voxelCoord);

                // 估算表面法向（从 SDF 梯度，中心差分）
                float gx = sdfAcc.getValue(voxelCoord.offsetBy(1,0,0)) -
                           sdfAcc.getValue(voxelCoord.offsetBy(-1,0,0));
                float gy = sdfAcc.getValue(voxelCoord.offsetBy(0,1,0)) -
                           sdfAcc.getValue(voxelCoord.offsetBy(0,-1,0));
                float gz = sdfAcc.getValue(voxelCoord.offsetBy(0,0,1)) -
                           sdfAcc.getValue(voxelCoord.offsetBy(0,0,-1));
                openvdb::Vec3f normal(gx, gy, gz);
                float nlen = normal.length();
                if (nlen > 1e-6f) normal /= nlen;
                else normal = openvdb::Vec3f(0, 0, 1);

                // 在该体素表面放 1 个面元（粗精度 IPW₀）
                // 投影到零等值面：沿法向偏移 -sdfVal
                Vec3d surfelPos = voxelCenter - Vec3d(normal.x(), normal.y(), normal.z()) * sdfVal;

                allPoints.push_back(surfelPos);
                allNormals.push_back(normal);
                allPrecision.push_back(0); // COARSE
            }
        }

        // Step 3: 创建 PointDataGrid（共享 Transform）
        if (!allPoints.empty()) {
            model.microGrid = openvdb::points::createPointDataGrid<
                openvdb::points::NullCodec, openvdb::points::PointDataGrid>(
                allPoints, xform);
        } else {
            model.microGrid = openvdb::points::PointDataGrid::create();
            model.microGrid->setTransform(model.sdfGrid->transformPtr());
        }
        model.microGrid->setName("micro_surfels");

        // Step 4: 附加属性（normal, precision）+ Group "active"
        openvdb::points::TypedAttributeArray<openvdb::Vec3f>::registerType();
        openvdb::points::TypedAttributeArray<uint8_t>::registerType();

        auto& tree = model.microGrid->tree();
        size_t globalIdx = 0;
        for (auto leaf = tree.beginLeaf(); leaf; ++leaf) {
            auto attrSet = leaf->stealAttributeSet();

            // Append normal
            if (attrSet->descriptor().find("normal") == openvdb::points::AttributeSet::INVALID_POS) {
                attrSet->appendAttribute("normal",
                    openvdb::points::TypedAttributeArray<openvdb::Vec3f>::attributeType(),
                    static_cast<openvdb::Index>(leaf->pointCount()));
            }
            // Append precision
            if (attrSet->descriptor().find("precision") == openvdb::points::AttributeSet::INVALID_POS) {
                attrSet->appendAttribute("precision",
                    openvdb::points::TypedAttributeArray<uint8_t>::attributeType(),
                    static_cast<openvdb::Index>(leaf->pointCount()));
            }
            // Append group "active" (as uint8 for now; Group migration deferred)
            if (attrSet->descriptor().find("active") == openvdb::points::AttributeSet::INVALID_POS) {
                attrSet->appendAttribute("active",
                    openvdb::points::TypedAttributeArray<uint8_t>::attributeType(),
                    static_cast<openvdb::Index>(leaf->pointCount()));
            }

            leaf->replaceAttributeSet(attrSet.release(), true);

            // Write normal + precision + active values
            {
                auto attrSetW = leaf->stealAttributeSet();
                auto* normalArr = attrSetW->get("normal");
                auto* precArr = attrSetW->get("precision");
                auto* activeArr = attrSetW->get("active");
                auto whN = openvdb::points::AttributeWriteHandle<openvdb::Vec3f>::create(*normalArr);
                auto whP = openvdb::points::AttributeWriteHandle<uint8_t>::create(*precArr);
                auto whA = openvdb::points::AttributeWriteHandle<uint8_t>::create(*activeArr);

                for (size_t i = 0; i < whN->size(); ++i) {
                    if (globalIdx < allNormals.size()) {
                        whN->set(i, allNormals[globalIdx]);
                        whP->set(i, allPrecision[globalIdx]);
                        whA->set(i, 1); // all active
                    }
                    globalIdx++;
                }
                leaf->replaceAttributeSet(attrSetW.release(), true);
            }
        }
    }

    return model;
}

} // namespace ygg
