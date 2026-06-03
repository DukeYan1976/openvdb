#include "core/BilletBuilder.h"
#include <openvdb/tools/LevelSetUtil.h>
#include <openvdb/tools/MeshToVolume.h>
#include <openvdb/points/PointConversion.h>
#include <openvdb/points/PointCount.h>
#include <openvdb/points/PointAttribute.h>
#include <cmath>
#include <vector>

namespace ygg {

// 使用 OpenVDB 内置 createLevelSetBox（内部已优化+并行）
static openvdb::FloatGrid::Ptr buildBoxSDF(double voxelSize, const Vec3d& origin, const Vec3d& dims) {
    auto xform = openvdb::math::Transform::createLinearTransform(voxelSize);
    openvdb::math::BBox<openvdb::Vec3d> bbox(origin, origin + dims);
    auto grid = openvdb::tools::createLevelSetBox<openvdb::FloatGrid>(bbox, *xform);
    grid->setName("billet");
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
        double d_v_init = std::max(10.0 * config.d_v, D_v); // ≥ 10×d_v
        int N_init = std::max(2, static_cast<int>(std::floor(D_v / d_v_init)));
        // 如果 d_v_init ≥ D_v, N_init=1 → 强制至少 2
        // 实际每体素面元数 = N_init²
        float bandWidth = 3.0f * static_cast<float>(D_v);

        std::vector<openvdb::Vec3R> allPoints;
        std::vector<openvdb::Vec3f> allNormals;
        std::vector<uint8_t> allPrecision;

        auto& xform = *model.sdfGrid->transformPtr();
        auto sdfAcc = model.sdfGrid->getConstAccessor();

        // 遍历 FloatGrid 窄带表面体素
        for (auto iter = model.sdfGrid->cbeginValueOn(); iter; ++iter) {
            float sdfVal = *iter;
            if (sdfVal >= -static_cast<float>(D_v) && sdfVal <= static_cast<float>(D_v)) {
                openvdb::Coord voxelCoord = iter.getCoord();
                Vec3d voxelCenter = xform.indexToWorld(voxelCoord);

                // SDF 梯度 → 法向量
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

                // 构建局部切平面坐标系
                Vec3d n(normal.x(), normal.y(), normal.z());
                Vec3d u, v;
                if (std::abs(n.x()) < 0.9) u = Vec3d(1,0,0).cross(n);
                else u = Vec3d(0,1,0).cross(n);
                u.normalize();
                v = n.cross(u);

                // 在切平面上做 N_init × N_init 采样
                double step = D_v / N_init;
                for (int i = 0; i < N_init; ++i) {
                    for (int j = 0; j < N_init; ++j) {
                        Vec3d offset = ((i + 0.5 - N_init/2.0) * step) * u +
                                       ((j + 0.5 - N_init/2.0) * step) * v;
                        Vec3d candidate = voxelCenter + offset;
                        // 投影到零等值面
                        candidate = candidate - n * (double)sdfVal;

                        allPoints.push_back(candidate);
                        allNormals.push_back(normal);
                        allPrecision.push_back(0); // COARSE
                    }
                }
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
