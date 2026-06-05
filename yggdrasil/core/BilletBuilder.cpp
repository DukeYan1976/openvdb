#include "core/BilletBuilder.h"
#include <openvdb/tools/LevelSetUtil.h>

namespace ygg {

BilletModel buildBillet(const ResolutionConfig& config,
                        const Vec3d& origin, const Vec3d& dims) {
    BilletModel model;
    model.config = config;
    model.origin = origin;
    model.dims = dims;
    model.geometry = {GeometryDef::BOX, origin, dims, 0, 0};

    if (config.mode == ResolutionConfig::SINGLE_TRACK) {
        // 单轨：d_v 精度的完整 SDF
        auto xform = openvdb::math::Transform::createLinearTransform(config.d_v);
        openvdb::math::BBox<Vec3d> bbox(origin, origin + dims);
        model.sdfGrid = openvdb::tools::createLevelSetBox<openvdb::FloatGrid>(bbox, *xform);
        model.sdfGrid->setName("billet");
        model.microGrid = nullptr;
        model.dirtyMask = nullptr;
    } else {
        // 双轨：D_v 精度的宏观 SDF + 空 dirtyMask，零面元
        auto xform = openvdb::math::Transform::createLinearTransform(config.D_v);
        openvdb::math::BBox<Vec3d> bbox(origin, origin + dims);
        model.sdfGrid = openvdb::tools::createLevelSetBox<openvdb::FloatGrid>(bbox, *xform);
        model.sdfGrid->setName("billet");

        // 延迟面元化：microGrid 初始为空
        model.microGrid = nullptr;

        // 脏区标记（共享 Transform）
        model.dirtyMask = openvdb::MaskGrid::create(false);
        model.dirtyMask->setTransform(model.sdfGrid->transformPtr());
    }

    return model;
}

} // namespace ygg
