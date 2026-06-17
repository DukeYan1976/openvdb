#include "core/IPWBuilder.h"
#include <openvdb/tools/MeshToVolume.h>
#include <openvdb/points/PointConversion.h>
#include <openvdb/points/PointAttribute.h>
#include <cmath>

namespace midgard {

openvdb::FloatGrid::Ptr IPWBuilder::buildMacroGrid(
    const GeometryDef& geom, const ToleranceConfig& config)
{
    openvdb::math::Transform::Ptr xform =
        openvdb::math::Transform::createLinearTransform(config.voxelMacro);

    switch (geom.type) {
        case GeometryDef::BOX: {
            openvdb::math::BBox<openvdb::Vec3d> bbox(
                geom.origin, geom.origin + geom.dims);
            return openvdb::tools::createLevelSetBox<openvdb::FloatGrid>(
                bbox, *xform, config.halfwidth);
        }
        default:
            // TODO: CYLINDER, SPHERE, MESH
            return nullptr;
    }
}

openvdb::points::PointDataGrid::Ptr IPWBuilder::buildMicroGrid(
    const openvdb::FloatGrid::Ptr& macroGrid,
    const GeometryDef& geom,
    double t_billet)
{
    const double V = macroGrid->voxelSize()[0];

    // 收集表面点和法线
    std::vector<openvdb::Vec3f> positions;
    std::vector<openvdb::Vec3f> normals;

    auto acc = macroGrid->getConstAccessor();
    const auto& xform = macroGrid->transform();

    for (auto iter = macroGrid->cbeginValueOn(); iter; ++iter) {
        const float sdfVal = iter.getValue();
        // 表面 voxel: |SDF| < V
        if (std::abs(sdfVal) >= V) continue;

        const openvdb::Coord coord = iter.getCoord();
        const openvdb::Vec3d center = xform.indexToWorld(coord);

        // 计算梯度 (中心差分)
        openvdb::Vec3d grad;
        grad.x() = (acc.getValue(coord.offsetBy(1,0,0)) - acc.getValue(coord.offsetBy(-1,0,0))) / (2.0 * V);
        grad.y() = (acc.getValue(coord.offsetBy(0,1,0)) - acc.getValue(coord.offsetBy(0,-1,0))) / (2.0 * V);
        grad.z() = (acc.getValue(coord.offsetBy(0,0,1)) - acc.getValue(coord.offsetBy(0,0,-1))) / (2.0 * V);

        double gradLen = grad.length();
        if (gradLen < 1e-10) continue;

        openvdb::Vec3d normal = grad / gradLen;

        // 投影到零等值面
        openvdb::Vec3d surfacePoint = center - sdfVal * normal;

        positions.push_back(openvdb::Vec3f(surfacePoint));
        normals.push_back(openvdb::Vec3f(normal));
    }

    if (positions.empty()) return nullptr;

    // 构建 PointDataGrid
    // 使用与MacroGrid相同的Transform
    openvdb::points::PointAttributeVector<openvdb::Vec3f> posWrapper(positions);
    auto pointGrid = openvdb::points::createPointDataGrid<
        openvdb::points::NullCodec, openvdb::points::PointDataGrid>(
        positions, xform);

    // 附加法线属性
    openvdb::points::appendAttribute<openvdb::Vec3f>(
        pointGrid->tree(), "N");

    // 写入法线
    size_t idx = 0;
    for (auto leaf = pointGrid->tree().beginLeaf(); leaf; ++leaf) {
        openvdb::points::AttributeWriteHandle<openvdb::Vec3f> handle(
            leaf->attributeArray("N"));
        for (auto it = leaf->beginIndexOn(); it; ++it) {
            handle.set(*it, normals[idx++]);
        }
    }

    return pointGrid;
}

IPWState IPWBuilder::build(const GeometryDef& geom, const ToleranceConfig& config) {
    IPWState ipw(config.user_t);
    ipw.config = config;

    ipw.macroGrid = buildMacroGrid(geom, config);
    if (!ipw.macroGrid) return ipw;

    double t_billet = 2.0 * config.user_t;
    ipw.microGrid = buildMicroGrid(ipw.macroGrid, geom, t_billet);

    return ipw;
}

} // namespace midgard
