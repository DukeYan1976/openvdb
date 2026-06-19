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

PointBuffer IPWBuilder::sampleBoundary(const GeometryDef& geom, const openvdb::BBoxd& region) {
    PointBuffer buffer;
    
    if (geom.type == GeometryDef::BOX) {
        Vec3d bmin = geom.origin;
        Vec3d bmax = geom.origin + geom.dims;
        
        struct Face { Vec3d normal; double val; int axis; };
        Face faces[6] = {
            {{-1, 0, 0}, bmin.x(), 0}, {{1, 0, 0}, bmax.x(), 0},
            {{0, -1, 0}, bmin.y(), 1}, {{0, 1, 0}, bmax.y(), 1},
            {{0, 0, -1}, bmin.z(), 2}, {{0, 0, 1}, bmax.z(), 2}
        };

        for (const auto& f : faces) {
            if (f.val >= region.min()[f.axis] && f.val <= region.max()[f.axis]) {
                int a1 = (f.axis + 1) % 3;
                int a2 = (f.axis + 2) % 3;
                
                double intersect_min1 = std::max(region.min()[a1], bmin[a1]);
                double intersect_max1 = std::min(region.max()[a1], bmax[a1]);
                double intersect_min2 = std::max(region.min()[a2], bmin[a2]);
                double intersect_max2 = std::min(region.max()[a2], bmax[a2]);
                
                if (intersect_min1 <= intersect_max1 && intersect_min2 <= intersect_max2) {
                    Vec3d p;
                    p[f.axis] = f.val;
                    p[a1] = (intersect_min1 + intersect_max1) * 0.5;
                    p[a2] = (intersect_min2 + intersect_max2) * 0.5;
                    
                    buffer.positions.push_back(Vec3f(p));
                    buffer.normals.push_back(Vec3f(f.normal));
                }
            }
        }
    }
    
    return buffer;
}

IPWState IPWBuilder::build(const GeometryDef& geom, const ToleranceConfig& config) {
    IPWState ipw(config);
    ipw.billetDef = geom; // 存储原始定义

    ipw.macroGrid = buildMacroGrid(geom, config);
    if (!ipw.macroGrid) return ipw;

    // MicroGrid: 创建空grid（保留transform用于后续切削写入）
    ipw.microGrid = openvdb::points::PointDataGrid::create();
    ipw.microGrid->setTransform(ipw.macroGrid->transformPtr());

    return ipw;
}

} // namespace midgard
