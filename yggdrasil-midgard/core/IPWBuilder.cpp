#include "core/IPWBuilder.h"
#include "core/IDebugDisplay.h"
#include "debug/RtDebugSys.h"
#include <openvdb/tools/MeshToVolume.h>
#include <openvdb/tree/LeafManager.h>
#include <openvdb/points/PointConversion.h>
#include <openvdb/points/PointAttribute.h>
#include <cmath>
#include <sstream>
#include <cstdio>

namespace midgard {

openvdb::FloatGrid::Ptr IPWBuilder::buildMacroGrid(
    const GeometryDef& geom, const ToleranceConfig& config)
{
    openvdb::math::Transform::Ptr xform =
        openvdb::math::Transform::createLinearTransform(config.voxelMacro);

    openvdb::FloatGrid::Ptr grid;

    switch (geom.type) {
        case GeometryDef::BOX: {
            openvdb::math::BBox<openvdb::Vec3d> bbox(
                geom.origin, geom.origin + geom.dims);
            grid = openvdb::tools::createLevelSetBox<openvdb::FloatGrid>(
                bbox, *xform, config.halfwidth);
            break;
        }
        case GeometryDef::CYLINDER: {
            // 解析法构造圆柱 SDF，避免 meshToVolume 的三角面逼近误差
            // (chordal error + 端面接缝处 SDF 不准 → 不同视角对齐偏差)
            double cx = geom.origin[0], cy = geom.origin[1];
            double cz0 = geom.origin[2], cz1 = geom.origin[2] + geom.height;
            double r  = geom.radius;
            double czMid = (cz0 + cz1) * 0.5;
            double halfH = geom.height * 0.5;
            double voxel = config.voxelMacro;
            double halfWidth = config.halfwidth * voxel; // narrow band (world units)

            // 世界空间包围盒（含 narrow band）
            openvdb::Vec3d wMin(cx - r - halfWidth, cy - r - halfWidth, cz0 - halfWidth);
            openvdb::Vec3d wMax(cx + r + halfWidth, cy + r + halfWidth, cz1 + halfWidth);
            openvdb::Vec3d iMin = xform->worldToIndex(wMin);
            openvdb::Vec3d iMax = xform->worldToIndex(wMax);

            grid = openvdb::FloatGrid::create((float)config.halfwidth);
            grid->setTransform(xform);
            auto acc = grid->getAccessor();

            openvdb::Coord i0((int)std::floor(iMin.x()), (int)std::floor(iMin.y()), (int)std::floor(iMin.z()));
            openvdb::Coord i1((int)std::ceil(iMax.x()),   (int)std::ceil(iMax.y()),   (int)std::ceil(iMax.z()));

            // Inigo Quilez capped cylinder SDF:
            //   min(max(d_xy,d_z),0) + length(max(vec2(d_xy,d_z),0))
            for (int ix = i0.x(); ix <= i1.x(); ++ix) {
            for (int iy = i0.y(); iy <= i1.y(); ++iy) {
            for (int iz = i0.z(); iz <= i1.z(); ++iz) {
                openvdb::Vec3d w = xform->indexToWorld(openvdb::Coord(ix, iy, iz));
                double dx = w.x() - cx, dy = w.y() - cy;
                double dz = w.z() - czMid;

                double d_xy = std::sqrt(dx*dx + dy*dy) - r;
                double d_z  = std::fabs(dz) - halfH;
                double d_ext = std::sqrt(std::max(d_xy, 0.0) * std::max(d_xy, 0.0) +
                                         std::max(d_z,  0.0) * std::max(d_z,  0.0));
                double d_int = std::min(std::max(d_xy, d_z), 0.0);
                double sdf = d_ext + d_int;

                if (std::fabs(sdf) < halfWidth) {
                    acc.setValue(openvdb::Coord(ix, iy, iz), (float)sdf);
                }
            }
            }
            }
            break;
        }
        default:
            return nullptr;
    }

    return grid;
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

IPWState IPWBuilder::build(const GeometryDef& geom, const ToleranceConfig& config,
                             DebugLogFn logFn, bool showBoundary) {
    IPWState ipw(config);

    ipw.macroGrid = buildMacroGrid(geom, config);
    if (!ipw.macroGrid) return ipw;

    // prune: 清除全正/全负冗余 leaf → 余下全为 boundary leaf
    ipw.macroGrid->tree().prune();

    // MicroGrid: 创建空grid（保留transform用于后续切削写入）
    ipw.microGrid = openvdb::points::PointDataGrid::create();
    ipw.microGrid->setTransform(ipw.macroGrid->transformPtr());

    // ── Debug Section: 显示所有 zero-cross voxel 的 bbox ──
    DEBUG_SECTION(IPW_BUILD) {
    if (!g_debugDisplay) return ipw;

    if (showBoundary)
        g_debugDisplay->clear();

    openvdb::tree::LeafManager<openvdb::FloatGrid::TreeType> leafMgr(ipw.macroGrid->tree());
    size_t boundaryCnt = leafMgr.leafCount();
    size_t totalActive = ipw.macroGrid->activeVoxelCount();

    const auto& xform = ipw.macroGrid->transform();
    // 三态分类判据 (和 MacroCut::classifyVoxels 一致):
    // threshold = V·√3/2 + 1e-4 (体素外接球半径)
    // - |SDF| > threshold → 外接球完全在某侧 → 确定为 Air 或 Interior (跳过)
    // - |SDF| ≤ threshold → 曲面可能穿过 → Boundary (显示)
    // 保证: 零误判 (no false positive), 可漏判 (false negative 允许)
    static constexpr double kMechEpsilon = 1e-4;  // 机械加工最小分辨率 (0.1μm)
    const double threshold = config.voxelMacro * std::sqrt(3.0) / 2.0 + kMechEpsilon;
    const double halfVoxel = config.voxelMacro * 0.5;
    const bool useBbox = (config.voxelMacro >= 0.15);
    int zeroVoxels = 0;

    std::vector<float> centers;
    std::vector<float> lines;
    centers.reserve(4096 * 3);
    lines.reserve(4096 * 72);

    for (size_t n = 0; n < boundaryCnt; ++n) {
        const auto& leaf = leafMgr.leaf(n);
        for (auto v = leaf.cbeginValueOn(); v; ++v) {
            if (std::fabs(*v) > threshold) continue;  // Air/Interior: 外接球不触曲面
            ++zeroVoxels;

            openvdb::Vec3d w = xform.indexToWorld(v.getCoord());

            // 圆柱端面 boundary 显示：端面上 SDF=0 的 voxel 构成第 1 层 disc，
            // 相邻 voxel 层（z0±V）中贴近侧壁的 voxel 构成第 2 层环。
            // SDF eps 检查自然区分了边界/内部体素，无需额外过滤。

            if (useBbox) {
                double x0 = w.x() - halfVoxel, x1 = w.x() + halfVoxel;
                double y0 = w.y() - halfVoxel, y1 = w.y() + halfVoxel;
                double z0 = w.z() - halfVoxel, z1 = w.z() + halfVoxel;
                float e[72] = {
                    (float)x0,(float)y0,(float)z0, (float)x1,(float)y0,(float)z0,
                    (float)x0,(float)y1,(float)z0, (float)x1,(float)y1,(float)z0,
                    (float)x0,(float)y0,(float)z1, (float)x1,(float)y0,(float)z1,
                    (float)x0,(float)y1,(float)z1, (float)x1,(float)y1,(float)z1,
                    (float)x0,(float)y0,(float)z0, (float)x0,(float)y1,(float)z0,
                    (float)x1,(float)y0,(float)z0, (float)x1,(float)y1,(float)z0,
                    (float)x0,(float)y0,(float)z1, (float)x0,(float)y1,(float)z1,
                    (float)x1,(float)y0,(float)z1, (float)x1,(float)y1,(float)z1,
                    (float)x0,(float)y0,(float)z0, (float)x0,(float)y0,(float)z1,
                    (float)x1,(float)y0,(float)z0, (float)x1,(float)y0,(float)z1,
                    (float)x0,(float)y1,(float)z0, (float)x0,(float)y1,(float)z1,
                    (float)x1,(float)y1,(float)z0, (float)x1,(float)y1,(float)z1,
                };
                lines.insert(lines.end(), e, e + 72);
            } else {
                centers.insert(centers.end(), {(float)w.x(), (float)w.y(), (float)w.z()});
            }
        }
    }

    if (showBoundary) {
        if (useBbox && !lines.empty())
            g_debugDisplay->drawLines(lines.data(), lines.size()/3, 0xCC00FF00);
        else if (!centers.empty())
            g_debugDisplay->drawPoints(centers.data(), centers.size()/3, 0xFF4488FF);
    }

    if (logFn) {
        const char* typeName = (geom.type == GeometryDef::BOX) ? "Box" : "Cylinder";
        char buf[256];
        if (geom.type == GeometryDef::BOX) {
            std::snprintf(buf, sizeof(buf),
                "IPW0 rebuilt [%s]: %.0fx%.0fx%.0f @ (%.1f,%.1f,%.1f) bbox=(%.1f-%.1f) — %zu leaves, %zu boundary voxels, %zu total voxels",
                typeName,
                geom.dims[0], geom.dims[1], geom.dims[2],
                geom.origin[0], geom.origin[1], geom.origin[2],
                geom.origin[0], geom.origin[0]+geom.dims[0],
                boundaryCnt, (size_t)zeroVoxels, totalActive);
        } else {
            std::snprintf(buf, sizeof(buf),
                "IPW0 rebuilt [%s]: r=%.1f h=%.1f @ (%.1f,%.1f,%.1f) bbox=(%.1f-%.1f) — %zu leaves, %zu boundary voxels, %zu total voxels",
                typeName,
                geom.radius, geom.height,
                geom.origin[0], geom.origin[1], geom.origin[2],
                geom.origin[0] - geom.radius, geom.origin[0] + geom.radius,
                boundaryCnt, (size_t)zeroVoxels, totalActive);
        }
        logFn("Build", buf);
    }
    }

    return ipw;
}

} // namespace midgard
