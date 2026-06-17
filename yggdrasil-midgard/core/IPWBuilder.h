#pragma once

#include "core/Types.h"
#include <openvdb/points/PointDataGrid.h>

namespace midgard {

struct GeometryDef {
    enum Type { BOX, CYLINDER, SPHERE, MESH };
    Type type = BOX;
    Vec3d origin{0, 0, 0};
    Vec3d dims{0, 0, 0};       // BOX: length/width/height
    double radius = 0;          // CYLINDER/SPHERE
    double height = 0;          // CYLINDER
};

class IPWBuilder {
public:
    /// 构建完整 IPW0, t_billet = 2 * config.user_t
    IPWState build(const GeometryDef& geom, const ToleranceConfig& config);

private:
    openvdb::FloatGrid::Ptr buildMacroGrid(
        const GeometryDef& geom, const ToleranceConfig& config);

    openvdb::points::PointDataGrid::Ptr buildMicroGrid(
        const openvdb::FloatGrid::Ptr& macroGrid,
        const GeometryDef& geom,
        double t_billet);
};

} // namespace midgard
