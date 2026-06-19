#pragma once

#include "core/Types.h"
#include <openvdb/openvdb.h>
#include <string>

namespace midgard {

/// 导出 MacroGrid 为 OBJ mesh (volumeToMesh)
void exportMacroMesh(const openvdb::FloatGrid::Ptr& grid, const std::string& filename);

/// 导出 MicroGrid 点云为 OBJ (顶点+法线)
void exportMicroPoints(const openvdb::points::PointDataGrid::Ptr& grid, const std::string& filename);

/// 导出 MicroGrid 点云为 PLY (位置+法向, ASCII)
void exportMicroPLY(const openvdb::points::PointDataGrid::Ptr& grid, const std::string& filename);

} // namespace midgard
