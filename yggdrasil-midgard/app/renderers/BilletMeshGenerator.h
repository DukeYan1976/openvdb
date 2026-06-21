#pragma once
#include <vector>
#include <cstdint>
#include "AppState.h"

namespace midgard {

// 毛坯网格生成器
class BilletMeshGenerator {
public:
    // 生成 Box 毛坯的线框和表面
    static void buildBox(const Vec3d& origin, const Vec3d& dims,
                         std::vector<float>& verts,    // 6 floats per vertex: x,y,z,nx,ny,nz
                         std::vector<uint32_t>& indices);
    
    // 生成 Cylinder 毛坯
    static void buildCylinder(const Vec3d& origin, double radius, double height,
                              std::vector<float>& verts,
                              std::vector<uint32_t>& indices);

    // Cylinder 线框（上下圆 + 4条母线）
    static void buildCylinderWireframe(const Vec3d& origin, double radius, double height,
                                       std::vector<float>& lines);
    
    // 线框版本（只生成边线，用于透明显示）
    static void buildBoxWireframe(const Vec3d& origin, const Vec3d& dims,
                                   std::vector<float>& lines);  // 3 floats per point
};

} // namespace midgard
