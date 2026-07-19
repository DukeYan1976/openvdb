#pragma once
/// @file SurfaceMesher.h
/// @brief 从点法式集合重建视觉三角形网格（独立模块，零外部依赖）
///
/// 算法: MLS隐式场 + 稀疏体素 + Marching Cubes
/// 用途: 切削界面可视化渲染

#include <cstdint>
#include <vector>

namespace midgard {

// 前向声明避免引入 Types.h（保持独立性）
// 调用方负责传入 double[3] 格式的 position/normal 数组

/// 三角形网格（渲染用）
struct TriMesh {
    std::vector<float> vertices;      // 交错格式: [x,y,z, nx,ny,nz] × nVerts
    std::vector<uint32_t> indices;    // 三角形索引: nTris × 3

    int vertexCount() const { return (int)vertices.size() / 6; }
    int triangleCount() const { return (int)indices.size() / 3; }
    bool empty() const { return indices.empty(); }
    void clear() { vertices.clear(); indices.clear(); }
};

/// 从点法式集合重建视觉三角形网格
///
/// @param positions  点位置数组 (每点3个double: x,y,z)
/// @param normals    点法线数组 (每点3个double: nx,ny,nz)，与 positions 等长
/// @param count      点数量
/// @param resolution 每轴最大体素数（默认128，越大越精细但越慢）
/// @return 三角形网格（输入 <3 个点时返回空）
///
/// 性能参考:
///   10k 点 → ~8ms
///   50k 点 → ~40ms
///
TriMesh buildSurfaceMesh(
    const double* positions,
    const double* normals,
    size_t count,
    int resolution = 128);

} // namespace midgard
