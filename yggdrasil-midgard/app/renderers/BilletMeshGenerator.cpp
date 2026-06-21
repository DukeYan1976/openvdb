#include "BilletMeshGenerator.h"

namespace midgard {

void BilletMeshGenerator::buildBox(const Vec3d& origin, const Vec3d& dims,
                                    std::vector<float>& verts,
                                    std::vector<uint32_t>& indices) {
    float x0 = origin[0], y0 = origin[1], z0 = origin[2];
    float x1 = x0 + dims[0], y1 = y0 + dims[1], z1 = z0 + dims[2];
    
    // 24 vertices (4 per face, each with face normal)
    // 格式: x, y, z, nx, ny, nz
    float vtx[24][6] = {
        // -Z face
        {x0,y0,z0, 0,0,-1}, {x1,y0,z0, 0,0,-1}, {x1,y1,z0, 0,0,-1}, {x0,y1,z0, 0,0,-1},
        // +Z face
        {x0,y0,z1, 0,0,1}, {x1,y0,z1, 0,0,1}, {x1,y1,z1, 0,0,1}, {x0,y1,z1, 0,0,1},
        // -Y face
        {x0,y0,z0, 0,-1,0}, {x1,y0,z0, 0,-1,0}, {x1,y0,z1, 0,-1,0}, {x0,y0,z1, 0,-1,0},
        // +Y face
        {x0,y1,z0, 0,1,0}, {x1,y1,z0, 0,1,0}, {x1,y1,z1, 0,1,0}, {x0,y1,z1, 0,1,0},
        // -X face
        {x0,y0,z0, -1,0,0}, {x0,y1,z0, -1,0,0}, {x0,y1,z1, -1,0,0}, {x0,y0,z1, -1,0,0},
        // +X face
        {x1,y0,z0, 1,0,0}, {x1,y1,z0, 1,0,0}, {x1,y1,z1, 1,0,0}, {x1,y0,z1, 1,0,0},
    };
    
    verts.clear();
    for (int i = 0; i < 24; ++i) {
        for (int j = 0; j < 6; ++j) {
            verts.push_back(vtx[i][j]);
        }
    }
    
    // 12 triangles (2 per face)
    uint32_t idx[] = {
        0,1,2, 0,2,3,       // -Z
        4,6,5, 4,7,6,       // +Z
        8,9,10, 8,10,11,    // -Y
        12,14,13, 12,15,14, // +Y
        16,17,18, 16,18,19, // -X
        20,22,21, 20,23,22, // +X
    };
    indices.clear();
    for (int i = 0; i < 36; ++i) {
        indices.push_back(idx[i]);
    }
}

void BilletMeshGenerator::buildBoxWireframe(const Vec3d& origin, const Vec3d& dims,
                                               std::vector<float>& lines) {
    float x0 = origin[0], y0 = origin[1], z0 = origin[2];
    float x1 = x0 + dims[0], y1 = y0 + dims[1], z1 = z0 + dims[2];
    
    // 12 edges of a box
    float edges[12][2][3] = {
        // bottom face
        {{x0,y0,z0},{x1,y0,z0}}, {{x1,y0,z0},{x1,y1,z0}}, {{x1,y1,z0},{x0,y1,z0}}, {{x0,y1,z0},{x0,y0,z0}},
        // top face
        {{x0,y0,z1},{x1,y0,z1}}, {{x1,y0,z1},{x1,y1,z1}}, {{x1,y1,z1},{x0,y1,z1}}, {{x0,y1,z1},{x0,y0,z1}},
        // vertical edges
        {{x0,y0,z0},{x0,y0,z1}}, {{x1,y0,z0},{x1,y0,z1}}, {{x1,y1,z0},{x1,y1,z1}}, {{x0,y1,z0},{x0,y1,z1}},
    };
    
    lines.clear();
    for (int e = 0; e < 12; ++e) {
        for (int p = 0; p < 2; ++p) {
            for (int c = 0; c < 3; ++c) {
                lines.push_back(edges[e][p][c]);
            }
        }
    }
}

void BilletMeshGenerator::buildCylinder(const Vec3d& origin, double radius, double height,
                                          std::vector<float>& verts,
                                          std::vector<uint32_t>& indices) {
    const int N = 32;  // 圆周分段
    float cx = (float)origin[0], cy = (float)origin[1];
    float z0 = (float)origin[2], z1 = z0 + (float)height;
    float r = (float)radius;

    verts.clear();
    indices.clear();

    // 底部圆心 (顶点 0)
    verts.insert(verts.end(), {cx, cy, z0, 0.f, 0.f, -1.f});
    // 底圈 (顶点 1..N)
    for (int i = 0; i < N; ++i) {
        float a = (float)(2.0 * M_PI * i / N);
        float x = cx + r * cosf(a), y = cy + r * sinf(a);
        verts.insert(verts.end(), {x, y, z0, 0.f, 0.f, -1.f});
    }

    // 顶部圆心 (顶点 N+1)
    verts.insert(verts.end(), {cx, cy, z1, 0.f, 0.f, 1.f});
    // 顶圈 (顶点 N+2..2N+1)
    for (int i = 0; i < N; ++i) {
        float a = (float)(2.0 * M_PI * i / N);
        float x = cx + r * cosf(a), y = cy + r * sinf(a);
        verts.insert(verts.end(), {x, y, z1, 0.f, 0.f, 1.f});
    }

    // 侧面环 (顶点 2N+2..4N+1): 底圈 N 个 + 顶圈 N 个，带侧法线
    uint32_t sideBase = (uint32_t)(2 * N + 2);
    for (int i = 0; i < N; ++i) {
        float a = (float)(2.0 * M_PI * i / N);
        float nx = cosf(a), ny = sinf(a);
        float x = cx + r * nx, y = cy + r * ny;
        verts.insert(verts.end(), {x, y, z0, nx, ny, 0.f});  // 底
        verts.insert(verts.end(), {x, y, z1, nx, ny, 0.f});  // 顶
    }

    // 底部三角扇: center(0) + ring(1..N)
    for (int i = 0; i < N; ++i) {
        indices.push_back(0);
        indices.push_back(1 + i);
        indices.push_back(1 + (i + 1) % N);
    }

    // 顶部三角扇: center(N+1) + ring(N+2..2N+1)
    for (int i = 0; i < N; ++i) {
        indices.push_back(N + 1);
        indices.push_back(N + 2 + (i + 1) % N);
        indices.push_back(N + 2 + i);
    }

    // 侧面四边形: 每段 2 个三角形
    for (int i = 0; i < N; ++i) {
        uint32_t b0 = sideBase + 2 * i;       // 底 i
        uint32_t t0 = sideBase + 2 * i + 1;   // 顶 i
        uint32_t b1 = sideBase + 2 * ((i + 1) % N);     // 底 i+1
        uint32_t t1 = sideBase + 2 * ((i + 1) % N) + 1; // 顶 i+1
        indices.insert(indices.end(), {b0, t0, b1, b1, t0, t1});
    }
}

void BilletMeshGenerator::buildCylinderWireframe(const Vec3d& origin, double radius, double height,
                                                  std::vector<float>& lines) {
    const int N = 32;
    float cx = (float)origin[0], cy = (float)origin[1];
    float z0 = (float)origin[2], z1 = z0 + (float)height;
    float r = (float)radius;

    lines.clear();

    // 底圆
    for (int i = 0; i < N; ++i) {
        float a0 = (float)(2.0 * M_PI * i / N);
        float a1 = (float)(2.0 * M_PI * (i + 1) / N);
        lines.insert(lines.end(), {cx + r*cosf(a0), cy + r*sinf(a0), z0,
                                    cx + r*cosf(a1), cy + r*sinf(a1), z0});
    }

    // 顶圆
    for (int i = 0; i < N; ++i) {
        float a0 = (float)(2.0 * M_PI * i / N);
        float a1 = (float)(2.0 * M_PI * (i + 1) / N);
        lines.insert(lines.end(), {cx + r*cosf(a0), cy + r*sinf(a0), z1,
                                    cx + r*cosf(a1), cy + r*sinf(a1), z1});
    }

    // 4 条母线
    for (int i = 0; i < 4; ++i) {
        float a = (float)(2.0 * M_PI * i / 4);
        float x = cx + r * cosf(a), y = cy + r * sinf(a);
        lines.insert(lines.end(), {x, y, z0, x, y, z1});
    }
}

} // namespace midgard
