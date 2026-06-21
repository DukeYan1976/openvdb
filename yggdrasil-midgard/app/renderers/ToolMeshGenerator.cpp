#include "ToolMeshGenerator.h"
#include <cmath>
#include <algorithm>

namespace midgard {

static constexpr int SEGS = 24;
static constexpr double PI = 3.14159265358979323846;

// ─── 圆柱体侧面 ───────────────────────────────────────────────
static void addCylinderBody(double R, double zBot, double zTop,
                            std::vector<float>& verts,
                            std::vector<uint32_t>& indices) {
    uint32_t base = (uint32_t)(verts.size() / 6);
    for (int i = 0; i <= SEGS; ++i) {
        double a = 2.0 * PI * i / SEGS;
        float nx = (float)cos(a), ny = (float)sin(a);
        float px = (float)(R * cos(a)), py = (float)(R * sin(a));
        verts.insert(verts.end(), {px, py, (float)zBot, nx, ny, 0});
        verts.insert(verts.end(), {px, py, (float)zTop, nx, ny, 0});
    }
    for (int i = 0; i < SEGS; ++i) {
        uint32_t b0 = base + i*2, b1 = b0+1, t0 = b0+2, t1 = b0+3;
        indices.insert(indices.end(), {b0, t0, b1, b1, t0, t1});
    }
}

// ─── 底部圆盘（法线 = (0,0,-1)，z=bottom）─────────────────────
static void addBottomCap(double R, double z,
                         std::vector<float>& verts,
                         std::vector<uint32_t>& indices) {
    uint32_t center = (uint32_t)(verts.size() / 6);
    verts.insert(verts.end(), {0,0,(float)z, 0,0,-1});
    for (int i = 0; i <= SEGS; ++i) {
        double a = 2.0 * PI * i / SEGS;
        verts.insert(verts.end(), {(float)(R*cos(a)), (float)(R*sin(a)), (float)z, 0,0,-1});
    }
    for (int i = 0; i < SEGS; ++i)
        indices.insert(indices.end(), {center, center+2+i, center+1+i});  // CCW from below
}

// ─── 下半球（球头朝下）─────────────────────────────────────────
// 球心在 (0,0,R)，下半球 z ∈ [0, R]
static void addLowerHemisphere(double R,
                                std::vector<float>& verts,
                                std::vector<uint32_t>& indices) {
    int phiSegs = SEGS / 4 + 1;  // φ: π/2(赤道) → π(底极点)
    uint32_t base = (uint32_t)(verts.size() / 6);
    int ringVerts = SEGS + 1;

    for (int p = 0; p <= phiSegs; ++p) {
        double phi = PI/2.0 + (PI/2.0) * p / phiSegs;  // π/2 → π
        float sinPhi = (float)sin(phi), cosPhi = (float)cos(phi);
        float r = (float)(R * sinPhi);
        float z = (float)(R + R * cosPhi);  // R(1+cosφ): 0 at φ=π, R at φ=π/2
        for (int i = 0; i <= SEGS; ++i) {
            double a = 2.0 * PI * i / SEGS;
            float nx = (float)(cos(a) * sinPhi);
            float ny = (float)(sin(a) * sinPhi);
            float nz = cosPhi;
            verts.insert(verts.end(), {(float)(r*cos(a)), (float)(r*sin(a)), z, nx, ny, nz});
        }
    }
    for (int p = 0; p < phiSegs; ++p) {
        for (int i = 0; i < SEGS; ++i) {
            uint32_t a = base + p*ringVerts + i;
            uint32_t b = a + 1, c = a + ringVerts, d = c + 1;
            indices.insert(indices.end(), {a, c, b, b, c, d});
        }
    }
}

// ═══════════════════════════════════════════════════════════════
// Ball End: 球头在下(z∈[0,R]) + 圆柱体(z∈[R,R+H])
// ═══════════════════════════════════════════════════════════════
void ToolMeshGenerator::buildBallEnd(double R, double H,
                                      std::vector<float>& verts,
                                      std::vector<uint32_t>& indices) {
    verts.clear(); indices.clear();
    addLowerHemisphere(R, verts, indices);       // z=0 → R
    addCylinderBody(R, R, R + H, verts, indices); // z=R → R+H
}

// ═══════════════════════════════════════════════════════════════
// Flat End: 底圆盘(z=0) + 圆柱体(z=0→H)
// ═══════════════════════════════════════════════════════════════
void ToolMeshGenerator::buildFlatEnd(double R, double H,
                                      std::vector<float>& verts,
                                      std::vector<uint32_t>& indices) {
    verts.clear(); indices.clear();
    addBottomCap(R, 0, verts, indices);
    addCylinderBody(R, 0, H, verts, indices);
}

// ═══════════════════════════════════════════════════════════════
// Bull Nose: 平底圆盘(z=0, rInner) + 锥台(z=0→r) + 圆柱(z=r→r+H)
// ═══════════════════════════════════════════════════════════════
void ToolMeshGenerator::buildBullNose(double R, double r, double H,
                                       std::vector<float>& verts,
                                       std::vector<uint32_t>& indices) {
    verts.clear(); indices.clear();
    double rInner = std::max(R - r, 0.01);
    addBottomCap(rInner, 0, verts, indices);

    // 锥台过渡
    uint32_t base = (uint32_t)(verts.size() / 6);
    for (int i = 0; i <= SEGS; ++i) {
        double a = 2.0 * PI * i / SEGS;
        float nx = (float)cos(a), ny = (float)sin(a);
        verts.insert(verts.end(), {(float)(rInner*cos(a)), (float)(rInner*sin(a)), 0, nx, ny, -0.5f});
        verts.insert(verts.end(), {(float)(R*cos(a)), (float)(R*sin(a)), (float)r, nx, ny, 0.5f});
    }
    for (int i = 0; i < SEGS; ++i) {
        uint32_t b0 = base + i*2, b1 = b0+1, t0 = b0+2, t1 = b0+3;
        indices.insert(indices.end(), {b0, t0, b1, b1, t0, t1});
    }
    // 主体圆柱
    addCylinderBody(R, r, r + H, verts, indices);
}

// ─── 线框 ─────────────────────────────────────────────────────

static void addCylinderWireframe(double R, double zBot, double zTop,
                                 std::vector<float>& lines) {
    for (int i = 0; i < SEGS; ++i) {
        double a0 = 2*PI*i/SEGS, a1 = 2*PI*(i+1)/SEGS;
        float x0=(float)(R*cos(a0)), y0=(float)(R*sin(a0));
        float x1=(float)(R*cos(a1)), y1=(float)(R*sin(a1));
        lines.insert(lines.end(), {x0,y0,(float)zBot, x1,y1,(float)zBot});
        lines.insert(lines.end(), {x0,y0,(float)zTop, x1,y1,(float)zTop});
    }
    for (int i = 0; i < SEGS; i += SEGS/4) {
        double a = 2*PI*i/SEGS;
        float x=(float)(R*cos(a)), y=(float)(R*sin(a));
        lines.insert(lines.end(), {x,y,(float)zBot, x,y,(float)zTop});
    }
}

static void addHemisphereWireframe(double R, std::vector<float>& lines) {
    // 赤道环
    for (int i = 0; i < SEGS; ++i) {
        double a0 = 2*PI*i/SEGS, a1 = 2*PI*(i+1)/SEGS;
        lines.insert(lines.end(), {(float)(R*cos(a0)), (float)(R*sin(a0)), (float)R,
                                   (float)(R*cos(a1)), (float)(R*sin(a1)), (float)R});
    }
    // 经线（从赤道到极点）
    for (int i = 0; i < SEGS; i += SEGS/6) {
        double a = 2*PI*i/SEGS;
        float x=(float)(R*cos(a)), y=(float)(R*sin(a));
        lines.insert(lines.end(), {x, y, (float)R, 0, 0, 0});
    }
}

void ToolMeshGenerator::buildBallEndWireframe(double R, double H,
                                               std::vector<float>& lines) {
    lines.clear();
    addHemisphereWireframe(R, lines);
    addCylinderWireframe(R, R, R+H, lines);
}

void ToolMeshGenerator::buildFlatEndWireframe(double R, double H,
                                               std::vector<float>& lines) {
    lines.clear();
    addCylinderWireframe(R, 0, H, lines);
    for (int i = 0; i < SEGS; i += SEGS/4) {
        double a = 2*PI*i/SEGS;
        float x=(float)(R*cos(a)), y=(float)(R*sin(a));
        lines.insert(lines.end(), {0,0,0, x,y,0});
    }
}

void ToolMeshGenerator::buildBullNoseWireframe(double R, double r, double H,
                                                std::vector<float>& lines) {
    lines.clear();
    addCylinderWireframe(R, r, r+H, lines);
    double ri = std::max(R - r, 0.01);
    addCylinderWireframe(ri, 0, 0, lines);  // 底部小圆
    for (int i = 0; i < SEGS; i += SEGS/4) {
        double a = 2*PI*i/SEGS;
        lines.insert(lines.end(), {(float)(ri*cos(a)), (float)(ri*sin(a)), 0,
                                   (float)(R*cos(a)),  (float)(R*sin(a)),  (float)r});
    }
}

} // namespace midgard