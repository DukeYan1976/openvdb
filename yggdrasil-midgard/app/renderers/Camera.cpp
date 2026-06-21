#include "Camera.h"
#include <cmath>
#include <algorithm>

namespace midgard {

// ─── 内部：计算相机坐标轴 ───────────────────────────────────────
void Camera::getAxes(float& rx, float& ry, float& rz,
                     float& ux, float& uy, float& uz,
                     float& fx, float& fy, float& fz) const {
    float yr = yaw   * 3.14159265f / 180.0f;
    float pr = pitch * 3.14159265f / 180.0f;
    float ex = tx + dist * cosf(pr) * cosf(yr);
    float ey = ty + dist * cosf(pr) * sinf(yr);
    float ez = tz + dist * sinf(pr);

    // Forward: target → eye (视方向)
    fx = tx - ex; fy = ty - ey; fz = tz - ez;
    float fl = sqrtf(fx*fx + fy*fy + fz*fz);
    fx /= fl; fy /= fl; fz /= fl;

    // Right: F × worldUp(Z)
    rx = fy; ry = -fx; rz = 0;
    float rl = sqrtf(rx*rx + ry*ry);
    if (rl < 1e-6f) { rx = 1; ry = rz = 0; }
    else { rx /= rl; ry /= rl; }

    // Up: R × F
    ux = ry*fz - rz*fy;
    uy = rz*fx - rx*fz;
    uz = rx*fy - ry*fx;
}

// ─── MVP ──────────────────────────────────────────────────────
void Camera::buildMVP(int w, int h, float mvp[16], float nm[9]) const {
    float rx, ry, rz, ux, uy, uz, fx, fy, fz;
    getAxes(rx, ry, rz, ux, uy, uz, fx, fy, fz);

    float yr = yaw   * 3.14159265f / 180.0f;
    float pr = pitch * 3.14159265f / 180.0f;
    float ex = tx + dist * cosf(pr) * cosf(yr);
    float ey = ty + dist * cosf(pr) * sinf(yr);
    float ez = tz + dist * sinf(pr);

    float asp = (float)w / (float)h;
    // 动态 near/far：确保以 dist 为中心的范围完全包含模型
    float zN = std::max(0.01f, dist - orthoSize * 4.0f);
    float zF = dist + orthoSize * 4.0f;
    float P[16] = {0};
    P[0]  = 1.0f / (asp * orthoSize);
    P[5]  = 1.0f / orthoSize;
    P[10] = -2.0f / (zF - zN);
    P[14] = -(zF + zN) / (zF - zN);
    P[15] = 1.0f;

    float V[16] = {
         rx,  ux, -fx,  0,
         ry,  uy, -fy,  0,
         rz,  uz, -fz,  0,
        -(rx*ex + ry*ey + rz*ez),
        -(ux*ex + uy*ey + uz*ez),
         (fx*ex + fy*ey + fz*ez), 1
    };

    for (int col = 0; col < 4; col++)
        for (int row = 0; row < 4; row++) {
            float s = 0;
            for (int k = 0; k < 4; k++)
                s += P[row + k*4] * V[k + col*4];
            mvp[row + col*4] = s;
        }

    nm[0] = rx; nm[1] = ux; nm[2] = -fx;
    nm[3] = ry; nm[4] = uy; nm[5] = -fy;
    nm[6] = rz; nm[7] = uz; nm[8] = -fz;
}

// ─── 交互 ─────────────────────────────────────────────────────
void Camera::orbit(float dx, float dy) {
    yaw   += dx * 0.5f;
    pitch += dy * 0.5f;
    pitch = std::max(-90.0f, std::min(90.0f, pitch));
}

void Camera::zoom(float delta) {
    orthoSize *= (1.0f + delta * 0.1f);
    orthoSize = std::max(1.0f, std::min(500.0f, orthoSize));
}

void Camera::pan(float dx, float dy) {
    // 屏幕空间平移：dx=右 dy=上 → 注视点沿相机 right/up 移动
    float rx, ry, rz, ux, uy, uz, fx, fy, fz;
    getAxes(rx, ry, rz, ux, uy, uz, fx, fy, fz);

    float s = orthoSize * 0.002f;  // 与视口大小成比例
    // 拖右 → 场景右移 → target 沿 -R 移动
    tx += (-rx * dx + ux * dy) * s;
    ty += (-ry * dx + uy * dy) * s;
    tz += (-rz * dx + uz * dy) * s;
}

// ─── 视图快捷操作 ─────────────────────────────────────────────
void Camera::zoomAll(const Vec3d& bo, const Vec3d& bd) {
    // 注视点 = 毛坯中心
    tx = (float)(bo[0] + bd[0] * 0.5);
    ty = (float)(bo[1] + bd[1] * 0.5);
    tz = (float)(bo[2] + bd[2] * 0.5);

    // 包围球半径
    float r = (float)sqrt(bd[0]*bd[0] + bd[1]*bd[1] + bd[2]*bd[2]) * 0.55f;
    orthoSize = std::max(r, 1.0f);

    // 视角不变（仅调整注视点和缩放）
    dist = orthoSize * 3.0f;
}

void Camera::viewTop()    { yaw = 0;   pitch = 90; }
void Camera::viewFront()  { yaw = 0;   pitch = 0;  }
void Camera::viewRight()  { yaw = 90;  pitch = 0;  }
void Camera::viewIso()    { yaw = 45;  pitch = 30; }

} // namespace midgard