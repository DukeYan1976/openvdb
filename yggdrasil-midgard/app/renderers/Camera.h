#pragma once
#include <cmath>
#include "AppState.h"

namespace midgard {

// CAD 风格轨道相机，正交投影
// 世界坐标: XY = 水平面, Z = 向上
struct Camera {
    float yaw = 45.0f, pitch = 30.0f;
    float tx = 15.0f, ty = 15.0f, tz = 7.5f;  // 注视点
    float dist = 200.0f;                        // 到注视点的距离
    float orthoSize = 50.0f;                    // 正交投影半高

    void buildMVP(int w, int h, float mvp[16], float nm[9]) const;

    // ─── 鼠标交互 ───
    void orbit(float dx, float dy);   // 左键拖拽旋转
    void zoom(float delta);           // 滚轮缩放
    void pan(float dx, float dy);     // 中键平移（屏幕空间）

    // ─── 视图快捷操作 ───
    void zoomAll(const Vec3d& billetOrigin, const Vec3d& billetDims);
    void viewTop();
    void viewBottom();
    void viewFront();
    void viewRight();
    void viewIso();
    void zoomToRect(float x0, float y0, float x1, float y1, int vpW, int vpH);

private:
    void getAxes(float& rx, float& ry, float& rz,
                 float& ux, float& uy, float& uz,
                 float& fx, float& fy, float& fz) const;
};

} // namespace midgard