#include "core/ToolSweepSurface.h"
#include <cmath>
#include <algorithm>

namespace midgard {

ToolSweepSurface::ToolSweepSurface(const ToolDef& tool, const MoveSegment& seg)
    : mTool(tool), mSeg(seg)
{
    Vec3d ab = seg.end - seg.start;
    mLength = ab.length();
    mDir = (mLength > 1e-15) ? ab / mLength : Vec3d(0);

    if (mLength < 1e-15) {
        // 静态: 无mid补丁, 全部由cap覆盖(360°)
        mVSplit = 0.0;
        mPhiGrazing = 0.0;
    } else {
        // 线性刀路: 计算包络角和vSplit
        mPhiGrazing = std::atan2(-mDir.x(), mDir.y()) + M_PI;
        // vSplit按弧长比: mid弧长≈L, cap弧长≈πR
        mVSplit = mLength / (mLength + M_PI * mTool.R);
    }
}

// ─── 1D 球头刀轮廓 ─────────────────────────────────────────────
// u ∈ [0,1] 映射球面纬度 φ ∈ [0, π] (南极到北极)
// r(u) = R·sin(πu), z(u) = -R·cos(πu)

double ToolSweepSurface::rProfile(double u) const {
    return mTool.R * std::sin(M_PI * u);
}

double ToolSweepSurface::zProfile(double u) const {
    return -mTool.R * std::cos(M_PI * u);
}

// ─── Cap 补丁: 球面参数化 ──────────────────────────────────────
// 静态时覆盖360°, 线性刀路时覆盖前向180°
// u ∈ [0,1]: 纬度 (南极→北极)
// w ∈ [0,1]: 经度范围

Vec3d ToolSweepSurface::evalCap(double u, double w) const {
    Vec3d center = (mLength < 1e-15) ? mSeg.start : mSeg.end;

    if (mLength < 1e-15) {
        // 静态: 全球面
        double phi = M_PI * u;          // 纬度: 0→π (南极→北极)
        double theta = 2.0 * M_PI * w;  // 经度: 全360°
        double r = mTool.R * std::sin(phi);
        double z = -mTool.R * std::cos(phi);
        return center + Vec3d(r * std::cos(theta), r * std::sin(theta), z);
    } else {
        // 线性: 前向半球 (以dir为极轴)
        // u ∈ [0,1] → 半球纬度 α ∈ [0, π/2] (赤道到极点)
        // w ∈ [0,1] → 经度 β ∈ [0, 2π] (绕dir轴全圆)
        double alpha = (M_PI / 2.0) * u;  // 0=赤道, π/2=极点(最前方)
        double beta = 2.0 * M_PI * w;

        // 构建局部坐标系: dir为极轴, 需要两个正交轴
        // dir在XY平面 → 用Z作为一个正交轴
        Vec3d axisZ(0, 0, 1);
        Vec3d axisPerp = axisZ.cross(mDir);  // 垂直于dir且在XY平面
        if (axisPerp.length() < 1e-10) axisPerp = Vec3d(0, 1, 0);
        else axisPerp.normalize();
        // 第三轴
        Vec3d axisUp = mDir.cross(axisPerp);
        axisUp.normalize();

        // 半球面上的点 (以dir为极轴)
        double cosA = std::cos(alpha), sinA = std::sin(alpha);
        double cosB = std::cos(beta), sinB = std::sin(beta);
        Vec3d offset = mTool.R * (cosA * (cosB * axisPerp + sinB * axisUp) + sinA * mDir);
        return center + offset;
    }
}

Vec3d ToolSweepSurface::normalCap(double u, double w) const {
    if (mLength < 1e-15) {
        double phi = M_PI * u;
        double theta = 2.0 * M_PI * w;
        double sp = std::sin(phi), cp = std::cos(phi);
        return Vec3d(sp * std::cos(theta), sp * std::sin(theta), -cp);
    } else {
        double alpha = (M_PI / 2.0) * u;
        double beta = 2.0 * M_PI * w;

        Vec3d axisZ(0, 0, 1);
        Vec3d axisPerp = axisZ.cross(mDir);
        if (axisPerp.length() < 1e-10) axisPerp = Vec3d(0, 1, 0);
        else axisPerp.normalize();
        Vec3d axisUp = mDir.cross(axisPerp);
        axisUp.normalize();

        double cosA = std::cos(alpha), sinA = std::sin(alpha);
        double cosB = std::cos(beta), sinB = std::sin(beta);
        // 法线 = 从球心到表面点的方向
        return (cosA * (cosB * axisPerp + sinB * axisUp) + sinA * mDir);
    }
}

// ─── Mid 补丁: 包络面 (线性刀路) ──────────────────────────────
// u ∈ [0,1]: 沿轮廓
// t ∈ [0,1]: 沿路径

Vec3d ToolSweepSurface::evalMid(double u, double t) const {
    if (mLength < 1e-15) return mSeg.start; // 不应被调用

    double r = rProfile(u);
    double z = zProfile(u);

    // 擦掠线方向 = φ_grazing（面向进给的那侧）
    Vec3d local(r * std::cos(mPhiGrazing), r * std::sin(mPhiGrazing), z);
    Vec3d center = mSeg.start + t * mLength * mDir;
    return center + local;
}

Vec3d ToolSweepSurface::normalMid(double u, double t) const {
    double phi = M_PI * u;
    double sp = std::sin(phi), cp = std::cos(phi);
    // 法线沿径向外
    return Vec3d(sp * std::cos(mPhiGrazing), sp * std::sin(mPhiGrazing), -cp);
}

// ─── 统一接口 ──────────────────────────────────────────────────

Vec3d ToolSweepSurface::eval(double u, double v) const {
    if (v <= mVSplit && mVSplit > 0.0) {
        double t = v / mVSplit;
        return evalMid(u, t);
    } else {
        double w = (mVSplit >= 1.0) ? v : (v - mVSplit) / (1.0 - mVSplit);
        return evalCap(u, w);
    }
}

Vec3d ToolSweepSurface::normal(double u, double v) const {
    if (v <= mVSplit && mVSplit > 0.0) {
        double t = v / mVSplit;
        return normalMid(u, t);
    } else {
        double w = (mVSplit >= 1.0) ? v : (v - mVSplit) / (1.0 - mVSplit);
        return normalCap(u, w);
    }
}

openvdb::BBoxd ToolSweepSurface::bbox(double u0, double u1, double v0, double v1) const {
    openvdb::BBoxd box;
    // 加密采样（特别是u接近0/1的极点区域变化剧烈）
    const int N = 12;
    for (int i = 0; i <= N; ++i) {
        for (int j = 0; j <= N; ++j) {
            double u = u0 + (u1 - u0) * i / N;
            double v = v0 + (v1 - v0) * j / N;
            box.expand(eval(u, v));
        }
    }
    // 极点区域补偿: u接近0或1时参数面退化，bbox可能欠采样
    // 额外采样边界处的点确保覆盖
    if (u0 < 0.05 || u1 > 0.95) {
        for (int j = 0; j <= N; ++j) {
            double v = v0 + (v1 - v0) * j / N;
            box.expand(eval(u0, v));
            box.expand(eval(u1, v));
            box.expand(eval((u0+u1)*0.5, v));
        }
    }
    return box;
}

} // namespace midgard
