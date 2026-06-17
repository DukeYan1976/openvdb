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
        // u ∈ [0,1] → 极角 alpha ∈ [0, π/2] (0=正前, π/2=赤道)
        // w ∈ [0,1] → 绕轴经度 beta ∈ [0, 2π]
        double alpha = (M_PI / 2.0) * (1.0 - u);  // u=1时在最前面
        double beta = 2.0 * M_PI * w;

        Vec3d axisZ(0, 0, 1);
        Vec3d axisPerp = axisZ.cross(mDir);
        if (axisPerp.length() < 1e-10) axisPerp = Vec3d(0, 1, 0);
        else axisPerp.normalize();
        Vec3d axisUp = mDir.cross(axisPerp);

        double cosA = std::cos(alpha), sinA = std::sin(alpha);
        double cosB = std::cos(beta), sinB = std::sin(beta);
        
        // 极轴是 mDir
        Vec3d offset = mTool.R * (sinA * mDir + cosA * (cosB * axisPerp + sinB * axisUp));
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
        double alpha = (M_PI / 2.0) * (1.0 - u);
        double beta = 2.0 * M_PI * w;

        Vec3d axisZ(0, 0, 1);
        Vec3d axisPerp = axisZ.cross(mDir);
        if (axisPerp.length() < 1e-10) axisPerp = Vec3d(0, 1, 0);
        else axisPerp.normalize();
        Vec3d axisUp = mDir.cross(axisPerp);

        double cosA = std::cos(alpha), sinA = std::sin(alpha);
        double cosB = std::cos(beta), sinB = std::sin(beta);
        
        return (sinA * mDir + cosA * (cosB * axisPerp + sinB * axisUp));
    }
}

// ─── Mid 补丁: 包络面 (线性刀路) ──────────────────────────────
// u ∈ [0,1]: 绕轴角度 [0, 2π]
// t ∈ [0,1]: 沿路径 [0, L]

Vec3d ToolSweepSurface::evalMid(double u, double t) const {
    if (mLength < 1e-15) return mSeg.start;

    double theta = 2.0 * M_PI * u;
    
    // 构建垂直于 mDir 的基向量
    Vec3d axisZ(0, 0, 1);
    Vec3d axis1 = axisZ.cross(mDir);
    if (axis1.length() < 1e-10) axis1 = Vec3d(0, 1, 0);
    else axis1.normalize();
    Vec3d axis2 = mDir.cross(axis1);
    
    // 侧面包络是一个圆柱（对球头刀而言）
    // 注意：如果是平底刀，这里逻辑不同，但目前主要针对球头刀优化
    Vec3d local = mTool.R * (std::cos(theta) * axis1 + std::sin(theta) * axis2);
    Vec3d center = mSeg.start + t * mLength * mDir;
    
    return center + local;
}

Vec3d ToolSweepSurface::normalMid(double u, double t) const {
    double theta = 2.0 * M_PI * u;
    Vec3d axisZ(0, 0, 1);
    Vec3d axis1 = axisZ.cross(mDir);
    if (axis1.length() < 1e-10) axis1 = Vec3d(0, 1, 0);
    else axis1.normalize();
    Vec3d axis2 = mDir.cross(axis1);
    
    return (std::cos(theta) * axis1 + std::sin(theta) * axis2);
}

// ─── 统一接口 ──────────────────────────────────────────────────

Vec3d ToolSweepSurface::eval(double u, double v) const {
    if (mLength < 1e-15) {
        // 静态情况：全由 evalCap 处理 (w=v)
        return evalCap(u, v);
    }

    // 线性刀路：划分 StartCap, Mid, EndCap
    // 比例: 1:3:1
    if (v < 0.2) {
        // Start Cap (后向半球)
        double w = v / 0.2;
        Vec3d center = mSeg.start;
        double alpha = (M_PI / 2.0) * (1.0 - w);
        double beta = 2.0 * M_PI * u;
        Vec3d axisZ(0, 0, 1);
        Vec3d axis1 = axisZ.cross(mDir);
        if (axis1.length() < 1e-10) axis1 = Vec3d(0, 1, 0); else axis1.normalize();
        Vec3d axis2 = mDir.cross(axis1);
        double cosA = std::cos(alpha), sinA = std::sin(alpha);
        double cosB = std::cos(beta), sinB = std::sin(beta);
        // 极轴是 -mDir
        return center + mTool.R * (sinA * (-mDir) + cosA * (cosB * axis1 + sinB * axis2));
    } else if (v < 0.8) {
        // Mid (圆柱侧面)
        double t = (v - 0.2) / 0.6;
        return evalMid(u, t);
    } else {
        // End Cap (前向半球)
        double w = (v - 0.8) / 0.2;
        return evalCap(u, w);
    }
}

Vec3d ToolSweepSurface::normal(double u, double v) const {
    if (mLength < 1e-15) return normalCap(u, v);

    if (v < 0.2) {
        double w = v / 0.2;
        double alpha = (M_PI / 2.0) * (1.0 - w);
        double beta = 2.0 * M_PI * u;
        Vec3d axisZ(0, 0, 1);
        Vec3d axis1 = axisZ.cross(mDir);
        if (axis1.length() < 1e-10) axis1 = Vec3d(0, 1, 0); else axis1.normalize();
        Vec3d axis2 = mDir.cross(axis1);
        double cosA = std::cos(alpha), sinA = std::sin(alpha);
        double cosB = std::cos(beta), sinB = std::sin(beta);
        return (sinA * (-mDir) + cosA * (cosB * axis1 + sinB * axis2));
    } else if (v < 0.8) {
        double t = (v - 0.2) / 0.6;
        return normalMid(u, t);
    } else {
        double w = (v - 0.8) / 0.2;
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
