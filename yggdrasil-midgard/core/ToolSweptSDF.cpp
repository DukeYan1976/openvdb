#include "core/ToolSweptSDF.h"
#include <algorithm>
#include <cmath>

namespace midgard {

ToolSweptSDF::ToolSweptSDF(const ToolDef& tool, const MoveSegment& seg)
    : mTool(tool), mSeg(seg)
{
    Vec3d ab = seg.end - seg.start;
    mLength = ab.length();
    mDir = (mLength > 1e-15) ? ab / mLength : Vec3d(0);

    // 刀轴坐标系
    mAxis = seg.axis;
    double axisLen = mAxis.length();
    if (axisLen < 1e-15) mAxis = Vec3d(0, 0, 1);
    else mAxis /= axisLen;

    // 三轴快速路径判定
    mIsZAxis = (std::abs(mAxis.x()) < 1e-10 &&
                std::abs(mAxis.y()) < 1e-10 &&
                mAxis.z() > 0.0);

    // 构造正交基 (mE1, mE2, mAxis)
    if (mIsZAxis) {
        mE1 = Vec3d(1, 0, 0);
        mE2 = Vec3d(0, 1, 0);
    } else {
        Vec3d aux = (std::abs(mAxis.z()) < 0.9) ? Vec3d(0, 0, 1) : Vec3d(1, 0, 0);
        mE1 = mAxis.cross(aux);
        mE1.normalize();
        mE2 = mAxis.cross(mE1);
        mE2.normalize();
    }
}

double ToolSweptSDF::closestLambda(const Vec3d& p) const {
    if (mLength < 1e-15) return 0.0;
    Vec3d ap = p - mSeg.start;
    double t = ap.dot(mDir) / mLength;
    return std::clamp(t, 0.0, 1.0);
}

Vec3d ToolSweptSDF::toLocal(const Vec3d& p, const Vec3d& origin) const {
    Vec3d d = p - origin;
    return Vec3d(d.dot(mE1), d.dot(mE2), d.dot(mAxis));
}

// ============================================================
// 2D 距离核函数 (radial, axial) — 与刀轴方向无关
// ============================================================

static double roundedCylinderSDF(double r_p, double z_p, double R, double r, double H) {
    double r_core = R - r;
    if (z_p >= r && z_p <= H) return r_p - R;
    if (z_p > H) {
        double dz = z_p - H, dr = std::max(0.0, r_p - R);
        return std::sqrt(dr * dr + dz * dz);
    }
    double dx = std::max(0.0, r_p - r_core);
    double dy = std::max(0.0, r - z_p);
    if (r_p <= r_core && z_p <= 0) return -z_p;
    return std::sqrt(dx * dx + dy * dy) - r;
}

// ============================================================
// Ball End
// ============================================================

double ToolSweptSDF::evalBallEnd(const Vec3d& p) const {
    // BallEnd: 投影到球心轨迹 (start + R·axis + λ·L·mDir)
    double lam;
    if (mLength < 1e-15) {
        lam = 0.0;
    } else {
        Vec3d ap = p - mSeg.start - mTool.R * mAxis;
        lam = std::clamp(ap.dot(mDir) / mLength, 0.0, 1.0);
    }
    Vec3d tip = mSeg.start + lam * mLength * mDir;

    if (mIsZAxis) {
        Vec3d center = tip + Vec3d(0, 0, mTool.R);
        if (p.z() < center.z()) {
            return (p - center).length() - mTool.R;
        }
        double dx = p.x() - center.x(), dy = p.y() - center.y();
        double radial = std::sqrt(dx * dx + dy * dy) - mTool.R;
        double axialAbove = p.z() - (center.z() + mTool.H - mTool.R);
        if (radial <= 0.0 && axialAbove <= 0.0) return std::max(radial, axialAbove);
        double dr = std::max(radial, 0.0), dz = std::max(axialAbove, 0.0);
        return std::sqrt(dr * dr + dz * dz);
    }

    Vec3d loc = toLocal(p, tip);
    double r_p = std::sqrt(loc.x() * loc.x() + loc.y() * loc.y());
    double z_p = loc.z();

    if (z_p < mTool.R) {
        double z_rel = z_p - mTool.R;
        return std::sqrt(r_p * r_p + z_rel * z_rel) - mTool.R;
    }
    double radial = r_p - mTool.R;
    double axialAbove = z_p - mTool.H;
    if (radial <= 0.0 && axialAbove <= 0.0) return std::max(radial, axialAbove);
    double dr = std::max(radial, 0.0), dz = std::max(axialAbove, 0.0);
    return std::sqrt(dr * dr + dz * dz);
}

Vec3d ToolSweptSDF::gradBallEnd(const Vec3d& p) const {
    double lam;
    if (mLength < 1e-15) {
        lam = 0.0;
    } else {
        Vec3d ap = p - mSeg.start - mTool.R * mAxis;
        lam = std::clamp(ap.dot(mDir) / mLength, 0.0, 1.0);
    }
    Vec3d tip = mSeg.start + lam * mLength * mDir;

    if (mIsZAxis) {
        Vec3d center = tip + Vec3d(0, 0, mTool.R);
        if (p.z() < center.z()) {
            Vec3d diff = p - center;
            double d = diff.length();
            return (d < 1e-15) ? Vec3d(0, 0, -1) : diff / d;
        }
        double dx = p.x() - center.x(), dy = p.y() - center.y();
        double rDist = std::sqrt(dx * dx + dy * dy);
        if (rDist < 1e-15) return Vec3d(0, 0, 1);
        return Vec3d(dx / rDist, dy / rDist, 0.0);
    }

    Vec3d loc = toLocal(p, tip);
    double r_p = std::sqrt(loc.x() * loc.x() + loc.y() * loc.y());
    double z_p = loc.z();
    Vec3d gLocal;

    if (z_p < mTool.R) {
        Vec3d diff(loc.x(), loc.y(), z_p - mTool.R);
        double d = diff.length();
        gLocal = (d < 1e-15) ? Vec3d(0, 0, -1) : diff / d;
    } else {
        if (r_p < 1e-15) {
            gLocal = Vec3d(0, 0, 1);
        } else {
            gLocal = Vec3d(loc.x() / r_p, loc.y() / r_p, 0.0);
        }
    }
    return gLocal.x() * mE1 + gLocal.y() * mE2 + gLocal.z() * mAxis;
}

// ============================================================
// Flat End & Bull Nose
// ============================================================

double ToolSweptSDF::evalFlatEnd(const Vec3d& p) const {
    double lam = closestLambda(p);
    Vec3d tip = mSeg.start + lam * mLength * mDir;

    if (mIsZAxis) {
        double dx = p.x() - tip.x(), dy = p.y() - tip.y();
        return roundedCylinderSDF(std::sqrt(dx * dx + dy * dy), p.z() - tip.z(), mTool.R, 0.01, mTool.H);
    }

    Vec3d loc = toLocal(p, tip);
    double r_p = std::sqrt(loc.x() * loc.x() + loc.y() * loc.y());
    return roundedCylinderSDF(r_p, loc.z(), mTool.R, 0.01, mTool.H);
}

double ToolSweptSDF::evalBullNose(const Vec3d& p) const {
    double lam = closestLambda(p);
    Vec3d tip = mSeg.start + lam * mLength * mDir;

    if (mIsZAxis) {
        double dx = p.x() - tip.x(), dy = p.y() - tip.y();
        return roundedCylinderSDF(std::sqrt(dx * dx + dy * dy), p.z() - tip.z(), mTool.R, mTool.r, mTool.H);
    }

    Vec3d loc = toLocal(p, tip);
    double r_p = std::sqrt(loc.x() * loc.x() + loc.y() * loc.y());
    return roundedCylinderSDF(r_p, loc.z(), mTool.R, mTool.r, mTool.H);
}

Vec3d ToolSweptSDF::gradFlatEnd(const Vec3d& p) const {
    double h = 1e-6;
    Vec3d g(
        (evalFlatEnd(p + Vec3d(h, 0, 0)) - evalFlatEnd(p - Vec3d(h, 0, 0))) / (2 * h),
        (evalFlatEnd(p + Vec3d(0, h, 0)) - evalFlatEnd(p - Vec3d(0, h, 0))) / (2 * h),
        (evalFlatEnd(p + Vec3d(0, 0, h)) - evalFlatEnd(p - Vec3d(0, 0, h))) / (2 * h)
    );
    double len = g.length();
    return (len < 1e-12) ? mAxis : g / len;
}

// ============================================================
// 统一入口
// ============================================================

double ToolSweptSDF::eval(const Vec3d& p) const {
    switch (mTool.type) {
        case ToolType::BALL_END:  return evalBallEnd(p);
        case ToolType::FLAT_END:  return evalFlatEnd(p);
        case ToolType::BULL_NOSE: return evalBullNose(p);
        default: return evalBallEnd(p);
    }
}

Vec3d ToolSweptSDF::gradient(const Vec3d& p) const {
    switch (mTool.type) {
        case ToolType::BALL_END: return gradBallEnd(p);
        case ToolType::FLAT_END: return gradFlatEnd(p);
        default: break;
    }
    double h = 1e-6;
    Vec3d g(
        (eval(p + Vec3d(h, 0, 0)) - eval(p - Vec3d(h, 0, 0))) / (2 * h),
        (eval(p + Vec3d(0, h, 0)) - eval(p - Vec3d(0, h, 0))) / (2 * h),
        (eval(p + Vec3d(0, 0, h)) - eval(p - Vec3d(0, 0, h))) / (2 * h)
    );
    double len = g.length();
    return (len < 1e-12) ? mAxis : g / len;
}

openvdb::BBoxd ToolSweptSDF::boundingBox() const {
    double R = mTool.R, H = mTool.H;
    Vec3d minP = openvdb::math::minComponent(mSeg.start, mSeg.end);
    Vec3d maxP = openvdb::math::maxComponent(mSeg.start, mSeg.end);

    if (mIsZAxis) {
        if (mTool.type == ToolType::BALL_END)
            return openvdb::BBoxd(minP - Vec3d(R), maxP + Vec3d(R, R, H));
        return openvdb::BBoxd(minP - Vec3d(R, R, 0), maxP + Vec3d(R, R, H));
    }

    // 通用：保守AABB扩展
    Vec3d expand(R + H * std::abs(mAxis.x()),
                 R + H * std::abs(mAxis.y()),
                 R + H * std::abs(mAxis.z()));
    return openvdb::BBoxd(minP - expand, maxP + expand);
}

// ============================================================
// 参数面: u ∈ [0,1] 环向角, t ∈ [0,1] 路径参数
// ============================================================

Vec3d ToolSweptSDF::evalSurface(double u, double t) const {
    double phi = u * 2.0 * M_PI;
    Vec3d pathPt = mSeg.start + t * mLength * mDir;

    if (mIsZAxis) {
        return pathPt + Vec3d(mTool.R * std::cos(phi), mTool.R * std::sin(phi), 0);
    }

    Vec3d radial = mTool.R * (std::cos(phi) * mE1 + std::sin(phi) * mE2);
    return pathPt + radial;
}

openvdb::BBoxd ToolSweptSDF::surfaceBBox(double u0, double u1, double t0, double t1) const {
    openvdb::BBoxd box;
    constexpr int N = 8;
    for (int i = 0; i <= N; ++i) {
        double t = t0 + (t1 - t0) * i / N;
        for (int j = 0; j <= N; ++j) {
            double u = u0 + (u1 - u0) * j / N;
            box.expand(evalSurface(u, t));
        }
    }
    return box;
}

} // namespace midgard
