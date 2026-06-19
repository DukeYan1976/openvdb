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
}

double ToolSweptSDF::closestLambda(const Vec3d& p) const {
    if (mLength < 1e-15) return 0.0;
    Vec3d ap = p - mSeg.start;
    double t = ap.dot(mDir) / mLength;
    return std::clamp(t, 0.0, 1.0);
}

// ============================================================
// Ball End: Sphere centered at tip + (0,0,R)
// ============================================================

double ToolSweptSDF::evalBallEnd(const Vec3d& p) const {
    double lam = closestLambda(p);
    Vec3d tip = mSeg.start + lam * mLength * mDir;
    Vec3d center = tip + Vec3d(0, 0, mTool.R);

    if (p.z() < center.z()) {
        return (p - center).length() - mTool.R;
    } else {
        double dx = p.x() - center.x();
        double dy = p.y() - center.y();
        double radial = std::sqrt(dx*dx + dy*dy) - mTool.R;
        double hTop = center.z() + mTool.H - mTool.R;
        double axialAbove = p.z() - hTop;

        if (radial <= 0.0 && axialAbove <= 0.0) return std::max(radial, axialAbove);
        double dr = std::max(radial, 0.0);
        double dz = std::max(axialAbove, 0.0);
        return std::sqrt(dr*dr + dz*dz);
    }
}

Vec3d ToolSweptSDF::gradBallEnd(const Vec3d& p) const {
    double lam = closestLambda(p);
    Vec3d tip = mSeg.start + lam * mLength * mDir;
    Vec3d center = tip + Vec3d(0, 0, mTool.R);

    if (p.z() < center.z()) {
        Vec3d diff = p - center;
        double d = diff.length();
        return (d < 1e-15) ? Vec3d(0, 0, -1) : diff / d;
    } else {
        double dx = p.x() - center.x(), dy = p.y() - center.y();
        double rDist = std::sqrt(dx*dx + dy*dy);
        if (rDist < 1e-15) return Vec3d(0, 0, 1);
        return Vec3d(dx / rDist, dy / rDist, 0.0);
    }
}

// ============================================================
// Bull Nose & Flat End (Rounded Cylinder)
// ============================================================

static double roundedCylinderSDF(double r_p, double z_p, double R, double r, double H) {
    double r_core = R - r;
    if (z_p >= r && z_p <= H) return r_p - R;
    if (z_p > H) {
        double dz = z_p - H, dr = std::max(0.0, r_p - R);
        return std::sqrt(dr*dr + dz*dz);
    }
    double dx = std::max(0.0, r_p - r_core);
    double dy = std::max(0.0, r - z_p);
    if (r_p <= r_core && z_p <= 0) return -z_p;
    return std::sqrt(dx*dx + dy*dy) - r;
}

double ToolSweptSDF::evalFlatEnd(const Vec3d& p) const {
    double lam = closestLambda(p);
    Vec3d tip = mSeg.start + lam * mLength * mDir;
    double dx = p.x() - tip.x(), dy = p.y() - tip.y();
    return roundedCylinderSDF(std::sqrt(dx*dx + dy*dy), p.z() - tip.z(), mTool.R, 0.01, mTool.H);
}

double ToolSweptSDF::evalBullNose(const Vec3d& p) const {
    double lam = closestLambda(p);
    Vec3d tip = mSeg.start + lam * mLength * mDir;
    double dx = p.x() - tip.x(), dy = p.y() - tip.y();
    return roundedCylinderSDF(std::sqrt(dx*dx + dy*dy), p.z() - tip.z(), mTool.R, mTool.r, mTool.H);
}

double ToolSweptSDF::eval(const Vec3d& p) const {
    switch (mTool.type) {
        case ToolType::BALL_END: return evalBallEnd(p);
        case ToolType::FLAT_END: return evalFlatEnd(p);
        case ToolType::BULL_NOSE: return evalBullNose(p);
        default: return evalBallEnd(p);
    }
}

Vec3d ToolSweptSDF::gradient(const Vec3d& p) const {
    double h = 1e-6;
    Vec3d g(
        (eval(p + Vec3d(h,0,0)) - eval(p - Vec3d(h,0,0))) / (2*h),
        (eval(p + Vec3d(0,h,0)) - eval(p - Vec3d(0,h,0))) / (2*h),
        (eval(p + Vec3d(0,0,h)) - eval(p - Vec3d(0,0,h))) / (2*h)
    );
    double len = g.length();
    return (len < 1e-12) ? Vec3d(0,0,1) : g / len;
}

openvdb::BBoxd ToolSweptSDF::boundingBox() const {
    double R = mTool.R, H = mTool.H;
    Vec3d minP = openvdb::math::minComponent(mSeg.start, mSeg.end);
    Vec3d maxP = openvdb::math::maxComponent(mSeg.start, mSeg.end);
    if (mTool.type == ToolType::BALL_END) return openvdb::BBoxd(minP - Vec3d(R), maxP + Vec3d(R, R, H));
    return openvdb::BBoxd(minP - Vec3d(R, R, 0), maxP + Vec3d(R, R, H));
}

Vec3d ToolSweptSDF::evalSurface(double u, double t) const {
    return mSeg.start + t * mLength * mDir + Vec3d(mTool.R * std::cos(u * 2 * M_PI), mTool.R * std::sin(u * 2 * M_PI), 0);
}

openvdb::BBoxd ToolSweptSDF::surfaceBBox(double u0, double u1, double t0, double t1) const {
    openvdb::BBoxd box;
    for (double t : {t0, (t0+t1)*0.5, t1}) for (double u : {u0, (u0+u1)*0.5, u1}) box.expand(evalSurface(u, t));
    return box;
}

} // namespace midgard
