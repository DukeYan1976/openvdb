#include "core/ToolSweepSDF.h"
#include <algorithm>
#include <cmath>

namespace midgard {

ToolSweepSDF::ToolSweepSDF(const ToolDef& tool, const MoveSegment& seg)
    : mTool(tool), mSeg(seg)
{
    Vec3d ab = seg.end - seg.start;
    mLength = ab.length();
    mDir = (mLength > 1e-15) ? ab / mLength : Vec3d(0);
}

double ToolSweepSDF::closestLambda(const Vec3d& p) const {
    if (mLength < 1e-15) return 0.0;
    Vec3d ap = p - mSeg.start;
    double t = ap.dot(mDir) / mLength;
    return std::clamp(t, 0.0, 1.0);
}

// ============================================================
// Ball End: SDF = dist(p, segment) - R
// Capsule (Minkowski sum of sphere along segment)
// ============================================================

double ToolSweepSDF::evalBallEnd(const Vec3d& p) const {
    double lam = closestLambda(p);
    Vec3d closest = mSeg.start + lam * mLength * mDir;
    double dist = (p - closest).length();
    return dist - mTool.R;
}

Vec3d ToolSweepSDF::gradBallEnd(const Vec3d& p) const {
    double lam = closestLambda(p);
    Vec3d closest = mSeg.start + lam * mLength * mDir;
    Vec3d diff = p - closest;
    double dist = diff.length();
    if (dist < 1e-15) return Vec3d(0, 0, 1); // degenerate: arbitrary unit
    return diff / dist;
}

// ============================================================
// Flat End: 圆柱(R, H) 沿Z轴, 刀尖在seg点位置(底面)
// 扫掠 = 圆柱沿XY平面内线段平移
// 
// 分解为:
//   radial: XY平面内到线段的距离 - R
//   axial:  Z方向的上下限 [seg.z, seg.z + H]
// 
// 对于外部点: SDF = 组合距离
// ============================================================

double ToolSweepSDF::evalFlatEnd(const Vec3d& p) const {
    // 投影到XY平面的线段距离
    // 线段在XY平面内: start_xy -> end_xy (Z分量用于确定底面位置)
    // 刀轴沿Z, 底面在 seg.start.z(), 顶面在 seg.start.z() + H
    // 注意: 三轴固定Z轴, start.z() == end.z() (同高度切削)
    // 但允许不同高度, 此时closest point的Z确定底面位置
    
    double lam = closestLambda(p);
    Vec3d closest = mSeg.start + lam * mLength * mDir;
    
    // radial distance (XY plane relative to closest point on segment)
    double dx = p.x() - closest.x();
    double dy = p.y() - closest.y();
    double radial = std::sqrt(dx*dx + dy*dy) - mTool.R;
    
    // axial distance (Z relative to tool at closest point)
    double zBottom = closest.z();           // 底面
    double zTop = closest.z() + mTool.H;   // 顶面
    double axialBelow = zBottom - p.z();    // >0 when below bottom
    double axialAbove = p.z() - zTop;       // >0 when above top

    // Case analysis for SDF of rounded cylinder sweep
    if (radial <= 0.0 && axialBelow <= 0.0 && axialAbove <= 0.0) {
        // Inside: SDF = max(radial, axialBelow, axialAbove) (all negative)
        return std::max({radial, axialBelow, axialAbove});
    }
    
    // Outside: compute distance based on which region
    double dr = std::max(radial, 0.0);
    double dz = std::max(std::max(axialBelow, axialAbove), 0.0);
    
    if (radial > 0.0 && axialBelow <= 0.0 && axialAbove <= 0.0) {
        // Pure side
        return radial;
    }
    if (radial <= 0.0 && (axialBelow > 0.0 || axialAbove > 0.0)) {
        // Pure top/bottom
        return dz;
    }
    // Corner (outside both radially and axially)
    return std::sqrt(dr*dr + dz*dz);
}

Vec3d ToolSweepSDF::gradFlatEnd(const Vec3d& p) const {
    double lam = closestLambda(p);
    Vec3d closest = mSeg.start + lam * mLength * mDir;
    
    double dx = p.x() - closest.x();
    double dy = p.y() - closest.y();
    double rDist = std::sqrt(dx*dx + dy*dy);
    double radial = rDist - mTool.R;
    
    double zBottom = closest.z();
    double zTop = closest.z() + mTool.H;
    double axialBelow = zBottom - p.z();
    double axialAbove = p.z() - zTop;
    
    // Radial unit direction in XY
    Vec3d radDir(0);
    if (rDist > 1e-15) {
        radDir = Vec3d(dx / rDist, dy / rDist, 0.0);
    } else {
        radDir = Vec3d(1, 0, 0); // degenerate
    }
    
    if (radial <= 0.0 && axialBelow <= 0.0 && axialAbove <= 0.0) {
        // Inside: gradient of max(radial, axialBelow, axialAbove)
        double maxVal = std::max({radial, axialBelow, axialAbove});
        if (maxVal == radial) return radDir;
        if (maxVal == axialBelow) return Vec3d(0, 0, -1);
        return Vec3d(0, 0, 1);
    }
    
    if (radial > 0.0 && axialBelow <= 0.0 && axialAbove <= 0.0) {
        return radDir;
    }
    if (radial <= 0.0 && axialBelow > 0.0) {
        return Vec3d(0, 0, -1);
    }
    if (radial <= 0.0 && axialAbove > 0.0) {
        return Vec3d(0, 0, 1);
    }
    
    // Corner region: gradient of sqrt(dr² + dz²)
    double dr = std::max(radial, 0.0);
    double dz = (axialBelow > 0.0) ? axialBelow : axialAbove;
    double dist = std::sqrt(dr*dr + dz*dz);
    if (dist < 1e-15) return Vec3d(0, 0, 1);
    
    Vec3d zDir = (axialBelow > 0.0) ? Vec3d(0, 0, -1) : Vec3d(0, 0, 1);
    return (dr * radDir + dz * zDir) / dist;
}

// ============================================================
// Dispatch
// ============================================================

double ToolSweepSDF::eval(const Vec3d& p) const {
    switch (mTool.type) {
        case ToolType::BALL_END: return evalBallEnd(p);
        case ToolType::FLAT_END: return evalFlatEnd(p);
        default: return evalBallEnd(p); // BULL_NOSE: TODO
    }
}

Vec3d ToolSweepSDF::gradient(const Vec3d& p) const {
    switch (mTool.type) {
        case ToolType::BALL_END: return gradBallEnd(p);
        case ToolType::FLAT_END: return gradFlatEnd(p);
        default: return gradBallEnd(p);
    }
}

// ============================================================
// BoundingBox / Surface (placeholder for M1, full impl in M4)
// ============================================================

openvdb::BBoxd ToolSweepSDF::boundingBox() const {
    double R = mTool.R;
    double H = mTool.H;
    Vec3d minP = openvdb::math::minComponent(mSeg.start, mSeg.end);
    Vec3d maxP = openvdb::math::maxComponent(mSeg.start, mSeg.end);
    
    if (mTool.type == ToolType::BALL_END) {
        return openvdb::BBoxd(minP - Vec3d(R), maxP + Vec3d(R));
    }
    // Flat end: expand R in XY, [0, H] in Z from bottom
    minP -= Vec3d(R, R, 0);
    maxP += Vec3d(R, R, H);
    return openvdb::BBoxd(minP, maxP);
}

Vec3d ToolSweepSDF::evalSurface(double u, double t) const {
    // Placeholder: parametric surface evaluation
    // u ∈ [0,1]: around circumference, t ∈ [0,1]: along path
    Vec3d center = mSeg.start + t * mLength * mDir;
    double angle = u * 2.0 * M_PI;
    return center + Vec3d(mTool.R * std::cos(angle), mTool.R * std::sin(angle), 0);
}

openvdb::BBoxd ToolSweepSDF::surfaceBBox(double u0, double u1,
                                          double t0, double t1) const {
    // Conservative: sample corners + midpoints
    openvdb::BBoxd box;
    for (double t : {t0, (t0+t1)*0.5, t1}) {
        for (double u : {u0, (u0+u1)*0.5, u1}) {
            box.expand(evalSurface(u, t));
        }
    }
    return box;
}

} // namespace midgard
