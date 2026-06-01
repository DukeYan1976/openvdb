#include "core/ToolSweepSDF.h"
#include <algorithm>
#include <cmath>

namespace ygg {

ToolSweepSDF::ToolSweepSDF(ToolType type, double R, double r, double H,
                           const Vec3d& A, const Vec3d& B)
    : mType(type), mR(R), mr(r), mH(H), mA(A), mB(B)
{
    mAB = mB - mA;
    mABdot = mAB.dot(mAB);
}

double ToolSweepSDF::eval(const Vec3d& p) const {
    switch (mType) {
        case ToolType::BALL_END: return evalBallEnd(p);
        case ToolType::FLAT_END: return evalFlatEnd(p);
        default: return evalBallEnd(p);
    }
}

// 球头刀：胶囊体 SDF（无分支）
double ToolSweepSDF::evalBallEnd(const Vec3d& p) const {
    Vec3d AP = p - mA;
    double t = (mABdot > 0.0) ? std::clamp(AP.dot(mAB) / mABdot, 0.0, 1.0) : 0.0;
    Vec3d closest = mA + t * mAB;
    return (p - closest).length() - mR;
}

// 平底刀：扁平胶囊体 SDF
double ToolSweepSDF::evalFlatEnd(const Vec3d& p) const {
    Vec3d AP = p - mA;

    // 沿运动方向投影（XYZ全方向）
    double t = (mABdot > 0.0) ? std::clamp(AP.dot(mAB) / mABdot, 0.0, 1.0) : 0.0;
    Vec3d center = mA + t * mAB;

    // 径向距离（XY平面）
    double dx = p.x() - center.x();
    double dy = p.y() - center.y();
    double distRadial = std::sqrt(dx*dx + dy*dy) - mR;

    // Z方向约束：底面在 center.z()，顶面在 center.z() + H
    double zBottom = center.z();
    double zTop = center.z() + mH;
    double distZ = std::max(zBottom - p.z(), p.z() - zTop);

    // 组合
    if (distRadial <= 0.0 && distZ <= 0.0) {
        return std::max(distRadial, distZ); // 内部：取最大负值
    } else if (distRadial > 0.0 && distZ > 0.0) {
        return std::sqrt(distRadial*distRadial + distZ*distZ); // 角外
    } else {
        return std::max(distRadial, distZ); // 边外
    }
}

Vec3d ToolSweepSDF::gradient(const Vec3d& p) const {
    const double eps = 1e-6;
    return Vec3d(
        (eval({p.x()+eps, p.y(), p.z()}) - eval({p.x()-eps, p.y(), p.z()})) / (2*eps),
        (eval({p.x(), p.y()+eps, p.z()}) - eval({p.x(), p.y()-eps, p.z()})) / (2*eps),
        (eval({p.x(), p.y(), p.z()+eps}) - eval({p.x(), p.y(), p.z()-eps})) / (2*eps)
    );
}

openvdb::BBoxd ToolSweepSDF::getBoundingBox() const {
    Vec3d minPt(std::min(mA.x(), mB.x()) - mR,
                std::min(mA.y(), mB.y()) - mR,
                std::min(mA.z(), mB.z()) - mR);
    Vec3d maxPt(std::max(mA.x(), mB.x()) + mR,
                std::max(mA.y(), mB.y()) + mR,
                std::max(mA.z(), mB.z()) + mH);
    return openvdb::BBoxd(minPt, maxPt);
}

} // namespace ygg
