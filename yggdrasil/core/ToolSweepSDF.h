#pragma once

#include "types/YggTypes.h"
#include <openvdb/math/BBox.h>

namespace ygg {

class ToolSweepSDF {
public:
    ToolSweepSDF(ToolType type, double R, double r, double H,
                 const Vec3d& A, const Vec3d& B);

    /// 计算空间点到扫掠体的有符号距离（< 0 = 内部）
    double eval(const Vec3d& p) const;

    /// 梯度（解析梯度，消除 6× 有限差分开销）
    Vec3d gradient(const Vec3d& p) const;

    /// 包围盒
    openvdb::BBoxd getBoundingBox() const;

private:
    double evalBallEnd(const Vec3d& p) const;
    double evalFlatEnd(const Vec3d& p) const;
    Vec3d gradientBallEnd(const Vec3d& p) const;
    Vec3d gradientFlatEnd(const Vec3d& p) const;

    ToolType mType;
    double mR, mr, mH;
    Vec3d mA, mB, mAB;
    double mABdot;
};

} // namespace ygg
