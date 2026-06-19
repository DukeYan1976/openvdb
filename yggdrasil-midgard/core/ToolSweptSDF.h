#pragma once

#include "core/Types.h"
#include <openvdb/math/BBox.h>

namespace midgard {

class ToolSweptSDF {
public:
    ToolSweptSDF(const ToolDef& tool, const MoveSegment& seg);

    /// 有符号距离 (< 0 = 内部)
    double eval(const Vec3d& p) const;

    /// 解析梯度
    Vec3d gradient(const Vec3d& p) const;

    /// 扫掠体AABB
    openvdb::BBoxd boundingBox() const;

    /// 参数面求值 S(u,t) → R³
    Vec3d evalSurface(double u, double t) const;

    /// 参数子域的3D AABB
    openvdb::BBoxd surfaceBBox(double u0, double u1,
                               double t0, double t1) const;

private:
    double evalBallEnd(const Vec3d& p) const;
    double evalFlatEnd(const Vec3d& p) const;
    double evalBullNose(const Vec3d& p) const;
    Vec3d gradBallEnd(const Vec3d& p) const;
    Vec3d gradFlatEnd(const Vec3d& p) const;
    // TODO M2: BULL_NOSE 三段构造 (evalCylinder + evalTorus + evalDisc)

    /// 点p到线段AB的最近参数λ∈[0,1]和最近点
    double closestLambda(const Vec3d& p) const;

    ToolDef mTool;
    MoveSegment mSeg;
    Vec3d mDir;       // 归一化方向 (或零向量)
    double mLength;   // 刀路段长度
};

} // namespace midgard
