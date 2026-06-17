#pragma once

#include "core/Types.h"
#include <openvdb/math/BBox.h>

namespace midgard {

/// 刀具扫掠体参数面 (双补丁模型)
/// 静态刀具(路径=0): 退化为全球面(仅S_cap)
/// 线性刀路(L>0): S_mid(包络面) + S_cap(前向端帽)
class ToolSweepSurface {
public:
    ToolSweepSurface(const ToolDef& tool, const MoveSegment& seg);

    // ─── 统一接口 ──────────────────────────────────────────
    // v ∈ [0, vSplit]: mid 补丁
    // v ∈ [vSplit, 1]: cap 补丁
    Vec3d eval(double u, double v) const;
    Vec3d normal(double u, double v) const;
    openvdb::BBoxd bbox(double u0, double u1, double v0, double v1) const;

    double vSplit() const { return mVSplit; }
    bool hasMidPatch() const { return mVSplit > 0.0; }

    // ─── 分补丁接口 ────────────────────────────────────────
    Vec3d evalMid(double u, double t) const;
    Vec3d normalMid(double u, double t) const;

    Vec3d evalCap(double u, double w) const;
    Vec3d normalCap(double u, double w) const;

private:
    // 1D 轮廓: 球头刀
    double rProfile(double u) const;  // 半径 at u
    double zProfile(double u) const;  // Z高度 at u

    ToolDef mTool;
    MoveSegment mSeg;
    Vec3d mDir;
    double mLength;
    double mVSplit;
    double mPhiGrazing;  // 包络角
};

} // namespace midgard
