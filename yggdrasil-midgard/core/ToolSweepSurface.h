#pragma once

#include "core/Types.h"
#include <openvdb/math/BBox.h>

namespace midgard {

/// 刀具扫掠体参数面
/// 静态刀具(路径=0): 退化为全球面(仅S_cap)
/// 线性刀路(L>0):
///   closedLoop=true  → Left-Mid + Front-Cap + Right-Mid + Back-Cap (完整闭合，首刀/单段)
///   closedLoop=false → Left-Mid + Front-Cap + Right-Mid (开放，连续切削时后端帽由前段覆盖)
class ToolSweepSurface {
public:
    ToolSweepSurface(const ToolDef& tool, const MoveSegment& seg, bool closedLoop = true);

    // ─── 统一接口 ──────────────────────────────────────────
    // u ∈ [0, 1]: 沿剖面 (弧长映射: Bottom/latitude -> Top/linear)
    // v ∈ [0, 1]: 环向 (闭合时含Back-Cap，开放时Right-Mid延伸至v=1)
    Vec3d eval(double u, double v) const;
    Vec3d normal(double u, double v) const;
    openvdb::BBoxd bbox(double u0, double u1, double v0, double v1) const;

    double uSplit1() const { return mUSplit1; }
    double uSplit2() const { return mUSplit2; }
    double v1() const { return mV1; }
    double v2() const { return mV2; }
    double v3() const { return mV3; }
    bool isClosedLoop() const { return mClosedLoop; }

    double totalArcU() const { return mHflat + mHarc + mHcyl; }
    double totalArcV() const {
        return mClosedLoop ? (2.0 * mLength + 2.0 * M_PI * mTool.R)
                           : (2.0 * mLength + M_PI * mTool.R);
    }

private:
    // 1D 轮廓: 弧长映射
    double rProfile(double u) const;
    double zProfile(double u) const;
    Vec3d nProfile(double u) const; // (nr, nz)

    ToolDef mTool;
    MoveSegment mSeg;
    Vec3d mDir;
    double mLength;
    bool mClosedLoop;

    // 分段参数 (弧长比例)
    double mUSplit1; // u 分界 1 (Bottom Flat / Corner Arc)
    double mUSplit2; // u 分界 2 (Corner Arc / Cylindrical)
    double mV1;      // v 分界 1 (Left-Mid / Front-Cap)
    double mV2;      // v 分界 2 (Front-Cap / Right-Mid)
    double mV3;      // v 分界 3 (Right-Mid / Back-Cap), =1.0 when !closedLoop

    // 几何常量
    double mPhiGrazingL; // 左侧擦掠角
    double mPhiGrazingR; // 右侧擦掠角
    double mHflat;       // 底平段弧长
    double mHarc;        // 圆角段弧长
    double mHcyl;        // 圆柱段弧长
};

} // namespace midgard
