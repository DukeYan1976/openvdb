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
    double mPhiGrazingL; // 左侧擦掠角 (仅用于 caps)
    double mPhiGrazingR; // 右侧擦掠角 (仅用于 caps)
    double mHflat;       // 底平段弧长
    double mHarc;        // 圆角段弧长
    double mHcyl;        // 圆柱段弧长

    // ── 斜坡路径擦掠修正 (mDir.z ≠ 0) ──
    double m_r_xy;          // |mDir| in xy-plane
    double mGrazCosL, mGrazSinL;   // 圆柱段 Left-Mid 擦掠 cos/sin φ
    double mGrazCosR, mGrazSinR;   // 圆柱段 Right-Mid 擦掠 cos/sin φ
    double mE1_x, mE1_y;       // 擦掠大圆基 e1 = mDir×ẑ 归一化 (z=0)
    double mE2_x, mE2_y, mE2_z;   // 擦掠大圆基 e2 = mDir×e1
    double mBlendEps;           // 球底→圆柱衔接 blend 半宽

    // ── 圆角弧参数 (按刀型预计算) ──
    double mCornerCx, mCornerCz;  // 圆角弧中心 (工具坐标系)
    double mCornerRad;            // 圆角弧半径

    // ── 刀轴坐标系 (支持非Z刀轴) ──
    Vec3d mAxis;        // 归一化刀轴方向
    Vec3d mAxE1;        // 径向基1 (⊥ mAxis)
    Vec3d mAxE2;        // 径向基2 (⊥ mAxis, ⊥ mAxE1)
    bool mIsZAxis;      // 三轴快速路径标志
};

} // namespace midgard
