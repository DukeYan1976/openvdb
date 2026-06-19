#include "core/ToolSweepSurface.h"
#include <cmath>
#include <algorithm>

namespace midgard {

ToolSweepSurface::ToolSweepSurface(const ToolDef& tool, const MoveSegment& seg, bool closedLoop)
    : mTool(tool), mSeg(seg), mClosedLoop(closedLoop)
{
    Vec3d ab = seg.end - seg.start;
    mLength = ab.length();
    mDir = (mLength > 1e-15) ? ab / mLength : Vec3d(0);

    // 数值稳定圆角：对于平底刀，使用一个极小的圆角来避免 QuadTree 在 90度棱边处的奇异性
    // 圆角大小取为公差的一半，既保证了计算稳定性，又不影响物理精度（在误差带内）
    double r_num = 0.01; // 默认 0.01mm

    // 1. u 向分段 (弧长比例)
    if (mTool.type == ToolType::BULL_NOSE) {
        mHflat = mTool.R - mTool.r;
        mHarc = (M_PI / 2.0) * mTool.r;
        mHcyl = mTool.H - mTool.r;
    } else if (mTool.type == ToolType::BALL_END) {
        mHflat = 0.0;
        mHarc = (M_PI / 2.0) * mTool.R;
        mHcyl = mTool.H - mTool.R;
    } else { // FLAT_END: 应用数值圆角
        mHflat = mTool.R - r_num;
        mHarc = (M_PI / 2.0) * r_num;
        mHcyl = mTool.H - r_num;
    }

    double hTotalU = mHflat + mHarc + mHcyl;
    mUSplit1 = (hTotalU > 1e-9) ? (mHflat / hTotalU) : 0.0;
    mUSplit2 = (hTotalU > 1e-9) ? ((mHflat + mHarc) / hTotalU) : 0.0;

    // 2. v 向分段 (准弧长比例)
    // closedLoop=true:  Left-Mid(L) + Front-Cap(πR) + Right-Mid(L) + Back-Cap(πR)
    // closedLoop=false: Left-Mid(L) + Front-Cap(πR) + Right-Mid(L)
    double arcCap = M_PI * mTool.R;
    double arcTotalV = mClosedLoop ? (2.0 * mLength + 2.0 * arcCap)
                                   : (2.0 * mLength + arcCap);
    
    if (mLength < 1e-12) {
        // 静态退化: 覆盖全圆 (only one cap)
        mV1 = 0.0;
        mV2 = 1.0;
        mV3 = 1.0;
    } else {
        mV1 = mLength / arcTotalV;
        mV2 = (mLength + arcCap) / arcTotalV;
        if (mClosedLoop) {
            mV3 = (2.0 * mLength + arcCap) / arcTotalV;
        } else {
            mV3 = 1.0; // 无Back-Cap, Right-Mid延伸至v=1
        }
    }

    double phiD = std::atan2(mDir.y(), mDir.x());
    mPhiGrazingL = phiD + M_PI / 2.0;
    mPhiGrazingR = phiD - M_PI / 2.0;
}

double ToolSweepSurface::rProfile(double u) const {
    if (u < mUSplit1) {
        double ratio = (mUSplit1 > 1e-9) ? (u / mUSplit1) : 0.0;
        return ratio * mHflat;
    } else if (u < mUSplit2) {
        double alpha = (M_PI / 2.0) * (u - mUSplit1) / (mUSplit2 - mUSplit1 + 1e-12);
        if (mTool.type == ToolType::BALL_END) return mTool.R * std::sin(alpha);
        double local_r = (mTool.type == ToolType::BULL_NOSE) ? mTool.r : (mTool.R - mHflat);
        return mHflat + local_r * std::sin(alpha);
    } else {
        return mTool.R;
    }
}

double ToolSweepSurface::zProfile(double u) const {
    if (u < mUSplit1) {
        return 0.0;
    } else if (u < mUSplit2) {
        double alpha = (M_PI / 2.0) * (u - mUSplit1) / (mUSplit2 - mUSplit1 + 1e-12);
        if (mTool.type == ToolType::BALL_END) return mTool.R * (1.0 - std::cos(alpha));
        double local_r = (mTool.type == ToolType::BULL_NOSE) ? mTool.r : (mTool.R - mHflat);
        return local_r * (1.0 - std::cos(alpha));
    } else {
        double hLocal = (u - mUSplit2) / (1.0 - mUSplit2 + 1e-12) * mHcyl;
        double base_z = (mTool.type == ToolType::BALL_END) ? mTool.R : 
                        ((mTool.type == ToolType::BULL_NOSE) ? mTool.r : (mTool.R - mHflat));
        return base_z + hLocal;
    }
}

Vec3d ToolSweepSurface::nProfile(double u) const {
    if (u < mUSplit1) {
        return Vec3d(0, 0, -1.0); // 底面法向向下
    } else if (u < mUSplit2) {
        double alpha = (M_PI / 2.0) * (u - mUSplit1) / (mUSplit2 - mUSplit1 + 1e-12);
        return Vec3d(std::sin(alpha), 0, -std::cos(alpha));
    } else {
        return Vec3d(1.0, 0, 0);
    }
}

// ─── 统一接口 ──────────────────────────────────────────────────

Vec3d ToolSweepSurface::eval(double u, double v) const {
    double r = rProfile(u);
    double z = zProfile(u);
    double phi = 0;
    Vec3d pos_offset(0);

    if (mLength < 1e-12) {
        // 静态退化: 覆盖全圆
        phi = mPhiGrazingL + v * 2.0 * M_PI;
        pos_offset = mSeg.start;
    } else {
        if (v < mV1) {
            // Left-Mid: 沿路径从起点到终点
            double t = v / mV1;
            phi = mPhiGrazingL;
            pos_offset = mSeg.start + t * mLength * mDir;
        } else if (v < mV2) {
            // Front-Cap: 终点处半球 (左→前→右)
            double w = (v - mV1) / (mV2 - mV1);
            phi = mPhiGrazingL - w * M_PI;
            pos_offset = mSeg.end;
        } else if (v < mV3) {
            // Right-Mid: 沿路径从终点回到起点
            double t = 1.0 - (v - mV2) / (mV3 - mV2);
            phi = mPhiGrazingR;
            pos_offset = mSeg.start + t * mLength * mDir;
        } else {
            // Back-Cap: 起点处半球 (右→后→左)
            double w = (v - mV3) / (1.0 - mV3);
            phi = mPhiGrazingR - w * M_PI;
            pos_offset = mSeg.start;
        }
    }

    return pos_offset + Vec3d(r * std::cos(phi), r * std::sin(phi), z);
}

Vec3d ToolSweepSurface::normal(double u, double v) const {
    Vec3d n = nProfile(u); // (nr, 0, nz) in local polar
    double phi = 0;

    if (mLength < 1e-12) {
        phi = mPhiGrazingL + v * 2.0 * M_PI;
    } else {
        if (v < mV1) {
            phi = mPhiGrazingL;
        } else if (v < mV2) {
            double w = (v - mV1) / (mV2 - mV1);
            phi = mPhiGrazingL - w * M_PI;
        } else if (v < mV3) {
            phi = mPhiGrazingR;
        } else {
            double w = (v - mV3) / (1.0 - mV3);
            phi = mPhiGrazingR - w * M_PI;
        }
    }

    return Vec3d(n.x() * std::cos(phi), n.x() * std::sin(phi), n.z());
}

openvdb::BBoxd ToolSweepSurface::bbox(double u0, double u1, double v0, double v1) const {
    openvdb::BBoxd box;
    const int N = 16; // 高密度采样确保曲率大区域的保守性
    for (int i = 0; i <= N; ++i) {
        double u = u0 + (u1 - u0) * i / N;
        for (int j = 0; j <= N; ++j) {
            double v = v0 + (v1 - v0) * j / N;
            box.expand(eval(u, v));
        }
    }
    // 接缝处补偿 (对齐 v1, v2, v3)
    if (v0 < mV1 && v1 > mV1) {
        for (int i = 0; i <= N; ++i) box.expand(eval(u0 + (u1 - u0) * i / N, mV1));
    }
    if (v0 < mV2 && v1 > mV2) {
        for (int i = 0; i <= N; ++i) box.expand(eval(u0 + (u1 - u0) * i / N, mV2));
    }
    if (v0 < mV3 && v1 > mV3) {
        for (int i = 0; i <= N; ++i) box.expand(eval(u0 + (u1 - u0) * i / N, mV3));
    }
    return box;
}

} // namespace midgard
