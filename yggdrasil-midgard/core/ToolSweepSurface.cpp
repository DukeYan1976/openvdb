#include "core/ToolSweepSurface.h"
#include <cmath>
#include <algorithm>
#include <tuple>

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

    // ── 斜坡路径擦掠修正：预计算圆柱段擦掠 + 球底大圆基 ──
    double mx = mDir.x(), my = mDir.y(), mz = mDir.z();
    double r_xy_sq = mx * mx + my * my;
    m_r_xy = std::sqrt(r_xy_sq);

    if (m_r_xy > 1e-12) {
        // 圆柱段擦掠 cos/sin (常数 φ)
        mGrazCosL = -my / m_r_xy;   // cos(φ_grazing_L)
        mGrazSinL =  mx / m_r_xy;   // sin(φ_grazing_L)
        mGrazCosR =  my / m_r_xy;   // cos(φ_grazing_R) = -cos(φ_grazing_L)
        mGrazSinR = -mx / m_r_xy;   // sin(φ_grazing_R) = -sin(φ_grazing_L)

        // 擦掠大圆基 {e1, e2}: 平面 ⊥ mDir
        // e1 = mDir × (0,0,1) / r_xy  (在 xy 平面内, ⊥ mDir)
        mE1_x =  my / m_r_xy;
        mE1_y = -mx / m_r_xy;
        // e2 = mDir × e1  (⊥ mDir, ⊥ e1)
        mE2_x =  mz * mx / m_r_xy;
        mE2_y =  mz * my / m_r_xy;
        mE2_z = -m_r_xy;
    } else {
        // mDir ≈ (0,0,±1): 垂直路径, 任意 φ 均可, 回退到 π/2
        mGrazCosL = 0.0; mGrazSinL = 1.0;
        mGrazCosR = 0.0; mGrazSinR = -1.0;
        mE1_x = 1.0; mE1_y = 0.0;
        mE2_x = 0.0; mE2_y = 1.0; mE2_z = 0.0;
    }

    // blend 半宽: u 参数的 ~1.5 段 (匹配 40 u-segs 球底 ~11 段)
    mBlendEps = 0.03;

    // ── 角弧参数: 用于球底/圆角擦掠大圆公式 ──
    if (mTool.type == ToolType::BALL_END) {
        mCornerCx = 0.0;
        mCornerCz = mTool.R;
        mCornerRad = mTool.R;
    } else if (mTool.type == ToolType::BULL_NOSE) {
        mCornerCx = mHflat;       // mTool.R - mTool.r
        mCornerCz = mTool.r;
        mCornerRad = mTool.r;
    } else { // FLAT_END: 数值圆角
        double r_num = 0.01;
        mCornerCx = mTool.R - r_num;
        mCornerCz = r_num;
        mCornerRad = r_num;
    }

    // ── 刀轴坐标系 ──
    mAxis = seg.axis;
    double axisLen = mAxis.length();
    if (axisLen < 1e-15) mAxis = Vec3d(0, 0, 1);
    else mAxis /= axisLen;

    mIsZAxis = (std::abs(mAxis.x()) < 1e-10 &&
                std::abs(mAxis.y()) < 1e-10 &&
                mAxis.z() > 0.0);

    if (mIsZAxis) {
        mAxE1 = Vec3d(1, 0, 0);
        mAxE2 = Vec3d(0, 1, 0);
    } else {
        Vec3d aux = (std::abs(mAxis.z()) < 0.9) ? Vec3d(0, 0, 1) : Vec3d(1, 0, 0);
        mAxE1 = mAxis.cross(aux);
        mAxE1.normalize();
        mAxE2 = mAxis.cross(mAxE1);
        mAxE2.normalize();
    }
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

// 辅助: 球底擦掠径向偏移 (dx, dy, z) — 擦掠大圆公式
// 返回 (dx, dy, z_corrected): 工具坐标系下的偏移，z 在钳位时修正
inline std::tuple<double, double, double> grazingRadial(
    double u, double r, double z,
    double cx, double cy, double cz, double rad,
    double r_xy, double e1x, double e1y,
    double e2x, double e2y, double e2z,
    double uSplit1, double uSplit2, double blendEps,
    double grazCosCyl, double grazSinCyl,
    bool isLeft)
{
    // 平底段 (u < uSplit1) 和圆柱段 (u >= uSplit2): 常数 φ
    if (u < uSplit1 || u >= uSplit2) {
        return {r * grazCosCyl, r * grazSinCyl, z};
    }

    if (r_xy < 1e-12) {
        return {r * grazCosCyl, r * grazSinCyl, z};
    }

    // 球底/圆角: 擦掠大圆
    double sinPsi_raw = (cz - z) / (rad * r_xy);

    // 钳位并计算大圆实际 z
    double sinPsi = std::max(-1.0, std::min(1.0, sinPsi_raw));
    double z_out = cz - rad * r_xy * sinPsi;  // 大圆上的实际 z

    double cosPsiSq = 1.0 - sinPsi * sinPsi;
    double cosPsi = (cosPsiSq > 0.0) ? std::sqrt(cosPsiSq) : 0.0;
    if (isLeft)
        cosPsi = -cosPsi;

    // P = C + rad * (cosΨ·e1 + sinΨ·e2)
    double dx = cx + rad * (cosPsi * e1x + sinPsi * e2x);
    double dy = cy + rad * (cosPsi * e1y + sinPsi * e2y);

    // ── 球底→圆柱衔接 Blend (保证 C1 连续) ──
    double uJ = uSplit2;
    double u0 = uJ - blendEps;
    if (u > u0 && blendEps > 1e-12) {
        double dxC = r * grazCosCyl;
        double dyC = r * grazSinCyl;

        double alpha = (u - u0) / blendEps;
        alpha = alpha * alpha * (3.0 - 2.0 * alpha);

        dx = dxC + (1.0 - alpha) * (dx - dxC);
        dy = dyC + (1.0 - alpha) * (dy - dyC);
        z_out = z + (1.0 - alpha) * (z_out - z);  // blend z 回 zProfile
    }

    return {dx, dy, z_out};
}

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
            // Left-Mid: 沿路径从起点到终点 (擦掠线)
            double t = v / mV1;
            pos_offset = mSeg.start + t * mLength * mDir;

            // 使用擦掠大圆公式计算径向偏移
            double r = rProfile(u);
            double z = zProfile(u);
            auto [dx, dy, zc] = grazingRadial(u, r, z,
                mCornerCx, 0.0, mCornerCz, mCornerRad,
                m_r_xy, mE1_x, mE1_y, mE2_x, mE2_y, mE2_z,
                mUSplit1, mUSplit2, mBlendEps,
                mGrazCosL, mGrazSinL, true);

            if (mIsZAxis) return pos_offset + Vec3d(dx, dy, zc);
            return pos_offset + dx * mAxE1 + dy * mAxE2 + zc * mAxis;
        } else if (v < mV2) {
            // Front-Cap: 终点处半球 (左→前→右)
            double w = (v - mV1) / (mV2 - mV1);
            phi = mPhiGrazingL - w * M_PI;
            pos_offset = mSeg.end;
        } else if (v < mV3) {
            // Right-Mid: 沿路径从终点回到起点 (擦掠线)
            double t = 1.0 - (v - mV2) / (mV3 - mV2);
            pos_offset = mSeg.start + t * mLength * mDir;

            double r = rProfile(u);
            double z = zProfile(u);
            auto [dx, dy, zc] = grazingRadial(u, r, z,
                mCornerCx, 0.0, mCornerCz, mCornerRad,
                m_r_xy, mE1_x, mE1_y, mE2_x, mE2_y, mE2_z,
                mUSplit1, mUSplit2, mBlendEps,
                mGrazCosR, mGrazSinR, false);

            if (mIsZAxis) return pos_offset + Vec3d(dx, dy, zc);
            return pos_offset + dx * mAxE1 + dy * mAxE2 + zc * mAxis;
        } else {
            // Back-Cap: 起点处半球 (右→后→左)
            double w = (v - mV3) / (1.0 - mV3);
            phi = mPhiGrazingR - w * M_PI;
            pos_offset = mSeg.start;
        }
    }

    double lx = r * std::cos(phi);
    double ly = r * std::sin(phi);
    if (mIsZAxis) return pos_offset + Vec3d(lx, ly, z);
    return pos_offset + lx * mAxE1 + ly * mAxE2 + z * mAxis;
}

Vec3d ToolSweepSurface::normal(double u, double v) const {
    Vec3d n = nProfile(u); // (nr, 0, nz) in local polar

    if (mLength < 1e-12) {
        double phi = mPhiGrazingL + v * 2.0 * M_PI;
        double nx = n.x() * std::cos(phi), ny = n.x() * std::sin(phi), nz = n.z();
        if (mIsZAxis) return Vec3d(nx, ny, nz);
        return nx * mAxE1 + ny * mAxE2 + nz * mAxis;
    }

    if (v < mV1) {
        // Left-Mid: 擦掠线
        if (u >= mUSplit2 || u < mUSplit1 || m_r_xy < 1e-12) {
            double nx = mGrazCosL, ny = mGrazSinL, nz = 0.0;
            if (mIsZAxis) return Vec3d(nx, ny, nz);
            return nx * mAxE1 + ny * mAxE2 + nz * mAxis;
        }
        double r = rProfile(u);
        double z = zProfile(u);
        auto [dx, dy, zc] = grazingRadial(u, r, z,
            mCornerCx, 0.0, mCornerCz, mCornerRad,
            m_r_xy, mE1_x, mE1_y, mE2_x, mE2_y, mE2_z,
            mUSplit1, mUSplit2, mBlendEps,
            mGrazCosL, mGrazSinL, true);
        double nx = (dx - mCornerCx) / mCornerRad;
        double ny = (dy - 0.0) / mCornerRad;
        double nz = (zc - mCornerCz) / mCornerRad;
        double invLen = 1.0 / std::sqrt(nx*nx + ny*ny + nz*nz);
        nx *= invLen; ny *= invLen; nz *= invLen;
        if (mIsZAxis) return Vec3d(nx, ny, nz);
        return nx * mAxE1 + ny * mAxE2 + nz * mAxis;
    }

    if (v >= mV2 && v < mV3) {
        // Right-Mid: 擦掠线
        if (u >= mUSplit2 || u < mUSplit1 || m_r_xy < 1e-12) {
            double nx = mGrazCosR, ny = mGrazSinR, nz = 0.0;
            if (mIsZAxis) return Vec3d(nx, ny, nz);
            return nx * mAxE1 + ny * mAxE2 + nz * mAxis;
        }
        double r = rProfile(u);
        double z = zProfile(u);
        auto [dx, dy, zc] = grazingRadial(u, r, z,
            mCornerCx, 0.0, mCornerCz, mCornerRad,
            m_r_xy, mE1_x, mE1_y, mE2_x, mE2_y, mE2_z,
            mUSplit1, mUSplit2, mBlendEps,
            mGrazCosR, mGrazSinR, false);
        double nx = (dx - mCornerCx) / mCornerRad;
        double ny = (dy - 0.0) / mCornerRad;
        double nz = (zc - mCornerCz) / mCornerRad;
        double invLen = 1.0 / std::sqrt(nx*nx + ny*ny + nz*nz);
        nx *= invLen; ny *= invLen; nz *= invLen;
        if (mIsZAxis) return Vec3d(nx, ny, nz);
        return nx * mAxE1 + ny * mAxE2 + nz * mAxis;
    }

    // Front-Cap / Back-Cap
    double phi = 0;
    if (v < mV2) {
        double w = (v - mV1) / (mV2 - mV1);
        phi = mPhiGrazingL - w * M_PI;
    } else {
        double w = (v - mV3) / (1.0 - mV3);
        phi = mPhiGrazingR - w * M_PI;
    }
    double nx = n.x() * std::cos(phi), ny = n.x() * std::sin(phi), nz = n.z();
    if (mIsZAxis) return Vec3d(nx, ny, nz);
    return nx * mAxE1 + ny * mAxE2 + nz * mAxis;
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
