#pragma once

#include "core/Types.h"
#include "core/ToolSweptSDF.h"
#include <openvdb/math/BBox.h>
#include <vector>

namespace midgard {

// ─────────────────────────────────────────────────────────────
// 表面采样点
// ─────────────────────────────────────────────────────────────
struct SurfaceSample {
    Vec3d position;            // 世界坐标
    Vec3d normal;              // 指向材料外部 (= 刀具外部 = ∇tool_sdf 方向)

    SurfaceSample() = default;
    SurfaceSample(const Vec3d& pos, const Vec3d& nml)
        : position(pos), normal(nml) {}
};

// ─────────────────────────────────────────────────────────────
// 八叉树精修配置
// ─────────────────────────────────────────────────────────────
struct OctreeConfig {
    double chordalTol   = 0.01;   // 弦高容差 (mm)
    int    maxDepth     = 6;      // 最大细分深度
    double zeroCrossTol = 0.0;    // 零交叉定位精度（0=chordalTol/10）
    int    evalCount    = 0;      // [output] SDF 评估次数（调用后回填）

    /// 实际零交叉定位精度
    double effectiveZeroCrossTol() const {
        return zeroCrossTol > 0 ? zeroCrossTol : chordalTol * 0.1;
    }
};

// ─────────────────────────────────────────────────────────────
// 自适应八叉树表面提取
// ─────────────────────────────────────────────────────────────
///
/// 在给定 world-space bbox 内，用自适应八叉树细分提取 tool_sdf 的零等值面。
///
/// 算法（方案 G 精修阶段）:
///   1. 求 bbox 8 角点的 tool_sdf
///   2. 全正/全负 → 停止（无表面）
///   3. MIXED 且 cell 对角线 > chordalTol → 细分 8 子格元，递归
///   4. MIXED 且 cell 对角线 ≤ chordalTol → 提取零交叉点 + 法向
///
/// @param ceBbox  ce 的 world-space 轴对齐包围盒
/// @param toolSDF 新刀的隐式距离场
/// @param config  精修配置
/// @return 表面采样点集（可能为空）
///
std::vector<SurfaceSample> extractSurface(
    const openvdb::BBoxd& ceBbox,
    const ToolSweptSDF& toolSDF,
    OctreeConfig& config);

} // namespace midgard
