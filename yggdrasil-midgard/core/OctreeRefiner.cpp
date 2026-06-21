#include "core/OctreeRefiner.h"
#include <algorithm>
#include <cmath>
#include <unordered_set>
#include <utility>

namespace midgard {

namespace {

/// 3 种分类状态
enum class CellState { ALL_POS, ALL_NEG, MIXED };

/// 归类 bbox 的 8 角点 SDF
CellState classifyCorners(const float sdf[8], double threshold) {
    bool hasPos = false, hasNeg = false;
    for (int i = 0; i < 8; ++i) {
        if (sdf[i] >  threshold) hasPos = true;
        if (sdf[i] < -threshold) hasNeg = true;
        if (hasPos && hasNeg) return CellState::MIXED;
    }
    if (hasPos) return CellState::ALL_POS;
    if (hasNeg) return CellState::ALL_NEG;
    // All within [-thresh, +thresh] → cross at some edge anyway → MIXED
    return CellState::MIXED;
}

/// 8 角点坐标（单位 cube [0,1]³）
const Vec3d kUnitCorners[8] = {
    {0,0,0}, {1,0,0}, {0,1,0}, {1,1,0},
    {0,0,1}, {1,0,1}, {0,1,1}, {1,1,1}
};

/// 12 条边（角点索引对）
const int kEdges[12][2] = {
    {0,1},{2,3},{4,5},{6,7}, // X-edges
    {0,2},{1,3},{4,6},{5,7}, // Y-edges
    {0,4},{1,5},{2,6},{3,7}  // Z-edges
};

/// 将单位 cube 映射到 world bbox
Vec3d mapToWorld(const Vec3d& unit, const openvdb::BBoxd& bbox) {
    Vec3d size  = bbox.extents();
    Vec3d origin = bbox.min();
    return Vec3d(origin.x() + unit.x() * size.x(),
                 origin.y() + unit.y() * size.y(),
                 origin.z() + unit.z() * size.z());
}

/// 零交叉二分搜索
/// 在 [a, b] 上找 tool_sdf = 0 的点，精度 zeroTol
Vec3d findZeroCrossing(const Vec3d& a, const Vec3d& b,
                        const ToolSweptSDF& toolSDF,
                        double zeroTol) {
    Vec3d lo = a, hi = b;
    double sdfLo = toolSDF.eval(lo);
    double sdfHi = toolSDF.eval(hi);

    // 确保 lo 为负，hi 为正（负 = 刀内部）
    if (sdfLo > 0) {
        std::swap(lo, hi);
        std::swap(sdfLo, sdfHi);
    }

    for (int iter = 0; iter < 16; ++iter) {
        Vec3d mid = (lo + hi) * 0.5;
        if ((mid - lo).length() < zeroTol) return mid;

        double sdfMid = toolSDF.eval(mid);
        if (sdfMid < 0)
            lo = mid;
        else
            hi = mid;
    }
    return (lo + hi) * 0.5;
}

/// 提取 leaf cell 的零交叉点
/// 用边哈希去重：同一条边被相邻 cell 共享时只采一次
using EdgeKey = std::pair<Vec3d, Vec3d>;

void normalizeEdge(Vec3d& a, Vec3d& b) {
    if (a.x() > b.x() || (a.x() == b.x() && a.y() > b.y()) ||
        (a.x() == b.x() && a.y() == b.y() && a.z() > b.z())) {
        std::swap(a, b);
    }
}

void extractLeafPoints(const openvdb::BBoxd& bbox,
                        const float sdf[8],
                        const ToolSweptSDF& toolSDF,
                        double zeroTol,
                        std::vector<SurfaceSample>& out,
                        std::unordered_set<uint64_t>& edgeHash) {
    Vec3d cornersW[8];
    for (int i = 0; i < 8; ++i)
        cornersW[i] = mapToWorld(kUnitCorners[i], bbox);

    for (const auto& e : kEdges) {
        int i0 = e[0], i1 = e[1];
        float s0 = sdf[i0], s1 = sdf[i1];

        // 符号相同 → 无跨零
        if ((s0 > 0 && s1 > 0) || (s0 < 0 && s1 < 0)) continue;
        // 两者都非常接近零 → 跳过边界情况
        if (std::abs(s0) < zeroTol * 1e-4 && std::abs(s1) < zeroTol * 1e-4) continue;

        const Vec3d& p0 = cornersW[i0];
        const Vec3d& p1 = cornersW[i1];

        // 边哈希去重
        Vec3d ea = p0, eb = p1;
        normalizeEdge(ea, eb);
        // 简单哈希：X*1e6 + Y*1e3 + Z 的整数部分（适用于 mm 尺度）
        int64_t hx = static_cast<int64_t>(ea.x() * 1e6);
        int64_t hy = static_cast<int64_t>(ea.y() * 1e6);
        int64_t hz = static_cast<int64_t>(ea.z() * 1e6);
        uint64_t h = static_cast<uint64_t>(hx * 73856093ULL + hy * 19349663ULL + hz * 83492791ULL);
        if (!edgeHash.insert(h).second) continue;  // 已提取过

        // 二分搜索零交叉
        Vec3d crossPt = findZeroCrossing(p0, p1, toolSDF, zeroTol);

        // 解析法向 = ∇tool_sdf（指向刀具外部 = 材料外部）
        Vec3d normal = toolSDF.gradient(crossPt);
        double len = normal.length();
        if (len > 1e-12)
            normal *= 1.0 / len;

        out.emplace_back(crossPt, normal);
    }
}

/// 递归八叉树细分
void refineOctant(const openvdb::BBoxd& bbox,
                   int depth,
                   const ToolSweptSDF& toolSDF,
                   const OctreeConfig& config,
                   std::vector<SurfaceSample>& out,
                   std::unordered_set<uint64_t>& edgeHash,
                   int& evalCount) {
    // 求 8 角点 SDF
    float sdf[8];
    for (int i = 0; i < 8; ++i) {
        Vec3d wc = mapToWorld(kUnitCorners[i], bbox);
        sdf[i] = static_cast<float>(toolSDF.eval(wc));
    }
    evalCount += 8;

    // 分类
    double eps = 1e-9;  // 数值容差
    CellState state = classifyCorners(sdf, eps);

    if (state == CellState::ALL_POS || state == CellState::ALL_NEG)
        return;  // 完全在刀外/内 → 无表面

    // 检查终止条件
    double diag = bbox.extents().length();
    bool stopBySize  = (diag <= config.chordalTol);
    bool stopByDepth = (depth >= config.maxDepth);

    if (stopBySize || stopByDepth) {
        // 中心 SDF 弦高验证：cell 中心离零面太远 → 曲率大/ grazing angle
        // 仅在 size 达标的 leaf 检查（深度强制终止时不检查）
        bool needMore = false;
        if (stopBySize && !stopByDepth) {
            Vec3d center = bbox.min() + bbox.extents() * 0.5;
            double centerSDF = toolSDF.eval(center);
            if (std::abs(centerSDF) > config.chordalTol * 1.2) {
                needMore = true;
            }
        }

        if (!needMore) {
            extractLeafPoints(bbox, sdf, toolSDF,
                              config.effectiveZeroCrossTol(), out, edgeHash);
            return;
        }
        // needMore → fall through to subdivide (stay inside the else-branch)
        evalCount += 1;  // center sdf eval
    }

    // 细分 8 个子格元
    Vec3d halfExt = bbox.extents() * 0.5;
    Vec3d mid = bbox.min() + halfExt;

    for (int child = 0; child < 8; ++child) {
        double cx = (child & 1) ? mid.x() : bbox.min().x();
        double cy = (child & 2) ? mid.y() : bbox.min().y();
        double cz = (child & 4) ? mid.z() : bbox.min().z();

        openvdb::BBoxd childBBox(Vec3d(cx, cy, cz),
                                  Vec3d(cx + halfExt.x(),
                                        cy + halfExt.y(),
                                        cz + halfExt.z()));
        refineOctant(childBBox, depth + 1, toolSDF, config, out, edgeHash, evalCount);
    }
}

} // anonymous namespace

std::vector<SurfaceSample> extractSurface(
    const openvdb::BBoxd& ceBbox,
    const ToolSweptSDF& toolSDF,
    OctreeConfig& config) {

    std::vector<SurfaceSample> points;
    std::unordered_set<uint64_t> edgeHash;
    config.evalCount = 0;

    refineOctant(ceBbox, 0, toolSDF, config, points, edgeHash, config.evalCount);

    return points;
}

} // namespace midgard
