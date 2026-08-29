// pseudocode.cpp — Marching Cubes 误差分析伪代码实现
// 注意: 类C++伪代码，不可直接编译，用于展示算法逻辑

#include "interface.h"
#include <cmath>
#include <algorithm>

namespace midgard {

// ============================================================
// MCErrorAnalyzer 实现
// ============================================================

MCErrorAnalyzer::MCErrorAnalyzer(const VoxelConfig& config)
    : mConfig(config) {}

ErrorMetrics MCErrorAnalyzer::computeTheoreticalBounds(
    double maxCurvature, double surfaceAngle) const 
{
    ErrorMetrics metrics;
    
    // 定理 1: 位置误差上界 V^2 * κ / 8
    metrics.maxPositionError = positionErrorBound(maxCurvature);
    metrics.meanPositionError = metrics.maxPositionError * 0.5;  // 经验: 平均约最大的一半
    
    // 定理 2: 法线误差上界 V^2 * κ / 6
    metrics.maxNormalDeviation = normalErrorBound(maxCurvature);
    metrics.meanNormalDeviation = metrics.maxNormalDeviation * 0.5;
    
    // 体积误差: O(V^2 * H)，H 为平均曲率
    metrics.volumeError = mConfig.voxelSize * mConfig.voxelSize * maxCurvature;
    
    // 阶梯效应: V * |cos α|
    metrics.staircaseMetric = staircaseHeight(surfaceAngle) / mConfig.voxelSize;
    
    return metrics;
}

ErrorMetrics MCErrorAnalyzer::computeActualError(
    const std::vector<Vec3d>& mcVertices,
    const std::vector<Vec3i>& mcTriangles,
    std::function<double(const Vec3d&)> exactSDF,
    std::function<Vec3d(const Vec3d&)> exactGradient,
    const openvdb::BBoxd& bbox) const
{
    ErrorMetrics metrics = {};
    metrics.maxPositionError = 0;
    metrics.maxNormalDeviation = 0;
    
    // 采样评估点 (在 bbox 内均匀采样)
    const int sampleCount = 1000;
    double totalPosError = 0;
    double totalNormError = 0;
    int validSamples = 0;
    
    for (int i = 0; i < sampleCount; ++i) {
        // 生成采样点 (伪随机或网格)
        Vec3d p = samplePoint(bbox, i, sampleCount);
        
        // 找到最近的 MC 表面点
        Vec3d closestPoint = findClosestPointOnMesh(p, mcVertices, mcTriangles);
        
        // 位置误差: |p_mesh - p_exact| where f(p_exact) = 0
        double sdfVal = exactSDF(closestPoint);
        Vec3d grad = exactGradient(closestPoint);
        grad.normalize();
        
        // 将 closestPoint 投影到真实零等值面
        Vec3d exactPoint = closestPoint - sdfVal * grad;
        double posError = (closestPoint - exactPoint).length();
        
        // 法线误差
        Vec3d exactNormal = grad;
        Vec3d meshNormal = computeMeshNormalAtPoint(closestPoint, mcVertices, mcTriangles);
        double normError = std::acos(std::clamp(meshNormal.dot(exactNormal), -1.0, 1.0));
        
        metrics.maxPositionError = std::max(metrics.maxPositionError, posError);
        metrics.maxNormalDeviation = std::max(metrics.maxNormalDeviation, normError);
        totalPosError += posError;
        totalNormError += normError;
        validSamples++;
    }
    
    if (validSamples > 0) {
        metrics.meanPositionError = totalPosError / validSamples;
        metrics.meanNormalDeviation = totalNormError / validSamples;
    }
    
    // 体积误差: 用蒙特卡洛积分估计
    metrics.volumeError = estimateVolumeError(mcVertices, mcTriangles, exactSDF, bbox);
    
    return metrics;
}

bool MCErrorAnalyzer::needsSubdivision(double localCurvature, double tolerance) const {
    // 如果理论位置误差超过容差，需要细分
    double posError = positionErrorBound(localCurvature);
    return posError > tolerance;
}

double MCErrorAnalyzer::estimateStaircase(const std::vector<Vec3d>& normals) const {
    if (normals.size() < 2) return 0;
    
    // 计算相邻法线的角度变化
    double totalVariation = 0;
    for (size_t i = 1; i < normals.size(); ++i) {
        double cosAngle = normals[i].dot(normals[i-1]);
        cosAngle = std::clamp(cosAngle, -1.0, 1.0);
        double angle = std::acos(cosAngle);
        totalVariation += angle;
    }
    
    // 归一化: 平均角度变化 / (π/2)
    double avgVariation = totalVariation / (normals.size() - 1);
    return std::min(avgVariation / (M_PI / 2.0), 1.0);
}

// ============================================================
// AdaptiveMCController 实现
// ============================================================

AdaptiveMCController::AdaptiveMCController(const SubdivisionRule& rule)
    : mRule(rule) {}

int AdaptiveMCController::computeSubdivisionLevel(
    double localCurvature, double baseVoxelSize) const 
{
    if (localCurvature < mRule.curvatureThreshold) {
        return 0;  // 不需要细分
    }
    
    // 计算需要的细分层数使误差满足阈值
    // 误差 = (V/2^L)^2 * κ / 8 <= threshold
    // => 2^L >= V * sqrt(κ / (8 * threshold))
    // => L >= log2(V * sqrt(κ / (8 * threshold)))
    
    double requiredResolution = baseVoxelSize * std::sqrt(
        localCurvature / (8.0 * mRule.curvatureThreshold));
    
    int level = 0;
    double currentSize = baseVoxelSize;
    while (currentSize > mRule.minVoxelSize && level < mRule.maxLevel) {
        if (currentSize / 2.0 < requiredResolution) break;
        currentSize /= 2.0;
        level++;
    }
    
    return level;
}

// ============================================================
// 辅助函数
// ============================================================

Vec3d samplePoint(const openvdb::BBoxd& bbox, int index, int total) {
    // 使用低差异序列 (如 Halton) 进行均匀采样
    double u = haltonSequence(index, 2);  // 基2
    double v = haltonSequence(index, 3);  // 基3
    double w = haltonSequence(index, 5);  // 基5
    
    return Vec3d(
        bbox.min().x() + u * (bbox.max().x() - bbox.min().x()),
        bbox.min().y() + v * (bbox.max().y() - bbox.min().y()),
        bbox.min().z() + w * (bbox.max().z() - bbox.min().z())
    );
}

Vec3d findClosestPointOnMesh(const Vec3d& query,
                             const std::vector<Vec3d>& vertices,
                             const std::vector<Vec3i>& triangles) {
    Vec3d closest;
    double minDist = 1e308;
    
    for (const auto& tri : triangles) {
        Vec3d a = vertices[tri.x()];
        Vec3d b = vertices[tri.y()];
        Vec3d c = vertices[tri.z()];
        
        Vec3d candidate = closestPointOnTriangle(query, a, b, c);
        double dist = (candidate - query).lengthSqr();
        
        if (dist < minDist) {
            minDist = dist;
            closest = candidate;
        }
    }
    
    return closest;
}

Vec3d closestPointOnTriangle(const Vec3d& p, const Vec3d& a, 
                              const Vec3d& b, const Vec3d& c) {
    // 使用重心坐标计算最近点
    Vec3d ab = b - a;
    Vec3d ac = c - a;
    Vec3d ap = p - a;
    
    double d1 = ab.dot(ap);
    double d2 = ac.dot(ap);
    if (d1 <= 0 && d2 <= 0) return a;  // 顶点 A
    
    Vec3d bp = p - b;
    double d3 = ab.dot(bp);
    double d4 = ac.dot(bp);
    if (d3 >= 0 && d4 <= d3) return b;  // 顶点 B
    
    double vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) {
        double v = d1 / (d1 - d3);
        return a + v * ab;  // 边 AB
    }
    
    Vec3d cp = p - c;
    double d5 = ab.dot(cp);
    double d6 = ac.dot(cp);
    if (d6 >= 0 && d5 <= d6) return c;  // 顶点 C
    
    double vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) {
        double w = d2 / (d2 - d6);
        return a + w * ac;  // 边 AC
    }
    
    double va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
        double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + w * (c - b);  // 边 BC
    }
    
    // 内部
    double denom = 1.0 / (va + vb + vc);
    double v = vb * denom;
    double w = vc * denom;
    return a + ab * v + ac * w;
}

Vec3d computeMeshNormalAtPoint(const Vec3d& p,
                               const std::vector<Vec3d>& vertices,
                               const std::vector<Vec3i>& triangles) {
    // 找到包含 p 的三角形，返回其法线
    // (简化: 假设 p 在三角形上，找到最近三角形)
    Vec3d closest;
    double minDist = 1e308;
    Vec3d bestNormal;
    
    for (const auto& tri : triangles) {
        Vec3d a = vertices[tri.x()];
        Vec3d b = vertices[tri.y()];
        Vec3d c = vertices[tri.z()];
        
        Vec3d candidate = closestPointOnTriangle(p, a, b, c);
        double dist = (candidate - p).lengthSqr();
        
        if (dist < minDist) {
            minDist = dist;
            Vec3d n = (b - a).cross(c - a);
            n.normalize();
            bestNormal = n;
        }
    }
    
    return bestNormal;
}

double estimateVolumeError(const std::vector<Vec3d>& vertices,
                           const std::vector<Vec3i>& triangles,
                           std::function<double(const Vec3d&)> exactSDF,
                           const openvdb::BBoxd& bbox) {
    // 蒙特卡洛体积估计
    const int samples = 10000;
    int insideMC = 0;
    int insideExact = 0;
    
    for (int i = 0; i < samples; ++i) {
        Vec3d p = samplePoint(bbox, i, samples);
        
        // 精确 SDF: 负值表示内部
        bool inExact = exactSDF(p) < 0;
        
        // MC 网格: 射线投射判断内外
        bool inMC = pointInsideMesh(p, vertices, triangles);
        
        if (inExact) insideExact++;
        if (inMC) insideMC++;
    }
    
    double volExact = (double)insideExact / samples * bboxVolume(bbox);
    double volMC = (double)insideMC / samples * bboxVolume(bbox);
    
    if (volExact == 0) return 0;
    return std::abs(volMC - volExact) / volExact;
}

bool pointInsideMesh(const Vec3d& p,
                     const std::vector<Vec3d>& vertices,
                     const std::vector<Vec3i>& triangles) {
    // 射线投射算法
    Vec3d rayDir(1, 0, 0);  // 沿 X 轴
    int crossings = 0;
    
    for (const auto& tri : triangles) {
        if (rayTriangleIntersect(p, rayDir, 
                                 vertices[tri.x()], 
                                 vertices[tri.y()], 
                                 vertices[tri.z()])) {
            crossings++;
        }
    }
    
    return (crossings % 2) == 1;  // 奇数次穿越 = 内部
}

bool rayTriangleIntersect(const Vec3d& orig, const Vec3d& dir,
                          const Vec3d& a, const Vec3d& b, const Vec3d& c) {
    Vec3d ab = b - a;
    Vec3d ac = c - a;
    Vec3d h = dir.cross(ac);
    double det = ab.dot(h);
    
    if (std::abs(det) < 1e-10) return false;  // 平行
    
    double invDet = 1.0 / det;
    Vec3d s = orig - a;
    double u = invDet * s.dot(h);
    
    if (u < 0 || u > 1) return false;
    
    Vec3d q = s.cross(ab);
    double v = invDet * dir.dot(q);
    
    if (v < 0 || u + v > 1) return false;
    
    double t = invDet * ac.dot(q);
    return t > 1e-10;  // 在射线正方向相交
}

double bboxVolume(const openvdb::BBoxd& bbox) {
    Vec3d size = bbox.max() - bbox.min();
    return size.x() * size.y() * size.z();
}

double haltonSequence(int index, int base) {
    double result = 0;
    double f = 1.0 / base;
    int i = index;
    while (i > 0) {
        result += f * (i % base);
        i /= base;
        f /= base;
    }
    return result;
}

} // namespace midgard
