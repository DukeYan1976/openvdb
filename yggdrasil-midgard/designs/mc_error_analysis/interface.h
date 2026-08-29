#pragma once
#include "core/Types.h"
#include <vector>
#include <functional>

namespace midgard {

/**
 * @brief Marching Cubes 误差分析器
 * 
 * 提供对体素化表面提取算法的误差度量、上界估计和自适应控制。
 */
class MCErrorAnalyzer {
public:
    struct ErrorMetrics {
        double maxPositionError;      // 最大位置误差 (mm)
        double meanPositionError;     // 平均位置误差 (mm)
        double maxNormalDeviation;    // 最大法线偏差 (rad)
        double meanNormalDeviation;   // 平均法线偏差 (rad)
        double volumeError;           // 体积相对误差
        double staircaseMetric;       // 阶梯效应度量
        
        bool withinTolerance(double tolerance) const {
            return maxPositionError <= tolerance && 
                   maxNormalDeviation <= tolerance;
        }
    };
    
    struct VoxelConfig {
        double voxelSize;             // 体素尺寸 V
        double narrowBandWidth;       // 窄带半宽
        int adaptiveSubdivLevel;      // 自适应细分层数 (0=关闭)
    };

    /**
     * @brief 构造函数
     * @param config 体素配置参数
     */
    explicit MCErrorAnalyzer(const VoxelConfig& config);
    
    /**
     * @brief 计算解析曲面的理论误差上界
     * @param maxCurvature 曲面最大曲率 κ_max (1/mm)
     * @param surfaceAngle 表面与网格夹角 (rad)
     * @return 理论误差上界
     */
    ErrorMetrics computeTheoreticalBounds(
        double maxCurvature, 
        double surfaceAngle) const;
    
    /**
     * @brief 计算与解析曲面的实际误差
     * @param mcMesh 待评估的 MC 提取网格
     * @param exactSDF 解析 SDF 函数
     * @param bbox 评估区域
     * @return 实际误差度量
     */
    ErrorMetrics computeActualError(
        const std::vector<Vec3d>& mcVertices,
        const std::vector<Vec3i>& mcTriangles,
        std::function<double(const Vec3d&)> exactSDF,
        std::function<Vec3d(const Vec3d&)> exactGradient,
        const openvdb::BBoxd& bbox) const;
    
    /**
     * @brief 判断是否需要自适应细分
     * @param localCurvature 局部曲率
     * @param tolerance 目标公差
     * @return true 如果当前分辨率不足
     */
    bool needsSubdivision(double localCurvature, double tolerance) const;
    
    /**
     * @brief 估计阶梯效应严重程度
     * @param normals 相邻三角面法线
     * @return 阶梯度量 (0=平滑, 1=严重阶梯)
     */
    double estimateStaircase(const std::vector<Vec3d>& normals) const;

private:
    VoxelConfig mConfig;
    
    // 位置误差上界: V^2 * κ / 8
    double positionErrorBound(double curvature) const {
        return mConfig.voxelSize * mConfig.voxelSize * curvature / 8.0;
    }
    
    // 法线误差上界: V^2 * κ / 6
    double normalErrorBound(double curvature) const {
        return mConfig.voxelSize * mConfig.voxelSize * curvature / 6.0;
    }
    
    // 阶梯高度: V * |cos α|
    double staircaseHeight(double angle) const {
        return mConfig.voxelSize * std::abs(std::cos(angle));
    }
};

/**
 * @brief 自适应 MC 控制器
 * 
 * 根据局部误差估计动态调整体素分辨率。
 */
class AdaptiveMCController {
public:
    struct SubdivisionRule {
        double curvatureThreshold;    // 曲率阈值 (1/mm)
        int maxLevel;                 // 最大细分层数
        double minVoxelSize;          // 最小体素尺寸 (mm)
    };
    
    explicit AdaptiveMCController(const SubdivisionRule& rule);
    
    /**
     * @brief 计算局部细分级别
     * @param localCurvature 局部曲率
     * @param baseVoxelSize 基础体素尺寸
     * @return 建议的细分级别 (0 = 不细分)
     */
    int computeSubdivisionLevel(double localCurvature, double baseVoxelSize) const;
    
    /**
     * @brief 计算自适应后的有效体素尺寸
     */
    double effectiveVoxelSize(double baseVoxelSize, int level) const {
        return baseVoxelSize / (1 << level);
    }

private:
    SubdivisionRule mRule;
};

} // namespace midgard
