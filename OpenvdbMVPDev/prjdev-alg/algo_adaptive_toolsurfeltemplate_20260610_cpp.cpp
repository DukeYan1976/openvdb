#include <vector>
#include <cmath>
#include <algorithm>

struct LocalSurfel {
    float x, y, z;
    float nx, ny, nz;
};

// 工具函数：根据目标精度和曲率计算自适应步长
float computeAdaptiveStep(float R, float target_delta, float max_step) {
    if (R < 1e-6) return target_delta; // 极小半径或尖角，回退到目标精度分辨率
    if (R > 1e6) return max_step;      // 平面或直墙，使用上限保证宏观网格连通
    
    // 弦截差核心公式: dv = sqrt(8 * R * delta)
    float dv = std::sqrt(8.0f * R * target_delta);
    return std::min(dv, max_step);     // 不可超过 Dv/2 的上限约束
}

/**
 * @brief 构建基于曲率自适应的刀具面元模板 (以非解析 2D 轮廓旋转体为例)
 * @param profileCurve  刀具半剖面的离散采样或解析曲线 [z -> (r, R_curvature)]
 * @param D_v           宏观网格 FloatGrid 分辨率 (如 0.5mm)
 * @param delta_target  微观几何逼近目标精度 (如 0.01mm)
 */
std::vector<LocalSurfel> buildAdaptiveToolTemplate(
    const ToolProfile& profile, 
    float D_v, 
    float delta_target) 
{
    std::vector<LocalSurfel> surfels;
    
    // 强制约束上限: 确保至少有 2 个点落在一个宏观 Voxel 内
    const float d_v_max = D_v / 2.0f; 
    
    // 沿刀具轮廓的弧长参数 s 进行自适应积分
    float current_s = 0.0f;
    float max_s = profile.getTotalArcLength();
    
    while (current_s <= max_s) {
        ProfilePoint pt = profile.evaluateAt(current_s); // 获取位置、法向、局部曲率半径 R
        
        // 1. 计算纵向(沿轮廓)的自适应步长
        float ds = computeAdaptiveStep(pt.radius_of_curvature, delta_target, d_v_max);
        
        // 2. 对于旋转刀具，横向(圆周)曲率等于当前回转半径 r
        // 如果是尖角处 (R_curvature = 0)，此时 r 不为 0 (刀具半径)，圆周需密集采样
        float r = pt.radial_distance;
        float d_theta_arc = computeAdaptiveStep(r, delta_target, d_v_max);
        
        // 防止除零或极小切片
        if (r > 1e-4) {
            float d_theta = d_theta_arc / r; 
            int num_slices = std::max(4, (int)std::ceil(2.0f * M_PI / d_theta));
            
            // 环绕 Z 轴生成该层的所有 Surfels
            for (int i = 0; i < num_slices; ++i) {
                float theta = i * (2.0f * M_PI / num_slices);
                LocalSurfel s;
                s.x = r * std::cos(theta);
                s.y = r * std::sin(theta);
                s.z = pt.z;
                
                // 法向旋转
                s.nx = pt.normal_r * std::cos(theta);
                s.ny = pt.normal_r * std::sin(theta);
                s.nz = pt.normal_z;
                
                surfels.push_back(s);
            }
        } else {
            // 刀尖顶点 (底心)
            LocalSurfel s = {0, 0, pt.z, 0, 0, -1.0f};
            surfels.push_back(s);
        }
        
        // 步进到下一个轮廓点
        // 对于平底刀角点，ds 会自动缩减到 delta_target，形成致密的特征线簇
        current_s += ds;
        
        // 保证尾点处理
        if (current_s > max_s && current_s - ds < max_s) {
            current_s = max_s; 
        }
    }
    
    return surfels;
}