#pragma once
#include <vector>
#include <cstdint>
#include "AppState.h"

namespace midgard {

// 道具网格生成器
// 刀具原点在 (0,0,0) 尖端，轴沿 +Z
class ToolMeshGenerator {
public:
    // Ball End: 圆柱体 + 半球
    static void buildBallEnd(double R, double H,
                             std::vector<float>& verts,
                             std::vector<uint32_t>& indices);

    // Flat End: 圆柱体
    static void buildFlatEnd(double R, double H,
                             std::vector<float>& verts,
                             std::vector<uint32_t>& indices);

    // Bull Nose: 圆柱体 + 圆角
    static void buildBullNose(double R, double r, double H,
                              std::vector<float>& verts,
                              std::vector<uint32_t>& indices);

    // 线框版本
    static void buildBallEndWireframe(double R, double H,
                                      std::vector<float>& lines);
    static void buildFlatEndWireframe(double R, double H,
                                      std::vector<float>& lines);
    static void buildBullNoseWireframe(double R, double r, double H,
                                       std::vector<float>& lines);
};

} // namespace midgard