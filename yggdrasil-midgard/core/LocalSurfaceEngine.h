#pragma once
#include "core/Types.h"
#include <vector>

namespace midgard {

/**
 * @brief 基于 Voxel 内既有点云的特征感知局部隐式场引擎。
 * 
 * 使用多法向半空间约束 (Multi-Normal Half-space Constraint) 来计算 SDF。
 * 在脊线 (Ridge) 区域，采用 max(dot_product) 逻辑，确保切削点不会在空气侧误采样。
 */
class LocalSurfaceEngine {
public:
    /**
     * @brief 构造函数
     * @param positions 既有点的位置
     * @param normals 既有的法向 (指向空气)
     * @param bbox 当前 Voxel 的物理 AABB
     */
    LocalSurfaceEngine(const std::vector<Vec3f>& positions, 
                       const std::vector<Vec3f>& normals,
                       const openvdb::BBoxd& bbox);
    
    /// 计算点 p 处的 SDF 值 (<0 = 内部, >0 = 外部)
    double eval(const Vec3d& p) const;
    
    bool isEmpty() const { return mPositions.empty(); }

private:
    std::vector<Vec3f> mPositions;
    std::vector<Vec3f> mNormals;
    openvdb::BBoxd mVoxelBox;

    int mRes = 0;                    // 格栅分辨率 (0, 2, 4, 8)
    std::vector<int> mGridHead;      // 每个 Cell 的起始偏移 (Size = Res^3 + 1)
    std::vector<int> mPointIndices;  // 按 Cell 排布的点索引数组

    /// 将归一化坐标转换为一维格栅索引
    inline int encode(int ix, int iy, int iz) const {
        if (mRes == 2) return (ix << 2) | (iy << 1) | iz;       // 2x2x2: (ix*4 + iy*2 + iz)
        if (mRes == 4) return (ix << 4) | (iy << 2) | iz;       // 4x4x4: (ix*16 + iy*4 + iz)
        return (ix << 6) | (iy << 3) | iz;                      // 8x8x8: (ix*64 + iy*8 + iz)
    }
};

} // namespace midgard
