#include "core/LocalSurfaceEngine.h"
#include <algorithm>

namespace midgard {

LocalSurfaceEngine::LocalSurfaceEngine(const std::vector<Vec3f>& positions, 
                                       const std::vector<Vec3f>& normals,
                                       const openvdb::BBoxd& bbox)
    : mPositions(positions), mNormals(normals), mVoxelBox(bbox)
{
    size_t N = mPositions.size();
    if (N < 16) {
        mRes = 0; // 规模太小，直接暴力扫描更划算
        return;
    }

    // 1. 决定格栅分辨率
    if (N < 64) mRes = 2;
    else if (N < 256) mRes = 4;
    else mRes = 8;

    int numCells = mRes * mRes * mRes;
    mGridHead.assign(numCells + 1, 0);
    mPointIndices.resize(N);

    // 2. 统计每个 Cell 的点数
    std::vector<int> counts(numCells, 0);
    Vec3d size = mVoxelBox.max() - mVoxelBox.min();
    Vec3d invSize(1.0 / (size.x() + 1e-15), 1.0 / (size.y() + 1e-15), 1.0 / (size.z() + 1e-15));

    auto getCellIdx = [&](const Vec3f& p) {
        int ix = std::clamp((int)((p.x() - mVoxelBox.min().x()) * invSize.x() * mRes), 0, mRes - 1);
        int iy = std::clamp((int)((p.y() - mVoxelBox.min().y()) * invSize.y() * mRes), 0, mRes - 1);
        int iz = std::clamp((int)((p.z() - mVoxelBox.min().z()) * invSize.z() * mRes), 0, mRes - 1);
        return encode(ix, iy, iz);
    };

    for (const auto& p : mPositions) {
        counts[getCellIdx(p)]++;
    }

    // 3. 构建 Prefix Sum (mGridHead)
    int offset = 0;
    for (int i = 0; i < numCells; ++i) {
        mGridHead[i] = offset;
        offset += counts[i];
    }
    mGridHead[numCells] = offset;

    // 4. 填充点索引 (使用局部计数器重置)
    std::fill(counts.begin(), counts.end(), 0);
    for (int i = 0; i < (int)N; ++i) {
        int cellIdx = getCellIdx(mPositions[i]);
        mPointIndices[mGridHead[cellIdx] + counts[cellIdx]] = i;
        counts[cellIdx]++;
    }
}

double LocalSurfaceEngine::eval(const Vec3d& p) const {
    if (mPositions.empty()) return 1e9; 

    // Level 0: 暴力扫描
    if (mRes == 0) {
        double maxSdf = -1e12;
        for (size_t i = 0; i < mPositions.size(); ++i) {
            double s = (p - Vec3d(mPositions[i])).dot(Vec3d(mNormals[i]));
            if (s > maxSdf) maxSdf = s;
        }
        return maxSdf;
    }

    // Level 1-3: 格栅查询 (1-ring 邻域扫描确保覆盖)
    Vec3d size = mVoxelBox.max() - mVoxelBox.min();
    Vec3d rel = (p - mVoxelBox.min());
    int ix = std::clamp((int)(rel.x() / (size.x() + 1e-15) * mRes), 0, mRes - 1);
    int iy = std::clamp((int)(rel.y() / (size.y() + 1e-15) * mRes), 0, mRes - 1);
    int iz = std::clamp((int)(rel.z() / (size.z() + 1e-15) * mRes), 0, mRes - 1);

    double maxSdf = -1e12;
    bool foundAny = false;

    for (int dx = -1; dx <= 1; ++dx) {
        int curX = ix + dx;
        if (curX < 0 || curX >= mRes) continue;
        for (int dy = -1; dy <= 1; ++dy) {
            int curY = iy + dy;
            if (curY < 0 || curY >= mRes) continue;
            for (int dz = -1; dz <= 1; ++dz) {
                int curZ = iz + dz;
                if (curZ < 0 || curZ >= mRes) continue;

                int cellIdx = encode(curX, curY, curZ);
                for (int i = mGridHead[cellIdx]; i < mGridHead[cellIdx + 1]; ++i) {
                    int ptIdx = mPointIndices[i];
                    double s = (p - Vec3d(mPositions[ptIdx])).dot(Vec3d(mNormals[ptIdx]));
                    if (s > maxSdf) maxSdf = s;
                    foundAny = true;
                }
            }
        }
    }

    // 如果 1-ring 邻域都空了（对于分布极不均的点云），回退到全局扫描
    if (!foundAny) {
        for (size_t i = 0; i < mPositions.size(); ++i) {
            double s = (p - Vec3d(mPositions[i])).dot(Vec3d(mNormals[i]));
            if (s > maxSdf) maxSdf = s;
        }
    }

    return maxSdf;
}

} // namespace midgard
