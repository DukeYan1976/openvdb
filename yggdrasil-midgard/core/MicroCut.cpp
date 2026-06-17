#include "core/MicroCut.h"
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <openvdb/points/PointConversion.h>
#include <openvdb/points/PointCount.h>
#include <chrono>
#include <unordered_set>
#include <cmath>

namespace midgard {

void MicroCut::quadtreeEval(
    double u0, double u1, double v0, double v1,
    const openvdb::BBoxd& voxelAABB,
    const ToolSweepSurface& surface,
    double chordalLimit,
    int depth,
    PointBuffer& output,
    const openvdb::FloatGrid::ConstAccessor* billetAcc,
    const openvdb::math::Transform* billetXform)
{
    // 子域中心点
    double uMid = (u0 + u1) * 0.5;
    double vMid = (v0 + v1) * 0.5;
    Vec3d pCenter = surface.eval(uMid, vMid);

    // 3D 拒止: 仅在细分足够深时检查（避免粗粒度时误拒）
    if (depth > 3) {
        openvdb::BBoxd subBBox;
        subBBox.expand(pCenter);
        subBBox.expand(surface.eval(u0, v0));
        subBBox.expand(surface.eval(u1, v0));
        subBBox.expand(surface.eval(u0, v1));
        subBBox.expand(surface.eval(u1, v1));
        if (!voxelAABB.hasOverlap(subBBox)) return;
    }

    // 最大深度 → 强制输出（插值点）
    if (depth >= MAX_DEPTH) {
        Vec3d pmin = voxelAABB.min() - Vec3d(1e-6);
        Vec3d pmax = voxelAABB.max() + Vec3d(1e-6);
        if (openvdb::BBoxd(pmin, pmax).isInside(pCenter)) {
            if (billetAcc && billetXform && billetAcc->getValue(openvdb::Coord::round(
                billetXform->worldToIndex(pCenter))) > 0) return;
            Vec3d n = surface.normal(uMid, vMid);
            output.positions.push_back(Vec3f(pCenter));
            output.normals.push_back(Vec3f(n));
        }
        return;
    }

    // 弦高估算
    Vec3d corners[4] = {
        surface.eval(u0, v0),
        surface.eval(u1, v0),
        surface.eval(u0, v1),
        surface.eval(u1, v1)
    };
    Vec3d avg = (corners[0] + corners[1] + corners[2] + corners[3]) * 0.25;
    double maxChordal = (pCenter - avg).length();

    // 弦高 ≤ 容差 → 精确点
    if (maxChordal <= chordalLimit) {
        Vec3d pmin = voxelAABB.min() - Vec3d(1e-6);
        Vec3d pmax = voxelAABB.max() + Vec3d(1e-6);
        if (openvdb::BBoxd(pmin, pmax).isInside(pCenter)) {
            if (billetAcc && billetXform && billetAcc->getValue(openvdb::Coord::round(
                billetXform->worldToIndex(pCenter))) > 0) return;
            Vec3d n = surface.normal(uMid, vMid);
            output.positions.push_back(Vec3f(pCenter));
            output.normals.push_back(Vec3f(n));
        }
        return;
    }

    // 细分为4个子域
    quadtreeEval(u0, uMid, v0, vMid, voxelAABB, surface, chordalLimit, depth+1, output, billetAcc, billetXform);
    quadtreeEval(uMid, u1, v0, vMid, voxelAABB, surface, chordalLimit, depth+1, output, billetAcc, billetXform);
    quadtreeEval(u0, uMid, vMid, v1, voxelAABB, surface, chordalLimit, depth+1, output, billetAcc, billetXform);
    quadtreeEval(uMid, u1, vMid, v1, voxelAABB, surface, chordalLimit, depth+1, output, billetAcc, billetXform);
}

std::unordered_map<openvdb::Coord, PointBuffer>
MicroCut::sampleNewSurface(
    const std::vector<VoxelTask>& tasks,
    const ToolSweepSurface& surface,
    const ToolSweepSDF& sdf,
    const ToleranceConfig& config,
    const openvdb::FloatGrid::Ptr& billetGrid)
{
    auto t0 = std::chrono::high_resolution_clock::now();

    // 毛坯SDF accessor（用于过滤空气中的点）
    std::unique_ptr<openvdb::FloatGrid::ConstAccessor> billetAccPtr;
    const openvdb::FloatGrid::ConstAccessor* billetAcc = nullptr;
    const openvdb::math::Transform* billetXform = nullptr;
    if (billetGrid) {
        billetAccPtr = std::make_unique<openvdb::FloatGrid::ConstAccessor>(billetGrid->getConstAccessor());
        billetAcc = billetAccPtr.get();
        billetXform = &billetGrid->transform();
    }

    // Per-task 输出 (并行安全: 每个task写自己的buffer)
    std::vector<PointBuffer> taskBuffers(tasks.size());

    // TBB 并行: 每个task独立执行四叉树采样
    tbb::parallel_for(tbb::blocked_range<size_t>(0, tasks.size()),
        [&](const tbb::blocked_range<size_t>& range) {
            for (size_t i = range.begin(); i < range.end(); ++i) {
                const auto& task = tasks[i];
                quadtreeEval(
                    task.u_min, task.u_max,
                    task.t_min, task.t_max,
                    task.aabb,
                    surface,
                    config.chordalLimit,
                    0,
                    taskBuffers[i],
                    billetAcc,
                    billetXform);
            }
        });

    auto t1 = std::chrono::high_resolution_clock::now();

    // 按 voxel coord 聚合结果（纯数据搬运，无统计开销）
    std::unordered_map<openvdb::Coord, PointBuffer> result;
    int totalPoints = 0;
    for (size_t i = 0; i < tasks.size(); ++i) {
        if (!taskBuffers[i].positions.empty()) {
            totalPoints += taskBuffers[i].positions.size();
            result[tasks[i].origin] = std::move(taskBuffers[i]);
        }
    }

    // 统计仅在debug标签激活时执行（运行时零开销）
    DEBUG_SECTION(Phase2_Quadtree) {
        double ms = std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - t0).count();

        int totalExact = 0, totalInterpolated = 0;
        double maxSdfDeviation = 0;
        int minPts = result.empty() ? 0 : INT_MAX, maxPts = 0;

        for (const auto& [coord, buf] : result) {
            int n = buf.positions.size();
            minPts = std::min(minPts, n);
            maxPts = std::max(maxPts, n);
            for (const auto& pos : buf.positions) {
                double s = std::abs(sdf.eval(Vec3d(pos)));
                if (s > config.chordalLimit) {
                    totalInterpolated++;
                    maxSdfDeviation = std::max(maxSdfDeviation, s);
                } else {
                    totalExact++;
                }
            }
        }

        DEBUG_INFO_OUT("Phase2: total_pts=" + std::to_string(totalPoints)
                     + " exact=" + std::to_string(totalExact)
                     + " interpolated=" + std::to_string(totalInterpolated)
                     + " max_deviation=" + std::to_string(maxSdfDeviation)
                     + " | voxels=" + std::to_string(result.size())
                     + " pts_range=[" + std::to_string(minPts) + "," + std::to_string(maxPts) + "]"
                     + " | time=" + std::to_string(ms) + "ms");
    }

    return result;
}

void MicroCut::rebuildLeaves(
    IPWState& ipw,
    const CutClassification& cls,
    const std::unordered_map<openvdb::Coord, PointBuffer>& newBuffers,
    const ToolSweepSDF& sdf,
    const ToleranceConfig& config)
{
    auto t0 = std::chrono::high_resolution_clock::now();
    auto& microGrid = ipw.microGrid;
    const auto& xform = microGrid->transform();
    const double cullThreshold = config.user_t;

    // 收集受影响的 coords，按 leaf origin 分组
    std::unordered_map<openvdb::Coord, std::vector<openvdb::Coord>> leafGroups;
    auto groupCoord = [&](const openvdb::Coord& c) {
        openvdb::Coord origin(c.x() & ~7, c.y() & ~7, c.z() & ~7);
        leafGroups[origin].push_back(c);
    };
    for (const auto& c : cls.deleted) groupCoord(c);
    for (const auto& c : cls.cut) groupCoord(c);
    for (const auto& c : cls.newBoundary) groupCoord(c);

    // 构建 deleted/cut 查找集合
    std::unordered_set<int64_t> deletedSet, cutSet;
    auto coordKey = [](const openvdb::Coord& c) -> int64_t {
        return (int64_t(c.x()) << 40) | (int64_t(c.y() & 0xFFFFF) << 20) | int64_t(c.z() & 0xFFFFF);
    };
    for (const auto& c : cls.deleted) deletedSet.insert(coordKey(c));
    for (const auto& c : cls.cut) cutSet.insert(coordKey(c));

    int totalCulled = 0, totalSurvived = 0, totalNew = 0;
    using LeafNodeType = openvdb::points::PointDataGrid::TreeType::LeafNodeType;

    std::vector<Vec3f> allPositions;
    std::vector<Vec3f> allNormals;
    allPositions.reserve(leafGroups.size() * 100); // 预估
    allNormals.reserve(leafGroups.size() * 100);

    for (const auto& [leafOrigin, coords] : leafGroups) {
        auto* oldLeaf = microGrid->tree().probeLeaf(leafOrigin);

        if (oldLeaf) {
            auto posHandle = openvdb::points::AttributeHandle<Vec3f>::create(
                oldLeaf->constAttributeArray("P"));
            auto nrmHandle = openvdb::points::AttributeHandle<Vec3f>::create(
                oldLeaf->constAttributeArray("N"));

            for (auto voxIt = oldLeaf->cbeginValueOn(); voxIt; ++voxIt) {
                openvdb::Coord voxCoord = voxIt.getCoord();
                int64_t key = coordKey(voxCoord);

                if (deletedSet.count(key)) {
                    auto off = oldLeaf->coordToOffset(voxCoord);
                    auto end = oldLeaf->getValue(off);
                    decltype(end) start = (off == 0) ? decltype(end)(0) : oldLeaf->getValue(off-1);
                    totalCulled += (int)(end - start);
                } else {
                    bool isCut = cutSet.count(key);
                    for (auto ptIt = oldLeaf->beginIndexVoxel(voxCoord); ptIt; ++ptIt) {
                        Vec3f pos = posHandle->get(*ptIt);
                        Vec3f nrm = nrmHandle->get(*ptIt);
                        Vec3d wp = xform.indexToWorld(voxCoord.asVec3d() + Vec3d(pos));
                        
                        if (isCut && sdf.eval(wp) < cullThreshold) {
                            totalCulled++;
                            continue;
                        }
                        
                        allPositions.push_back(Vec3f(wp));
                        allNormals.push_back(nrm);
                        totalSurvived++;
                    }
                }
            }
        }

        // 加入新采样点
        for (const auto& c : coords) {
            auto it = newBuffers.find(c);
            if (it != newBuffers.end()) {
                for (size_t i = 0; i < it->second.positions.size(); ++i) {
                    allPositions.push_back(it->second.positions[i]);
                    allNormals.push_back(it->second.normals[i]);
                    totalNew++;
                }
            }
        }

        // 移除旧节点
        microGrid->tree().stealNode<LeafNodeType>(leafOrigin, 0, false);
    }

    // 批量构建新 Grid 并 Merge
    if (!allPositions.empty()) {
        auto tempGrid = openvdb::points::createPointDataGrid<
            openvdb::points::NullCodec, openvdb::points::PointDataGrid>(
            allPositions, xform);

        openvdb::points::appendAttribute<Vec3f>(tempGrid->tree(), "N");
        size_t idx = 0;
        for (auto leaf = tempGrid->tree().beginLeaf(); leaf; ++leaf) {
            openvdb::points::AttributeWriteHandle<Vec3f> handle(leaf->attributeArray("N"));
            for (auto it = leaf->beginIndexOn(); it; ++it) {
                handle.set(*it, allNormals[idx++]);
            }
        }

        // Merge 回主 grid
        microGrid->tree().merge(tempGrid->tree());
    }

    auto t1 = std::chrono::high_resolution_clock::now();

    DEBUG_SECTION(Phase3_Cull) {
        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        DEBUG_INFO_OUT("Phase3+4: culled=" + std::to_string(totalCulled)
                     + " survived=" + std::to_string(totalSurvived)
                     + " new=" + std::to_string(totalNew)
                     + " affected_leaves=" + std::to_string(leafGroups.size())
                     + " | time=" + std::to_string(ms) + "ms");
    }
}

} // namespace midgard
