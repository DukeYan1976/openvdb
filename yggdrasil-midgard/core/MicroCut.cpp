#include "core/MicroCut.h"
#include "core/IPWBuilder.h"
#include "core/LocalSurfaceEngine.h"
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/concurrent_unordered_map.h>
#include <openvdb/points/PointConversion.h>
#include <openvdb/points/PointCount.h>
#include <openvdb/tools/Interpolation.h>
#include <chrono>
#include <unordered_set>
#include <cmath>

namespace midgard {

struct SimpleCoordHash {
    size_t operator()(const openvdb::Coord& c) const {
        return (c.x() * 73856093) ^ (c.y() * 19349663) ^ (c.z() * 83492791);
    }
};

// ... (keep SimpleCoordHash) ...

std::unordered_map<openvdb::Coord, PointBuffer>
MicroCut::primeBilletBoundaries(
    const std::vector<VoxelTask>& tasks,
    const GeometryDef& billetDef,
    const ToleranceConfig& config)
{
    tbb::concurrent_unordered_map<openvdb::Coord, PointBuffer, SimpleCoordHash> concurrentBuffers;

    tbb::parallel_for(tbb::blocked_range<size_t>(0, tasks.size()),
        [&](const tbb::blocked_range<size_t>& range) {
            for (size_t i = range.begin(); i != range.end(); ++i) {
                const auto& task = tasks[i];
                if (task.classification != VoxelClass::NEW_BOUNDARY) continue;

                // 直接调用 IPW0 标准服务进行高精度采样
                PointBuffer buf = IPWBuilder::sampleBoundary(billetDef, task.aabb);
                if (!buf.positions.empty()) {
                    concurrentBuffers[task.origin] = std::move(buf);
                }
            }
        });

    return std::unordered_map<openvdb::Coord, PointBuffer>(concurrentBuffers.begin(), concurrentBuffers.end());
}

void MicroCut::quadtreeEval(
    double u0, double u1, double v0, double v1,
    const openvdb::BBoxd& voxelAABB,
    const ToolSweepSurface& surface,
    double chordalLimit,
    int depth,
    PointBuffer& output,
    const openvdb::FloatGrid::ConstAccessor* billetAcc,
    const openvdb::math::Transform* billetXform,
    const PointBuffer* existingData,
    double cullThreshold)
{
    // 子域中心点
    double uMid = (u0 + u1) * 0.5;
    double vMid = (v0 + v1) * 0.5;
    Vec3d pCenter = surface.eval(uMid, vMid);

    // 3D 拒止: 用9点近似子域AABB，与膨胀的体素做overlap检查
    // depth>=2确保子域已经足够小不会因离散采样误拒
    if (depth >= 2) {
        double uMid2 = (u0 + u1) * 0.5;
        double vMid2 = (v0 + v1) * 0.5;
        openvdb::BBoxd subBBox;
        subBBox.expand(pCenter);
        subBBox.expand(surface.eval(u0, v0));
        subBBox.expand(surface.eval(u1, v0));
        subBBox.expand(surface.eval(u0, v1));
        subBBox.expand(surface.eval(u1, v1));
        subBBox.expand(surface.eval(uMid2, v0));
        subBBox.expand(surface.eval(uMid2, v1));
        subBBox.expand(surface.eval(u0, vMid2));
        subBBox.expand(surface.eval(u1, vMid2));
        // 膨胀子域AABB以补偿离散采样的非保守性
        subBBox.expand(chordalLimit);
        if (!voxelAABB.hasOverlap(subBBox)) return;
    }

    auto validateAndOutput = [&](const Vec3d& p, const Vec3d& n) {
        if (existingData && !existingData->positions.empty()) {
            LocalSurfaceEngine engine(existingData->positions, existingData->normals, voxelAABB);
            double s_old = engine.eval(p);
            if (s_old > -cullThreshold) return; // Air or removed
        } else if (billetAcc && billetXform) {
            if (billetAcc->getValue(openvdb::Coord::round(billetXform->worldToIndex(p))) > cullThreshold) return;
        }
        output.positions.push_back(Vec3f(p));
        output.normals.push_back(Vec3f(n));
    };

    // 最大深度 → 强制输出（插值点）
    if (depth >= MAX_DEPTH) {
        Vec3d pmin = voxelAABB.min() - Vec3d(1e-6);
        Vec3d pmax = voxelAABB.max() + Vec3d(1e-6);
        if (openvdb::BBoxd(pmin, pmax).isInside(pCenter)) {
            Vec3d n = surface.normal(uMid, vMid);
            validateAndOutput(pCenter, n);
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

    // 弦高 ≤ 容差 → 尝试输出中心点
    if (maxChordal <= chordalLimit) {
        Vec3d pmin = voxelAABB.min() - Vec3d(1e-7);
        Vec3d pmax = voxelAABB.max() + Vec3d(1e-7);
        bool inside = openvdb::BBoxd(pmin, pmax).isInside(pCenter);
        
        if (inside) {
            Vec3d n = surface.normal(uMid, vMid);
            validateAndOutput(pCenter, n);
            return; // 成功在 Voxel 内生成点，可以返回
        }
        // ⚠️ 关键修复：如果弦高达标但中心点在 Voxel 外，
        // 说明当前参数块还太大，不能直接 return。
        // 只有当递归深度过深（无法进一步细分）时才放弃。
        if (depth >= MAX_DEPTH - 2) return;
    }

    // 继续细分为4个子域
    quadtreeEval(u0, uMid, v0, vMid, voxelAABB, surface, chordalLimit, depth+1, output, billetAcc, billetXform, existingData, cullThreshold);
    quadtreeEval(uMid, u1, v0, vMid, voxelAABB, surface, chordalLimit, depth+1, output, billetAcc, billetXform, existingData, cullThreshold);
    quadtreeEval(u0, uMid, vMid, v1, voxelAABB, surface, chordalLimit, depth+1, output, billetAcc, billetXform, existingData, cullThreshold);
    quadtreeEval(uMid, u1, vMid, v1, voxelAABB, surface, chordalLimit, depth+1, output, billetAcc, billetXform, existingData, cullThreshold);
}

std::unordered_map<openvdb::Coord, PointBuffer>
MicroCut::sampleNewSurface(
    const std::vector<VoxelTask>& tasks,
    const ToolSweepSurface& surface,
    const ToolSweptSDF& sdf,
    const ToleranceConfig& config,
    const IPWState& ipw)
{
    auto t0 = std::chrono::high_resolution_clock::now();

    auto billetGrid = ipw.macroGrid;

    std::unique_ptr<openvdb::FloatGrid::ConstAccessor> billetAccPtr;
    const openvdb::FloatGrid::ConstAccessor* billetAcc = nullptr;
    const openvdb::math::Transform* billetXform = nullptr;
    if (billetGrid) {
        billetAccPtr = std::make_unique<openvdb::FloatGrid::ConstAccessor>(billetGrid->getConstAccessor());
        billetAcc = billetAccPtr.get();
        billetXform = &billetGrid->transform();
    }

    // PHASE 1.5: 动态毛坯补全
    auto primedBuffers = primeBilletBoundaries(tasks, ipw.billetDef, config);

    std::vector<PointBuffer> taskBuffers(tasks.size());

    // 提前构建需要去查询 MicroGrid 的 leaf 分组
    // 或者每个任务独立探查 MicroGrid。MicroGrid 是并发可读的。
    const auto* microGridTree = ipw.microGrid ? &ipw.microGrid->tree() : nullptr;
    const auto* microGridXform = ipw.microGrid ? &ipw.microGrid->transform() : nullptr;

    tbb::parallel_for(tbb::blocked_range<size_t>(0, tasks.size()),
        [&](const tbb::blocked_range<size_t>& range) {
            for (size_t i = range.begin(); i < range.end(); ++i) {
                const auto& task = tasks[i];
                PointBuffer localExistingData;
                const PointBuffer* existingDataPtr = nullptr;

                if (task.classification == VoxelClass::NEW_BOUNDARY) {
                    // NEW_BOUNDARY 不使用 existingData 过滤:
                    // primeBilletBoundaries 数据仅用于 rebuildLeaves 的初始点集,
                    // 不应作为 LocalSurfaceEngine 的凸脊保护输入
                    existingDataPtr = nullptr;
                } else if (task.classification == VoxelClass::CUT && microGridTree) {
                    // 从 MicroGrid 中提取该 Voxel 的所有点
                    openvdb::Coord leafOrigin(task.origin.x() & ~7, task.origin.y() & ~7, task.origin.z() & ~7);
                    auto* leaf = microGridTree->probeConstLeaf(leafOrigin);
                    if (leaf && leaf->isValueOn(task.origin)) {
                        auto posHandle = openvdb::points::AttributeHandle<Vec3f>::create(leaf->constAttributeArray("P"));
                        auto nrmHandle = openvdb::points::AttributeHandle<Vec3f>::create(leaf->constAttributeArray("N"));
                        
                        for (auto ptIt = leaf->beginIndexVoxel(task.origin); ptIt; ++ptIt) {
                            Vec3f p = posHandle->get(*ptIt);
                            Vec3f n = nrmHandle->get(*ptIt);
                            Vec3d wp = microGridXform->indexToWorld(task.origin.asVec3d() + Vec3d(p));
                            localExistingData.positions.push_back(Vec3f(wp));
                            localExistingData.normals.push_back(n);
                        }
                        if (!localExistingData.positions.empty()) {
                            existingDataPtr = &localExistingData;
                        }
                    }
                }

                quadtreeEval(
                    task.u_min, task.u_max,
                    task.t_min, task.t_max,
                    task.aabb,
                    surface,
                    config.chordalLimit,
                    0,
                    taskBuffers[i],
                    // NEW_BOUNDARY体素不使用billetAcc过滤:
                    // macroGrid已被CSG修改,切削区内部SDF=background(正值),会误拒
                    (task.classification == VoxelClass::NEW_BOUNDARY) ? nullptr : billetAcc,
                    (task.classification == VoxelClass::NEW_BOUNDARY) ? nullptr : billetXform,
                    existingDataPtr,
                    config.user_t);

                // Fallback: 若四叉树搜索无结果，用SDF梯度投影从体素中心生成保底点
                if (taskBuffers[i].positions.empty()) {
                    Vec3d center = (task.aabb.min() + task.aabb.max()) * 0.5;
                    double s = sdf.eval(center);
                    Vec3d grad = sdf.gradient(center);
                    Vec3d proj = center - s * grad;
                    // Newton refinement
                    for (int iter = 0; iter < 3; ++iter) {
                        double s2 = sdf.eval(proj);
                        if (std::abs(s2) < 1e-10) break;
                        proj = proj - s2 * sdf.gradient(proj);
                    }
                    // 接受条件: SDF≈0 且在体素1V邻域内
                    openvdb::BBoxd nearBox = task.aabb;
                    nearBox.expand(config.voxelMacro);
                    if (std::abs(sdf.eval(proj)) < config.user_t && nearBox.isInside(proj)) {
                        taskBuffers[i].positions.push_back(Vec3f(proj));
                        taskBuffers[i].normals.push_back(Vec3f(sdf.gradient(proj)));
                    }
                }
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
    const ToolSweptSDF& sdf,
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
