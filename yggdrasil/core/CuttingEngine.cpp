#include "core/CuttingEngine.h"
#include "core/SurfelGenerator.h"
#include <openvdb/tools/Prune.h>
#include <openvdb/tools/Composite.h>
#include <openvdb/tools/Interpolation.h>
#include <openvdb/tools/LevelSetMeasure.h>
#include <openvdb/points/PointCount.h>
#include <openvdb/points/PointAttribute.h>
#include <openvdb/points/PointConversion.h>
#include <openvdb/tree/LeafManager.h>
#include <tbb/parallel_for.h>
#include <tbb/enumerable_thread_specific.h>
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <vector>
#include <unordered_map>
#include <chrono>
#include <ctime>
#include <string>

namespace ygg {

// ─── helper: current local timestamp as [YYYY/MM/DD HH:MM:SS.mmm] ───
static std::string ts() {
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm* tm = std::localtime(&t);
    char buf[32];
    std::snprintf(buf, sizeof(buf), "[%04d/%02d/%02d %02d:%02d:%02d.%03d]",
        tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
        tm->tm_hour, tm->tm_min, tm->tm_sec, static_cast<int>(ms.count()));
    return buf;
}

// ═══════════════════════════════════════════════════════════════════════
// 单轨切削策略评估代码（4种方案，精度一致，比较效率和内存）
// 选择方式：修改 CUT_STRATEGY 宏 (A / B / C / D)
// A: 暴力包围盒遍历（基线）
// B: 仅遍历活跃体素 + 包围盒过滤（零额外内存，串行）
// C: 光栅化刀具SDF为Grid + csgDifference（OpenVDB原生优化，需临时Grid）
// D: 方案B的TBB并行版本（LeafManager + parallel_for）
// ═══════════════════════════════════════════════════════════════════════
#ifndef CUT_STRATEGY
#define CUT_STRATEGY D  // 默认使用方案D(并行B)
#endif

// 方案A: 暴力包围盒三重循环（基线）
static void cutSingleTrack_A(BilletModel& billet, const ToolSweepSDF& toolSDF) {
    auto& grid = billet.sdfGrid;
    auto& xform = grid->transform();
    double voxelSize = xform.voxelSize()[0];
    float bandWidth = 3.0f * static_cast<float>(voxelSize);

    auto bbox = toolSDF.getBoundingBox();
    auto minIdx = xform.worldToIndexCellCentered(bbox.min());
    auto maxIdx = xform.worldToIndexCellCentered(bbox.max());

    auto accessor = grid->getAccessor();
    openvdb::Coord ijk;
    for (ijk[0] = minIdx[0]; ijk[0] <= maxIdx[0]; ++ijk[0]) {
        for (ijk[1] = minIdx[1]; ijk[1] <= maxIdx[1]; ++ijk[1]) {
            for (ijk[2] = minIdx[2]; ijk[2] <= maxIdx[2]; ++ijk[2]) {
                Vec3d worldPos = xform.indexToWorld(ijk);
                double toolDist = toolSDF.eval(worldPos);
                if (toolDist < bandWidth) {
                    float billetVal = accessor.getValue(ijk);
                    float newVal = std::max(billetVal, static_cast<float>(-toolDist));
                    if (newVal != billetVal)
                        accessor.setValue(ijk, newVal);
                }
            }
        }
    }
    openvdb::tools::pruneLevelSet(grid->tree());
}

// 方案B: 仅遍历已有活跃体素（窄带），包围盒过滤跳过无关体素
// 优势：零额外内存，只处理有数据的体素
static void cutSingleTrack_B(BilletModel& billet, const ToolSweepSDF& toolSDF) {
    auto& grid = billet.sdfGrid;
    auto& xform = grid->transform();
    double voxelSize = xform.voxelSize()[0];
    float bandWidth = 3.0f * static_cast<float>(voxelSize);

    auto bbox = toolSDF.getBoundingBox();
    auto minIdx = xform.worldToIndexCellCentered(bbox.min());
    auto maxIdx = xform.worldToIndexCellCentered(bbox.max());

    // 先激活刀具BBox内的 inactive 负值体素
    {
        auto acc = grid->getAccessor();
        openvdb::Coord ijk;
        for (ijk[0] = minIdx[0]; ijk[0] <= maxIdx[0]; ++ijk[0])
            for (ijk[1] = minIdx[1]; ijk[1] <= maxIdx[1]; ++ijk[1])
                for (ijk[2] = minIdx[2]; ijk[2] <= maxIdx[2]; ++ijk[2]) {
                    float val = acc.getValue(ijk);
                    if (val < 0 && !acc.isValueOn(ijk))
                        acc.setValueOn(ijk, val);
                }
    }

    // 遍历活跃体素
    for (auto iter = grid->beginValueOn(); iter; ++iter) {
        auto coord = iter.getCoord();
        if (coord.x() < minIdx.x() || coord.x() > maxIdx.x() ||
            coord.y() < minIdx.y() || coord.y() > maxIdx.y() ||
            coord.z() < minIdx.z() || coord.z() > maxIdx.z())
            continue;

        Vec3d worldPos = xform.indexToWorld(coord);
        double toolDist = toolSDF.eval(worldPos);
        if (toolDist < bandWidth) {
            float billetVal = iter.getValue();
            float newVal = std::max(billetVal, static_cast<float>(-toolDist));
            if (newVal != billetVal)
                iter.setValue(newVal);
        }
    }
    openvdb::tools::pruneLevelSet(grid->tree());
}

// 方案B_parallel: 方案B的TBB并行版本（LeafManager + parallel_for）
#include <openvdb/tree/LeafManager.h>
#include <tbb/parallel_for.h>

static void cutSingleTrack_B_parallel(BilletModel& billet, const ToolSweepSDF& toolSDF) {
    auto& grid = billet.sdfGrid;
    auto& xform = grid->transform();
    double voxelSize = xform.voxelSize()[0];
    float bandWidth = 3.0f * static_cast<float>(voxelSize);

    auto bbox = toolSDF.getBoundingBox();
    auto minIdx = xform.worldToIndexCellCentered(bbox.min());
    auto maxIdx = xform.worldToIndexCellCentered(bbox.max());

    // 关键：先将刀具BBox内的 inactive 负值区域激活为活跃体素
    // （内部 tile 需要被展开为单独体素才能被窄带更新）
    grid->tree().voxelizeActiveTiles();
    // fill 刀具 BBox 内的区域为活跃（保留原值）
    openvdb::CoordBBox idxBBox(minIdx, maxIdx);
    auto acc = grid->getAccessor();
    openvdb::Coord ijk;
    for (ijk[0] = minIdx[0]; ijk[0] <= maxIdx[0]; ++ijk[0]) {
        for (ijk[1] = minIdx[1]; ijk[1] <= maxIdx[1]; ++ijk[1]) {
            for (ijk[2] = minIdx[2]; ijk[2] <= maxIdx[2]; ++ijk[2]) {
                float val = acc.getValue(ijk);
                if (val < 0 && !acc.isValueOn(ijk)) {
                    acc.setValueOn(ijk, val);
                }
            }
        }
    }

    // 并行遍历所有活跃体素
    using TreeT = openvdb::FloatGrid::TreeType;
    openvdb::tree::LeafManager<TreeT> leafMgr(grid->tree());

    tbb::parallel_for(leafMgr.leafRange(),
        [&](const openvdb::tree::LeafManager<TreeT>::LeafRange& range) {
            for (auto leafIter = range.begin(); leafIter; ++leafIter) {
                auto& leaf = *leafIter;
                auto leafOrigin = leaf.origin();
                if (leafOrigin.x() + 8 < minIdx.x() || leafOrigin.x() > maxIdx.x() ||
                    leafOrigin.y() + 8 < minIdx.y() || leafOrigin.y() > maxIdx.y() ||
                    leafOrigin.z() + 8 < minIdx.z() || leafOrigin.z() > maxIdx.z())
                    continue;

                for (auto it = leaf.beginValueOn(); it; ++it) {
                    auto coord = it.getCoord();
                    if (coord.x() < minIdx.x() || coord.x() > maxIdx.x() ||
                        coord.y() < minIdx.y() || coord.y() > maxIdx.y() ||
                        coord.z() < minIdx.z() || coord.z() > maxIdx.z())
                        continue;

                    Vec3d worldPos = xform.indexToWorld(coord);
                    double toolDist = toolSDF.eval(worldPos);
                    if (toolDist < bandWidth) {
                        float billetVal = it.getValue();
                        float newVal = std::max(billetVal, static_cast<float>(-toolDist));
                        if (newVal != billetVal)
                            leaf.setValueOn(it.pos(), newVal);
                    }
                }
            }
        });
    openvdb::tools::pruneLevelSet(grid->tree());
}

// 方案C: 光栅化刀具SDF为临时Grid + csgDifferenceSDF
// 优势：OpenVDB内部拓扑级优化；劣势：临时Grid内存开销
static void cutSingleTrack_C(BilletModel& billet, const ToolSweepSDF& toolSDF) {
    auto& grid = billet.sdfGrid;
    auto& xform = grid->transform();
    double voxelSize = xform.voxelSize()[0];
    float bandWidth = 3.0f * static_cast<float>(voxelSize);

    auto bbox = toolSDF.getBoundingBox();
    auto minIdx = xform.worldToIndexCellCentered(bbox.min());
    auto maxIdx = xform.worldToIndexCellCentered(bbox.max());

    // 光栅化刀具SDF为临时Grid
    auto toolGrid = openvdb::FloatGrid::create(bandWidth);
    toolGrid->setTransform(grid->transformPtr());
    toolGrid->setGridClass(openvdb::GRID_LEVEL_SET);

    auto accessor = toolGrid->getAccessor();
    openvdb::Coord ijk;
    for (ijk[0] = minIdx[0]; ijk[0] <= maxIdx[0]; ++ijk[0]) {
        for (ijk[1] = minIdx[1]; ijk[1] <= maxIdx[1]; ++ijk[1]) {
            for (ijk[2] = minIdx[2]; ijk[2] <= maxIdx[2]; ++ijk[2]) {
                Vec3d worldPos = xform.indexToWorld(ijk);
                float dist = static_cast<float>(toolSDF.eval(worldPos));
                if (std::abs(dist) < bandWidth)
                    accessor.setValue(ijk, dist);
            }
        }
    }

    // OpenVDB 原生布尔差集
    openvdb::tools::csgDifference(*grid, *toolGrid);
}

static void cutSingleTrack(BilletModel& billet, const ToolSweepSDF& toolSDF, CuttingEngine::Strategy strat) {
    switch (strat) {
        case CuttingEngine::STRAT_A: cutSingleTrack_A(billet, toolSDF); break;
        case CuttingEngine::STRAT_C: cutSingleTrack_C(billet, toolSDF); break;
        case CuttingEngine::STRAT_D: cutSingleTrack_B_parallel(billet, toolSDF); break;
        default: cutSingleTrack_B(billet, toolSDF); break;
    }
}

// Helper: attach normal and active attributes to a PointDataGrid
static void attachAttributes(openvdb::points::PointDataGrid::Ptr& grid,
                             const std::vector<openvdb::Vec3f>& normals) {
    auto& tree = grid->tree();

    // Use tree-level API to append attributes uniformly across all leaves
    openvdb::points::appendAttribute<openvdb::Vec3f>(tree, "normal");
    openvdb::points::appendAttribute<uint8_t>(tree, "active");

    // Fill values per-leaf
    size_t idx = 0;
    for (auto leaf = tree.beginLeaf(); leaf; ++leaf) {
        auto whN = openvdb::points::AttributeWriteHandle<openvdb::Vec3f>::create(
            leaf->attributeArray("normal"));
        auto whA = openvdb::points::AttributeWriteHandle<uint8_t>::create(
            leaf->attributeArray("active"));
        const openvdb::Index count = leaf->pointCount();
        for (openvdb::Index i = 0; i < count; ++i, ++idx) {
            whN->set(i, idx < normals.size() ? normals[idx] : openvdb::Vec3f(0));
            whA->set(i, 1);
        }
    }
}

// Helper: extract active points from an existing leaf
static void extractActivePointsFromLeaf(
    const openvdb::points::PointDataTree::LeafNodeType* leaf,
    const openvdb::math::Transform& xform,
    std::vector<openvdb::Vec3R>& outPos,
    std::vector<openvdb::Vec3f>& outNorm) {

    auto lo = leaf->origin();
    auto& as = leaf->attributeSet();
    auto* posArr = as.get("P");
    auto* normArr = as.get("normal");
    auto* actArr = as.get("active");
    if (!posArr) return;

    auto ph = openvdb::points::AttributeHandle<openvdb::Vec3f>::create(*posArr);
    auto nh = normArr ? openvdb::points::AttributeHandle<openvdb::Vec3f>::create(*normArr) : nullptr;
    auto ah = actArr ? openvdb::points::AttributeHandle<uint8_t>::create(*actArr) : nullptr;

    for (openvdb::Index vIdx = 0; vIdx < 512; ++vIdx) {
        openvdb::Index end = static_cast<openvdb::Index>(leaf->getValue(vIdx));
        openvdb::Index start = (vIdx==0) ? openvdb::Index(0) :
            static_cast<openvdb::Index>(leaf->getValue(vIdx-1));
        if (start==end) continue;
        openvdb::Coord vc = lo + openvdb::Coord((vIdx>>6)&7,(vIdx>>3)&7,vIdx&7);
        for (openvdb::Index i = start; i < end; ++i) {
            if (ah && ah->get(i)==0) continue;
            auto p = ph->get(i);
            outPos.push_back(xform.indexToWorld(
                Vec3d(vc.x()+p.x(), vc.y()+p.y(), vc.z()+p.z())));
            outNorm.push_back(nh ? nh->get(i) : openvdb::Vec3f(0,0,1));
        }
    }
}

// ─── Phase 2 helper: extract active points from entire PointDataGrid ───
static void extractActivePointsFromGrid(
    const openvdb::points::PointDataGrid::Ptr& grid,
    const openvdb::math::Transform& xform,
    std::vector<openvdb::Vec3R>& outPos,
    std::vector<openvdb::Vec3f>& outNorm) {
    if (!grid) return;
    for (auto leaf = grid->tree().cbeginLeaf(); leaf; ++leaf) {
        extractActivePointsFromLeaf(&*leaf, xform, outPos, outNorm);
    }
}

// ─── Phase 2 helper: Leaf-Local Splice incremental injection ───
// Only rebuilds leaves that receive new points. Untouched leaves = zero cost.
// Complexity: O(dirty_leaves × points_per_leaf) instead of O(total_points).
static void injectSurfels(BilletModel& billet,
                          const std::vector<openvdb::Vec3R>& positions,
                          const std::vector<openvdb::Vec3f>& normals,
                          const openvdb::math::Transform& xform) {
    if (positions.empty()) return;
    openvdb::points::TypedAttributeArray<openvdb::Vec3f>::registerType();
    openvdb::points::TypedAttributeArray<uint8_t>::registerType();

    printf("%s   [inject] positions=%zu existing_grid=%s\n",
        ts().c_str(), positions.size(), billet.microGrid ? "yes" : "no");
    fflush(stdout);

    // First injection: build from scratch
    if (!billet.microGrid) {
        auto grid = openvdb::points::createPointDataGrid<
            openvdb::points::NullCodec, openvdb::points::PointDataGrid>(positions, xform);
        grid->setName("micro_surfels");
        attachAttributes(grid, normals);
        billet.microGrid = grid;
        printf("%s   [inject] first done, leaves=%zu\n",
            ts().c_str(), grid->tree().leafCount()); fflush(stdout);
        return;
    }

    // ── Step 1: Bucket new points by leaf origin ──
    struct LeafBucket {
        openvdb::Coord origin;
        std::vector<openvdb::Vec3R> pos;
        std::vector<openvdb::Vec3f> norm;
    };
    std::unordered_map<int64_t, size_t> originToIdx;
    std::vector<LeafBucket> buckets;

    for (size_t i = 0; i < positions.size(); ++i) {
        auto idx = xform.worldToIndexCellCentered(positions[i]);
        openvdb::Coord lo((idx.x() >> 3) << 3, (idx.y() >> 3) << 3, (idx.z() >> 3) << 3);
        int64_t key = (int64_t(lo.x()) << 20) ^ (int64_t(lo.y()) << 10) ^ int64_t(lo.z());
        auto it = originToIdx.find(key);
        if (it == originToIdx.end()) {
            originToIdx[key] = buckets.size();
            buckets.push_back({lo, {positions[i]}, {normals[i]}});
        } else {
            buckets[it->second].pos.push_back(positions[i]);
            buckets[it->second].norm.push_back(normals[i]);
        }
    }

    printf("%s   [inject] splice: %zu pts → %zu dirty leaves / %zu total\n",
        ts().c_str(), positions.size(), buckets.size(),
        billet.microGrid->tree().leafCount()); fflush(stdout);

    // ── Step 2: Per dirty leaf: extract existing + merge new → build mini grid ──
    struct LeafResult {
        openvdb::Coord origin;
        std::vector<openvdb::Vec3R> allPos;
        std::vector<openvdb::Vec3f> allNorm;
    };
    std::vector<LeafResult> results(buckets.size());

    tbb::parallel_for(tbb::blocked_range<size_t>(0, buckets.size()),
        [&](const tbb::blocked_range<size_t>& r) {
            for (size_t bi = r.begin(); bi != r.end(); ++bi) {
                auto& b = buckets[bi];
                auto& res = results[bi];
                res.origin = b.origin;

                // Extract existing active points from this leaf
                auto* existingLeaf = billet.microGrid->tree().probeConstLeaf(b.origin);
                if (existingLeaf) {
                    extractActivePointsFromLeaf(existingLeaf, xform, res.allPos, res.allNorm);
                }
                // Append new
                res.allPos.insert(res.allPos.end(), b.pos.begin(), b.pos.end());
                res.allNorm.insert(res.allNorm.end(), b.norm.begin(), b.norm.end());
            }
        });

    // ── Step 3: Collect all points from dirty leaves, build a single mini grid ──
    // (createPointDataGrid handles bin-sorting into correct leaves)
    std::vector<openvdb::Vec3R> dirtyPos;
    std::vector<openvdb::Vec3f> dirtyNorm;
    size_t totalDirtyPts = 0;
    for (auto& res : results) totalDirtyPts += res.allPos.size();
    dirtyPos.reserve(totalDirtyPts);
    dirtyNorm.reserve(totalDirtyPts);
    for (auto& res : results) {
        dirtyPos.insert(dirtyPos.end(), res.allPos.begin(), res.allPos.end());
        dirtyNorm.insert(dirtyNorm.end(), res.allNorm.begin(), res.allNorm.end());
    }

    auto dirtyGrid = openvdb::points::createPointDataGrid<
        openvdb::points::NullCodec, openvdb::points::PointDataGrid>(dirtyPos, xform);
    attachAttributes(dirtyGrid, dirtyNorm);

    // ── Step 4: Splice dirty leaves into main tree ──
    auto& mainTree = billet.microGrid->tree();
    for (auto leaf = dirtyGrid->tree().beginLeaf(); leaf; ++leaf) {
        // addLeaf replaces existing leaf at same origin or inserts new
        using LeafT = openvdb::points::PointDataTree::LeafNodeType;
        mainTree.addLeaf(new LeafT(*leaf));
    }

    // Ensure descriptor consistency: all leaves must share same descriptor
    // After addLeaf, new leaves may have a different descriptor. Fix by
    // re-appending attributes (no-op for leaves that already have them)
    auto firstLeaf = mainTree.cbeginLeaf();
    if (firstLeaf) {
        auto& desc = firstLeaf->attributeSet().descriptor();
        bool hasNormal = desc.find("normal") != openvdb::points::AttributeSet::INVALID_POS;
        bool hasActive = desc.find("active") != openvdb::points::AttributeSet::INVALID_POS;
        if (!hasNormal) openvdb::points::appendAttribute<openvdb::Vec3f>(mainTree, "normal");
        if (!hasActive) openvdb::points::appendAttribute<uint8_t>(mainTree, "active");
    }

    printf("%s   [inject] splice done, dirty_pts=%zu total_leaves=%zu\n",
        ts().c_str(), totalDirtyPts, mainTree.leafCount()); fflush(stdout);
}

// 双轨切削：工业级 4-phase（增量注入 + 并行裁剪 + 分块采样）
static void cutDualTrack(BilletModel& billet, const ToolSweepSDF& toolSDF) {
    using Clock = std::chrono::high_resolution_clock;
    auto t0_total = Clock::now();

    using TreeT = openvdb::FloatGrid::TreeType;
    auto& sdfGrid = billet.sdfGrid;
    auto& xform = sdfGrid->transform();
    double D_v = xform.voxelSize()[0];
    float bandWidth = 3.0f * static_cast<float>(D_v);

    auto bbox = toolSDF.getBoundingBox();
    //将工具包围盒在索引空间中向外扩张一个完整的窄带宽度，确保 CSG 操作不会遗漏或破坏边界处的有效 SDF 数据。
    auto minIdx = xform.worldToIndexCellCentered(bbox.min()) - openvdb::Coord(3);
    auto maxIdx = xform.worldToIndexCellCentered(bbox.max()) + openvdb::Coord(3);

    printf("%s [DualTrack] D_v=%.4f d_v=%.4f N=%d BBox=[%d,%d,%d]..[%d,%d,%d]\n",
        ts().c_str(), D_v, billet.config.d_v, billet.config.N,
        minIdx.x(), minIdx.y(), minIdx.z(), maxIdx.x(), maxIdx.y(), maxIdx.z());
    fflush(stdout);

    // ═══ Phase 1: SDF CSG diff (TBB parallel) + dirty region ═══
    auto t0_p1 = Clock::now();

    // Activate inactive negative voxels in tool BBox (serial for thread safety)
    auto t0_p1a = Clock::now();
    {
        auto acc = sdfGrid->getAccessor();
        openvdb::Coord ijk;
        for (ijk[0]=minIdx[0]; ijk[0]<=maxIdx[0]; ++ijk[0])
            for (ijk[1]=minIdx[1]; ijk[1]<=maxIdx[1]; ++ijk[1])
                for (ijk[2]=minIdx[2]; ijk[2]<=maxIdx[2]; ++ijk[2]) {
                    float v = acc.getValue(ijk);
                    if (v < 0 && !acc.isValueOn(ijk)) acc.setValueOn(ijk, v);
                }
    }
    auto t1_p1a = Clock::now();


    //备注duke
    // 被激活的swepttool bbbox中的voxel， 意味着原来未被切削， 所有需要构建其中的高精度切削表面。 
    // 这个voxel在macrogrid中， 按照D_v Voxelsize，与swepttool做csgdifference计算， 可以形成D_v粒度的切削表面窄带， 这个窄带可以做粗略显示（碰撞检测精度）。
    // 但在microgrid中， 需要按照d_v精度，来构建高精度的切削表面（带法向的点集），存储在microgrid的voxel中。 
    // macrogrid和microgrid时刻保持相同的node结构以及voxel状态，唯一的不同是microgrid负责在切削后的边界的voxel中，存储高精度点集切削面。 
    // 这个切削面构建后， 会驻留在内存中，下一次该voxel中的切削需要使用。 
    //问题：  macrogrid和microgrid有一个node状态同步的过程？ 这个有方法可以瞬时同步吗，microgri是跟随的。 以macrogrid为主。 

    //phase1 ： 1   Activate inactive negative voxels in tool BBox
    //case1_1: 如果voxel是inactive negative的，说明没有被切削过，先激活。 
    //case1_2: 如果voxel是active的，说明有毛坯的表面或者切削表面（意味着有窄带SDF， macrogrid中是D_v粒度，而microgrid中有对应的精确点集）。
/*
    phase 2:  Dual Cut
            
    case1_1:   swepttool与完整voxel的csgdifference问题。
    case1_2:   swepttool与voxel+volumn with pointdata surface的csgdifference问题。 
               
           output：
               macrogrid： 直接与swepttool csgdifference后的新的voxel with narrowband；
               microgrid： 计算出的新的volumn with pointdata surface

               这一步是关键， 如何在合理管控内存的情况下高效高精完成？  挑战是： 在设定的精度要求下，在规定的cycletime中完成，否则会引起整个pipeline的性能问题。 
                1 构建一个临时的高精度openvdb，与swepttool执行csddifference？ 还是利用microgrid中的 volumn with pointdata surface，来设计算法来完成？ 
                2 考虑到脏区的voxel随着swepttool的规模（bbox），数目也增长， 是所有的脏区voxel一起计算，还是单个计算（可并行或者GPU）？

    这一步要注意： 确定哪些voxel被影响并记录， 后面的流程需要用到
                1 voxel可能被完全切除；  macrogrid的voxel被设为inactive nagative，  microgrid中的pointdata被删掉，对应的voxel被设为inactive nagative
                2 大概率被部分切除，原表面被完全抹去，形成新的表面；原表面被部分抹去，形成新的表面； 
                  macrogrid中形成新的narrowband，  microgrid中，形成新的pointdata； 

                问题：在视图渲染中，意味着有被从视图中删除的部分，也有新增加的部分， 如何记录并处理？ 
                
*/
          
    //phase 3: 对脏区的voxel做重整，确保数据一直和精炼，并异步构建渲染数据（ 低resolution情况下，显示macrogrid的边界， 高resolution情况下，显示micro的曲面）



    // TBB parallel SDF update + dirty voxel collection
    auto t0_p1b = Clock::now();
    openvdb::tree::LeafManager<TreeT> leafMgr(sdfGrid->tree());
    using DirtyVec = std::vector<openvdb::Coord>;
    tbb::enumerable_thread_specific<DirtyVec> tlsDirty;

    //并行遍历 sdfGrid 所有 active leaf nodes:
    tbb::parallel_for(leafMgr.leafRange(),
        [&](const openvdb::tree::LeafManager<TreeT>::LeafRange& range) {
            auto& localDirty = tlsDirty.local();
            //所有的leafnode
            for (auto leafIt = range.begin(); leafIt; ++leafIt) {
                auto& leaf = *leafIt;
                auto lo = leaf.origin();
                if (lo.x()+8<minIdx.x() || lo.x()>maxIdx.x() ||
                    lo.y()+8<minIdx.y() || lo.y()>maxIdx.y() ||
                    lo.z()+8<minIdx.z() || lo.z()>maxIdx.z()) continue;// 跳过不在 tool BBox 范围的 leaf
                
                //对于在toolbbox范围内部的leaf（而且是valueon，被激活的node），
                for (auto it = leaf.beginValueOn(); it; ++it) {
                    auto coord = it.getCoord();
                    if (coord.x()<minIdx.x() || coord.x()>maxIdx.x() ||
                        coord.y()<minIdx.y() || coord.y()>maxIdx.y() ||
                        coord.z()<minIdx.z() || coord.z()>maxIdx.z()) continue;

                    Vec3d wp = xform.indexToWorld(coord);
                    double toolDist = toolSDF.eval(wp);
                    if (toolDist < bandWidth) {
                        float oldVal = it.getValue();
                        float newVal = std::max(oldVal, static_cast<float>(-toolDist));
                        if (newVal != oldVal) {
                            it.setValue(newVal);
                        }
                        // Dirty = tool surface passes through this voxel:
                        // toolDist in [-bandWidth, D_v] and voxel is in narrowband
                        if (toolDist < D_v && std::abs(newVal) < bandWidth)
                            localDirty.push_back(coord);
                    }
                }
            }
        });
    auto t1_p1b = Clock::now();

    // Merge + dedup dirty voxels
    auto t0_p1c = Clock::now();
    std::vector<openvdb::Coord> dirtyVoxels;
    for (auto& v : tlsDirty) {
        for (auto& coord : v) {
            billet.dirtyMask->getAccessor().setValueOn(coord);
            dirtyVoxels.push_back(coord);
        }
    }
    auto t1_p1c = Clock::now();

    double ms_p1a = std::chrono::duration<double, std::milli>(t1_p1a - t0_p1a).count();
    double ms_p1b = std::chrono::duration<double, std::milli>(t1_p1b - t0_p1b).count();
    double ms_p1c = std::chrono::duration<double, std::milli>(t1_p1c - t0_p1c).count();

    printf("%s [DualTrack] Phase1: activate=%.1fms tbb_sdf=%.1fms dirty_merge=%.1fms dirty_count=%zu\n",
        ts().c_str(), ms_p1a, ms_p1b, ms_p1c, dirtyVoxels.size());
    fflush(stdout);

    // ═══ Phase 2: Chunked surfel generation + single injection ═══
    auto t0_p2 = Clock::now();
    size_t totalSurfels = 0;
    size_t totalChunks = 0;
    if (!dirtyVoxels.empty()) {
        const int N = billet.config.N;
        const double d_v = billet.config.d_v;
        const size_t CHUNK = 256;

        std::vector<openvdb::Vec3R> allPos;
        std::vector<openvdb::Vec3f> allNorm;
        allPos.reserve(dirtyVoxels.size() * N * N);
        allNorm.reserve(dirtyVoxels.size() * N * N);

        auto t0_p2a = Clock::now();
        for (size_t start = 0; start < dirtyVoxels.size(); start += CHUNK) {
            size_t end = std::min(start + CHUNK, dirtyVoxels.size());
            std::vector<openvdb::Coord> chunk(dirtyVoxels.begin() + start,
                                              dirtyVoxels.begin() + end);
            auto batch = SurfelGenerator::sampleToolSurface(toolSDF, chunk, xform, d_v, N, &billet.geometry);
            allPos.insert(allPos.end(), batch.positions.begin(), batch.positions.end());
            allNorm.insert(allNorm.end(), batch.normals.begin(), batch.normals.end());
            totalChunks++;
        }
        auto t1_p2a = Clock::now();
        totalSurfels = allPos.size();

        auto t0_p2b = Clock::now();
        if (!allPos.empty()) {
            injectSurfels(billet, allPos, allNorm, xform);
        }
        auto t1_p2b = Clock::now();

        double ms_p2a = std::chrono::duration<double, std::milli>(t1_p2a - t0_p2a).count();
        double ms_p2b = std::chrono::duration<double, std::milli>(t1_p2b - t0_p2b).count();
        printf("%s [DualTrack] Phase2: surfel_gen=%.1fms inject=%.1fms surfels=%zu chunks=%zu\n",
            ts().c_str(), ms_p2a, ms_p2b, totalSurfels, totalChunks);
        fflush(stdout);
    }
    auto t1_p2 = Clock::now();
    double ms_p2 = std::chrono::duration<double, std::milli>(t1_p2 - t0_p2).count();

    // ═══ Phase 3: Parallel surfel clipping (BBox-local leaves only) ═══
    auto t0_p3 = Clock::now();
    int affectedLeaves = 0;
    int skippedLeaves = 0;
    if (billet.microGrid) {
        openvdb::Coord leafMin(
            (minIdx.x() >> 3) << 3,
            (minIdx.y() >> 3) << 3,
            (minIdx.z() >> 3) << 3
        );
        openvdb::Coord leafMax(
            (maxIdx.x() >> 3) << 3,
            (maxIdx.y() >> 3) << 3,
            (maxIdx.z() >> 3) << 3
        );

        int leafCountX = (leafMax.x() - leafMin.x()) / 8 + 1;
        int leafCountY = (leafMax.y() - leafMin.y()) / 8 + 1;
        int leafCountZ = (leafMax.z() - leafMin.z()) / 8 + 1;
        int totalLeaves = leafCountX * leafCountY * leafCountZ;

        tbb::enumerable_thread_specific<std::pair<int,int>> tlsLeafStats;

        tbb::parallel_for(tbb::blocked_range<int>(0, totalLeaves),
            [&](const tbb::blocked_range<int>& r) {
                auto& stats = tlsLeafStats.local();
                for (int lin = r.begin(); lin != r.end(); ++lin) {
                    int ix = lin / (leafCountY * leafCountZ);
                    int iy = (lin / leafCountZ) % leafCountY;
                    int iz = lin % leafCountZ;
                    openvdb::Coord leafOrigin(
                        leafMin.x() + ix * 8,
                        leafMin.y() + iy * 8,
                        leafMin.z() + iz * 8
                    );
                    auto* leaf = billet.microGrid->tree().probeLeaf(leafOrigin);
                    if (!leaf) continue;

                    auto lo = leaf->origin();
                    Vec3d lw = xform.indexToWorld(lo);
                    Vec3d lmax = lw + Vec3d(8.0 * D_v);
                    if (lw.x()>bbox.max().x()+D_v || lmax.x()<bbox.min().x()-D_v ||
                        lw.y()>bbox.max().y()+D_v || lmax.y()<bbox.min().y()-D_v ||
                        lw.z()>bbox.max().z()+D_v || lmax.z()<bbox.min().z()-D_v) {
                        stats.second++;
                        continue;
                    }
                    stats.first++;

                    auto as = leaf->stealAttributeSet();
                    auto* posArr = as->get("P");
                    auto* actArr = as->get("active");
                    if (!posArr || !actArr) { leaf->replaceAttributeSet(as.release(), true); continue; }

                    auto ph = openvdb::points::AttributeHandle<openvdb::Vec3f>::create(*posArr);
                    auto wh = openvdb::points::AttributeWriteHandle<uint8_t>::create(*actArr);

                    for (openvdb::Index vIdx = 0; vIdx < 512; ++vIdx) {
                        openvdb::Index endI = static_cast<openvdb::Index>(leaf->getValue(vIdx));
                        openvdb::Index startI = (vIdx==0) ? openvdb::Index(0) :
                            static_cast<openvdb::Index>(leaf->getValue(vIdx-1));
                        if (startI == endI) continue;
                        openvdb::Coord vc = lo + openvdb::Coord((vIdx>>6)&7,(vIdx>>3)&7,vIdx&7);
                        for (openvdb::Index i = startI; i < endI; ++i) {
                            if (wh->get(i) == 0) continue;
                            auto p = ph->get(i);
                            Vec3d wp = xform.indexToWorld(
                                Vec3d(vc.x()+p.x(), vc.y()+p.y(), vc.z()+p.z()));
                            if (toolSDF.eval(wp) <= 0.0) wh->set(i, 0);
                        }
                    }
                    leaf->replaceAttributeSet(as.release(), true);
                }
            });

        for (auto& s : tlsLeafStats) {
            affectedLeaves += s.first;
            skippedLeaves += s.second;
        }
    }
    auto t1_p3 = Clock::now();
    double ms_p3 = std::chrono::duration<double, std::milli>(t1_p3 - t0_p3).count();
    printf("%s [DualTrack] Phase3: clip=%.1fms affected=%d skipped=%d\n",
        ts().c_str(), ms_p3, affectedLeaves, skippedLeaves);
    fflush(stdout);

    // ═══ Phase 4: Prune + clear ═══
    auto t0_p4 = Clock::now();
    openvdb::tools::pruneLevelSet(sdfGrid->tree());
    billet.dirtyMask->clear();
    auto t1_p4 = Clock::now();
    double ms_p4 = std::chrono::duration<double, std::milli>(t1_p4 - t0_p4).count();

    auto t1_total = Clock::now();
    double ms_total = std::chrono::duration<double, std::milli>(t1_total - t0_total).count();

    printf("%s [DualTrack] Phase4: prune=%.1fms | TOTAL=%.1fms\n", ts().c_str(), ms_p4, ms_total);
    fflush(stdout);
}

void CuttingEngine::cut(BilletModel& billet, const ToolSweepSDF& toolSDF) {
    if (billet.isSingleTrack()) {
        cutSingleTrack(billet, toolSDF, strategy);
    } else {
        cutDualTrack(billet, toolSDF);
    }
}

double computeVolume(const openvdb::FloatGrid::Ptr& grid) {
    openvdb::tools::LevelSetMeasure<openvdb::FloatGrid> measure(*grid);
    return measure.volume();
}

openvdb::FloatGrid::Ptr buildLocalCutSurface(
    const BilletModel& billet, const ToolSweepSDF& lastTool) {
    // Show ONLY the newly exposed cut face: where the CSG result surface
    // is determined by the tool (not the original billet boundary)
    double displayVs = std::max(billet.config.d_v, 0.05);
    float bandWidth = 3.0f * static_cast<float>(displayVs);

    auto xform = openvdb::math::Transform::createLinearTransform(displayVs);
    auto grid = openvdb::FloatGrid::create(bandWidth);
    grid->setTransform(xform);
    grid->setGridClass(openvdb::GRID_LEVEL_SET);

    auto bbox = lastTool.getBoundingBox();
    bbox.expand(displayVs * 3);
    auto minIdx = xform->worldToIndexCellCentered(bbox.min());
    auto maxIdx = xform->worldToIndexCellCentered(bbox.max());

    auto& geo = billet.geometry;
    auto acc = grid->getAccessor();

    openvdb::Coord ijk;
    for (ijk[0] = minIdx[0]; ijk[0] <= maxIdx[0]; ++ijk[0]) {
        for (ijk[1] = minIdx[1]; ijk[1] <= maxIdx[1]; ++ijk[1]) {
            for (ijk[2] = minIdx[2]; ijk[2] <= maxIdx[2]; ++ijk[2]) {
                Vec3d wp = xform->indexToWorld(ijk);

                // Billet signed distance (analytic box)
                double dx = std::max(geo.origin.x()-wp.x(), wp.x()-(geo.origin.x()+geo.dims.x()));
                double dy = std::max(geo.origin.y()-wp.y(), wp.y()-(geo.origin.y()+geo.dims.y()));
                double dz = std::max(geo.origin.z()-wp.z(), wp.z()-(geo.origin.z()+geo.dims.z()));
                float billetSdf;
                if (dx<=0 && dy<=0 && dz<=0)
                    billetSdf = (float)std::max({dx,dy,dz});
                else {
                    double ex=std::max(dx,0.0), ey=std::max(dy,0.0), ez=std::max(dz,0.0);
                    billetSdf = (float)std::sqrt(ex*ex+ey*ey+ez*ez);
                }

                float toolSdf = -(float)lastTool.eval(wp); // negated: inside tool = positive

                // CSG difference: max(billet, -tool)
                float csgSdf = std::max(billetSdf, toolSdf);

                // Only write if near the CSG surface AND the surface is
                // determined by the tool (not the original billet face)
                if (std::abs(csgSdf) < bandWidth && toolSdf >= billetSdf) {
                    acc.setValue(ijk, csgSdf);
                }
            }
        }
    }
    return grid;
}

openvdb::FloatGrid::Ptr buildSurfelMesh(
    const BilletModel& billet, const ToolSweepSDF& lastTool) {
    if (!billet.microGrid) return nullptr;

    // Use d_v precision for the cut surface mesh (surfel-level accuracy)
    const double d_v = billet.config.d_v;
    const float bandWidth = 3.0f * static_cast<float>(d_v);

    auto xform = openvdb::math::Transform::createLinearTransform(d_v);
    auto grid = openvdb::FloatGrid::create(bandWidth);
    grid->setTransform(xform);
    grid->setGridClass(openvdb::GRID_LEVEL_SET);

    // Only reconstruct within tool BBox (avoids full-domain rebuild)
    auto bbox = lastTool.getBoundingBox();
    bbox.expand(d_v * 3);
    auto minIdx = xform->worldToIndexCellCentered(bbox.min());
    auto maxIdx = xform->worldToIndexCellCentered(bbox.max());

    auto& geo = billet.geometry;
    auto acc = grid->getAccessor();

    openvdb::Coord ijk;
    for (ijk[0] = minIdx[0]; ijk[0] <= maxIdx[0]; ++ijk[0]) {
        for (ijk[1] = minIdx[1]; ijk[1] <= maxIdx[1]; ++ijk[1]) {
            for (ijk[2] = minIdx[2]; ijk[2] <= maxIdx[2]; ++ijk[2]) {
                Vec3d wp = xform->indexToWorld(ijk);

                // Billet signed distance (analytic box)
                double dx = std::max(geo.origin.x()-wp.x(), wp.x()-(geo.origin.x()+geo.dims.x()));
                double dy = std::max(geo.origin.y()-wp.y(), wp.y()-(geo.origin.y()+geo.dims.y()));
                double dz = std::max(geo.origin.z()-wp.z(), wp.z()-(geo.origin.z()+geo.dims.z()));
                float billetSdf;
                if (dx<=0 && dy<=0 && dz<=0)
                    billetSdf = (float)std::max({dx,dy,dz});
                else {
                    double ex=std::max(dx,0.0), ey=std::max(dy,0.0), ez=std::max(dz,0.0);
                    billetSdf = (float)std::sqrt(ex*ex+ey*ey+ez*ez);
                }

                float toolSdf = -(float)lastTool.eval(wp);

                // CSG difference surface: only where tool determines the surface
                float csgSdf = std::max(billetSdf, toolSdf);
                if (std::abs(csgSdf) < bandWidth && toolSdf >= billetSdf) {
                    acc.setValue(ijk, csgSdf);
                }
            }
        }
    }

    return grid;
}

} // namespace ygg
