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

namespace ygg {

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

// ─── Phase 2 helper: inject surfels into microGrid (safe rebuild if exists) ───
static void injectSurfels(BilletModel& billet,
                          const std::vector<openvdb::Vec3R>& positions,
                          const std::vector<openvdb::Vec3f>& normals,
                          const openvdb::math::Transform& xform) {
    if (positions.empty()) return;
    openvdb::points::TypedAttributeArray<openvdb::Vec3f>::registerType();
    openvdb::points::TypedAttributeArray<uint8_t>::registerType();

    // Collect existing active points if microGrid exists
    std::vector<openvdb::Vec3R> allPos;
    std::vector<openvdb::Vec3f> allNorm;

    if (billet.microGrid) {
        for (auto leaf = billet.microGrid->tree().cbeginLeaf(); leaf; ++leaf) {
            auto lo = leaf->origin();
            auto& as = leaf->attributeSet();
            auto* posArr = as.get("P");
            auto* normArr = as.get("normal");
            auto* actArr = as.get("active");
            if (!posArr) continue;
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
                    if (ah && ah->get(i)==0) continue; // skip inactive
                    auto p = ph->get(i);
                    allPos.push_back(xform.indexToWorld(
                        Vec3d(vc.x()+p.x(), vc.y()+p.y(), vc.z()+p.z())));
                    allNorm.push_back(nh ? nh->get(i) : openvdb::Vec3f(0,0,1));
                }
            }
        }
    }

    // Append new points
    allPos.insert(allPos.end(), positions.begin(), positions.end());
    allNorm.insert(allNorm.end(), normals.begin(), normals.end());

    // Build fresh PointDataGrid
    auto grid = openvdb::points::createPointDataGrid<
        openvdb::points::NullCodec, openvdb::points::PointDataGrid>(allPos, xform);
    grid->setName("micro_surfels");

    // Attach attributes
    size_t idx = 0;
    for (auto leaf = grid->tree().beginLeaf(); leaf; ++leaf) {
        auto as = leaf->stealAttributeSet();
        if (as->descriptor().find("normal") == openvdb::points::AttributeSet::INVALID_POS)
            as->appendAttribute("normal",
                openvdb::points::TypedAttributeArray<openvdb::Vec3f>::attributeType(),
                static_cast<openvdb::Index>(leaf->pointCount()));
        if (as->descriptor().find("active") == openvdb::points::AttributeSet::INVALID_POS)
            as->appendAttribute("active",
                openvdb::points::TypedAttributeArray<uint8_t>::attributeType(),
                static_cast<openvdb::Index>(leaf->pointCount()));
        leaf->replaceAttributeSet(as.release(), true);

        auto as2 = leaf->stealAttributeSet();
        auto whN = openvdb::points::AttributeWriteHandle<openvdb::Vec3f>::create(*as2->get("normal"));
        auto whA = openvdb::points::AttributeWriteHandle<uint8_t>::create(*as2->get("active"));
        for (size_t i = 0; i < whN->size(); ++i, ++idx) {
            whN->set(i, idx < allNorm.size() ? allNorm[idx] : openvdb::Vec3f(0));
            whA->set(i, 1);
        }
        leaf->replaceAttributeSet(as2.release(), true);
    }

    billet.microGrid = grid;
}

// 双轨切削：工业级 4-phase（增量注入 + 并行裁剪 + 分块采样）
static void cutDualTrack(BilletModel& billet, const ToolSweepSDF& toolSDF) {
    using TreeT = openvdb::FloatGrid::TreeType;
    auto& sdfGrid = billet.sdfGrid;
    auto& xform = sdfGrid->transform();
    double D_v = xform.voxelSize()[0];
    float bandWidth = 3.0f * static_cast<float>(D_v);

    auto bbox = toolSDF.getBoundingBox();
    auto minIdx = xform.worldToIndexCellCentered(bbox.min()) - openvdb::Coord(1);
    auto maxIdx = xform.worldToIndexCellCentered(bbox.max()) + openvdb::Coord(1);

    printf("[DualTrack] D_v=%.4f d_v=%.4f N=%d\n", D_v, billet.config.d_v, billet.config.N);
    fflush(stdout);

    // ═══ Phase 1: SDF CSG diff (TBB parallel) + dirty region ═══
    // Activate inactive negative voxels in tool BBox
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

    // TBB parallel SDF update + dirty voxel collection
    openvdb::tree::LeafManager<TreeT> leafMgr(sdfGrid->tree());
    using DirtyVec = std::vector<openvdb::Coord>;
    tbb::enumerable_thread_specific<DirtyVec> tlsDirty;

    tbb::parallel_for(leafMgr.leafRange(),
        [&](const openvdb::tree::LeafManager<TreeT>::LeafRange& range) {
            auto& localDirty = tlsDirty.local();
            for (auto leafIt = range.begin(); leafIt; ++leafIt) {
                auto& leaf = *leafIt;
                auto lo = leaf.origin();
                if (lo.x()+8<minIdx.x() || lo.x()>maxIdx.x() ||
                    lo.y()+8<minIdx.y() || lo.y()>maxIdx.y() ||
                    lo.z()+8<minIdx.z() || lo.z()>maxIdx.z()) continue;

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
                            // Dirty criteria: tool interior + near new surface
                            // Catches deep penetration (oldVal was background positive)
                            if (toolDist <= 0.0 && std::abs(newVal) < bandWidth)
                                localDirty.push_back(coord);
                        }
                    }
                }
            }
        });

    // Merge + dedup dirty voxels
    std::vector<openvdb::Coord> dirtyVoxels;
    for (auto& v : tlsDirty) {
        for (auto& coord : v) {
            if (billet.microGrid && billet.microGrid->tree().isValueOn(coord)) continue;
            billet.dirtyMask->getAccessor().setValueOn(coord);
            dirtyVoxels.push_back(coord);
        }
    }

    printf("[DualTrack] Phase1: %zu dirty voxels\n", dirtyVoxels.size());
    fflush(stdout);

    // ═══ Phase 2: Chunked surfel generation + single injection ═══
    if (!dirtyVoxels.empty()) {
        const int N = billet.config.N;
        const double d_v = billet.config.d_v;
        const size_t CHUNK = 256;

        // Collect all surfels from all chunks
        std::vector<openvdb::Vec3R> allPos;
        std::vector<openvdb::Vec3f> allNorm;

        for (size_t start = 0; start < dirtyVoxels.size(); start += CHUNK) {
            size_t end = std::min(start + CHUNK, dirtyVoxels.size());
            std::vector<openvdb::Coord> chunk(dirtyVoxels.begin() + start,
                                              dirtyVoxels.begin() + end);
            auto batch = SurfelGenerator::sampleToolSurface(toolSDF, chunk, xform, d_v, N);
            allPos.insert(allPos.end(), batch.positions.begin(), batch.positions.end());
            allNorm.insert(allNorm.end(), batch.normals.begin(), batch.normals.end());
        }

        printf("[DualTrack] Phase2: %zu surfels generated (N=%d)\n", allPos.size(), N);
        fflush(stdout);

        // Single injection
        if (!allPos.empty()) {
            injectSurfels(billet, allPos, allNorm, xform);
        }
    }

    // ═══ Phase 3: Parallel surfel clipping ═══
    if (billet.microGrid) {
        using PtTreeT = openvdb::points::PointDataGrid::TreeType;
        openvdb::tree::LeafManager<PtTreeT> ptLeafMgr(billet.microGrid->tree());

        tbb::parallel_for(ptLeafMgr.leafRange(),
            [&](const openvdb::tree::LeafManager<PtTreeT>::LeafRange& range) {
                for (auto leafIt = range.begin(); leafIt; ++leafIt) {
                    auto& leaf = *leafIt;
                    auto lo = leaf.origin();
                    Vec3d lw = xform.indexToWorld(lo);
                    Vec3d lmax = lw + Vec3d(8.0 * D_v);
                    if (lw.x()>bbox.max().x()+D_v || lmax.x()<bbox.min().x()-D_v ||
                        lw.y()>bbox.max().y()+D_v || lmax.y()<bbox.min().y()-D_v ||
                        lw.z()>bbox.max().z()+D_v || lmax.z()<bbox.min().z()-D_v) continue;

                    // Each leaf is independent; steal its attribute set for mutable access
                    auto as = leaf.stealAttributeSet();
                    auto* posArr = as->get("P");
                    auto* actArr = as->get("active");
                    if (!posArr || !actArr) { leaf.replaceAttributeSet(as.release(), true); continue; }

                    auto ph = openvdb::points::AttributeHandle<openvdb::Vec3f>::create(*posArr);
                    auto wh = openvdb::points::AttributeWriteHandle<uint8_t>::create(*actArr);

                    for (openvdb::Index vIdx = 0; vIdx < 512; ++vIdx) {
                        openvdb::Index endI = static_cast<openvdb::Index>(leaf.getValue(vIdx));
                        openvdb::Index startI = (vIdx==0) ? openvdb::Index(0) :
                            static_cast<openvdb::Index>(leaf.getValue(vIdx-1));
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
                    leaf.replaceAttributeSet(as.release(), true);
                }
            });

        printf("[DualTrack] Phase3: parallel clip done\n"); fflush(stdout);
    }

    // ═══ Phase 4: Prune + clear ═══
    openvdb::tools::pruneLevelSet(sdfGrid->tree());
    billet.dirtyMask->clear();
    printf("[DualTrack] Done.\n"); fflush(stdout);
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

} // namespace ygg
