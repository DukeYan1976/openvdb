#include "core/CuttingEngine.h"
#include <openvdb/tools/Prune.h>
#include <openvdb/tools/LevelSetMeasure.h>
#include <openvdb/points/PointCount.h>
#include <openvdb/points/PointAttribute.h>
#include <openvdb/points/PointConversion.h>
#include <cmath>
#include <map>
#include <vector>

namespace ygg {

// 单轨切削（Phase 1 逻辑，不变）
static void cutSingleTrack(BilletModel& billet, const ToolSweepSDF& toolSDF) {
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

// 双轨切削：4-phase 流程
static void cutDualTrack(BilletModel& billet, const ToolSweepSDF& toolSDF) {
    auto& sdfGrid = billet.sdfGrid;
    auto& microGrid = billet.microGrid;
    auto& xform = sdfGrid->transform();
    double D_v = xform.voxelSize()[0];
    float bandWidth = 3.0f * static_cast<float>(D_v);

    // Phase 1: 宏观过滤 — 更新 SDF (同单轨逻辑)
    auto bbox = toolSDF.getBoundingBox();
    auto minIdx = xform.worldToIndexCellCentered(bbox.min());
    auto maxIdx = xform.worldToIndexCellCentered(bbox.max());

    auto sdfAccessor = sdfGrid->getAccessor();
    openvdb::Coord ijk;
    for (ijk[0] = minIdx[0]; ijk[0] <= maxIdx[0]; ++ijk[0]) {
        for (ijk[1] = minIdx[1]; ijk[1] <= maxIdx[1]; ++ijk[1]) {
            for (ijk[2] = minIdx[2]; ijk[2] <= maxIdx[2]; ++ijk[2]) {
                Vec3d worldPos = xform.indexToWorld(ijk);
                double toolDist = toolSDF.eval(worldPos);
                if (toolDist < bandWidth) {
                    float billetVal = sdfAccessor.getValue(ijk);
                    float newVal = std::max(billetVal, static_cast<float>(-toolDist));
                    if (newVal != billetVal)
                        sdfAccessor.setValue(ijk, newVal);
                }
            }
        }
    }

    // Phase 2: 微观面元剥离
    if (microGrid) {
        auto& tree = microGrid->tree();
        for (auto leaf = tree.beginLeaf(); leaf; ++leaf) {
            auto leafOrigin = leaf->origin();
            Vec3d leafWorld = xform.indexToWorld(leafOrigin);
            Vec3d leafMax = leafWorld + Vec3d(8 * D_v);
            if (leafWorld.x() > bbox.max().x() || leafMax.x() < bbox.min().x() ||
                leafWorld.y() > bbox.max().y() || leafMax.y() < bbox.min().y() ||
                leafWorld.z() > bbox.max().z() || leafMax.z() < bbox.min().z())
                continue;

            auto attrSetPtr = leaf->stealAttributeSet();
            auto* activeArr = attrSetPtr->get("active");
            if (!activeArr) {
                leaf->replaceAttributeSet(attrSetPtr.release(), true);
                continue;
            }

            auto* posArr = attrSetPtr->get("P");
            auto posHandle = openvdb::points::AttributeHandle<openvdb::Vec3f>::create(*posArr);
            auto activeHandle = openvdb::points::AttributeWriteHandle<uint8_t>::create(*activeArr);

            // 遍历叶节点中所有体素，确定每个点所属的体素
            // PointDataLeaf 的 value 存储的是累积 offset
            // 体素 n 的点范围 = [offset(n-1), offset(n))
            for (openvdb::Index voxelIdx = 0; voxelIdx < 512; ++voxelIdx) {
                openvdb::Index end = static_cast<openvdb::Index>(leaf->getValue(voxelIdx));
                openvdb::Index start = (voxelIdx == 0) ? openvdb::Index(0) :
                    static_cast<openvdb::Index>(leaf->getValue(voxelIdx - 1));
                if (start == end) continue;

                // 体素在叶节点中的局部坐标
                openvdb::Coord localCoord(
                    (voxelIdx >> 6) & 7,   // x: bits 6-8
                    (voxelIdx >> 3) & 7,   // y: bits 3-5
                    voxelIdx & 7);         // z: bits 0-2
                openvdb::Coord voxelCoord = leafOrigin + localCoord;

                for (openvdb::Index idx = start; idx < end; ++idx) {
                    if (activeHandle->get(idx) == 0) continue;

                    openvdb::Vec3f localP = posHandle->get(idx);
                    Vec3d worldPos = xform.indexToWorld(
                        Vec3d(voxelCoord.x() + localP.x(),
                              voxelCoord.y() + localP.y(),
                              voxelCoord.z() + localP.z()));

                    double dist = toolSDF.eval(worldPos);
                    if (dist <= 0.0) {
                        activeHandle->set(idx, 0);
                    }
                }
            }

            leaf->replaceAttributeSet(attrSetPtr.release(), true);
        }
    }

    // Phase 3: 边界面元注入（在刀具零等值面上注入精细面元）
    // 策略：找到部分切削的叶节点，在刀具表面采样新点，
    //        重建该叶节点的属性集（追加新点）
    {
        auto& tree = microGrid->tree();
        double d_v = billet.config.d_v;
        int N = billet.config.N;

        // 收集需要注入的新面元（按叶节点 origin 分组）
        struct NewSurfel { openvdb::Vec3f localP; openvdb::Vec3f normal; };
        std::map<openvdb::Coord, std::vector<NewSurfel>> injectionMap;

        for (auto leaf = tree.cbeginLeaf(); leaf; ++leaf) {
            auto leafOrigin = leaf->origin();
            Vec3d leafWorld = xform.indexToWorld(leafOrigin);
            Vec3d leafMax = leafWorld + Vec3d(8 * D_v);
            if (leafWorld.x() > bbox.max().x() || leafMax.x() < bbox.min().x() ||
                leafWorld.y() > bbox.max().y() || leafMax.y() < bbox.min().y() ||
                leafWorld.z() > bbox.max().z() || leafMax.z() < bbox.min().z())
                continue;

            // 检查是否部分切削
            auto& attrSet = leaf->attributeSet();
            auto* activeArr = attrSet.get("active");
            if (!activeArr) continue;
            auto ah = openvdb::points::AttributeHandle<uint8_t>::create(*activeArr);
            bool hasActive = false, hasInactive = false;
            for (size_t i = 0; i < ah->size() && !(hasActive && hasInactive); ++i) {
                if (ah->get(i) == 1) hasActive = true; else hasInactive = true;
            }
            if (!hasActive || !hasInactive) continue;

            // 在叶节点中心附近的刀具表面采样
            Vec3d center = (leafWorld + leafMax) * 0.5;
            Vec3d toolGrad = toolSDF.gradient(center);
            double gradLen = toolGrad.length();
            if (gradLen < 1e-10) continue;
            Vec3d normal = toolGrad / gradLen;

            Vec3d u, v;
            if (std::abs(normal.x()) < 0.9) u = Vec3d(1,0,0).cross(normal);
            else u = Vec3d(0,1,0).cross(normal);
            u.normalize(); v = normal.cross(u);

            int sampleN = std::min(N, 8);
            for (int i = 0; i < sampleN; ++i) {
                for (int j = 0; j < sampleN; ++j) {
                    Vec3d offset = ((i + 0.5 - sampleN/2.0) * d_v) * u +
                                   ((j + 0.5 - sampleN/2.0) * d_v) * v;
                    Vec3d candidate = center + offset;
                    double dist = toolSDF.eval(candidate);
                    candidate = candidate - dist * normal;

                    if (candidate.x() < leafWorld.x() || candidate.x() > leafMax.x() ||
                        candidate.y() < leafWorld.y() || candidate.y() > leafMax.y() ||
                        candidate.z() < leafWorld.z() || candidate.z() > leafMax.z())
                        continue;
                    if (std::abs(toolSDF.eval(candidate)) > d_v) continue;

                    // 转为 index-space 相对于体素的偏移
                    Vec3d idxPos = xform.worldToIndex(candidate);
                    openvdb::Coord voxelCoord(
                        static_cast<int>(std::floor(idxPos.x())),
                        static_cast<int>(std::floor(idxPos.y())),
                        static_cast<int>(std::floor(idxPos.z())));
                    openvdb::Vec3f localP(
                        static_cast<float>(idxPos.x() - voxelCoord.x()),
                        static_cast<float>(idxPos.y() - voxelCoord.y()),
                        static_cast<float>(idxPos.z() - voxelCoord.z()));

                    injectionMap[leafOrigin].push_back({localP,
                        openvdb::Vec3f(float(normal.x()), float(normal.y()), float(normal.z()))});
                }
            }
        }

        // 注入：用 createPointDataGrid 重建包含新旧点的叶节点不现实
        // 简化方案：创建新的 PointDataGrid 只包含新面元，存入独立 Grid
        // 实际合并需要 OpenVDB 的 PointMerge（复杂）
        // 务实方案：直接为新面元创建独立 PointDataGrid 挂在 BilletModel 上
        if (!injectionMap.empty()) {
            std::vector<openvdb::Vec3R> newWorldPts;
            std::vector<openvdb::Vec3f> newNormals;
            for (auto& [origin, surfels] : injectionMap) {
                for (auto& s : surfels) {
                    // 重建世界坐标（近似：用叶节点 origin + 偏移中心）
                    Vec3d leafW = xform.indexToWorld(origin);
                    Vec3d worldP = leafW + Vec3d(s.localP.x() * D_v,
                                                  s.localP.y() * D_v,
                                                  s.localP.z() * D_v);
                    newWorldPts.push_back(worldP);
                    newNormals.push_back(s.normal);
                }
            }

            if (!newWorldPts.empty()) {
                // 创建新 PointDataGrid 并追加属性
                auto newPtGrid = openvdb::points::createPointDataGrid<
                    openvdb::points::NullCodec, openvdb::points::PointDataGrid>(
                    newWorldPts, *microGrid->transformPtr());

                openvdb::points::TypedAttributeArray<openvdb::Vec3f>::registerType();
                openvdb::points::TypedAttributeArray<uint8_t>::registerType();

                size_t normalIdx = 0;
                for (auto tLeaf = newPtGrid->tree().beginLeaf(); tLeaf; ++tLeaf) {
                    auto tAttr = tLeaf->stealAttributeSet();
                    tAttr->appendAttribute("normal",
                        openvdb::points::TypedAttributeArray<openvdb::Vec3f>::attributeType(),
                        static_cast<openvdb::Index>(tLeaf->pointCount()));
                    tAttr->appendAttribute("precision",
                        openvdb::points::TypedAttributeArray<uint8_t>::attributeType(),
                        static_cast<openvdb::Index>(tLeaf->pointCount()));
                    tAttr->appendAttribute("active",
                        openvdb::points::TypedAttributeArray<uint8_t>::attributeType(),
                        static_cast<openvdb::Index>(tLeaf->pointCount()));

                    auto* nArr = tAttr->get("normal");
                    auto* pArr = tAttr->get("precision");
                    auto* aArr = tAttr->get("active");
                    auto whN = openvdb::points::AttributeWriteHandle<openvdb::Vec3f>::create(*nArr);
                    auto whP = openvdb::points::AttributeWriteHandle<uint8_t>::create(*pArr);
                    auto whA = openvdb::points::AttributeWriteHandle<uint8_t>::create(*aArr);
                    for (size_t i = 0; i < whN->size(); ++i) {
                        whN->set(i, (normalIdx < newNormals.size()) ? newNormals[normalIdx] : openvdb::Vec3f(0));
                        whP->set(i, 1); // FINE
                        whA->set(i, 1); // active
                        normalIdx++;
                    }
                    tLeaf->replaceAttributeSet(tAttr.release(), true);
                }

                // 合并新旧 Grid：用 steal + topologyCopy 方案
                // PointDataGrid::merge 对相同 descriptor 的树可用
                microGrid->tree().merge(newPtGrid->tree());
            }
        }
    }

    // Phase 4: pruneLevelSet
    openvdb::tools::pruneLevelSet(sdfGrid->tree());
}

void CuttingEngine::cut(BilletModel& billet, const ToolSweepSDF& toolSDF) {
    if (billet.isSingleTrack()) {
        cutSingleTrack(billet, toolSDF);
    } else {
        cutDualTrack(billet, toolSDF);
    }
}

double computeVolume(const openvdb::FloatGrid::Ptr& grid) {
    openvdb::tools::LevelSetMeasure<openvdb::FloatGrid> measure(*grid);
    return measure.volume();
}

} // namespace ygg
