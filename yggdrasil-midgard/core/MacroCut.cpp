#include "core/MacroCut.h"
#include <openvdb/tools/Composite.h>
#include <openvdb/tools/LevelSetRebuild.h>
#include <openvdb/points/PointDataGrid.h>
#include <cmath>
#include <chrono>
#include <unordered_map>

namespace midgard {

/// 在指定范围内光栅化刀具解析SDF为FloatGrid (与MacroGrid同分辨率)
/// 只写入narrowband内的voxel (|sdf| < background)
static openvdb::FloatGrid::Ptr rasterizeToolGrid(
    const ToolSweepSDF& tool,
    const openvdb::math::Transform::Ptr& xform,
    int halfwidth,
    const openvdb::BBoxd& cutBBox)
{
    auto toolGrid = openvdb::FloatGrid::create(halfwidth * xform->voxelSize()[0]);
    toolGrid->setTransform(xform);
    toolGrid->setGridClass(openvdb::GRID_LEVEL_SET);

    // 光栅化范围 = cutBBox ± halfwidth (index space)
    openvdb::Coord minC = openvdb::Coord::floor(xform->worldToIndex(cutBBox.min()))
                          - openvdb::Coord(halfwidth);
    openvdb::Coord maxC = openvdb::Coord::ceil(xform->worldToIndex(cutBBox.max()))
                          + openvdb::Coord(halfwidth);

    auto acc = toolGrid->getAccessor();
    const double bg = toolGrid->background();

    // 暴力遍历裁剪范围内每个voxel，求解析SDF
    for (int z = minC.z(); z <= maxC.z(); ++z) {
        for (int y = minC.y(); y <= maxC.y(); ++y) {
            for (int x = minC.x(); x <= maxC.x(); ++x) {
                openvdb::Coord ijk(x, y, z);
                openvdb::Vec3d world = xform->indexToWorld(ijk);
                double sdf = tool.eval(world);
                // 只存储narrowband内的值，外部保持background
                if (std::abs(sdf) < bg) {
                    acc.setValue(ijk, static_cast<float>(sdf));
                }
            }
        }
    }
    return toolGrid;
}

/// Phase 0 主入口：宏观切削分类
/// 输入: IPW (macroGrid + microGrid) + 刀具扫掠体SDF
/// 输出: 三态分类 (deleted / cut / newBoundary)
///
/// 算法流程:
///   Step 1: 光栅化范围裁剪 — tool AABB ∩ workpiece AABB, 排除刀杆等不接触区域
///   Step 2: 在裁剪范围内光栅化刀具SDF为临时FloatGrid
///   Step 3: CSG差集 — 更新MacroGrid拓扑，激活新的切削边界voxel
///   Step 4: 三态分类 — 只遍历tool bbox内的leaf nodes，按SDF阈值分类:
///           - deleted: 完全在刀具内部 (SDF < -threshold)
///           - cut: 在切削边界且MicroGrid有既有点数据 (需剔除旧点+补新点)
///           - newBoundary: 在切削边界但MicroGrid无数据 (需从零生成点集)
CutClassification MacroCut::classifyVoxels(IPWState& ipw, const ToolSweepSDF& tool) {
    auto& macroGrid = ipw.macroGrid;
    const double V = macroGrid->voxelSize()[0];
    // 保守阈值: voxel对角线半长，确保只有整个voxel在刀具内才标记deleted
    const double threshold = V * std::sqrt(3.0) / 2.0;
    const int halfwidth = 3;

    auto t0 = std::chrono::high_resolution_clock::now();

    // ─── Step 1: 光栅化范围裁剪 ───────────────────────────────────────
    // tool AABB ∩ workpiece active AABB → 只在实际接触区域光栅化
    // 效果: 刀杆在工件上方时自动排除，减少光栅化体积
    openvdb::BBoxd toolBBox = tool.boundingBox();
    openvdb::CoordBBox activeCoordBox = macroGrid->evalActiveVoxelBoundingBox();
    openvdb::BBoxd workBBox(macroGrid->indexToWorld(activeCoordBox.min()),
                            macroGrid->indexToWorld(activeCoordBox.max()));
    openvdb::BBoxd cutBBox(
        openvdb::math::maxComponent(toolBBox.min(), workBBox.min()),
        openvdb::math::minComponent(toolBBox.max(), workBBox.max()));

    // 无交集 → 刀具不接触工件，直接返回空分类
    CutClassification result;
    if (cutBBox.min().x() >= cutBBox.max().x() ||
        cutBBox.min().y() >= cutBBox.max().y() ||
        cutBBox.min().z() >= cutBBox.max().z()) {
        return result;
    }

    // ─── Step 2: 光栅化刀具SDF ──────────────────────────────────────────
    // 在裁剪后的范围内构建临时FloatGrid，与MacroGrid同分辨率
    auto toolGrid = rasterizeToolGrid(tool, macroGrid->transformPtr(), halfwidth, cutBBox);

    auto t1 = std::chrono::high_resolution_clock::now();

    // ─── Step 3: CSG差集 ────────────────────────────────────────────────
    // in-place修改MacroGrid: max(ipw_sdf, -tool_sdf)
    // 关键作用: 激活原本inactive的内部voxel为新的narrowband边界
    //
    // ⚠️ 时序注意: CSG后 MacroGrid 拓扑已更新，但 MicroGrid 仍是 IPW_{n-1} 状态
    // Step 4 的分类利用这个不一致来区分 cut(旧数据存在) vs newBoundary(无旧数据)
    // MicroGrid 的更新由后续 Phase 3/4 完成（per-leaf 重建）
    openvdb::tools::csgDifference(*macroGrid, *toolGrid);

    // 修复CSG后的SDF场质量（renormalize distance field）
    macroGrid = openvdb::tools::levelSetRebuild(*macroGrid, 0.0f,
        halfwidth * macroGrid->voxelSize()[0]);

    auto t2 = std::chrono::high_resolution_clock::now();

    // ─── Step 4: 三态分类 ───────────────────────────────────────────────
    // 只遍历tool bbox内的leaf nodes (O(局部) 而非 O(全量))
    // 通过查询MicroGrid区分 cut(有既有数据) vs newBoundary(无数据)

    openvdb::points::PointDataGrid::ConstAccessor* microAcc = nullptr;
    std::unique_ptr<openvdb::points::PointDataGrid::ConstAccessor> microAccPtr;
    if (ipw.microGrid) {
        microAccPtr = std::make_unique<openvdb::points::PointDataGrid::ConstAccessor>(
            ipw.microGrid->getConstAccessor());
        microAcc = microAccPtr.get();
    }

    // tool bbox 转 index space，扩展 halfwidth 确保保守覆盖
    openvdb::Coord bboxMin = openvdb::Coord::floor(
        macroGrid->transform().worldToIndex(toolBBox.min())) - openvdb::Coord(halfwidth);
    openvdb::Coord bboxMax = openvdb::Coord::ceil(
        macroGrid->transform().worldToIndex(toolBBox.max())) + openvdb::Coord(halfwidth);
    openvdb::CoordBBox toolIdxBox(bboxMin, bboxMax);

    for (auto leaf = macroGrid->tree().cbeginLeaf(); leaf; ++leaf) {
        // 跳过不在tool范围内的leaf nodes
        if (!leaf->getNodeBoundingBox().hasOverlap(toolIdxBox)) continue;

        for (auto iter = leaf->cbeginValueOn(); iter; ++iter) {
            const openvdb::Coord coord = iter.getCoord();
            const openvdb::Vec3d worldPos = macroGrid->indexToWorld(coord);
            // 用解析SDF判断该voxel相对刀具的位置
            const double toolSdf = tool.eval(worldPos);
            // CSG后的MacroGrid SDF值（判断是否在工件零等值面附近）
            const float macroSdf = iter.getValue();

            if (toolSdf < -threshold) {
                // voxel_d: 完全在刀具内部，后续从MicroGrid删除对应点
                result.deleted.push_back(coord);
            } else if (toolSdf <= threshold && std::abs(macroSdf) < V) {
                // 边界voxel: 靠近扫掠面 AND 在工件零等值面附近
                // 只有同时满足两个条件的才是真正的切削界面voxel
                bool hasExistingData = false;
                if (microAcc) {
                    auto* mleaf = microAcc->probeConstLeaf(coord);
                    if (mleaf) {
                        // 检查该coord对应的具体voxel是否有点
                        const auto offset = mleaf->coordToOffset(coord);
                        const auto end = mleaf->getValue(offset);
                        const decltype(end) start = (offset == 0)
                            ? decltype(end)(0) : mleaf->getValue(offset - 1);
                        hasExistingData = (end > start);
                    }
                }
                if (hasExistingData) {
                    // voxel_c: MicroGrid有点 → 需剔除旧点 + 生成新切削面点
                    result.cut.push_back(coord);
                } else {
                    // voxel_n: MicroGrid无数据 → 需从零生成切削面点集
                    result.newBoundary.push_back(coord);
                }
            }
            // sdf > threshold: voxel完全在刀具外部，不受影响
        }
    }

    auto t3 = std::chrono::high_resolution_clock::now();

    // ─── Debug 输出 ─────────────────────────────────────────────────────
    DEBUG_SECTION(Phase0_Classify) {
        double msRaster = std::chrono::duration<double, std::milli>(t1 - t0).count();
        double msCSG = std::chrono::duration<double, std::milli>(t2 - t1).count();
        double msClassify = std::chrono::duration<double, std::milli>(t3 - t2).count();
        double msTotal = std::chrono::duration<double, std::milli>(t3 - t0).count();

        auto minI = macroGrid->transform().worldToIndexCellCentered(toolBBox.min());
        auto maxI = macroGrid->transform().worldToIndexCellCentered(toolBBox.max());
        openvdb::CoordBBox idxBox(minI, maxI);
        int leafCount = 0;
        for (auto it = macroGrid->tree().cbeginLeaf(); it; ++it) {
            if (it->getNodeBoundingBox().hasOverlap(idxBox)) ++leafCount;
        }

        DEBUG_INFO_OUT("Phase0: deleted=" + std::to_string(result.deleted.size())
                     + " cut=" + std::to_string(result.cut.size())
                     + " newBoundary=" + std::to_string(result.newBoundary.size())
                     + " leafNodes_in_bbox=" + std::to_string(leafCount)
                     + " | raster=" + std::to_string(msRaster) + "ms"
                     + " csg=" + std::to_string(msCSG) + "ms"
                     + " classify=" + std::to_string(msClassify) + "ms"
                     + " total=" + std::to_string(msTotal) + "ms");
    }

    return result;
}

/// Phase 1: 为每个边界 voxel 生成 VoxelTask（含参数域范围）
/// 使用 ToolSweepSurface 的统一参数面进行反向查询
std::vector<VoxelTask> MacroCut::buildTaskList(
    const CutClassification& cls,
    const ToolSweepSurface& surface,
    const ToleranceConfig& config,
    const openvdb::math::Transform& xform)
{
    const double V = config.voxelMacro;

    // ─── 构建 task 列表 ─────────────────────────────────────────────
    std::vector<VoxelTask> tasks;
    tasks.reserve(cls.cut.size() + cls.newBoundary.size());

    auto addCoords = [&](const std::vector<openvdb::Coord>& coords, VoxelClass vc) {
        for (const auto& coord : coords) {

            VoxelTask task;
            task.origin = coord;
            task.classification = vc;
            // voxel 物理 AABB (使用grid的transform正确转换)
            openvdb::Vec3d wmin = xform.indexToWorld(coord);
            openvdb::Vec3d wmax = xform.indexToWorld(coord.offsetBy(1, 1, 1));
            task.aabb = openvdb::BBoxd(wmin, wmax);
            // 参数域初始化为空（后续expand）
            task.u_min = 1.0; task.u_max = 0.0;
            task.t_min = 1.0; task.t_max = 0.0;
            tasks.push_back(task);
        }
    };
    addCoords(cls.cut, VoxelClass::CUT);
    addCoords(cls.newBoundary, VoxelClass::NEW_BOUNDARY);

    if (tasks.empty()) return tasks;

    auto t0 = std::chrono::high_resolution_clock::now();

    // ─── 参数面统一分块 → 反向查询匹配 voxel ─────────────────────
    // 分块数动态计算: 使每个参数块映射的3D范围≈1-2个voxel大小
    openvdb::BBoxd fullBBox = surface.bbox(0, 1, 0, 1);
    openvdb::Vec3d fullExtent = fullBBox.max() - fullBBox.min();
    // u方向≈周长(取XY平面最大跨度*π), v方向≈轴向展开长
    double diameter = std::max(fullExtent.x(), fullExtent.y());
    double circumference = M_PI * diameter;
    double axialLength = fullExtent.length();

    int N_U = std::clamp((int)std::ceil(circumference / V), 4, 16);
    int N_V = std::clamp((int)std::ceil(axialLength / V), 4, 16);

    const double margin = V * 0.9;  // 裕量外扩（覆盖bbox采样误差）
    const double du = 1.0 / N_U;
    const double dv = 1.0 / N_V;

    for (int iu = 0; iu < N_U; ++iu) {
        for (int iv = 0; iv < N_V; ++iv) {
            double u0 = iu * du, u1 = (iu + 1) * du;
            double v0 = iv * dv, v1 = (iv + 1) * dv;

            openvdb::BBoxd blockBBox = surface.bbox(u0, u1, v0, v1);
            blockBBox.expand(margin);

            for (size_t i = 0; i < tasks.size(); ++i) {
                if (tasks[i].aabb.hasOverlap(blockBBox)) {
                    tasks[i].u_min = std::min(tasks[i].u_min, u0);
                    tasks[i].u_max = std::max(tasks[i].u_max, u1);
                    tasks[i].t_min = std::min(tasks[i].t_min, v0);
                    tasks[i].t_max = std::max(tasks[i].t_max, v1);
                }
            }
        }
    }

    // 统计匹配成功的task数
    int matched = 0;
    for (const auto& task : tasks) {
        if (task.u_min <= task.u_max) matched++;
    }

    // 对未匹配的task: 暴力搜索找到最近参数位置，设局部邻域
    const double searchDu = 1.0 / 20.0;
    const double searchDv = 1.0 / 20.0;
    const double localRadius = 2.0 * std::max(du, dv); // 邻域半径

    for (auto& task : tasks) {
        if (task.u_min <= task.u_max) continue;  // 已匹配，跳过

        // 找voxel中心最近的参数面点
        Vec3d center = (task.aabb.min() + task.aabb.max()) * 0.5;
        double bestDist = 1e9;
        double bestU = 0.5, bestV = 0.5;

        for (double u = 0; u <= 1.0; u += searchDu) {
            for (double v = 0; v <= 1.0; v += searchDv) {
                Vec3d p = surface.eval(u, v);
                double d = (p - center).lengthSqr();
                if (d < bestDist) {
                    bestDist = d;
                    bestU = u;
                    bestV = v;
                }
            }
        }

        task.u_min = std::max(0.0, bestU - localRadius);
        task.u_max = std::min(1.0, bestU + localRadius);
        task.t_min = std::max(0.0, bestV - localRadius);
        task.t_max = std::min(1.0, bestV + localRadius);
    }

    auto t1 = std::chrono::high_resolution_clock::now();

    DEBUG_SECTION(Phase1_TaskList) {
        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        DEBUG_INFO_OUT("Phase1: tasks=" + std::to_string(tasks.size())
                     + " block_matched=" + std::to_string(matched)
                     + " nearest_search=" + std::to_string((int)tasks.size() - matched)
                     + " blocks=" + std::to_string(N_U * N_V)
                     + " N_U=" + std::to_string(N_U)
                     + " N_V=" + std::to_string(N_V)
                     + " time=" + std::to_string(ms) + "ms");
    }

    return tasks;
}

} // namespace midgard
