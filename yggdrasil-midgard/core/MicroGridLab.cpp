#include "core/MicroGridLab.h"
#include "core/ToolSweptSDF.h"
#include "core/OctreeRefiner.h"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <unordered_set>

// TBB 并行化
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/enumerable_thread_specific.h>

namespace midgard {

// ═══════════════════════════════════════════════════════════════
// 辅助：popcount 在 [0, bit) 之间的 active 位数
// ═══════════════════════════════════════════════════════════════
namespace {

/// 计算 mask[0..7] 中 bit idx 之前有多少 active 位
int popcountBefore(const uint64_t mask[8], int idx) {
    int slot   = idx >> 6;       // idx / 64
    int bit    = idx & 63;       // idx % 64
    int count  = 0;
    // 前面整 slot 的全部
    for (int s = 0; s < slot; ++s)
        count += __builtin_popcountll(mask[s]);
    // 当前 slot 内 bit 之前的位
    if (bit > 0)
        count += __builtin_popcountll(mask[slot] & ((1ULL << bit) - 1));
    return count;
}

/// SDF 量化 / 反量化
inline int16_t encodeSDF(float sdf, float bg) {
    float clamped = std::max(-bg, std::min(bg, sdf));
    return static_cast<int16_t>(clamped / bg * 32767.0f);
}

inline float decodeSDF(int16_t v, float bg) {
    return static_cast<float>(v) * bg / 32767.0f;
}

/// ce 局部坐标 → 线性索引 (ix, iy, iz 各 0..7)
inline int ceIndex(int ix, int iy, int iz) {
    return (iz << 6) | (iy << 3) | ix;   // iz*64 + iy*8 + ix
}

} // anonymous

// ═══════════════════════════════════════════════════════════════
// MicroGridCell
// ═══════════════════════════════════════════════════════════════

int MicroGridCell::activeCount() const {
    int n = 0;
    for (int i = 0; i < 8; ++i)
        n += __builtin_popcountll(activeMask[i]);
    return n;
}

bool MicroGridCell::isActive(int idx) const {
    int slot = idx >> 6;
    int bit  = idx & 63;
    return (activeMask[slot] >> bit) & 1ULL;
}

bool MicroGridCell::isInside(int idx) const {
    int slot = idx >> 6;
    int bit  = idx & 63;
    return (signMask[slot] >> bit) & 1ULL;
}

bool MicroGridCell::isOutside(int idx) const {
    return !isInside(idx);
}

void MicroGridCell::setActiveBit(int idx, bool v) {
    int slot = idx >> 6;
    int bit  = idx & 63;
    if (v)
        activeMask[slot] |=  (1ULL << bit);
    else
        activeMask[slot] &= ~(1ULL << bit);
}

void MicroGridCell::setSignBit(int idx, bool v) {
    int slot = idx >> 6;
    int bit  = idx & 63;
    if (v)
        signMask[slot] |=  (1ULL << bit);
    else
        signMask[slot] &= ~(1ULL << bit);
}

int MicroGridCell::solidCount() const {
    int n = 0;
    for (int i = 0; i < 8; ++i)
        n += __builtin_popcountll(signMask[i]);
    return n;
}

void MicroGridCell::setAllSolid() {
    for (int i = 0; i < 8; ++i) {
        activeMask[i] = 0;
        signMask[i]   = 0xFFFFFFFFFFFFFFFFULL;
    }
    sdf.clear();
}

void MicroGridCell::clear() {
    for (int i = 0; i < 8; ++i) {
        activeMask[i] = 0;
        signMask[i]   = 0;
    }
    sdf.clear();
}

float MicroGridCell::getValue(int idx, float bg) const {
    if (!isActive(idx))
        return isInside(idx) ? +bg : -bg;   // 正=材料, 负=空气
    int offset = popcountBefore(activeMask, idx);
    return decodeSDF(sdf[offset], bg);
}

MicroGridCell::Classification
MicroGridCell::buildFromSDF(const float sdf512[512],
                             float threshold, float bg) {
    Classification cls;

    // 第一遍：分类，写入 mask
    for (int i = 0; i < 8; ++i) {
        activeMask[i] = 0;
        signMask[i]   = 0;
    }

    for (int i = 0; i < 512; ++i) {
        float s = sdf512[i];
        if (s > threshold) {
            // 远正 → 材料（内部）：signMask=1, activeMask=0
            setSignBit(i, true);
            ++cls.solidCount;
        } else if (s < -threshold) {
            // 远负 → 空气（外部）：signMask=0, activeMask=0
            ++cls.airCount;
        } else {
            // 窄带（边界）：activeMask=1, signMask=(s>0→材料)
            setActiveBit(i, true);
            setSignBit(i, s > 0);
            ++cls.boundaryCount;
        }
    }

    // 第二遍：重建压缩 SDF（只存 active ce 的值）
    sdf.clear();
    sdf.reserve(cls.boundaryCount);
    for (int i = 0; i < 512; ++i) {
        if (isActive(i)) {
            sdf.push_back(encodeSDF(sdf512[i], bg));
        }
    }

    return cls;
}

// ── M5: 增量 ce 更新 ────────────────────────────────────────
MicroGridCell::CeUpdate
MicroGridCell::updateCeFromToolSDF(int ceIdx, float toolSDF,
                                    float threshold, float bg) {
    bool oldActive = isActive(ceIdx);
    bool oldInside = isInside(ceIdx);

    // 旧状态分类
    // oldActive=1 → BOUNDARY (窄带)
    // oldActive=0, oldInside=1 → SOLID (材料)
    // oldActive=0, oldInside=0 → AIR (已切除)

    // 新刀 SDF 分类（只关新刀的影响）
    bool toolInside = (toolSDF < -threshold);      // ce 中心在刀内部 → 切除
    bool toolBoundary = (std::abs(toolSDF) < threshold); // 在刀面附近

    // 刀外部 (toolSDF > +threshold) → 刀不影响此 ce
    // 刀边界 (|toolSDF| < threshold) → ce 可能与刀面相交
    // 刀内部 (toolSDF < -threshold) → ce 被完全切掉

    // 已在 air 的 ce 不可逆 → NO_CHANGE
    if (!oldActive && !oldInside)
        return CeUpdate::NO_CHANGE;

    // SOLID ce: 被刀切到
    if (!oldActive && oldInside) {
        if (toolInside) {
            // SOLID → AIR: ce 中心深陷刀内，整个 ce 切除
            setSignBit(ceIdx, false);
            return CeUpdate::SOLID_TO_AIR;
        }
        if (toolBoundary) {
            // SOLID → BOUNDARY: ce 中心在刀面附近 → 窄带
            // 插入压缩 SDF（位置 = 当前 popcount(chunk) 之前 + chunk 内偏移）
            int offset = popcountBefore(activeMask, ceIdx);
            sdf.insert(sdf.begin() + offset, encodeSDF(toolSDF, bg));
            setActiveBit(ceIdx, true);
            setSignBit(ceIdx, toolSDF > 0);
            return CeUpdate::SOLID_TO_BND;
        }
        // toolSDF > +threshold → 刀离此 ce 远 → NO_CHANGE
        return CeUpdate::NO_CHANGE;
    }

    // BOUNDARY ce: 已在前表面，新刀可能影响
    if (oldActive) {
        if (toolInside) {
            // BND → AIR: 整个 ce 被完全切掉
            int offset = popcountBefore(activeMask, ceIdx);
            sdf.erase(sdf.begin() + offset);
            setActiveBit(ceIdx, false);
            setSignBit(ceIdx, false);
            return CeUpdate::BND_TO_AIR;
        }
        if (toolBoundary) {
            // BND → BND: 更新 SDF 值 (新刀面取代旧刀面)
            int offset = popcountBefore(activeMask, ceIdx);
            float oldVal = decodeSDF(sdf[offset], bg);
            float newVal = std::min(oldVal, toolSDF);  // composite = min
            sdf[offset] = encodeSDF(newVal, bg);
            setSignBit(ceIdx, newVal > 0);
            return CeUpdate::BND_UNCHANGED;
        }
        // toolSDF > +threshold → 刀在此 ce 外 → NO_CHANGE
        return CeUpdate::NO_CHANGE;
    }

    return CeUpdate::NO_CHANGE;  // unreachable
}

// ═══════════════════════════════════════════════════════════════
// MicroGridLabState
// ═══════════════════════════════════════════════════════════════

void MicroGridLabState::init() {
    int total = voxelCount();
    voxels.clear();
    voxels.resize(total);
    for (auto& v : voxels)
        v.setAllSolid();

    cutHistory.clear();
    totalSurfacePoints = 0;
    lastCutMs          = 0.0;
    maxChordalError    = 0.0;
    lastCutLog         = {};
}

void MicroGridLabState::ensureInit() {
    if (!voxels.empty()) return;
    int total = voxelCount();
    voxels.resize(total);
    for (auto& v : voxels)
        v.setAllSolid();
}

int MicroGridLabState::voxelCount() const {
    int pd = perDim();
    return pd * pd * pd;
}

int MicroGridLabState::perDim() const {
    return static_cast<int>(std::ceil(cubeSize / voxelSize));
}

int MicroGridLabState::maxOctreeDepth() const {
    double ceSize = voxelSize / 8.0;
    if (ceSize <= precision || precision <= 0.0)
        return 0;
    double ratio = ceSize / precision;
    return static_cast<int>(std::ceil(std::log2(ratio)));
}

void MicroGridLabState::addCutRecord(const CutRecord& rec) {
    CutRecord r = rec;
    r.seqIndex = static_cast<uint32_t>(cutHistory.size());
    cutHistory.push_back(r);
}

void MicroGridLabState::reset() {
    // 保留 cutHistory — 仅清空执行结果，让用户重新 Execute
    surfacePoints.clear();
    totalSurfacePoints  = 0;
    lastCutMs           = 0.0;
    maxChordalError     = 0.0;
    totalRefineEvals    = 0;
    deepestRefineDepth  = 0;
    lastCutLog          = {};
    lastDelta           = {};
    executeLog.clear();

    // 重设 voxels 为全 SOLID（不 touch cutHistory）
    if (!voxels.empty()) {
        for (auto& v : voxels) v.setAllSolid();
    }
}

void MicroGridLabState::clampSurfacePoints() {
    for (auto& sp : surfacePoints) {
        for (int i = 0; i < 3; ++i) {
            if (sp.position[i] < 0) sp.position[i] = 0;
            if (sp.position[i] > cubeSize) sp.position[i] = cubeSize;
        }
    }
}

// ─── 世界坐标计算 ────────────────────────────────────────────
Vec3d MicroGridLabState::voxelOrigin(int voxelIdx) const {
    int pd = perDim();
    int ix = voxelIdx % pd;
    int iy = (voxelIdx / pd) % pd;
    int iz = voxelIdx / (pd * pd);
    return Vec3d(ix * voxelSize, iy * voxelSize, iz * voxelSize);
}

Vec3d MicroGridLabState::ceCenterWorld(int voxelIdx, int ceIdx) const {
    int  cex = ceIdx & 7;              // ceIdx % 8
    int  cey = (ceIdx >> 3) & 7;       // (ceIdx / 8) % 8
    int  cez = ceIdx >> 6;             // ceIdx / 64
    double ceSize = voxelSize / 8.0;
    double halfCe = ceSize * 0.5;
    Vec3d vo = voxelOrigin(voxelIdx);
    return Vec3d(
        vo.x() + cex * ceSize + halfCe,
        vo.y() + cey * ceSize + halfCe,
        vo.z() + cez * ceSize + halfCe
    );
}

double MicroGridLabState::boundaryThreshold() const {
    // ce 对角线半长 = ceSize * sqrt(3) / 2
    double ceSize = voxelSize / 8.0;
    return ceSize * 0.8660254037844386;  // sqrt(3)/2
}

double MicroGridLabState::narrowBandHalfWidth() const {
    return 3.0 * (voxelSize / 8.0);      // 3 ce 窄带半宽
}

// ─── M2: 切削执行（粗筛）──────────────────────────────────────
void MicroGridLabState::executeCut(size_t cutIdx) {
    if (cutIdx >= cutHistory.size()) return;
    if (voxels.empty()) return;

    executeLog.clear();

    const auto& rec = cutHistory[cutIdx];
    const char* typeName = "???";
    switch (rec.tool.type) {
        case ToolType::BALL_END:  typeName = "BallEnd"; break;
        case ToolType::FLAT_END:  typeName = "FlatEnd"; break;
        case ToolType::BULL_NOSE: typeName = "BullNose"; break;
    }

    auto t0 = std::chrono::high_resolution_clock::now();

    char buf[256];
    snprintf(buf, sizeof(buf),
        "Cut #%u: %s R=%.1f (%.2f,%.2f,%.2f)→(%.2f,%.2f,%.2f)",
        rec.seqIndex, typeName, rec.tool.R,
        rec.segment.start[0], rec.segment.start[1], rec.segment.start[2],
        rec.segment.end[0],   rec.segment.end[1],   rec.segment.end[2]);
    executeLog += buf;
    executeLog += "\n";

    // 构建新刀的 SDF
    ToolSweptSDF toolSDF(rec.tool, rec.segment);

    double thresh = boundaryThreshold();
    double bg     = narrowBandHalfWidth();
    double ceSize = voxelSize / 8.0;

    // 临时缓冲区：512 个 float
    float sdfBuf[512];

    // 切削前的全局统计
    int totalAirBefore = 0, totalSolidBefore = 0, totalBndBefore = 0;

    // 对每个 voxel 执行粗筛
    for (int vi = 0; vi < static_cast<int>(voxels.size()); ++vi) {
        auto& cell = voxels[vi];

        // 统计切削前状态
        for (int i = 0; i < 512; ++i) {
            if (cell.isActive(i))      ++totalBndBefore;
            else if (cell.isInside(i)) ++totalSolidBefore;
            else                        ++totalAirBefore;
        }

        // ① 对每个 ce 中心 eval tool_sdf
        for (int ci = 0; ci < 512; ++ci) {
            Vec3d center = ceCenterWorld(vi, ci);
            sdfBuf[ci] = static_cast<float>(toolSDF.eval(center));
        }

        // ② 粗筛分类 + 更新 cell
        // 算法 7.14 ①:
        //   外部 ce → 跳过（已是空气，新刀无法再切）
        //   内部 ce → eval tool_sdf:
        //     tool > +thresh  → 保持实心
        //     tool < -thresh  → 变为空气
        //     |tool| < thresh → 变为边界
        //   边界 ce → eval tool_sdf:
        //     全正/全负/跨零 → M3 精修

        float newSDF[512];
        for (int i = 0; i < 512; ++i) {
            if (cell.isOutside(i) && !cell.isActive(i)) {
                // 空气（已切除）：保持为负（空气不复原）
                newSDF[i] = -bg;  // 远负 = 空气
            } else {
                // 内部或边界：取 min(旧值, 新刀)
                float oldVal = cell.getValue(i, bg);
                float toolVal = sdfBuf[i];
                newSDF[i] = std::min(oldVal, toolVal);
            }
        }

        // 用新的 SDF 重建 cell
        cell.buildFromSDF(newSDF, thresh, bg);
    }

    // 记录阶段 1-2 日志
    {
        int nVoxels = static_cast<int>(voxels.size());
        int totalCe = nVoxels * 512;
        snprintf(buf, sizeof(buf),
            "  Phase 1/3 Coarse screening: %d CE × %d voxels = %d SDF evals\n"
            "  Phase 2/3 Build SDF: before AIR=%d SOLID=%d BND=%d",
            totalCe, nVoxels, totalCe,
            totalAirBefore, totalSolidBefore, totalBndBefore);
        executeLog += buf;
        executeLog += "\n";
    }

    // ─── M3: 精修 — 对边界 ce 执行自适应八叉树表面提取 ───
    surfacePoints.clear();
    OctreeConfig octCfg;
    octCfg.chordalTol = precision;
    octCfg.maxDepth   = maxOctreeDepth();

    double zeroTol    = octCfg.effectiveZeroCrossTol();
    int    refinedCeCount = 0;
    totalRefineEvals   = 0;
    deepestRefineDepth = octCfg.maxDepth;  // 八叉树会用到的最大深度

    for (int vi = 0; vi < static_cast<int>(voxels.size()); ++vi) {
        const auto& cell = voxels[vi];
        Vec3d vo = voxelOrigin(vi);

        for (int ci = 0; ci < 512; ++ci) {
            if (!cell.isActive(ci)) continue;  // 只处理边界 ce

            ++refinedCeCount;

            // 构建 ce 的 world-space bbox
            int  cex = ci & 7;
            int  cey = (ci >> 3) & 7;
            int  cez = ci >> 6;
            double ceLowX = vo.x() + cex * ceSize;
            double ceLowY = vo.y() + cey * ceSize;
            double ceLowZ = vo.z() + cez * ceSize;

            openvdb::BBoxd ceBbox(Vec3d(ceLowX, ceLowY, ceLowZ),
                                   Vec3d(ceLowX + ceSize,
                                         ceLowY + ceSize,
                                         ceLowZ + ceSize));

            auto pts = extractSurface(ceBbox, toolSDF, octCfg);
            totalRefineEvals += octCfg.evalCount;
            for (auto& sp : pts)
                surfacePoints.push_back(sp);
        }
    }

    // ─── M4: 弦高误差（必须在日志之前计算）───
    maxChordalError = 0.0;
    for (const auto& sp : surfacePoints) {
        double err = std::abs(toolSDF.eval(sp.position));
        if (err > maxChordalError) maxChordalError = err;
    }
    totalRefineEvals += static_cast<int>(surfacePoints.size());  // 弦高误差 evals

    totalSurfacePoints = static_cast<int>(surfacePoints.size());

    // 切削后统计
    int totalAirAfter = 0, totalSolidAfter = 0, totalBndAfter = 0;
    for (auto& cell : voxels) {
        for (int i = 0; i < 512; ++i) {
            if (cell.isActive(i))      ++totalBndAfter;
            else if (cell.isInside(i)) ++totalSolidAfter;
            else                        ++totalAirAfter;
        }
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    lastCutMs = std::chrono::duration<double, std::milli>(t1 - t0).count();

    // 写入日志
    lastCutLog = {
        totalAirBefore, totalSolidBefore, totalBndBefore,
        totalAirAfter,  totalSolidAfter,  totalBndAfter,
        lastCutMs
    };

    totalSurfacePoints = static_cast<int>(surfacePoints.size());

    // 阶段 3 日志
    {
        int dAir   = totalAirAfter   - totalAirBefore;
        int dSolid = totalSolidAfter - totalSolidBefore;
        int dBnd   = totalBndAfter   - totalBndBefore;
        int activeVox = 0;
        for (auto& v : voxels) if (v.activeCount() > 0) ++activeVox;
        double ptsPerCe = refinedCeCount ? (double)totalSurfacePoints / refinedCeCount : 0;
        snprintf(buf, sizeof(buf),
            "  Phase 3/3 Octree refine: %d boundary CE → %d surface points (%.1f pts/CE)\n"
            "    Refine evals: %d  Max depth: %d  Chordal err: %.5f mm\n"
            "  After: AIR=%d (%+d) SOLID=%d (%+d) BND=%d (%+d)\n"
            "  Voxels active: %d/%zu  Total time: %.3f ms",
            refinedCeCount, totalSurfacePoints, ptsPerCe,
            totalRefineEvals, deepestRefineDepth, maxChordalError,
            totalAirAfter, dAir, totalSolidAfter, dSolid, totalBndAfter, dBnd,
            activeVox, voxels.size(), lastCutMs);
        executeLog += buf;
        executeLog += "\n";
    }
}

// ═══════════════════════════════════════════════════════════════
// M5: executeCutIncremental — 增量切削管线 (TBB 并行化)
// ═══════════════════════════════════════════════════════════════
void MicroGridLabState::executeCutIncremental(size_t cutIdx) {
    if (cutIdx >= cutHistory.size() || voxels.empty()) return;
    if (cutIdx == 0) {
        executeCut(0);
        return;
    }

    executeLog.clear();

    auto t0 = std::chrono::high_resolution_clock::now();
    double ceSize = voxelSize / 8.0;
    double thresh = boundaryThreshold();
    double bg     = narrowBandHalfWidth();

    const auto& rec = cutHistory[cutIdx];
    ToolSweptSDF toolSDF(rec.tool, rec.segment);

    // ── 保存旧表面点 ──
    std::vector<SurfaceSample> oldPoints = std::move(surfacePoints);
    surfacePoints.clear();

    int nVoxels = static_cast<int>(voxels.size());
    int totalCe = nVoxels * 512;

    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // ① 粗筛: TBB 并行 eval toolSDF → 串行 apply 状态更新
    //   (并行 eval 占 ~99% 耗时，串行位操作 <1%)
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    std::vector<float> toolVals(totalCe);
    std::vector<int>   updateTags(totalCe);  // 0=none,1=solid→air,2=solid→bnd,3=bnd→air,4=bnd→bnd

    tbb::parallel_for(tbb::blocked_range<int>(0, totalCe),
        [&](const tbb::blocked_range<int>& r) {
            for (int flat = r.begin(); flat < r.end(); ++flat) {
                int vi = flat / 512;
                int ci = flat % 512;
                const auto& cell = voxels[vi];
                Vec3d vo = voxelOrigin(vi);

                double cx = vo.x() + ((ci & 7) + 0.5) * ceSize;
                double cy = vo.y() + (((ci >> 3) & 7) + 0.5) * ceSize;
                double cz = vo.z() + ((ci >> 6) + 0.5) * ceSize;
                float tv = static_cast<float>(toolSDF.eval(Vec3d(cx, cy, cz)));
                toolVals[flat] = tv;

                // 预分类 (只读): 不修改 cell
                bool oldActive = cell.isActive(ci);
                bool oldInside = cell.isInside(ci);
                if (!oldActive && !oldInside) { updateTags[flat] = 0; continue; } // air→no change
                bool toolIn = (tv < -thresh);
                bool toolBnd = (std::abs(tv) < thresh);
                if (!oldActive && oldInside) {
                    updateTags[flat] = toolIn ? 1 : (toolBnd ? 2 : 0);
                } else {
                    updateTags[flat] = toolIn ? 3 : (toolBnd ? 4 : 0);
                }
            }
        });

    // 串行 apply (MicroGridCell 变长 SDF 写入有依赖，只能串行)
    int solidToAir  = 0, solidToBnd = 0;
    int bndToAir    = 0, bndUpdated = 0;
    int noChange    = 0;
    std::vector<int> refineCandidates;

    for (int flat = 0; flat < totalCe; ++flat) {
        int vi = flat / 512;
        int ci = flat % 512;
        int tag = updateTags[flat];
        if (tag == 0) { ++noChange; continue; }

        auto update = voxels[vi].updateCeFromToolSDF(ci, toolVals[flat], thresh, bg);
        switch (update) {
            case MicroGridCell::CeUpdate::SOLID_TO_AIR: ++solidToAir; break;
            case MicroGridCell::CeUpdate::SOLID_TO_BND:
                ++solidToBnd; refineCandidates.push_back(flat); break;
            case MicroGridCell::CeUpdate::BND_TO_AIR:   ++bndToAir;   break;
            case MicroGridCell::CeUpdate::BND_UNCHANGED:
                ++bndUpdated; refineCandidates.push_back(flat); break;
            default: ++noChange; break;
        }
    }

    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // ② 旧点验证: TBB 并行 eval → thread-local kept/pruned
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    int oldPointsKept  = 0;
    int oldPointsPruned = 0;

    if (!oldPoints.empty()) {
        struct ThreadLocal {
            std::vector<SurfaceSample> kept;
            int pruned = 0;
        };
        tbb::enumerable_thread_specific<ThreadLocal> tls;

        tbb::parallel_for(tbb::blocked_range<size_t>(0, oldPoints.size()),
            [&](const tbb::blocked_range<size_t>& r) {
                auto& local = tls.local();
                for (size_t i = r.begin(); i < r.end(); ++i) {
                    if (toolSDF.eval(oldPoints[i].position) < 0)
                        ++local.pruned;
                    else
                        local.kept.push_back(oldPoints[i]);
                }
            });

        for (auto& tl : tls) {
            oldPointsKept   += static_cast<int>(tl.kept.size());
            oldPointsPruned += tl.pruned;
            surfacePoints.insert(surfacePoints.end(), tl.kept.begin(), tl.kept.end());
        }
    }

    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // ③ 精修: TBB 并行 — 每线程独立 edgeHash + surface collection
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    totalRefineEvals   = 0;
    int maxDepth       = maxOctreeDepth();
    deepestRefineDepth = maxDepth;  // 八叉树会用到的最大深度

    if (!refineCandidates.empty()) {
        struct RefineThreadLocal {
            std::vector<SurfaceSample> points;
            int evals = 0;
        };
        tbb::enumerable_thread_specific<RefineThreadLocal> tlsRefine;

        tbb::parallel_for(tbb::blocked_range<size_t>(0, refineCandidates.size()),
            [&](const tbb::blocked_range<size_t>& r) {
                auto& local = tlsRefine.local();
                for (size_t i = r.begin(); i < r.end(); ++i) {
                    int flatIdx = refineCandidates[i];
                    int vi = flatIdx / 512;
                    int ci = flatIdx % 512;
                    if (!voxels[vi].isActive(ci)) continue;

                    Vec3d vo = voxelOrigin(vi);
                    int  cex = ci & 7;
                    int  cey = (ci >> 3) & 7;
                    int  cez = ci >> 6;
                    double ceLowX = vo.x() + cex * ceSize;
                    double ceLowY = vo.y() + cey * ceSize;
                    double ceLowZ = vo.z() + cez * ceSize;

                    openvdb::BBoxd ceBbox(Vec3d(ceLowX, ceLowY, ceLowZ),
                                           Vec3d(ceLowX + ceSize,
                                                 ceLowY + ceSize,
                                                 ceLowZ + ceSize));

                    OctreeConfig octCfg;
                    octCfg.chordalTol = precision;
                    octCfg.maxDepth   = maxDepth;
                    auto pts = extractSurface(ceBbox, toolSDF, octCfg);
                    local.evals += octCfg.evalCount;
                    local.points.insert(local.points.end(), pts.begin(), pts.end());
                }
            });

        for (auto& tl : tlsRefine) {
            totalRefineEvals += tl.evals;
            surfacePoints.insert(surfacePoints.end(),
                                 tl.points.begin(), tl.points.end());
        }
    }

    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // 统计 (串行，轻量)
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    int totalAirAfter = 0, totalSolidAfter = 0, totalBndAfter = 0;
    for (auto& cell : voxels) {
        for (int i = 0; i < 512; ++i) {
            if (cell.isActive(i))      ++totalBndAfter;
            else if (cell.isInside(i)) ++totalSolidAfter;
            else                        ++totalAirAfter;
        }
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    lastCutMs = std::chrono::duration<double, std::milli>(t1 - t0).count();

    lastCutLog = {
        totalAirAfter   - solidToAir - bndToAir,
        totalSolidAfter + solidToAir + solidToBnd,
        totalBndAfter   + bndToAir   - solidToBnd,
        totalAirAfter, totalSolidAfter, totalBndAfter,
        lastCutMs
    };

    lastDelta = { solidToAir, solidToBnd, bndToAir, bndUpdated,
                  oldPointsKept, oldPointsPruned };

    totalSurfacePoints = static_cast<int>(surfacePoints.size());

    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // 弦高误差 (TBB 并行)
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    maxChordalError = 0.0;
    if (!surfacePoints.empty()) {
        tbb::enumerable_thread_specific<double> tlsMaxErr(0.0);

        tbb::parallel_for(tbb::blocked_range<size_t>(0, surfacePoints.size()),
            [&](const tbb::blocked_range<size_t>& r) {
                double& localMax = tlsMaxErr.local();
                for (size_t i = r.begin(); i < r.end(); ++i) {
                    double err = std::abs(toolSDF.eval(surfacePoints[i].position));
                    if (err > localMax) localMax = err;
                }
            });

        for (double e : tlsMaxErr)
            maxChordalError = std::max(maxChordalError, e);
        totalRefineEvals += static_cast<int>(surfacePoints.size());
    }

    // 增量日志
    {
        char buf[512];
        // 反推 before 值
        int tAirBefore   = totalAirAfter   - solidToAir - bndToAir;
        int tSolidBefore = totalSolidAfter + solidToAir + solidToBnd;
        int tBndBefore   = totalBndAfter   + bndToAir   - solidToBnd;
        int dAir = totalAirAfter - tAirBefore;
        int dSolid = totalSolidAfter - tSolidBefore;
        int dBnd = totalBndAfter - tBndBefore;
        int activeVox = 0;
        for (auto& v : voxels) if (v.activeCount() > 0) ++activeVox;
        int refinedCount = static_cast<int>(refineCandidates.size());
        snprintf(buf, sizeof(buf),
            "  Phase 3/3 Octree refine (incr): %d boundary CE → %d surface points\n"
            "    Refine evals: %d  Chordal err: %.5f mm  Max depth: %d\n"
            "  Incremental: SOLID→AIR=%d SOLID→BND=%d BND→AIR=%d BND updated=%d\n"
            "  Old pts: %d kept / %d pruned  |  After: AIR=%d (%+d) SOLID=%d (%+d) BND=%d (%+d)\n"
            "  Voxels active: %d/%zu  |  %.3f ms",
            refinedCount, totalSurfacePoints,
            totalRefineEvals, maxChordalError, deepestRefineDepth,
            solidToAir, solidToBnd, bndToAir, bndUpdated,
            oldPointsKept, oldPointsPruned,
            totalAirAfter, dAir, totalSolidAfter, dSolid, totalBndAfter, dBnd,
            activeVox, voxels.size(), lastCutMs);
        executeLog += buf;
        executeLog += "\n";
    }
}

} // namespace midgard
