#include "MicroGridLabWindow.h"
#include "AppState.h"
#include "core/IDebugDisplay.h"
#include "core/ToolSweepSurface.h"
#include <imgui.h>
#include <cmath>
#include <cstdio>

namespace midgard {

void MicroGridLabWindow::draw() {
    ImGui::Begin("MicroGrid Lab");
    drawParams();
    ImGui::Separator();
    drawToolSetup();
    ImGui::Separator();
    drawCutHistory();
    ImGui::Separator();
    drawActions();
    ImGui::End();
}

// ─── 参数设置 ────────────────────────────────────────────────
void MicroGridLabWindow::drawParams() {
    auto& st = getAppState();
    ImGui::Text("── Parameters ──");

    ImGui::InputDouble("Cube Size", &state_.cubeSize, 0.1, 1.0, "%.1f mm");
    ImGui::InputDouble("Voxel Size", &state_.voxelSize, 0.1, 1.0, "%.2f mm");

    // 检测参数变化 → 自动重建
    static double prevCubeSize = state_.cubeSize;
    static double prevVoxelSize = state_.voxelSize;
    if (state_.cubeSize != prevCubeSize || state_.voxelSize != prevVoxelSize) {
        prevCubeSize = state_.cubeSize;
        prevVoxelSize = state_.voxelSize;
        state_.rebuild();
    }

    // 三档精度
    static const double precisions[] = {0.1, 0.01, 0.001};
    ImGui::Text("Precision:");
    ImGui::SameLine();
    ImGui::RadioButton("0.1", &precisionIndex_, 0);
    ImGui::SameLine();
    ImGui::RadioButton("0.01", &precisionIndex_, 1);
    ImGui::SameLine();
    ImGui::RadioButton("0.001", &precisionIndex_, 2);
    state_.precision = precisions[precisionIndex_];

    // 派生参数显示（关键参数可见性）
    int vc = state_.voxelCount();
    int depth = state_.maxOctreeDepth();
    double ceSize = state_.voxelSize / 8.0;
    double minCellSize = ceSize / (1 << depth);

    ImGui::Text("Voxels: %d  MaxDepth: %d", vc, depth);
    ImGui::Text("CE size: %.4f mm  Min cell: %.5f mm", ceSize, minCellSize);

    ImGui::Text("Bnd thresh: %.4f mm  NB halfwidth: %.3f mm",
        state_.voxelSize / 8.0 * 0.866, 3.0 * state_.voxelSize / 8.0);

    static bool prevSweep = true;
    if (ImGui::Checkbox("Show Sweep Body", &st.showSweepBody)) {
        pushToViewport();  // 即时刷新视口
    }

    if (ImGui::Button("Reinit")) {
        state_.init();
    }
}

// ─── 刀具设定 ────────────────────────────────────────────────
void MicroGridLabWindow::drawToolSetup() {
    ImGui::Text("── Tool ──");

    static const char* toolTypes[] = {"Ball End", "Flat End", "Bull Nose"};
    int typeIdx = static_cast<int>(state_.currentTool.type);
    if (ImGui::Combo("Type", &typeIdx, toolTypes, 3))
        state_.currentTool.type = static_cast<ToolType>(typeIdx);

    ImGui::InputDouble("R (radius)", &state_.currentTool.R, 0.5, 1.0, "%.1f mm");
    if (state_.currentTool.type == ToolType::BULL_NOSE)
        ImGui::InputDouble("r (fillet)", &state_.currentTool.r, 0.1, 0.5, "%.2f mm");
    ImGui::InputDouble("H (height)", &state_.currentTool.H, 1.0, 5.0, "%.1f mm");

    ImGui::Spacing();
    ImGui::Text("Segment:");
    ImGui::InputDouble("Start X", &state_.currentSegment.start[0], 0.1, 1.0, "%.2f");
    ImGui::InputDouble("Start Y", &state_.currentSegment.start[1], 0.1, 1.0, "%.2f");
    ImGui::InputDouble("Start Z", &state_.currentSegment.start[2], 0.1, 1.0, "%.2f");
    ImGui::InputDouble("End X",   &state_.currentSegment.end[0],   0.1, 1.0, "%.2f");
    ImGui::InputDouble("End Y",   &state_.currentSegment.end[1],   0.1, 1.0, "%.2f");
    ImGui::InputDouble("End Z",   &state_.currentSegment.end[2],   0.1, 1.0, "%.2f");

    if (ImGui::Button("Add to CutHistory")) {
        CutRecord rec;
        rec.tool    = state_.currentTool;
        rec.segment = state_.currentSegment;
        state_.addCutRecord(rec);
    }
}

// ─── 切削记录列表 ────────────────────────────────────────────
void MicroGridLabWindow::drawCutHistory() {
    ImGui::Text("── CutHistory (%zu records) ──", state_.cutHistory.size());

    for (auto& rec : state_.cutHistory) {
        const char* typeName = "???";
        switch (rec.tool.type) {
            case ToolType::BALL_END:  typeName = "BallEnd"; break;
            case ToolType::FLAT_END:  typeName = "FlatEnd"; break;
            case ToolType::BULL_NOSE: typeName = "BullNose"; break;
        }
        ImGui::Text("#%u %s R=%.1f (%.2f,%.2f,%.2f)→(%.2f,%.2f,%.2f)",
            rec.seqIndex, typeName, rec.tool.R,
            rec.segment.start[0], rec.segment.start[1], rec.segment.start[2],
            rec.segment.end[0],   rec.segment.end[1],   rec.segment.end[2]);
    }

    if (!state_.cutHistory.empty() && ImGui::Button("Clear All")) {
        state_.cutHistory.clear();
    }
}

// ─── 操作按钮 ────────────────────────────────────────────────
void MicroGridLabWindow::drawActions() {
    ImGui::Text("── Actions ──");

    // ── Load to Viewport: 将 MicroGridLab 结果推入主视图 ──
    if (ImGui::Button("Load to Viewport")) {
        pushToViewport();
        auto& st = getAppState();
        char buf[128];
        snprintf(buf, sizeof(buf), "Loaded %zu cut(s) to viewport", state_.cutHistory.size());
        st.addLog("MicroGrid", buf);
        logStats();
    }

    ImGui::Separator();

    // ── 切削执行 — Execute Next / Execute All ──
    static size_t nextCutIdx = 0;
    bool hasCuts  = !state_.cutHistory.empty();
    // 自动懒初始化：如果 voxels 为空则自动开辟（不清理 cutHistory）
    bool hasVoxels = !state_.voxels.empty();
    if (hasCuts && !hasVoxels) {
        state_.ensureInit();
        hasVoxels = !state_.voxels.empty();
        nextCutIdx = 0;
        auto& st = getAppState();
        char buf[128];
        snprintf(buf, sizeof(buf), "Auto-init %d voxels (%.2f mm³)",
            state_.voxelCount(), state_.voxelSize);
        st.addLog("MicroGrid", buf);
    }
    bool canExecNext = hasCuts && hasVoxels && nextCutIdx < state_.cutHistory.size();
    bool canExecAll  = hasCuts && hasVoxels;

    ImGui::BeginDisabled(!canExecAll);
    if (ImGui::Button("Execute All Cuts")) {
        auto& st = getAppState();
        char buf[128];
        snprintf(buf, sizeof(buf), "── Execute All (%zu cuts) ──", state_.cutHistory.size());
        st.addLog("MicroGrid", buf);

        for (auto& v : state_.voxels) v.setAllSolid();
        for (size_t i = 0; i < state_.cutHistory.size(); ++i) {
            state_.executeCut(i);
            std::string line;
            for (char c : state_.executeLog) {
                if (c == '\n') { if (!line.empty()) { st.addLog("MicroGrid", line); line.clear(); } }
                else { line += c; }
            }
            if (!line.empty()) st.addLog("MicroGrid", line);
        }
        nextCutIdx = state_.cutHistory.size();

            auto& log = state_.lastCutLog;
            snprintf(buf, sizeof(buf),
                "Done: %zu cuts — AIR=%d SOLID=%d BND=%d",
                state_.cutHistory.size(), log.airCeAfter, log.solidCeAfter, log.bndCeAfter);
            st.addLog("MicroGrid", buf);
            pushToViewport();
            logStats();
    }
    ImGui::EndDisabled();

    ImGui::SameLine();
    ImGui::BeginDisabled(!canExecNext);
    if (ImGui::Button("Execute Next")) {
        int activeBefore = 0;
        for (auto& v : state_.voxels) activeBefore += v.activeCount();

        state_.executeCut(nextCutIdx);

        int activeAfter = 0;
        for (auto& v : state_.voxels) activeAfter += v.activeCount();

        ++nextCutIdx;

        auto& st = getAppState();
        std::string line;
        for (char c : state_.executeLog) {
            if (c == '\n') { if (!line.empty()) { st.addLog("MicroGrid", line); line.clear(); } }
            else { line += c; }
        }
        if (!line.empty()) st.addLog("MicroGrid", line);

        auto& log = state_.lastCutLog;
        char buf[256];
        snprintf(buf, sizeof(buf),
            "Active CE: %d → %d  |  Surface pts: %d  |  Chordal err: %.5f mm",
            activeBefore, activeAfter,
            state_.totalSurfacePoints, state_.maxChordalError);
        st.addLog("MicroGrid", buf);
        pushToViewport();
        logStats();
    }
    ImGui::EndDisabled();

    ImGui::SameLine();
    if (ImGui::Button("Reset")) {
        state_.cutHistory.clear();
        state_.init();
        nextCutIdx = 0;
        auto& st = getAppState();
        st.addLog("MicroGrid", "── Full reset: voxels + cut history cleared ──");
        pushToViewport();
        logStats();
    }

    // ── 状态反馈 ──
    if (!hasCuts) {
        ImGui::TextDisabled("↳ Add cuts to CutHistory above, then Execute");
    } else if (!hasVoxels) {
        ImGui::TextDisabled("↳ Failed to init voxels — check cubeSize/voxelSize");
    } else if (nextCutIdx >= state_.cutHistory.size()) {
        ImGui::TextDisabled("↳ All %zu cuts executed. Reset to re-run.", state_.cutHistory.size());
    } else {
        ImGui::Text("Next cut: #%zu / %zu", nextCutIdx, state_.cutHistory.size());
    }
}

// ─── 统计日志 → Output 窗口 ─────────────────────────────────
void MicroGridLabWindow::logStats() {
    auto& st = getAppState();
    char buf[512];

    // ── 全局 CE 级三态统计 ──
    int totalAir = 0, totalSolid = 0, totalBoundary = 0;
    for (auto& v : state_.voxels) {
        for (int i = 0; i < 512; ++i) {
            if (v.isActive(i))        ++totalBoundary;
            else if (v.isInside(i))   ++totalSolid;
            else                       ++totalAir;
        }
    }
    int totalCE = static_cast<int>(state_.voxels.size()) * 512;

    snprintf(buf, sizeof(buf),
        "CE States: AIR=%d SOLID=%d BND=%d (total=%d, %.1f%%)",
        totalAir, totalSolid, totalBoundary, totalCE,
        totalCE ? 100.0 * totalBoundary / totalCE : 0.0);
    st.addLog("MicroGrid", buf);

    // ── Voxel 级三态统计 ──
    int solidVox = 0, bndVox = 0, airVox = 0;
    for (auto& v : state_.voxels) {
        if (v.activeCount() > 0) {
            ++bndVox;
        } else if (v.isInside(0)) {
            ++solidVox;
        } else {
            ++airVox;
        }
    }
    snprintf(buf, sizeof(buf),
        "Voxel States: SOLID=%d BND=%d AIR=%d (total=%zu)  Active CE=%d",
        solidVox, bndVox, airVox, state_.voxels.size(), totalBoundary);
    st.addLog("MicroGrid", buf);

    snprintf(buf, sizeof(buf),
        "Surface Points: %d  Refine Evals: %d  Max Chordal Error: %.5f mm  Last Cut: %.3f ms",
        state_.totalSurfacePoints, state_.totalRefineEvals,
        state_.maxChordalError, state_.lastCutMs);
    st.addLog("MicroGrid", buf);

    // ── 最近一次切削日志 ──
    auto& log = state_.lastCutLog;
    if (log.elapsedMs > 0.0) {
        int dAir   = log.airCeAfter  - log.airCeBefore;
        int dSolid = log.solidCeAfter - log.solidCeBefore;
        int dBnd   = log.bndCeAfter  - log.bndCeBefore;
        snprintf(buf, sizeof(buf),
            "Last Cut: Before AIR=%d SOLID=%d BND=%d",
            log.airCeBefore, log.solidCeBefore, log.bndCeBefore);
        st.addLog("MicroGrid", buf);
        snprintf(buf, sizeof(buf),
            "Last Cut: After  AIR=%d (%+d) SOLID=%d (%+d) BND=%d (%+d) [%.3f ms]",
            log.airCeAfter, dAir, log.solidCeAfter, dSolid, log.bndCeAfter, dBnd, log.elapsedMs);
        st.addLog("MicroGrid", buf);
    }

    // ── 增量切削 Delta ──
    auto& delta = state_.lastDelta;
    if (delta.solidToAir + delta.solidToBnd + delta.bndToAir + delta.bndUpdated > 0) {
        snprintf(buf, sizeof(buf),
            "Incremental Delta: SOLID→AIR=%d SOLID→BND=%d BND→AIR=%d BND updated=%d",
            delta.solidToAir, delta.solidToBnd, delta.bndToAir, delta.bndUpdated);
        st.addLog("MicroGrid", buf);
        if (delta.oldPtsKept + delta.oldPtsPruned > 0) {
            snprintf(buf, sizeof(buf),
                "Incremental Old Pts: %d kept / %d pruned",
                delta.oldPtsKept, delta.oldPtsPruned);
            st.addLog("MicroGrid", buf);
        }
    }

    snprintf(buf, sizeof(buf),
        "CutHistory: %zu records", state_.cutHistory.size());
    st.addLog("MicroGrid", buf);
}

// ─── 推送到视口 ─────────────────────────────────────────────
void MicroGridLabWindow::pushToViewport() {
    auto& st = getAppState();
    state_.ensureInit();

    // 毛坯
    st.billetDef.type = GeometryDef::BOX;
    st.billetDef.origin = Vec3d(0, 0, 0);
    st.billetDef.dims = Vec3d(state_.cubeSize, state_.cubeSize, state_.cubeSize);
    st.geometryLoaded = true;
    st.ipwDirty = true;
    st.showBillet = true;
    st.showToolPath = true;
    st.showTool = false;

    if (!g_debugDisplay) return;

    g_debugDisplay->clear();

    std::vector<float> sweepVerts, sweepLines, surfPts, ceLines, pathLines;
    std::vector<uint32_t> sweepIndices;

    // ── 刀具扫掠体：基于 ToolSweepSurface 参数面（平滑三角形网格）──
    if (st.showSweepBody) {
    const int uSegs = 40;   // 剖面方向 (工具底部→柄部，沿弧长均匀采样)
    const int vSegs = 80;   // 环向 (Left-Mid → Front-Cap → Right-Mid → Back-Cap)
    for (auto& rec : state_.cutHistory) {
        ToolSweepSurface sweep(rec.tool, rec.segment, true);
        bool isClosed = sweep.isClosedLoop();

        int nU = uSegs + 1;                             // u 采样点 (含端点)
        int nV = isClosed ? vSegs : (vSegs + 1);        // 闭圈时 v=1 ≡ v=0，多取一点用于 wrap

        // ── 顶点生成 (位置+法线交错，6 floats/顶点) ──
        std::vector<int> grid(nU * nV, -1);
        for (int i = 0; i < nU; ++i) {
            double u = (double)i / uSegs;
            for (int j = 0; j < nV; ++j) {
                double v = (double)j / vSegs;           // 闭圈时 v∈[0,1), 开口时 v∈[0,1]
                Vec3d p = sweep.eval(u, v);
                Vec3d n = sweep.normal(u, v);
                grid[i * nV + j] = (int)sweepVerts.size() / 6;
                sweepVerts.insert(sweepVerts.end(), {
                    (float)p[0],(float)p[1],(float)p[2],
                    (float)n[0],(float)n[1],(float)n[2]});
            }
        }

        // ── 三角形生成 (四边形 → 2△) ──
        for (int i = 0; i < uSegs; ++i) {
            for (int j = 0; j < vSegs; ++j) {
                int jn = isClosed ? ((j + 1) % vSegs) : (j + 1);
                int a = grid[ i      * nV + j ];
                int b = grid[(i + 1) * nV + j ];
                int c = grid[ i      * nV + jn];
                int d = grid[(i + 1) * nV + jn];

                sweepIndices.push_back(a); sweepIndices.push_back(b); sweepIndices.push_back(c);
                sweepIndices.push_back(c); sweepIndices.push_back(b); sweepIndices.push_back(d);
            }
        }

        // ── 端面线圈 (示意刀轴方向) ──
        double R = rec.tool.R;
        pathLines.insert(pathLines.end(), {
            (float)rec.segment.start[0], (float)rec.segment.start[1], (float)rec.segment.start[2],
            (float)rec.segment.end[0],   (float)rec.segment.end[1],   (float)rec.segment.end[2]});
        Vec3d axis = rec.segment.axis;
        double alen2 = sqrt(axis[0]*axis[0]+axis[1]*axis[1]+axis[2]*axis[2]);
        if (alen2<1e-6){alen2=1.0;axis=Vec3d(0,0,1);}
        Vec3d adir(axis[0]/alen2,axis[1]/alen2,axis[2]/alen2);
        Vec3d uVec,vVec;
        if(fabs(adir[0])<0.9)uVec=Vec3d(1,0,0);else uVec=Vec3d(0,1,0);
        double dp=adir[0]*uVec[0]+adir[1]*uVec[1]+adir[2]*uVec[2];
        uVec=Vec3d(uVec[0]-adir[0]*dp,uVec[1]-adir[1]*dp,uVec[2]-adir[2]*dp);
        double ul=sqrt(uVec[0]*uVec[0]+uVec[1]*uVec[1]+uVec[2]*uVec[2]);
        uVec=Vec3d(uVec[0]/ul,uVec[1]/ul,uVec[2]/ul);
        vVec=Vec3d(adir[1]*uVec[2]-adir[2]*uVec[1],adir[2]*uVec[0]-adir[0]*uVec[2],adir[0]*uVec[1]-adir[1]*uVec[0]);
        const int nAng=8;
        for(auto pos:{rec.segment.start,rec.segment.end})
            for(int a=0;a<nAng;++a){
                double ang=a*2.0*M_PI/nAng,ca=cos(ang),sa=sin(ang);
                Vec3d pt(pos[0]+R*(ca*uVec[0]+sa*vVec[0]),pos[1]+R*(ca*uVec[1]+sa*vVec[1]),pos[2]+R*(ca*uVec[2]+sa*vVec[2]));
                double ang2=((a+1)%nAng)*2.0*M_PI/nAng,ca2=cos(ang2),sa2=sin(ang2);
                Vec3d pt2(pos[0]+R*(ca2*uVec[0]+sa2*vVec[0]),pos[1]+R*(ca2*uVec[1]+sa2*vVec[1]),pos[2]+R*(ca2*uVec[2]+sa2*vVec[2]));
                sweepLines.insert(sweepLines.end(),{(float)pt[0],(float)pt[1],(float)pt[2],(float)pt2[0],(float)pt2[1],(float)pt2[2]});
            }
    }
    } // if (st.showSweepBody)

    // ── 切削表面点 ──
    // 注：表面点由八叉树在 CE 包围盒内提取，几何上必然在 [0, cubeSize]³ 内，无需钳位
    for (auto& sp : state_.surfacePoints)
        surfPts.insert(surfPts.end(), {(float)sp.position[0], (float)sp.position[1], (float)sp.position[2]});

    // ── 带 active CE 的 Voxel 线框 ──
    for (size_t vi = 0; vi < state_.voxels.size(); ++vi) {
        auto& cell = state_.voxels[vi];
        bool hasActive = false;
        for (int ci = 0; ci < 512; ++ci) { if (cell.isActive(ci)) { hasActive = true; break; } }
        if (!hasActive) continue;
        Vec3d vo = state_.voxelOrigin((int)vi);
        double vs = state_.voxelSize;
        float x0=(float)vo[0],y0=(float)vo[1],z0=(float)vo[2],x1=x0+(float)vs,y1=y0+(float)vs,z1=z0+(float)vs;
        float c[8][3]={{x0,y0,z0},{x1,y0,z0},{x1,y1,z0},{x0,y1,z0},{x0,y0,z1},{x1,y0,z1},{x1,y1,z1},{x0,y1,z1}};
        int e[12][2]={{0,1},{1,2},{2,3},{3,0},{4,5},{5,6},{6,7},{7,4},{0,4},{1,5},{2,6},{3,7}};
        for (auto& ed : e) ceLines.insert(ceLines.end(), {
            c[ed[0]][0],c[ed[0]][1],c[ed[0]][2],c[ed[1]][0],c[ed[1]][1],c[ed[1]][2]});
    }

    // ── 推送到 GPU ──
    // 刀具扫掠体: 透明蓝色实体
    if (!sweepVerts.empty() && !sweepIndices.empty())
        g_debugDisplay->drawTriangles(sweepVerts.data(), sweepIndices.data(),
                                       sweepIndices.size() / 3, 0x1AFF8800);
    // 切削表面点: 实色橙红
    if (!surfPts.empty())
        g_debugDisplay->drawPoints(surfPts.data(),   surfPts.size()/3,   0xFFFF4400);
    // 端面圈
    if (!sweepLines.empty())
        g_debugDisplay->drawLines(sweepLines.data(), sweepLines.size()/3, 0xFFFF6600);
    // 刀轴
    if (!pathLines.empty())
        g_debugDisplay->drawLines(pathLines.data(),  pathLines.size()/3,  0x44FFFF00);
    // Voxel 线框
    if (!ceLines.empty())
        g_debugDisplay->drawLines(ceLines.data(),    ceLines.size()/3,    0xCC00FF00);
}

} // namespace midgard
