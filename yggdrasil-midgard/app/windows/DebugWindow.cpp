#include "DebugWindow.h"
#include "AppState.h"
#include "core/IDebugDisplay.h"
#include "core/IPWBuilder.h"
#include "core/ToolSweptSDF.h"
#include "core/SimEngine.h"
#include <imgui.h>
#include <cmath>
#include <vector>
#include <string>

namespace {

void visualizeSweepBody(const midgard::ToolSweptSDF& sdf) {
    if (!midgard::g_debugDisplay) return;
    midgard::g_debugDisplay->clear();

    auto bbox = sdf.boundingBox();
    const double step = 0.5;
    std::vector<float> pts;

    for (double z = bbox.min().z(); z <= bbox.max().z(); z += step)
        for (double y = bbox.min().y(); y <= bbox.max().y(); y += step)
            for (double x = bbox.min().x(); x <= bbox.max().x(); x += step) {
                if (std::abs(sdf.eval(midgard::Vec3d(x, y, z))) < step * 0.5)
                    pts.insert(pts.end(), {(float)x, (float)y, (float)z});
            }

    if (!pts.empty())
        midgard::g_debugDisplay->drawPoints(pts.data(), pts.size() / 3, 0xFFFF8800);
}

void visualizeMacroBoundary(const midgard::IPWState& ipw) {
    if (!midgard::g_debugDisplay || !ipw.macroGrid) return;

    const auto& xform = ipw.macroGrid->transform();
    double vs = xform.voxelSize()[0];
    double hv = vs * 0.5;
    double threshold = vs * std::sqrt(3.0) / 2.0 + 1e-4;

    std::vector<float> lines;
    for (auto it = ipw.macroGrid->cbeginValueOn(); it; ++it) {
        if (std::abs(*it) > threshold) continue;
        auto w = xform.indexToWorld(it.getCoord());
        float x0 = (float)(w.x() - hv), x1 = (float)(w.x() + hv);
        float y0 = (float)(w.y() - hv), y1 = (float)(w.y() + hv);
        float z0 = (float)(w.z() - hv), z1 = (float)(w.z() + hv);
        float e[] = {
            x0,y0,z0, x1,y0,z0,  x1,y0,z0, x1,y1,z0,
            x1,y1,z0, x0,y1,z0,  x0,y1,z0, x0,y0,z0,
            x0,y0,z1, x1,y0,z1,  x1,y0,z1, x1,y1,z1,
            x1,y1,z1, x0,y1,z1,  x0,y1,z1, x0,y0,z1,
            x0,y0,z0, x0,y0,z1,  x1,y0,z0, x1,y0,z1,
            x1,y1,z0, x1,y1,z1,  x0,y1,z0, x0,y1,z1,
        };
        lines.insert(lines.end(), e, e + 72);
    }
    if (!lines.empty())
        midgard::g_debugDisplay->drawLines(lines.data(), lines.size() / 3, 0xCC00FF00);
}

} // anon

namespace midgard {

void DebugWindow::draw() {
    if (ImGui::Begin("Debug")) {
        drawTabs();
        ImGui::Separator();
        drawStats();
    }
    ImGui::End();
}

// ─── Tab 分页 ──────────────────────────────────────────────
void DebugWindow::drawTabs() {
    if (ImGui::BeginTabBar("DebugTabs")) {
        if (ImGui::BeginTabItem("Billet")) {
            activeTab_ = 0;
            drawTabBillet();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Tool")) {
            activeTab_ = 1;
            drawTabTool();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Simulation")) {
            activeTab_ = 2;
            drawTabSimulation();
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
}

// ─── Billet 调试 ───────────────────────────────────────────
void DebugWindow::drawTabBillet() {
    auto& state = getAppState();
    auto& bd = state.billetDef;

    const char* typeNames[] = {"Box", "Cylinder", "Mesh"};
    ImGui::Text("Type: %s", typeNames[(int)bd.type]);
    if (bd.type == GeometryDef::BOX) {
        ImGui::Text("Dims: %.1f x %.1f x %.1f", bd.dims[0], bd.dims[1], bd.dims[2]);
    } else if (bd.type == GeometryDef::CYLINDER) {
        ImGui::Text("Radius: %.1f  Height: %.1f", bd.radius, bd.height);
    }
    ImGui::Text("Origin: (%.1f, %.1f, %.1f)", bd.origin[0], bd.origin[1], bd.origin[2]);

    // IPW 状态
    if (state.ipw.macroGrid) {
        ImGui::Text("Macro: %llu voxels @ %.2f mm",
            (unsigned long long)state.ipw.macroGrid->activeVoxelCount(),
            state.ipw.macroGrid->voxelSize()[0]);
    } else {
        ImGui::TextColored(ImVec4(1,0.4f,0.4f,1), "Macro: (null)");
    }

    ImGui::Spacing();
    ImGui::TextDisabled("Config: tol=%.3f mm  voxel=%.2f mm",
        state.ipw.config.user_t, state.ipw.config.voxelMacro);

    ImGui::Spacing();

    // ── Macro Grid 显示控制 ──
    if (!g_debugDisplay) {
        ImGui::TextDisabled("No debug display");
        return;
    }

    ImGui::Checkbox("Show Macro Grid", &state.ipwShowMacroGrid);
    ImGui::SameLine();
    if (ImGui::Button("Clear")) {
        g_debugDisplay->clear();
    }

    ImGui::Spacing();

    // ── Billet Builder Test（build 内部含 Debug Section） ──
    if (ImGui::Button("Billet Builder Test")) {
        IPWBuilder builder;
        state.ipw = builder.build(state.billetDef, state.ipw.config,
            [](const char* level, const char* msg) { midgard::getAppState().addLog(level, msg); },
            true);  // Billet Builder Test → 始终显示 boundary bbox
    }
}

// ─── Tool 调试 ─────────────────────────────────────────────
void DebugWindow::drawTabTool() {
    auto& state = getAppState();

    if (state.toolLibrary.empty()) {
        ImGui::TextDisabled("No tools in library");
        return;
    }

    ImGui::Checkbox("Show Tool", &state.showTool);
    ImGui::SameLine();
    ImGui::Checkbox("Show Axis", &state.showToolAxis);
}

// ─── Simulation 调试 ───────────────────────────────────────
void DebugWindow::drawTabSimulation() {
    auto& state = getAppState();

    bool canCut = !state.pathSegments.empty()
        && state.currentSegment < state.totalSegments
        && state.ipw.macroGrid;

    if (!canCut) ImGui::BeginDisabled();
    if (ImGui::Button("MacroCut One Seg")) {
        if (state.ipwDirty || !state.ipw.macroGrid) {
            IPWBuilder builder;
            state.ipw = builder.build(state.billetDef, state.ipw.config);
            state.ipwDirty = false;
        }

        auto& seg = state.pathSegments[state.currentSegment];
        auto& toolDef = state.toolLibrary[seg.toolId].def;

        ToolSweptSDF sdf(toolDef, seg);
        visualizeSweepBody(sdf);

        SimEngine engine;
        auto result = engine.cutSegment(state.ipw, seg, toolDef, state.billetDef, state.ipw.config);

        visualizeMacroBoundary(state.ipw);

        state.activeVoxels = (int)state.ipw.macroGrid->activeVoxelCount();
        state.lastCutTimeMs = result.elapsedMs;
        state.addLog("Cut",
            "seg " + std::to_string(state.currentSegment + 1) + "/" + std::to_string(state.totalSegments)
            + " L=" + std::to_string((int)result.pathLength) + "mm"
            + " | d=" + std::to_string(result.deletedVoxels)
            + " c=" + std::to_string(result.cutVoxels)
            + " n=" + std::to_string(result.newBoundaryVoxels)
            + " | " + std::to_string((int)result.elapsedMs) + "ms");

        ++state.currentSegment;
    }
    if (!canCut) ImGui::EndDisabled();

    ImGui::SameLine();
    ImGui::Text("Seg %d/%d", state.currentSegment + 1, state.totalSegments);

    if (ImGui::Button("Reset")) {
        state.currentSegment = 0;
        state.ipwDirty = true;
        state.macroMeshClear = true;
        if (g_debugDisplay) g_debugDisplay->clear();
        state.addLog("Info", "Reset");
    }
    ImGui::SameLine();
    if (ImGui::Button("Show Macro Mesh")) {
        state.macroMeshRequested = true;
    }
}

// ─── 状态栏 ────────────────────────────────────────────────
void DebugWindow::drawStats() {
    auto& state = getAppState();

    float fps = ImGui::GetIO().Framerate;
    ImGui::Text("FPS: %.0f  |  IPW: %s  |  Tools: %zu  |  Path: %d",
        fps,
        state.ipw.microGrid ? "ready" : "empty",
        state.toolLibrary.size(),
        state.totalSegments);
}

} // namespace midgard
