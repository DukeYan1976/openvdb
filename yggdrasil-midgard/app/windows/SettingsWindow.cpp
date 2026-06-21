#include "SettingsWindow.h"
#include "AppState.h"
#include <imgui.h>

namespace midgard {

void SettingsWindow::draw() {
    if (ImGui::Begin("Settings")) {
        drawBilletSection();
        drawToolLibrarySection();
        drawPathSection();
    }
    ImGui::End();
}

// ─── Path ────────────────────────────────────────────────────
void SettingsWindow::drawPathSection() {
    if (ImGui::CollapsingHeader("Path", ImGuiTreeNodeFlags_DefaultOpen)) {
        auto& state = getAppState();

        if (ImGui::Button("Load .cls")) {
            // TODO: 文件对话框
        }
        ImGui::SameLine();
        if (ImGui::Button("Load Demo")) {
            state.loadDemoIT1();
            state.geometryLoaded = true;
        }

        double& tol = state.ipw.config.user_t;
        if (ImGui::InputDouble("Tolerance t", &tol, 0.005, 0.05, "%.3f")) {
            if (tol > 0.0001) {
                state.ipw.config = ToleranceConfig(tol);
                state.ipwDirty = true;
                state.addLog("Build", std::string("Tolerance changed: t=") +
                    std::to_string(tol) + " → voxel=" +
                    std::to_string(state.ipw.config.voxelMacro) + " mm");
            }
        }
        ImGui::SameLine();
        ImGui::TextDisabled("→ voxel %.2f mm", state.ipw.config.voxelMacro);

        ImGui::Checkbox("Show Path", &state.showToolPath);

        if (state.totalSegments > 0) {
            ImGui::Text("Segments: %d", state.totalSegments);
            if (!state.pathSegments.empty() && state.currentSegment < (int)state.pathSegments.size()) {
                int tid = state.pathSegments[state.currentSegment].toolId;
                ImGui::Text("Current Tool: #%d", tid);
            }
        }
    }
}

// ─── Billet ──────────────────────────────────────────────────
void SettingsWindow::drawBilletSection() {
    if (ImGui::CollapsingHeader("Billet")) {
        auto& state = getAppState();
        auto& bd = state.billetDef;

        ImGui::Checkbox("Show", &state.showBillet);
        ImGui::SliderFloat("Alpha", &state.billetAlpha, 0.0f, 1.0f);

        // Type
        const char* types[] = {"Box", "Cylinder", "Mesh"};
        int typeIdx = (int)bd.type;
        if (ImGui::Combo("Billet Type", &typeIdx, types, 3)) {
            auto oldType = bd.type;
            bd.type = (GeometryDef::Type)typeIdx;

            // 切换时调整 origin 以保持几何中心不变
            Vec3d oldCenter;
            if (oldType == GeometryDef::BOX) {
                oldCenter = bd.origin + Vec3d(bd.dims[0]*0.5, bd.dims[1]*0.5, bd.dims[2]*0.5);
            } else { // CYLINDER
                oldCenter = Vec3d(bd.origin[0], bd.origin[1], bd.origin[2] + bd.height*0.5);
            }

            if (bd.type == GeometryDef::CYLINDER) {
                if (bd.radius == 0.0) bd.radius = bd.dims[0] * 0.5;
                if (bd.height == 0.0) bd.height = bd.dims[2];
                bd.origin = Vec3d(oldCenter[0], oldCenter[1], oldCenter[2] - bd.height*0.5);
            } else { // BOX
                bd.origin = Vec3d(oldCenter[0] - bd.dims[0]*0.5,
                                   oldCenter[1] - bd.dims[1]*0.5,
                                   oldCenter[2] - bd.dims[2]*0.5);
            }

            state.ipwDirty = true;
        }

        // Size — 根据类型显示不同控件
        if (bd.type == GeometryDef::BOX) {
            float dims[3] = {(float)bd.dims[0], (float)bd.dims[1], (float)bd.dims[2]};
            if (ImGui::InputFloat3("Size (L/W/H)", dims, "%.1f")) {
                bd.dims = Vec3d(dims[0], dims[1], dims[2]);
                state.ipwDirty = true;
            }
        } else if (bd.type == GeometryDef::CYLINDER) {
            float rad = (float)bd.radius;
            float ht = (float)bd.height;
            if (ImGui::InputFloat("Radius", &rad, 1.0f, 5.0f, "%.1f")) {
                bd.radius = rad;
                state.ipwDirty = true;
            }
            if (ImGui::InputFloat("Height", &ht, 1.0f, 5.0f, "%.1f")) {
                bd.height = ht;
                state.ipwDirty = true;
            }
        }

        // Origin
        float org[3] = {(float)bd.origin[0], (float)bd.origin[1], (float)bd.origin[2]};
        if (ImGui::InputFloat3("Origin", org, "%.1f")) {
            bd.origin = Vec3d(org[0], org[1], org[2]);
            state.ipwDirty = true;
        }
    }
}

// ─── Tool Library ────────────────────────────────────────────
void SettingsWindow::drawToolLibrarySection() {
    if (ImGui::CollapsingHeader("Tool Library")) {
        auto& state = getAppState();

        for (size_t i = 0; i < state.toolLibrary.size(); ++i) {
            auto& entry = state.toolLibrary[i];
            bool selected = (state.currentToolId == entry.id);
            char label[32];
            snprintf(label, sizeof(label), "Tool #%d", entry.id);
            if (ImGui::Selectable(label, selected)) {
                state.currentToolId = entry.id;
                state.ipwDirty = true;
            }
        }

        if (!state.toolLibrary.empty()) {
            ToolDef* tool = &state.toolLibrary[0].def;
            for (auto& e : state.toolLibrary) {
                if (e.id == state.currentToolId) tool = &e.def;
            }

            const char* types[] = {"Ball End", "Flat End", "Bull Nose"};
            int typeIdx = (int)tool->type;
            if (ImGui::Combo("Tool Type", &typeIdx, types, 3)) {
                tool->type = (ToolType)typeIdx;
                state.ipwDirty = true;
            }
            if (ImGui::InputDouble("R", &tool->R, 0.1, 1.0, "%.1f")) {
                state.ipwDirty = true;
            }
            if (ImGui::InputDouble("r", &tool->r, 0.1, 1.0, "%.1f")) {
                state.ipwDirty = true;
            }
            if (ImGui::InputDouble("H", &tool->H, 0.1, 1.0, "%.1f")) {
                state.ipwDirty = true;
            }
        }

        ImGui::Separator();
        if (ImGui::Button("Add")) {
            int newId = state.toolLibrary.empty() ? 0 : state.toolLibrary.back().id + 1;
            state.toolLibrary.push_back({newId, {ToolType::BALL_END, 5.0, 0.0, 20.0}});
        }
        ImGui::SameLine();
        if (ImGui::Button("Remove") && !state.toolLibrary.empty()) {
            int cur = state.currentToolId;
            state.toolLibrary.erase(
                std::remove_if(state.toolLibrary.begin(), state.toolLibrary.end(),
                    [cur](const ToolEntry& e) { return e.id == cur; }),
                state.toolLibrary.end());
            state.currentToolId = state.toolLibrary.empty() ? 0 : state.toolLibrary[0].id;
            state.ipwDirty = true;
        }
    }
}

} // namespace midgard
