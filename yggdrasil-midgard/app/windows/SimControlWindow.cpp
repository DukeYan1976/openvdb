#include "SimControlWindow.h"
#include "AppState.h"
#include <imgui.h>

namespace midgard {

void SimControlWindow::draw() {
    if (ImGui::Begin("Simulation Control")) {
        drawControls();
        drawProgress();
    }
    ImGui::End();
}

void SimControlWindow::drawControls() {
    auto& state = getAppState();
    
    // Play / Pause / Stop
    if (ImGui::Button(state.simState == AppState::RUNNING ? "Pause" : "Play")) {
        if (state.simState == AppState::RUNNING) state.pauseSimulation();
        else state.startSimulation();
    }
    ImGui::SameLine();
    if (ImGui::Button("Stop")) state.stopSimulation();
    ImGui::SameLine();
    if (ImGui::Button("Step >")) {
        if (state.simState == AppState::IDLE) state.startSimulation();
        state.stepForward();
    }
    
    // Step / Speed
    ImGui::Checkbox("Step Mode", &state.stepMode);
    ImGui::SameLine();
    ImGui::SliderFloat("Speed", &state.speedMultiplier, 0.25f, 10.0f, "%.1fx");
}

void SimControlWindow::drawProgress() {
    auto& state = getAppState();
    
    if (state.totalSegments > 0) {
        float progress = (state.currentSegment + state.segmentProgress) / state.totalSegments;
        char buf[64];
        snprintf(buf, sizeof(buf), "Seg %d/%d (%.0f%%)",
                 state.currentSegment + 1, state.totalSegments,
                 progress * 100.0f);
        ImGui::ProgressBar(progress, ImVec2(-1, 0), buf);
    }
}

} // namespace midgard
