#include "OutputWindow.h"
#include "AppState.h"
#include <imgui.h>
#include <sstream>
#include <cstring>

namespace midgard {

void OutputWindow::draw() {
    if (ImGui::Begin("Output")) {
        drawStats();
        ImGui::Separator();
        drawLogStream();
    }
    ImGui::End();
}

void OutputWindow::drawLogStream() {
    auto& state = getAppState();
    auto& logs = state.logBuffer;

    // 构建完整文本（可选中、可 Ctrl+C 复制）
    std::stringstream ss;
    for (auto& entry : logs) {
        ss << "[" << entry.timestamp << "] [" << entry.level << "] "
           << entry.message << "\n";
    }

    // 用固定缓冲区适配 ImGui 的 char* 接口
    static std::vector<char> buf;
    std::string text = ss.str();
    if (buf.size() < text.size() + 1024)
        buf.resize(text.size() + 4096);
    std::memcpy(buf.data(), text.c_str(), text.size() + 1);

    ImGui::InputTextMultiline("##LogText", buf.data(), buf.size(),
        ImVec2(ImGui::GetContentRegionAvail().x, 150),
        ImGuiInputTextFlags_ReadOnly);

    // 自动追底
    if (autoScroll_)
        ImGui::SetScrollHereY(1.0f);

    // 底部按钮
    ImGui::Checkbox("Auto-scroll", &autoScroll_);
    ImGui::SameLine();
    if (ImGui::Button("Copy All")) {
        ImGui::SetClipboardText(text.c_str());
    }
    ImGui::SameLine();
    if (ImGui::Button("Clear")) {
        state.logBuffer.clear();
    }
}

void OutputWindow::drawStats() {
    auto& state = getAppState();

    // ── 算法核心状态 ──
    ImGui::TextDisabled("Engine Status");

    ImGui::Columns(2, nullptr, false);
    ImGui::Text("MacroGrid:");
    ImGui::NextColumn();
    if (state.ipw.macroGrid) {
        ImGui::Text("%d voxels @ %.2fmm",
            state.activeVoxels, state.ipw.config.voxelMacro);
    } else {
        ImGui::TextColored(ImVec4(1,0.4f,0.4f,1), "(null)");
    }
    ImGui::NextColumn();

    ImGui::Text("Last Cut:");
    ImGui::NextColumn();
    ImGui::Text("%.1f ms", state.lastCutTimeMs);
    ImGui::NextColumn();

    ImGui::Text("Avg Cut:");
    ImGui::NextColumn();
    ImGui::Text("%.1f ms", state.avgCutTimeMs);
    ImGui::NextColumn();

    ImGui::Columns(1);
}

} // namespace midgard