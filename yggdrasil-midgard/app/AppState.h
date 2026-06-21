#pragma once
#include "core/Types.h"
#include <vector>
#include <string>
#include <chrono>
#include <sstream>

namespace midgard {

// 日志条目（UI 概念）
struct LogEntry {
    std::string timestamp;
    std::string level;
    std::string message;
};

// 刀具库条目（id 是 UI 索引概念）
struct ToolEntry {
    int id = 0;
    ToolDef def;
};

// 全局应用状态
struct AppState {
    // ── 核心状态：IPW 全生命周期容器 ──
    IPWState ipw{ToleranceConfig(0.05)};
    GeometryDef billetDef;  // 毛坯定义 — 唯一来源
    bool ipwDirty = true;   // 需要 IPWBuilder::build() 重建

    // ── 仿真状态 ──
    enum SimState { IDLE, RUNNING, PAUSED };
    SimState simState = IDLE;
    int currentSegment = 0;
    int totalSegments = 0;
    float segmentProgress = 0.0f;
    float speedMultiplier = 1.0f;
    float cutSpeed = 10.0f;
    bool stepMode = false;
    bool cancelled = false;

    // ── Debug 控制 ──
    bool ipwShowMacroGrid = false;  // 自动显示 macroGrid leaf bboxes
    bool macroMeshRequested = false; // 请求 SceneRenderer 重建 macro mesh
    bool macroMeshClear = false;     // 请求清除 macro mesh

    // ── 加载/显示控制 ──
    bool geometryLoaded = false;   // 用户是否已加载几何（Load Demo / MicroGridLab Load）
    bool showBillet = true;
    bool showBilletWire = false;
    bool showToolPath = true;
    bool showToolAxis = false;
    bool showTool = true;
    bool showSweepBody = true;   // MicroGridLab 刀具扫掠体可见性
    bool inspectorActive = false;
    float billetAlpha = 0.6f;

    // ── 刀具库 ──
    std::vector<ToolEntry> toolLibrary;
    int currentToolId = 0;

    // ── 刀路数据 ──
    std::vector<MoveSegment> pathSegments;

    // ── 统计 ──
    int activeVoxels = 0;
    int activeSurfels = 0;
    double lastCutTimeMs = 0.0;
    double avgCutTimeMs = 0.0;
    double simTime = 0.0;

    // ── 日志 ──
    std::vector<LogEntry> logBuffer;
    void addLog(const std::string& level, const std::string& msg);

    // ── 方法 ──
    void loadDemoIT1();
    void tick(float dt);
    Vec3d getCurrentToolPosition() const;
    void startSimulation();
    void pauseSimulation();
    void stopSimulation();
    void stepForward();
};

AppState& getAppState();

} // namespace midgard
