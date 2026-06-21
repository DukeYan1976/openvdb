#include "AppState.h"
#include <cmath>
#include <iomanip>

namespace midgard {

static AppState g_state;

AppState& getAppState() { return g_state; }

void AppState::addLog(const std::string& level, const std::string& msg) {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%H:%M:%S")
       << '.' << std::setfill('0') << std::setw(3) << ms.count();
    logBuffer.push_back({ss.str(), level, msg});
    if (logBuffer.size() > 500) logBuffer.erase(logBuffer.begin());
}

void AppState::loadDemoIT1() {
    toolLibrary.clear();
    toolLibrary.push_back({0, {ToolType::BALL_END, 5.0, 0.0, 20.0}});

    billetDef.type = GeometryDef::BOX;
    billetDef.origin = Vec3d(0, 0, 0);
    billetDef.dims = Vec3d(50, 50, 20);
    ipwDirty = true;

    pathSegments.clear();
    MoveSegment seg;
    seg.start = Vec3d(5, 5, 14);
    seg.end   = Vec3d(45, 5, 14);
    seg.toolId = 0;
    pathSegments.push_back(seg);

    seg.start = Vec3d(45, 5, 14);
    seg.end   = Vec3d(45, 45, 14);
    pathSegments.push_back(seg);

    seg.start = Vec3d(45, 45, 14);
    seg.end   = Vec3d(5, 45, 14);
    pathSegments.push_back(seg);

    seg.start = Vec3d(5, 45, 14);
    seg.end   = Vec3d(5, 5, 14);
    pathSegments.push_back(seg);

    totalSegments = (int)pathSegments.size();
    currentSegment = 0;
    segmentProgress = 0.0f;
    simTime = 0.0;
    currentToolId = 0;
    addLog("Build", "Demo IT-1 loaded: 50x50x20 Box, BallEnd R=5, 4-segment path");
}

void AppState::startSimulation() {
    if (simState == IDLE) {
        simState = RUNNING;
        cancelled = false;
        currentSegment = 0;
        segmentProgress = 0.0f;
        simTime = 0.0;
        addLog("Info", "Simulation started");
    } else if (simState == PAUSED) {
        simState = RUNNING;
        addLog("Info", "Simulation resumed");
    }
}

void AppState::pauseSimulation() {
    if (simState == RUNNING) {
        simState = PAUSED;
        addLog("Info", "Simulation paused");
    }
}

void AppState::stopSimulation() {
    simState = IDLE;
    currentSegment = 0;
    segmentProgress = 0.0f;
    simTime = 0.0;
    cancelled = true;
    addLog("Info", "Simulation stopped");
}

void AppState::stepForward() {
    if (currentSegment < totalSegments) {
        segmentProgress = 0.0f;
        ++currentSegment;
        if (currentSegment >= totalSegments) {
            simState = IDLE;
            addLog("Info", "Step: reached end of path");
        } else {
            addLog("Info", std::string("Step: segment ") + std::to_string(currentSegment + 1));
        }
    }
}

Vec3d AppState::getCurrentToolPosition() const {
    if (pathSegments.empty() || currentSegment >= (int)pathSegments.size())
        return Vec3d(0, 0, 0);
    auto& seg = pathSegments[currentSegment];
    double t = (double)segmentProgress;
    return Vec3d(
        seg.start[0] + (seg.end[0] - seg.start[0]) * t,
        seg.start[1] + (seg.end[1] - seg.start[1]) * t,
        seg.start[2] + (seg.end[2] - seg.start[2]) * t
    );
}

void AppState::tick(float dt) {
    if (simState != RUNNING) return;
    if (pathSegments.empty() || currentSegment >= totalSegments) {
        simState = IDLE;
        return;
    }

    simTime += dt;

    auto& seg = pathSegments[currentSegment];
    double dx = seg.end[0] - seg.start[0];
    double dy = seg.end[1] - seg.start[1];
    double dz = seg.end[2] - seg.start[2];
    double segLen = sqrt(dx*dx + dy*dy + dz*dz);

    if (segLen < 1e-6) {
        segmentProgress = 1.0f;
    } else {
        float speed = cutSpeed * speedMultiplier;
        segmentProgress += (speed * dt) / (float)segLen;
    }

    if (segmentProgress >= 1.0f) {
        segmentProgress = 0.0f;
        ++currentSegment;
        ipwDirty = true;  // 换段时标记 IPW 需要更新

        if (currentSegment >= totalSegments) {
            simState = IDLE;
            currentSegment = 0;
            addLog("Info", std::string("Path complete (") + std::to_string(totalSegments) + " segments, " + std::to_string((int)simTime) + "s)");
        } else if (stepMode) {
            simState = PAUSED;
            addLog("Info", std::string("Segment ") + std::to_string(currentSegment) + " done (step mode)");
        }
    }
}

} // namespace midgard
