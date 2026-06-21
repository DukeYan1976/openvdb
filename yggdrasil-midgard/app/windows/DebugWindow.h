#pragma once

namespace midgard {

class DebugWindow {
public:
    void draw();

private:
    void drawTabs();
    void drawTabBillet();
    void drawTabTool();
    void drawTabSimulation();
    void drawStats();

    int activeTab_ = 0;
};

} // namespace midgard
