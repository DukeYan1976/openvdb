#pragma once

namespace midgard {

class SimControlWindow {
public:
    void draw();

private:
    void drawControls();
    void drawProgress();
};

} // namespace midgard
