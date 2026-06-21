#pragma once

namespace midgard {

class OutputWindow {
public:
    void draw();

private:
    void drawLogStream();
    void drawStats();

    bool autoScroll_ = true;
};

} // namespace midgard