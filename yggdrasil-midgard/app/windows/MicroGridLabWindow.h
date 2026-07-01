#pragma once

#include "core/MicroGridLab.h"
#include <string>

namespace midgard {

class MicroGridLabWindow {
public:
    void draw();
    void pushToViewport();

private:
    void drawParams();
    void drawToolSetup();
    void drawCutHistory();
    void drawActions();
    void logStats();

    // 三档精度选项
    int precisionIndex_ = 1;  // 0=0.1, 1=0.01, 2=0.001
};

} // namespace midgard
