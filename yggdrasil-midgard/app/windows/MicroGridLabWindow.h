#pragma once

#include "core/MicroGridLab.h"
#include <string>

namespace midgard {

class MicroGridLabWindow {
public:
    void draw();

private:
    void drawParams();
    void drawToolSetup();
    void drawCutHistory();
    void drawActions();
    void logStats();
    void pushToViewport();

    MicroGridLabState state_;

    // 三档精度选项
    int precisionIndex_ = 1;  // 0=0.1, 1=0.01, 2=0.001
};

} // namespace midgard
