#pragma once
#include "AppState.h"

namespace midgard {

class SettingsWindow {
public:
    void draw();

private:
    void drawPathSection();
    void drawToolLibrarySection();
    void drawBilletSection();
};

} // namespace midgard
