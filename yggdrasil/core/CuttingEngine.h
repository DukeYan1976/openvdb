#pragma once

#include "types/YggTypes.h"
#include "core/ToolSweepSDF.h"

namespace ygg {

class CuttingEngine {
public:
    void cut(BilletModel& billet, const ToolSweepSDF& toolSDF);
};

/// 计算 Level Set Grid 的内部体积
double computeVolume(const openvdb::FloatGrid::Ptr& grid);

} // namespace ygg
