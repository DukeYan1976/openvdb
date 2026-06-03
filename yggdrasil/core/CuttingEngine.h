#pragma once

#include "types/YggTypes.h"
#include "core/ToolSweepSDF.h"

namespace ygg {

class CuttingEngine {
public:
    enum Strategy { STRAT_A = 0, STRAT_B, STRAT_C, STRAT_D };
    Strategy strategy = STRAT_D;  // 默认并行

    void cut(BilletModel& billet, const ToolSweepSDF& toolSDF);
};

/// 计算 Level Set Grid 的内部体积
double computeVolume(const openvdb::FloatGrid::Ptr& grid);

} // namespace ygg
