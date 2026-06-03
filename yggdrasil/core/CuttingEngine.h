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

/// 在刀具BBox范围内用d_v精度重建切削面SDF（用于精确显示）
openvdb::FloatGrid::Ptr buildLocalCutSurface(
    const BilletModel& billet, const ToolSweepSDF& lastTool);

} // namespace ygg
