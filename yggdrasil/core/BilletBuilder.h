#pragma once

#include "types/YggTypes.h"

namespace ygg {

/// 构建毛坯模型 (IPW₀)
BilletModel buildBillet(const ResolutionConfig& config,
                        const Vec3d& origin, const Vec3d& dims);

} // namespace ygg
