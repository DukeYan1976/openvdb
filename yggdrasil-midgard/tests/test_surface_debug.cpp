#include <gtest/gtest.h>
#include "core/ToolSweepSurface.h"
#include "core/ToolSweepSDF.h"
using namespace midgard;

TEST(SurfaceDebug, EvalMidCheck) {
    ToolDef tool{ToolType::BALL_END, 5.0, 0.0, 20.0};
    MoveSegment seg{Vec3d(0,10,18), Vec3d(20,10,18)};
    ToolSweepSurface surf(tool, seg);
    ToolSweepSDF sdf(tool, seg);

    printf("vSplit=%.4f\n", surf.vSplit());
    printf("evalMid at t=0.5 (path center), varying u:\n");
    for (double u : {0.0, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875, 1.0}) {
        Vec3d p = surf.evalMid(u, 0.5);
        double s = sdf.eval(p);
        printf("  u=%.3f -> (%.2f, %.2f, %.2f) SDF=%.6f\n", u, p.x(), p.y(), p.z(), s);
    }
    printf("unified eval(u, v) at v=0.3 (in mid range):\n");
    for (double u : {0.0, 0.25, 0.5, 0.75}) {
        Vec3d p = surf.eval(u, 0.3);
        printf("  u=%.2f v=0.3 -> (%.2f, %.2f, %.2f)\n", u, p.x(), p.y(), p.z());
    }
}
