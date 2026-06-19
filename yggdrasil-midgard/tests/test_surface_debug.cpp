#include <gtest/gtest.h>
#include "core/ToolSweepSurface.h"
#include "core/ToolSweptSDF.h"
using namespace midgard;

TEST(SurfaceDebug, EvalMidCheck) {
    ToolDef tool{ToolType::BALL_END, 5.0, 0.0, 20.0};
    MoveSegment seg{Vec3d(0,10,18), Vec3d(20,10,18)};
    ToolSweepSurface surf(tool, seg);
    ToolSweptSDF sdf(tool, seg);

    printf("v1=%.4f, v2=%.4f\n", surf.v1(), surf.v2());
    printf("evalMid at path center, varying u:\n");
    for (double u : {0.0, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875, 1.0}) {
        // v = v1 * 0.5 对应 Left Mid 的 t=0.5
        double v = surf.v1() * 0.5;
        Vec3d p = surf.eval(u, v);
        double s = sdf.eval(p);
        printf("  u=%.3f -> (%.2f, %.2f, %.2f) SDF=%.6f\n", u, p.x(), p.y(), p.z(), s);
    }
    printf("unified eval(u, v) at v=0.3 (in mid range):\n");
    for (double u : {0.0, 0.25, 0.5, 0.75}) {
        Vec3d p = surf.eval(u, 0.3);
        printf("  u=%.2f v=0.3 -> (%.2f, %.2f, %.2f)\n", u, p.x(), p.y(), p.z());
    }
}
