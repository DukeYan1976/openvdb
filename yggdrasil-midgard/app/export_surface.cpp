#include "core/ToolSweepSurface.h"
#include "core/ToolSweptSDF.h"
#include "core/Types.h"
#include <fstream>
#include <cmath>
#include <iostream>
#include <sys/stat.h>
#include <iomanip>

using namespace midgard;

void exportHighRes(const ToolDef& tool, const MoveSegment& seg, const std::string& name) {
    ToolSweepSurface surf(tool, seg);
    ToolSweptSDF sdf(tool, seg);

    const int N = 200; // 提高分辨率到 200x200
    std::string path = "obj_output/" + name + ".obj";
    std::ofstream out(path);
    if (!out) {
        std::cerr << "Failed to open " << path << " (ensure obj_output dir exists)\n";
        return;
    }

    double maxSdfErr = 0;

    // 写入顶点
    for (int i = 0; i <= N; ++i) {
        for (int j = 0; j <= N; ++j) {
            Vec3d p = surf.eval(i / (double)N, j / (double)N);
            out << "v " << p.x() << " " << p.y() << " " << p.z() << "\n";

            // 验证 SDF
            double s = sdf.eval(p);
            maxSdfErr = std::max(maxSdfErr, std::abs(s));
        }
    }
    // 写入面
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            int a = i * (N + 1) + j + 1;
            int b = i * (N + 1) + (j + 1) + 1;
            int c = (i + 1) * (N + 1) + (j + 1) + 1;
            int d = (i + 1) * (N + 1) + j + 1;
            out << "f " << a << " " << b << " " << c << " " << d << "\n";
        }
    }
    out.close();
    std::cout << "Exported " << name << ".obj (" << (N+1)*(N+1) << " verts) | "
              << "Max SDF Error: " << std::scientific << std::setprecision(4) << maxSdfErr << "\n";
}

int main() {
    mkdir("obj_output", 0777);
    MoveSegment seg{Vec3d(0, 10, 10), Vec3d(20, 10, 10)};

    // 1. 球头刀
    ToolDef ball{ToolType::BALL_END, 5.0, 0.0, 20.0};
    exportHighRes(ball, seg, "surface_ball_end");

    // 2. 牛鼻刀 (R=5, r=2)
    ToolDef bull{ToolType::BULL_NOSE, 5.0, 2.0, 20.0};
    exportHighRes(bull, seg, "surface_bull_nose");

    // 3. 平底刀
    ToolDef flat{ToolType::FLAT_END, 5.0, 0.0, 20.0};
    exportHighRes(flat, seg, "surface_flat_end");

    return 0;
}
