#include "core/ToolSweepSurface.h"
#include "core/Types.h"
#include <fstream>
#include <cmath>
using namespace midgard;
int main() {
    ToolDef tool{ToolType::BALL_END, 5.0, 0.0, 20.0};
    MoveSegment seg{Vec3d(0,10,18), Vec3d(20,10,18)};
    ToolSweepSurface surf(tool, seg);
    
    std::ofstream out("sweep_surface.obj");
    int N = 50;
    // 输出顶点网格
    for (int i = 0; i <= N; ++i) {
        for (int j = 0; j <= N; ++j) {
            double u = (double)i / N;
            double v = (double)j / N;
            Vec3d p = surf.eval(u, v);
            out << "v " << p.x() << " " << p.y() << " " << p.z() << "\n";
        }
    }
    // 输出面
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            int a = i*(N+1)+j+1;
            int b = a+1;
            int c = a+(N+1);
            int d = c+1;
            out << "f " << a << " " << b << " " << d << " " << c << "\n";
        }
    }
    out.close();
    std::cout << "Exported sweep_surface.obj (" << (N+1)*(N+1) << " verts, " << N*N << " faces)\n";
    std::cout << "vSplit=" << surf.vSplit() << " hasMid=" << surf.hasMidPatch() << "\n";
}
