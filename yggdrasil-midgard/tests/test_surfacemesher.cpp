#include <gtest/gtest.h>
#include "core/SurfaceMesher.h"
#include <cmath>
#include <vector>

using namespace midgard;

// ─────────────────────────────────────────────────────────────────
// Helper: compute max vertex deviation from a plane z = z0
// ─────────────────────────────────────────────────────────────────
static double maxPlaneDeviation(const TriMesh& mesh, double z0) {
    double maxDev = 0.0;
    for (int i = 0; i < mesh.vertexCount(); ++i) {
        double z = mesh.vertices[i * 6 + 2];
        maxDev = std::max(maxDev, std::abs(z - z0));
    }
    return maxDev;
}

// ─────────────────────────────────────────────────────────────────
// 1. Plane: dense sampling, coarse resolution (point spacing < voxel size)
//    This is the regime that previously produced scattered triangles.
// ─────────────────────────────────────────────────────────────────
TEST(SurfaceMesher, PlaneDenseCoarseResolution) {
    const double size = 10.0;
    const double spacing = 0.05;
    const int n = static_cast<int>(size / spacing) + 1;
    std::vector<double> pos, nrm;
    pos.reserve(n * n * 3);
    nrm.reserve(n * n * 3);

    for (int iy = 0; iy < n; ++iy) {
        for (int ix = 0; ix < n; ++ix) {
            double x = ix * spacing;
            double y = iy * spacing;
            pos.push_back(x);
            pos.push_back(y);
            pos.push_back(0.0);
            nrm.push_back(0.0);
            nrm.push_back(0.0);
            nrm.push_back(1.0);
        }
    }

    TriMesh mesh = buildSurfaceMesh(pos.data(), nrm.data(), pos.size() / 3, 128);
    EXPECT_FALSE(mesh.empty());
    EXPECT_GT(mesh.triangleCount(), 0);

    // All vertices should lie very close to the plane z = 0.
    double maxDev = maxPlaneDeviation(mesh, 0.0);
    EXPECT_LT(maxDev, 0.05) << "max plane deviation = " << maxDev;

    // Normals should point roughly +Z.
    int badNormals = 0;
    for (int i = 0; i < mesh.vertexCount(); ++i) {
        double nz = mesh.vertices[i * 6 + 5];
        if (nz < 0.5) ++badNormals;
    }
    EXPECT_EQ(badNormals, 0) << "found " << badNormals << " inverted normals";
}

// ─────────────────────────────────────────────────────────────────
// 2. Sphere: check vertices lie near the sphere surface and normals
//    point outward.
// ─────────────────────────────────────────────────────────────────
TEST(SurfaceMesher, Sphere) {
    const double radius = 5.0;
    const int nTheta = 80;
    const int nPhi = 80;
    std::vector<double> pos, nrm;
    pos.reserve(nTheta * nPhi * 3);
    nrm.reserve(nTheta * nPhi * 3);

    for (int it = 0; it < nTheta; ++it) {
        double theta = M_PI * it / (nTheta - 1);
        for (int ip = 0; ip < nPhi; ++ip) {
            double phi = 2.0 * M_PI * ip / (nPhi - 1);
            double x = radius * std::sin(theta) * std::cos(phi);
            double y = radius * std::sin(theta) * std::sin(phi);
            double z = radius * std::cos(theta);
            pos.push_back(x);
            pos.push_back(y);
            pos.push_back(z);
            nrm.push_back(x / radius);
            nrm.push_back(y / radius);
            nrm.push_back(z / radius);
        }
    }

    TriMesh mesh = buildSurfaceMesh(pos.data(), nrm.data(), pos.size() / 3, 128);
    EXPECT_FALSE(mesh.empty());
    EXPECT_GT(mesh.triangleCount(), 0);

    double maxRadialError = 0.0;
    for (int i = 0; i < mesh.vertexCount(); ++i) {
        double x = mesh.vertices[i * 6 + 0];
        double y = mesh.vertices[i * 6 + 1];
        double z = mesh.vertices[i * 6 + 2];
        double r = std::sqrt(x * x + y * y + z * z);
        maxRadialError = std::max(maxRadialError, std::abs(r - radius));
    }
    EXPECT_LT(maxRadialError, 0.2) << "max radial error = " << maxRadialError;

    int badNormals = 0;
    for (int i = 0; i < mesh.vertexCount(); ++i) {
        double x = mesh.vertices[i * 6 + 0];
        double y = mesh.vertices[i * 6 + 1];
        double z = mesh.vertices[i * 6 + 2];
        double nx = mesh.vertices[i * 6 + 3];
        double ny = mesh.vertices[i * 6 + 4];
        double nz = mesh.vertices[i * 6 + 5];
        double r = std::sqrt(x * x + y * y + z * z);
        if (r < 1e-6) continue;
        double dot = (x * nx + y * ny + z * nz) / r;
        if (dot < 0.7) ++badNormals;
    }
    EXPECT_EQ(badNormals, 0) << "found " << badNormals << " inward normals";
}

// ─────────────────────────────────────────────────────────────────
// 3. Density sweep: varying point spacing vs voxel size should still
//    produce a connected, non-degenerate mesh.
// ─────────────────────────────────────────────────────────────────
TEST(SurfaceMesher, PlaneDensitySweep) {
    const double size = 10.0;
    for (double spacing : {0.02, 0.05, 0.1, 0.2}) {
        int n = static_cast<int>(size / spacing) + 1;
        std::vector<double> pos, nrm;
        for (int iy = 0; iy < n; ++iy) {
            for (int ix = 0; ix < n; ++ix) {
                pos.push_back(ix * spacing);
                pos.push_back(iy * spacing);
                pos.push_back(0.0);
                nrm.push_back(0.0);
                nrm.push_back(0.0);
                nrm.push_back(1.0);
            }
        }
        TriMesh mesh = buildSurfaceMesh(pos.data(), nrm.data(), pos.size() / 3, 128);
        EXPECT_FALSE(mesh.empty()) << "spacing = " << spacing;
        EXPECT_GT(mesh.triangleCount(), 0) << "spacing = " << spacing;
        EXPECT_LT(maxPlaneDeviation(mesh, 0.0), 0.05) << "spacing = " << spacing;
    }
}

// ─────────────────────────────────────────────────────────────────
// 4. Tiny input should return empty mesh without crashing.
// ─────────────────────────────────────────────────────────────────
TEST(SurfaceMesher, EmptyAndTinyInput) {
    EXPECT_TRUE(buildSurfaceMesh(nullptr, nullptr, 0).empty());
    double p[3] = {0.0, 0.0, 0.0};
    double n[3] = {0.0, 0.0, 1.0};
    EXPECT_TRUE(buildSurfaceMesh(p, n, 1).empty());
    EXPECT_TRUE(buildSurfaceMesh(p, n, 2).empty());
}
