#include "core/MeshExport.h"
#include <openvdb/tools/VolumeToMesh.h>
#include <openvdb/points/PointAttribute.h>
#include <fstream>
#include <cmath>

namespace midgard {

void exportMacroMesh(const openvdb::FloatGrid::Ptr& grid, const std::string& filename) {
    std::vector<openvdb::Vec3s> points;
    std::vector<openvdb::Vec3I> triangles;
    std::vector<openvdb::Vec4I> quads;

    openvdb::tools::volumeToMesh(*grid, points, triangles, quads, 0.0, 0.5);

    std::ofstream out(filename);
    out << "# MacroGrid mesh: " << points.size() << " vertices\n";

    // 顶点
    for (const auto& p : points) {
        out << "v " << p.x() << " " << p.y() << " " << p.z() << "\n";
    }

    // 三角面 (OpenVDB volumeToMesh 输出面法线已朝外，保持原始顺序)
    for (const auto& t : triangles) {
        out << "f " << (t[0]+1) << " " << (t[1]+1) << " " << (t[2]+1) << "\n";
    }
    // 四边面转两个三角形
    for (const auto& q : quads) {
        out << "f " << (q[0]+1) << " " << (q[1]+1) << " " << (q[2]+1) << "\n";
        out << "f " << (q[0]+1) << " " << (q[2]+1) << " " << (q[3]+1) << "\n";
    }
}

void exportMicroPoints(const openvdb::points::PointDataGrid::Ptr& grid, const std::string& filename) {
    const auto& xform = grid->transform();
    std::vector<openvdb::Vec3d> positions;
    std::vector<openvdb::Vec3f> normals;

    for (auto leaf = grid->tree().cbeginLeaf(); leaf; ++leaf) {
        auto posHandle = openvdb::points::AttributeHandle<openvdb::Vec3f>::create(
            leaf->constAttributeArray("P"));
        bool hasNormal = leaf->hasAttribute("N");
        std::unique_ptr<openvdb::points::AttributeHandle<openvdb::Vec3f>> nrmHandle;
        if (hasNormal) {
            nrmHandle = std::make_unique<openvdb::points::AttributeHandle<openvdb::Vec3f>>(
                leaf->constAttributeArray("N"));
        }

        for (auto it = leaf->beginIndexOn(); it; ++it) {
            openvdb::Vec3f pos = posHandle->get(*it);
            openvdb::Vec3d wp = xform.indexToWorld(it.getCoord().asVec3d() + openvdb::Vec3d(pos));
            positions.push_back(wp);
            if (nrmHandle) normals.push_back(nrmHandle->get(*it));
            else normals.push_back(openvdb::Vec3f(0, 0, 1));
        }
    }

    std::ofstream out(filename);
    out << "# MicroGrid points and normal vectors: " << positions.size() << "\n";

    double vectorLen = 0.5; // 法线矢量长度 0.5mm

    // 写入所有点
    for (size_t i = 0; i < positions.size(); ++i) {
        const auto& p = positions[i];
        out << "v " << p.x() << " " << p.y() << " " << p.z() << "\n";
    }

    // 写入法线末端点 (用于绘制线段)
    for (size_t i = 0; i < positions.size(); ++i) {
        const auto& p = positions[i];
        const auto& n = normals[i];
        openvdb::Vec3d pEnd = p + openvdb::Vec3d(n) * vectorLen;
        out << "v " << pEnd.x() << " " << pEnd.y() << " " << pEnd.z() << "\n";
    }

    // 写入线段 (Normal Vectors)
    int nPts = (int)positions.size();
    for (int i = 0; i < nPts; ++i) {
        // 连接 p[i] 和 p[i + nPts]
        out << "l " << (i + 1) << " " << (i + nPts + 1) << "\n";
    }

    // 同时也保留顶点显示 (部分查看器需要独立点云)
    out << "p";
    for (int i = 1; i <= nPts; ++i) out << " " << i;
    out << "\n";
}

void exportMicroPLY(const openvdb::points::PointDataGrid::Ptr& grid, const std::string& filename) {
    const auto& xform = grid->transform();
    std::vector<openvdb::Vec3d> positions;
    std::vector<openvdb::Vec3f> normals;

    for (auto leaf = grid->tree().cbeginLeaf(); leaf; ++leaf) {
        auto posHandle = openvdb::points::AttributeHandle<openvdb::Vec3f>::create(
            leaf->constAttributeArray("P"));
        bool hasNormal = leaf->hasAttribute("N");
        std::unique_ptr<openvdb::points::AttributeHandle<openvdb::Vec3f>> nrmHandle;
        if (hasNormal) {
            nrmHandle = std::make_unique<openvdb::points::AttributeHandle<openvdb::Vec3f>>(
                leaf->constAttributeArray("N"));
        }

        for (auto it = leaf->beginIndexOn(); it; ++it) {
            openvdb::Vec3f pos = posHandle->get(*it);
            openvdb::Vec3d wp = xform.indexToWorld(it.getCoord().asVec3d() + openvdb::Vec3d(pos));
            positions.push_back(wp);
            normals.push_back(hasNormal ? nrmHandle->get(*it) : openvdb::Vec3f(0, 0, 1));
        }
    }

    std::ofstream out(filename);
    out << "ply\n"
        << "format ascii 1.0\n"
        << "element vertex " << positions.size() << "\n"
        << "property float x\n"
        << "property float y\n"
        << "property float z\n"
        << "property float nx\n"
        << "property float ny\n"
        << "property float nz\n"
        << "end_header\n";

    for (size_t i = 0; i < positions.size(); ++i) {
        out << positions[i].x() << " " << positions[i].y() << " " << positions[i].z() << " "
            << normals[i].x() << " " << normals[i].y() << " " << normals[i].z() << "\n";
    }
}

} // namespace midgard
