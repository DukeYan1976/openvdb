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

    // 先收集所有点和法线
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
            openvdb::Vec3d wp = xform.indexToWorld(
                it.getCoord().asVec3d() + openvdb::Vec3d(pos));
            positions.push_back(wp);

            if (nrmHandle) {
                normals.push_back(nrmHandle->get(*it));
            } else {
                normals.push_back(openvdb::Vec3f(0, 0, 1));
            }
        }
    }

    // 写OBJ: 顶点 → 法线 → 点引用(用小三角面片可视化)
    std::ofstream out(filename);
    out << "# MicroGrid points: " << positions.size() << "\n";

    // 所有顶点
    for (const auto& p : positions) {
        out << "v " << p.x() << " " << p.y() << " " << p.z() << "\n";
    }
    // 所有法线
    for (const auto& n : normals) {
        out << "vn " << n.x() << " " << n.y() << " " << n.z() << "\n";
    }
    // 每个点生成一个极小的三角面片（oriented disc）用于在软件中可见
    // 面片大小 = 0.3mm (适合20mm工件的可视化)
    double sz = 0.3;
    int baseIdx = positions.size();
    for (size_t i = 0; i < positions.size(); ++i) {
        openvdb::Vec3f n = normals[i];
        // 构造两个切向量
        openvdb::Vec3f t1 = (std::abs(n.x()) < 0.9f)
            ? openvdb::Vec3f(1,0,0).cross(n) : openvdb::Vec3f(0,1,0).cross(n);
        t1.normalize();
        openvdb::Vec3f t2 = n.cross(t1);

        openvdb::Vec3d p = positions[i];
        openvdb::Vec3d v1 = p + sz * openvdb::Vec3d(t1);
        openvdb::Vec3d v2 = p + sz * openvdb::Vec3d(t2);

        out << "v " << v1.x() << " " << v1.y() << " " << v1.z() << "\n";
        out << "v " << v2.x() << " " << v2.y() << " " << v2.z() << "\n";
    }
    // 面: 每个点(i) + 两个辅助顶点 → 三角形
    for (size_t i = 0; i < positions.size(); ++i) {
        int vi = i + 1;  // 1-indexed
        int va = baseIdx + i*2 + 1;
        int vb = baseIdx + i*2 + 2;
        int ni = i + 1;
        out << "f " << vi << "//" << ni << " " << va << "//" << ni << " " << vb << "//" << ni << "\n";
    }
}

} // namespace midgard
