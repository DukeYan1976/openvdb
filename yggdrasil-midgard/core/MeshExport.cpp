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

    // 写OBJ: 点云 + 法线可视化面片
    std::ofstream out(filename);
    out << "# MicroGrid points: " << positions.size() << "\n";

    int n = (int)positions.size();
    double sz = 0.6;  // 面片大小 0.6mm

    // 为每个点生成3个顶点(oriented disc)
    for (size_t i = 0; i < positions.size(); ++i) {
        openvdb::Vec3d p = positions[i];
        openvdb::Vec3f nf = normals[i];
        openvdb::Vec3d nrm(nf.x(), nf.y(), nf.z());

        // 构造切平面上两个正交向量
        openvdb::Vec3d t1 = std::abs(nrm.x()) < 0.9
            ? openvdb::Vec3d(1,0,0).cross(nrm)
            : openvdb::Vec3d(0,1,0).cross(nrm);
        t1.normalize();
        openvdb::Vec3d t2 = nrm.cross(t1);

        // 3个顶点: 中心 + 两个切平面点
        out << "v " << p.x() << " " << p.y() << " " << p.z() << "\n";
        openvdb::Vec3d v1 = p + sz * t1;
        openvdb::Vec3d v2 = p + sz * t2;
        out << "v " << v1.x() << " " << v1.y() << " " << v1.z() << "\n";
        out << "v " << v2.x() << " " << v2.y() << " " << v2.z() << "\n";
    }

    // 法线
    for (const auto& n : normals) {
        out << "vn " << n.x() << " " << n.y() << " " << n.z() << "\n";
    }

    // 三角面（双面：正反各一）
    for (int i = 0; i < n; ++i) {
        int base = i * 3 + 1;  // 1-indexed
        int ni = i + 1;
        out << "f " << base << "//" << ni << " " << (base+1) << "//" << ni << " " << (base+2) << "//" << ni << "\n";
        out << "f " << base << "//" << ni << " " << (base+2) << "//" << ni << " " << (base+1) << "//" << ni << "\n";
    }
}

} // namespace midgard
