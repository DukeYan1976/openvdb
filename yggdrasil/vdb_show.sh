#!/bin/bash
# vdb_show.sh - 查看 VDB 文件（同时打开 OBJ mesh 和 vdb_view）
# 用法: ./vdb_show.sh <file.vdb>

set -e

if [ $# -lt 1 ]; then
    echo "Usage: $0 <file.vdb>"
    exit 1
fi

VDB_FILE="$1"
if [ ! -f "$VDB_FILE" ]; then
    echo "Error: $VDB_FILE not found"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
VDB_VIEW="$REPO_ROOT/build/openvdb_cmd/vdb_view/vdb_view"
VDB2OBJ="$SCRIPT_DIR/build/vdb2obj"
OBJ_FILE="/tmp/$(basename "${VDB_FILE%.vdb}").obj"

# 编译 vdb2obj（如果不存在）
if [ ! -f "$VDB2OBJ" ]; then
    echo "Building vdb2obj..."
    cat > /tmp/vdb2obj.cpp << 'EOF'
#include <openvdb/openvdb.h>
#include <openvdb/tools/VolumeToMesh.h>
#include <fstream>
#include <iostream>

int main(int argc, char* argv[]) {
    if (argc < 3) { std::cerr << "Usage: vdb2obj <in.vdb> <out.obj>\n"; return 1; }
    openvdb::initialize();
    openvdb::io::File file(argv[1]);
    file.open();
    auto grid = openvdb::gridPtrCast<openvdb::FloatGrid>(file.getGrids()->front());
    file.close();
    if (!grid) { std::cerr << "Error: no FloatGrid found\n"; return 1; }

    std::vector<openvdb::Vec3s> points;
    std::vector<openvdb::Vec3I> tris;
    std::vector<openvdb::Vec4I> quads;
    openvdb::tools::volumeToMesh(*grid, points, tris, quads, 0.0);

    std::ofstream obj(argv[2]);
    for (auto& p : points)
        obj << "v " << p.x() << " " << p.y() << " " << p.z() << "\n";
    for (auto& t : tris)
        obj << "f " << t[2]+1 << " " << t[1]+1 << " " << t[0]+1 << "\n";
    for (auto& q : quads)
        obj << "f " << q[3]+1 << " " << q[2]+1 << " " << q[1]+1 << " " << q[0]+1 << "\n";
    obj.close();
    std::cout << "Exported " << points.size() << " vertices, "
              << tris.size() + quads.size() << " faces\n";
    return 0;
}
EOF
    INSTALL_DIR="$REPO_ROOT/install"
    c++ -std=c++17 -O2 \
        -I/opt/homebrew/include \
        /tmp/vdb2obj.cpp \
        -L"$INSTALL_DIR/lib" -lopenvdb \
        -L/opt/homebrew/lib -ltbb \
        -Wl,-rpath,"$INSTALL_DIR/lib" \
        -Wl,-rpath,/opt/homebrew/lib \
        -o "$VDB2OBJ"
    echo "Built: $VDB2OBJ"
fi

# 转换为 OBJ
echo "Converting $VDB_FILE -> $OBJ_FILE"
"$VDB2OBJ" "$VDB_FILE" "$OBJ_FILE"

# 打开 OBJ 查看器
echo "Opening OBJ viewer..."
open "$OBJ_FILE"

# 打开 vdb_view
if [ -f "$VDB_VIEW" ]; then
    echo "Opening vdb_view..."
    "$VDB_VIEW" "$VDB_FILE" &
else
    echo "Warning: vdb_view not found at $VDB_VIEW"
fi

echo "Done."
