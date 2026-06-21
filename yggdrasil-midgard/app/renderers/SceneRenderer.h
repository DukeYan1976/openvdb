#pragma once
#include "Camera.h"
#include "GPURenderers.h"
#include "ShaderProgram.h"
#include "AppState.h"
#include <openvdb/openvdb.h>

namespace midgard {

class SceneRenderer {
public:
    void init();
    void render(const Camera& camera, int w, int h);
    void cleanup();

    void updateBillet(const Vec3d& origin, const Vec3d& dims);
    void rebuildBillet(const GeometryDef& bd);

    /// MacroGrid → mesh 化并上传 GPU
    void rebuildMacroMesh(const openvdb::FloatGrid::Ptr& grid);
    /// 清除 macro mesh
    void clearMacroMesh();
    bool hasMacroMesh() const { return macroMeshActive_; }

private:
    void renderBillet(const float mvp[16], const float nm[9]);
    void renderMacroMesh(const float mvp[16], const float nm[9]);
    void renderTool(const float mvp[16], const float nm[9]);
    void renderPath(const float mvp[16]);

    static void buildToolModel(const Vec3d& pos, const Vec3d& axis, float M[16]);

    GPUMesh billetMesh_, toolMesh_, macroMesh_;
    GPULines billetWire_, toolWire_, pathLines_;
    ShaderProgram meshShader_, lineShader_;
    bool billetDirty_ = true;
    bool toolDirty_ = true;
    bool macroMeshActive_ = false;
};

} // namespace midgard