#include "SceneRenderer.h"
#include "BilletMeshGenerator.h"
#include "ToolMeshGenerator.h"
#include "AppState.h"
#include <openvdb/tools/VolumeToMesh.h>
#include <cmath>
#include <cstdio>
#include <vector>
#include <cstring>

namespace midgard {

void SceneRenderer::init() {
    meshShader_.load(Shaders::meshVert, Shaders::meshFrag);
    lineShader_.load(Shaders::lineVert, Shaders::lineFrag);
}

// ─── 主渲染入口 ─────────────────────────────────────────────
void SceneRenderer::render(const Camera& camera, int w, int h) {
    auto& state = getAppState();
    if (!state.geometryLoaded) return;  // 未加载几何 → 空视口

    float mvp[16], nm[9];
    camera.buildMVP(w, h, mvp, nm);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // 1. 不透明物体先画（刀路线）— 写深度
    glDepthMask(GL_TRUE);
    renderPath(mvp);

    // 2. 半透明物体后画（毛坯、刀具）— 不写深度
    glDepthMask(GL_FALSE);
    if (macroMeshActive_)
        renderMacroMesh(mvp, nm);
    else
        renderBillet(mvp, nm);
    renderTool(mvp, nm);

    glDisable(GL_BLEND);
    glDepthMask(GL_TRUE);
}

// ─── 毛坯重建 (显式调用，保证与 IPW 同步) ─────────────────────
void SceneRenderer::rebuildBillet(const GeometryDef& bd) {
    std::vector<float> verts;
    std::vector<uint32_t> indices;

    if (bd.type == GeometryDef::BOX) {
        BilletMeshGenerator::buildBox(bd.origin, bd.dims, verts, indices);
    } else if (bd.type == GeometryDef::CYLINDER) {
        BilletMeshGenerator::buildCylinder(bd.origin, bd.radius, bd.height, verts, indices);
    }

    if (!verts.empty())
        billetMesh_.upload(verts.data(), verts.size()*4, indices.data(), indices.size()*4, (int)indices.size());

    std::vector<float> wires;
    if (bd.type == GeometryDef::BOX)
        BilletMeshGenerator::buildBoxWireframe(bd.origin, bd.dims, wires);
    else if (bd.type == GeometryDef::CYLINDER)
        BilletMeshGenerator::buildCylinderWireframe(bd.origin, bd.radius, bd.height, wires);
    if (!wires.empty())
        billetWire_.upload(wires.data(), wires.size()*4, (int)wires.size()/3);

    billetDirty_ = false;
}

// ─── 毛坯渲染 ───────────────────────────────────────────────
void SceneRenderer::renderBillet(const float mvp[16], const float nm[9]) {
    auto& state = getAppState();

    if (billetDirty_) {
        rebuildBillet(state.billetDef);
    }

    if (state.showBillet && billetMesh_.count > 0) {
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(1.0f, 1.0f);
        meshShader_.use();
        meshShader_.setMat4("uMVP", mvp);
        meshShader_.setMat3("uNormalMat", nm);
        meshShader_.setVec3("uLightDir", 0,0,1);
        meshShader_.setVec3("uColor", 0.55f, 0.6f, 0.68f);
        meshShader_.setFloat("uAlpha", state.billetAlpha);
        billetMesh_.draw();
        glDisable(GL_POLYGON_OFFSET_FILL);

        if (state.showBilletWire && billetWire_.count > 0) {
            lineShader_.use();
            lineShader_.setMat4("uMVP", mvp);
            lineShader_.setVec3("uColor", 1,1,0);
            lineShader_.setFloat("uAlpha", state.billetAlpha);
            billetWire_.draw();
        }
    }
}

// ─── 工具模型矩阵 ──────────────────────────────────────────
void SceneRenderer::buildToolModel(const Vec3d& pos, const Vec3d& axis, float M[16]) {
    std::memset(M, 0, 16*sizeof(float));
    M[0]=M[5]=M[10]=M[15]=1;

    M[12] = (float)pos[0];
    M[13] = (float)pos[1];
    M[14] = (float)pos[2];

    float ax = (float)axis[0], ay = (float)axis[1], az = (float)axis[2];
    float al = sqrtf(ax*ax + ay*ay + az*az);
    if (al < 1e-6f) { ax=0; ay=0; az=1; al=1; }
    ax /= al; ay /= al; az /= al;

    float dot = az;
    if (dot > 0.9999f) return;

    float rx = -ay, ry = ax, rz = 0;
    float rl = sqrtf(rx*rx + ry*ry);
    if (rl < 1e-6f) return;
    rx /= rl; ry /= rl;

    float angle = acosf(dot);
    float c = cosf(angle), s = sinf(angle), t = 1-c;

    float R[9] = {
        t*rx*rx + c,    t*rx*ry,        ry*s,
        t*rx*ry,        t*ry*ry + c,    -rx*s,
        -ry*s,          rx*s,           c
    };

    M[0]=R[0]; M[1]=R[1]; M[2]=R[2];
    M[4]=R[3]; M[5]=R[4]; M[6]=R[5];
    M[8]=R[6]; M[9]=R[7]; M[10]=R[8];
}

// ─── 工具渲染 ───────────────────────────────────────────────
void SceneRenderer::renderTool(const float vp[16], const float nm[9]) {
    auto& state = getAppState();
    if (!state.showTool) return;

    const ToolDef* tool = nullptr;
    for (auto& e : state.toolLibrary) {
        if (e.id == state.currentToolId) { tool = &e.def; break; }
    }
    if (!tool) return;
    if (state.pathSegments.empty()) return;

    auto toolPos = state.getCurrentToolPosition();
    auto& seg = state.pathSegments[state.currentSegment < (int)state.pathSegments.size()
                                   ? state.currentSegment : 0];

    if (toolDirty_ || state.ipwDirty) {
        std::vector<float> verts;
        std::vector<uint32_t> indices;

        switch (tool->type) {
        case ToolType::BALL_END:
            ToolMeshGenerator::buildBallEnd(tool->R, tool->H, verts, indices); break;
        case ToolType::FLAT_END:
            ToolMeshGenerator::buildFlatEnd(tool->R, tool->H, verts, indices); break;
        case ToolType::BULL_NOSE:
            ToolMeshGenerator::buildBullNose(tool->R, tool->r, tool->H, verts, indices); break;
        }

        if (!verts.empty())
            toolMesh_.upload(verts.data(), verts.size()*4, indices.data(), indices.size()*4, (int)indices.size());

        std::vector<float> wires;
        switch (tool->type) {
        case ToolType::BALL_END:
            ToolMeshGenerator::buildBallEndWireframe(tool->R, tool->H, wires); break;
        case ToolType::FLAT_END:
            ToolMeshGenerator::buildFlatEndWireframe(tool->R, tool->H, wires); break;
        case ToolType::BULL_NOSE:
            ToolMeshGenerator::buildBullNoseWireframe(tool->R, tool->r, tool->H, wires); break;
        }
        if (!wires.empty())
            toolWire_.upload(wires.data(), wires.size()*4, (int)wires.size()/3);

        toolDirty_ = false;
    }

    if (toolMesh_.count == 0) return;

    float M[16];
    buildToolModel(toolPos, seg.axis, M);

    float mvp[16] = {0};
    for (int col = 0; col < 4; ++col)
        for (int row = 0; row < 4; ++row)
            for (int k = 0; k < 4; ++k)
                mvp[row + col*4] += vp[row + k*4] * M[k + col*4];

    float tnm[9] = {
        nm[0]*M[0] + nm[1]*M[4] + nm[2]*M[8],
        nm[0]*M[1] + nm[1]*M[5] + nm[2]*M[9],
        nm[0]*M[2] + nm[1]*M[6] + nm[2]*M[10],
        nm[3]*M[0] + nm[4]*M[4] + nm[5]*M[8],
        nm[3]*M[1] + nm[4]*M[5] + nm[5]*M[9],
        nm[3]*M[2] + nm[4]*M[6] + nm[5]*M[10],
        nm[6]*M[0] + nm[7]*M[4] + nm[8]*M[8],
        nm[6]*M[1] + nm[7]*M[5] + nm[8]*M[9],
        nm[6]*M[2] + nm[7]*M[6] + nm[8]*M[10],
    };

    meshShader_.use();
    meshShader_.setMat4("uMVP", mvp);
    meshShader_.setMat3("uNormalMat", tnm);
    meshShader_.setVec3("uLightDir", 0,0,1);
    meshShader_.setVec3("uColor", 1.0f, 0.55f, 0.0f);
    meshShader_.setFloat("uAlpha", 0.6f);
    toolMesh_.draw();

    if (toolWire_.count > 0) {
        lineShader_.use();
        lineShader_.setMat4("uMVP", mvp);
        lineShader_.setVec3("uColor", 1, 0.7f, 0);
        lineShader_.setFloat("uAlpha", 1);
        toolWire_.draw();
    }
}

// ─── 刀路渲染 ───────────────────────────────────────────────
void SceneRenderer::renderPath(const float vp[16]) {
    auto& state = getAppState();
    if (!state.showToolPath || state.pathSegments.empty()) return;

    std::vector<float> lines;
    int n = (int)state.pathSegments.size();

    for (int i = 0; i < n; ++i) {
        auto& seg = state.pathSegments[i];
        lines.insert(lines.end(), {
            (float)seg.start[0], (float)seg.start[1], (float)seg.start[2],
            (float)seg.end[0],   (float)seg.end[1],   (float)seg.end[2]});

        float dx = (float)(seg.end[0] - seg.start[0]);
        float dy = (float)(seg.end[1] - seg.start[1]);
        float dz = (float)(seg.end[2] - seg.start[2]);
        float len = sqrtf(dx*dx + dy*dy + dz*dz);
        if (len > 1e-6f) {
            dx /= len; dy /= len; dz /= len;
            float s = fminf(len * 0.15f, 3.0f);
            float ax = (float)seg.end[0] - dx * s;
            float ay = (float)seg.end[1] - dy * s;
            float az = (float)seg.end[2] - dz * s;
            lines.insert(lines.end(), {ax, ay, az,
                                       (float)seg.end[0], (float)seg.end[1], (float)seg.end[2]});
        }
    }

    pathLines_.upload(lines.data(), lines.size() * 4, (int)lines.size() / 3);
    if (pathLines_.count == 0) return;

    lineShader_.use();
    lineShader_.setMat4("uMVP", vp);
    lineShader_.setFloat("uAlpha", 1.0f);
    glBindVertexArray(pathLines_.vao);

    int cur = state.currentSegment;
    int segVerts = 4;
    for (int i = 0; i < n; ++i) {
        if (i < cur)
            lineShader_.setVec3("uColor", 0.4f, 0.4f, 0.5f);
        else if (i == cur)
            lineShader_.setVec3("uColor", 0.0f, 1.0f, 0.0f);
        else
            lineShader_.setVec3("uColor", 0.6f, 0.6f, 0.7f);
        glDrawArrays(GL_LINES, i * segVerts, segVerts);
    }
}

void SceneRenderer::cleanup() {
    billetMesh_.cleanup();
    billetWire_.cleanup();
    toolMesh_.cleanup();
    toolWire_.cleanup();
    pathLines_.cleanup();
    macroMesh_.cleanup();
    meshShader_.cleanup();
    lineShader_.cleanup();
}

// ─── MacroGrid Mesh ─────────────────────────────────────────
void SceneRenderer::rebuildMacroMesh(const openvdb::FloatGrid::Ptr& grid) {
    if (!grid) return;

    std::vector<openvdb::Vec3s> points;
    std::vector<openvdb::Vec3I> triangles;
    std::vector<openvdb::Vec4I> quads;
    openvdb::tools::volumeToMesh(*grid, points, triangles, quads, 0.0, 0.5);

    // 转为 interleaved (pos, normal) — flat shading per face
    std::vector<float> verts;
    std::vector<uint32_t> indices;

    auto addTri = [&](const openvdb::Vec3s& a, const openvdb::Vec3s& b, const openvdb::Vec3s& c) {
        openvdb::Vec3s n = (b - a).cross(c - a);
        float nl = n.length();
        if (nl > 1e-8f) n /= nl; else n = openvdb::Vec3s(0,0,1);

        uint32_t base = (uint32_t)(verts.size() / 6);
        for (auto* p : {&a, &b, &c}) {
            verts.insert(verts.end(), {p->x(), p->y(), p->z(), n.x(), n.y(), n.z()});
        }
        indices.insert(indices.end(), {base, base+1, base+2});
    };

    for (auto& t : triangles)
        addTri(points[t[0]], points[t[1]], points[t[2]]);
    for (auto& q : quads) {
        addTri(points[q[0]], points[q[1]], points[q[2]]);
        addTri(points[q[0]], points[q[2]], points[q[3]]);
    }

    if (!verts.empty())
        macroMesh_.upload(verts.data(), verts.size()*4, indices.data(), indices.size()*4, (int)indices.size());

    macroMeshActive_ = true;
}

void SceneRenderer::clearMacroMesh() {
    macroMesh_.cleanup();
    macroMeshActive_ = false;
}

void SceneRenderer::renderMacroMesh(const float mvp[16], const float nm[9]) {
    if (macroMesh_.count == 0) return;

    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.0f, 1.0f);
    meshShader_.use();
    meshShader_.setMat4("uMVP", mvp);
    meshShader_.setMat3("uNormalMat", nm);
    meshShader_.setVec3("uLightDir", 0, 0, 1);
    meshShader_.setVec3("uColor", 0.7f, 0.75f, 0.8f);
    meshShader_.setFloat("uAlpha", 0.85f);
    macroMesh_.draw();
    glDisable(GL_POLYGON_OFFSET_FILL);
}

void SceneRenderer::updateBillet(const Vec3d& origin, const Vec3d& dims) {
    auto& state = getAppState();
    state.billetDef.origin = origin;
    state.billetDef.dims = dims;
    state.ipwDirty = true;
    billetDirty_ = true;
}

} // namespace midgard