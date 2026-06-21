#include "AppState.h"
#include "windows/SettingsWindow.h"
#include "windows/SimControlWindow.h"
#include "windows/OutputWindow.h"
#include "windows/DebugWindow.h"
#include "windows/MicroGridLabWindow.h"
#include "renderers/SceneRenderer.h"
#include "renderers/Camera.h"
#include "renderers/GPURenderers.h"
#include "renderers/ShaderProgram.h"
#include "core/IDebugDisplay.h"
#include "core/IPWBuilder.h"
#include <cmath>

namespace {
// 从 GeometryDef 计算 world-space 包围盒 (origin, extent)
std::pair<midgard::Vec3d, midgard::Vec3d> billetBBox(const midgard::GeometryDef& gd) {
    if (gd.type == midgard::GeometryDef::CYLINDER) {
        return {
            midgard::Vec3d(gd.origin[0] - gd.radius, gd.origin[1] - gd.radius, gd.origin[2]),
            midgard::Vec3d(gd.radius * 2.0, gd.radius * 2.0, gd.height)
        };
    }
    return { gd.origin, gd.dims };
}
} // namespace
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <cstdio>

// g_debugDisplay defined in core/IDebugDisplay.cpp

// ═══════════════════════════════════════════════════════════════
class DebugDisplayImpl : public midgard::IDebugDisplay {
public:
    midgard::GPULines lines;
    midgard::GPUPoints points;
    midgard::GPUMesh triangles;
    midgard::ShaderProgram lineShader, pointShader, triShader;

    uint32_t lineColor_ = 0xFFFFFF00;
    uint32_t pointColor_ = 0xFF4488FF;
    uint32_t triColor_ = 0x33CC33CC;

    bool init() {
        lineShader.load(midgard::Shaders::lineVert, midgard::Shaders::lineFrag);
        pointShader.load(midgard::Shaders::pointVert, midgard::Shaders::pointFrag);
        triShader.load(midgard::Shaders::meshVert, midgard::Shaders::meshFrag);
        return lineShader.id && pointShader.id && triShader.id;
    }

    void drawLines(const float* data, size_t count, uint32_t color) override {
        lineColor_ = color;
        lines.upload(data, count * 3 * sizeof(float), count);
    }
    void drawPoints(const float* data, size_t count, uint32_t color) override {
        pointColor_ = color;
        points.upload(data, count * 3 * sizeof(float), count);
    }
    void drawTriangles(const float* verts, const uint32_t* indices, size_t triCount, uint32_t color) override {
        triColor_ = color;
        // 从索引中找出最大顶点编号来确定正确的顶点缓冲区大小
        int maxIdx = 0;
        for (size_t i = 0; i < triCount * 3; ++i)
            if (indices[i] > maxIdx) maxIdx = indices[i];
        int vertCount = maxIdx + 1;
        triangles.upload(verts, vertCount * 6 * sizeof(float),
                         indices, triCount * 3 * sizeof(uint32_t), triCount * 3);
    }
    void clear() override { lines.cleanup(); points.cleanup(); triangles.cleanup(); }

    static void unpackColor(uint32_t c, float& r, float& g, float& b, float& a) {
        a = ((c >> 24) & 0xFF) / 255.0f;
        r = ((c >> 16) & 0xFF) / 255.0f;
        g = ((c >>  8) & 0xFF) / 255.0f;
        b = ((c >>  0) & 0xFF) / 255.0f;
    }

    void render(const float mvp[16]) {
        if (lines.count > 0) {
            float r, g, b, a;
            unpackColor(lineColor_, r, g, b, a);
            lineShader.use(); lineShader.setMat4("uMVP", mvp);
            lineShader.setVec3("uColor", r, g, b); lineShader.setFloat("uAlpha", a);
            lines.draw();
        }
        if (triangles.count > 0) {
            glDepthMask(GL_FALSE); // 透明面不写深度，避免遮挡后面绘制的点
            float r, g, b, a;
            unpackColor(triColor_, r, g, b, a);
            triShader.use(); triShader.setMat4("uMVP", mvp);
            triShader.setMat3("uNormalMat", mvp);
            triShader.setVec3("uLightDir", 0,0,1);
            triShader.setVec3("uColor", r, g, b); triShader.setFloat("uAlpha", a);
            triangles.draw();
            glDepthMask(GL_TRUE);
        }
        if (points.count > 0) {
            float r, g, b, a;
            unpackColor(pointColor_, r, g, b, a);
            pointShader.use(); pointShader.setMat4("uMVP", mvp);
            pointShader.setVec3("uColor", r, g, b); pointShader.setFloat("uAlpha", a);
            pointShader.setFloat("uPointSize", 3); points.draw(3);
        }
    }
};

// ═══════════════════════════════════════════════════════════════
int main() {
    if (!glfwInit()) return -1;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    GLFWwindow* window = glfwCreateWindow(1280, 720, "Yggdrasil-Midgard", nullptr, nullptr);
    if (!window) { glfwTerminate(); return -1; }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        glfwTerminate(); return -1;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330 core");

    DebugDisplayImpl debugDisplay;
    debugDisplay.init();
    midgard::g_debugDisplay = &debugDisplay;

    midgard::Camera camera;
    midgard::SceneRenderer sceneRenderer;
    sceneRenderer.init();

    midgard::SettingsWindow  settingsWin;
    midgard::SimControlWindow simControlWin;
    midgard::OutputWindow    outputWin;
    midgard::DebugWindow     debugWin;
    midgard::MicroGridLabWindow microGridLabWin;

    // ── 启动时不加载任何几何 — 用户通过 Settings Load Demo 或 MicroGridLab Load 触发
    auto& st = midgard::getAppState();

    ImVec2 vpPos, vpSize;

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        int winW, winH, fbW, fbH;
        glfwGetWindowSize(window, &winW, &winH);
        glfwGetFramebufferSize(window, &fbW, &fbH);

        // 仿真推进
        st.tick(ImGui::GetIO().DeltaTime);

        // Viewport 全屏背景
        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(ImVec2((float)winW, (float)winH));
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
        ImGui::Begin("Viewport", nullptr,
            ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse |
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus |
            ImGuiWindowFlags_NoBackground);
        ImGui::PopStyleVar(3);

        vpPos  = ImGui::GetWindowPos();
        vpSize = ImGui::GetWindowSize();

        if (ImGui::IsWindowHovered()) {
            if (ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
                ImVec2 d = ImGui::GetMouseDragDelta(ImGuiMouseButton_Left);
                camera.orbit(d.x, -d.y);
                ImGui::ResetMouseDragDelta(ImGuiMouseButton_Left);
            }
            if (ImGui::IsMouseDragging(ImGuiMouseButton_Middle)) {
                ImVec2 d = ImGui::GetMouseDragDelta(ImGuiMouseButton_Middle);
                camera.pan(d.x, d.y);
                ImGui::ResetMouseDragDelta(ImGuiMouseButton_Middle);
            }
            float wh = ImGui::GetIO().MouseWheel;
            if (wh != 0) camera.zoom(wh);
        }

        if (ImGui::BeginPopupContextWindow("ViewMenu")) {
            if (ImGui::MenuItem("Zoom All")) {
                auto bb = billetBBox(st.billetDef);
                camera.zoomAll(bb.first, bb.second);
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Top"))           camera.viewTop();
            if (ImGui::MenuItem("Front"))         camera.viewFront();
            if (ImGui::MenuItem("Right"))         camera.viewRight();
            if (ImGui::MenuItem("Isometric"))     camera.viewIso();
            ImGui::EndPopup();
        }

        ImGui::End();

        // Settings 左上
        ImGui::SetNextWindowPos(ImVec2(10, 30), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(280, 400), ImGuiCond_FirstUseEver);
        settingsWin.draw();

        // SimControl 顶部中段
        ImGui::SetNextWindowPos(ImVec2(300, 30), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(600, 80), ImGuiCond_FirstUseEver);
        simControlWin.draw();

        // Output 右侧
        ImGui::SetNextWindowPos(ImVec2((float)winW - 310, 30), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(300, 400), ImGuiCond_FirstUseEver);
        outputWin.draw();

        // Debug 左下
        ImGui::SetNextWindowPos(ImVec2(10, (float)winH - 220), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(280, 200), ImGuiCond_FirstUseEver);
        debugWin.draw();

        // MicroGrid Lab 右下
        ImGui::SetNextWindowPos(ImVec2(300, 120), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(400, 500), ImGuiCond_FirstUseEver);
        microGridLabWin.draw();

        ImGui::Render();

        glViewport(0, 0, fbW, fbH);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (vpSize.x > 0 && vpSize.y > 0) {
            float sx = (float)fbW / (float)winW;
            float sy = (float)fbH / (float)winH;
            GLint x = (GLint)(vpPos.x * sx);
            GLint y = (GLint)(fbH - (vpPos.y + vpSize.y) * sy);
            GLsizei w = (GLsizei)(vpSize.x * sx);
            GLsizei h = (GLsizei)(vpSize.y * sy);
            if (w > 0 && h > 0) {
                glEnable(GL_SCISSOR_TEST);
                glScissor(x, y, w, h);
                glViewport(x, y, w, h);
                glEnable(GL_DEPTH_TEST);
                glDepthFunc(GL_LESS);
                glClear(GL_DEPTH_BUFFER_BIT);
                // ── Mesh + IPW 重建 (用户加载几何后才触发) ──
                if (st.geometryLoaded && st.ipwDirty) {
                    sceneRenderer.rebuildBillet(st.billetDef);

                    midgard::IPWBuilder builder;
                    st.ipw = builder.build(st.billetDef, st.ipw.config,
                        [&st](const char* level, const char* msg) { st.addLog(level, msg); },
                        st.ipwShowMacroGrid);
                    st.ipwDirty = false;
                    sceneRenderer.clearMacroMesh();
                }

                if (st.macroMeshRequested) {
                    sceneRenderer.rebuildMacroMesh(st.ipw.macroGrid);
                    st.macroMeshRequested = false;
                }
                if (st.macroMeshClear) {
                    sceneRenderer.clearMacroMesh();
                    st.macroMeshClear = false;
                }

                sceneRenderer.render(camera, w, h);

                float mvp[16], nm[9];
                camera.buildMVP(w, h, mvp, nm);
                glDepthFunc(GL_ALWAYS);   // debug voxel 不受任何深度遮挡
                debugDisplay.render(mvp);
                glDepthFunc(GL_LESS);
            }
        }

        glViewport(0, 0, fbW, fbH);
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_SCISSOR_TEST);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    sceneRenderer.cleanup();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
