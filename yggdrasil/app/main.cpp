#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#if YGG_HAS_OPENVDB
#include "VDBToMesh.h"
#include "core/ResolutionSolver.h"
#include "core/BilletBuilder.h"
#include "core/ToolSweepSDF.h"
#include "core/CuttingEngine.h"
#include <openvdb/points/PointCount.h>
#include <openvdb/points/PointAttribute.h>
#include <openvdb/tools/LevelSetSphere.h>
#endif

#include <cstdio>
#include <cmath>
#include <vector>
#include <cstdint>

// --- Shader sources ---
static const char* vertSrc = R"(
#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNorm;
uniform mat4 uMVP;
uniform mat3 uNormalMat;
uniform vec4 uClipPlane;
out vec3 vNorm;
out float vClipDist;
void main(){
    gl_Position = uMVP * vec4(aPos, 1.0);
    vNorm = normalize(uNormalMat * aNorm);
    vClipDist = dot(aPos, uClipPlane.xyz) + uClipPlane.w;
    gl_ClipDistance[0] = vClipDist;
    gl_PointSize = 4.0;
})";

static const char* fragSrc = R"(
#version 330 core
in vec3 vNorm;
out vec4 fragColor;
uniform vec3 uLightDir;
uniform vec3 uColor;
void main(){
    float diff = abs(dot(vNorm, uLightDir)) * 0.7 + 0.3;
    fragColor = vec4(uColor * diff, 1.0);
})";

// --- GL state ---
static GLuint g_prog = 0, g_VAO = 0, g_VBO = 0, g_EBO = 0;
static int g_idxCount = 0;

static void initGL() {
    GLuint vs = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vs, 1, &vertSrc, nullptr); glCompileShader(vs);
    GLint ok; glGetShaderiv(vs, GL_COMPILE_STATUS, &ok);
    if(!ok){char log[512];glGetShaderInfoLog(vs,512,nullptr,log);printf("VS err: %s\n",log);}
    GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fs, 1, &fragSrc, nullptr); glCompileShader(fs);
    glGetShaderiv(fs, GL_COMPILE_STATUS, &ok);
    if(!ok){char log[512];glGetShaderInfoLog(fs,512,nullptr,log);printf("FS err: %s\n",log);}
    g_prog = glCreateProgram();
    glAttachShader(g_prog, vs); glAttachShader(g_prog, fs);
    glLinkProgram(g_prog);
    glGetProgramiv(g_prog, GL_LINK_STATUS, &ok);
    if(!ok){char log[512];glGetProgramInfoLog(g_prog,512,nullptr,log);printf("Link err: %s\n",log);}
    glDeleteShader(vs); glDeleteShader(fs);
    glGenVertexArrays(1, &g_VAO);
    glGenBuffers(1, &g_VBO);
    glGenBuffers(1, &g_EBO);
}

static void uploadMesh(const float* verts, size_t vertBytes,
                       const uint32_t* idx, size_t idxBytes, int count) {
    glBindVertexArray(g_VAO);
    glBindBuffer(GL_ARRAY_BUFFER, g_VBO);
    glBufferData(GL_ARRAY_BUFFER, vertBytes, verts, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, g_EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, idxBytes, idx, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    glEnableVertexAttribArray(1);
    g_idxCount = count;
}

// --- Camera ---
struct Camera { float dist=60, yaw=45, pitch=30, tx=15, ty=15, tz=5; };

static void buildMVP(const Camera& c, int w, int h, float mvp[16], float nm[9]) {
    // Perspective projection (column-major)
    float asp = (float)w/(float)h;
    float fov = 45.0f * 3.14159f / 180.0f;
    float zNear = 0.1f, zFar = 500.0f;
    float t = tanf(fov * 0.5f);
    float P[16] = {0};
    P[0]  = 1.0f / (asp * t);
    P[5]  = 1.0f / t;
    P[10] = -(zFar + zNear) / (zFar - zNear);
    P[11] = -1.0f;
    P[14] = -2.0f * zFar * zNear / (zFar - zNear);

    // Camera position
    float yr = c.yaw * 3.14159f / 180.0f;
    float pr = c.pitch * 3.14159f / 180.0f;
    float eyeX = c.tx + c.dist * cosf(pr) * cosf(yr);
    float eyeY = c.ty + c.dist * cosf(pr) * sinf(yr);
    float eyeZ = c.tz + c.dist * sinf(pr);

    // Forward (target - eye, normalized)
    float fwdX = c.tx - eyeX, fwdY = c.ty - eyeY, fwdZ = c.tz - eyeZ;
    float fl = sqrtf(fwdX*fwdX + fwdY*fwdY + fwdZ*fwdZ);
    fwdX /= fl; fwdY /= fl; fwdZ /= fl;

    // Right = forward x up(0,0,1)
    float rX = fwdY * 1.0f - fwdZ * 0.0f;
    float rY = fwdZ * 0.0f - fwdX * 1.0f;
    float rZ = fwdX * 0.0f - fwdY * 0.0f;
    float rl = sqrtf(rX*rX + rY*rY + rZ*rZ);
    if (rl < 1e-6f) { rX = 1; rY = rZ = 0; rl = 1; }
    rX /= rl; rY /= rl; rZ /= rl;

    // Up = right x forward
    float upX = rY*fwdZ - rZ*fwdY;
    float upY = rZ*fwdX - rX*fwdZ;
    float upZ = rX*fwdY - rY*fwdX;

    // View matrix (column-major)
    float V[16] = {
         rX,   upX,  -fwdX,  0,
         rY,   upY,  -fwdY,  0,
         rZ,   upZ,  -fwdZ,  0,
        -(rX*eyeX + rY*eyeY + rZ*eyeZ),
        -(upX*eyeX + upY*eyeY + upZ*eyeZ),
         (fwdX*eyeX + fwdY*eyeY + fwdZ*eyeZ),
         1
    };

    // MVP = P * V (column-major multiplication)
    for (int col = 0; col < 4; col++) {
        for (int row = 0; row < 4; row++) {
            float sum = 0;
            for (int k = 0; k < 4; k++)
                sum += P[row + k*4] * V[k + col*4];
            mvp[row + col*4] = sum;
        }
    }

    // Normal matrix = upper 3x3 of View
    nm[0]=rX;  nm[1]=upX; nm[2]=-fwdX;
    nm[3]=rY;  nm[4]=upY; nm[5]=-fwdY;
    nm[6]=rZ;  nm[7]=upZ; nm[8]=-fwdZ;
}

int main() {
#if YGG_HAS_OPENVDB
    openvdb::initialize();
#endif

    if (!glfwInit()) return -1;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* win = glfwCreateWindow(1280, 720, "Yggdrasil MachiningSim", nullptr, nullptr);
    if (!win) { glfwTerminate(); return -1; }
    glfwMakeContextCurrent(win);
    glfwSwapInterval(1);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) return -1;

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui_ImplGlfw_InitForOpenGL(win, true);
    ImGui_ImplOpenGL3_Init("#version 330");
    ImGui::StyleColorsDark();

    initGL();
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);

    Camera cam;
    bool drag = false; double lx=0, ly=0;

#if YGG_HAS_OPENVDB
    auto cfg = ygg::solveResolution(0.5, 10.0, 2.0, {30, 30, 15});
    auto billet = ygg::buildBillet(cfg, {0,0,0}, {30, 30, 15});
    auto mesh = ygg::vdbToMesh(billet.sdfGrid);
    printf("Initial mesh: %zu verts, %zu indices\n",
           mesh.vertices.size()/6, mesh.indices.size());
    uploadMesh(mesh.vertices.data(), mesh.vertices.size()*sizeof(float),
               mesh.indices.data(), mesh.indices.size()*sizeof(uint32_t),
               (int)mesh.indices.size());
    double volume = ygg::computeVolume(billet.sdfGrid);
    printf("Initial volume: %.1f mm3\n", volume);
    int cutCount = 0;
    static float cutY = 15.0f, cutR = 3.0f;
    static float cutZ = 16.0f;  // 球心Z位置（顶面=15，切入深度=15+R-cutZ）
    static bool showTool = true;
    static float toolColor[3] = {1.0f, 0.3f, 0.1f};
    static float toolCutColor[3] = {0.2f, 0.6f, 1.0f};
    static bool useDualTrack = false;
    static bool needRebuild = false;
#endif

    glfwSetWindowUserPointer(win, &cam);
    glfwSetScrollCallback(win, [](GLFWwindow* w, double, double y){
        auto* c=(Camera*)glfwGetWindowUserPointer(w);
        c->dist-=(float)y*3; if(c->dist<5)c->dist=5;});

    // ── Debug Viz State ──
    static int renderMode = 0; // 0=Solid, 1=Wireframe, 2=Solid+Wire
    static bool showMicroGrid = false;
    static float pointSize = 4.0f;
    static float wireWidth = 1.5f;
    static bool clipEnabled = false;
    static int clipAxis = 2;
    static float clipPos = 0.5f;
    static float macroColor[3] = {0.7f, 0.75f, 0.8f};
    static float microColorActive[3] = {0.1f, 0.85f, 0.3f};
    static float microColorInactive[3] = {0.85f, 0.1f, 0.1f};

    // ── MicroGrid point cloud data ──
    static GLuint ptVAO = 0, ptVBO = 0;
    static int ptCount = 0;
    static bool ptDirty = true;

    while (!glfwWindowShouldClose(win)) {
        glfwPollEvents();

        // Orbit drag
        if (glfwGetMouseButton(win, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS) {
            double mx,my; glfwGetCursorPos(win,&mx,&my);
            if(!drag){drag=true;lx=mx;ly=my;}
            cam.yaw+=(float)(mx-lx)*0.3f; cam.pitch+=(float)(my-ly)*0.3f;
            if(cam.pitch>89)cam.pitch=89; if(cam.pitch<-89)cam.pitch=-89;
            lx=mx;ly=my;
        } else drag=false;

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Yggdrasil Debug Panel", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

#if YGG_HAS_OPENVDB
        // ══════ 状态概览栏 ══════
        ImGui::TextColored(ImVec4(0.4f,0.8f,1.0f,1.0f), "⚙ Simulation State");
        ImGui::Separator();
        const char* modeStr = cfg.mode == ygg::ResolutionConfig::SINGLE_TRACK ? "SINGLE_TRACK" :
                              cfg.mode == ygg::ResolutionConfig::DUAL_TRACK ? "DUAL_TRACK" : "ATLAS";
        ImGui::Text("Mode: %s | d_v=%.3f D_v=%.3f N=%d", modeStr, cfg.d_v, cfg.D_v, cfg.N);
        ImGui::Text("Volume: %.1f mm³ | Cuts: %d", volume, cutCount);
        size_t activeVox = billet.sdfGrid->activeVoxelCount();
        ImGui::Text("FloatGrid: %zu voxels (%.2f MB)",
                    activeVox, billet.sdfGrid->memUsage()/1e6);
        if (billet.microGrid) {
            size_t ptCnt = openvdb::points::pointCount(billet.microGrid->tree());
            ImGui::Text("PointGrid: %zu surfels (%.2f MB)",
                        ptCnt, billet.microGrid->memUsage()/1e6);
        }
        ImGui::Spacing();

        // ══════ TabBar ══════
        if (ImGui::BeginTabBar("DebugTabs")) {
            // ── Tab: 外观模式 ──
            if (ImGui::BeginTabItem("Appearance")) {
                ImGui::Text("Render Mode:");
                ImGui::RadioButton("Solid", &renderMode, 0); ImGui::SameLine();
                ImGui::RadioButton("Wireframe", &renderMode, 1); ImGui::SameLine();
                ImGui::RadioButton("Solid+Wire", &renderMode, 2);

                ImGui::Checkbox("Show MicroGrid Points", &showMicroGrid);
                ImGui::SliderFloat("Point Size", &pointSize, 1.0f, 10.0f);
                ImGui::SliderFloat("Wire Width", &wireWidth, 0.5f, 5.0f);
                ImGui::ColorEdit3("Macro Color", macroColor);
                ImGui::ColorEdit3("Micro Active", microColorActive);
                ImGui::EndTabItem();
            }

            // ── Tab: 切削控制 ──
            if (ImGui::BeginTabItem("Cutting")) {
                ImGui::SliderFloat("Cut Y", &cutY, 2.0f, 28.0f);
                ImGui::SliderFloat("Cut Z (sphere center)", &cutZ, 10.0f, 20.0f);
                ImGui::SliderFloat("Tool R", &cutR, 1.0f, 8.0f);
                ImGui::Text("Cut depth: %.1f mm", 15.0f + cutR - cutZ);
                ImGui::Separator();
                ImGui::Checkbox("Show Tool", &showTool);
                if (showTool) {
                    ImGui::ColorEdit3("Tool Color", toolColor);
                    ImGui::ColorEdit3("Cut Zone Color", toolCutColor);
                }
                ImGui::Separator();
                if (ImGui::Checkbox("Dual Track Mode", &useDualTrack)) {
                    needRebuild = true;
                }
                if (needRebuild) {
                    if (useDualTrack) {
                        cfg.mode = ygg::ResolutionConfig::DUAL_TRACK;
                        cfg.d_v = 0.5; cfg.D_v = 4.0; cfg.N = 8;
                    } else {
                        cfg.mode = ygg::ResolutionConfig::SINGLE_TRACK;
                        cfg.d_v = 0.5; cfg.D_v = 0.5; cfg.N = 1;
                    }
                    billet = ygg::buildBillet(cfg, {0,0,0}, {30, 30, 15});
                    mesh = ygg::vdbToMesh(billet.sdfGrid);
                    uploadMesh(mesh.vertices.data(), mesh.vertices.size()*sizeof(float),
                               mesh.indices.data(), mesh.indices.size()*sizeof(uint32_t),
                               (int)mesh.indices.size());
                    volume = ygg::computeVolume(billet.sdfGrid);
                    cutCount = 0;
                    ptDirty = true;
                    needRebuild = false;
                }
                ImGui::Separator();
                if (ImGui::Button("Execute Cut")) {
                    ygg::CuttingEngine engine;
                    engine.cut(billet, ygg::ToolSweepSDF(
                        ygg::ToolType::BALL_END, cutR, 0, 20,
                        {2,(double)cutY,(double)cutZ}, {28,(double)cutY,(double)cutZ}));
                    mesh = ygg::vdbToMesh(billet.sdfGrid);
                    uploadMesh(mesh.vertices.data(), mesh.vertices.size()*sizeof(float),
                               mesh.indices.data(), mesh.indices.size()*sizeof(uint32_t),
                               (int)mesh.indices.size());
                    volume = ygg::computeVolume(billet.sdfGrid);
                    cutCount++;
                    ptDirty = true;
                }
                ImGui::SameLine();
                if (ImGui::Button("Reset")) {
                    billet = ygg::buildBillet(cfg, {0,0,0}, {30, 30, 15});
                    mesh = ygg::vdbToMesh(billet.sdfGrid);
                    uploadMesh(mesh.vertices.data(), mesh.vertices.size()*sizeof(float),
                               mesh.indices.data(), mesh.indices.size()*sizeof(uint32_t),
                               (int)mesh.indices.size());
                    volume = ygg::computeVolume(billet.sdfGrid);
                    cutCount = 0;
                    ptDirty = true;
                }
                ImGui::EndTabItem();
            }

            // ── Tab: 数据检查 ──
            if (ImGui::BeginTabItem("Inspect")) {
                ImGui::Checkbox("Clip Plane", &clipEnabled);
                if (clipEnabled) {
                    ImGui::RadioButton("X", &clipAxis, 0); ImGui::SameLine();
                    ImGui::RadioButton("Y", &clipAxis, 1); ImGui::SameLine();
                    ImGui::RadioButton("Z", &clipAxis, 2);
                    ImGui::SliderFloat("Position", &clipPos, 0.0f, 1.0f);
                }
                ImGui::EndTabItem();
            }

            // ── Tab: 性能 ──
            if (ImGui::BeginTabItem("Performance")) {
                ImGui::Text("FPS: %.1f (%.2f ms)", ImGui::GetIO().Framerate, 1000.0f/ImGui::GetIO().Framerate);
                ImGui::Text("Mesh vertices: %zu", mesh.vertices.size()/6);
                ImGui::Text("Mesh indices: %zu", mesh.indices.size());
                ImGui::Separator();
                ImGui::Text("FloatGrid: %.2f MB", billet.sdfGrid->memUsage()/1e6);
                if (billet.microGrid)
                    ImGui::Text("PointGrid: %.2f MB", billet.microGrid->memUsage()/1e6);
                ImGui::EndTabItem();
            }

            ImGui::EndTabBar();
        }
#else
        ImGui::Text("OpenVDB not enabled.");
#endif
        ImGui::End();
        ImGui::Render();

        // ══════ Render ══════
        int w, h; glfwGetFramebufferSize(win, &w, &h);
        glViewport(0, 0, w, h);
        glClearColor(0.15f, 0.15f, 0.18f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (g_idxCount > 0) {
            float mvp[16], nm[9];
            buildMVP(cam, w, h, mvp, nm);
            glUseProgram(g_prog);
            glUniformMatrix4fv(glGetUniformLocation(g_prog,"uMVP"),1,GL_FALSE,mvp);
            glUniformMatrix3fv(glGetUniformLocation(g_prog,"uNormalMat"),1,GL_FALSE,nm);
            glUniform3f(glGetUniformLocation(g_prog,"uLightDir"),0.30f,0.51f,0.81f);

            // Clip plane
            float clipPlane[4] = {0,0,0,0};
            if (clipEnabled) {
                float billetDims[3] = {30.0f, 30.0f, 15.0f};
                clipPlane[clipAxis] = 1.0f;
                clipPlane[3] = -(clipPos * billetDims[clipAxis]);
                glEnable(GL_CLIP_DISTANCE0);
            } else {
                glDisable(GL_CLIP_DISTANCE0);
            }
            glUniform4f(glGetUniformLocation(g_prog,"uClipPlane"),
                        clipPlane[0], clipPlane[1], clipPlane[2], clipPlane[3]);

            // Solid pass
            if (renderMode == 0 || renderMode == 2) {
                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
                glUniform3f(glGetUniformLocation(g_prog,"uColor"),
                            macroColor[0], macroColor[1], macroColor[2]);
                glBindVertexArray(g_VAO);
                glDrawElements(GL_TRIANGLES, g_idxCount, GL_UNSIGNED_INT, nullptr);
            }
            // Wireframe pass
            if (renderMode == 1 || renderMode == 2) {
                glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                glLineWidth(wireWidth);
                glEnable(GL_POLYGON_OFFSET_LINE);
                glPolygonOffset(-1.0f, -1.0f);
                glUniform3f(glGetUniformLocation(g_prog,"uColor"), 0.2f, 0.2f, 0.25f);
                glBindVertexArray(g_VAO);
                glDrawElements(GL_TRIANGLES, g_idxCount, GL_UNSIGNED_INT, nullptr);
                glDisable(GL_POLYGON_OFFSET_LINE);
                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            }

            glDisable(GL_CLIP_DISTANCE0);
        }

#if YGG_HAS_OPENVDB
        // ── Tool visualization ──
        if (showTool) {
            // 生成刀具胶囊体的简化表示（两端球 + 中间线）
            // 简化：只画一个球体代表刀具当前位置（中点）
            static GLuint toolVAO=0, toolVBO=0, toolEBO=0;
            static int toolIdxCount=0;
            static float lastToolR=0, lastToolZ=0, lastToolY=0;
            if (cutR != lastToolR || cutZ != lastToolZ || cutY != lastToolY) {
                // 重建刀具 mesh（用 SDF 球体光栅化 + volumeToMesh）
                auto toolXform = openvdb::math::Transform::createLinearTransform(0.3);
                auto toolSphere = openvdb::tools::createLevelSetSphere<openvdb::FloatGrid>(
                    float(cutR), openvdb::Vec3f(15.0f, cutY, cutZ), float(0.5), float(3.0));
                auto toolMesh = ygg::vdbToMesh(toolSphere);
                if (!toolVAO) { glGenVertexArrays(1,&toolVAO); glGenBuffers(1,&toolVBO); glGenBuffers(1,&toolEBO); }
                glBindVertexArray(toolVAO);
                glBindBuffer(GL_ARRAY_BUFFER, toolVBO);
                glBufferData(GL_ARRAY_BUFFER, toolMesh.vertices.size()*sizeof(float), toolMesh.vertices.data(), GL_DYNAMIC_DRAW);
                glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, toolEBO);
                glBufferData(GL_ELEMENT_ARRAY_BUFFER, toolMesh.indices.size()*sizeof(uint32_t), toolMesh.indices.data(), GL_DYNAMIC_DRAW);
                glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,6*sizeof(float),(void*)0);
                glEnableVertexAttribArray(0);
                glVertexAttribPointer(1,3,GL_FLOAT,GL_FALSE,6*sizeof(float),(void*)(3*sizeof(float)));
                glEnableVertexAttribArray(1);
                toolIdxCount = (int)toolMesh.indices.size();
                lastToolR=cutR; lastToolZ=cutZ; lastToolY=cutY;
            }
            if (toolIdxCount > 0) {
                float mvp[16], nm[9];
                buildMVP(cam, w, h, mvp, nm);
                glUseProgram(g_prog);
                glUniformMatrix4fv(glGetUniformLocation(g_prog,"uMVP"),1,GL_FALSE,mvp);
                glUniformMatrix3fv(glGetUniformLocation(g_prog,"uNormalMat"),1,GL_FALSE,nm);
                glUniform3f(glGetUniformLocation(g_prog,"uLightDir"),0.30f,0.51f,0.81f);
                // 底部半球（切削区）用 cutColor，上半用 toolColor
                // 简化：整个球用 toolColor，半透明
                glEnable(GL_BLEND);
                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                glUniform3f(glGetUniformLocation(g_prog,"uColor"),toolColor[0],toolColor[1],toolColor[2]);
                glBindVertexArray(toolVAO);
                glDrawElements(GL_TRIANGLES, toolIdxCount, GL_UNSIGNED_INT, nullptr);
                glDisable(GL_BLEND);
            }
        }
#endif

#if YGG_HAS_OPENVDB
        // ── MicroGrid point cloud rendering ──
        if (showMicroGrid && billet.microGrid) {
            if (ptDirty) {
                // Extract surfel positions + colors
                std::vector<float> ptData; // x,y,z,r,g,b per point
                auto& tree = billet.microGrid->tree();
                auto& xf = billet.microGrid->transform();
                for (auto leaf = tree.cbeginLeaf(); leaf; ++leaf) {
                    auto leafOrigin = leaf->origin();
                    auto& attrSet = leaf->attributeSet();
                    auto* posArr = attrSet.get("P");
                    auto* activeArr = attrSet.get("active");
                    if (!posArr || !activeArr) continue;
                    auto ph = openvdb::points::AttributeHandle<openvdb::Vec3f>::create(*posArr);
                    auto ah = openvdb::points::AttributeHandle<uint8_t>::create(*activeArr);

                    for (openvdb::Index vIdx = 0; vIdx < 512; ++vIdx) {
                        openvdb::Index end = static_cast<openvdb::Index>(leaf->getValue(vIdx));
                        openvdb::Index start = (vIdx==0) ? openvdb::Index(0) :
                            static_cast<openvdb::Index>(leaf->getValue(vIdx-1));
                        openvdb::Coord lc((vIdx>>6)&7,(vIdx>>3)&7,vIdx&7);
                        openvdb::Coord vc = leafOrigin + lc;
                        for (openvdb::Index i = start; i < end; ++i) {
                            auto p = ph->get(i);
                            auto wp = xf.indexToWorld(openvdb::Vec3d(
                                vc.x()+p.x(), vc.y()+p.y(), vc.z()+p.z()));
                            ptData.push_back(float(wp.x()));
                            ptData.push_back(float(wp.y()));
                            ptData.push_back(float(wp.z()));
                            if (ah->get(i) == 1) {
                                ptData.push_back(microColorActive[0]);
                                ptData.push_back(microColorActive[1]);
                                ptData.push_back(microColorActive[2]);
                            } else {
                                ptData.push_back(microColorInactive[0]);
                                ptData.push_back(microColorInactive[1]);
                                ptData.push_back(microColorInactive[2]);
                            }
                        }
                    }
                }
                ptCount = (int)(ptData.size() / 6);
                if (!ptVAO) { glGenVertexArrays(1, &ptVAO); glGenBuffers(1, &ptVBO); }
                glBindVertexArray(ptVAO);
                glBindBuffer(GL_ARRAY_BUFFER, ptVBO);
                glBufferData(GL_ARRAY_BUFFER, ptData.size()*sizeof(float), ptData.data(), GL_DYNAMIC_DRAW);
                glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
                glEnableVertexAttribArray(0);
                glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
                glEnableVertexAttribArray(1);
                ptDirty = false;
            }
            if (ptCount > 0) {
                float mvp[16], nm[9];
                buildMVP(cam, w, h, mvp, nm);
                glUseProgram(g_prog);
                glUniformMatrix4fv(glGetUniformLocation(g_prog,"uMVP"),1,GL_FALSE,mvp);
                glUniform3f(glGetUniformLocation(g_prog,"uLightDir"),0,0,1);
                glPointSize(pointSize);
                glEnable(GL_PROGRAM_POINT_SIZE);
                glBindVertexArray(ptVAO);
                // Use vertex color as uColor trick: set uColor to white, multiply in shader
                glUniform3f(glGetUniformLocation(g_prog,"uColor"), 1.0f, 1.0f, 1.0f);
                glDrawArrays(GL_POINTS, 0, ptCount);
            }
        }
#endif

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(win);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(win);
    glfwTerminate();
    return 0;
}
