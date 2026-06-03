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
out vec3 vNorm;
void main(){
    gl_Position = uMVP * vec4(aPos, 1.0);
    vNorm = normalize(uNormalMat * aNorm);
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
#endif

    glfwSetWindowUserPointer(win, &cam);
    glfwSetScrollCallback(win, [](GLFWwindow* w, double, double y){
        auto* c=(Camera*)glfwGetWindowUserPointer(w);
        c->dist-=(float)y*3; if(c->dist<5)c->dist=5;});

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

        ImGui::Begin("Control Panel");
        ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);
#if YGG_HAS_OPENVDB
        ImGui::Text("Volume: %.1f mm3", volume);
        ImGui::Text("Cuts: %d", cutCount);
        ImGui::Separator();
        ImGui::SliderFloat("Cut Y", &cutY, 2.0f, 28.0f);
        ImGui::SliderFloat("Tool R", &cutR, 1.0f, 8.0f);
        if (ImGui::Button("Execute Cut")) {
            ygg::CuttingEngine engine;
            engine.cut(billet, ygg::ToolSweepSDF(
                ygg::ToolType::BALL_END, cutR, 0, 20,
                {2,(double)cutY,0}, {28,(double)cutY,0}));
            mesh = ygg::vdbToMesh(billet.sdfGrid);
            printf("After cut: %zu verts, %zu indices\n",
                   mesh.vertices.size()/6, mesh.indices.size());
            uploadMesh(mesh.vertices.data(), mesh.vertices.size()*sizeof(float),
                       mesh.indices.data(), mesh.indices.size()*sizeof(uint32_t),
                       (int)mesh.indices.size());
            volume = ygg::computeVolume(billet.sdfGrid);
            printf("Volume after cut: %.1f\n", volume);
            cutCount++;
        }
        if (ImGui::Button("Reset Billet")) {
            billet = ygg::buildBillet(cfg, {0,0,0}, {30, 30, 15});
            mesh = ygg::vdbToMesh(billet.sdfGrid);
            uploadMesh(mesh.vertices.data(), mesh.vertices.size()*sizeof(float),
                       mesh.indices.data(), mesh.indices.size()*sizeof(uint32_t),
                       (int)mesh.indices.size());
            volume = ygg::computeVolume(billet.sdfGrid);
            cutCount = 0;
        }
#else
        ImGui::Text("OpenVDB not enabled. Window-only mode.");
#endif
        ImGui::End();
        ImGui::Render();

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
            glUniform3f(glGetUniformLocation(g_prog,"uColor"),0.7f,0.75f,0.8f);
            glBindVertexArray(g_VAO);
            glDrawElements(GL_TRIANGLES, g_idxCount, GL_UNSIGNED_INT, nullptr);
        }

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
