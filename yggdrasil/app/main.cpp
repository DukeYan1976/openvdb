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
#include <string>
#include <chrono>
#include <ctime>
#include <thread>
#include <atomic>

// ─── helper: current local timestamp as [YYYY/MM/DD HH:MM:SS.mmm] ───
static std::string ts() {
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm* tm = std::localtime(&t);
    char buf[32];
    std::snprintf(buf, sizeof(buf), "[%04d/%02d/%02d %02d:%02d:%02d.%03d]",
        tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
        tm->tm_hour, tm->tm_min, tm->tm_sec, static_cast<int>(ms.count()));
    return buf;
}

// ═══════════════════════════════════════════════════════════════
// Shaders
// ═══════════════════════════════════════════════════════════════
static const char* g_vertSrc = R"(
#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNorm;
uniform mat4 uMVP;
uniform mat3 uNormalMat;
out vec3 vNorm;
void main(){
    gl_Position = uMVP * vec4(aPos, 1.0);
    vNorm = normalize(uNormalMat * aNorm);
    gl_PointSize = 4.0;
})";

static const char* g_fragSrc = R"(
#version 330 core
in vec3 vNorm;
out vec4 fragColor;
uniform vec3 uLightDir;
uniform vec3 uColor;
uniform float uAlpha;
void main(){
    float diff = abs(dot(vNorm, uLightDir)) * 0.7 + 0.3;
    fragColor = vec4(uColor * diff, uAlpha);
})";

// Line shader (no lighting)
static const char* g_lineVertSrc = R"(
#version 330 core
layout(location=0) in vec3 aPos;
uniform mat4 uMVP;
void main(){ gl_Position = uMVP * vec4(aPos, 1.0); }
)";

static const char* g_lineFragSrc = R"(
#version 330 core
out vec4 fragColor;
uniform vec3 uColor;
uniform float uAlpha;
void main(){ fragColor = vec4(uColor, uAlpha); }
)";

// ═══════════════════════════════════════════════════════════════
// GL helpers
// ═══════════════════════════════════════════════════════════════
static GLuint compileProgram(const char* vs, const char* fs) {
    GLuint v = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(v, 1, &vs, nullptr); glCompileShader(v);
    GLint ok; glGetShaderiv(v, GL_COMPILE_STATUS, &ok);
    if(!ok){char log[512];glGetShaderInfoLog(v,512,nullptr,log);printf("VS: %s\n",log);}
    GLuint f = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(f, 1, &fs, nullptr); glCompileShader(f);
    glGetShaderiv(f, GL_COMPILE_STATUS, &ok);
    if(!ok){char log[512];glGetShaderInfoLog(f,512,nullptr,log);printf("FS: %s\n",log);}
    GLuint p = glCreateProgram();
    glAttachShader(p, v); glAttachShader(p, f);
    glLinkProgram(p);
    glDeleteShader(v); glDeleteShader(f);
    return p;
}

struct GPUMesh {
    GLuint vao=0, vbo=0, ebo=0;
    int count=0;
    void init() { glGenVertexArrays(1,&vao); glGenBuffers(1,&vbo); glGenBuffers(1,&ebo); }
    void upload(const float* verts, size_t vBytes, const uint32_t* idx, size_t iBytes, int n) {
        if(!vao) init();
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, vBytes, verts, GL_DYNAMIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, iBytes, idx, GL_DYNAMIC_DRAW);
        glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,6*sizeof(float),(void*)0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1,3,GL_FLOAT,GL_FALSE,6*sizeof(float),(void*)(3*sizeof(float)));
        glEnableVertexAttribArray(1);
        count = n;
    }
    void draw() { if(count>0){glBindVertexArray(vao);glDrawElements(GL_TRIANGLES,count,GL_UNSIGNED_INT,nullptr);} }
};

struct GPULines {
    GLuint vao=0, vbo=0;
    int count=0;
    void init() { glGenVertexArrays(1,&vao); glGenBuffers(1,&vbo); }
    void upload(const float* data, size_t bytes, int vertCount) {
        if(!vao) init();
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, bytes, data, GL_DYNAMIC_DRAW);
        glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,3*sizeof(float),(void*)0);
        glEnableVertexAttribArray(0);
        count = vertCount;
    }
    void draw() { if(count>0){glBindVertexArray(vao);glDrawArrays(GL_LINES,0,count);} }
};

// ═══════════════════════════════════════════════════════════════
// Camera (CAD-style orbit, orthographic)
// ═══════════════════════════════════════════════════════════════
struct Camera {
    float yaw=45, pitch=30, dist=200;
    float tx=15, ty=15, tz=7.5f;
    float orthoSize=25.0f;
};

static void buildMVP(const Camera& c, int w, int h, float mvp[16], float nm[9]) {
    float asp=(float)w/(float)h;
    float zN=0.1f, zF=1000.f;
    float P[16]={0};
    P[0]=1.f/(asp*c.orthoSize); P[5]=1.f/c.orthoSize;
    P[10]=-2.f/(zF-zN); P[14]=-(zF+zN)/(zF-zN); P[15]=1.f;

    float yr=c.yaw*3.14159f/180.f, pr=c.pitch*3.14159f/180.f;
    float ex=c.tx+c.dist*cosf(pr)*cosf(yr);
    float ey=c.ty+c.dist*cosf(pr)*sinf(yr);
    float ez=c.tz+c.dist*sinf(pr);

    float fx=c.tx-ex, fy=c.ty-ey, fz=c.tz-ez;
    float fl=sqrtf(fx*fx+fy*fy+fz*fz); fx/=fl;fy/=fl;fz/=fl;
    float rx=fy, ry=-fx, rz=0;
    float rl=sqrtf(rx*rx+ry*ry+rz*rz);
    if(rl<1e-6f){rx=1;ry=rz=0;rl=1;} rx/=rl;ry/=rl;rz/=rl;
    float ux=ry*fz-rz*fy, uy=rz*fx-rx*fz, uz=rx*fy-ry*fx;

    float V[16]={rx,ux,-fx,0, ry,uy,-fy,0, rz,uz,-fz,0,
        -(rx*ex+ry*ey+rz*ez),-(ux*ex+uy*ey+uz*ez),(fx*ex+fy*ey+fz*ez),1};

    for(int col=0;col<4;col++)for(int row=0;row<4;row++){
        float s=0;for(int k=0;k<4;k++)s+=P[row+k*4]*V[k+col*4];
        mvp[row+col*4]=s;
    }
    nm[0]=rx;nm[1]=ux;nm[2]=-fx; nm[3]=ry;nm[4]=uy;nm[5]=-fy; nm[6]=rz;nm[7]=uz;nm[8]=-fz;
}

// ═══════════════════════════════════════════════════════════════
// Layer 1: Original Billet (analytic geometry)
// ═══════════════════════════════════════════════════════════════
static void buildBoxMesh(float ox, float oy, float oz,
                         float dx, float dy, float dz,
                         std::vector<float>& verts, std::vector<uint32_t>& idx) {
    // 8 corners, 6 faces, 12 triangles
    float x0=ox,y0=oy,z0=oz, x1=ox+dx,y1=oy+dy,z1=oz+dz;
    struct V { float x,y,z,nx,ny,nz; };
    // 24 vertices (4 per face, each with face normal)
    V vtx[24] = {
        // -Z face
        {x0,y0,z0, 0,0,-1},{x1,y0,z0, 0,0,-1},{x1,y1,z0, 0,0,-1},{x0,y1,z0, 0,0,-1},
        // +Z face
        {x0,y0,z1, 0,0,1},{x1,y0,z1, 0,0,1},{x1,y1,z1, 0,0,1},{x0,y1,z1, 0,0,1},
        // -Y face
        {x0,y0,z0, 0,-1,0},{x1,y0,z0, 0,-1,0},{x1,y0,z1, 0,-1,0},{x0,y0,z1, 0,-1,0},
        // +Y face
        {x0,y1,z0, 0,1,0},{x1,y1,z0, 0,1,0},{x1,y1,z1, 0,1,0},{x0,y1,z1, 0,1,0},
        // -X face
        {x0,y0,z0, -1,0,0},{x0,y1,z0, -1,0,0},{x0,y1,z1, -1,0,0},{x0,y0,z1, -1,0,0},
        // +X face
        {x1,y0,z0, 1,0,0},{x1,y1,z0, 1,0,0},{x1,y1,z1, 1,0,0},{x1,y0,z1, 1,0,0},
    };
    verts.resize(24*6);
    for(int i=0;i<24;i++){
        verts[i*6+0]=vtx[i].x; verts[i*6+1]=vtx[i].y; verts[i*6+2]=vtx[i].z;
        verts[i*6+3]=vtx[i].nx; verts[i*6+4]=vtx[i].ny; verts[i*6+5]=vtx[i].nz;
    }
    uint32_t faces[36]={
        0,1,2, 0,2,3,   4,6,5, 4,7,6,
        8,9,10, 8,10,11, 12,14,13, 12,15,14,
        16,17,18, 16,18,19, 20,22,21, 20,23,22
    };
    idx.assign(faces, faces+36);
}

// ═══════════════════════════════════════════════════════════════
// Layer 2: MacroGrid Wireframe
// ═══════════════════════════════════════════════════════════════
#if YGG_HAS_OPENVDB
static void addWireBox(std::vector<float>& lines, double ox, double oy, double oz, double s) {
    float x0=(float)ox,y0=(float)oy,z0=(float)oz;
    float x1=x0+(float)s,y1=y0+(float)s,z1=z0+(float)s;
    // 12 edges = 24 vertices
    auto edge=[&](float ax,float ay,float az,float bx,float by,float bz){
        lines.push_back(ax);lines.push_back(ay);lines.push_back(az);
        lines.push_back(bx);lines.push_back(by);lines.push_back(bz);
    };
    // bottom
    edge(x0,y0,z0,x1,y0,z0); edge(x1,y0,z0,x1,y1,z0);
    edge(x1,y1,z0,x0,y1,z0); edge(x0,y1,z0,x0,y0,z0);
    // top
    edge(x0,y0,z1,x1,y0,z1); edge(x1,y0,z1,x1,y1,z1);
    edge(x1,y1,z1,x0,y1,z1); edge(x0,y1,z1,x0,y0,z1);
    // verticals
    edge(x0,y0,z0,x0,y0,z1); edge(x1,y0,z0,x1,y0,z1);
    edge(x1,y1,z0,x1,y1,z1); edge(x0,y1,z0,x0,y1,z1);
}

static void buildMacroGridLines(const openvdb::FloatGrid::Ptr& grid, int lod,
                                std::vector<float>& lines) {
    lines.clear();
    auto& xform = grid->transform();
    double vs = xform.voxelSize()[0];

    if (lod == 0) {
        // Per active voxel (expensive for large grids)
        for (auto iter = grid->cbeginValueOn(); iter; ++iter) {
            auto wp = xform.indexToWorld(iter.getCoord());
            addWireBox(lines, wp.x()-vs*0.5, wp.y()-vs*0.5, wp.z()-vs*0.5, vs);
        }
    } else if (lod == 1) {
        // Per LeafNode (8^3 voxels)
        for (auto leaf = grid->tree().cbeginLeaf(); leaf; ++leaf) {
            auto wp = xform.indexToWorld(leaf->origin());
            addWireBox(lines, wp.x(), wp.y(), wp.z(), vs*8);
        }
    } else {
        // Coarse: bounding box of entire grid
        auto bbox = grid->evalActiveVoxelBoundingBox();
        auto minW = xform.indexToWorld(bbox.min());
        auto maxW = xform.indexToWorld(bbox.max()+openvdb::Coord(1));
        addWireBox(lines, minW.x(), minW.y(), minW.z(),
                   maxW.x()-minW.x());
    }
}
#endif

// ═══════════════════════════════════════════════════════════════
// Main
// ═══════════════════════════════════════════════════════════════
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

    GLuint meshProg = compileProgram(g_vertSrc, g_fragSrc);
    GLuint lineProg = compileProgram(g_lineVertSrc, g_lineFragSrc);

    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);

    Camera cam;
    bool drag=false; double lx=0,ly=0;

    // ── Layer state ──
    GPUMesh billetOrigMesh;   // Layer 1: analytic billet
    GPUMesh sdfMesh;          // SDF marching cubes result
    GPUMesh microDetailMesh;  // Layer 3: focus area detail
    GPULines macroLines;      // Layer 2: wireframe

    // Visibility & appearance
    bool showOrigBillet = true;
    float origAlpha = 0.5f;
    float origColor[3] = {0.55f, 0.6f, 0.65f};

    bool showMacroGrid = false;
    int macroLod = 1; // 0=voxel, 1=leaf, 2=bbox
    float macroColor[3] = {0.3f, 0.55f, 0.9f};
    float macroLineWidth = 1.5f;

    bool showMicroDetail = false;
    float microColor[3] = {0.2f, 0.85f, 0.35f};
    float focusRadius = 15.0f;
    bool autoFocus = true;

    bool showSdfMesh = true;
    float sdfColor[3] = {0.7f, 0.75f, 0.8f};

#if YGG_HAS_OPENVDB
    // ── Simulation state ──
    float billetSize[3] = {30,30,15};
    auto cfg = ygg::solveResolution(0.5, 10.0, 2.0, {30,30,15});
    auto billet = ygg::buildBillet(cfg, {0,0,0}, {30,30,15});
    double volume = ygg::computeVolume(billet.sdfGrid);
    int cutCount = 0;
    float cutY=15, cutR=3, cutZ=16;
    bool layersDirty = true;
    bool showTool = true;
    float toolColor[3] = {1.0f, 0.3f, 0.1f};
    float tolerance = 0.5f;
    float memBudgetMB = 500.0f;
    GPUMesh toolMesh;

    // Async mesh rebuild state
    std::thread rebuildThread;
    std::atomic<bool> rebuildRunning{false};
    std::atomic<bool> rebuildPendingUpload{false};
    ygg::MeshData pendingSdfMesh;
    std::vector<float> pendingMacroLines;
    ygg::MeshData pendingMicroMesh;
    bool pendingHasMacro = false;
    bool pendingHasMicro = false;

    // ── Voxel Inspector state ──
    bool inspectorActive = false;
    openvdb::Coord inspVoxel{0,0,0};
    int inspDepth = 0;
    bool inspValid = false;
    GPULines inspVoxelWire;  // yellow box
    GPULines inspLeafWire;   // blue box

    // Build original billet mesh (analytic box)
    {
        std::vector<float> v; std::vector<uint32_t> i;
        buildBoxMesh(0,0,0, billetSize[0],billetSize[1],billetSize[2], v, i);
        billetOrigMesh.upload(v.data(), v.size()*sizeof(float),
                              i.data(), i.size()*sizeof(uint32_t), (int)i.size());
    }
#endif

    glfwSetWindowUserPointer(win, &cam);
    glfwSetScrollCallback(win, [](GLFWwindow* w, double, double y){
        auto* c=(Camera*)glfwGetWindowUserPointer(w);
        c->orthoSize *= (y>0)?0.9f:1.1f;
        if(c->orthoSize<1)c->orthoSize=1;
        if(c->orthoSize>500)c->orthoSize=500;
    });

    while (!glfwWindowShouldClose(win)) {
        glfwPollEvents();

        // ── Mouse interaction ──
        {
            double mx,my; glfwGetCursorPos(win,&mx,&my);
            bool mmb=glfwGetMouseButton(win,GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS;
            bool rmb=glfwGetMouseButton(win,GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS;
            bool shift=(glfwGetKey(win,GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS);
            if(mmb||rmb){
                if(!drag){drag=true;lx=mx;ly=my;}
                double dx=mx-lx, dy=my-ly;
                if(mmb&&!shift){
                    cam.yaw+=(float)dx*0.3f; cam.pitch+=(float)dy*0.3f;
                    if(cam.pitch>89)cam.pitch=89; if(cam.pitch<-89)cam.pitch=-89;
                } else {
                    int vw,vh; glfwGetFramebufferSize(win,&vw,&vh);
                    float ps=2.f*cam.orthoSize/(float)vh;
                    float yr=cam.yaw*3.14159f/180.f;
                    cam.tx+=(float)dx*ps*(-sinf(yr));
                    cam.ty+=(float)dx*ps*(cosf(yr));
                    cam.tz+=(float)dy*ps;
                }
                lx=mx;ly=my;
            } else drag=false;
        }

#if YGG_HAS_OPENVDB
        // ── Voxel Inspector: ray pick ──
        if (inspectorActive && showMicroDetail && billet.sdfGrid) {
            double mx, my; glfwGetCursorPos(win, &mx, &my);
            int vw, vh; glfwGetFramebufferSize(win, &vw, &vh);
            // Ortho unproject: screen → world (simplified for ortho)
            float asp = (float)vw/(float)vh;
            float ndcX = (float)(2.0*mx/vw - 1.0);
            float ndcY = (float)(1.0 - 2.0*my/vh);
            float yr = cam.yaw*3.14159f/180.f, pr = cam.pitch*3.14159f/180.f;
            // Right and up in world space
            float rx = -sinf(yr), ry = cosf(yr), rz = 0;
            float ux = -sinf(pr)*cosf(yr), uy = -sinf(pr)*sinf(yr), uz = cosf(pr);
            float worldX = cam.tx + ndcX*asp*cam.orthoSize*rx + ndcY*cam.orthoSize*ux;
            float worldY = cam.ty + ndcX*asp*cam.orthoSize*ry + ndcY*cam.orthoSize*uy;
            float worldZ = cam.tz + ndcX*asp*cam.orthoSize*rz + ndcY*cam.orthoSize*uz;

            // Find voxel coord at this world position (use SDF grid transform)
            auto& xf = billet.sdfGrid->transform();
            auto idx = xf.worldToIndexCellCentered(ygg::Vec3d(worldX, worldY, worldZ));

            // Walk along view direction to find N-th surface voxel
            float fwdX = cosf(pr)*cosf(yr), fwdY = cosf(pr)*sinf(yr), fwdZ = sinf(pr);
            inspValid = false;
            int found = 0;
            double D_v = xf.voxelSize()[0];
            auto sdfAcc = billet.sdfGrid->getConstAccessor();
            for (int step = -50; step <= 50; ++step) {
                ygg::Vec3d probe(worldX - fwdX*step*D_v,
                                 worldY - fwdY*step*D_v,
                                 worldZ - fwdZ*step*D_v);
                auto c = xf.worldToIndexCellCentered(probe);
                if (billet.sdfGrid->tree().isValueOn(c)) {
                    float sv = sdfAcc.getValue(c);
                    // Boundary voxel: SDF near zero (sign change with neighbors)
                    if (std::abs(sv) < (float)D_v) {
                        if (found == inspDepth) {
                            inspVoxel = c; inspValid = true;
                            break;
                        }
                        found++;
                    }
                }
            }

            // Left click: advance depth
            static bool lmbWasPressed = false;
            bool lmbNow = glfwGetMouseButton(win, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS;
            if (lmbNow && !lmbWasPressed && !ImGui::GetIO().WantCaptureMouse) inspDepth++;
            lmbWasPressed = lmbNow;

            // Build wireframe boxes for hovered voxel
            if (inspValid) {
                auto wp = xf.indexToWorld(inspVoxel);
                std::vector<float> vlines, llines;
                addWireBox(vlines, wp.x(), wp.y(), wp.z(), D_v);
                inspVoxelWire.upload(vlines.data(), vlines.size()*sizeof(float), (int)vlines.size()/3);
                // Leaf bbox (8³ aligned)
                openvdb::Coord leafOrigin(inspVoxel.x()&~7, inspVoxel.y()&~7, inspVoxel.z()&~7);
                auto lwp = xf.indexToWorld(leafOrigin);
                addWireBox(llines, lwp.x(), lwp.y(), lwp.z(), D_v*8);
                inspLeafWire.upload(llines.data(), llines.size()*sizeof(float), (int)llines.size()/3);
            }
        }
#endif

#if YGG_HAS_OPENVDB
        // ── Trigger async mesh rebuild when dirty ──
        if (layersDirty && !rebuildRunning) {
            rebuildRunning = true;
            rebuildPendingUpload = false;
            if (rebuildThread.joinable()) rebuildThread.join();

            auto* billetPtr = &billet;
            int currentMacroLod = macroLod;
            bool currentShowMacro = showMacroGrid;
            bool currentShowMicro = showMicroDetail;
            int currentCutCount = cutCount;
            float currentCutR = cutR;
            float currentCutY = cutY;
            float currentCutZ = cutZ;
            float currentBilletSizeX = billetSize[0];

            rebuildThread = std::thread([=, &pendingSdfMesh, &pendingMacroLines, &pendingMicroMesh,
                                         &pendingHasMacro, &pendingHasMicro,
                                         &rebuildRunning, &rebuildPendingUpload]() {
                pendingSdfMesh = ygg::vdbToMesh(billetPtr->sdfGrid);

                pendingHasMacro = currentShowMacro;
                if (currentShowMacro) {
                    buildMacroGridLines(billetPtr->sdfGrid, currentMacroLod, pendingMacroLines);
                }

                pendingHasMicro = (currentShowMicro && currentCutCount > 0 && billetPtr->isDualTrack());
                if (pendingHasMicro) {
                    ygg::ToolSweepSDF lastTool(ygg::ToolType::BALL_END, currentCutR, 0, 20,
                        {2, (double)currentCutY, (double)currentCutZ},
                        {(double)currentBilletSizeX - 2.0, (double)currentCutY, (double)currentCutZ});
                    auto csGrid = ygg::buildLocalCutSurface(*billetPtr, lastTool);
                    pendingMicroMesh = ygg::vdbToMesh(csGrid);
                }

                rebuildPendingUpload = true;
                rebuildRunning = false;
            });

            layersDirty = false;
        }

        // ── Upload completed results (main thread only for OpenGL) ──
        if (rebuildPendingUpload) {
            sdfMesh.upload(pendingSdfMesh.vertices.data(), pendingSdfMesh.vertices.size()*sizeof(float),
                           pendingSdfMesh.indices.data(), pendingSdfMesh.indices.size()*sizeof(uint32_t),
                           (int)pendingSdfMesh.indices.size());
            if (pendingHasMacro) {
                macroLines.upload(pendingMacroLines.data(), pendingMacroLines.size()*sizeof(float),
                                  (int)pendingMacroLines.size()/3);
            }
            if (pendingHasMicro) {
                microDetailMesh.upload(pendingMicroMesh.vertices.data(), pendingMicroMesh.vertices.size()*sizeof(float),
                                       pendingMicroMesh.indices.data(), pendingMicroMesh.indices.size()*sizeof(uint32_t),
                                       (int)pendingMicroMesh.indices.size());
            }
            rebuildPendingUpload = false;
        }
#endif

        // ── ImGui ──
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Yggdrasil Control", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
#if YGG_HAS_OPENVDB
        ImGui::Text("FPS: %.0f | Volume: %.1f mm3 | Cuts: %d", ImGui::GetIO().Framerate, volume, cutCount);
        ImGui::Separator();

        if (ImGui::BeginTabBar("Tabs")) {
            // ══ Visualization Tab ══
            if (ImGui::BeginTabItem("Visualization")) {
                ImGui::TextColored(ImVec4(1,0.8f,0.3f,1), "Layer 1: Original Billet");
                ImGui::Checkbox("Show##orig", &showOrigBillet);
                if (showOrigBillet) {
                    ImGui::SameLine(); ImGui::SetNextItemWidth(120);
                    ImGui::SliderFloat("Alpha##orig", &origAlpha, 0.0f, 1.0f);
                    ImGui::ColorEdit3("Color##orig", origColor, ImGuiColorEditFlags_NoInputs);
                }
                ImGui::Spacing();

                ImGui::TextColored(ImVec4(0.3f,0.6f,1,1), "Layer 2: MacroGrid Wireframe");
                if (ImGui::Checkbox("Show##macro", &showMacroGrid)) layersDirty = true;
                if (showMacroGrid) {
                    ImGui::SameLine();
                    if (ImGui::RadioButton("Voxel", &macroLod, 0)) layersDirty = true;
                    ImGui::SameLine();
                    if (ImGui::RadioButton("Leaf(8^3)", &macroLod, 1)) layersDirty = true;
                    ImGui::SameLine();
                    if (ImGui::RadioButton("BBox", &macroLod, 2)) layersDirty = true;
                    ImGui::SliderFloat("Line Width", &macroLineWidth, 0.5f, 5.0f);
                    ImGui::ColorEdit3("Color##macro", macroColor, ImGuiColorEditFlags_NoInputs);
                }
                ImGui::Spacing();

                ImGui::TextColored(ImVec4(0.2f,0.9f,0.4f,1), "Layer 3: MicroGrid Detail Surface");
                if (ImGui::Checkbox("Show##micro", &showMicroDetail)) layersDirty = true;
                if (showMicroDetail) {
                    ImGui::Checkbox("Auto Focus (follow cut)", &autoFocus);
                    ImGui::SliderFloat("Focus Radius (mm)", &focusRadius, 5.0f, 50.0f);
                    ImGui::ColorEdit3("Color##micro", microColor, ImGuiColorEditFlags_NoInputs);
                    ImGui::Checkbox("Voxel Inspector (hover)", &inspectorActive);
                    if (inspectorActive) {
                        ImGui::SameLine(); ImGui::Text("Depth:%d", inspDepth);
                        ImGui::SameLine(); if(ImGui::SmallButton("Reset##insp")) inspDepth=0;
                    }
                }
                ImGui::Spacing();

                ImGui::TextColored(ImVec4(0.7f,0.75f,0.8f,1), "SDF Mesh (Marching Cubes)");
                ImGui::Checkbox("Show##sdf", &showSdfMesh);
                if (showSdfMesh)
                    ImGui::ColorEdit3("Color##sdf", sdfColor, ImGuiColorEditFlags_NoInputs);

                ImGui::EndTabItem();
            }

            // ══ Cutting Tab ══
            if (ImGui::BeginTabItem("Cutting")) {
                // ─── Build Section ───
                ImGui::TextColored(ImVec4(0.4f,0.8f,1,1), "Build Settings");
                ImGui::InputFloat3("Billet (mm)", billetSize);
                ImGui::SliderFloat("Tolerance t", &tolerance, 0.001f, 2.0f, "%.4f", ImGuiSliderFlags_Logarithmic);
                ImGui::SliderFloat("Mem Budget (MB)", &memBudgetMB, 50.0f, 4000.0f);

                auto previewCfg = ygg::solveResolution(
                    (double)tolerance, (double)cutR, 2.0,
                    {(double)billetSize[0],(double)billetSize[1],(double)billetSize[2]},
                    static_cast<size_t>(memBudgetMB*1024*1024));
                const char* mStr = previewCfg.mode==ygg::ResolutionConfig::SINGLE_TRACK?"SINGLE":
                    previewCfg.mode==ygg::ResolutionConfig::DUAL_TRACK?"DUAL":"ATLAS";
                ImGui::TextColored(ImVec4(0.3f,1,0.3f,1), "Preview: %s d_v=%.4f D_v=%.3f N=%d",
                    mStr, previewCfg.d_v, previewCfg.D_v, previewCfg.N);

                static double buildMs=0;
                if (ImGui::Button("Apply (Rebuild)")) {
                    auto t0=std::chrono::high_resolution_clock::now();
                    cfg = previewCfg;
                    billet = ygg::buildBillet(cfg, {0,0,0},
                        {(double)billetSize[0],(double)billetSize[1],(double)billetSize[2]});
                    volume = ygg::computeVolume(billet.sdfGrid);
                    cutCount=0; layersDirty=true;
                    // Rebuild analytic billet mesh
                    std::vector<float> bv; std::vector<uint32_t> bi;
                    buildBoxMesh(0,0,0,billetSize[0],billetSize[1],billetSize[2],bv,bi);
                    billetOrigMesh.upload(bv.data(),bv.size()*sizeof(float),
                        bi.data(),bi.size()*sizeof(uint32_t),(int)bi.size());
                    cam.tx=billetSize[0]/2; cam.ty=billetSize[1]/2; cam.tz=billetSize[2]/2;
                    auto t1=std::chrono::high_resolution_clock::now();
                    buildMs=std::chrono::duration<double,std::milli>(t1-t0).count();
                }
                if (buildMs>0) ImGui::SameLine(), ImGui::Text("%.1f ms", buildMs);
                ImGui::Separator();

                // ─── Cut Section ───
                ImGui::TextColored(ImVec4(1,0.6f,0.2f,1), "Cut Parameters");
                ImGui::SliderFloat("Cut Y", &cutY, 2.0f, billetSize[1]-2.0f);
                ImGui::SliderFloat("Cut Z", &cutZ, billetSize[2]*0.5f, billetSize[2]+5.0f);
                ImGui::SliderFloat("Tool R", &cutR, 1.0f, 8.0f);
                ImGui::Text("Depth: %.1f mm", billetSize[2]+cutR-cutZ);

                static double cutMs=0;
                static std::string cutInfo;
                // Pre-compute cut scale info
                {
                    double pathLen = billetSize[0] - 4.0; // start=2, end=billetSize[0]-2
                    ygg::ToolSweepSDF previewTool(ygg::ToolType::BALL_END, cutR, 0, 20,
                        {2,(double)cutY,(double)cutZ}, {billetSize[0]-2.0,(double)cutY,(double)cutZ});
                    auto tbbox = previewTool.getBoundingBox();
                    auto tMin = billet.sdfGrid->transform().worldToIndexCellCentered(tbbox.min());
                    auto tMax = billet.sdfGrid->transform().worldToIndexCellCentered(tbbox.max());
                    long long voxelEst = (long long)(tMax.x()-tMin.x()+1)*(tMax.y()-tMin.y()+1)*(tMax.z()-tMin.z()+1);
                    char buf[256];
                    snprintf(buf, sizeof(buf),
                        "Path: %.1fmm | R=%.1f | d_v=%.4f | BBox voxels: %lld | N=%d (surfels/voxel: %d)",
                        pathLen, cutR, cfg.d_v, voxelEst, cfg.N, cfg.N*cfg.N);
                    cutInfo = buf;
                }
                ImGui::TextWrapped("%s", cutInfo.c_str());

                if (rebuildRunning) {
                    ImGui::BeginDisabled();
                }
                if (ImGui::Button("Execute Cut")) {
                    printf("%s [Cut] Computing... %s\n", ts().c_str(), cutInfo.c_str());
                    fflush(stdout);
                    auto t0=std::chrono::high_resolution_clock::now();
                    ygg::CuttingEngine engine;
                    engine.cut(billet, ygg::ToolSweepSDF(
                        ygg::ToolType::BALL_END, cutR, 0, 20,
                        {2,(double)cutY,(double)cutZ}, {billetSize[0]-2.0,(double)cutY,(double)cutZ}));
                    auto t1=std::chrono::high_resolution_clock::now();
                    volume = ygg::computeVolume(billet.sdfGrid);
                    cutCount++;
                    layersDirty = true;
                    cutMs=std::chrono::duration<double,std::milli>(t1-t0).count();
                }
                if (rebuildRunning) {
                    ImGui::EndDisabled();
                }
                ImGui::SameLine();
                if (ImGui::Button("Reset")) {
                    billet = ygg::buildBillet(cfg, {0,0,0},
                        {(double)billetSize[0],(double)billetSize[1],(double)billetSize[2]});
                    volume = ygg::computeVolume(billet.sdfGrid);
                    cutCount=0; layersDirty=true;
                }
                if (cutMs>0) ImGui::Text("Cut: %.1f ms", cutMs);

                ImGui::Separator();
                // ─── Tool Display ───
                ImGui::Checkbox("Show Tool", &showTool);
                if (showTool) ImGui::ColorEdit3("Tool Color", toolColor, ImGuiColorEditFlags_NoInputs);

                ImGui::Separator();
                const char* curMode = cfg.mode==ygg::ResolutionConfig::SINGLE_TRACK?"SINGLE":
                    cfg.mode==ygg::ResolutionConfig::DUAL_TRACK?"DUAL":"ATLAS";
                ImGui::Text("Active: %s d_v=%.4f D_v=%.3f N=%d", curMode, cfg.d_v, cfg.D_v, cfg.N);
                ImGui::Text("Voxels: %zu | Mem: %.2f MB",
                    billet.sdfGrid->activeVoxelCount(), billet.sdfGrid->memUsage()/1e6);
                ImGui::EndTabItem();
            }

            // ══ Performance Tab ══
            if (ImGui::BeginTabItem("Performance")) {
                ImGui::Text("FPS: %.1f (%.2f ms/frame)", ImGui::GetIO().Framerate, 1000.f/ImGui::GetIO().Framerate);
                ImGui::Text("SDF Mesh: %d triangles", sdfMesh.count/3);
                ImGui::Text("MacroGrid: %d line segments", macroLines.count/2);
                ImGui::Text("MicroDetail: %d triangles", microDetailMesh.count/3);
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }
#else
        ImGui::Text("OpenVDB not enabled.");
#endif
        ImGui::End();

#if YGG_HAS_OPENVDB
        // ── Voxel Inspector Tooltip ──
        if (inspectorActive && inspValid && billet.sdfGrid) {
            auto& xf = billet.sdfGrid->transform();
            double D_v = xf.voxelSize()[0];
            auto wp = xf.indexToWorld(inspVoxel);
            float sdfVal = billet.sdfGrid->getConstAccessor().getValue(inspVoxel);

            // Count surfels in this voxel
            int totalSurfels=0, activeSurfels=0;
            int leafTotal=0, leafActive=0;
            if (billet.microGrid) {
                openvdb::Coord leafOrigin(inspVoxel.x()&~7, inspVoxel.y()&~7, inspVoxel.z()&~7);
                auto* leaf = billet.microGrid->tree().probeConstLeaf(leafOrigin);
                if (leaf) {
                    auto& as = leaf->attributeSet();
                    auto* actArr = as.get("active");
                    auto ah = actArr ? openvdb::points::AttributeHandle<uint8_t>::create(*actArr) : nullptr;
                    // Count per-voxel
                    openvdb::Coord lc = inspVoxel - leafOrigin;
                    int vIdx = (lc.x()<<6) | (lc.y()<<3) | lc.z();
                    openvdb::Index endI = static_cast<openvdb::Index>(leaf->getValue(vIdx));
                    openvdb::Index startI = (vIdx==0)?0:static_cast<openvdb::Index>(leaf->getValue(vIdx-1));
                    for (openvdb::Index i=startI; i<endI; ++i) {
                        totalSurfels++;
                        if (!ah || ah->get(i)==1) activeSurfels++;
                    }
                    // Leaf totals
                    for (openvdb::Index v=0; v<512; ++v) {
                        openvdb::Index e = static_cast<openvdb::Index>(leaf->getValue(v));
                        openvdb::Index s = (v==0)?0:static_cast<openvdb::Index>(leaf->getValue(v-1));
                        for (openvdb::Index i=s; i<e; ++i) {
                            leafTotal++;
                            if (!ah || ah->get(i)==1) leafActive++;
                        }
                    }
                }
            }

            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().MousePos.x+15, ImGui::GetIO().MousePos.y+15), ImGuiCond_Always);
            ImGui::Begin("##VoxelInspector", nullptr,
                ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|ImGuiWindowFlags_AlwaysAutoResize|
                ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoSavedSettings);
            ImGui::TextColored(ImVec4(1,0.9f,0.3f,1), "Voxel Inspector");
            ImGui::Text("Coord: (%d, %d, %d)", inspVoxel.x(), inspVoxel.y(), inspVoxel.z());
            ImGui::Text("World: (%.3f, %.3f, %.3f) mm", wp.x(), wp.y(), wp.z());
            ImGui::Text("BBox: [%.3f,%.3f,%.3f]-[%.3f,%.3f,%.3f]",
                wp.x(),wp.y(),wp.z(), wp.x()+D_v,wp.y()+D_v,wp.z()+D_v);
            ImGui::Text("D_v: %.4f mm", D_v);
            ImGui::Separator();
            ImGui::TextColored(ImVec4(0.6f,0.8f,1,1), "SDF (MacroGrid)");
            const char* state = sdfVal < -0.01f ? "INSIDE" : sdfVal > 0.01f ? "OUTSIDE" : "SURFACE";
            ImGui::Text("Value: %.4f (%s)", sdfVal, state);
            ImGui::Text("Narrowband: %s", std::abs(sdfVal)<3.0f*(float)D_v?"YES":"NO");
            ImGui::Separator();
            ImGui::TextColored(ImVec4(0.3f,1,0.5f,1), "MicroGrid (this voxel)");
            ImGui::Text("Surfels: %d total | %d active | %d inactive",
                totalSurfels, activeSurfels, totalSurfels-activeSurfels);
            if (totalSurfels>0)
                ImGui::Text("Coverage: %.1f%%", 100.0f*activeSurfels/totalSurfels);
            ImGui::Separator();
            ImGui::TextColored(ImVec4(0.5f,0.7f,1,1), "LeafNode (8x8x8 = 512 voxels)");
            ImGui::Text("Leaf surfels: %d total | %d active", leafTotal, leafActive);
            ImGui::Separator();
            ImGui::TextDisabled("LClick=next depth | Depth=%d", inspDepth);
            ImGui::End();

            // Console dump on Ctrl+Click
            static bool ctrlClickPrev = false;
            bool ctrlClick = glfwGetMouseButton(win,GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS &&
                             glfwGetKey(win,GLFW_KEY_LEFT_CONTROL)==GLFW_PRESS;
            if (ctrlClick && !ctrlClickPrev) {
                printf("\n[Inspector] Voxel (%d,%d,%d) World(%.3f,%.3f,%.3f)\n",
                    inspVoxel.x(),inspVoxel.y(),inspVoxel.z(), wp.x(),wp.y(),wp.z());
                printf("  SDF=%.4f | Surfels: %d active/%d total | Leaf: %d/%d\n",
                    sdfVal, activeSurfels, totalSurfels, leafActive, leafTotal);
                // Sample up to 8 surfel positions
                if (billet.microGrid && totalSurfels > 0) {
                    openvdb::Coord leafOrigin(inspVoxel.x()&~7, inspVoxel.y()&~7, inspVoxel.z()&~7);
                    auto* leaf = billet.microGrid->tree().probeConstLeaf(leafOrigin);
                    if (leaf) {
                        auto& as = leaf->attributeSet();
                        auto* posArr = as.get("P");
                        if (posArr) {
                            auto ph = openvdb::points::AttributeHandle<openvdb::Vec3f>::create(*posArr);
                            openvdb::Coord lc = inspVoxel - leafOrigin;
                            int vIdx = (lc.x()<<6)|(lc.y()<<3)|lc.z();
                            openvdb::Index endI = static_cast<openvdb::Index>(leaf->getValue(vIdx));
                            openvdb::Index startI = (vIdx==0)?0:static_cast<openvdb::Index>(leaf->getValue(vIdx-1));
                            int shown = 0;
                            for (openvdb::Index i=startI; i<endI && shown<8; ++i, ++shown) {
                                auto p = ph->get(i);
                                auto swp = xf.indexToWorld(ygg::Vec3d(
                                    inspVoxel.x()+p.x(), inspVoxel.y()+p.y(), inspVoxel.z()+p.z()));
                                printf("  [%d] pos=(%.4f,%.4f,%.4f)\n", shown, swp.x(),swp.y(),swp.z());
                            }
                            if (totalSurfels > 8) printf("  ... (%d more)\n", totalSurfels-8);
                        }
                    }
                }
            }
            ctrlClickPrev = ctrlClick;
        }
#endif
        ImGui::Render();

        // ═══════════════════════════════════════════════════════════
        // Render passes
        // ═══════════════════════════════════════════════════════════
        int w,h; glfwGetFramebufferSize(win,&w,&h);
        glViewport(0,0,w,h);
        glClearColor(0.12f,0.12f,0.15f,1);
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

        float mvp[16], nm[9];
        buildMVP(cam, w, h, mvp, nm);

        // ── Pass 1: Opaque objects (depth write ON) ──
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);

        // SDF mesh (main workpiece)
        if (showSdfMesh && sdfMesh.count>0) {
            glUseProgram(meshProg);
            glUniformMatrix4fv(glGetUniformLocation(meshProg,"uMVP"),1,GL_FALSE,mvp);
            glUniformMatrix3fv(glGetUniformLocation(meshProg,"uNormalMat"),1,GL_FALSE,nm);
            glUniform3f(glGetUniformLocation(meshProg,"uLightDir"),0.30f,0.51f,0.81f);
            glUniform3f(glGetUniformLocation(meshProg,"uColor"),sdfColor[0],sdfColor[1],sdfColor[2]);
            glUniform1f(glGetUniformLocation(meshProg,"uAlpha"),1.0f);
            sdfMesh.draw();
        }

        // MicroGrid detail surface
        if (showMicroDetail && microDetailMesh.count>0) {
            glUseProgram(meshProg);
            glUniformMatrix4fv(glGetUniformLocation(meshProg,"uMVP"),1,GL_FALSE,mvp);
            glUniformMatrix3fv(glGetUniformLocation(meshProg,"uNormalMat"),1,GL_FALSE,nm);
            glUniform3f(glGetUniformLocation(meshProg,"uLightDir"),0.30f,0.51f,0.81f);
            glUniform3f(glGetUniformLocation(meshProg,"uColor"),microColor[0],microColor[1],microColor[2]);
            glUniform1f(glGetUniformLocation(meshProg,"uAlpha"),1.0f);
            microDetailMesh.draw();
        }

        // MacroGrid wireframe
        if (showMacroGrid && macroLines.count>0) {
            glUseProgram(lineProg);
            glUniformMatrix4fv(glGetUniformLocation(lineProg,"uMVP"),1,GL_FALSE,mvp);
            glUniform3f(glGetUniformLocation(lineProg,"uColor"),macroColor[0],macroColor[1],macroColor[2]);
            glUniform1f(glGetUniformLocation(lineProg,"uAlpha"),1.0f);
            glLineWidth(macroLineWidth);
            macroLines.draw();
        }

#if YGG_HAS_OPENVDB
        // Inspector wireframe boxes + surfel points
        if (inspectorActive && inspValid) {
            glUseProgram(lineProg);
            glUniformMatrix4fv(glGetUniformLocation(lineProg,"uMVP"),1,GL_FALSE,mvp);
            // Yellow box: hovered voxel
            if (inspVoxelWire.count>0) {
                glUniform3f(glGetUniformLocation(lineProg,"uColor"),1.0f,0.9f,0.2f);
                glUniform1f(glGetUniformLocation(lineProg,"uAlpha"),1.0f);
                glLineWidth(2.5f);
                inspVoxelWire.draw();
            }
            // Blue box: leaf node
            if (inspLeafWire.count>0) {
                glUniform3f(glGetUniformLocation(lineProg,"uColor"),0.3f,0.5f,1.0f);
                glUniform1f(glGetUniformLocation(lineProg,"uAlpha"),0.7f);
                glLineWidth(1.5f);
                inspLeafWire.draw();
            }

            // Render surfel points in this voxel (green = active, red = inactive)
            if (billet.microGrid) {
                static GLuint inspPtVAO=0, inspPtVBO=0;
                static int inspPtCount=0;
                static openvdb::Coord lastInspVoxel{-9999,-9999,-9999};
                if (inspVoxel != lastInspVoxel) {
                    lastInspVoxel = inspVoxel;
                    std::vector<float> ptData; // x,y,z per point
                    auto& xf2 = billet.microGrid->transform();
                    openvdb::Coord leafOrig(inspVoxel.x()&~7, inspVoxel.y()&~7, inspVoxel.z()&~7);
                    auto* leaf = billet.microGrid->tree().probeConstLeaf(leafOrig);
                    if (leaf) {
                        auto& as = leaf->attributeSet();
                        auto* posArr = as.get("P");
                        if (posArr) {
                            auto ph = openvdb::points::AttributeHandle<openvdb::Vec3f>::create(*posArr);
                            openvdb::Coord lc = inspVoxel - leafOrig;
                            int vIdx = (lc.x()<<6)|(lc.y()<<3)|lc.z();
                            openvdb::Index endI = static_cast<openvdb::Index>(leaf->getValue(vIdx));
                            openvdb::Index startI = (vIdx==0)?0:static_cast<openvdb::Index>(leaf->getValue(vIdx-1));
                            // Sample: show up to 512 points (skip if more)
                            int step = std::max(1, (int)(endI-startI)/512);
                            for (openvdb::Index i=startI; i<endI; i+=step) {
                                auto p = ph->get(i);
                                auto swp = xf2.indexToWorld(ygg::Vec3d(
                                    inspVoxel.x()+p.x(), inspVoxel.y()+p.y(), inspVoxel.z()+p.z()));
                                ptData.push_back((float)swp.x());
                                ptData.push_back((float)swp.y());
                                ptData.push_back((float)swp.z());
                            }
                        }
                    }
                    inspPtCount = (int)ptData.size()/3;
                    if (!inspPtVAO) { glGenVertexArrays(1,&inspPtVAO); glGenBuffers(1,&inspPtVBO); }
                    glBindVertexArray(inspPtVAO);
                    glBindBuffer(GL_ARRAY_BUFFER, inspPtVBO);
                    glBufferData(GL_ARRAY_BUFFER, ptData.size()*sizeof(float), ptData.data(), GL_DYNAMIC_DRAW);
                    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,3*sizeof(float),(void*)0);
                    glEnableVertexAttribArray(0);
                }
                if (inspPtCount > 0) {
                    glUseProgram(lineProg);
                    glUniformMatrix4fv(glGetUniformLocation(lineProg,"uMVP"),1,GL_FALSE,mvp);
                    glUniform3f(glGetUniformLocation(lineProg,"uColor"),0.1f,1.0f,0.4f);
                    glUniform1f(glGetUniformLocation(lineProg,"uAlpha"),1.0f);
                    glPointSize(5.0f);
                    glBindVertexArray(inspPtVAO);
                    glDrawArrays(GL_POINTS, 0, inspPtCount);
                }
            }
        }
#endif

#if YGG_HAS_OPENVDB
        // Tool visualization (sphere at last path endpoint)
        if (showTool) {
            static float lastR=0, lastY=0, lastZ=0, lastBX=0;
            if (cutR!=lastR || cutY!=lastY || cutZ!=lastZ || billetSize[0]!=lastBX) {
                auto sphere = openvdb::tools::createLevelSetSphere<openvdb::FloatGrid>(
                    float(cutR), openvdb::Vec3f(float(billetSize[0]-2), cutY, cutZ), 0.5f);
                auto tm = ygg::vdbToMesh(sphere);
                toolMesh.upload(tm.vertices.data(), tm.vertices.size()*sizeof(float),
                    tm.indices.data(), tm.indices.size()*sizeof(uint32_t), (int)tm.indices.size());
                lastR=cutR; lastY=cutY; lastZ=cutZ; lastBX=billetSize[0];
            }
            if (toolMesh.count > 0) {
                glEnable(GL_BLEND);
                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                glUseProgram(meshProg);
                glUniformMatrix4fv(glGetUniformLocation(meshProg,"uMVP"),1,GL_FALSE,mvp);
                glUniformMatrix3fv(glGetUniformLocation(meshProg,"uNormalMat"),1,GL_FALSE,nm);
                glUniform3f(glGetUniformLocation(meshProg,"uLightDir"),0.30f,0.51f,0.81f);
                glUniform3f(glGetUniformLocation(meshProg,"uColor"),toolColor[0],toolColor[1],toolColor[2]);
                glUniform1f(glGetUniformLocation(meshProg,"uAlpha"),0.6f);
                toolMesh.draw();
                glDisable(GL_BLEND);
            }
        }
#endif

        // ── Pass 2: Transparent objects (depth test ON, depth write OFF) ──
        if (showOrigBillet && billetOrigMesh.count>0) {
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glDepthMask(GL_FALSE);

            glUseProgram(meshProg);
            glUniformMatrix4fv(glGetUniformLocation(meshProg,"uMVP"),1,GL_FALSE,mvp);
            glUniformMatrix3fv(glGetUniformLocation(meshProg,"uNormalMat"),1,GL_FALSE,nm);
            glUniform3f(glGetUniformLocation(meshProg,"uLightDir"),0.30f,0.51f,0.81f);
            glUniform3f(glGetUniformLocation(meshProg,"uColor"),origColor[0],origColor[1],origColor[2]);
            glUniform1f(glGetUniformLocation(meshProg,"uAlpha"),origAlpha);
            billetOrigMesh.draw();

            glDepthMask(GL_TRUE);
            glDisable(GL_BLEND);
        }

        // ── Pass 3: Overlay (ImGui) ──
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(win);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

#if YGG_HAS_OPENVDB
    if (rebuildThread.joinable()) rebuildThread.join();
#endif

    glfwDestroyWindow(win);
    glfwTerminate();
    return 0;
}
