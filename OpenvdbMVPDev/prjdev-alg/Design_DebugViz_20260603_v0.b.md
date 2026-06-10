# Yggdrasil 调试可视化控制面板设计

**文档编号**：`Design_DebugViz_20260603_v0.b.md`  
**状态**：修订  
**作者**：Duke / Jarvas  
**日期**：2026-06-03  
**依赖**：Phase 2 DualGrid 架构（Sprint 6-7）

---

## 摘要

**核心洞见**：调试 CNC 仿真算法的最大痛点不是渲染效果，而是**在正确的抽象层级看到正确的数据**。设计 4 个调试视图（外观模式/数据检查/算法诊断/性能概览），以最小的渲染管线改动获得最大的调试效率。

**关键技术决策**：
1. MacroGrid `glPolygonMode` 懒人线框 — 零额外数据开销
2. MicroGrid `GL_POINTS` + instanced sphere — 通用且省 GPU
3. 切削界面检测用 `SDF 符号翻转` 着色 — 无需额外几何计算
4. 截面用 `clip plane` shader — 一个 uniform 解决

**适用场景**：Phase 2 双轨切削引擎开发 + 日常算法调试

---

## 1. 现状分析

### 1.1 当前渲染管线

```
FloatGrid → vdbToMesh() → MeshData → uploadMesh → glDrawElements(GL_TRIANGLES)
    |              |             |              |
  (单个)      isovalue/adapt   (单缓冲)     (单色 uniform)
```

| 资产 | 状态 |
|------|------|
| Shader | 单文件 vert/frag，位置+法向+漫反射 |
| Mesh | 单缓冲，每次切削重建 |
| 颜色 | 单色 uniform (`uColor`) |
| 模式 | 仅实心三角面 |
| UI | "Control Panel": Volume / Cuts / Cut Y / Tool R / Execute / Reset |

### 1.2 Phase 2 新增数据源

| 数据 | 来源 | 特性 |
|------|------|------|
| `macroGrid` (FloatGrid) | buildBillet | 现有，体素级 SDF 窄带 |
| `microGrid` (PointDataGrid) | buildBillet/CuttingEngine | **新增**，面元点云（非网格） |
| `cuttingInterface` | CuttingEngine Phase 2 | 动态，刀具与毛坯的交界面元集合 |
| `affectedVoxels` | CuttingEngine Phase 1 | 动态，宏观过滤的受影体素集合 |

### 1.3 最大约束

- `microGrid` 没有等值面，不能用 `volumeToMesh` 三角化
- 面元是离散点集合，天然适合**点云渲染**
- Mesh 提取（`volumeToMesh`）开销 ≈ 数百 ms，不能每帧执行

---

## 2. 调试控制面板设计

### 2.1 面板结构（1 状态栏 + 4 功能 Tab）

```
┌─ ⚙ 仿真状态（始终可见）─────────────────────────────────────────┐
│ 模式: DUAL_TRACK │ d_v=0.10 D_v=6.40 N=64 │ 切削: 第3次 142ms  │
│ FloatGrid: ✅ 8K体素 │ PointGrid: ✅ 12K/12K面元(100%)         │
└──────────────────────────────────────────────────────────────┘
┌─ 🎨 外观模式 ──┬── 📐 数据检查 ──┬── 🔬 算法诊断 ──┬── 📊 性能 ──┐
│               │                │                │              │
│ 渲染模式       │ 截面平面       │ 切削界面着色    │ FPS          │
│ 网格可见性     │ 体素/面元拣选  │ 精度分级着色    │ 内存统计     │
│ 颜色/线宽/点径 │ 面元属性显示   │ 受影体素范围    │ Mesh时间     │
│ LOD 控制       │ SDF 热力图     │ 时间序列回放    │ 面元统计     │
│ 背景/光照      │ 坐标轴         │                │              │
└────────────────┴────────────────┴────────────────┴──────────────┘

### 2.2 状态概览栏（始终可见）

在所有 Tab 上方固定显示当前仿真配置和核心状态，无需切换 Tab 即可一目了然。

```cpp
// ── 仿真模式 ──
ResolutionConfig::Mode mode;          // SINGLE_TRACK / DUAL_TRACK / ATLAS_REGION
double d_v, D_v;                      // 面元间距 / 体素尺寸
int    N;                              // 精细面元密度因子 (D_v/d_v)

// ── 毛坯参数 ──
Vec3d billetOrigin, billetDims;       // 毛坯原点/尺寸 (mm)
double billetVolume;                   // 当前体积 (mm³)

// ── 切削状态 ──
int    cutCount;                      // 已执行切削次数
double lastCutDurationMs;             // 最近一次切削耗时

// ── 网格状态 ──
bool   macroGridReady;                // FloatGrid 是否就绪
bool   microGridReady;                // PointDataGrid 是否就绪 (仅 DUAL_TRACK)
size_t macroActiveVoxels;             // FloatGrid 活跃体素数
size_t microSurfelCount;              // 面元总数
size_t microActiveSurfels;            // 活跃面元数
```

```
┌──────────────────────────────────────────────────────────────┐
│ ⚙ 仿真状态                                                   │
│──────────────────────────────────────────────────────────────│
│ 模式: DUAL_TRACK  │  d_v=0.10mm  D_v=6.40mm  N=64          │
│ 毛坯: [0,0,0] → [300,200,100] mm  │  体积: 5,984,231 mm³   │
│ 切削: 第 3 次  │  上次耗时: 142.3 ms                        │
│──────────────────────────────────────────────────────────────│
│ FloatGrid: ✅ 就绪  │  活跃体素: 8,421                      │
│ PointGrid: ✅ 就绪  │  面元: 12,544 / 活跃: 12,544 (100%)   │
└──────────────────────────────────────────────────────────────┘
```

> **实现**: 作为 ImGui 主面板第一行，始终展开。数据源：`BilletModel.config` / `BilletModel.sdfGrid->activeVoxelCount()` / `MemoryStats`（T6.1）。

### 2.3 「外观模式」面板 🎨

这是日常最常用的面板，控制"看到什么、怎么看"。

```cpp
// ── 渲染模式 ──
enum class RenderMode { Solid, Wireframe, Points, SolidWireOverlay };
RenderMode mode = RenderMode::Solid;

// ── 网格可见性 ──
bool showMacroGrid = true;   // FloatGrid 等值面网格
bool showMicroGrid = false;  // PointDataGrid 面元点云
bool showCutSurface = false; // 切削界面高亮

// ── 颜色 (ImGui::ColorEdit3) ──
float macroColor[3] = {0.7f, 0.75f, 0.80f};  // 钢灰 — 默认毛坯色
float microColor[3] = {1.0f, 0.55f, 0.10f};  // 暖橙 — 面元点云
float cutColor[3]   = {0.20f, 0.85f, 0.35f}; // 翠绿 — 切削界面
float wireColor[3]  = {0.30f, 0.30f, 0.35f}; // 深灰 — 线框

// ── 线宽 / 点径 ──
float wireWidth = 1.5f;  // 线框线宽 (px)
float pointSize = 4.0f;  // 面元点径 (px)

// ── LOD 控制 ──
float macroAdaptivity = 0.0f;  // volumeToMesh adaptivity (0=全精度, 越大越简化)
float macroIsovalue   = 0.0f;  // 等值面偏移 (调试窄带用)
bool  lodAuto = true;          // 自动LOD: 跟随相机距离动态调整 adaptivity

// ── 背景 ──
float bgColor[3] = {0.15f, 0.15f, 0.18f};

// ── 菜单项 ──
┌──────────────────────────────────────┐
│ 🎨 外观模式                          │
│──────────────────────────────────────│
│ 渲染模式: [Solid ▾]                  │
│   ○ Solid   ○ Wireframe              │
│   ○ Points  ● Solid+WireOverlay      │
│──────────────────────────────────────│
│ ☑ MacroGrid  [钢灰 ■■■□□□□]         │
│ ☐ MicroGrid  [暖橙 ■■■■■□□]         │
│ ☐ CutSurface [翠绿 ■■■■■■□]         │
│──────────────────────────────────────│
│ 线框线宽: [====|========] 1.5 px     │
│ 面元点径: [====|========] 4.0 px     │
│──────────────────────────────────────│
│ LOD Adaptivity: [=|=========] 0.00   │
│ ☑ 自动LOD (跟随视距)                 │
│ 等值面偏移:    [=|=========] 0.00    │
│──────────────────────────────────────│
│           [Apply LOD] [Reset]        │
└──────────────────────────────────────┘
```

> **Solid+WireOverlay 实现**: 同一网格渲染两次——先 `glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)` 画线框（略偏移深度），再 `GL_FILL` 画实体面。**零额外内存/数据开销**。

### 2.4 「数据检查」面板 📐

用于切片观察内部结构、逐面元查看属性值。

```cpp
// ── 截面裁剪 (Shader clip plane) ──
bool  clipEnabled = false;
int   clipAxis = 2;           // 0=X, 1=Y, 2=Z
float clipPosition = 0.5f;    // 归一化位置 [0,1]
bool  clipInvert = false;     // 翻转裁剪方向

// ── 体素/面元拣选 ──
bool  pickMode = false;       // 鼠标点击拣选
struct PickedItem {
    bool  valid = false;
    Vec3i voxelCoord;          // 所属体素坐标
    int   surfelIndex;         // 面元序号 (-1 = 体素级)
    Vec3f position;
    Vec3f normal;
    uint8_t precision;
    bool  active;
};

// ── SDF 热力图 ──
bool  sdfHeatmap = false;     // 用颜色映射 SDF 值
float sdfHeatMin = -3.0f;     // 蓝(内部)
float sdfHeatMid = 0.0f;      // 白(表面)
float sdfHeatMax = 3.0f;      // 红(外部)

// ── 面元属性显示 ──
bool showSurfelNormals = false;  // 面元法向箭头
float normalScale = 1.0f;        // 法向箭头长度

// ── 坐标轴 ──
bool showAxes = true;
float axesSize = 5.0f;

// ── 菜单项 ──
┌──────────────────────────────────────┐
│ 📐 数据检查                          │
│──────────────────────────────────────│
│ ☑ 截面裁剪                          │
│   轴向: [X] [Y] ●[Z]                │
│   位置: [======|====] 50%            │
│   ☐ 翻转裁剪方向                     │
│──────────────────────────────────────│
│ ☐ SDF 热力图                        │
│   范围: 蓝[-3.0] 白[0.0] 红[+3.0]   │
│──────────────────────────────────────│
│ ☐ 面元法向箭头  长度: [===|===] 1.0 │
│──────────────────────────────────────│
│ ☑ 坐标轴  大小: [======|====] 5.0   │
│──────────────────────────────────────│
│ 拣选信息: (无)                       │
│   Voxel: (—,—,—)                    │
│   Surfel: —  精度: —  活跃: —       │
│   Pos: (—,—,—)                      │
│   Normal: (—,—,—)                   │
└──────────────────────────────────────┘
```

### 2.5 「算法诊断」面板 🔬

Phase 2 双轨切削的专用调试视图。

```cpp
// ── 切削界面着色 ──
bool colorCutSurface = false;  // SDF 符号翻转体素用 CutColor 着色
// 检测逻辑: 遍历 affectedVoxels, 比较切削前后 SDF 值的符号变化
//   signBefore = sign(sdfGrid->getValue(voxel))
//   signAfter  = sign(sdfGrid->getValue(voxel) after cut)
//   若 signBefore != signAfter → 该体素的三角面元标记为 cutColor

// ── 精度分级着色 ──
enum class PrecisionColorMode { None, ByType, ByAge };
PrecisionColorMode precColorMode = PrecisionColorMode::None;
// ByType: COARSE=暖橙, FINE=翠绿
// ByAge:  按切削次数渐变 (蓝→红)

// ── 活跃状态 ──
bool showOnlyActive = true;   // 仅显示 active group 内的面元
bool ghostInactive = false;   // 被切除面元以半透明/细线显示

// ── 受影体素范围 ──
bool showAffectedRange = false;  // 框出刀具影响区 (线框包围盒)
float affectedRangeColor[3] = {1.0f, 0.85f, 0.10f}; // 金黄色

// ── 切削时间序列 ──
int  cutHistoryIndex = -1;  // -1 = 当前, 0..N = 历史第N次切削
std::vector<BilletModel> cutHistory;  // 最多保留 20 帧快照

// ── 菜单项 ──
┌──────────────────────────────────────┐
│ 🔬 算法诊断                          │
│──────────────────────────────────────│
│ ☑ 切削界面着色 [翠绿 ■■■■■□]        │
│──────────────────────────────────────│
│ 精度分级: ○无  ●按类型  ○按切削次数  │
│   COARSE=暖橙  FINE=翠绿             │
│──────────────────────────────────────│
│ 面元筛选:                            │
│   ☑ 仅显示活跃面元                   │
│   ☐ Ghost模式(被切面元半透明)         │
│   Ghost不透明度: [===|=====] 0.25    │
│──────────────────────────────────────│
│ ☐ 受影体素范围 [金黄 ■■■■■■□]        │
│──────────────────────────────────────│
│ 切削历史: [←] 5/12 [→]               │
│   ☐ 自动记录历史                     │
│            [Clear History]           │
└──────────────────────────────────────┘
```

### 2.6 「性能」面板 📊

```cpp
// ── 实时统计 ──
struct PerfStats {
    float fps;
    float frameTimeMs;
    float meshExtractMs;     // volumeToMesh 耗时
    float uploadMs;          // GPU upload 耗时

    // MemoryStats (来自 T6.1)
    size_t floatGridBytes;
    size_t pointGridBytes;
    size_t totalBytes;
    size_t pointCount;
    size_t activePointCount;
    size_t leafNodeCount;
};

// ── 菜单项 ──
┌──────────────────────────────────────┐
│ 📊 性能                              │
│──────────────────────────────────────│
│ FPS:        60.0  (16.7 ms)         │
│ Mesh提取:   142.3 ms                 │
│ GPU上传:     2.1 ms                  │
│──────────────────────────────────────│
│ 内存:                               │
│   FloatGrid:   2.3 MB               │
│   PointGrid:   1.8 MB               │
│   Total:       4.1 MB               │
│──────────────────────────────────────│
│ 面元:                               │
│   总数:      12,544                  │
│   活跃:      12,544 (100%)           │
│   叶节点:        52                  │
│   平均/叶:    241.2                  │
└──────────────────────────────────────┘
```

---

## 3. 渲染管线扩展设计

### 3.1 新增 Shader 需求

当前 shader 仅支持 single-color diffuse。需要扩展到：

| Shader 特性 | 用途 | 实现方式 |
|-------------|------|----------|
| 逐面元颜色 | 精度/活跃/切削界面着色 | `glVertexAttribPointer` 附加颜色属性 |
| 线框模式 | Wireframe overlay | `glPolygonMode` + depth offset |
| 点精灵 | microGrid 点云 | `GL_POINTS` + `gl_PointSize` + `glEnable(GL_PROGRAM_POINT_SIZE)` |
| 截面裁剪 | 裁剪平面 | `discard` when `dot(pos, clipPlane.xyz) > clipPlane.w` |
| SDF 热力图 | 颜色映射 | 将 SDF 值作为颜色属性 → fragment shader 查色表 |

### 3.2 Mesh 数据结构扩展

```cpp
struct MeshData {
    std::vector<float> vertices;     // x,y,z,nx,ny,nz (不变)
    std::vector<uint32_t> indices;   // (不变)

    // Phase 2 新增: 逐顶点调试属性
    std::vector<float> sdfValues;         // 每个顶点的 SDF 值 (热力图用)
    std::vector<uint8_t> vertexFlags;    // bit0=cutSurface, bit1=affected
};

// 面元点云数据 (microGrid 可视化)
struct SurfelPointCloud {
    std::vector<float> positions;    // x,y,z (世界坐标)
    std::vector<float> normals;      // nx,ny,nz
    std::vector<float> colors;       // r,g,b (预计算，精度/活跃态着色)
    std::vector<uint8_t> flags;      // bit0:active, bit1:COARSE/FINE
    size_t count;
};
```

### 3.3 渲染循环结构

```
每帧:
  1. ImGui 更新 → 读取调试参数
  2. 检查是否需要重建 mesh (LOD 变化 / 切削后 / 手动触发)
  3. 检查是否需要提取面元点云 (microGrid 变化 / 点径变化)
  4. 渲染:
     if showMacroGrid:
       if mode == Solid or SolidWireOverlay:
         glPolygonMode(GL_FILL)
         drawMacro(cutSurface colored if enabled)
       if mode == Wireframe or SolidWireOverlay:
         glPolygonMode(GL_LINE)
         glLineWidth(wireWidth)
         drawMacro(wireColor)
     if showMicroGrid:
       glPointSize(pointSize)
       drawMicroPoints()
     if showAffectedRange:
       drawBoundingBox(affectedRange, affectedRangeColor)
```

### 3.4 性能策略

| 操作 | 频率 | 策略 |
|------|------|------|
| `volumeToMesh` | 手动触发 / 切削后 / LOD 变更 | 缓存结果，不清除即不重建 |
| PointDataGrid 点云提取 | 切削后 | 遍历叶节点提取坐标，~1ms |
| 切削界面检测 | 切削后一次 | 比较 SDF sign，标记顶点 flag |
| GPU upload | 仅数据变化时 | `glBufferSubData` 增量更新 |

---

## 4. 实现路线

### Phase 2a: 最小可用 (Sprint 6 并行)

配合 Sprint 6 开发，先做最关键的 3 个功能：

| 优先级 | 功能 | 理由 |
|--------|------|------|
| P0 | Wireframe overlay + 双色渲染 | 验证 macroGrid/microGrid 共享 Transform 是否对齐 |
| P0 | microGrid 点云渲染 (GL_POINTS) | Sprint 6 核心产出可视化 |
| P0 | 性能面板内存统计 | 连接 T6.1 MemoryStats |
| P1 | 切削界面着色 | Sprint 7 开发必需 |

### Phase 2b: 完整面板 (Sprint 7 同步)

| 优先级 | 功能 |
|--------|------|
| P1 | 截面裁剪 plane |
| P1 | 精度分级着色 / 活跃态过滤 |
| P1 | 受影体素范围 |
| P2 | SDF 热力图 |
| P2 | 拣选 / 面元属性查看 |
| P2 | 切削历史回放 |
| P3 | 自动 LOD |

### Phase 2c: 打磨

| 优先级 | 功能 |
|--------|------|
| P3 | 背景渐变 / 光照调节 |
| P3 | 导出截图 (PNG) |
| P3 | 预设配置保存/加载 |

---

## 5. 文件清单 (预估)

| 文件 | 作用 |
|------|------|
| `yggdrasil/app/DebugViz.h` | 调试状态 struct + 枚举定义 |
| `yggdrasil/app/DebugPanel.cpp` | ImGui 4个Tab的 UI 实现 |
| `yggdrasil/app/MeshBuilder.cpp` | 扩展 vdbToMesh：附加 SDF/flag 属性 |
| `yggdrasil/app/SurfelCloud.cpp` | PointDataGrid → SurfelPointCloud 提取 |
| `yggdrasil/app/shaders/debug.vert` | 升级版 vertex shader |
| `yggdrasil/app/shaders/debug.frag` | 升级版 fragment shader（多色+热力图+裁剪） |
| `yggdrasil/app/main.cpp` | 修改：集成多 pass 渲染 + ImGui tabs |

---

## 修订记录

| 版本 | 日期 | 修订内容 |
|------|------|----------|
| v0.a | 2026-06-03 | 初稿创建 |
| v0.b | 2026-06-03 | §2.2 新增「状态概览栏」：仿真模式/双轨状态/毛坯参数/切削次数/网格就绪状态始终可见 |
