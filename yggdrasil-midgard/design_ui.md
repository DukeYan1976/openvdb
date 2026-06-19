# Yggdrasil-Midgard 多轴刀路仿真 — 集成界面设计方案

> 版本：v1.0（Final） | 日期：2026-06-19
> 状态：**已审查通过**
> 原则：**简洁就是美**。

---

## 修订记录

| 版本 | 时间 | 修改内容 | 触发原因 |
|------|------|----------|----------|
| v0.1 | 11:00 | 初始设计：三窗口（Settings/SimulationControl/Output） | 用户初始需求 |
| v0.2 | 11:26 | 增加单步模式、顶部居中布局、Path区块前置、Demo场景、Debug窗口 | 用户反馈：位置调整、单步执行、.cls格式、IT-1 Demo |
| v0.3 | 11:34 | 调整布局：Settings+Debug左侧，SimControl顶部右侧，Output右侧；增加布局持久化 | 用户反馈：窗口停靠位置调整 |
| v0.4 | 12:13 | 精简内容，增加字体DPI自适应，明确Speed与feedrate关系 | Loop Engineering 自检 |
| v0.5 | 13:07 | 增加算法调试显示接口（IDebugDisplay） | 用户反馈：算法调试可视化需求 |
| **v1.0 Final** | 13:47 | 审查通过，形成正式设计文档 1.0 | 用户确认 |

---

## 审查意见与处理

| # | 审查意见 | 处理状态 | 修改位置 |
|---|----------|----------|----------|
| 1 | 精度 `t` 放在 Path 区块 | ✅ 已采纳 | 3.1 Path |
| 2 | 需要单步执行，刀具路径一段一段执行 | ✅ 已采纳 | 4. Simulation Control - Step模式 |
| 3 | UI字体根据屏幕分辨率自适应 | ✅ 已采纳 | 2.1 布局 - 字体说明 |
| 4 | 支持 `*.cls` 格式 | ✅ 已采纳 | 3.1 Path - Load .cls |
| 5 | 预设Demo场景，参考IT-1测试案例 | ✅ 已采纳 | 3.1 Path - Load Demo |
| 6 | 增加专门Debug控制窗口，简化Voxel Inspector | ✅ 已采纳 | 6. Debug窗口 |
| 7 | 仿真控制窗口默认在顶部右侧 | ✅ 已采纳 | 2.1 布局 |
| 8 | Settings和Debug停靠左侧，Output右侧 | ✅ 已采纳 | 2.1 布局 |
| 9 | 用户调整后布局下次启动保留 | ✅ 已采纳 | 2.1 布局 - imgui.ini |
| 10 | Path区块放在Settings最前面 | ✅ 已采纳 | 3. Settings - Path默认展开 |
| 11 | 设计抽象Display接口供算法调试 | ✅ 已采纳 | 7. 算法调试显示接口 |
| 12 | Debug窗口缺少设计说明 | ✅ 已采纳 | 6. Debug窗口（本节补全） |
| 13 | 增加 IProgressReporter 接口，支持算法进度显示 | ✅ 已采纳 | 8. 算法进度报告接口（新增） |

---

## 1. 技术参考

借鉴 `yggdrasil/app/main.cpp` 的渲染栈（OpenGL 3.3 + GLFW + GLAD + ImGui），不继承其 UI 功能。

---

## 2. 布局（默认停靠，用户调整持久化）

```
┌──────────────────────────────────────────────────────────────┐
│  3D Viewport                                                 │
│                                                              │
│  ┌─ Settings ────────┐  ┌─ Simulation Control ─────────────┐ │
│  │ ▼ Path            │  │ ▶ ⏸ ⏹ ⏮ ⏭ [████░░] 60%        │ │
│  │ ▶ Billet          │  │ Step[x] Speed[===]1x             │ │
│  │ ▶ Tool            │  └──────────────────────────────────┘ │
│  └────────────────────┘  ┌─ Output ─────────────────────────┐ │
│  ┌─ Debug ────────────┐  │ [11:00:01] [Build]...            │ │
│  │ [x] Inspector      │  │ [11:00:02] [Cut 744/1240]        │ │
│  │ Depth: 0 [+][-]    │  │ ...                              │ │
│  │ [x] RtDebug On     │  │ ───────────────────────────────  │ │
│  │ Coord: (123,45,67) │  │ Voxels: 47,726  Mem: 128 MB      │ │
│  └────────────────────┘  └──────────────────────────────────┘ │
└──────────────────────────────────────────────────────────────┘
```

| 窗口 | 默认位置 | 持久化 |
|------|----------|--------|
| Settings | 左上 | imgui.ini |
| Simulation Control | 顶部右侧（Settings 右侧） | imgui.ini |
| Output | 右侧 | imgui.ini |
| Debug | 左下（Settings 下方） | imgui.ini |

**字体**：ImGui 默认字体根据 `io.FontGlobalScale` 自动适配屏幕 DPI（Retina 2x，普通 1x）。

---

## 3. Settings 窗口

Path 区块默认展开，其余折叠。

### 3.1 Path

```
┌─ Path ─────────────────────┐
│ [x] Show Path [ ] Axis     │
│ Col [■] [■]                │
│ [Load .cls...]  test.cls   │
│ Segments: 1,240            │
│ ─────────────────          │
│ Tolerance t: [0.05] mm     │
│ d_v: 0.0500  D_v: 0.500    │
│ N: 4  Mode: DUAL           │
│ [Recalculate]              │
│ [Load Demo]                │
└───────────────────────────┘
```

| 控件 | 说明 |
|------|------|
| Show Path / Axis | 显示开关 |
| Load .cls... | 加载刀路 |
| Tolerance t | 继承 CAM，可修改 |
| Recalculate | 重算分辨率配置 |
| **Load Demo** | 加载 IT-1 预设场景 |

**Demo 场景（IT-1 单切精度验证）**：
- 毛坯：50×50×20 mm Box
- 刀具：球头刀 R=5mm
- 刀路：X 单段 (0,0,18)→(50,0,18)，切深 2mm
- 容差：t=0.01mm

### 3.2 Billet（折叠）

Show, Alpha, Color, Type[Box/Cylinder/Mesh], Size[3], Origin[3], [Apply]

### 3.3 Tool（折叠）

Show, Color, Type[Ball/Flat/Bull], R, r, H

---

## 4. Simulation Control 窗口

```
┌─ Simulation Control ───────────────────────────────────────┐
│ ▶ ⏸ ⏹ ⏮ ⏭  [████████░░░░] 60%  744/1240  [x]Step  Speed[===]1x │
└────────────────────────────────────────────────────────────┘
```

| 控件 | 说明 |
|------|------|
| ▶ ⏸ ⏹ ⏮ ⏭ | 播放/暂停/停止/进退 |
| Progress | 进度条 + 百分比 |
| Segment | 当前/总段 |
| Step | 单步开关。ON：按 ▶ 执行 1 个 move 段后暂停 |
| Speed | 0.25x ~ 10x，连续模式下 feedrate × speedMultiplier |

**状态机**：
- IDLE → Play → RUNNING → Pause → PAUSED → Stop → IDLE
- RUNNING 时 Settings 灰显

---

## 5. Output 窗口

```
┌─ Output ──────────────────┐
│ [11:00:01] [Build] Billet │
│ [11:00:02] [Cut 744/1240] │
│ ...                       │
│ [Clear] [Save]            │
│ ───────────────────────── │
│ Voxels: 47,726            │
│ Surfels: 12,340           │
│ Mem: 128 MB               │
│ Volume: 12,450 mm³        │
│ Last: 28 ms  Avg: 25 ms   │
└───────────────────────────┘
```

- 日志：自动滚动，500 行上限，颜色分级（Build蓝/Cut绿/Error红）
- 统计：每段切削后更新

---

## 6. Debug 窗口

> 左侧 Settings 下方，简化 Inspector + RtDebug 控制。
> 与 `RtDebugSys` 联动：Inspector 显示 3D 拾取信息，RtDebug 控制算法日志级别。

```
┌─ Debug ────────────────────┐
│ [x] Voxel Inspector        │
│   Depth: 0  [+][-]         │
│ ─────────────────          │
│ [x] RtDebug                │
│ Level: [On ▼]              │
│ ─────────────────          │
│ Coord: (123, 45, 67)       │
│ SDF: -0.0234               │
│ Surfels: 4                 │
└───────────────────────────┘
```

### 6.1 Voxel Inspector（简化）

| 控件 | 说明 |
|------|------|
| Voxel Inspector | 总开关。关闭时 3D 视图不拾取 voxel |
| Depth +/- | 循环深度。鼠标悬停 voxel 时，用 +/- 切换拾取深度 |
| Coord | 当前 voxel 的网格坐标（只读） |
| SDF | 有符号距离值（只读） |
| Surfels | 该 voxel 内 surfel 数量（只读） |

**交互**：鼠标悬停 3D 视图 → 自动 ray pick → 显示 voxel 信息。无需左键点击，用 +/- 按钮循环深度。

### 6.2 RtDebug 控制

| 控件 | 说明 |
|------|------|
| RtDebug | 总开关。映射 `Debugger::Activate/Deactivate` |
| Level | Off / On / Verbose。映射 `_debugLevel` |

**联动逻辑**：
- RtDebug OFF → `DEBUG_SECTION` 宏不执行，算法不输出调试信息
- RtDebug ON → `DEBUG_SECTION` 执行，算法输出到 `DebugInfo.txt`
- Verbose → 额外输出 `DEBUG_SECTION_VERBOSE` 内容

### 6.3 与算法调试显示接口的协作

Debug 窗口是**控制面板**，算法调试显示接口（`IDebugDisplay`）是**数据通道**：
- Debug 窗口的 Inspector 开关，控制是否拾取 voxel 并显示
- RtDebug 的 tag 系统，控制算法中哪些中间数据通过 `g_debugDisplay` 绘制到 3D 视图
- 两者独立：可以只开 Inspector 看 SDF，或只开 RtDebug 看算法输出，或同时开启

---

## 7. 算法调试显示接口（核心设计）

### 7.1 设计目标

- 在 `core` 中定义抽象接口，算法代码全局可用
- 集成 UI 环境下，实现该接口并注入引擎
- 无 UI 时指针为 `nullptr`，显示代码零开销
- 与 `RtDebugSys` 配合，按 tag 条件显示中间数据

### 7.2 接口定义（`interface/DebugDisplay.h`）

```cpp
#pragma once
#include "Types.h"
#include <vector>
#include <memory>

namespace midgard {

/// 调试显示抽象接口
/// 由 UI 层实现，通过指针注入引擎。nullptr = 无显示
class IDebugDisplay {
public:
    virtual ~IDebugDisplay() = default;

    /// 批量 BBox（如 LeafNode 集合、任务分块）
    virtual void drawBBoxes(const std::vector<openvdb::BBoxd>& boxes,
                            uint32_t color = 0xFF00FFFF,  // 默认黄色
                            float lineWidth = 1.0f) = 0;

    /// 单个 BBox（如当前聚焦 voxel、刀具包围盒）
    virtual void drawBBox(const openvdb::BBoxd& box,
                          uint32_t color = 0xFF00FFFF,
                          float lineWidth = 1.5f) = 0;

    /// 点+法向集（如 surfel 点云、采样点）
    virtual void drawPoints(const std::vector<Vec3d>& points,
                            const std::vector<Vec3d>& normals,
                            uint32_t color = 0xFF3366FF,   // 默认蓝色
                            float pointSize = 3.0f) = 0;

    /// 无 normals 的点云
    virtual void drawPoints(const std::vector<Vec3d>& points,
                            uint32_t color = 0xFF3366FF,
                            float pointSize = 3.0f) = 0;

    /// 三角片数据（如局部重建 mesh、刀具 mesh）
    virtual void drawTriangles(const std::vector<Vec3d>& vertices,
                               const std::vector<uint32_t>& indices,
                               uint32_t color = 0xFF66CC33,  // 默认绿色
                               float alpha = 0.8f) = 0;

    /// 线段（如刀轴方向、法线指示）
    virtual void drawLines(const std::vector<Vec3d>& points,  // 每 2 个点一条线段
                           uint32_t color = 0xFFFFFFFF,
                           float lineWidth = 1.0f) = 0;

    /// 箭头（如刀轴方向、梯度方向）
    virtual void drawArrow(const Vec3d& from, const Vec3d& to,
                           uint32_t color = 0xFFFFFFFF,
                           float lineWidth = 1.5f,
                           float headSize = 2.0f) = 0;

    /// 清除该 tag 的所有显示数据（按帧或按调用清理）
    virtual void clear(const std::string& tag = "") = 0;
};

/// 全局显示指针。UI 初始化时注入，无 UI 时为 nullptr
extern std::shared_ptr<IDebugDisplay> g_debugDisplay;

} // namespace midgard
```

### 7.3 引擎中的使用模式

```cpp
// core/MacroCut.cpp 示例
void MacroCut::classifyVoxels(IPWState& ipw, const ToolSweptSDF& tool) {
    // ... 正常计算 ...

    DEBUG_SECTION(MACRO_CUT) {
        // 仅当 RtDebug tag 激活且显示接口存在时显示
        if (g_debugDisplay) {
            std::vector<openvdb::BBoxd> taskBoxes;
            for (const auto& task : tasks) {
                taskBoxes.push_back(task.aabb);
            }
            g_debugDisplay->clear("macro_tasks");
            g_debugDisplay->drawBBoxes(taskBoxes, 0xFF00FFFF, 1.0f);
        }
    }
}
```

### 7.4 UI 层实现（`app/renderers/DebugDisplayImpl.h`）

```cpp
#pragma once
#include "core/DebugDisplay.h"
#include <map>
#include <string>

namespace midgard {

/// OpenGL 实现：将调试数据缓存为 GPU buffer，每帧渲染
class DebugDisplayImpl : public IDebugDisplay {
public:
    void drawBBoxes(const std::vector<openvdb::BBoxd>& boxes,
                    uint32_t color, float lineWidth) override;
    void drawBBox(const openvdb::BBoxd& box,
                  uint32_t color, float lineWidth) override;
    void drawPoints(const std::vector<Vec3d>& points,
                    const std::vector<Vec3d>& normals,
                    uint32_t color, float pointSize) override;
    void drawPoints(const std::vector<Vec3d>& points,
                    uint32_t color, float pointSize) override;
    void drawLines(const std::vector<Vec3d>& points,
                   uint32_t color, float lineWidth) override;
    void drawArrow(const Vec3d& from, const Vec3d& to,
                   uint32_t color, float lineWidth, float headSize) override;
    void drawTriangles(const std::vector<Vec3d>& vertices,
                       const std::vector<uint32_t>& indices,
                       uint32_t color, float alpha) override;
    void clear(const std::string& tag) override;

    /// 由 SceneRenderer 每帧调用
    void render(const float mvp[16]);

private:
    struct Batch {
        GLuint vao = 0, vbo = 0, ebo = 0;
        int count = 0;
        uint32_t color = 0xFFFFFFFF;
        float lineWidth = 1.0f;
        float pointSize = 3.0f;
        float alpha = 1.0f;
        GLenum primitive = GL_LINES;
    };
    std::map<std::string, std::vector<Batch>> m_taggedBatches;
};

} // namespace midgard
```

### 7.5 初始化与注入（`app/main.cpp`）

```cpp
#include "core/DebugDisplay.h"
#include "renderers/DebugDisplayImpl.h"

int main() {
    // ... GLFW/ImGui 初始化 ...

    // 创建显示实现并注入引擎
    auto display = std::make_shared<midgard::DebugDisplayImpl>();
    midgard::g_debugDisplay = display;

    // ... 主循环 ...
    while (!glfwWindowShouldClose(win)) {
        // ... 仿真计算 ...

        // 渲染场景（含调试层）
        sceneRenderer.render(cam);
        display->render(mvp);  // 在场景之后渲染调试数据

        // ... ImGui ...
    }

    midgard::g_debugDisplay = nullptr;
    return 0;
}
```

### 7.6 零开销保证

- `g_debugDisplay` 为 `nullptr` 时，`DEBUG_SECTION` 内代码不执行
- 无 UI 环境下，不链接 `DebugDisplayImpl`，不创建 OpenGL 资源
- 发布模式可定义 `NDEBUG` 完全移除 `DEBUG_SECTION` 宏

---

## 8. 算法进度报告接口（新增）

### 8.1 设计目标

- 算法执行过程中向 UI 报告进度，与 Simulation Control 窗口的进度条联动
- 支持任务初始化（总工作量统计）、过程中推进、完成报告
- 无 UI 时接口为 `nullptr`，进度代码零开销
- 典型场景：Phase 0 体素分类、Phase 1 任务生成、Phase 2 四叉树采样等长耗时操作

### 8.2 接口定义（`interface/ProgressReporter.h`）

```cpp
#pragma once
#include <string>
#include <memory>

namespace midgard {

/// 进度报告抽象接口
/// 由 UI 层实现，通过指针注入引擎。nullptr = 不报告
class IProgressReporter {
public:
    virtual ~IProgressReporter() = default;

    /// 开始一个任务，报告总工作量
    /// @param taskName  任务名称（如 "Phase 0: MacroCut"）
    /// @param totalWork 总工作量（如 voxel 数、任务数、采样点数）
    /// @param unit      单位描述（如 "voxels", "tasks", "samples"）
    virtual void beginTask(const std::string& taskName,
                           int64_t totalWork,
                           const std::string& unit = "") = 0;

    /// 推进进度
    /// @param completed 已完成工作量
    /// @param message   当前状态信息（可选，如 "processing leaf 45/128"）
    virtual void update(int64_t completed,
                        const std::string& message = "") = 0;

    /// 增加已完成量（增量更新）
    /// @param delta   本次完成量
    /// @param message 当前状态信息（可选）
    virtual void advance(int64_t delta = 1,
                         const std::string& message = "") = 0;

    /// 任务完成
    /// @param message 完成信息（如 "classified 2911 voxels in 18ms"）
    virtual void endTask(const std::string& message = "") = 0;

    /// 取消当前任务（用户点击 Stop 时调用）
    virtual void cancel() = 0;

    /// 是否已请求取消（算法应定期检查并提前退出）
    virtual bool isCancelled() const = 0;
};

/// 全局进度报告指针。UI 初始化时注入，无 UI 时为 nullptr
extern std::shared_ptr<IProgressReporter> g_progressReporter;

} // namespace midgard
```

### 8.3 引擎中的使用模式

```cpp
// core/MacroCut.cpp 示例
void MacroCut::classifyVoxels(IPWState& ipw, const ToolSweptSDF& tool) {
    auto* reporter = g_progressReporter.get();
    
    // 1. 统计总工作量
    int64_t totalVoxels = ipw.macroGrid->activeVoxelCount();
    if (reporter) {
        reporter->beginTask("Phase 0: MacroCut", totalVoxels, "voxels");
    }
    
    // 2. 遍历处理
    int64_t processed = 0;
    for (auto iter = ipw.macroGrid->cbeginValueOn(); iter; ++iter) {
        // ... 分类逻辑 ...
        
        // 每 64 个 voxel 报告一次进度（避免频繁调用）
        if (++processed % 64 == 0 && reporter) {
            reporter->advance(64, "classifying voxels...");
            
            // 检查用户是否点击了取消
            if (reporter->isCancelled()) {
                // 清理并退出
                if (reporter) reporter->cancel();
                return;
            }
        }
    }
    
    // 3. 完成报告
    if (reporter) {
        reporter->endTask("classified " + std::to_string(processed) + " voxels");
    }
}
```

### 8.4 UI 层实现（`app/windows/SimControlWindow.h`）

```cpp
#pragma once
#include "core/ProgressReporter.h"
#include <atomic>

namespace midgard {

/// Simulation Control 窗口实现 IProgressReporter
/// 进度更新直接反映在 ImGui 进度条上
class SimControlWindow : public IProgressReporter {
public:
    void beginTask(const std::string& taskName, int64_t totalWork,
                   const std::string& unit) override;
    void update(int64_t completed, const std::string& message) override;
    void advance(int64_t delta, const std::string& message) override;
    void endTask(const std::string& message) override;
    void cancel() override;
    bool isCancelled() const override;

    // ImGui 渲染时调用，显示当前进度
    void drawProgressBar();

private:
    struct TaskState {
        std::string name;
        std::string unit;
        int64_t total = 0;
        int64_t completed = 0;
        std::string currentMessage;
        double startTime = 0.0;
    };
    
    std::atomic<bool> m_cancelled{false};
    TaskState m_currentTask;
    bool m_hasActiveTask = false;
};

} // namespace midgard
```

### 8.5 与 Simulation Control 窗口的联动

```
┌─ Simulation Control ───────────────────────────────────────┐
│ ▶ ⏸ ⏹ ⏮ ⏭  [████████░░░░] 60%  744/1240  [x]Step  Speed[===]1x │
│                                                            │
│ Phase 0: MacroCut  [████████████░░░░] 80%  2329/2911       │
│ > classifying voxels...                                    │
└────────────────────────────────────────────────────────────┘
```

**显示规则**：
- 有活跃任务时，在控制按钮下方显示子进度条
- 子进度条显示：任务名 + 进度条 + 百分比 + 已完成/总数 + 当前消息
- 多阶段任务（Phase 0→1→2）依次显示，前一阶段完成后显示下一阶段
- `endTask()` 后子进度条淡出或显示完成信息 2 秒

### 8.6 初始化与注入（`app/main.cpp`）

```cpp
#include "core/ProgressReporter.h"
#include "windows/SimControlWindow.h"

int main() {
    // ... GLFW/ImGui 初始化 ...
    
    SimControlWindow simControl;
    
    // 注入进度报告接口
    midgard::g_progressReporter = std::shared_ptr<IProgressReporter>(
        &simControl, [](IProgressReporter*){}  // 不拥有所有权，避免 double-free
    );
    
    // ... 主循环 ...
    while (!glfwWindowShouldClose(win)) {
        // ... 仿真计算（内部使用 g_progressReporter）...
        
        // ImGui 渲染
        simControl.draw();  // 内部调用 drawProgressBar()
    }
    
    midgard::g_progressReporter = nullptr;
    return 0;
}
```

### 8.7 零开销保证

- `g_progressReporter` 为 `nullptr` 时，所有进度代码不执行
- 批量推进（`advance(64)`）减少调用频率，避免每 voxel/每点都调用
- `isCancelled()` 检查与进度更新合并，不增加额外原子操作
- 无 UI 环境下不创建 `SimControlWindow`，不占用内存

---

## 9. 3D 视口渲染层

| 层 | 条件 | 方式 |
|----|------|------|
| SDF Mesh | always | 三角面片 |
| Billet (orig) | `showBillet` | 透明 Box |
| Tool Path | `showToolPath` | GL_LINES，分色 |
| Tool Axis | `showToolAxis` | GL_LINES 箭头 |
| Current Tool | `showTool` | 球/圆柱 mesh |
| Voxel Wire | `inspectorActive` | 黄色线框 |
| **Debug Display** | `g_debugDisplay != nullptr` | 按 tag 渲染 BBox/点/三角片 |

**刀路轨迹分色**：已执行 #888888α0.4 / 当前 #FFAA00α1.0 / 未执行 #444444α0.2

---

## 10. 实现文件结构

```
app/
├── main.cpp
├── AppState.h/.cpp
├── windows/
│   ├── SettingsWindow.h/.cpp
│   ├── SimControlWindow.h/.cpp    # 新增：实现 IProgressReporter
│   ├── OutputWindow.h/.cpp
│   └── DebugWindow.h/.cpp
├── renderers/
│   ├── SceneRenderer.h/.cpp
│   ├── PathRenderer.h/.cpp
│   ├── ToolRenderer.h/.cpp
│   └── DebugDisplayImpl.h/.cpp    # 新增：调试显示实现
└── utils/
    └── LogBuffer.h/.cpp
interface/                          # 新增：抽象接口层
├── DebugDisplay.h                  # IDebugDisplay 接口
├── ProgressReporter.h              # IProgressReporter 接口
└── Globals.cpp                     # 全局指针定义
core/
├── CMakeLists.txt                  # 链接 interface
├── Types.h
├── ToolSweptSDF.h/.cpp
└── ...
```

---

## 11. 待审查事项

本次审查意见：

| # | 审查意见 | 处理方案 |
|---|----------|----------|
| 14 | Voxel Inspector 简化程度足够 | 无需修改 |
| 15 | IDebugDisplay 增加 `drawLine()` / `drawArrow()` | 已更新接口 |
| 16 | Demo 场景 IT-1 足够 | 无需修改 |
| 17 | DebugDisplay.h / ProgressReporter.h 作为独立项目 | 已更新为 `interface/` 方案，见 11.1 节 |
| 18 | GPU 扩展兼容性 | ✅ 已分析 | 11.2 节 |

### 11.1 抽象接口的部署方式讨论

**当前方案**：接口定义在 `core/` 目录，与算法核心同仓库。

**用户建议**：作为独立 DLL 项目，core 引用抽象接口，宿主程序实现并通过指针传递。

**分析对比**：

| 方案 | 优点 | 缺点 |
|------|------|------|
| **A. 当前（同仓库）** | 简单，无额外依赖，头文件直接包含 | core 与 UI 耦合在代码层面（虽然运行时不耦合） |
| **B. 独立 DLL** | 物理隔离，core 完全不感知 UI；多宿主共享 | 增加构建复杂度；C++ ABI 兼容性问题；需要显式加载/链接 |
| **C. 独立头文件库（推荐）** | 仅头文件，无 ABI 问题；物理隔离但构建简单；多宿主共享 | 仍需管理头文件路径 |

**推荐方案 D（同仓库 interface/ 目录）**：

```
yggdrasil-midgard/
├── interface/              # 抽象接口，不依赖任何实现
│   ├── DebugDisplay.h
│   ├── ProgressReporter.h
│   └── Globals.cpp         # g_debugDisplay, g_progressReporter 定义
├── core/                   # 算法引擎
│   ├── CMakeLists.txt      # 链接 interface
│   └── ...
├── app/                    # UI 集成环境
│   ├── CMakeLists.txt      # 链接 interface
│   └── ...
└── CMakeLists.txt          # 总控，add_subdirectory(interface)
```

**关键**：`interface/` 是独立目录，但同仓库管理。`core` 和 `app` 都链接 `interface` 静态库，共享同一个全局指针定义。

### 11.2 GPU 扩展兼容性说明

当前 midgard 采用 **TBB 并行（CPU）**，未来规划 **GPU 加速四叉树细分**。

`interface/` 方案对 GPU 扩展的影响分析：

| 场景 | 影响 | 处理方式 |
|------|------|----------|
| **GPU 计算 + CPU 调试显示（过渡期）** | 无影响 | GPU kernel 完成后，CPU 侧读取结果再调用 `IDebugDisplay` |
| **纯 GPU 管线（未来）** | 需 GPU-aware 实现 | 实现类使用 CUDA-OpenGL interop 直接渲染，或 device→host 回传后调用 |

**设计保证**：
- `IDebugDisplay` 是纯虚接口，不绑定 OpenGL
- 未来可提供 `CudaDebugDisplay` 或 `MetalDebugDisplay` 实现
- 高频调用已优化：批量推进（`advance(64)`），避免每 voxel 调用

**注意事项**：
- GPU 异步执行时，CPU 侧 `g_debugDisplay` 可能销毁，需 `std::atomic` 或显式同步
- GPU 产生的 device vector 需拷贝到 host 再传给 `IDebugDisplay`（除非实现类直接处理 GPU 内存）

---

*设计文档 v1.0（Final）已审查通过，可进入实现阶段。*
