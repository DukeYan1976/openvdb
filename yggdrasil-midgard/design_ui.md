# Yggdrasil-Midgard 仿真界面设计

> 版本：v2.0 | 日期：2026-06-19
> 原则：**Simplicity is beautiful.**

---

## 修订记录

| 版本 | 日期 | 修改内容 |
|------|------|----------|
| v1.0 | 06-19 | 初始设计（过度工程版） |
| v2.0 | 06-19 | 精简重构：删除 IProgressReporter、精简 IDebugDisplay 至 3 方法、合并 interface 到 core、修复交互冲突 |
| v2.1 | 06-19 | 增加 §10 异步计算-渲染架构（可选，后期实现） |
| v2.2 | 06-19 | 修正刀具设计：分离刀具库与路径段，避免数据冗余 |

---

## 1. 技术栈

OpenGL 3.3 + GLFW + GLAD + ImGui。借鉴 `yggdrasil/app/main.cpp` 渲染栈。

---

## 2. 布局

```
┌──────────────────────────────────────────────────────────────┐
│                      3D Viewport                             │
│                                                              │
│  ┌─ Settings ─────┐     ┌─ Simulation Control ─────────────┐│
│  │ ▼ Path         │     │ ▶ ⏸ ⏹ ⏮ ⏭ [████░░] 60% 744/1240││
│  │ ▶ Billet       │     │ [x]Step   Speed[===]1x           ││
│  │ ▶ Tool         │     └──────────────────────────────────┘│
│  └─────────────────┘     ┌─ Output ────────────────────────┐│
│  ┌─ Debug ─────────┐     │ [11:00:01] Cut 744/1240  28ms   ││
│  │ [x] Inspector   │     │ Voxels:47726 Surfels:17688      ││
│  │ [x] RtDebug On  │     └─────────────────────────────────┘│
│  │ SDF: -0.023     │                                        │
│  └─────────────────┘                                        │
└──────────────────────────────────────────────────────────────┘
```

- 所有窗口可拖拽停靠，布局通过 `imgui.ini` 持久化
- 字体：`io.FontGlobalScale` 按屏幕 DPI 自适应

---

## 3. Settings

Path 默认展开，其余折叠。

### 3.1 Path

| 控件 | 说明 |
|------|------|
| Load .cls | 加载刀路文件（含刀具表+路径段） |
| Load Demo | IT-1 预设（50×50×20 Box, BallEnd R=5, 单段X向, t=0.01mm） |
| Tolerance t | 用户容差，可修改后 Recalculate |
| Show Path | 显示开关 |
| Segments | 只读，总段数 |
| Current Tool | 只读，当前段使用的刀具（从刀具库索引） |

### 3.2 Tool Library（折叠）

| 控件 | 说明 |
|------|------|
| Tool List | 刀具库列表，点击选中 |
| Type[Ball/Flat/Bull] | 选中刀具的类型 |
| R, r, H | 选中刀具的几何参数 |
| Add / Remove | 增删刀具 |

### 3.3 Billet（折叠）

Type[Box/Cyl/Mesh], Size, Origin, Alpha, Show

---

## 4. Simulation Control

```
▶ ⏸ ⏹ ⏮ ⏭   [████████░░░░] 60%   744/1240   [x]Step   Speed[===]1x
```

| 控件 | 说明 |
|------|------|
| ▶ ⏸ ⏹ | Play / Pause / Stop |
| ⏮ ⏭ | 回退/前进一段 |
| Step | ON时按▶执行1段后暂停 |
| Speed | 段间延迟缩放：1x=按feedrate实时，10x=快进10倍 |

**状态机**: IDLE → RUNNING → PAUSED → IDLE。RUNNING时Settings灰显。

**取消**: Stop按钮设置 `cancelled=true`，主循环段间检查：
```cpp
for (auto& seg : segments) {
    if (cancelled) break;
    simulate(seg);
    updateProgress(++current, total);
}
```

---

## 5. Output

- 日志：自动滚动，500行上限，颜色分级（Build蓝 / Cut绿 / Error红）
- 统计：Voxels / Surfels / Mem / Volume / Last(ms) / Avg(ms)
- 每段切削后更新

---

## 6. Debug

### 6.1 Voxel Inspector

- **交互**：Shift+左键点击拾取（避免与3D导航冲突）
- 显示：Coord / SDF / Surfels 数量
- 3D视口中高亮选中voxel黄色线框

### 6.2 RtDebug

| 控件 | 映射 |
|------|------|
| ON/OFF | `Debugger::Activate / Deactivate` |
| Level [On/Verbose] | `_debugLevel` |

---

## 7. 调试显示接口

### 7.1 设计

- 定义在 `core/IDebugDisplay.h`，算法代码可选调用
- `g_debugDisplay == nullptr` 时零开销
- UI 环境注入实现；无 UI 时不链接

### 7.2 接口（3个方法，够用）

```cpp
// core/IDebugDisplay.h
#pragma once
#include <cstddef>
#include <cstdint>

namespace midgard {

struct Vec3f { float x, y, z; };

class IDebugDisplay {
public:
    virtual ~IDebugDisplay() = default;

    virtual void drawLines(const Vec3f* data, size_t count, uint32_t color) = 0;
    virtual void drawPoints(const Vec3f* data, size_t count, uint32_t color) = 0;
    virtual void drawTriangles(const Vec3f* verts, const uint32_t* indices, size_t triCount, uint32_t color) = 0;
    virtual void clear() = 0;
};

extern IDebugDisplay* g_debugDisplay;

} // namespace midgard

// 编译隔离宏
#ifdef MIDGARD_DEV
  #define DEBUG_DRAW(code) do { if (midgard::g_debugDisplay) { code; } } while(0)
#else
  #define DEBUG_DRAW(code) ((void)0)
#endif
```

### 7.3 使用模式

```cpp
// core/MacroCut.cpp
DEBUG_SECTION(MACRO_CUT) {
    if (g_debugDisplay) {
        // 画所有task的voxel bbox (12条线段/box)
        std::vector<Vec3f> lines;
        for (auto& t : tasks) appendBBoxLines(lines, t.aabb);
        g_debugDisplay->drawLines(lines.data(), lines.size(), 0xFF00FFFF);
    }
}
```

### 7.4 约束

- **只在主线程调用**（TBB worker 不直接调用，计算完成后在主线程汇总绘制）
- 发布版 `#ifdef MIDGARD_DEV` 编译开关移除所有调试UI

---

## 8. 3D 渲染层

| 层 | 条件 |
|----|------|
| SDF Mesh | always |
| Billet (透明) | showBillet |
| Tool Path | showPath |
| Current Tool | showTool |
| Voxel Highlight | inspector active |
| Debug Lines/Points | g_debugDisplay |

刀路分色：已执行 灰α0.4 / 当前 橙α1.0 / 未执行 暗灰α0.2

---

## 9. 数据模型（修正）

### 9.1 刀具与路径分离

```cpp
// 刀具定义（独立库）
struct ToolDef {
    int id = 0;
    ToolType type = ToolType::BALL_END;
    double R = 5.0, r = 0.0, H = 20.0;
};

// 刀路段（引用刀具，不内嵌参数）
struct ToolPathSegment {
    Vec3d start = {0, 0, 0};
    Vec3d end = {0, 0, 0};
    Vec3d axis = {0, 0, 1};
    int toolId = 0;  // 引用 ToolDef
};
```

**优势**：
- 一把刀对应多段路径，符合 CAM 实际
- 换刀时只需改 `toolId`
- 刀具参数集中管理，避免冗余

### 9.2 文件结构

```
app/
├── main.cpp
├── AppState.h
├── windows/
│   ├── SettingsWindow.cpp    # Path / Tool Library / Billet
│   ├── SimControlWindow.cpp
│   ├── OutputWindow.cpp
│   └── DebugWindow.cpp
├── renderers/
│   ├── SceneRenderer.cpp
│   ├── DebugDisplayImpl.cpp
│   └── PathRenderer.cpp
└── utils/
    └── LogBuffer.h
core/
├── IDebugDisplay.h
├── Types.h
├── ToolDef.h              # 刀具定义（新增）
├── MicroCut.cpp
└── ...
```

---

## 10. 异步计算-渲染架构（可选，后期实现）

> 注：此架构用于解决计算与渲染争用，提升高倍速仿真时的流畅度。算法稳定后可实现。

### 10.1 核心设计

```
┌──────────────┐         ┌──────────────┐
│  Compute     │  epoch  │   Render     │
│  Thread      │ ──────→ │   Thread     │
│  (TBB+CPU)   │  atomic │   (OpenGL)   │
└──────────────┘         └──────────────┘
```

| 设计点 | 说明 |
|--------|------|
| **双线程不互等** | 计算全速跑，渲染按帧读取最新完成状态 |
| **epoch 原子计数** | 一段完整 Phase 0→4 后才 +1，渲染不读到半写状态 |
| **OpenVDB ConstAccessor** | 读写天然无竞争（tree 的 leaf 替换是原子的） |
| **视锥裁剪** | 只 mesh 化可见 leaf，epoch 不变时零开销复用上帧 |

### 10.2 数据流

```cpp
// Compute Thread
while (running) {
    // 1. 执行完整切削段（Phase 0→4）
    engine.cut(billet, tool);
    
    // 2. 原子递增 epoch，标记新数据可用
    ++epoch;
    
    // 3. 非阻塞，立即进入下一段
}

// Render Thread (每帧)
void renderFrame() {
    // 1. 读取当前 epoch（原子读）
    uint64_t currentEpoch = epoch.load(std::memory_order_acquire);
    
    // 2. epoch 未变？复用上帧 mesh
    if (currentEpoch == lastRenderedEpoch) {
        drawLastFrame();  // 零开销
        return;
    }
    
    // 3. epoch 变了？用 ConstAccessor 读取新 grid
    auto acc = grid.getConstAccessor();
    
    // 4. 视锥裁剪：只 mesh 化可见 leaf
    for (visible leaf in frustum) {
        meshify(leaf, acc);
    }
    
    // 5. 更新缓存
    lastRenderedEpoch = currentEpoch;
    draw();
}
```

### 10.3 关键保证

- **无锁读**：OpenVDB `ConstAccessor` 不阻塞 `stealNode` 写入
- **不读半写**：epoch 在 Phase 4 完成后才递增，渲染永远读到完整状态
- **零拷贝**：mesh 化时直接读取 voxel 数据，不复制 grid
- **帧率稳定**：计算再快，渲染按 60fps 固定节奏，不卡顿

### 10.4 实现时机

- **Phase 1（当前）**：单线程，计算与渲染串行，简单可靠
- **Phase 2（后期）**：引入双线程，当算法稳定且需要高倍速仿真时实现

---

## 11. 编译隔离

```cpp
// core/IDebugDisplay.h 末尾
#ifdef MIDGARD_DEV
  #define DEBUG_DRAW(code) do { if (g_debugDisplay) { code; } } while(0)
#else
  #define DEBUG_DRAW(code) ((void)0)
#endif
```

发布版本：调试UI代码、`DebugWindow`、`DebugDisplayImpl` 全部不编译。

---

---

## 12. 三态分类判据

> 原则：**可以漏判（false negative），不能错判（zero false positive）。**
>
> 两类位置需要分类：
> 1. **MacroCut::classifyVoxels** — 将 active voxel 分三态（deleted / boundary / keep）用于 Phase 0 加工模拟
> 2. **IPWBuilder::build() debug** — 仅显示 boundary voxel，用于可视化验证
>
> 两者共用同一判据，保证一致性。

### 12.1 外接球判据

体素中心到最远角点的距感为外接球半径 $R_{\text{circ}} = V \cdot \sqrt{3} / 2$。

利用 SDF 的 **1-Lipschitz 性质**（体素内任意两点的 SDF 差不超过它们的欧氏距感）：

- 若体素中心 SDF $D > R_{\text{circ}} + \varepsilon$：
  即使最靠近实体的角点（沿梯度反方向 $R_{\text{circ}}$），其 SDF $\geq D - R_{\text{circ}} > \varepsilon > 0$。
  → **整个体素在实体外** → 确定 **Air**。

- 若体素中心 SDF $D < -(R_{\text{circ}} + \varepsilon)$：
  即使最远离实体的角点，其 SDF $\leq D + R_{\text{circ}} < -\varepsilon < 0$。
  → **整个体素在实体内** → 确定 **Interior**。

- 否则 $|D| \leq R_{\text{circ}} + \varepsilon$：
  存在角点 SDF 符号相反的可能性 → 曲面**可能**穿过此体素 → **Boundary**。

**零误判保证**：任何被判定为 boundary 的体素，其外接球确实触碰到曲面。
**漏判允许**：体素某角落被曲面划过但中心 SDF 恰好 $> R_{\text{circ}}$（极其罕见），判为 Air/Interior 也不产生错误微表面（切削时由相邻 voxel 覆盖）。

### 12.2 公式

```cpp
static constexpr double kMechEpsilon = 1e-4;  // 0.1μm 固定机械分辨率，仅防浮点抖动
const double threshold = V * std::sqrt(3.0) / 2.0 + kMechEpsilon;

// ─── 三态分类 ───
if      (sdf < -threshold)  →  Interior / Deleted     (全在实体内)
else if (sdf >  threshold)  →  Air / Keep              (全在实体外)
else                         →  Boundary / Cut+Boundary (可能被穿过)
```

### 12.3 生效位置

| 文件 | 函数 | 用途 | 判据 |
|------|------|------|------|
| `core/MacroCut.cpp` | `classifyVoxels()` | Phase 0 三态分类 | `V·√3/2 + 1e-4` |
| `core/IPWBuilder.cpp` | `build()` debug | 可视化 boundary | `V·√3/2 + 1e-4` |

> 两者共用同一 `threshold`。MacroCut 的三态中 `deleted` 映射到 Interior、`keep` 映射到 Air、其余为 Boundary（再细分为 cut/newBoundary）。

### 12.4 端面双层 boundary 显示

圆柱端面 disc 内部 voxel 的 SDF 恒为 0（端面=边界面），通过 threshold 检查。
不做额外过滤，SDF 判据自行产生双层效果：

- **第 1 层**（iz=0/izTop）：完整端面 disc，SDF=0 → `|SDF| ≤ threshold` → 显示
- **第 2 层**（iz=1/izTop-1）：侧壁边界环。内部 voxel 因 `|SDF|=V > threshold` 自筛掉

---

## 修订记录

| 版本 | 日期 | 修改内容 |
|------|------|----------|
| v2.5 | 06-20 | SimEngine 单段切削管线封装 + GTest |
| v2.4 | 06-20 | §12 重写为三态分类判据，外接球保证零误判 |
| v2.3 | 06-20 | 增加 §12 边界 Voxel 宽容判据，`threshold` 叠加 `user_t` |

---

## 13. SimEngine 单段切削管线

将 MacroCut + MicroCut 五阶段管线封装为 `SimEngine::cutSegment()`。

### 13.1 接口

```cpp
struct CutResult {
    bool success = false;
    int  deletedVoxels = 0;
    int  newSurfacePoints = 0;
    double elapsedMs = 0.0;
};

class SimEngine {
public:
    CutResult cutSegment(IPWState& ipw, const MoveSegment& seg,
                         const ToolDef& tool, const GeometryDef& billet,
                         const ToleranceConfig& config);
};
```

### 13.2 管线流程

1. `ToolSweptSDF(tool, seg)` — 刀具扫掠 SDF
2. `ToolSweepSurface(tool, seg)` — 参数面
3. `MacroCut::classifyVoxels()` — 三态分类
4. `MacroCut::buildTaskList()` — 生成 VoxelTask
5. `MicroCut::primeBilletBoundaries()` — 冷启动毛坯边界
6. `MicroCut::sampleNewSurface()` — 刀具表面采样
7. 合并 billetBuf → newBuf（仅补充缺失 key）
8. `MicroCut::rebuildLeaves()` — 重建 leaf
9. 返回 `CutResult`

### 13.3 设计约束

- 无状态：不缓存任何 grid，可复用
- 不引入新依赖
- 不修改已稳定的 core 算法文件

*设计文档 v2.5 — 已更新，可进入实现阶段。*
