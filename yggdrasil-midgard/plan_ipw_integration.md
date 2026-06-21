# IPWBuilder 集成计划 v2

> 核心洞察：`IPWState` 是工件状态的全生命周期容器。毛坯→切削→结果，全部通过它流动。
> **Core = 计算，App = 显示。** 状态在 Core，UI 只读不造。

---

## 架构原则

```
┌─ 用户定义毛坯 ──────────────────────────────────────────┐
│  GeometryDef billetDef {type, origin, dims, ...}         │
└────────────────────┬────────────────────────────────────┘
                     ▼
         ┌───────────────────────┐
         │  IPWBuilder::build()  │  ← Core 计算
         │  → macroGrid (SDF 体素)│
         │  → microGrid (表面点云)│
         └───────────┬───────────┘
                     ▼
         ┌───────────────────────────────────────┐
         │           IPWState                     │  ← 状态容器
         │  macroGrid ──┐                        │
         │  microGrid ──┤  随切削仿真持续变化       │
         │  billetDef ──┘                        │
         └───────────────┬───────────────────────┘
                         │
         ┌───────────────┼───────────────┐
         ▼               ▼               ▼
   SceneRenderer    MacroCut        MicroCut
   (可视化当前状态)   (宏观体素修改)   (微观surfel更新)
```

**AppState 直接持有 IPWState**，不再有单独的 `billetOrigin`/`billetDims` 等散字段。

---

## 阶段一：修复 Core 内部冲突

**文件**: `core/IDebugDisplay.h`

当前问题：重新定义了 `Vec3d = std::array<double,3>`，与 `core/Types.h` 的 `Vec3d = openvdb::Vec3d` 冲突。

```cpp
// Before:
using Vec3f = std::array<float, 3>;
virtual void drawLines(const Vec3f* data, size_t count, uint32_t color) = 0;

// After:
virtual void drawLines(const float* data, size_t count, uint32_t color) = 0;
// count 单位：顶点数（每顶点 3 个 float）
```

用裸指针消除类型依赖。这是唯一的兼容性问题。

---

## 阶段二：集成测试

**目标**: 验证 `GeometryDef → IPWBuilder → IPWState` 通路。

**新文件**: `test/test_ipw_builder.cpp`
**新 CMake 目标**: `test-ipw` (独立编译，链接 OpenVDB)

| 测试 | 输入 | 验证点 |
|------|------|--------|
| TC1 | Box 50×50×20, tol=0.5 | macroGrid 非空，bbox 匹配 |
| TC2 | Cylinder R=25 H=20 | 柱面区域 voxel 值正确 |
| TC3 | 重建：改 GeometryDef→rebuild | 新 IPWState 覆盖旧数据 |
| TC4 | 公差对比 tol=0.1 vs 1.0 | voxel 尺寸 = 1mm vs 10mm |

---

## 阶段三：AppState 迁移

**目标**: 消除类型重复，AppState 直接持有 IPWState。

### 3a — AppState.h 瘦身

```cpp
// Before (68行，重复定义):
using Vec3d = std::array<double,3>;
using Vec3f = std::array<float,3>;
enum class ToolType : uint8_t { ... };
struct ToolDef { int id; ... };
struct ToolPathSegment { ... };
// 散字段: billetOrigin, billetDims, billetType

// After (30行，全部引用 core):
#include "core/Types.h"
#include "core/IPWBuilder.h"

struct AppState {
    // ── 核心状态（唯一真源）──
    IPWState ipw{ToleranceConfig(0.5)};
    bool ipwDirty = true;

    // ── 刀具库（UI 概念，id 是索引）──
    struct ToolEntry { int id; ToolDef def; };
    std::vector<ToolEntry> toolLibrary;
    int currentToolId = 0;

    // ── 刀路 ──
    std::vector<MoveSegment> pathSegments;
    int currentSegment = 0;
    float segmentProgress = 0.0f;
    int totalSegments = 0;

    // ── UI 状态 ──
    enum SimState { IDLE, RUNNING, PAUSED };
    SimState simState = IDLE;
    float speedMultiplier = 1.0f;
    bool stepMode = false;
    bool showBillet = true;
    bool showToolPath = true;
    // ...
};
```

**删除的类型**:
- ~~`using Vec3d = std::array<double,3>`~~ → `core/Types.h` 的 `openvdb::Vec3d`
- ~~`enum class ToolType`~~ → `core/Types.h`
- ~~`struct ToolDef`~~ → `core/Types.h` (+ `ToolEntry` 包一层 id)
- ~~`struct ToolPathSegment`~~ → `core/Types.h` 的 `MoveSegment`
- ~~`billetOrigin, billetDims, billetType`~~ → `ipw.billetDef`

### 3b — 适配渲染层

`BilletMeshGenerator` 当前用 `std::array<double,3>`，需改为 `openvdb::Vec3d`。

```cpp
// Before:
static void buildBox(const std::array<double,3>& origin, 
                     const std::array<double,3>& dims, ...);

// After: 直接用 openvdb::Vec3d (API 相同: origin[0], dims[1], ...)
static void buildBox(const openvdb::Vec3d& origin, 
                     const openvdb::Vec3d& dims, ...);
```

### 3c — 切换到 IPWState

SettingsWindow 的毛坯修改不再直接改散字段，而是：
```cpp
// 用户改 Type/Size/Origin → 修改 ipw.billetDef
state.ipw.billetDef.type = GeometryDef::BOX;
state.ipw.billetDef.dims = Vec3d(w, h, d);
state.ipwDirty = true;   // 触发 IPWBuilder::build() 重建
```

---

## 阶段四：IPW 可视化（可选后续）

从 `IPWState.macroGrid` 提取表面 Mesh：
```
macroGrid → Marching Cubes(threshold=0) → triangle mesh
→ SceneRenderer 渲染为半透明叠加层
```

---

## 文件变更总览

| 文件 | 动作 | 阶段 |
|------|------|------|
| `core/IDebugDisplay.h` | 修：裸指针替代 Vec3f | 1 |
| `test/test_ipw_builder.cpp` | 新：集成测试 | 2 |
| `CMakeLists.txt` | 新增 `test-ipw` 目标 | 2 |
| `app/AppState.h` | 改：include core, 删除重复类型, 持有 IPWState | 3 |
| `app/AppState.cpp` | 改：使用 ipw.billetDef | 3 |
| `app/renderers/BilletMeshGenerator.h` | 改：openvdb::Vec3d 替代 std::array | 3 |
| `app/renderers/SceneRenderer.cpp` | 改：读 ipw.billetDef | 3 |
| `app/windows/SettingsWindow.cpp` | 改：通过 ipw.billetDef 修改毛坯 | 3 |

---

## 不变更

| 保持 | 原因 |
|------|------|
| `ToolEntry { int id; ToolDef def }` | id 是 UI 索引概念，core 不需要 |
| `AppState.logBuffer, simTime, speedMultiplier` 等 | 纯 UI 状态 |
| BilletMeshGenerator 作为独立工具 | 从 GeometryDef 生成 GL mesh，逻辑简单独立 |
