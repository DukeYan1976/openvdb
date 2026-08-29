# 任务 1.1 设计计划：多段刀路仿真 — AppState::pathSegments 与 MicroGridLabState::cutHistory 打通

**计划日期**: 2026-06-26  
**目标**: 设计一套方案，让 `AppState` 中的多段刀路 (`pathSegments`) 能够驱动 `MicroGridLabState` 的增量切削管线 (`cutHistory` + `executeCutIncremental`)，实现球头刀的真实多段连续切削仿真。  
**约束**: 先设计，不编码。设计产物为本文档 + 接口变更清单。

---

## 1. 现状分析

### 1.1 当前两个独立系统

```
┌─────────────────────────────────────────────────────────────────┐
│  AppState (主应用状态)                                           │
│  ├── pathSegments: vector<MoveSegment>  ← 刀路数据               │
│  ├── toolLibrary: vector<ToolEntry>     ← 刀具库                │
│  ├── currentSegment: int                ← 当前执行段索引         │
│  ├── simState: IDLE/RUNNING/PAUSED      ← 仿真状态机             │
│  └── tick(dt) → 更新 segmentProgress   ← 时间推进                │
└─────────────────────────────────────────────────────────────────┘
                              ↓ 渲染：SceneRenderer 显示刀具位置
┌─────────────────────────────────────────────────────────────────┐
│  MicroGridLabState (实验面板状态)                                │
│  ├── cutHistory: vector<CutRecord>      ← 切削记录               │
│  ├── voxels: vector<MicroGridCell>      ← 体素网格              │
│  ├── executeCut(idx)                    ← 全量切削（单段）       │
│  └── executeCutIncremental(idx)         ← 增量切削（单段）       │
└─────────────────────────────────────────────────────────────────┘
                              ↓ 渲染：MicroGridLabWindow 显示表面点
```

**问题**: `AppState` 的刀路推进（`tick()`）与 `MicroGridLabState` 的切削执行（`executeCut`）是**完全独立的**。用户在 SimControlWindow 点击 Play 时，刀具在视口中移动，但**没有任何材料被切除**。

### 1.2 MicroGridLabWindow 的当前工作流

1. 用户在 MicroGridLab 面板手动输入刀具参数和单段刀路
2. 点击 "Add to CutHistory" 添加记录
3. 点击 "Execute Next" 或 "Execute All" 执行切削
4. 点击 "Load to Viewport" 将结果推送到主视口

这个流程是**手动、单段、实验性**的，不适合真实仿真。

### 1.3 需要打通的核心问题

| 问题 | 现状 | 目标 |
|------|------|------|
| **刀路来源** | MicroGridLab 手动输入 | 从 `AppState::pathSegments` 自动导入 |
| **执行时机** | 用户点击 Execute | 仿真运行时自动触发（按段） |
| **状态同步** | MicroGridLab 独立状态 | 与 AppState 仿真状态联动 |
| **可视化** | 需手动 Load to Viewport | 切削结果实时显示在主视口 |
| **增量更新** | 支持（executeCutIncremental） | 仿真推进时自动增量更新 |

---

## 2. 设计方案

### 2.1 核心原则

1. **单点 truth**: `AppState::pathSegments` 是刀路的唯一来源
2. **懒转换**: `pathSegments` → `cutHistory` 的转换在需要时进行（首次执行或刀路变更时）
3. **增量执行**: 利用已有的 `executeCutIncremental`，每段只处理新刀的影响
4. **状态机驱动**: 由 `AppState::simState` 控制何时执行下一段切削
5. **可观测性**: 每段切削的统计信息（时间、表面点数、材料去除量）反馈到 UI

### 2.2 架构变更

```
┌─────────────────────────────────────────────────────────────────┐
│  AppState (增强)                                                 │
│  ├── pathSegments: vector<MoveSegment>      [不变]               │
│  ├── toolLibrary: vector<ToolEntry>         [不变]               │
│  ├── currentSegment: int                    [不变]               │
│  ├── simState: enum                         [不变]               │
│  ├── ipw: IPWState                          [不变]               │
│  ├── billetDef: GeometryDef                 [不变]               │
│  ├── ▶ microGridLab: MicroGridLabState      [新增：持有实例]     │
│  │   └── 之前是 MicroGridLabWindow 的私有成员                   │
│  ├── ▶ cutHistorySynced: bool               [新增：转换状态标志]  │
│  └── ▶ lastExecutedSegment: int             [新增：最后执行段]   │
│                                                                  │
│  tick(dt) →                                                     │
│    1. 推进 segmentProgress                                      │
│    2. 若换段 → ipwDirty = true                                  │
│    3. ▶ 若换段且 microGridLab 模式 → 触发增量切削               │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│  SimulationDriver (新增协调器)                                   │
│  ├── syncPathToCutHistory(AppState&)                            │
│  │   └── pathSegments + toolLibrary → cutHistory                │
│  ├── executeNextCut(AppState&)                                  │
│  │   └── 调用 microGridLab.executeCutIncremental(last+1)        │
│  └── onSegmentComplete(AppState&)                               │
│      └── 更新统计、推送可视化                                   │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│  SceneRenderer (增强)                                            │
│  ├── render()                                                   │
│  │   └── ▶ 新增：渲染 microGridLab.surfacePoints 作为切削表面   │
│  └── ▶ rebuildCutSurface()                                      │
│      └── surfacePoints → GPU point cloud / mesh                 │
└─────────────────────────────────────────────────────────────────┘
```

### 2.3 数据流设计

#### 阶段 A：刀路导入（一次性）

```cpp
// 当用户点击 "Load Demo" 或导入 G-code 时
void AppState::loadDemoIT1() {
    // ... 现有代码：设置 pathSegments, toolLibrary, billetDef ...
    
    // 新增：同步到 MicroGridLab
    syncPathToMicroGridLab();
}

void AppState::syncPathToMicroGridLab() {
    microGridLab.cutHistory.clear();
    microGridLab.voxels.clear();  // 强制重建
    microGridLab.init();          // 重建为全 SOLID
    
    for (size_t i = 0; i < pathSegments.size(); ++i) {
        const auto& seg = pathSegments[i];
        // 从 toolLibrary 查找对应刀具
        const ToolDef* tool = findToolById(seg.toolId);
        if (!tool) continue;  // 或报错
        
        CutRecord rec;
        rec.tool = *tool;
        rec.segment = seg;
        rec.seqIndex = static_cast<uint32_t>(i);
        microGridLab.addCutRecord(rec);
    }
    
    cutHistorySynced = true;
    lastExecutedSegment = -1;  // 未执行任何段
}
```

#### 阶段 B：仿真推进（每帧）

```cpp
void AppState::tick(float dt) {
    if (simState != RUNNING) return;
    // ... 现有代码：推进 segmentProgress ...
    
    // 新增：检测换段
    int prevSegment = currentSegment;
    // ... 现有代码：更新 currentSegment ...
    
    // 换段时触发增量切削
    if (currentSegment > prevSegment && cutHistorySynced) {
        // 执行新段的增量切削
        microGridLab.executeCutIncremental(currentSegment);
        lastExecutedSegment = currentSegment;
        
        // 更新统计
        activeVoxels = /* 从 microGridLab 计算 */;
        activeSurfels = microGridLab.totalSurfacePoints;
        lastCutTimeMs = microGridLab.lastCutMs;
        
        // 通知渲染器更新
        macroMeshRequested = true;  // 或新增 cutSurfaceDirty = true
    }
}
```

#### 阶段 C：可视化（渲染时）

```cpp
// SceneRenderer::render()
void SceneRenderer::render(const Camera& camera, int w, int h) {
    // ... 现有代码：渲染毛坯、刀具、刀路 ...
    
    // 新增：渲染切削表面点
    if (cutSurfaceDirty_) {
        rebuildCutSurface();
        cutSurfaceDirty_ = false;
    }
    renderCutSurface(mvp, nm);
}

void SceneRenderer::rebuildCutSurface() {
    auto& state = getAppState();
    auto& points = state.microGridLab.surfacePoints;
    
    // 方案 1：点云（最简单，先实现这个）
    std::vector<float> verts;
    for (auto& sp : points) {
        verts.push_back(sp.position.x());
        verts.push_back(sp.position.y());
        verts.push_back(sp.position.z());
    }
    cutSurfacePoints_.upload(verts.data(), verts.size() * sizeof(float), verts.size() / 3);
    
    // 方案 2：三角网格（后续）
    // ... Delaunay / Marching Cubes ...
}
```

### 2.4 状态机设计

```
[IDLE] ──Play──► [RUNNING] ──Pause──► [PAUSED]
   ▲                │                    │
   │                │ Stop               │ Play
   │                ▼                    ▼
   └──────────── [IDLE] ◄──────────── [RUNNING]

// 新增：MicroGridLab 执行状态
enum class CutExecutionState {
    NOT_READY,      // cutHistory 未同步
    READY,          // 已同步，等待执行
    EXECUTING,      // 正在执行某段
    COMPLETE        // 全部执行完毕
};
```

### 2.5 时序图

```
User          AppState        SimControlWindow    MicroGridLabState    SceneRenderer
 │              │                    │                    │                  │
 │──Load Demo──►│                    │                    │                  │
 │              │──syncPathToMicroGridLab()──────────────►│                  │
 │              │                    │                    │──init()          │
 │              │                    │                    │──addCutRecord()xN│
 │              │◄─cutHistorySynced=true──────────────────│                  │
 │              │                    │                    │                  │
 │──Play───────►│──startSimulation() │                    │                  │
 │              │──simState=RUNNING──►│                    │                  │
 │              │                    │                    │                  │
 │              │◄────tick(dt)───────│                    │                  │
 │              │──segmentProgress++ │                    │                  │
 │              │──换段检测          │                    │                  │
 │              │──executeCutIncremental(i)──────────────►│                  │
 │              │                    │                    │──粗筛/精修/增量  │
 │              │                    │                    │──surfacePoints   │
 │              │◄─统计更新──────────│                    │                  │
 │              │──cutSurfaceDirty=true─────────────────────────────────────►│
 │              │                    │                    │                  │
 │              │◄────render()────────────────────────────────────────────────│
 │              │                    │                    │                  │──rebuildCutSurface()
 │              │                    │                    │                  │──draw points
 │              │                    │                    │                  │
 │◄──视口更新───│                    │                    │                  │
```

---

## 3. 接口变更清单

### 3.1 AppState.h 变更

```cpp
// 新增前向声明
struct MicroGridLabState;

struct AppState {
    // ... 现有成员不变 ...
    
    // ── 新增：MicroGridLab 集成 ──
    MicroGridLabState microGridLab;      // 嵌入实例（非指针）
    bool cutHistorySynced = false;       // pathSegments 是否已同步到 cutHistory
    int lastExecutedSegment = -1;        // 最后执行的段索引（-1 = 未执行）
    bool cutSurfaceDirty = true;         // 切削表面需要重建
    
    // ── 新增：方法 ──
    void syncPathToMicroGridLab();       // pathSegments → cutHistory 转换
    void resetMicroGridLab();            // 重置并重新同步
    
    // ... 现有方法不变 ...
};
```

### 3.2 MicroGridLabWindow.h 变更

```cpp
class MicroGridLabWindow {
public:
    void draw();
    
    // 新增：允许外部访问 state（用于 SimulationDriver）
    MicroGridLabState& state() { return state_; }
    const MicroGridLabState& state() const { return state_; }
    
private:
    // 不变：state_ 仍是私有，但提供引用访问
    MicroGridLabState state_;
    int precisionIndex_ = 1;
};
```

**替代方案**（推荐）：将 `MicroGridLabState` 从 `MicroGridLabWindow` 中移出，改为由 `AppState` 持有，`MicroGridLabWindow` 通过引用操作。这样避免两个状态源。

### 3.3 SceneRenderer.h 变更

```cpp
class SceneRenderer {
public:
    // ... 现有接口不变 ...
    
    // 新增：切削表面渲染
    void rebuildCutSurface();
    void clearCutSurface();
    bool hasCutSurface() const;
    
private:
    // ... 现有成员不变 ...
    
    // 新增
    GPUPoints cutSurfacePoints_;     // 点云渲染
    bool cutSurfaceDirty_ = true;
};
```

### 3.4 新增 SimulationDriver（可选）

如果逻辑复杂，可以新增一个协调器类：

```cpp
// core/SimulationDriver.h
#pragma once
#include "core/Types.h"

namespace midgard {

struct AppState;  // 前向声明

class SimulationDriver {
public:
    /// 将 AppState 的 pathSegments 同步到 MicroGridLabState
    static void syncPathToCutHistory(AppState& state);
    
    /// 执行下一段切削（如果 ready）
    static bool executeNextSegment(AppState& state);
    
    /// 检查是否需要执行（当前段 > 最后执行段）
    static bool hasPendingCut(const AppState& state);
    
    /// 获取执行统计信息
    static std::string getCutStats(const AppState& state);
};

} // namespace midgard
```

**简化方案**：不新增类，直接在 `AppState` 中增加方法。先采用简化方案，后续如果需要更复杂的调度（如多线程、预览模式）再提取为独立类。

---

## 4. 关键决策点

### 决策 1：MicroGridLabState 的归属

| 方案 | 描述 | 优点 | 缺点 |
|------|------|------|------|
| **A** | `AppState` 嵌入 `MicroGridLabState` | 单点 truth，仿真直接驱动 | MicroGridLabWindow 需要改引用 |
| **B** | `MicroGridLabWindow` 保持私有，`AppState` 通过指针访问 | 改动最小 | 两个状态源，容易不一致 |
| **C** | 新增 `SimulationEngine` 持有两者 | 架构清晰 | 过度设计，当前不需要 |

**推荐：方案 A**。将 `MicroGridLabState` 从 `MicroGridLabWindow` 移出，由 `AppState` 持有。`MicroGridLabWindow` 改为操作 `AppState::microGridLab` 的引用。

### 决策 2：执行时机

| 方案 | 描述 | 优点 | 缺点 |
|------|------|------|------|
| **A** | 换段时立即执行（tick 中同步调用） | 简单，实时 | 大段切削可能卡顿 |
| **B** | 换段时异步执行（后台线程） | 不卡顿 | 需要线程同步，复杂 |
| **C** | 用户点击 Step 时才执行 | 完全可控 | 非自动仿真 |

**推荐：方案 A（先实现）**，后续如果性能不够再考虑 B。当前 MicroGridLab 的增量切削在测试中已经很快（<10ms），应该可以实时。

### 决策 3：可视化方案

| 阶段 | 方案 | 实现复杂度 |
|------|------|-----------|
| **Phase 1** | 点云渲染（`GL_POINTS`） | 低（1天） |
| **Phase 2** | 三角网格（Delaunay / Marching Cubes） | 中（2-3天） |
| **Phase 3** | 平滑着色 + 法线插值 | 中（2天） |

**推荐：先 Phase 1 点云**，快速验证链路。Phase 2/3 在任务 1.3 中处理。

---

## 5. 任务拆分与依赖

```
1.1.1 数据层：MicroGridLabState 迁移到 AppState
    ├─ 修改 AppState.h：添加 microGridLab 成员
    ├─ 修改 MicroGridLabWindow：改为引用访问
    └─ 验证：编译通过，MicroGridLab 面板功能正常
    
1.1.2 转换层：pathSegments → cutHistory
    ├─ 实现 AppState::syncPathToMicroGridLab()
    ├─ 处理刀具查找（toolId → ToolDef）
    └─ 验证：Load Demo 后 cutHistory 长度 = pathSegments 长度
    
1.1.3 执行层：tick() 中触发增量切削
    ├─ 修改 AppState::tick()：换段时调用 executeCutIncremental
    ├─ 添加执行状态跟踪（lastExecutedSegment）
    └─ 验证：Play 后每换一段，microGridLab 状态更新
    
1.1.4 渲染层：切削表面可视化
    ├─ SceneRenderer 添加点云渲染
    ├─ 从 microGridLab.surfacePoints 构建 GPU buffer
    └─ 验证：视口中能看到切削产生的表面点
    
1.1.5 集成测试：完整链路验证
    ├─ Load Demo → Play → 观察多段切削
    ├─ 验证材料去除（air voxel 增加）
    └─ 验证统计信息（每段时间、表面点数）
```

---

## 6. 风险与缓解

| 风险 | 影响 | 缓解措施 |
|------|------|----------|
| 增量切削耗时 > 16ms | 帧率下降，卡顿 | 先测试性能，若超标则改为异步或降低精度 |
| surfacePoints 数量爆炸 | GPU 内存不足 | 添加上限，或八叉树降级采样 |
| 刀具查找失败（toolId 不存在） | 崩溃 | 添加 fallback（使用默认刀具）+ 日志警告 |
| 多段累积误差 | 最终形状偏差 | 定期（如每 10 段）执行一次全量重建 |
| MicroGridLab 面板与仿真冲突 | 用户手动操作干扰自动仿真 | 仿真运行时禁用面板编辑，或分离模式 |

---

## 7. 下一步

等待确认本设计计划。确认后，按 1.1.1 → 1.1.2 → 1.1.3 → 1.1.4 → 1.1.5 的顺序逐步实施。

---

*设计人: OpenClaw CAM-Architect*  
*日期: 2026-06-26*
