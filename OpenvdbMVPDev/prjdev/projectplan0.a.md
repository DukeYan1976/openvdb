# Yggdrasil 多轴机加工仿真引擎 — 项目总体规划

**文档编号**：`projectplan0.a.md`  
**状态**：初稿  
**作者**：Duke / Kiro  
**日期**：2026-06-10  
**定位**：原型研究结束后的工程化总体规划

---

## 1. 项目定位与目标

**产品定位**：跨平台工业级多轴机加工仿真引擎（核心计算库 + 可视化应用）

**核心能力**：
- 给定毛坯几何 + 刀具路径 → 实时计算切削后工件几何
- 精度可达 μm 级（d_v = 0.005mm），内存可控（双轨架构）
- 支持 3/4/5 轴切削仿真

**最终交付物**：
1. `libyggdrasil` — 独立跨平台 C++ 计算库（无 GUI 依赖）
2. `ygg_app` — 基于 Filament/ImGui 的可视化调试/演示应用
3. 文档 + 测试套件

---

## 2. 原型研究总结（已完成）

### 2.1 已验证的核心算法

| 模块 | 状态 | 关键结论 |
|------|------|----------|
| R1: ResolutionSolver | ✅ 完成 | 自动选择 SINGLE/DUAL/ATLAS 模式，内存预算约束有效 |
| R2: BilletBuilder | ✅ 完成 | FloatGrid Level Set 构建正确，BOX/CYLINDER 解析几何 |
| R3: ToolSweepSDF | ✅ 完成 | 球头/平底刀解析 SDF + 梯度，线段扫掠 |
| R4: CuttingEngine (单轨) | ✅ 完成 | 4种策略(A/B/C/D)对比验证，方案D(TBB并行)最优 |
| R4: CuttingEngine (双轨) | ✅ 原型完成 | 4-phase 架构验证通过，存在已知 bug（已修复） |
| SurfelGenerator | ✅ 原型完成 | 刀具零面投影采样，N×N 面元生成 |
| 可视化原型 | ✅ 完成 | OpenGL3.3 + ImGui，多层渲染，Voxel Inspector |

### 2.2 已发现的问题与教训

1. **dirty voxel 边界条件**：需包含外侧边界 voxel（toolDist < D_v 而非 ≤ 0）
2. **多次切削重注入**：不能跳过已有面元的 voxel，需允许重新采样
3. **幽灵面元**：刀具面延伸到毛坯外时需 billet 边界裁剪
4. **inactive tile 激活范围**：需对齐窄带宽度（±3 voxel）
5. **Phase 执行顺序**：Phase2 注入 → Phase3 裁剪的顺序是正确的（新面元在零面上不会被误杀）

### 2.3 当前代码结构

```
yggdrasil/
├── core/                 # 计算内核
│   ├── ResolutionSolver  # 分辨率/模式决策
│   ├── BilletBuilder     # 毛坯构建
│   ├── ToolSweepSDF      # 刀具扫掠体 SDF
│   ├── CuttingEngine     # 切削引擎（单轨+双轨）
│   └── SurfelGenerator   # 面元采样
├── types/                # 数据类型定义
│   ├── YggTypes.h        # BilletModel, GeometryDef, ResolutionConfig
│   └── MemoryStats.h     # 内存监控
├── app/                  # 可视化应用（OpenGL+ImGui 原型）
├── tests/                # GTest 测试套件
└── CMakeLists.txt
```

---

## 3. 工程化阶段划分

### Phase A: 计算内核工程化（核心库重构）

**目标**：将原型代码重构为生产级 C++ 库，接口稳定、线程安全、可独立编译

| 任务 | 内容 | 优先级 |
|------|------|--------|
| A1: 接口定义与封装 | 定义 public API（`ygg::Engine` 单一入口类），隐藏实现细节 | P0 |
| A2: 双轨切削算法修正 | 基于原型 bug 修复结果，重写 cutDualTrack 为生产版本 | P0 |
| A3: IPW₀ 延迟注入 | 实现设计文档中的延迟注入机制（首次触及 voxel 时按需生成面元） | P0 |
| A4: 多步切削路径 | 支持刀具路径序列（多段线），逐段调用 CuttingEngine | P1 |
| A5: 内存管理 | MemoryStats 实时监控 + 内存预算自动降级 | P1 |
| A6: 线程安全 | microGrid 读写隔离（rebuild thread vs rendering thread） | P1 |
| A7: 单元测试加固 | 补充边界条件测试、回归测试、精度验证测试 | P1 |

### Phase B: 几何输入扩展

**目标**：支持实际工业场景的毛坯和刀具输入

| 任务 | 内容 | 优先级 |
|------|------|--------|
| B1: STL/OBJ 导入 | 三角网格 → Level Set（OpenVDB meshToVolume） | P1 |
| B2: BVH 加速结构 | 为 Mesh 毛坯构建 BVH，支持 IPW₀ 射线精确定位 | P1 |
| B3: 圆柱/球体毛坯 | 解析几何毛坯扩展 | P2 |
| B4: 刀具类型扩展 | 牛鼻刀（Bull Nose）、锥形刀 | P2 |
| B5: 多轴扫掠体 | 5轴连续旋转扫掠体 SDF（离散化+插值） | P2 |

### Phase C: 渲染引擎升级

**目标**：从 OpenGL 原型迁移到 Filament PBR 渲染，支持工业级可视化

| 任务 | 内容 | 优先级 |
|------|------|--------|
| C1: Filament 集成 | Engine/SwapChain/Scene 初始化，Metal(mac)/Vulkan(win) | P1 |
| C2: 增量 Mesh 更新 | Dirty Region 局部重构 → Filament VertexBuffer 更新 | P1 |
| C3: 双面材质 | 切削面（橙色）vs 原始面（金属色）区分渲染 | P2 |
| C4: 面元点云渲染 | microGrid 中 active surfels 实时显示（调试/验证用） | P2 |
| C5: 刀具路径可视化 | 刀具轨迹线 + 当前位置动画 | P2 |

### Phase D: 应用层与交互

**目标**：完整的用户交互工作流

| 任务 | 内容 | 优先级 |
|------|------|--------|
| D1: G-code / NC 路径解析 | 读取标准 G-code，生成刀具路径序列 | P1 |
| D2: 仿真播放控制 | 播放/暂停/步进/回退 | P1 |
| D3: 状态快照与回滚 | RCU 或 Copy-on-Write 机制支持 Undo | P2 |
| D4: 碰撞检测 | 刀柄/夹具 vs 工件干涉检查 | P2 |
| D5: 测量工具 | 点到面距离、截面分析 | P3 |

### Phase E: 性能优化

**目标**：满足工业级实时性要求

| 任务 | 内容 | 优先级 |
|------|------|--------|
| E1: SIMD/AVX 优化 | ToolSweepSDF eval 批量计算向量化 | P2 |
| E2: GPU 加速 | CUDA/Metal compute shader 并行 SDF 评估 | P3 |
| E3: RCU 内存紧凑化 | 定期合并 inactive surfels，释放碎片内存 | P2 |
| E4: LOD 渲染 | 远处用粗网格，近处用精细网格 | P3 |
| E5: 异步切削管线 | 切削计算与渲染解耦，双缓冲 | P2 |

---

## 4. 里程碑计划

| 里程碑 | 目标 | 预计时间 | 交付物 |
|--------|------|----------|--------|
| **M1: 计算内核 v1.0** | Phase A 完成，公开 API 稳定 | 4周 | libyggdrasil.a + API 文档 + 测试通过率 >95% |
| **M2: 工业输入支持** | Phase B1-B2 完成，STL 毛坯可用 | +3周 | STL 导入 → 切削 → 导出 端到端验证 |
| **M3: 可视化 MVP** | Phase C1-C2 + D1-D2 | +4周 | Filament 渲染 + G-code 播放完整演示 |
| **M4: 产品化** | 全部 P1 任务完成 | +4周 | 跨平台发布包（Win/Mac/Linux） |

---

## 5. 技术架构（目标状态）

```
┌─────────────────────────────────────────────────────────────┐
│                     Application Layer                         │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐   │
│  │ G-code   │  │ Playback │  │ Measure  │  │ Export   │   │
│  │ Parser   │  │ Control  │  │ Tools    │  │ (STL/VDB)│   │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘   │
├───────┼──────────────┼──────────────┼──────────────┼────────┤
│       │         Engine API (ygg::Engine)           │         │
├───────┼──────────────┼──────────────┼──────────────┼────────┤
│                     Core Library                             │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │ Resolution   │  │   Billet     │  │  ToolSweep   │      │
│  │   Solver     │  │   Builder    │  │    SDF       │      │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘      │
│         │                  │                  │              │
│         └──────────────────┼──────────────────┘              │
│                            ▼                                 │
│              ┌──────────────────────────┐                    │
│              │     CuttingEngine        │                    │
│              │  ┌────────┐ ┌────────┐   │                    │
│              │  │MacroSDF│ │MicroPts│   │                    │
│              │  │(Float) │ │(PtData)│   │                    │
│              │  └────────┘ └────────┘   │                    │
│              └──────────────────────────┘                    │
├──────────────────────────────────────────────────────────────┤
│                     Rendering Layer                           │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐                  │
│  │ Filament │  │ VDBToMesh│  │ ImGui    │                  │
│  │ Engine   │  │ (局部更新)│  │ Panels   │                  │
│  └──────────┘  └──────────┘  └──────────┘                  │
├──────────────────────────────────────────────────────────────┤
│  Platform: GLFW | TBB | OpenVDB | Filament                  │
│  OS: Windows (MSVC) | macOS (Clang) | Linux (GCC)           │
└──────────────────────────────────────────────────────────────┘
```

---

## 6. 公开 API 设计草案（M1 目标）

```cpp
namespace ygg {

/// 引擎配置
struct EngineConfig {
    double tolerance;        // 精度要求 (mm)
    double toolRadius;       // 最小刀具半径 (mm)
    size_t memoryBudget;     // 内存预算 (bytes)
};

/// 刀具路径段
struct ToolMove {
    ToolType type;           // BALL_END, FLAT_END, BULL_NOSE
    double R, r, H;          // 刀具几何参数
    Vec3d start, end;        // 起止点
    // 未来扩展: Vec3d axis_start, axis_end; // 5轴
};

/// 引擎主接口
class Engine {
public:
    /// 初始化毛坯
    void buildBillet(const GeometryDef& geo, const EngineConfig& cfg);

    /// 执行单步切削
    void cut(const ToolMove& move);

    /// 执行路径序列
    void cutPath(const std::vector<ToolMove>& path);

    /// 查询
    double volume() const;
    const FloatGrid::Ptr& sdfGrid() const;
    const PointDataGrid::Ptr& microGrid() const;

    /// Mesh 导出（局部/全局）
    MeshData extractMesh() const;
    MeshData extractMesh(const BBoxd& region) const;

    /// 内存统计
    MemoryStats memStats() const;
};

} // namespace ygg
```

---

## 7. 质量标准

| 维度 | 标准 |
|------|------|
| 精度 | active surfel 到理论表面距离 < d_v |
| 性能 | 单步切削 < 100ms（30mm 路径，D_v=0.5mm） |
| 内存 | 500×500×200mm 毛坯 + t=0.01mm：< 500MB |
| 测试覆盖 | 核心算法分支覆盖 > 90% |
| 跨平台 | Windows/macOS/Linux 编译零错误 |
| 线程安全 | 无 data race（ThreadSanitizer 验证） |

---

## 8. 风险与缓解

| 风险 | 影响 | 缓解 |
|------|------|------|
| PointDataGrid 动态注入性能 | 高频切削内存碎片 | Phase E3 RCU 紧凑化 |
| 5轴扫掠体 SDF 精度 | 旋转插值误差 | 自适应采样密度 |
| Filament 跨平台兼容 | Metal/Vulkan 差异 | 先 OpenGL 原型验证功能，后迁移 |
| 大规模 G-code 回放性能 | 万段路径实时播放 | 异步管线 + 批量切削合并 |

---

## 9. 下一步行动（M1 启动）

1. **定义 `ygg::Engine` 接口**（A1）— 本周
2. **重写 cutDualTrack**（A2）— 基于原型 bug 修复，生产级实现
3. **实现延迟注入**（A3）— 补充 IPW₀ 机制
4. **多段路径支持**（A4）— 循环调用 cut
5. **测试加固**（A7）— 边界条件回归测试

---

## 修订记录

| 版本 | 日期 | 修订内容 |
|------|------|----------|
| v0.a | 2026-06-10 | 初稿：项目总体规划，基于原型研究结论 |
