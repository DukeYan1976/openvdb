# **MachiningSim MVP 开发计划与架构规范**

**文档版本**: 1.0

**作者**: Duke, Gemini

**更新日期**: 2026-06-01

**项目定位**: 跨平台工业级机加工仿真软件 MVP

## **1\. 架构总览与约束条件 (Context for AI)**

此阶段目标是搭建一个高扩展性的、支持多平台的机加工仿真基础框架，采用 Agile \+ TDD 驱动开发。

* **平台支持**: 核心逻辑完全跨平台。macOS (ARM64/x86\_64, Metal), Windows (x64, Vulkan/D3D12), Linux。  
* **技术栈**:  
  * 语言: C++17  
  * 构建: CMake 3.19+  
  * 算法层: OpenVDB (体素几何), TBB (多线程)  
  * 渲染层: Google Filament (PBR 渲染)  
  * 窗口与事件: GLFW  
  * UI 层: Dear ImGui  
  * 测试框架: GoogleTest (GTest)  
* **硬性约束**:  
  1. **绝对跨平台**: 业务逻辑禁止出现特定 OS 的 API，窗口句柄绑定必须使用 \#if defined(\_\_APPLE\_\_) 等宏隔离。  
  2. **TDD 原则**: 核心算法（如刀具扫掠体积生成、VDB 布尔运算）必须先编写 GTest 单元测试。  
  3. **零废话编码**: 代码需具备工业级鲁棒性，资源遵循 RAII 原则，避免内存泄漏。

## **2\. 迭代开发计划 (Agile Sprints)**

请 AI 辅助编程工具严格按照以下阶段顺序执行，完成一个阶段并验证通过后，再进入下一阶段。

### **Sprint 1: 基础设施与跨平台底座 (Platform Support)**

| 任务 ID | 模块 | 任务描述 (AI Prompt) | 验收标准 (Acceptance Criteria) |
| :---- | :---- | :---- | :---- |
| **1.1** | CMake | 配置全平台 CMakeLists.txt，集成 GLFW, OpenVDB, TBB 依赖，预留 Filament 二进制链接逻辑。 | Windows/macOS 均能成功生成构建文件。 |
| **1.2** | 窗口桥接 | 编写 main.cpp，初始化 GLFW，实现 GetNativeWindowHandle 函数隔离 OS 差异。 | 编译通过，能弹出一个跨平台的空白窗口。 |
| **1.3** | TDD 环境 | 集成 GoogleTest，创建 tests 目录和基本的测试 CMake 配置。 | 运行 ctest 输出 passed。 |

### **Sprint 2: 渲染引擎与 UI 融合 (High-efficiency Rendering)**

*注意：Filament 采用 PBR 物理渲染，需管理好材质实例和显存 Buffer。*

| 任务 ID | 模块 | 任务描述 (AI Prompt) | 验收标准 (Acceptance Criteria) |
| :---- | :---- | :---- | :---- |
| **2.1** | Filament | 在 main.cpp 中初始化 Filament Engine (Mac-\>Metal, Win-\>Vulkan)，挂载 SwapChain。创建基本的光照、相机和空场景。 | 窗口不再是黑屏/白屏，显示 Filament 默认清屏色。 |
| **2.2** | ImGui | 引入 Dear ImGui 及其 Filament backend (backend/include/imgui)。搭建侧边栏面板骨架。 | 渲染循环中成功绘制 ImGui 悬浮窗，FPS \> 60。 |
| **2.3** | 材质系统 | 加载预编译的 .filamat 文件。创建一个基础的静态渲染实体 (Renderable) 用于后续测试。 | 成功解析材质文件不报错。 |

### **Sprint 3: 核心算法 MVP (Scalability & Machining Logic)**

*关键纠偏：原始方案中全量 volumeToMesh 耗时过长，MVP 阶段虽简化处理，但需封装为增量更新接口，为扩展留空间。*

| 任务 ID | 模块 | 任务描述 (AI Prompt) | 验收标准 (Acceptance Criteria) |
| :---- | :---- | :---- | :---- |
| **3.1** | 算法层 TDD | 编写测试：利用 OpenVDB 创建工件 (Box) 和刀具 (Cylinder)，进行布尔减运算 (CSG Difference)。 | ctest 验证布尔减运算后 VDB 的 Active Voxel 数量正确减少。 |
| **3.2** | 数据转换 | 编写 VDBToMesh 转换模块，将 OpenVDB 提取为 Filament 可识别的 MeshVertex (Pos+Norm) 和 Indices。 | 转换逻辑时间复杂度可控，内存无泄漏。 |
| **3.3** | 渲染同步 | 实现动态 Buffer 更新逻辑。当发生切削（用户通过 ImGui 点击“模拟一步”）时，重新提取局部网格并更新 Filament VertexBuffer。 | 界面中工件体积发生可视化的被切削变化，程序不崩溃。 |
| **3.4** | UI 扩展 | 在 ImGui 面板中显示：1. 当前 FPS；2. 工件剩余体积 (通过 VDB 计算)；3. 模拟控制按钮。 | UI 数据实时响应后台算法变化。 |

## **3\. 风险评估与架构建议 (Critical Evaluation)**

为确保 AI 生成代码的质量，特设定以下评估准则：

1. **渲染瓶颈风险 (置信度评级：高)**  
   * *问题*: OpenVDB 的 volumeToMesh 即使在多核 TBB 加速下，对于高分辨率网格的全量提取仍无法满足 60FPS 的实时渲染需求。  
   * *缓解策略要求*: 要求 AI 在设计 VDBToMesh 模块时，**必须引入 Bounding Box 脏区标记（Dirty Region）机制**，仅对发生切削的局部 Voxel 块进行重构，或者考虑使用 OpenVDB 的延迟网格化策略。  
2. **跨平台二进制依赖风险 (置信度评级：中)**  
   * *问题*: Filament 和 OpenVDB 的第三方静态/动态库在不同系统下路径和链接符号复杂。  
   * *缓解策略要求*: CMake 脚本必须严格解耦，提供明确的 find\_package 或预编译库查找路径断言，找不到依赖时需给出明确的报错信息，而非静默失败。

**给 AI Coding Agent 的最终指令：**

请确认你已理解上述架构、技术栈以及 TDD 规范。如果理解，请直接开始生成 **Sprint 1** 所需的 CMakeLists.txt 以及基础目录结构脚本。