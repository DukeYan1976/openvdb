# IPG_C / IPW_C 计算表达设计报告

| 项目 | 内容 |
|---|---|
| 版本 | 0.a |
| 日期 | 2026-08-27 |
| 状态 | Draft for review |
| 范围 | IPG/IPG_C、IPW/IPW_C 的概念、输入归一化、双向转换、核心数据结构与算法决策 |

> 说明：本文将需求第 5 项中的 `IPW_W` 按上下文解释为 `IPW_C`。本文中的公式与参数是架构约束和规划依据；具体阈值必须通过基准模型、误差验证和性能测试标定。

## 关键设计决策

1. **多源几何统一适配为 IPG_C。** 精确 BREP、Mesh 和点云均通过 `GeometrySourceAdapter` 进入同一种 IPG_C 布局；来源误差必须进入误差预算，IPG_C 不得产生高于来源质量的精度保证。
2. **取消独立 MicroGrid/PointDataGrid 权威层。** MacroVoxel 内直接使用固定 `8 x 8 x 8` 的 CEBlock；SurfaceSample 不再依附 OpenVDB LeafNode 的 PointData。
3. **CE 是分类、误差证明和样本归属的最小计算单元。** SurfaceSample 只允许属于 Boundary CE；但 Boundary CE 可以仅保留来源几何引用或重建配方，暂不常驻样本。
4. **OpenVDB 管稀疏空间索引，SamplePool 管点法数据。** 样本逻辑上归属 CE，按 MacroVoxel 批量索引，物理上存放在独立、分页、可压缩的 SamplePool 中。
5. **IPG_C 静态、IPW_C 动态，但可在阶段边界双向转换。** IPG_C 可转为 IPW_C 并立即作为毛坯参与仿真；已提交的 IPW_C 也可冻结为新的 IPG_C，继续参与 CAM 计算。
6. **`intol/outol` 是硬约束，`voxelSize` 是内部决策。** 分辨率规划必须综合误差、曲率、最小特征、刀具、加工阶段、内存和 GPU 工作集，不能只使用 `voxelSize = K * tolerance`。
7. **转换采用同构共享、异构重采样的混合策略。** 兼容布局共享只读页并在 IPW_C 首次写入时按块复制；不兼容布局在阶段切换时一次性重采样和验证，使转换后的 CAM 与仿真热路径不承担额外分支。

---

## 1. 核心概念与定义

### 1.1 四个顶层对象

| 概念 | 定义 | 权威性与生命周期 |
|---|---|---|
| **IPG** | In-Process Geometry，代表一个 CAM Operation 的加工目标（Machining Feature）及其加工语义。其几何来源通常是精确 BREP，少数情况下是 Mesh 或点云。 | 在当前 CAM Operation 内静态；在其已知精度和完备性范围内，是该加工目标的语义与来源真值。 |
| **IPG_C** | IPG Compute Representation，由 IPG 的 BREP、Mesh 或点云来源统一派生，面向刀路和几何查询。【可能的实现方案：包含 MacroGrid、CEBlock、误差证书、来源几何引用、Machining Feature 绑定和 SurfaceSample。】 | 不可变计算快照；可由 IPG 重建，也可由已提交的 IPW_C 转换生成。不是 IPG 或来源几何本身。 |
| **IPW** | In-Process Workpiece，某一加工时刻实际存在的在制件材料状态。 | 随切削演化；是物理对象的语义概念。 |
| **IPW_C** | IPW Compute Representation，用于生成、更新和查询 IPW 的切削仿真计算模型。可由独立毛坯或 IPG_C 初始化，以稀疏 CE 状态、局部场和切削历史表达材料变化。 | 动态；局部增量更新，是仿真中的当前计算状态；可在已提交 epoch 上转换为 IPG_C。 |

```mermaid
flowchart LR
    A1[精确 BREP] --> I[IPG\nCAM Operation 的 Machining Feature]
    A2[Mesh] --> I
    A3[点云] --> I
    I --> A[GeometrySourceAdapter]
    A -->|统一派生、可重建| B[IPG_C\n不可变 CAM 计算表达]
    S[独立初始毛坯] --> C[IPW_C\n动态切削仿真表达]
    B -->|ToIPW_C\n作为毛坯| C
    T[刀具与运动轨迹] -->|扫掠体 / 切削事件| C
    C -->|解释当前材料状态| D[IPW\n在制件]
    C -->|ToIPG_C\n冻结已提交状态| B2[新 IPG_C 快照]
    B -->|刀路、余量、碰撞等查询| E[CAM 计算]
    B2 -->|再规划 / 下一加工阶段| E
    C -->|切削结果、余量、可视化| E
```

### 1.2 空间层级

| 层级 | 核心职责 | 不承担的职责 |
|---|---|---|
| **MacroGrid** | 基于 OpenVDB 的稀疏空间目录；定位活跃 MacroVoxel，支持拓扑遍历、邻域查询和按块调度。 | 不直接存放海量 SurfaceSample 点法数据。 |
| **MacroVoxel** | MacroGrid 中的活跃空间块。对 Boundary 区域挂接一个 CEBlock，并作为 CPU/GPU 传输、分页和并行任务的主要批次。 | 不是样本的最小语义所有者。 |
| **CEBlock** | 一个 MacroVoxel 内固定 `8^3 = 512` 个 CE 的紧凑数组。固定布局有利于连续访问、SIMD 和 Metal 线程组映射。 | 不形成独立 MicroGrid 数据层。 |
| **CE** | Computational Element，分类、误差证明、局部几何查询和 SurfaceSample 归属的最小计算单元。 | 不拥有独立堆分配；不必为每个 Boundary CE 常驻样本。 |
| **SurfaceSample** | 从 BREP、Mesh、点云重建边界或当前仿真边界派生的点法样本，用于接触、局部曲面重建或其他需要显式表面信息的算法。其可信度受来源误差限制。 | 不是权威几何；Air/Solid CE 中禁止存在。 |

设 MacroVoxel 边长为 `V`，固定 CE 分辨率为 `8`，则 CE 边长：

```text
h = V / 8
```

CE 的稳定空间标识建议为：

```text
CEKey = (macroCoord.x, macroCoord.y, macroCoord.z, ceLocalId)
ceLocalId in [0, 511]
```

`ceLocalId` 可采用 Morton 顺序或固定线性顺序，但必须在 CPU、Metal 和序列化格式中统一。

### 1.3 CE 状态与有证据分类

CE 至少包含三种语义状态：

- `SOLID`：整个 CE 被证明位于材料内部；
- `AIR`：整个 CE 被证明位于材料外部；
- `BOUNDARY`：不能证明为纯 Solid/Air，或已确认与边界相交。

本文统一使用与 OpenVDB level set 一致的符号约定：

```text
phi(x) < 0  表示材料内部
phi(x) > 0  表示空气/材料外部
phi(x) = 0  表示边界
```

若来源适配器或历史实现采用相反符号，必须在 `GeometrySourceAdapter` 或进入 `GridSpec` 前一次性归一化；CE 状态判断、误差证明、布尔组合、缓存文件和 GPU kernel 均以该 OpenVDB 符号约定为准，不以既有代码的临时约定为准。

在 CE 中心 `c` 计算近似场值 `phi_hat(c)`，其保守误差上界为 `epsilon(c)`；立方 CE 的中心覆盖半径为：

```text
rho = sqrt(3) * h / 2
```

若场满足局部 1-Lipschitz 条件，或 `rho` 已乘入可靠的局部 Lipschitz 上界，则采用以下分类：

```text
SOLID iff phi_hat(c) + epsilon(c) < -rho
AIR   iff phi_hat(c) - epsilon(c) >  rho
otherwise BOUNDARY_CANDIDATE
```

`BOUNDARY_CANDIDATE` 必须进一步执行来源适配器支持的有界相交检查、局部求根、覆盖检查或保守保留。该规则的目的不是“猜测”边界，而是只在具有证明时将 CE 归为纯 Solid/Air。

### 1.4 SurfaceSample 的所有权

SurfaceSample 使用三层关系：

1. **逻辑所有权：CE。** 每个样本只属于一个 Boundary CE，避免跨 CE 生命周期不清和重复释放。
2. **索引/调度粒度：MacroVoxel。** 一个 MacroVoxel 的 CE 样本目录和数据尽量批量构建、上传与回收。
3. **物理存储：独立 SamplePool。** OpenVDB 只保存轻量句柄，不承载变长点数组。

关键不变量：

```text
CE.state != BOUNDARY  =>  CE.sampleCount == 0
CE.sampleCount > 0    =>  CE.state == BOUNDARY
```

Boundary CE 不一定有常驻样本：只需保留可定位来源几何或重建边界的 `GeometryRef/SampleRecipe`，便可在需要时生成样本。`GeometryRef` 必须包含来源类型和稳定标识，不能假定其一定指向 BREP 实体。

### 1.5 MicroGrid 的处理结论

取消的是**独立、可被误认为几何真值的 MicroGrid/PointDataGrid 数据层**，不是取消 MacroVoxel 内的细粒度计算。

- 保留：固定 `8^3` CEBlock、CE 局部坐标、CE 分类和 CE 级样本归属；
- 删除：额外的 MicroGrid 拓扑、LeafNode PointData 作为 SurfaceSample 主存储、与 CEBlock 重复的空间层；
- 迁移：MicroGridLab 中有价值的分类、相交、细化和采样算法，改写为 CEBlock + SamplePool 算法。

---

## 2. IPG_C 与 IPW_C 的设计需求

### 2.1 共同需求

| 需求 | 设计含义 |
|---|---|
| 稀疏性 | 仅为有计算价值的 MacroVoxel/CE 分配数据，避免三维全域稠密化。 |
| 几何误差可控 | `intol/outol` 为硬约束；每一级近似都必须进入误差预算并可验证。 |
| 保守分类 | 不确定区域归 Boundary，不能将不确定性错误地归为 Air/Solid。 |
| 稳定索引 | CPU、GPU、缓存和序列化共享同一 MacroVoxel/CE 标识。 |
| 数据局部性 | 以 MacroVoxel 为批次组织 CEBlock、样本目录和 GPU 工作集。 |
| 可扩展存储 | 使用 64 位全局计数；避免每点指针和每 CE 小对象分配。 |
| GPU 可移植布局 | 使用 POD、offset/count、显式位宽和对齐；首个后端为 Metal Compute。 |
| 来源无关性 | BREP、Mesh 和点云通过统一评估接口进入相同 IPG_C 布局；来源差异不得渗入下游 CAM 热路径。 |
| Operation 语义 | IPG 的 CAM Operation 标识、Machining Feature 标识及角色约束必须随 IPG_C 保留；计算快照不得丢失其加工目标上下文。 |
| 可验证性 | 支持对来源几何或独立高精度参考结果做单边误差、分类和拓扑验证，并显式记录来源不确定度。 |
| 可转换性 | IPG_C 与 IPW_C 共享空间、误差和编码契约，以支持有验证、有溯源的双向转换。 |

### 2.2 特色需求对比

| 维度 | IPG_C | IPW_C |
|---|---|---|
| 权威来源 | IPG：当前 CAM Operation 的 Machining Feature，通常以精确 BREP 表示，少数为 Mesh 或点云。 | 初始毛坯、刀具扫掠和已提交切削历史；也可由 IPG_C 初始化。 |
| 变化原因 | 只因目标精度、查询策略或缓存层级改变。当前 Operation 的 Machining Feature 在该 operation 内不变。 | 每次切削事件都可能改变局部材料和边界。 |
| 更新模式 | 构建后不可变；新精度生成新快照。 | 局部增量更新；按 dirty region 重算。 |
| 样本生命周期 | 当前有效精度快照内稳定并优先 CPU 常驻。 | 可生成、失效、释放和按需再生。 |
| 一致性模型 | 快照发布；读者看到完整旧版或新版。 | 版本号/epoch；切削事务提交后原子发布受影响块。 |
| 主要优化目标 | 查询吞吐、压缩率、重复使用和确定性。 | 更新延迟、脏区规模、回收效率和 GPU 工作集命中率。 |
| 阶段转换 | 可作为 IPW_C 毛坯；同构时共享只读页，异构时重采样。 | 可在已提交 epoch 上冻结为新的 IPG_C，供下一阶段 CAM 使用。 |
| 失败策略 | 构建、来源验证或转换验证失败时拒绝发布新快照。 | 单个切削事务或转换失败时保留上一已提交状态。 |

---

## 3. IPG_C 设计决策

### 3.1 总体架构

```mermaid
flowchart TB
    A1[精确 BREP] --> I[IPG\nCAM Operation 的 Machining Feature]
    A2[Mesh] --> I
    A3[点云] --> I
    I --> A[GeometrySourceAdapter]
    A --> B[GeometryEvaluator\n场、投影、法向、曲率/特征、相交、误差界]
    R[intol / outol\n刀具、特征、加工阶段\n内存与 GPU 预算] --> P[Resolution Planner]
    B --> M[OpenVDB MacroGrid\n稀疏 MacroVoxel 索引]
    P --> M
    M --> C[CEBlock\n每块 512 CE]
    B --> C
    C --> D[CE Sample Directory\noffset / count / flags]
    D --> S[独立 SamplePool\nArena -> Page -> Sample]
    B --> G[GeometryRef / SampleRecipe]
    G --> D
    C --> V[误差与分类验证]
    S --> V
    V -->|通过后发布| Q[Immutable IPG_C Snapshot]
```

架构原则：

- `IPG` 拥有 CAM Operation 的加工目标语义、Machining Feature 标识和来源描述；`IPG_C` 必须保留可查询的 Operation/Machining Feature 绑定；
- `GeometrySourceAdapter` 负责来源校验、坐标/单位/OpenVDB level-set 符号归一化和来源误差建模；
- `GeometryEvaluator` 是 IPG_C Builder 唯一使用的几何查询接口，屏蔽 BREP 内核、Mesh 加速结构和点云重建方法；
- BREP 适配器优先提供解析或容差有界查询；Mesh 适配器携带离散化误差；点云适配器携带测量、配准、法向估计和表面重建误差；
- `MacroGrid` 负责“在哪里”，`CEBlock` 负责“是什么状态”；
- `SamplePool` 负责“显式表面样本在哪里”；
- IPG_C 以不可变快照发布，避免读取过程中精度层被原地替换。

三类来源最终生成相同的 IPG_C 查询和存储格式。来源类型只保留在 `SourceDescriptor/GeometryRef`、误差证书和诊断信息中；刀路、碰撞、余量等下游算法不得通过来源类型选择不同的 CE 查询路径。

### 3.2 Resolution Planner：从硬误差反推内部精度

`intol/outol` 表示最终计算结果允许的内向/外向几何或弦高误差，不等同于 OpenVDB `voxelSize`。规划器输入至少包括：

- `intol/outol` 及其法向/OpenVDB level-set 符号约定；
- 来源类型、来源自身精度、覆盖完整性及可认证程度；
- 可靠的局部曲率和曲率变化；若来源不能可靠提供，则使用保守估计或局部细化；
- 最小特征、间隙、薄壁和尖角尺度；
- 刀具直径、刀尖/接触区曲率及步距；
- 粗加工或精加工场景；
- CPU 内存、构建时间和 Metal 工作集预算。

规划器输出：

- MacroVoxel 边长 `V` 和 CE 边长 `h = V/8`；
- 活跃区域与必要的局部细化等级；
- 每个曲面片/Boundary CE 的采样间距或 `SampleRecipe`；
- 分类、场近似、采样、数值计算各自的误差预算。

推荐将单边误差拆分为可审计预算：

```text
E_in  = E_source,in  + E_field,in  + E_class,in  + E_sample,in  + E_numeric,in  <= intol
E_out = E_source,out + E_field,out + E_class,out + E_sample,out + E_numeric,out <= outol
```

其中 `E_source` 是来源模型相对真实目标几何的已知误差：BREP 通常由建模内核容差给出，Mesh 来自离散化/修复误差，点云来自测量、配准和重建误差。若 `E_source` 未知或无法形成保守上界，IPG_C 只能标记相应区域为 `UNCERTIFIED/DEGRADED`，不能仅靠加密 CE 或增加 SurfaceSample 宣称满足更严格的 `intol/outol`。

对局部最大主曲率 `kappa`、样本间距 `delta`，圆弧弦高估算为：

```text
E_chord ~= kappa * delta^2 / 8
delta <= sqrt(8 * E_sample / kappa)
```

该式只用于初始规划；尖角、修剪边、曲率不连续、小特征，以及 Mesh/点云中的噪声和采样空洞，必须使用来源适配器提供的拓扑、覆盖与误差约束进行验证。`voxelSize = K * min(intol, outol)` 仅可作为缺少局部信息时的保守启动值，不能作为最终精度证明。

#### 粗加工策略

- 在满足当前刀路、余量和防过切硬约束的前提下，取较大的 `V/h`；
- 对刀具不可达、远离接触区且没有薄壁/小间隙的区域降低样本密度；
- 对保护面、余量边界和小于刀具但影响可达性的特征保持保守分类；
- 使用粗级快照，不降低最终结果所需的单边误差保障。

#### 精加工策略

- 以最终 `intol/outol` 和局部曲率控制样本间距；
- 对刀具接触区、尖角、修剪边、薄壁和高曲率区优先细化；
- 将更多预算分配给接触/曲面重建，减少场量化和采样误差；
- BREP 来源通过内核回查最终刀位或接触结果；Mesh/点云来源通过其独立加速结构、保留样本或重建模型回查，不能只相信体素近似。

### 3.3 IPG_C 构建算法

```text
1. 固定 CAM Operation 标识、Machining Feature 标识与角色、坐标系、单位、OpenVDB level-set 符号约定、来源类型、来源版本和来源质量元数据。
2. GeometrySourceAdapter 校验并归一化来源：BREP 校验内核容差和拓扑；Mesh 校验方向、流形/闭合性和离散化误差；点云校验覆盖、密度、噪声、法向/符号，并构建带误差界的场或表面重建。
3. Resolution Planner 将来源误差纳入预算，生成全局基准分辨率与局部细化策略。
4. 由来源几何的窄带包围范围创建稀疏 MacroGrid。
5. 对每个活跃 MacroVoxel 创建 CEBlock，计算 phi_hat 和 epsilon。
6. 使用有证据规则分类 CE；不确定者进入 BOUNDARY_CANDIDATE。
7. 对候选 CE 做来源适配器支持的有界相交/求根或保守覆盖检查，并记录 GeometryRef。
8. 按可靠曲率、特征、采样密度和误差预算生成 SurfaceSample 或 SampleRecipe。
9. 按 MacroVoxel 打包 CE 目录与样本，写入 SamplePool。
10. 对 intol/outol、来源覆盖、分类不变量和邻块连续性进行验证；可修复区域局部细化，不可认证区域显式降级或拒绝构建。
11. 全部验证通过后，原子发布来源可追溯的不可变 IPG_C 快照。
```

同一来源数据、相同适配器/重建策略版本和相同构建参数应产生确定性布局，便于缓存、比较和回归测试。

### 3.4 SamplePool 设计

#### 地址与分页

建议三级物理结构：

```text
SamplePool
  -> Arena（建议约 256 MiB）
      -> Page（建议约 64 KiB）
          -> 连续 Sample 记录
```

- 全局样本数、容量和文件偏移使用 64 位；
- Arena 内偏移使用 32 位，降低 CE 目录和 GPU 地址成本；
- CE 不保存裸指针，而保存 `arenaId + offset + count + format + generation`；
- 同一 MacroVoxel 的样本尽量连续，CE 目录采用 offset/count 或前缀和；
- 大型重排只在单个 Arena 内执行，禁止为了压缩而复制整个 SamplePool。

推荐的逻辑句柄：

```cpp
struct SampleRange {
    uint32_t arenaId;
    uint32_t byteOffset;
    uint32_t count;
    uint16_t format;
    uint16_t generation;
};
```

具体 C++ 类型应在实现阶段根据对齐和跨平台序列化规则固定，并用静态断言验证大小。

#### 样本编码

典型压缩记录可控制在约 12 bytes/sample：

- 位置：相对 CE 或 MacroVoxel 的 3 个量化局部坐标；
- 法向：octahedral 编码的 2 个有符号分量；
- 其余位：曲面片/边界标志、精度等级或必要的局部属性。

若计算内核要求 16 字节对齐，可在 GPU staging buffer 中解码为 16 字节结构，而不扩大 CPU 主池。量化步长本身必须计入 `E_numeric`，并满足单边误差预算。

#### 超大规模策略

超过 `10^9` 个样本时，仅 12 bytes 的点法载荷已需要约 12 GB，尚未包含目录、页表、分配器和几何引用。设计上不应把“每个 Boundary CE 至少一个点”作为硬规则，而应依次采用：

1. 曲率/特征自适应采样，平坦区域使用更大间距；
2. Boundary CE 保留 `GeometryRef/SampleRecipe`，只为当前精度和算法确实需要的区域物化样本；
3. CPU 中保留压缩的当前有效快照，GPU 只上传活跃 MacroVoxel 工作集；
4. 使用 Arena/Page 局部回收和压缩，避免每点指针、每 CE 分配和全池双缓冲；
5. 若业务必须同时常驻十亿级样本，则显式配置 12--20 GB 级 CPU 内存预算，并以 64 位容量测试验证上限；不能依赖偶然可用内存。

更进一步的可选优化是以解析曲面片、Mesh 微面片或点云局部重建片作为平坦/规则区域的重建配方，只在接触查询前展开为临时点集。该优化不改变“IPG 来源模型及其质量边界是权威基准”的原则。

### 3.5 1 m x 1 m、0.01 mm 示例估算

曲面面积按 `1 m^2 = 10^6 mm^2` 计算：

| 策略 | 估算 | 结果 |
|---|---|---|
| 全局 `0.01 mm` 方格间距 | `10^6 / 0.01^2` | 约 `10^10` 点，不可接受；仅 12 B 载荷约 120 GB。 |
| `V = 0.3 mm`、`h = 0.0375 mm`，每个相交 CE 一个点 | `A / h^2`，再考虑表面方向因子 | 约 7.1 亿--12.3 亿点；载荷约 8.5--14.8 GB。 |
| 曲率/特征自适应 | 平坦区稀疏，高曲率和小特征区加密 | 工程目标约 2,000 万--1 亿点；12 B 载荷约 0.24--1.2 GB。 |

最后一项是容量规划目标，不是未经验证的精度保证。真实规模取决于曲率分布、边界/特征长度、薄壁/小间隙数量、刀具接触区域、Mesh/点云采样质量以及 `intol/outol` 的分配。构建前应先通过来源适配器统计这些量，再生成容量预测。

---

## 4. IPW_C 设计决策

### 4.1 状态模型

IPW_C 不以动态 BREP 为前提。其当前材料状态由以下信息共同确定：

- 独立初始毛坯场/CE 状态，或由 IPG_C 转换得到的初始材料快照；
- 已提交刀具扫掠体/切削事件；
- 受影响 MacroVoxel 中的当前 CE 分类和局部复合场；
- 当前边界的 SurfaceSample 或可再生配方。

由 IPG_C 初始化时，转换完成并发布的状态就是 IPW_C 的 `epoch 0`。此后 IPW_C 的修改必须遵循自己的事务、generation 和回收规则；即使底层页采用只读共享，语义上也不得回写或改变来源 IPG_C 快照。

若 `phi_W < 0` 表示当前工件内部，`phi_T < 0` 表示刀具扫掠体内部，则材料差集 `W \ T` 的符号正确组合可写为：

```text
phi_new = max(phi_old, -phi_T)
```

`max` 组合通常不是严格欧氏有符号距离；后续算法若依赖距离幅值而不只是符号，必须在局部重建、重新距离化或携带额外误差界。

### 4.2 增量切削算法

```text
1. 将刀具运动离散或构造成带误差界的连续扫掠体。
2. 计算扫掠体包围盒与安全扩张，定位 dirty MacroVoxel 集合。
3. 为缺失块分配 CEBlock；读取现有块的稳定版本。
4. 在 dirty CE 上合成局部场并重新执行有证据分类。
5. 状态未变的 CE 保留原样；发生变化的 CE 增加 generation。
6. CE 变为 Air/Solid 时立即解除 SampleRange，并延迟回收旧页。
7. CE 变为 Boundary 时记录重建配方；按查询需求生成新 SurfaceSample。
8. 验证局部误差、邻块连续性和材料单调去除约束。
9. 提交新块目录；在 CPU/GPU 读者退出旧 epoch 后回收旧数据。
```

材料去除仿真默认应满足单调性：切削事件不能把 Air 变回 Solid。若未来加入增材、弹性回弹或热变形，它们应使用独立事件类型和状态转移规则，不能悄悄破坏此不变量。

### 4.3 CE 与 SurfaceSample 生命周期

```mermaid
stateDiagram-v2
    [*] --> SolidOrAir: 初始化 / 有证据分类
    SolidOrAir --> Boundary: 切削使边界进入 CE
    Boundary --> BoundarySampled: 查询触发样本生成
    BoundarySampled --> Boundary: 精度变化或切削导致样本失效
    Boundary --> SolidOrAir: 重新分类为纯区域
    BoundarySampled --> SolidOrAir: 切削后不再与边界相交
    SolidOrAir --> Reclaimed: MacroVoxel 不再活跃
    Boundary --> Reclaimed: 块被淘汰
    BoundarySampled --> RetiredPages: 解除 SampleRange
    RetiredPages --> Reclaimed: CPU/GPU epoch 安全
```

生命周期规则：

- `generation` 绑定 CE 几何状态与 SampleRange，旧 generation 的样本永远不能服务新查询；
- 失效先解除可见索引，再等待 CPU 引用和 Metal command buffer 完成，最后回收页；
- 频繁变化块使用空闲页链表或大小类别，避免全池紧缩；
- 样本按需生成，但同一 generation 内结果应确定或携带明确随机种子；
- GPU 只接收当前任务需要的块目录、CEBlock 和样本页，不镜像整个池。

### 4.4 IPW_C 的一致性与恢复

- 每次刀具扫掠作为一个切削事务，先构建新版本，验证后发布；
- MacroVoxel 目录保存版本/epoch，读者不观察到半更新的 CEBlock；
- 失败时丢弃未发布的新页和块，继续使用上一已提交状态；
- 切削历史保存足以重放的刀具、轨迹、容差、策略版本和输入状态哈希；
- 定期生成检查点，避免从初始毛坯重放全部历史。

---

## 5. IPG_C 与 IPW_C 的复用及双向转换设计

### 5.1 建议复用的基础模块

| 模块 | 可复用内容 | 策略差异 |
|---|---|---|
| `GridSpec` | 坐标系、单位、MacroVoxel/CE 尺寸、局部编号、OpenVDB level-set 符号约定。 | IPG_C 通常固定；IPW_C 需携带版本。 |
| `MacroGridAdapter` | OpenVDB 稀疏拓扑、邻域遍历、块键和批量访问。 | IPW_C 增加 dirty-set 与写时复制。 |
| `CEBlockLayout` | 512 CE 的公共不可变 `CEBlockPayload`：状态位、误差字段、样本句柄。 | IPG_C 使用查询/来源 sidecar；IPW_C 将 generation/epoch/dirty 状态放入独立 `IPWBlockMeta`。 |
| `CertifiedClassifier` | `phi_hat/epsilon/rho` 的保守分类和测试向量。 | 几何场来源和误差模型不同。 |
| `ResolutionPlannerCore` | 单边误差预算、曲率/特征/刀具约束、容量估算。 | IPG_C 优化构建与查询；IPW_C 优化更新与活跃窗口。 |
| `SampleCodec` | CE 局部位置、法向压缩、误差评估和 Metal 解码。 | IPW_C 样本更频繁地重编码。 |
| `PagedSamplePool` | Arena/Page、64 位容量、32 位局部偏移、批量分配。 | IPG_C 偏向只读紧凑；IPW_C 需要延迟回收和空闲链。 |
| `GpuWorksetBuilder` | offset/count 缓冲、页上传、Metal 参数布局。 | IPG_C 可缓存；IPW_C 按 dirty/visible 集合刷新。 |
| `GeometryValidator` | 单边误差、分类不变量、邻块连续性和容量检查。 | IPG_C 对照来源适配器；IPW_C 对照扫掠事件与前一状态。 |
| `RepresentationConverter` | 兼容性判定、同构页共享、异构重采样、转换报告和发布事务。 | `ToIPW_C` 创建动态元数据；`ToIPG_C` 冻结并剥离动态状态。 |

### 5.2 必须共享的格式约束

1. 使用同一坐标系、长度单位和 `phi < 0` 表示材料内部的 OpenVDB level-set 符号约定；
2. 使用同一 `CEKey`、`ceLocalId` 编码和 MacroVoxel 边界归属规则；
3. 所有跨 CPU/GPU 结构使用显式位宽，不包含裸指针、虚函数或平台相关 `size_t`；
4. `SampleRange` 统一采用 offset/count，格式版本和量化尺度显式记录；
5. 所有误差字段声明单位、置信/保守语义和累计方式；
6. 文件/缓存头记录 schema、算法策略、来源类型、来源模型或切削历史哈希，拒绝静默读取不兼容数据；
7. 共享页具有独立于 IPG_C/IPW_C 对象生命周期的引用计数或等价所有权，且一旦被共享就保持只读。

### 5.3 转换契约与路径选择

转换发生在粗加工、精加工、余料再规划等阶段边界，预期频率较低。设计优先级依次为：转换结果正确且可追溯、转换后的热路径无额外开销、兼容场景快速切换、最后才是压缩一次性转换耗时。因此允许在阶段边界执行完整验证、必要的重打包或重采样，但不能让 CAM 查询或切削内核长期承担“当前对象来自哪种表示”的条件分支。

`RepresentationConverter` 接收已发布的来源快照、目标 `GridSpec`、目标误差预算、材料解释策略和存储策略，原子地产生目标对象及 `ConversionReport`。转换必须选择并报告以下一种路径：

| 路径 | 适用条件 | 数据动作 | 性能特征 |
|---|---|---|---|
| `DIRECT_SHARE` | 坐标系、`GridSpec`、CE 编码、OpenVDB level-set 符号、量化尺度、SampleCodec 和材料语义均兼容。 | 共享只读 MacroGrid/CEBlock/SamplePool 页；只创建目标对象的目录和生命周期元数据。 | 最低延迟和内存增量；后续首次写入按 MacroVoxel 复制。 |
| `REPACK` | 几何离散和 `GridSpec` 兼容，但目录、压缩格式或页布局不同。 | 不重新求值几何，只线性解码、重排和编码活跃块。 | 一次性 `O(active blocks + samples)`；目标热路径为原生布局。 |
| `RESAMPLE` | 变换、分辨率、误差预算或 CE/样本语义不兼容。 | 通过来源表示的统一查询接口重新求值、分类、采样和验证。 | 成本最高但通用；不得静默执行或伪装成无损转换。 |

目标误差不能优于来源快照及转换过程共同能够证明的精度：

```text
E_target >= E_source_snapshot + E_conversion
```

如果请求精度更严格，转换器只能回到仍可用的原始 BREP/Mesh/点云、以更高精度重放切削历史，或明确返回 `DEGRADED/REJECTED`；增加样本数量不能凭空恢复已经丢失的几何信息。任何路径失败时都不发布半转换对象，并保留原来源对象可用。

### 5.4 IPG_C -> IPW_C：作为毛坯进入仿真

```text
1. 要求来源 IPG_C 已发布且不可变，读取其 GridSpec、误差证书、来源哈希和转换清单。
2. 应用 MaterializationPolicy，确认哪些已分类体积构成真实毛坯；带合成封闭面或仅有开放表面的 IPG_C 不得静默生成虚假材料。
3. 根据兼容性选择 DIRECT_SHARE、REPACK 或 RESAMPLE。
4. 创建独立的 IPW_C 目录、epoch 0、generation、dirty-set、切削历史根和回收器。
5. DIRECT_SHARE 路径引用只读公共载荷；IPW_C 首次修改某个 MacroVoxel 时，先复制该块及必要的样本页，再在新版本上执行切削。
6. 验证材料解释、分类不变量、邻块连续性和误差预算后，原子发布 IPW_C epoch 0。
```

为了使“立即作为毛坯”可实现，公共数据应拆成不可变 `CEBlockPayload` 与对象专属 sidecar：IPG_C 只拥有查询/来源元数据，IPW_C 的 `generation/epoch/dirty/free-list` 放在 `IPWBlockMeta` 中。IPG_C 发布时生成包含 schema、块/页哈希和验证摘要的 `ConversionManifest`；直接转换只需验证清单和根对象，不必重新扫描全部 CE。

共享只发生在不可变载荷层。IPW_C 不得持有可回写 IPG_C 的裸指针，也不得复用 IPG_C 的可见目录作为可变目录。切削后的脏块、样本失效和页回收完全服从 IPW_C 的事务与 epoch 规则。

### 5.5 IPW_C -> IPG_C：仿真结果返回 CAM

```text
1. 完成或回滚正在执行的切削事务，等待相关 CPU 读者和 Metal command buffer 到达一致的已提交 epoch。
2. 为后续 CAM Operation 绑定既有或显式派生的 Machining Feature 上下文；转换本身不得臆造加工目标语义。
3. 固定该 epoch 的网格根、CEBlockPayload、样本页、误差状态和切削历史哈希。
4. 根据目标 GridSpec 和精度选择 DIRECT_SHARE、REPACK 或 RESAMPLE。
5. 构建只读 CAM 查询目录，移除或旁路 dirty-set、回收链和其他 IPW_C 动态 sidecar。
6. 验证当前边界、分类、Operation/Machining Feature 绑定、来源溯源和目标误差预算后，发布新的不可变 IPG_C 快照。
```

该 IPG_C 的 `OriginKind` 必须记录为 `IPW_SNAPSHOT`，并记录来源 IPW_C 标识、epoch、切削历史哈希、转换路径及后续 CAM Operation/Machining Feature 绑定；它是当前仿真材料状态的 CAM 计算表达，不等同于从仿真结果恢复出精确 BREP 或自动创建新的 Machining Feature。原始 Machining Feature/几何引用只有在能够证明对应表面未改变时才可保留；新切削表面使用稳定的 `SIMULATED_SURFACE` 标识，失效的 BREP/Mesh/点云引用必须移除或标记为历史溯源，不能继续用于几何回查。

发布后的 IPG_C 与继续演化的 IPW_C 生命周期独立。即使两者共享同一已提交块版本或样本页，IPW_C 的后续切削也只能创建新版本；既有 IPG_C 看到的内容永久不变。

### 5.6 转换性能与内存策略

- 阶段转换低频，因此允许在转换点进行同步、完整一致性检查和可选压缩；完成后生成原生 IPG_C 或 IPW_C 目录，查询/切削内核不判断转换来源；
- `DIRECT_SHARE` 以 Arena/Page 为引用计数单位，以 MacroVoxel 为写时复制单位，避免每个 Sample 或 CE 的原子引用计数；
- 若后续切削覆盖范围很大，写时复制可能同时保留大量旧页。转换器应统计共享页滞留率和复制放大率，并可在阈值触发时后台 detach/repack，但不得阻塞正在提交的切削事务；
- `REPACK/RESAMPLE` 可按 MacroVoxel 并行并使用有界 staging 内存，禁止为整个模型建立第二份未压缩样本池；
- 基准测试至少记录转换总时间、首个可用目标快照时间、共享/复制/重采样字节数、峰值内存、首次切削复制放大率，以及转换前后的 CAM 查询或 IPW 更新吞吐；
- 只有转换后吞吐与原生构建对象处于同一性能等级，且没有来源类型分支，才认为性能设计达标。

### 5.7 不应错误复用的部分

- 不把 IPG_C 的不可变样本生命周期直接用于 IPW_C；
- 不把 IPW_C 的局部复合场或由其生成的 IPG_C 当成精确 BREP；
- 不强求两个对象采用相同的常驻率、压缩策略或重建成本；
- 不共享可变实例；只共享不可变载荷、布局、算法核、验证器和接口契约；
- 不因支持转换而合并 IPG_C 与 IPW_C 的顶层类型或状态机。

---

## 建议的近期验证顺序

1. 用几何等价的 BREP、Mesh 和点云平面/球面建立来源适配、符号、来源误差和退化状态基准；
2. 用薄壁、窄槽、尖角、修剪曲面、非流形 Mesh、稀疏/含噪点云建立 CE 分类、覆盖检查与单边误差基准；
3. 将 MicroGridLab 的现有算法映射为 `MacroVoxel -> CEBlockPayload`，确认不再依赖独立 MicroGrid/PointDataGrid；
4. 实现统一输入的只读 IPG_C 原型：GeometrySourceAdapter + OpenVDB MacroGrid + CEBlock + 单 Arena SamplePool；
5. 对典型模具统计曲率/特征/采样质量分布，验证 2,000 万--1 亿样本目标是否现实；
6. 完成 64 位规模和 32 位 Arena 内偏移的边界测试，再接入 Metal 工作集；
7. 为公共载荷增加 IPW_C sidecar、dirty-set 和 epoch 回收，验证 `IPG_C -> IPW_C` 的 DIRECT_SHARE、首次写时复制及来源快照不变性；
8. 验证 `IPW_C -> IPG_C` 的 epoch 冻结、动态元数据剥离、模拟表面溯源和后续 IPW_C 更新隔离；
9. 对 GridSpec/量化不兼容场景验证 REPACK/RESAMPLE、误差不收窄规则、失败原子性及转换后热路径性能。

## 参考实现与规范位置

- OpenVDB level set 符号规范：[OpenVDB Overview](https://www.openvdb.org/documentation/doxygen/overview.html)
- IPG_C 需求：`../.kiro/specs/ipg-compute-representation/requirements.md`
- CE/MicroGridLab 原型：`../core/MicroGridLab.h`
- 增量 CE 与采样算法：`../core/MicroGridLab.cpp`
- Octree 提取：`../core/OctreeRefiner.cpp`
- GPU 分层切削规划：`../docs/superpowers/plans/layered-cut-architecture.md`
