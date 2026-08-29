md_content = """# 需求文档：IPG_C (在制品几何计算表示)

**作者**: Duke, Gemini  
**日期**: 2026-08-26  
**版本**: `IPG_C_Requirements_Document_20260826_v1.0.md`

---

## 1. 修订记录

| 版本 | 日期 | 概要 |
|---|---|---|
| 1.0 | 2026-08-26 | 综合审查：统一了内部为正的场约定，将启发式 CE 分类替换为可认证的保守分类，修正了特征/支撑语义，并增加了严格的分类证明。 |
| 1.a | 2026-08-26 | 最终一致性精炼，涵盖可认证评估、验证采样、特征交集、查询有效性和显式恢复。 |

---

## 2. 引言

IPG_C（In-Process Geometry — Compute Representation，在制品几何计算表示）是用于 CAM 计算的静态加工目标表示。IPG_C 复用了 IPW 所采用的成熟 OpenVDB 稀疏网格、`MicroGridCell` 和自适应表面采样概念，同时增加了从计算单元到源 BREP 特征的不可变链接。

IPG_C 在整个规范中使用统一的规范有符号距离场（Canonical Signed-Distance Field）：正值表示材料/内部（material/interior），负值表示空气/外部（air/exterior），零表示几何边界。该约定与现有的 `MicroGridCell` 符号约定一致。CE（计算单元）的分类是保守的：AIR（空气）和 SOLID（实体）是经过认证的整单元分类，而存储的 BOUNDARY（边界）仅意味着在现有精度下该 CE 无法被认证为 AIR 或 SOLID。因此，BOUNDARY CE 是一个边界候选单元（boundary candidate），并不必然意味着存在零交叉（zero crossing）。

FEATURE（特征）面表示真实的加工目标。SUPPORT（支撑）面可以将原本开放的加工表面封闭为水密实体（watertight solid），但 SUPPORT 几何体本身并不是加工目标。FEATURE/SUPPORT 角色会影响感知特征的查询和自适应表面采样；这些角色不会改变场精度、CE 拓扑或保守分类阈值。

---

## 3. 术语表

- **IPG_C**: 在制品几何计算表示（In-Process Geometry Compute Representation）——本文档规范的不可变加工目标计算表示。
- **IPG_C_Builder**: 校验输入几何体并构建 IPG_C 的组件。
- **IPW**: 动态在制品（In-Process Workpiece）——切削仿真过程中使用的演化工件表示。
- **Geometry_Field ($\varphi$)**: 规范的精确有符号距离场。$\varphi(x) > 0$ 表示材料/内部，$\varphi(x) < 0$ 表示空气/外部，$\varphi(x) = 0$ 表示几何边界。Geometry_Field 要求满足 1-Lipschitz 连续性。
- **Geometry_Field_Evaluator**: 返回 CERTIFIED（已认证）或 UNCERTIFIED（未认证）场评估结果以及（在有定义时）场梯度的接口。
- **Field_Estimate ($\hat{\varphi}(x)$)**: 在评估点 $x$ 处计算出的精确场值 $\varphi(x)$ 的估计值。
- **CERTIFIED**: 场评估结果状态，表明评估器已建立 Certified_Result 界限。
- **Certified_Result**: 包含有限值 $\hat{\varphi}(x)$ 和有限值 $\varepsilon(x) \ge 0$ 的场评估结果，满足 $|\hat{\varphi}(x) - \varphi(x)| \le \varepsilon(x)$。
- **Certified_Error ($\varepsilon(x)$)**: Certified_Result 中包含的有限非负误差界限。
- **UNCERTIFIED**: 当评估器无法建立 Certified_Result 界限时使用的场评估结果状态；任何随附的数值估计均不作为 Certified_Error。
- **External_Field_Adapter**: 将外部几何内核场转换为规范 Geometry_Field 符号约定的适配器。
- **Voxel**: OpenVDB 叶节点尺度的空间单元，当实例化为 `MicroGridCell` 时包含 $8 \times 8 \times 8 = 512$ 个 CE。
- **CE**: 计算单元（Computational Element）——Voxel 内部的一个轴对齐封闭立方体 $C$。
- **$c$**: CE 立方体 $C$ 的中心。
- **$s$**: CE 的边长，等于 $\text{voxelSize} / 8$。
- **$\rho$ (rho)**: CE 的外接球半径，等于 $\frac{\sqrt{3} \cdot s}{2}$。
- **MicroGridCell**: 成熟的 Voxel 载荷，包含 `activeMask[8]`、`signMask[8]` 以及用于兼容性的变长量化场向量 `sdf`。
- **AIR**: 经过认证的 CE 状态，断言对于 CE 中的每个点 $x$，均有 $\varphi(x) < 0$。
- **SOLID**: 经过认证的 CE 状态，断言对于 CE 中的每个点 $x$，均有 $\varphi(x) > 0$。
- **BOUNDARY_CANDIDATE**: 当无法认证为 AIR 或 SOLID 时使用的保守 CE 状态。兼容的存储状态名称为 BOUNDARY。BOUNDARY_CANDIDATE 可能与 $\varphi = 0$ 相交，可能完全为 AIR，也可能完全为 SOLID。
- **Navigation_Value**: 为 BOUNDARY_CANDIDATE 存储的量化、截断 Field_Estimate，用以支持近似导航。Navigation_Value 不是零交叉证书，也不能用于证明 AIR 或 SOLID。
- **Narrow_Band_Half_Width ($w$)**: 场量化范围，默认值为 $3 \times s$。
- **Narrow_Band**: 评估目标评估器精度的区域 $|\varphi(x)| \le w$。
- **BREP**: 边界表示（Boundary Representation）——由面（Face）、边（Edge）和顶点（Vertex）组成的 CAD 边界模型。
- **Feature_Entity**: 具有稳定不透明标识符的非退化 BREP 面、边或顶点。
- **Face_Role**: 面归属的标识，取值为 FEATURE 或 SUPPORT。
- **FEATURE**: 标识真实加工目标表面的 Face_Role。
- **SUPPORT**: 标识仅用于封闭拓扑的人造几何体的 Face_Role。
- **Feature_Precision**: 由 BREP 面拥有的正自适应采样精度。默认情况下，FEATURE 的全局公差为 $t$，SUPPORT 的全局公差为 $K_{\text{support}} \times t$（默认 $K_{\text{support}} = 10$）。
- **Inherited_Role**: 由边或顶点继承的关联 Face_Role 集合。当至少有一个关联面为 FEATURE 时，边或顶点具有 FEATURE 资格；当每个关联面均为 SUPPORT 时，仅为 support-only。
- **Entity_Effective_Precision**: 与 Feature_Entity 关联的所有面中 Feature_Precision 的最小值；面本身使用其自身的 Feature_Precision。
- **CE_Effective_Precision**: 与 CE 在几何上相交的 Feature_Entity 中的最小 Entity_Effective_Precision；当不存在 Feature_Entity 关联时，为全局公差 $t$。
- **Feature_Index**: 在几何相交的 BOUNDARY_CANDIDATE CE 与 Feature_Entity 之间的不可变双向关联。
- **SurfaceSample**: 采样的边界点及指向外部的单位法线。
- **Outward_Unit_Normal**: 在 Geometry_Field 可微且 $\nabla\varphi \neq 0$ 的位置，定义为向量 $-\text{normalize}(\nabla\varphi)$。负号来源于内部为正的场约定。
- **IPG_C_Query_Service**: 在 IPG_C 上执行空间、状态、角色过滤和特征查询的组件。
- **IPG_Comparison_Service**: 对齐对比 IPG_C 和 IPW 表示的组件。
- **Comparison_Eligible_CE**: 与至少一个 FEATURE 实体关联并被活动比对过滤器选中的 IPG_C BOUNDARY_CANDIDATE。
- **Genuine_Closed_Design_Solid**: 不包含人造 SUPPORT 面且不包含仅为拓扑封闭而创建的体积的水密设计实体。
- **Synthetic_Closure_Volume**: 其内部分类仅因 SUPPORT 面封闭了原本开放的加工表面而存在的体积。
- **DEGRADED**: 显式构建模式，当实现的窄带认证精度粗于 $t / 10$ 或部分评估保持未认证时使用。
- **UNRESOLVED**: 只读状态，在此状态下序列化的 Feature_Entity 标识符可用，但源 BREP 几何体不可用或未验证。
- **RECOVERY**: 显式反序列化模式，将识别到的已验证数据块从原本不支持的文件恢复到只读 UNRESOLVED 状态。
- **IPG_C_Serializer**: 读写便携式 IPG_C 文件格式的组件。
- **Construction_Domain**: IPG_C 构建所覆盖的有限声明世界空间区域。
- **Global_Tolerance ($t$)**: 支持范围 $0.001\text{ mm} \le t \le 0.5\text{ mm}$ 内的请求加工公差。

---

## 4. 需求规范

### 需求 1：具有可认证性的 IPG_C 网格构建

**用户故事**：作为 CAM 算法开发人员，我希望将加工目标几何体转换为保守的、兼容 MicroGridCell 的稀疏表示，以便 AIR 和 SOLID 单元在几何上可信。

#### 验收标准

1. **WHEN** 当提供全局公差 $t$ 时，IPG_C_Builder **SHALL** 推导 $\text{voxelSize} = \text{clamp}(K \times t, 0.02\text{ mm}, 5.0\text{ mm})$（可配置正系数 $K$ 默认为 30），推导 $s = \text{voxelSize} / 8$，并推导 $\rho = \frac{\sqrt{3} \cdot s}{2}$。
2. **WHEN** 当接受 Construction_Domain 和体素变换时，IPG_C_Builder **SHALL** 构建 OpenVDB 稀疏表示，其实例化的 Voxel 载荷保留已建立的 `MicroGridCell` 布局和 CE 索引。
3. **WHEN** 当 CE 中心评估返回包含 $\hat{\varphi}(c)$ 和 $\varepsilon(c)$ 的 Certified_Result 时，IPG_C_Builder **SHALL** 当且仅当 $\hat{\varphi}(c) - \varepsilon(c) > \rho$ 时将 CE 分类为 SOLID；当且仅当 $\hat{\varphi}(c) + \varepsilon(c) < -\rho$ 时分类为 AIR；否则分类为 BOUNDARY_CANDIDATE。
4. **IF** 若 CE 中心评估返回 UNCERTIFIED，则 IPG_C_Builder **SHALL** 将该 CE 分类为 BOUNDARY_CANDIDATE，将该 CE 记录为未认证，并输出诊断信息，而非将其分类为 AIR 或 SOLID。
5. **WHEN** 当 BOUNDARY_CANDIDATE 具有有限 Field_Estimate 时，IPG_C_Builder **SHALL** 存储 $q = \text{round}\left(\text{clamp}(\hat{\varphi}(c), -w, w) / w \times 32767\right)$ 作为兼容的量化 Navigation_Value，且不得将 $q$ 作为分类依据。
6. **WHEN** 当 BOUNDARY_CANDIDATE 缺乏有限 Field_Estimate 时，IPG_C_Builder **SHALL** 存储中性 Navigation_Value 0，并保留未认证诊断状态。
7. **WHEN** 当 CE 状态编码在 `MicroGridCell` 中时，IPG_C_Builder **SHALL** 对 BOUNDARY_CANDIDATE 使用 `activeMask = 1`；对 SOLID 使用 `activeMask = 0` 且 `signMask = 1`；对 AIR 使用 `activeMask = 0` 且 `signMask = 0`。
8. **WHEN** 当构建成功完成时，IPG_C_Builder **SHALL** 使网格和 Feature_Index 变为不可变，且不暴露任何切削、布尔求差或改变拓扑的操作。
9. **IF** 若输入包含退化面、零长度边、孤立顶点或零面积网格图元，则 IPG_C_Builder **SHALL** 报告实体标识符和类型，跳过退化实体，并重新校验剩余拓扑。
10. **IF** 若移除退化实体后的可用输入几何体不是封闭的可定向流形，则 IPG_C_Builder **SHALL** 拒绝构建，并报告需要使用明确标识的 SUPPORT 面进行上游封闭。
11. **IF** 若 Construction_Domain 或变换无法在不发生整数溢出的情况下将每个覆盖的 CE 映射到受支持的网格坐标，则 IPG_C_Builder **SHALL** 拒绝构建并报告不受支持的范围或分辨率。
12. **WHEN** 当执行基准构建时，IPG_C_Builder **SHALL** 在 Mac mini M4 和 ThinkPad X1 基准平台上均报告模型标识符、包围盒范围、公差、面数量、实例化 Voxel 数量、BOUNDARY_CANDIDATE 数量、场评估数量、特征关联数量、构建时间以及峰值内存。

### 需求 2：具有可认证性的几何场评估

**用户故事**：作为 CAM 算法开发人员，我希望通过一个具有认证性的、内部为正的场接口来评估 BREP 和网格输入，以便保守分类在不同几何来源和精度水平下保持有效。

#### 验收标准

1. Geometry_Field_Evaluator **SHALL** 为每次请求的场评估准确返回一个评估状态：CERTIFIED 或 UNCERTIFIED。
2. **WHEN** 当 Geometry_Field_Evaluator 在点 $x$ 处返回 CERTIFIED 时，Geometry_Field_Evaluator **SHALL** 返回满足 $|\hat{\varphi}(x) - \varphi(x)| \le \varepsilon(x)$ 的有限值 $\hat{\varphi}(x)$ 和有限值 $\varepsilon(x) \ge 0$。
3. **IF** 若 Geometry_Field_Evaluator 无法在有限值 $\hat{\varphi}(x)$ 和有限值 $\varepsilon(x) \ge 0$ 下确立 $|\hat{\varphi}(x) - \varphi(x)| \le \varepsilon(x)$，则 Geometry_Field_Evaluator **SHALL** 返回 UNCERTIFIED，且不得将任何附带的数值估计指定为 Certified_Error。
4. Geometry_Field_Evaluator **SHALL** 提供法线支持操作，在 Geometry_Field 可微的位置返回场梯度。
5. Geometry_Field_Evaluator **SHALL** 定义 Geometry_Field，使得材料/内部满足 $\varphi(x) > 0$，空气/外部满足 $\varphi(x) < 0$，封闭边界上满足 $\varphi(x) = 0$。
6. **WHEN** 当外部 CAD 内核或网格库返回传统的内部为负的有符号距离时，External_Field_Adapter **SHALL** 在 IPG_C 使用前对返回的场值和场梯度取反，并保留认证误差幅值。
7. **WHEN** 当在具有非零梯度的可微 Geometry_Field 位置推导外法线时，Geometry_Field_Evaluator **SHALL** 将法线计算为 $-\text{normalize}(\nabla\varphi)$。
8. **IF** 若在正常非 DEGRADED 模式下的任何 Narrow_Band 评估返回 UNCERTIFIED，或返回大于 $t / 10$ 的 Certified_Error，则 IPG_C_Builder **SHALL** 拒绝构建并报告受影响的评估。
9. **WHERE** 在选择了显式 DEGRADED 模式的情况下，**WHEN** 当一个或多个 Narrow_Band 评估返回 UNCERTIFIED 或大于 $t / 10$ 的 Certified_Error 时，IPG_C_Builder **SHALL** 允许构建，同时保留每个可用的 Certified_Result，并报告达到的认证精度、未认证评估数量以及受影响的 BOUNDARY_CANDIDATE 数量。
10. **WHEN** 当提供三角网格时，Geometry_Field_Evaluator **SHALL** 校验封闭可定向流形拓扑，并使用对于空间选择性查询其查询开销随面数量呈亚线性增长的空间加速结构。
11. **IF** 若提供的网格未能通过封闭可定向流形校验，则 Geometry_Field_Evaluator **SHALL** 拒绝该网格并报告检测到的边界或非流形拓扑。
12. **WHERE** 在启用并行构建的情况下，Geometry_Field_Evaluator **SHALL** 支持在独立 Voxel 之间的无竞态评估，并产生满足与串行评估相同认证要求的分类。
13. **WHEN** 当串行和并行基准构建使用相同的模型、变换、公差和评估器配置时，IPG_C_Builder **SHALL** 报告吞吐量和扩展性，而不要求固定的加速比。

### 需求 3：自适应表面采样

**用户故事**：作为 CAM 算法开发人员，我希望在需要时按特征感知的精度对保守边界候选单元进行采样，以便下游计算获得有用的表面点，而无需假定每个候选单元都包含表面。

#### 验收标准

1. **WHEN** 当 BOUNDARY_CANDIDATE 表面查询开始时，IPG_C **SHALL** 将 CE_Effective_Precision 计算为几何相交 Feature_Entities 中的最小正精度，或在无 Feature_Entity 关联时计算为 $t$，并应强制满足 $0 < \text{CE\_Effective\_Precision} \le \text{配置的 SUPPORT 精度}$。
2. **WHEN** 当为边长为 $s$ 且有效精度为 $p$ 的 CE 调用自适应采样时，IPG_C **SHALL** 使用 $\text{chordalTol} = p$ 和 $\text{maxDepth} = \max\left(0, \lceil \log_2(s / p) \rceil\right)$ 配置细化。
3. **WHILE** 当表面采样查询正在主动评估一个仅支撑（support-only）的 CE 时，IPG_C **SHALL** 仅将继承的 SUPPORT 精度用于自适应采样。
4. IPG_C **SHALL** 针对 FEATURE、SUPPORT 及混合角色 CE 保留相同的 Geometry_Field 精度要求和保守 CE 分类阈值。
5. **WHEN** 当在位置 $x$ 返回具有有效精度 $p$ 的 SurfaceSample 时，IPG_C **SHALL** 提供满足 $|\hat{\varphi}(x)| + \varepsilon(x) \le p / 10$ 的 Certified_Result（从而证明 $|\varphi(x)| \le p / 10$），以及长度误差不大于 $1\times 10^{-6}$ 的外法线。
6. **IF** 若 Geometry_Field 在返回的采样位置不可微或具有不可用的梯度，则 IPG_C **SHALL** 从有公差界限的关联 BREP 面推导外法线，或报告没有可靠的采样法线可用。
7. **IF** 若在 $\text{maxDepth}$ 下没有候选采样点满足精度 $p$ 下的认证残差要求和可靠法线要求，则 IPG_C **SHALL** 返回 0 个 SurfaceSample 作为明确状态为“在精度 $p$ 下未验证”（not verified at precision p）的成功结果，保留 CE 状态，并报告真实表面的存在性仍未确定。
8. **WHERE** 在启用预计算的情况下，IPG_C_Builder **SHALL** 使用与按需采样相同的查询语义和有效精度规则缓存 SurfaceSample。
9. **WHILE** 在 IPG_C 无预计算采样缓存运行期间，IPG_C **SHALL** 为每次表面采样查询保持对兼容 Geometry_Field_Evaluator 的访问。
10. **WHEN** 当在不可变发布之前，BOUNDARY_CANDIDATE 获得了更紧凑的认证区间时，IPG_C_Builder **SHALL** 仅允许根据需求 1 的不等式重新分类为已认证的 AIR 或 SOLID。
11. **IF** 若在不可变发布之后获得了更紧凑的认证区间，则 IPG_C **SHALL** 将任何重新分类置于单独版本化的衍生结果中，并保持已发布的 IPG_C 不变。

### 需求 4：完整特征锚定

**用户故事**：作为 CAM 算法开发人员，我希望每个相关的边界单元都能完整链接到真正与其相交的 BREP 实体，以便解析几何保持可追溯且不发生隐式关联丢失。

#### 验收标准

1. **WHEN** 当读取 BREP 面时，Feature_Index **SHALL** 记录该面拥有的 FEATURE 或 SUPPORT 角色以及该面的正 Feature_Precision。
2. **WHEN** 当读取边或顶点时，Feature_Index **SHALL** 将 Entity_Effective_Precision 推导为关联面的最小精度，并继承所有关联的 Face_Role。
3. **IF** 若面的 Feature_Precision 非正，或其 SUPPORT 精度粗于配置的 SUPPORT 精度上限，则 IPG_C_Builder **SHALL** 拒绝该精度配置并标识该面。
4. IPG_C_Builder **SHALL** 要求每一个确认的 Feature_Entity 与 CE 的交集在完整的 Feature_Index 中映射到一个 BOUNDARY_CANDIDATE。
5. **IF** 若最终的精确或有公差界限的实体与 CE 交集测试确认了与被认证为 AIR 或 SOLID 的 CE 相交，则 IPG_C_Builder **SHALL** 将该结果视为证书/几何不一致，在不替换认证状态的情况下使受影响衍生结果的构建或发布失败，并报告该 CE、Feature_Entity 标识符、Field_Estimate、Certified_Error 和施加的交集公差。
6. **WHERE** 在使用扩展实体 AABB 进行粗筛选择的情况下，IPG_C_Builder **SHALL** 使用精确或有公差界限的实体与 CE 交集测试来确认每个最终的 Feature_Index 关联。
7. Feature_Index **SHALL** 仅包含通过最终实体与 CE 交集测试确认的关联。
8. Feature_Index **SHALL** 保持双向不变性：即每个 CE 到 Feature 的关联均有唯一的 Feature 到 CE 关联相对应，反之亦然。
9. **WHEN** 当有效关联的数量超过内联或配置的内存容量时，Feature_Index **SHALL** 使用完整的变长或溢出存储，而不截断或丢弃关联。
10. **IF** 若完整的 Feature_Index 存储超过了配置的构建资源限制，则 IPG_C_Builder **SHALL** 警告调用方并要求显式继续，或在不发布部分 Feature_Index 的情况下使构建失败。
11. **IF** 若 BREP 实体退化，则 Feature_Index **SHALL** 与需求 1 保持一致省略该实体，并保留相关诊断信息。
12. **WHEN** 当输入为无 BREP 拓扑的网格时，Feature_Index **SHALL** 保持为空，并无错返回空特征结果集。
13. **WHEN** 当 Feature_Index 构建完成时，Feature_Index **SHALL** 随已发布的 IPG_C 保持不可变。

### 需求 5：空间与特征查询

**用户故事**：作为刀轨规划人员，我希望获得可预测的状态、区域、角色和特征查询，以便加工计算能够将真实的加工目标几何体与人造的拓扑封闭几何体区分开来。

#### 验收标准

1. **WHEN** 当查询 Construction_Domain 中的点时，IPG_C_Query_Service **SHALL** 通过有界深度的网格查找和 CE 索引计算，返回包含该点的 CE 状态、Navigation_Value 可用性、认证状态和关联的 Feature_Entities。
2. **WHEN** 当查询 AABB 区域时，IPG_C_Query_Service **SHALL** 使用稀疏树遍历返回与该区域相交的已实例化非 AIR CE，其时间开销与遍历开销加上返回的数据量成正比。
3. **WHEN** 当查询 Feature_Entity 标识符时，IPG_C_Query_Service **SHALL** 通过 Feature_Index 反向映射返回所有关联的 BOUNDARY_CANDIDATE CE。
4. **WHERE** 在指定了 FEATURE、SUPPORT、FEATURE-eligible 或 support-only 过滤器的情况下，IPG_C_Query_Service **SHALL** 仅返回符合所请求继承角色语义的关联和 SurfaceSample。
5. **IF** 若点或 AABB 查询包含非有限坐标，或者 AABB 的任何最小坐标大于对应的最大坐标，则 IPG_C_Query_Service **SHALL** 将该查询作为无效查询拒绝。
6. **IF** 若有效点位于 Construction_Domain 之外，则 IPG_C_Query_Service **SHALL** 返回域外结果，该结果应与 AIR 和空结果明确区分。
7. **IF** 若有效的点、AABB 或 Feature_Entity 查询没有实际匹配的数据，则 IPG_C_Query_Service **SHALL** 无错返回适用的空结果集。
8. **IF** 若存在匹配数据但处于 UNRESOLVED 状态，则 IPG_C_Query_Service **SHALL** 返回不透明标识符和未解决状态，而非空结果。

### 需求 6：IPW 互操作性与目标感知比对

**用户故事**：作为仿真工程师，我希望在具有显式 FEATURE 资格的共享坐标系中对比 IPG_C 和 IPW，以便人造封闭几何体不会产生误报的残余材料或过切报告。

#### 验收标准

1. **WHEN** 当准备 IPG_C 和 IPW 进行直接对应时，IPG_Comparison_Service **SHALL** 验证完全相同的世界到索引变换、voxelSize、原点、轴向和 CE 索引。
2. **IF** 若任何所需的变换或分辨率组件不同，则 IPG_Comparison_Service **SHALL** 拒绝直接比对并报告每次不匹配，而不进行隐式重采样。
3. IPG_Comparison_Service **SHALL** 将默认的加工余量和过切评估限制在 Comparison_Eligible_CE 和 FEATURE SurfaceSample 内。
4. IPG_Comparison_Service **SHALL** 在默认加工目标违规报告中排除仅支撑（support-only）表面和 Synthetic_Closure_Volume。
5. **WHEN** 当对应的比对合格 IPG_C 与 IPW CE 均为 BOUNDARY_CANDIDATE 时，IPG_Comparison_Service **SHALL** 根据兼容的解码 Navigation_Value 自动计算 $\Delta\varphi = \varphi_{\text{IPW}} - \varphi_{\text{IPG}}$，并将 $\Delta\varphi$ 标注为原始场诊断信息，而不单凭 $\Delta\varphi$ 的符号赋予残余材料或过切语义。
6. **WHEN** 当 IPW 表面点 $\mathbf{q}$ 在配置的对应公差内与具有外法线 $\mathbf{n}_{\text{out}}$ 的 FEATURE 目标采样点 $\mathbf{p}$ 匹配时，IPG_Comparison_Service **SHALL** 计算面向表面的偏差 $d_n = (\mathbf{q} - \mathbf{p}) \cdot \mathbf{n}_{\text{out}}$，并将高于正公差的 $d_n$ 分类为残余材料（stock），低于负公差的 $d_n$ 分类为过切（gouging），公差范围内的值分类为匹配。
7. **IF** 若无法获得可靠的 FEATURE 采样、外法线或 IPW 对应关系，则 IPG_Comparison_Service **SHALL** 返回带有诊断信息的未确定比对结果，而非单凭 CE 状态推导残余材料或过切。
8. **WHERE** 在 IPG_C 输入为 Genuine_Closed_Design_Solid 的情况下，IPG_Comparison_Service **SHALL** 允许除面向 FEATURE 的比对之外，进行直接的全 CE 状态比对。
9. **IF** 若 IPG_C 输入包含任何人造 SUPPORT 面，则 IPG_Comparison_Service **SHALL** 在默认情况下禁用直接的全 CE 加工违规分类，并要求显式的非加工诊断模式来暴露原始全 CE 差异。
10. **WHEN** 当允许直接的全 CE 诊断比对时，IPG_Comparison_Service **SHALL** 通过状态位比对 AIR 和 SOLID，并经由解码后的 Navigation_Value 比对 BOUNDARY_CANDIDATE 值，每个对应 CE 的工作量为常数。

### 需求 7：便携序列化与安全恢复

**用户故事**：作为 CAM 系统集成人员，我希望 IPG_C 以具有安全版本控制和可恢复标识符的方式便携持久化，以便会话可以在 macOS ARM64 和 Windows x86_64 之间恢复而不会导致不安全的几何使用。

#### 验收标准

1. IPG_C_Serializer **SHALL** 写入包含网格掩码、量化 Navigation_Value、分类与 DEGRADED 元数据、构建参数、变换、Face_Role、Feature_Precision、Feature_Index 映射、可选 SurfaceSample 以及不透明 Feature_Entity 标识符的便携分块文件。
2. **WHEN** 当写入 IPG_C 文件时，IPG_C_Serializer **SHALL** 使用固定宽度类型和小端字节序编码数值字段。
3. **WHEN** 当对受支持的文件进行序列化和反序列化时，IPG_C_Serializer **SHALL** 按位保留网格掩码和量化值，精确保留 Feature_Index 关联和角色元数据，并将缓存的采样位置保留在 $1\times 10^{-10}\text{ mm}$ 以内，法线角度偏差保留在 $1\times 10^{-6}\text{ rad}$ 以内。
4. IPG_C_Serializer **SHALL** 包含文件格式版本以及文件所需的最小读取器版本。
5. **IF** 若在正常加载期间文件版本或所需功能超过了读取器的支持范围，则 IPG_C_Serializer **SHALL** 拒绝该文件而不进入 RECOVERY，并报告所需的最小读取器版本。
6. **WHEN** 当在 macOS ARM64 或 Windows x86_64 上加载受支持的文件时，IPG_C_Serializer **SHALL** 产生同等的状态、映射、角色、参数和认证元数据，无需调用方进行特定于平台的转换。
7. **WHERE** 在针对不受支持版本的文件显式请求 RECOVERY 模式的情况下，IPG_C_Serializer **SHALL** 仅校验并恢复识别到的数据块，忽略未知数据块，且仅在只读 UNRESOLVED 状态下发布恢复的数据。
8. **WHILE** 当恢复的数据保持 UNRESOLVED 和未验证状态时，IPG_C **SHALL** 允许进行状态和网格检查，并拒绝几何计算、表面采样及加工比对。
9. **IF** 若针对原本受支持的文件源 BREP 几何体不可用，则 IPG_C_Serializer **SHALL** 在 UNRESOLVED 状态下加载包含不透明标识符的网格和 Feature_Index 数据，同时保留状态、网格、角色和标识符查询。
10. **IF** 若解析几何特征操作需要未解决的 BREP 几何体，则 IPG_C **SHALL** 使操作失败，并标识该操作所需的每个未解决 Feature_Entity 标识符。
11. **WHEN** 当恢复或未解决的数据针对兼容的源几何体完成验证，或迁移到受支持的格式时，IPG_C_Serializer **SHALL** 要求在启用几何计算之前成功通过完整性和关联性校验。

---

## 附录 A：保守型 CE 分类证明

### A.1 定义与假设

设 $C$ 为一个封闭的立方体 CE，中心为 $c$，边长为 $s$，外接球半径为：
$$\rho = \frac{\sqrt{3} \cdot s}{2}$$

对于每个 $x \in C$，中心到点的距离满足：
$$\|x - c\| \le \rho$$

设 $\varphi$ 为遵循内部为正约定的精确有符号距离场。设评估器在 $c$ 处返回一个 Certified_Result，其中包含有限值 $\hat{\varphi}(c)$ 和有限值 $\varepsilon(c) \ge 0$，满足：
$$|\hat{\varphi}(c) - \varphi(c)| \le \varepsilon(c)$$

等价地：
$$\hat{\varphi}(c) - \varepsilon(c) \le \varphi(c) \le \hat{\varphi}(c) + \varepsilon(c)$$

分类规则如下：
- 当且仅当 $\hat{\varphi}(c) - \varepsilon(c) > \rho$ 时，为 SOLID。
- 当且仅当 $\hat{\varphi}(c) + \varepsilon(c) < -\rho$ 时，为 AIR。
- 否则为 BOUNDARY_CANDIDATE。

AIR 和 SOLID 证书要求严格不等式。相等情况保持为 BOUNDARY_CANDIDATE。

### A.2 1-Lipschitz 属性

要求的精确 Geometry_Field 满足对于所有点 $x$ 和 $y$：
$$|\varphi(x) - \varphi(y)| \le \|x - y\|$$

对于位于边界同侧的点，这遵循标准的“点到闭集距离”不等式。对于位于异侧的点，连接这两点的线段必在某点 $z$ 处穿过封闭边界；因此 $|\varphi(x)| \le \|x - z\|$ 且 $|\varphi(y)| \le \|y - z\|$，导出：
$$|\varphi(x) - \varphi(y)| = |\varphi(x)| + |\varphi(y)| \le \|x - z\| + \|z - y\| = \|x - y\|$$

下文的 CE 证明依赖此性质。本身不满足 1-Lipschitz 的近似场，仅当其认证区间在每个分类中心处仍能界定精确 1-Lipschitz Geometry_Field 时才是可接受的。

### A.3 SOLID 认证证明

假设：
$$\hat{\varphi}(c) - \varepsilon(c) > \rho$$

对于任意 $x \in C$，由误差证书和 1-Lipschitz 属性可得：
$$\varphi(x) \ge \varphi(c) - \|x - c\|$$
$$\varphi(x) \ge \hat{\varphi}(c) - \varepsilon(c) - \|x - c\|$$
$$\varphi(x) \ge \hat{\varphi}(c) - \varepsilon(c) - \rho > 0$$

因此，$C$ 中的每个点均位于内部/材料区，故 SOLID 分类不存在假阳性（false-positive）的整单元错误分类。

### A.4 AIR 认证证明

假设：
$$\hat{\varphi}(c) + \varepsilon(c) < -\rho$$

对于任意 $x \in C$，由误差证书和 1-Lipschitz 属性可得：
$$\varphi(x) \le \varphi(c) + \|x - c\|$$
$$\varphi(x) \le \hat{\varphi}(c) + \varepsilon(c) + \|x - c\|$$
$$\varphi(x) \le \hat{\varphi}(c) + \varepsilon(c) + \rho < 0$$

因此，$C$ 中的每个点均位于外部/空气区，故 AIR 分类不存在假阳性（false-positive）的整单元错误分类。

### A.5 边界交集的完备性证明

假设 $C$ 与几何边界相交。则存在 $z \in C$ 使得 $\varphi(z) = 0$。

如果 $C$ 满足 SOLID 证书，则 A.3 节将导出 $\varphi(z) > 0$，这与 $\varphi(z) = 0$ 矛盾。如果 $C$ 满足 AIR 证书，则 A.4 节将导出 $\varphi(z) < 0$，这也与 $\varphi(z) = 0$ 矛盾。

因此，与 $\varphi = 0$ 相交的每个 CE 必须被分类为 BOUNDARY_CANDIDATE。因为每个有效的面、边和顶点均位于 BREP 边界上，所以与有效 Feature_Entity 在几何上相交的每个 CE 也必须是 BOUNDARY_CANDIDATE。

### A.6 BOUNDARY_CANDIDATE 的保守含义

BOUNDARY_CANDIDATE 是两个充分条件证书的逻辑补集。未能证明在整个 CE 上 $\varphi > 0$ 以及未能证明在整个 CE 上 $\varphi < 0$，并不能证明在 CE 中必然发生 $\varphi = 0$。

因此，BOUNDARY_CANDIDATE 可能具有以下任何几何情形：
1. 零表面与 CE 相交。
2. CE 完全位于内部，但可用区间过宽而无法认证为 SOLID。
3. CE 完全位于外部，但可用区间过宽而无法认证为 AIR。
4. 评估为未认证或非有限值。

因此，表面查询可以合法地返回 0 个 SurfaceSample。存储的 Navigation_Value 无法强化分类，因为量化和截断并不提供认证界限。

### A.7 精细化单调性

将中心证书表示为区间：
$$I = [\hat{\varphi}(c) - \varepsilon(c), \hat{\varphi}(c) + \varepsilon(c)]$$

当精细化区间
$$I' = [\hat{\varphi}'(c) - \varepsilon'(c), \hat{\varphi}'(c) + \varepsilon'(c)]$$
对于相同的精确 $\varphi(c)$ 有效且满足 $I' \subseteq I$ 时，精细化评估是证书单调的。对于未改变的估计值，$\varepsilon'(c) \le \varepsilon(c)$ 足以确立区间包含关系。

如果 BOUNDARY_CANDIDATE 获得了更窄的有效区间，则精细化后的下端点可能变得大于 $\rho$（从而认证为 SOLID），或者精细化后的上端点可能变得小于 $-\rho$（从而认证为 AIR）。

如果原始区间已认证为 SOLID，则 $\inf(I) > \rho$。任何包含于其中的精细化区间均满足 $\inf(I') \ge \inf(I) > \rho$，因此 SOLID 证书保持有效。如果原始区间已认证为 AIR，则 $\sup(I) < -\rho$。任何包含于其中的精细化区间均满足 $\sup(I') \le \sup(I) < -\rho$，因此 AIR 证书保持有效。

具有较小数值 $\varepsilon'$ 但区间不一致或未包含的新计算评估器结果不能确立分类单调性。仅当原始认证界限保持有效时，原始 AIR 或 SOLID 几何结论才保持有效；任意不一致的评估器输出必须予以诊断，而不能用于撤销或反驳有效证书。

### A.8 符号与法线推论

因为 Geometry_Field 为内部为正，所以从边界向外移动会减小 $\varphi$。在梯度非零的可微边界点处，$\nabla\varphi$ 指向场值增加的方向，因此指向内部。因此，指向外部的单位法线为：
$$\mathbf{n}_{\text{out}} = -\text{normalize}(\nabla\varphi)$$

对于对齐的 IPG_C 和 IPW 场，$\Delta\varphi = \varphi_{\text{IPW}} - \varphi_{\text{IPG}}$ 是两个内部为正的标量估计值之差。在 CE 中心处 $\Delta\varphi$ 的符号本身并不能证明表面偏移方向或加工违规。因此，残余材料和过切分类必须按照需求 6 的规定，使用 FEATURE 表面点、经验证的外法线以及 IPW 表面对应关系。
"""

filename = "IPG_C_Requirements_Document_20260826_v1.0.md"
with open(filename, "w", encoding="utf-8") as f:
    f.write(md_content)

print(f"File created successfully: {filename}")

