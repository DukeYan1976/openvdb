# 工业级多轴切削仿真算法 V4.0 详细设计文档

## 目录

1. [问题定义与约束分析](#1-问题定义与约束分析)
2. [方案对比与决策依据](#2-方案对比与决策依据)
3. [算法架构设计](#3-算法架构设计)
4. [五阶段详细设计](#4-五阶段详细设计)
5. [数学证明与复杂度分析](#5-数学证明与复杂度分析)
6. [关键缺陷修复建议](#6-关键缺陷修复建议)
7. [参考文献](#7-参考文献)

---

## 1. 问题定义与约束分析

### 1.1 问题定义

**工业场景**：三轴数控铣削加工仿真
本项目目标是三轴，将来扩展为五轴。 目前所有的设计，都是针对三轴的特化或者优化，但应该考虑在五轴情况下的可扩展性。 

**输入**：
- **工件初始几何** $IPW_0$：由几何参数（如长方体、圆柱体）或三角网格（STL/OBJ）定义的毛坯
- **刀具**：标准铣刀类型
  - 球头刀（Ball End Mill）：刀尖为半球面，适用于曲面精加工
  - 平底刀（Flat End Mill）：刀底为平面，侧面垂直，适用于型腔粗加工
  - 牛鼻刀（Bull Nose Mill）：刀底带圆角过渡，兼顾强度与表面质量
- **刀路**：单段线性刀路，包含
  - 起点 $P_{start}$、终点 $P_{end}$
  - 刀轴方向 $\boldsymbol{a}$（三轴固定，通常沿 Z 轴）
  - 进给速度 $F$（mm/min）
- **用户容差** $\epsilon > 0$：表面精度要求（如 0.5 mm 粗加工；0.05 mm半精加工； 0.01 mm精加工； 0.001mm，极端小工件高精度加工）

**输出**：
- **更新后的工件几何** $IPW_n$：布尔差集 $IPW_{n-1} - T$，其中 $T$ 为刀具扫掠体
- **表面质量**：满足 Hausdorff 距离 $\|IPW_{n} - (IPW_{n-1} - T)\|_H \leq \epsilon$

- **材料去除体积**：可选输出，用于加工时间估算

**核心操作**：
给定刀具参数和线性刀路，计算刀具扫掠体 $T$，执行布尔差集 $W_{new} = W_{old} \setminus T$。

**刀具扫掠体定义**（三轴固定刀轴）：
$$T = \bigcup_{\lambda \in [0,1]} \left( C + \lambda(P_{end} - P_{start}) \right)$$
其中 $C$ 为刀具截面（圆/矩形/圆角矩形），沿刀轴方向拉伸。

### 1.2 约束分析

| 约束类型 | 具体约束 | 影响 |
|---------|---------|------|
| **精度约束** | Hausdorff 距离 $\leq \epsilon$ | 决定细分终止条件 |
| **性能约束** | 实时性要求（通常 > 30 FPS） | 限制算法复杂度上限 |
| **内存约束** | GPU 显存有限（通常 8-24 GB） | 需要碎片规整机制 |
| **并发约束** | 无锁写入要求 | 相对坐标注射方案 |
| **数值鲁棒性** | 免疫参数域奇异点 | 正向预分块而非逆向求导 |

### 1.3 关键观察

1. **时间相干性**：相邻时间步的刀具位置变化微小，旧 IPW（In-Process Workpiece）可作为下一帧的初始状态
2. **空间局部性**：切削仅发生在刀具与工件接触区域，大部分体素不受影响
3. **非对称性**：新点（来自刀具表面）与旧点（来自工件表面）具有不同的几何特性

---

## 2. 方案对比与决策依据

### 2.1 解决思路

#### 2.1.1 基础数据

1. 毛坯 $IPW_n$ : 基于openvdb的dualtrack架构。 

    摒弃传统单一数据结构的局限，系统采用“宏观拓扑导航 + 微观特征重构”解耦的存储方案：

    宏观隐式场 (MacroGrid：FloatGrid)：使用较大体素尺寸构建宽分支 B+ 树，维护工件（IPW）的空间索引与全局拓扑连通性，支持对数级别的时间访问，避免极度深层的树遍历。

    微观离散场 (MicroGrid：PointDataGrid)：在表面叶节点中，利用体素内 相对浮点偏移量 (Fractional Offset) 存储点云坐标。其数值被严格约束在 $-0.5, 0.5$ 的极小区间内，彻底消除了全局大尺寸（如 500mm）工件下的浮点截断误差，提供远超 $0.001$ mm 的绝对物理精度。

2. 刀具扫掠体 $T$, 解析的SDF表达式。  
   刀具扫掠体分为两部分，切削部分和非切削部分， 非切削部分只做碰撞检测，切削部分需要负责切削界面的点法数据的生成和采样后的精度保证。
   
   在切削仿真阶段，可以只处理切削部分。

#### 2.1.2  计算管线
前提：上游程序已经构建好的 $IPW0$ ，并根据完整的刀路和仿真设置，完成了仿真计算的路径分段的规划，并启动了一个针对小段的线性/路径的仿真循环。

设 $IPW_{n-1}$ = $IPW0$

1. 阶段0，构建 $T$ 的解析函数和切削曲面的$S(u,t)$ 表达。
2. 阶段1，完成 $IPW_{n-1}$-Macrogrid与  $T$  的csgdifference计算（要求非常快速）。目的是构建切削边界（旧的保留，新的生成，相交部分涵盖）。
如何验证：新生成$IPW_{n}$-Macrogrid的leafnode中存储的窄带Voxel集合中应该包含所有的切削边界部分。由于不需要从Floatgrid中计算梯度场，此处的narrowband halfwidth是否可以设置为1以节约内存？（开发中评估）
        在本阶段需要收集在当前操作中，影响到的leafnode和voxel。考虑到刀具扫掠体的体积和leafnode的BBbox的大小与用户设置和voxelsize有关，我们不能确定其相对大小关系。 所以leafnode有可能被去除/部分去除/产生新的边界。但leafnode的状态变化，是由其下的voxel阵列状态表达的，所以这里需要记录所有被影响到的voxels_dirty。
        被影响到的voxel有三种：
        1）voxel_d: voxel被完全切除；
        3）voxel_c：voxel被部分切除； 意味着这个voxel中需要做精细的切削计算，判断切削界面被去除的部分以及新生成的界面，确保新的切削界面保持水密，并保留特征。 这部分会是最有挑战的部分。 
        2）voxel_n: voxel是新生成的，意味着这个voxel中可能（大概率，除非因为macrogrid的误差原因被误判）会首次产生新的切削界面。

3. 阶段2，此时$IPW_{n-1}$-Microgrid中仍然是未切削状态，由pointdatagrid中所有的点法式点集表示。本阶段的目的是参考$IPW_{n-1}$-Macrogrid中的变化，对$IPW_{n-1}$-Microgrid中的相关的voxels进行高精度切削计算，最后产生新的切削界面，并与$IPW_{n-1}$-Macrogrid的internalnode结构和voxel的存活状态保持一致。
    3.1 对于voxel_d， 删除Microgrid中对应的voxel中的pointdata数据，并inactive voxel。
    3.2 对与voxel_d和voxel_n，启动切削计算MicroCut过程（并行计算）。
    
    1）构建刀具扫掠界面，形成$S(u,t)$。  对voxel对应的 $S(u,t)$进行参数域分块，voxel->BB：{u_min,t_min; u_max,t_max}, 确保$S(u,t)$中，被包含在voxel内部的扫掠面的部分都包含在BB参数域内。
    
    2）在$S(u,t)_BB$的参数域进行自适应细分，获取切削后刀具扫掠面与原始IPW的微观曲面进行布尔减运算后的相交边界；以及边界内基于刀具扫掠面上该点的曲率的采样点，同时产生法线。 这部需要是确保$\epsilon$  的关键步骤。

    3）voxel内部所有在刀具扫掠体内的点被去除（设置掩码）； 误差范围内的边界点（法线一致）去重，法线不一致则意味着有局部特征，保留。
        
4. 完成后$IPW_n$进行一次计算和存储的内存重整，产生新的$IPW_n$，进入下一次切削计算步。


#### 2.1.2  渲染管线
按照渲染频率的要求，生成屏幕刷新的数据： 在高resolution情况下，显示pointgridgrid中受影响的高精度曲面，在低resolution情况下，显示macrogrid中的所含voxel的mesh界面。


### 2.2 本方案（V4.0）的核心优势

**优势 1：自适应精度**
- 仅在需要区域细分，整体复杂度 $O(N_{surface} \cdot \log(L/\epsilon))$

**优势 2：正向预分块免疫奇异点**
- 只进行正向映射 $S(u,v) \to \mathbb{R}^3$，不涉及求逆
- 裕量外扩保证保守性：$S_{conservative} \supseteq S_{exact}$

**优势 3：非对称处理提升效率**
- 旧点：只需判断是否在刀具内，可批量处理
- 新点：需要精确求值和重投影（相交线计算精确完备，则不需要重投影）。

**优势 4：Host-Device 解耦**
- Phase 0-1 在 CPU 完成粗筛和任务打包（$O(1)$ 或 $O(\log N)$）
- Phase 2-4 在 TBB/GPU 并行执行计算密集型操作

### 2.4 决策依据

```
决策树：
是否需要工业级精度（< 0.01mm）？
├── 否 → 方案 A（均匀网格）
└── 是 → 是否存在参数域奇异点？
    ├── 否 → 传统自适应细分
    └── 是 → 本方案 V4.0（正向预分块 + 单趟四叉树）
```

---

## 3. 算法架构设计

### 3.1 全局架构：Host-Device 解耦模式

```
┌─────────────────────────────────────────────────────────────┐
│                         Host (CPU)                          │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────────┐ │
│  │  Phase 0    │ -> │  Phase 1    │ -> │  Task Queue     │ │
│  │  粗筛去活    │    │  极简预分块  │    │  (O(1)下发)     │ │
│  └─────────────┘    └─────────────┘    └─────────────────┘ │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                    Device (TBB/GPU)                         │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────────┐ │
│  │  Phase 2    │ -> │  Phase 3    │ -> │  Phase 4        │ │
│  │ 单趟四叉树   │    │ 非对称剔除   │    │ 无锁相对坐标注射 │ │
│  │ 核内裁边    │    │ 边界重投影（若必要）   │    │                 │ │
│  └─────────────┘    └─────────────┘    └─────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

### 3.2 核心数据结构

#### 3.2.1 点云存储（OpenVDB PointDataGrid）

```
PointDataGrid
├── 体素坐标 (i, j, k)  →  LeafNode
│   └── AttributeSet
│       ├── position:    Vec3f  (相对坐标 [-0.5, 0.5]^3)
│       ├── normal:      Vec3f  (可选)
│       └── flag:        uint8  (alive/dead/marked)
└── 内部节点（用于空间索引）
```

**关键设计**：使用相对坐标而非绝对坐标
- 绝对坐标：$p_{abs} \in \mathbb{R}^3$，需要 96 bit
- 相对坐标：$p_{rel} \in [-0.5, 0.5]^3$，可用 16-32 bit 量化
- 无锁写入：不同线程可向同一体素追加点，无需锁

#### 3.2.2 任务负载（VoxelTaskPayload）

```cpp
struct VoxelTaskPayload {
    LeafNode*           voxel_ptr;      // 目标体素指针
    BoundingBox         voxel_aabb;     // 体素 3D AABB
    Range               u_conservative; // 保守 U 域（含裕量）
    Range               t_conservative; // 保守 T 域（含裕量）
};
```

### 3.3 核心术语定义

| 术语 | 定义 |
|------|------|
| **极简预分块** | 将参数面划分为 2D 块，计算 3D AABB，外扩裕量后匹配体素 |
| **单趟自适应四叉树** | 参数域自适应细分，一次遍历完成求值，不回溯 |
| **核内裁边** | 3D AABB 拒止判断在求值核（kernel）内部完成，不返回宿主 |
| **非对称布尔剔除** | 新旧点云采用不同策略：旧点硬删除/保留，新点重投影 |
| **无锁相对坐标无损注射** | 使用相对坐标无锁追加到体素，无精度损失 |
| **Batched Compaction** | 使用 GPU 并行前缀和定期整理内存碎片 |

---

## 4. 五阶段详细设计

### 4.1 Phase 0：宏观碰撞粗筛与内存大块去活（Host）

**目标**：快速剔除完全不受影响的体素，释放被完全切除的内存区域。

**输入**：
- 旧工件（$IPW_n$, 代表第n次切削计算后，剩下的IPW）的DualTrack数据（MacroGrid/Microgrid）点云 $W_{old}$（OpenVDB PointDataGrid）
- 刀具扫掠体包围盒 $B_{tool}$

**输出**：
- 活跃体素列表 $L_{active}$
- 去活体素列表 $L_{inactive}$（内存可回收）

**算法步骤**：

1. 计算刀具扫掠体的全局 AABB：$B_{tool} = \bigcup_{t} AABB(S(\cdot, \cdot, t))$
2. 遍历 PointDataGrid 的所有叶节点：
   - 若叶节点 AABB 与 $B_{tool}$ 不相交 → 标记为 **去活**
   - 若叶节点 AABB 完全包含于 $B_{tool}$ → 标记为 **完全切除**（整叶释放）
   - 否则 → 标记为 **活跃**
3. 将完全切除的叶节点内存回收到缓冲池

**复杂度**：$O(N_{leaves})$，其中 $N_{leaves}$ 为叶节点总数

**伪代码**：
```cpp
// 见 pseudocode.cpp: phase0_coarse_culling()
```

---

### 4.2 Phase 1：极简预分块（Host）

**目标**：将参数域与体素空间匹配，生成保守任务包。

**核心思想**：
- 正向映射：参数块 → 3D AABB（避免逆向求导）
- 裕量外扩：保证保守性（不遗漏真实切削区域）
- $O(1)$ 任务下发：每个体素对应一个任务包

**算法步骤**：

1. **参数域分块**：
   - 将参数域 $\Omega$ 均匀划分为 $N_u \times N_t$ 个初始块
   - 块大小选择：使每个块的 3D AABB 大致匹配体素尺寸

2. **正向映射与匹配**：
   ```
   for each param_block in param_blocks:
       block_aabb = Compute3DBBox(S_uv, block.u_range, block.t_range)
       for each voxel in active_voxels:
           if voxel.aabb.intersects(block_aabb):
               voxel.u_local.expand(block.u_range)
               voxel.t_local.expand(block.t_range)
               voxel.has_intersection = true
   ```

3. **裕量外扩**：
   - 对匹配的参数域添加安全裕量：
     $$u_{conservative} = [u_{min} - \Delta_u, u_{max} + \Delta_u]$$
     $$t_{conservative} = [t_{min} - \Delta_t, t_{max} + \Delta_t]$$
   - 裕量选择：$\Delta = C \cdot \epsilon$，其中 $C \geq 1$ 为安全系数

4. **任务打包**：
   - 为每个活跃体素生成 `VoxelTaskPayload`
   - 任务队列直接下发到 TBB/GPU

**复杂度**：
- 参数块数 $N_{blocks} = O(N_{surface} / voxel\_size^2)$
- 体素数 $N_{voxels} = O(N_{leaves})$
- 总复杂度：$O(N_{blocks} \cdot N_{voxels})$，但可通过空间索引优化到 $O(N_{blocks} \log N_{voxels})$

**关键性质（保守性）**：
> **定理 1（保守性）**：裕量外扩后的参数域覆盖包含真实切削区域。
>
> **证明**：设真实接触参数为 $(u^*, t^*)$，对应 3D 点 $p^* = S(u^*, t^*)$。
> 由于分块匹配时，包含 $(u^*, t^*)$ 的块与目标体素相交，该块的参数域被加入体素的局部域。
> 外扩裕量后，$(u^*, t^*) \in [u_{min} - \Delta, u_{max} + \Delta]$。
> 因此真实接触点不会被遗漏。∎

**伪代码**：
```cpp
// 见 pseudocode.cpp: phase1_minimal_preblocking()
```

---

### 4.3 Phase 2：单趟自适应四叉树极速求值与核内裁边（TBB/GPU）

**目标**：在参数域自适应细分，生成满足精度要求的特征点云，并在求值核内完成 3D AABB 拒止。

**核心思想**：
- **单趟**：自顶向下细分，不回溯，一次遍历完成
- **核内裁边**：3D AABB 包含判断在求值核内部完成，避免冗余通信
- **自适应终止**：基于几何误差或参数域大小

**算法步骤**：

1. **初始化**：从 Phase 1 获取任务包 $(u_{conservative}, t_{conservative}, v_{aabb})$

2. **单趟四叉树细分**：
   ```
   function QuadtreeEvaluate(u_range, t_range, depth):
       // 1. 计算中心点
       u_c = (u_min + u_max) / 2
       t_c = (t_min + t_max) / 2
       p_center = S(u_c, t_c)
       
       // 2. 核内裁边：3D AABB 拒止
       if NOT v_aabb.contains(p_center):
           return  // 在核内直接丢弃，不返回宿主
       
       // 3. 终止条件检查
       if ShouldTerminate(u_range, t_range, p_center, epsilon):
           // 生成特征点
           buffer.append(p_center, normal, attributes)
           return
       
       // 4. 单趟细分（四等分）
       du = (u_max - u_min) / 2
       dt = (t_max - t_min) / 2
       
       QuadtreeEvaluate([u_min, u_min+du], [t_min, t_min+dt], depth+1)
       QuadtreeEvaluate([u_min+du, u_max], [t_min, t_min+dt], depth+1)
       QuadtreeEvaluate([u_min, u_min+du], [t_min+dt, t_max], depth+1)
       QuadtreeEvaluate([u_min+du, u_max], [t_min+dt, t_max], depth+1)
   ```

3. **终止条件**（满足任一即终止）：
   - 参数域尺寸：$|u_{max} - u_{min}| < \delta_u$ 且 $|t_{max} - t_{min}| < \delta_t$
   - 几何误差：估计的 Hausdorff 误差 $< \epsilon$
   - 最大深度：$depth > D_{max}$

**关键设计：核内裁边**

传统方法：
```
GPU Kernel → 返回所有细分点 → CPU 筛选在 AABB 内的点
问题：大量冗余点传输，带宽瓶颈
```

本方案（核内裁边）：
```
GPU Kernel → 内部判断 AABB 包含 → 仅保留内部点
优势：零冗余传输，带宽节省 10-100x
```

**复杂度分析**：
- 设细分深度为 $d$，则每任务最多 $4^d$ 个叶节点
- 但实际由于核内裁边，大部分细分在 early-out 时终止
- 有效复杂度：$O(N_{surface} \cdot \log(L/\epsilon))$

**伪代码**：
```cpp
// 见 pseudocode.cpp: phase2_quadtree_evaluation()
```

---

### 4.4 Phase 3：非对称布尔剔除与边界重投影（TBB/GPU）

**目标**：处理旧点云，区分新旧点的不同策略，消除离散锯齿。

**核心思想**：
- **非对称性**：旧点（工件表面）与新点（刀具表面）几何特性不同
- **旧点策略**：硬删除（在刀具内）或保留（在刀具外）
- **过渡区策略**：SDF 梯度重投影，将点拉回解析等值面

**数学模型**：

**旧点处理**：
```
for each old_point in W_old:
    sdf_value = SDF_tool(old_point.position)
    if sdf_value < 0:
        // 点在刀具内部 → 硬删除
        old_point.flag = DEAD
    else if sdf_value < epsilon:
        // 点在过渡区 → 边界重投影
        old_point.position = Reproject(old_point, sdf_value, gradient)
    else:
        // 点在安全区 → 保留
        pass
```

**边界重投影公式**：

$$\mathbf{p}_{new} = \mathbf{p}_{old} - SDF_{csv}(\mathbf{p}_{old}) \cdot \nabla SDF_{csv}(\mathbf{p}_{old})$$

**几何解释**：
- $SDF_{csv}(\mathbf{p}_{old})$：带符号距离（负值表示在内部）
- $\nabla SDF_{csv}(\mathbf{p}_{old})$：指向外部的单位法向量
- 乘积：将点沿法向移动到等值面（$SDF = 0$）

**收敛性分析**：
> **定理 2（重投影收敛）**：若 $SDF$ 为精确距离场，则一次重投影即可到达零等值面。
>
> **证明**：设 $\mathbf{p}_{old}$ 到等值面的最短距离为 $d = SDF(\mathbf{p}_{old})$，方向为 $\mathbf{n} = \nabla SDF(\mathbf{p}_{old})$。
> 则 $\mathbf{p}_{new} = \mathbf{p}_{old} - d \cdot \mathbf{n}$。
> 由距离场性质，$SDF(\mathbf{p}_{old} - d \cdot \mathbf{n}) = 0$。
> 因此 $\mathbf{p}_{new}$ 精确位于等值面上。∎

**非对称策略对比**：

| 点类型 | 处理策略 | 复杂度 | 精度 |
|--------|---------|--------|------|
| 旧点（工件） | SDF 判剔 + 重投影 | $O(1)$ 每点 | 依赖 SDF 精度 |
| 新点（刀具） | 四叉树精确求值 | $O(\log(L/\epsilon))$ | 自适应至 $\epsilon$ |

**伪代码**：
```cpp
// 见 pseudocode.cpp: phase3_asymmetric_culling()
```

---

### 4.5 Phase 4：无锁相对坐标无损注射（TBB/GPU）

**目标**：将 Phase 2 生成的新点云注入 PointDataGrid，使用无锁相对坐标。

**核心思想**：
- **相对坐标**：存储相对于体素中心的偏移，而非绝对坐标
- **无锁追加**：利用原子操作或并行前缀和，无需锁
- **无损**：不量化精度，保持原始浮点值

**坐标转换**：

设体素中心为 $\mathbf{c}$，体素尺寸为 $s$，绝对坐标为 $\mathbf{p}_{abs}$。

相对坐标：
$$\mathbf{p}_{rel} = \frac{\mathbf{p}_{abs} - \mathbf{c}}{s}$$

逆变换：
$$\mathbf{p}_{abs} = \mathbf{c} + s \cdot \mathbf{p}_{rel}$$

**性质**：
- $\mathbf{p}_{rel} \in [-0.5, 0.5]^3$（点在体素内时）
- 可用 half/float 存储，节省带宽

**无锁注射算法**：

```
// 方法 1：原子追加（简单，可能有碎片）
for each new_point in new_points:
    voxel = grid->getOrCreateLeaf(new_point.coord)
    idx = atomic_increment(voxel->point_count)
    voxel->positions[idx] = new_point.rel_position

// 方法 2：并行前缀和（无碎片，推荐）
// 1. 统计每体素点数
// 2. 前缀和计算全局偏移
// 3. 并行写入
```

**本方案采用 Batched Compaction**（见第 5 节）。

**伪代码**：
```cpp
// 见 pseudocode.cpp: phase4_lockfree_injection()
```

---

## 5. 数学证明与复杂度分析

### 5.1 正确性证明

**定理 3（算法正确性）**：算法输出的 $W_{new}$ 满足 $W_{new} = W_{old} \setminus S$（在 $\epsilon$ 精度内）。

**证明**：

1. **完备性（Completeness）**：
   - 由定理 1（保守性），Phase 1 的裕量外扩保证所有真实接触区域被覆盖。
   - Phase 2 的单趟四叉树在覆盖区域内自适应细分至 $\epsilon$ 精度。
   - 因此所有应被切除的点都被处理。

2. **可靠性（Soundness）**：
   - Phase 3 的旧点处理：SDF 判剔精确（或保守）地识别在刀具内的点。
   - 重投影公式（定理 2）保证过渡区点被精确拉回等值面。
   - Phase 4 的坐标转换是双射，无精度损失。

3. **无冲突**：
   - 相对坐标注射使用原子操作或前缀和，无数据竞争。
   - 新旧点云处理顺序：先剔除旧点，再注入新点，无重叠。

因此 $W_{new} = W_{old} \setminus S$ 在 Hausdorff 距离 $\epsilon$ 内成立。∎

### 5.2 复杂度分析

#### 时间复杂度

| Phase | 操作 | 复杂度 | 说明 |
|-------|------|--------|------|
| Phase 0 | 遍历叶节点 | $O(N_{leaves})$ | 线性扫描 |
| Phase 1 | 预分块匹配 | $O(N_{blocks} \cdot N_{active})$ | 可优化至 $O(N_{blocks} \log N_{active})$ |
| Phase 2 | 四叉树求值 | $O(N_{surface} \cdot \log(L/\epsilon))$ | 自适应细分 |
| Phase 3 | 布尔剔除 | $O(N_{old})$ | 每点 $O(1)$ SDF 求值 |
| Phase 4 | 坐标注射 | $O(N_{new})$ | 并行前缀和 $O(N_{new})$ |

**总时间复杂度**：
$$T_{total} = O(N_{leaves} + N_{blocks} \cdot N_{active} + N_{surface} \cdot \log(L/\epsilon) + N_{old} + N_{new})$$

在典型场景下：
- $N_{active} \ll N_{leaves}$（仅局部切削）
- $N_{surface} \sim N_{new}$
- 主导项：$O(N_{surface} \cdot \log(L/\epsilon))$

#### 空间复杂度

| 数据 | 空间 | 说明 |
|------|------|------|
| PointDataGrid | $O(N_{old})$ | 旧点云 |
| 任务队列 | $O(N_{active})$ | Phase 1 生成 |
| 新点缓冲 | $O(N_{new})$ | Phase 2 生成 |
| SDF 纹理 | $O(M^3)$ | 刀具 SDF（常数） |

**总空间复杂度**：
$$S_{total} = O(N_{old} + N_{new} + N_{active} + M^3)$$

### 5.3 Batched Compaction 复杂度

**问题**：动态删除导致内存碎片，叶节点内点分布稀疏。

**解决方案**：GPU 并行前缀和（Parallel Prefix Sum）

**算法**：
1. 标记阶段：标记所有 DEAD 点
2. 前缀和：计算存活点的紧凑偏移
3. 压缩：并行移动存活点到连续区域

**复杂度**：
- 时间：$O(N_{points})$，高度并行
- 空间：$O(N_{points})$ 临时缓冲
- 频率：每 $K$ 帧执行一次（$K \approx 100-1000$）

**与 RCU 对比**：

| 特性 | RCU | Batched Compaction |
|------|-----|-------------------|
| 读开销 | 无锁读 | 需要版本检查 |
| 写开销 | 延迟释放 | 批量整理 |
| 内存碎片 | 累积 | 定期消除 |
| 实时性 | 好 | 需要暂停 |
| 实现复杂度 | 高 | 低（GPU 前缀和成熟） |

**本方案选择 Batched Compaction 的理由**：
1. GPU 前缀和高度优化（cuDPP、CUB）
2. 定期整理可接受（加工过程有自然停顿）
3. 避免 RCU 的复杂性和内存累积

---

## 6. 关键缺陷修复建议

### 6.1 缺陷 1：裕量外扩导致的过度细分

**问题**：裕量外扩可能引入大量冗余参数域，导致四叉树过度细分。

**修复建议**：
- 动态裕量调整：根据参数面曲率自适应调整 $\Delta$
- 快速拒绝：在四叉树早期增加曲率测试，快速剔除远离真实表面的区域

### 6.2 缺陷 2：单趟四叉树的早期终止误差

**问题**：单趟遍历可能错过最优细分策略，导致局部精度不足。

**修复建议**：
- 增加局部误差估计：使用二次采样估计当前块的 Hausdorff 误差
- 自适应深度：允许局部区域超过全局最大深度

### 6.3 缺陷 3：无锁注射的内存碎片

**问题**：频繁的原子追加导致叶节点内点分布碎片化。

**修复建议**：
- 双缓冲：每个叶节点维护两个缓冲区（当前帧和下一帧）
- 定期 Compaction：严格执行 Batched Compaction 策略

### 6.4 缺陷 4：SDF 梯度计算的数值稳定性

**问题**：在 SDF 的零等值面附近，数值梯度可能不稳定。

**修复建议**：
- 解析梯度：若刀具有解析表达式，优先使用解析梯度
- 中心差分：使用高阶差分格式（如 5 点 stencil）
- 梯度裁剪：限制梯度模长在 $[0.9, 1.1]$ 内

### 6.5 缺陷 5：GPU 内存不足

**问题**：大规模工件可能导致 GPU 显存溢出。

**修复建议**：
- 分块处理：将工件划分为多个子域，逐块处理
- 流式处理：使用 CUDA Unified Memory，自动页迁移
- 精度降级：在远离切削区域使用较低精度

---

## 7. 参考文献

1. Museth, K. (2013). VDB: High-resolution sparse volumes with dynamic topology. ACM TOG.
2. TBB Documentation: https://github.com/oneapi-src/oneTBB
3. CUDA CUB Library: https://nvlabs.github.io/cub/
4. Selle, A., et al. (2008). A Massively Parallel Adaptive Fast Multipole Method on Heterogeneous Architectures.
5. 原始算法文档：design0.a.md

---

**文档版本**：V4.0  
**最后更新**：2026-06-15  
**作者**：Loop Engineering Algorithm Designer
