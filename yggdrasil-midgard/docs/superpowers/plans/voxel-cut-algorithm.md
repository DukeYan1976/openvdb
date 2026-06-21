# 单 Voxel 切削算法设计

> 版本：v0.a | 日期：2026-06-20

## 修订记录

| 版本 | 日期 | 说明 |
|------|------|------|
| 0.a | 06-20 | 问题定义 + 算法头脑风暴 |

---

## 1. 问题定义

### 1.1 应用背景

多轴 CNC 铣削仿真。刀具沿任意方向运动，轴向可变。毛坯被离散为宏观 Voxel 网格，每个 Voxel 独立计算其内部的材料去除结果。

### 1.2 输入

| 符号 | 含义 |
|------|------|
| V | Voxel 边长 (mm)，代表一个实心毛坯小方块 |
| t | 用户加工公差 (mm)，输出表面点集的弦高精度上限 |
| CutHistory[n] | 按时序排列的 n 次切削记录 |

每条切削记录：

```cpp
struct CutRecord {
    // ── 隐式表达（距离场）──
    // eval(P) → float:
    //   > 0: P 在刀具外部（材料保留）
    //   < 0: P 在刀具内部（材料去除）
    //   = 0: 扫掠体表面
    ToolSweptSDF sdf;

    // ── 显式表达（参数面）──
    // S(u, v) → R³: 扫掠体表面的参数化
    //   u ∈ [0,1]: 刀具轮廓方向
    //   v ∈ [0,1]: 扫掠运动方向
    ToolSweepSurface surface;

    // ── 元数据 ──
    uint32_t seqIndex;     // 全局时序编号
    ToolDef tool;          // 刀具几何参数
    MoveSegment segment;   // 运动段（start, end, axis）
};
```

**关键：每条记录同时提供 SDF（隐式）和参数面（显式）。**
- SDF → 快速内外判断
- 参数面 → 精确表面采样

### 1.3 输出

```cpp
struct VoxelSurface {
    std::vector<Vec3f> positions;  // 表面点（世界系）
    std::vector<Vec3f> normals;    // 法向（指向材料外部）
};
```

满足：相邻点间弦高偏差 < t。

### 1.4 材料状态

```
compositeSDF(P) = min(sdf_0(P), sdf_1(P), ..., sdf_{n-1}(P))

compositeSDF > 0 → 材料（实心）
compositeSDF < 0 → 空气（被切掉）
compositeSDF = 0 → 最终材料表面
```

表面归属：P 处表面属于第 k 个扫掠体，k = argmin_i(sdf_i(P))。

脊线：sdf_i(P) = sdf_j(P) = 0（两把刀交汇处，法向不连续）。

### 1.5 约束

| 约束 | 说明 |
|------|------|
| sdf_i.eval(P) = O(1) | 与扫掠长度无关 |
| surface_i.eval(u,v) = O(1) | 直接参数求值 |
| n ≤ 16 | 超过触发压实 |
| V/t 可达 500 | 算法不应 O((V/t)³) |
| Voxel 间无依赖 | 可完全并行 |

### 1.6 核心问题

> 在 V³ 盒子内，给定 n 个 SDF（及对应参数面），高效提取 min(sdf_0,...,sdf_{n-1})=0 的等值面点集，精度 < t。

---

## 2. 算法头脑风暴

### 方案 A：粗网格 + 边零交叉 + 弦高加密

**思路**：经典 Marching Cubes 变体，但用自适应精度。

```
① 在 Voxel 内放 K³ 粗网格（K=4~8），eval compositeSDF
② 找所有跨零 edge，二分法定位零交叉点（精度 t）
③ 检查相邻点弦高，不足时在中间加密采样
```

**eval 次数**：~K³ + 跨零edge × log2(step/t) + 加密点
**优势**：通用，不依赖参数面
**劣势**：K 选择困难；薄壁/微小特征可能被粗网格漏掉

---

### 方案 B：参数面直接采样 + SDF 裁剪

**思路**：利用参数面在扫掠体表面上直接采样，用其他 SDF 做裁剪。

```
对每个 CutRecord_i:
  ① 在 surface_i(u,v) 参数域内四叉树采样
  ② 对每个采样点 P = S_i(u,v):
       - 检查 P ∈ Voxel？（AABB 拒止）
       - 检查 P 是否被后续刀具切掉：∃j≠i, sdf_j(P) < 0？
       - 两个都通过 → P 是最终表面点
  ③ 弦高自适应（四叉树递归条件）
```

**eval 次数**：每个参数面采样点需要 (n-1) 次 SDF eval 做裁剪
**优势**：
- 直接在曲面上采样，精度天然满足
- 利用参数面的几何连续性，四叉树效率高
- 法向直接从参数面求（解析，无需数值梯度）
**劣势**：
- n 个参数面各采样一遍，有冗余
- 需要处理脊线（两个面的交线）

---

### 方案 C：分层——SDF 定位 + 参数面精修

**思路**：先用 SDF 快速找到表面大致位置，再用参数面精确采样。

```
① 粗定位：K³ 网格 eval compositeSDF，找到零交叉 cell
② 归属判断：对每个零交叉 cell，确定 k = argmin(sdf_i)
③ 精修：在 surface_k 上用局部参数域精确采样
   - 从零交叉点反投影到 surface_k 的 (u,v)
   - 在邻域内四叉树细化到弦高 < t
```

**eval 次数**：K³×n（粗定位） + 精修点数×1（参数面 eval）
**优势**：结合了 SDF 的全局视野和参数面的精确性
**劣势**：反投影步骤的实现复杂度

---

### 方案 D：事件驱动——只处理变化

**思路**：如果 Voxel 已有上一次的表面点集（来自前 n-1 次切削），第 n 次切削只需处理变化。

```
已有：旧表面点集 {P_old}
新来：sdf_n

① 旧点验证：对每个 P_old, 检查 sdf_n(P_old)
   - sdf_n > 0 → 未被新刀切到，保留
   - sdf_n < 0 → 被切掉，删除
   - sdf_n ≈ 0 → 在新刀表面上，可能需要精度修正

② 新表面采样：在 surface_n 上采样，只保留：
   - P ∈ Voxel
   - ∀j<n: sdf_j(P) > 0（P 不在旧刀具内部——但 Voxel=毛坯，旧刀已切走了那些区域）
   
   修正：实际条件是 compositeSDF(P) = sdf_n(P)（即新刀是此处最深切削者）
   简化：sdf_n(P) ≤ sdf_j(P), ∀j≠n → P 处表面归属于新刀

③ 脊线处理：旧面与新面的交线上补采样点
```

**eval 次数**：旧点数 × 1（验证） + 新面采样数 × (n-1)（裁剪）
**优势**：
- 增量式，大部分旧点直接保留（O(1) 验证）
- 天然支持 cutHistory 逐步追加
- 与分层架构完美契合
**劣势**：
- 脊线处理复杂
- 累积误差（多次增量后可能需要全局重算=压实）

---

### 方案 E：混合（推荐）

**D 为主线 + B 为新面采样 + A 为压实/冷启动**

```
正常切削（增量，第 n 次）:
  ① 旧点 SDF 验证 → 保留/删除            [方案 D.①]
  ② 新面参数域采样 + 裁剪 → 新增点         [方案 B]
  ③ 脊线检测 + 补点                       [方案 D.③]

冷启动/压实（全局，history 过长时）:
  ④ K³ 粗网格 + 归属判断 + 参数面精修      [方案 C]
```

**性能特征**：
- 常规增量切削：~50~200 eval/voxel（验证旧点 + 新面采样）
- 压实：~500~2000 eval/voxel（全局重算，偶发）

---

## 3. 脊线处理

脊线是两个扫掠体表面的相交线。在此处：
- compositeSDF = sdf_i = sdf_j = 0
- 法向不连续（左右归属不同）

**检测**：对表面点 P，若 `|sdf_i(P) - sdf_j(P)| < ε` 且两者都接近零。

**采样**：沿脊线方向步进。脊线方向 = `grad(sdf_i) × grad(sdf_j)`（两个梯度的叉积）。

**精度要求**：脊线是工件上的尖锐特征（残高脊），精度要求与 t 相同。

---

## 4. 待决策

| 问题 | 选项 |
|------|------|
| 首选增量(D/E)还是全量(A/C)？ | 建议 E（混合） |
| 粗网格 K 值 | 4（快但可能漏）vs 8（稳但贵） |
| 参数面裁剪的具体实现 | closestLambda 确定 v 范围 + 全 u |
| 脊线精度等级 | 与 t 相同，还是放宽为 2t |
| 压实触发阈值 | n > 8 or n > 16 |

---

## 5. 方案 F：空间八叉树自适应细分（渐进精度）

### 5.1 核心思想

在 Voxel 内部构建一棵**空间八叉树**，自适应细分到表面附近。细分由 compositeSDF 的符号变化驱动。精度可以从粗到细**渐进增加**——首次计算给出粗略结果（交互用），后续按需继续细分到目标精度 t。

### 5.2 数据结构

```cpp
struct OctNode {
    float cornerSDF[8];     // 8 角点的 compositeSDF 值
    uint8_t children;       // 位掩码：哪些子节点已细分
    uint8_t state;          // EMPTY(全正) / SOLID(全负) / MIXED(跨零)
    // children 指针或偏移（pool 分配）
};
```

每个 Voxel 拥有一棵独立的八叉树，根节点 = Voxel 本身。

### 5.3 算法

```
function buildOctree(node, bbox, depth):
    // ① 在 8 角点求 compositeSDF
    for i in 0..7:
        node.cornerSDF[i] = compositeSDF(corner_i(bbox))
    
    // ② 分类
    if 全正 → node.state = EMPTY, return
    if 全负 → node.state = SOLID, return
    node.state = MIXED
    
    // ③ 精度检查：当前 cell 边长
    cellSize = bbox.extent / (1 << depth)
    if cellSize ≤ t:
        // 达到目标精度，在此 cell 的跨零 edge 上提取表面点
        extractSurface(node, bbox)
        return
    
    // ④ 递归细分 8 个子节点
    for child in 0..7:
        childBBox = octantBBox(bbox, child)
        buildOctree(node.child[child], childBBox, depth+1)
```

### 5.4 渐进精度机制

**关键特性**：八叉树不需要一次性细分到底。可以分多次执行：

```
Level 1: 细分到 V/2  → 8 节点，粗略轮廓（~1ms）
Level 2: 细分到 V/4  → 最多 64 节点（~5ms）
Level 3: 细分到 V/8  → 实时显示足够（~10ms）
...
Level k: 细分到 V/2^k ≤ t → 最终精度
```

每一级只对 MIXED 节点继续细分，EMPTY/SOLID 永远不再访问。

**应用场景**：
- 交互仿真：只细分到 Level 3（视觉足够）
- 精度导出：继续细分到 Level k
- 局部放大：用户 zoom 到某区域时，只对可见 voxel 继续细分

### 5.5 表面提取

在最小 MIXED cell 的 12 条 edge 上，检查符号变化：

```cpp
for (auto& edge : cell.edges12) {
    float s0 = cornerSDF[edge.v0];
    float s1 = cornerSDF[edge.v1];
    if (sign(s0) == sign(s1)) continue;
    
    // 线性插值零交叉
    float alpha = s0 / (s0 - s1);
    Vec3d pt = lerp(corner[edge.v0], corner[edge.v1], alpha);
    Vec3d normal = numericalGradient(pt, compositeSDF);
    emit(pt, normal);
}
```

若需要更高精度，可在该 edge 上用二分法替代线性插值。

### 5.6 与 CutHistory 的交互

**增量更新**：当新的 CutRecord 加入时，不需要重建整棵树：

```
function updateOctree(node, bbox, newSDF):
    // 只有 MIXED 和 SOLID 节点可能受影响
    if node.state == EMPTY: return  // 已是空气，新刀无法再切
    
    // 重新求 8 角点的 min(oldCorner, newSDF)
    bool changed = false
    for i in 0..7:
        float newVal = newSDF.eval(corner_i)
        if newVal < node.cornerSDF[i]:
            node.cornerSDF[i] = newVal
            changed = true
    
    if !changed: return  // 新刀未影响此 cell
    
    // 重分类
    reclassify(node)
    
    // 对 MIXED 子节点递归
    if node.hasChildren:
        for child in 0..7:
            if node.child[child].state != EMPTY:
                updateOctree(node.child[child], childBBox, newSDF)
```

**优势**：新切削只更新受影响的子树，大部分节点不变。

### 5.7 内存分析

最坏情况：表面穿过 Voxel，细分到 t 级别。
- 表面是 2D 流形 → 占据的 MIXED cell 数 ∝ (V/cellSize)²
- 深度 k = log2(V/t)，MIXED 节点 ≈ 4^k = (V/t)²
- V=0.5, t=0.001 → (500)² = 250k 节点 → 每节点 ~40B → **10MB/voxel**

**这太大了**。但实际情况远好于最坏：
- 通常只细分到 Level 3~4（交互），此时 MIXED ≈ 几十个
- 最终精度只对导出时的少量脏 voxel 触发
- 可以计算完提取点集后**丢弃八叉树**（只保留结果点集）

### 5.8 优化：不持久化八叉树

```
需要精确数据时:
  ① 构建八叉树到目标深度
  ② 提取表面点集
  ③ 丢弃八叉树，只保留点集
```

八叉树是**临时计算结构**，不存储。内存开销 = 计算峰值，非持久占用。

对于 V=0.5, t=0.001 的极端精度，峰值 ~10MB/voxel 也可接受（一次只处理一个 voxel）。

### 5.9 复杂度

| 操作 | eval 次数 |
|------|-----------|
| 构建到 Level k | 8 × MIXED_nodes × n (每节点 8 角点，每点 n 次 SDF) |
| 增量更新 | affected_nodes × 8 × 1 (只 eval 新 SDF) |
| 表面提取 | 0（复用已有 cornerSDF） |

MIXED_nodes ≈ (V/t)² = 表面面积 / cellSize²

对 V=0.5, t=0.01, n=4:
- MIXED ≈ (50)² = 2500 节点
- eval 次数 ≈ 2500 × 8 × 4 = 80000

对 V=0.5, t=0.001, n=4:
- MIXED ≈ 250000 节点
- eval 次数 ≈ 250000 × 8 × 4 = 8M → 约 80ms @ 100M eval/s

### 5.10 优缺点总结

| 优势 | 劣势 |
|------|------|
| 概念简单，实现直接 | 极高精度时 eval 次数大 |
| 渐进精度，随时可停 | 内存峰值高（但临时） |
| 增量更新高效 | 不利用参数面的连续性 |
| 与 compositeSDF 黑盒兼容 | 脊线处线性插值精度受限 |
| 天然并行（子树独立） | — |

### 5.11 与方案 E 的结合

方案 F 可作为方案 E 中"压实/冷启动"（Step ④）的具体实现：

```
方案 E 混合策略:
  正常切削 → 增量验证+新面参数采样 (快)
  压实触发 → 方案 F 八叉树全局重算 (准)
  渐进显示 → 方案 F 只做前 3~4 级 (交互)
```

---

## 6. 八叉树编码与数据压缩

### 6.1 核心观察

八叉树绝大部分节点是 EMPTY/SOLID（均匀区），只有表面附近的 MIXED 需要展开。MIXED 节点数 ∝ (V/t)²（表面是 2D 流形）。

### 6.2 编码方案

#### 方案 1：隐式拓扑（BFS 位流）

```cpp
struct CompactOctree {
    std::vector<uint8_t> topology;  // 每 MIXED 节点 1 byte: 8 bit = 哪些子节点继续展开
    std::vector<int16_t> leafSDF;   // 只存 leaf MIXED 的角点 SDF（量化）
    uint8_t maxDepth;
};
```

BFS 序遍历，bit=1 表示子节点是 MIXED 需继续，bit=0 表示 EMPTY/SOLID 终止。

#### 方案 2：增量角点编码

父节点细分时，8 子节点的角点 = 父 8 角点 + 19 个新增中间点（6 面心 + 12 棱中点 + 1 体心）。

```
只存新增 19 个 SDF 值，子节点角点通过重组获得。
每次细分增量 = 19 值（非 64 值）
压缩比 ≈ 3.4x
```

#### 方案 3：SDF 量化

表面附近 SDF 范围 ∈ [-V, +V]，精度 t/10 足够：
```
bits = log2(2V / (t/10))
V=0.5, t=0.01 → log2(1000) ≈ 10 bit → int16 足够
```

float → int16，存储减半。

### 6.3 存储估算

| 精度 t | MIXED 节点 | 编码2+量化 | 纯点集 |
|--------|-----------|------------|--------|
| 0.01mm | ~2500 | 95KB | 60KB |
| 0.001mm | ~250k | 9.5MB | 6MB |

### 6.4 推荐策略：临时八叉树 + 持久点集

```
┌─────────────────────────────────────────┐
│ 持久存储（per voxel）                   │
│   • 表面点集 (position + normal)        │
│   • cutHistory 索引 (几十 bytes)        │
│   • cacheEpoch (有效性标记)             │
├─────────────────────────────────────────┤
│ 临时结构（计算时分配，完成后释放）      │
│   • 八叉树节点池                        │
│   • 角点 SDF 缓存                       │
└─────────────────────────────────────────┘
```

**工作流程**：
```
需要精确数据:
  ① 从 cutHistory 构建 compositeSDF
  ② 构建八叉树（渐进到目标精度）
  ③ 提取表面点集 → 写入持久存储
  ④ 释放八叉树内存
  
增量切削:
  ⑤ 新刀来 → 旧点集 SDF 验证 + 新面采样 → 更新持久点集
  ⑥ cutHistory 过长 → 触发 ①~④ 压实
```

**内存峰值**：只在单个 voxel 计算时产生（几百 KB~几 MB），计算完立即释放。全局持久存储保持在 MB 级。

### 6.5 可选：编码持久化八叉树

在某些场景（频繁渐进查询同一 voxel），可选择持久化编码后的八叉树：

```
持久化条件: cutHistory.size() > K 且 voxel 被频繁访问
存储: 编码2+量化 格式 (~95KB/voxel for t=0.01)
好处: 后续加密细分无需重算粗层级
```

通常不推荐持久化——重算成本低（从 cutHistory 回放），按需构建更简单。

---

## 7. 方案 G：MicroGrid 8³ + Adaptive Octree 精修

### 7.1 核心思想

将单 Voxel 切削分为两阶段：
1. **粗阶段**：在 bbox 内构建 8³ 压缩网格（MicroGrid），快速定位切削界面
2. **精阶段**：仅对界面 ce 内部做 adaptive octree 细分，直到弦高满足精度

### 7.2 输入适配

输入 = 一个 **bbox**（对应 MacroGrid 的一个 LeafNode 或单个 voxel 的 AABB）。
这样与上层 OpenVDB 拓扑解耦——无论 MacroGrid 用什么配置，本算法只需要一个轴对齐盒子。

### 7.3 MicroGrid 定义

```cpp
struct MicroGrid {
    // 8×8×8 = 512 个 cell element (ce)
    // 每个 ce 边长 = bbox_side / 8
    float sdf[512];           // compositeSDF 在 ce 中心的值
    uint64_t surfaceMask[8];  // 512-bit: ce 是否包含切削界面 (跨零)
    BBoxd bbox;               // 世界坐标 AABB
};
```

- **ce 尺寸**：若 MacroGrid voxelSize=0.5mm，bbox=0.5mm → ce=0.0625mm
- **surfaceMask**：标记哪些 ce 包含表面（SDF 符号变化）

### 7.4 阶段一：粗定位

```
① 对 8³ 网格中心点 eval compositeSDF → sdf[512]
② 对每个 ce 检查 6 个面邻居是否符号相反
③ 有符号变化 → surfaceMask 置 1
④ 在 surfaceMask=1 的 ce 的跨零 edge 上线性插值 → 粗采样点
⑤ 弦高检查：相邻采样点与实际零面的偏差
```

eval 次数 = 512 × (1 + N_tools)。对 n=4: 2048 次。

### 7.5 弦高检查

对粗采样点做验证：
```
在两个相邻采样点中间取中点 M
eval compositeSDF(M)
if |compositeSDF(M)| > t:
    该区域精度不足 → 对应 ce 需要细分
```

### 7.6 阶段二：Adaptive Octree 精修

**仅对未满足弦高的 ce 展开 octree**：

```
function refineCE(ce_bbox, depth):
    // 当前 ce 已经是 MicroGrid 的一个 cell
    // 边长 = bbox_side / 8 / (2^depth)
    
    8 角点 eval compositeSDF
    找跨零 edge → 插值得零交叉点
    
    弦高检查:
      if 通过 → 输出点，return
      if 未通过 且 depth < max_depth → 八叉树细分 8 子格元，递归
      if depth == max_depth → 输出点（已达精度极限）
```

### 7.7 最大细分深度

```
ce 边长 = V / 8 = 0.0625mm (V=0.5mm)
每细分一层 ÷ 2:
  Level 0: 0.0625mm (ce 自身)
  Level 1: 0.03125mm
  Level 2: 0.015625mm
  Level 3: 0.0078mm
  Level 4: 0.0039mm
  Level 5: 0.00195mm
  Level 6: 0.00098mm < 0.001mm ✓

max_depth = 6（极端精度 t=0.001mm）
```

对于常见精度 t=0.01mm → max_depth = 3 即可。

**细分是有限的，不会爆炸。**

### 7.8 数据策略

- **MicroGrid 结构**：可持久化（512 float + 8 byte mask = ~2KB/voxel），轻量
- **Octree 细分**：临时结构，只记录最终采样点，不保留树
- **输出**：`{position, normal}` 点集写入 SurfaceCache

```
持久存储:
  MicroGrid.sdf[512]       → 用于增量 SDF 验证
  MicroGrid.surfaceMask    → 快速判断哪些 ce 有界面
  SurfaceCache points[]    → 最终精确点集

临时（用完释放）:
  Octree 节点              → 细分计算过程
```

### 7.9 增量切削更新

新刀来时：
```
① 对 512 个 ce 中心 eval new_tool_sdf → 更新 sdf[] = min(old, new_tool)
② 重算 surfaceMask（符号变化检测）
③ 对变化的 ce：
   - 旧 surfaceMask=1 但新=0 → 该 ce 被完全切掉，删除旧点
   - 旧=0 但新=1 → 新暴露界面，触发精修
   - 旧=1 且新=1 → 界面可能变化，重新精修
④ 对需要精修的 ce 执行阶段二
```

增量 eval = 512 × 1（只 eval 新刀）+ 精修 ce 的细分 eval。

### 7.10 复杂度

| 操作 | eval 次数 |
|------|-----------|
| 粗定位（首次或压实） | 512 × n |
| 增量更新（新刀） | 512 + refined_ce × 8^depth |
| 精修单个 ce (t=0.01) | ~8³ = 512 (depth=3) |
| 精修单个 ce (t=0.001) | ~8⁶... 不对 → 每层只细分 MIXED |

修正——精修是 adaptive，只有 MIXED 子格元继续：
- 表面是 2D → MIXED 子格元数 ∝ (ce_size/t)² 
- ce=0.0625, t=0.01 → (6.25)² ≈ 39 个 MIXED 子格元
- 每个 MIXED 8 次 eval → 39 × 8 × n ≈ 1248 eval (n=4)

**单个 voxel 全流程**：512×n + surface_ce_count × ~1000 ≈ 几千次 eval。

### 7.11 MicroGrid 精简存储设计

借鉴 OpenVDB LeafNode 的思路：**只存窄带，其余用状态位表示**。

#### 数据结构

```cpp
struct MicroGrid {
    // ── 拓扑 (固定 128 bytes) ──
    uint64_t activeMask[8];   // 512-bit: ce 在窄带内（需要存 SDF 值）
    uint64_t signMask[8];     // 512-bit: ce 的 SDF 符号 (0=正/外部, 1=负/内部)
    
    // ── 窄带 SDF (变长) ──
    // 只存 activeMask=1 的 ce 的 SDF 值
    // 长度 = popcount(activeMask)
    int16_t* sdf;             // 量化 SDF，或用 float* 精确存储
    
    // ── 元数据 ──
    uint16_t activeCount;     // popcount 缓存
};
```

#### 三态编码

每个 ce 由 `activeMask` 和 `signMask` 两个 bit 联合编码：

| activeMask | signMask | 含义 | SDF 值 |
|:---:|:---:|------|--------|
| 0 | 0 | 外部（空气） | +bg（隐式，不存） |
| 0 | 1 | 内部（实心） | -bg（隐式，不存） |
| 1 | × | 窄带（界面附近） | 从 sdf[] 读取 |

**外部和内部不存值**——只用 1 bit 表示。只有窄带 ce 才存显式 SDF。

#### 内存占用

```
固定开销: 128 bytes (两个 mask)
变长开销: activeCount × sizeof(value)
```

典型情况（表面穿过 voxel）：
- 窄带 ce ≈ 表面面积对应的 ce 数 ≈ (V/ce)² = 8² = **~64 个**
- 用 int16: 64 × 2 = 128 bytes
- 用 float: 64 × 4 = 256 bytes

**总计：128 + 128~256 = 256~384 bytes/voxel**（vs 之前 2KB 的全量存储，节省 5~8x）

#### SDF 量化 (int16)

```cpp
// 范围 [-bg, +bg], bg = halfwidth × ce_size
// 对于 halfwidth=3, ce=0.0625mm: bg = 0.1875mm
// int16 精度 = 2*bg / 65535 = 0.375/65535 ≈ 0.0000057mm (远超 t=0.001)

int16_t encode(float sdf, float bg) { return (int16_t)(sdf / bg * 32767); }
float   decode(int16_t v, float bg)  { return v * bg / 32767.0f; }
```

#### 访问接口

```cpp
float MicroGrid::getValue(int idx) const {
    if (!isActive(idx))
        return isInside(idx) ? -bg : +bg;  // 隐式值
    // active: 从压缩数组中定位
    int offset = popcount(activeMask, idx);  // idx 之前有多少 active
    return decode(sdf[offset], bg);
}

void MicroGrid::setValue(int idx, float val) {
    if (abs(val) >= bg) {
        clearActive(idx);
        setSign(idx, val < 0);
    } else {
        setActive(idx);
        int offset = popcount(activeMask, idx);
        sdf[offset] = encode(val, bg);
    }
}
```

`popcount(mask, idx)` = `mask[0..idx]` 中 1 的个数 → 用 `__builtin_popcountll` 实现，O(1)。

#### 与 LeafNode 的对比

| | OpenVDB LeafNode | MicroGrid |
|--|-----------------|-----------|
| 格元数 | 8³=512 | 8³=512 |
| 值存储 | 全量 float[512] = 2KB | 窄带 int16[~64] = 128B |
| 拓扑 | 512-bit active mask | activeMask + signMask = 128B |
| 总大小 | ~2KB | **~256~384B** |
| 访问 | O(1) 直接索引 | O(1) popcount + 索引 |

### 7.12 优势总结（更新）

| 特性 | 说明 |
|------|------|
| 内存极致 | ~300B/voxel（而非 2KB） |
| 窄带语义一致 | 与 level set 的 narrowband 概念完全对齐 |
| 快速重建 SDF | 任意 ce 的 SDF O(1) 读取 |
| 增量友好 | 新刀更新：eval 512 次 → 更新 mask + 变长数组 |
| SIMD 友好 | mask 操作全是 64-bit 位运算 |

### 7.13 关键洞察：不需要 IPW 的 SDF

**MicroGrid 的三态 mask 替代了 IPW SDF 的角色。**

- 外部 (signMask=0)：空气，无需计算
- 内部 (signMask=1)：实心材料，只需判断新刀是否穿过
- 窄带 (activeMask=1)：已有切削面，新刀可能切掉部分旧点

切削时只 eval **新刀的 tool_sdf**，从不 eval IPW SDF。compositeSDF 回放仅用于压实/冷启动。

### 7.14 修正后的单次切削流程

```
输入: MicroGrid (当前 IPW 三态) + tool_sdf (新刀)
输出: 更新后的 MicroGrid + 新/删表面点

① 粗筛 512 ce (O(1)/ce, 纯 mask 操作 + 1次 eval):
   外部 ce (signMask=0) → 跳过（空气，刀切不到东西）
   内部 ce (signMask=1) → eval tool_sdf(ce_center):
     tool > +threshold  → 刀离材料远，不变
     tool < -threshold  → 整个 ce 被切掉 → 改为外部
     |tool| < threshold → 刀穿过实心区 → 变为窄带，进入②

② 窄带 ce → eval tool_sdf 在 8 角点:
   全正 → 刀不在此 ce → 不变
   全负 → ce 被完全切掉 → 旧点删除，改为外部
   跨零 → 刀面穿过 → 进入③

③ Adaptive octree 细分 (只 eval tool_sdf):
   递归细分到弦高 < t
   输出: 新表面点 = tool_sdf 零交叉 (法向 = tool_sdf 梯度)

④ 旧窄带点验证:
   对窄带 ce 中已有的旧表面点: eval tool_sdf(P_old)
     tool_sdf < 0 → 被切掉，删除
     tool_sdf ≥ 0 → 保留

⑤ 更新 MicroGrid:
   重算 activeMask / signMask
   重建 SurfaceCache (CSR 拼接)
```

### 7.15 Eval 次数分析

| 步骤 | eval 对象 | 次数 |
|------|-----------|------|
| ① 粗筛 | tool_sdf | ≤512 (只 eval 内部+窄带 ce) |
| ② 角点 | tool_sdf | 窄带ce数 × 8 ≈ 64×8=512 |
| ③ 细分 | tool_sdf | 跨零ce数 × ~64 (depth=3) |
| ④ 旧点验证 | tool_sdf | 旧点数 ~200 |

典型总计：**~2000 次 tool_sdf.eval**，全是新刀的解析函数。

### 7.16 compositeSDF 回放的唯一用途

仅在以下情况触发（非常规路径）：
- 压实：cutHistory > 16 → 从头重算全部三态
- 冷启动：首次构建 MicroGrid
- 时间回溯：回退到第 k 刀状态

正常增量切削路径：**零 compositeSDF 计算**。

---

*待选定方案后进入详细设计与实现。*
