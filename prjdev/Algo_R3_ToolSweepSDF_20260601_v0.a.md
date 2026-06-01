# R3 算法设计：建立刀具固定轴线位移扫描体 SDF


  **文档编号**：`Algo_R3_ToolSweepSDF_20260601_v0.a.md`

  **状态**：初稿

  **作者**：Duke / Kiro

  **日期**：2026-06-01

  **前置依赖**：R1（ResolutionConfig 提供 VoxelSize）

  **对应里程碑**：M3


  ---

  ## 1. 问题定义

  给定 3 轴加工场景下的刀具几何参数和一段直线位移（起点 A → 终点 B），计算刀具运动扫掠体的解析 SDF（符号距离场）。

  **约束**：
  - 刀具轴线固定（Z 轴方向），仅做 XYZ 平移
  - 第一阶段支持三种刀具类型：球头刀、平底刀、牛鼻刀（圆角端铣刀）
  - SDF 必须为无分支解析解（Branchless Exact SDF），适配后续 SIMD 优化

  ---

  ## 2. 输入

  | 参数 | 类型 | 含义 |
  |------|------|------|
  | `toolType` | enum | BALL_END / FLAT_END / BULL_NOSE |
  | `R` | double | 刀具半径 (mm) |
  | `r` | double | 圆角半径 (mm)，仅 BULL_NOSE 有效 |
  | `H` | double | 刀具有效切削长度 (mm) |
  | `A` | Vec3d | 刀尖中心起点（世界坐标） |
  | `B` | Vec3d | 刀尖中心终点（世界坐标） |

  ---

  ## 3. 扫掠体几何分析

  3 轴直线运动下，刀具扫掠体的几何形状取决于刀具类型：

  | 刀具类型 | 静态几何 | 直线扫掠体几何 |
  |----------|----------|----------------|
  | 球头刀 (BALL_END) | 半球 + 圆柱 | 胶囊体底部 + 扁平圆柱带 |
  | 平底刀 (FLAT_END) | 圆柱 + 平底 | 扁平圆柱（沿运动方向拉伸） |
  | 牛鼻刀 (BULL_NOSE) | 圆柱 + 底部圆环 | 圆环扫掠 + 圆柱带 |

  **第一阶段简化**：仅考虑刀尖部分（底部切削区域）的扫掠体。刀具侧面（圆柱壁）的扫掠在大多数 3 轴精加工中不参与切削，暂不建模。

  ---

  ## 4. 核心算法：无分支解析 SDF

  ### 4.1 球头刀扫掠体 — 胶囊体 SDF

  球头刀底部为半球，直线扫掠后形成**胶囊体（Capsule）**：

  function capsuleSDF(p, A, B, R) -> double:
      // p: 空间查询点
      // A, B: 刀尖中心起止点
      // R: 球头半径

      AB = B - A
      AP = p - A

      // 无分支投影：clamp 替代 if/else
      t = clamp(dot(AP, AB) / dot(AB, AB), 0.0, 1.0)

      // 最近点
      closest = A + t * AB

      // 有符号距离（负值 = 内部）
      return length(p - closest) - R

  **数学性质**：
  - t=0 时退化为以 A 为圆心的球
  - t=1 时退化为以 B 为圆心的球
  - 0<t<1 时为线段上最近点的球
  - 全过程无条件分支，clamp 映射为 `max(0, min(1, x))`

  ### 4.2 平底刀扫掠体 — 扁平胶囊体 SDF

  平底刀底部为圆盘，直线扫掠后形成**扁平胶囊体**（圆柱沿运动方向的闵可夫斯基和）：

  function flatEndSweepSDF(p, A, B, R, H) -> double:
      AB = B - A
      AP = p - A

      // 水平投影（XY 平面内的胶囊体）
      t = clamp(dot(AP.xy, AB.xy) / dot(AB.xy, AB.xy), 0.0, 1.0)
      closest_xy = A.xy + t * AB.xy
      dist_radial = length(p.xy - closest_xy) - R

      // 垂直约束（Z 方向的圆柱高度）
      // 刀尖在 A.z（或 B.z），刀顶在 A.z + H
      z_bottom = A.z + t * (B.z - A.z)  // 沿路径的底面 Z
      z_top = z_bottom + H
      dist_z = max(z_bottom - p.z, p.z - z_top)

      // 组合：取外部最大距离
      return max(dist_radial, dist_z)

  **注意**：当 A.z ≠ B.z（斜向运动）时，底面 Z 随 t 线性变化。这仍然是无分支的。

  ### 4.3 牛鼻刀扫掠体 — 圆环胶囊体 SDF

  牛鼻刀底部为圆环面（Torus 截面），直线扫掠后形成**圆环胶囊体**：

  function bullNoseSweepSDF(p, A, B, R, r) -> double:
      // R: 刀具外半径
      // r: 底部圆角半径
      // 圆环中心半径 = R - r

      AB = B - A
      AP = p - A

      // 沿运动方向投影
      t = clamp(dot(AP, AB) / dot(AB, AB), 0.0, 1.0)
      center = A + t * AB  // 当前刀尖中心

      // 相对于刀尖中心的局部坐标
      local = p - center

      // 圆环 SDF（旋转体）
      // 圆环中心环半径 = R - r，截面圆半径 = r
      q = Vec2(length(local.xy) - (R - r), local.z)
      dist_torus = length(q) - r

      // 圆柱壁（圆环以上部分）
      dist_cyl_radial = length(local.xy) - R
      dist_cyl_z = local.z - 0  // 底面以上
      dist_cylinder = max(dist_cyl_radial, dist_cyl_z)  // 仅上半部分

      // 组合：底部用圆环，上部用圆柱
      if local.z <= 0:
          return dist_torus
      else:
          return max(dist_cyl_radial, -local.z + H)  // 简化

  **注意**：牛鼻刀的 Z≤0 区域（底部圆角）用圆环 SDF，Z>0 区域用圆柱 SDF。这里有一个条件分支。第一阶段可接受，后续用 `max/min` 组合消除：

  // 无分支版本（近似）
  return min(dist_torus, dist_cylinder)

  ---

  ## 5. 统一接口

  ```cpp
  class ToolSweepSDF {
  public:
      ToolSweepSDF(ToolType type, double R, double r, double H,
                   const Vec3d& A, const Vec3d& B);

      // 核心：计算空间点 p 到扫掠体的有符号距离
      // 返回值 < 0 表示 p 在扫掠体内部（被切削）
      double eval(const Vec3d& p) const;

      // 批量评估（为后续 SIMD 预留）
      void evalBatch(const Vec3d* points, double* distances, size_t count) const;

      // 包围盒（用于空间过滤加速）
      BBoxd getBoundingBox() const;

  private:
      ToolType mType;
      double mR, mr, mH;
      Vec3d mA, mB;
      Vec3d mAB;         // B - A（预计算）
      double mABdotAB;   // dot(AB, AB)（预计算）
  };

  ───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────

  6. 包围盒计算

  用于宏观过滤，快速跳过不受影响的体素：

  function computeBBox(A, B, R, H) -> BBoxd:
      minPt = min(A, B) - Vec3d(R, R, 0)
      maxPt = max(A, B) + Vec3d(R, R, H)
      return BBoxd(minPt, maxPt)

  ───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
  ## 7. 与 OpenVDB 的集成方式

  ### 7.1 单轨模式（SINGLE_TRACK）

  两种等价方案，精度相同（都受限于 VoxelSize = d_v），误差 ≤ d_v/2 = t/4 < t。

  #### 方案 A：光栅化 + csgDifferenceSDF（第一阶段采用）

  将解析 SDF 光栅化为临时 FloatGrid，利用 OpenVDB 原生布尔运算：

  function cutBillet_RasterizeCSG(billetSDF, toolSDF, voxelSize, halfWidth):
      // 1. 光栅化刀具 SDF 为临时 Grid
      toolGrid = rasterizeToolSDF(toolSDF, voxelSize, halfWidth)

      // 2. 原生布尔差集（OpenVDB 内部优化：拓扑合并、Tile 剪枝、窄带重建）
      tools::csgDifferenceSDF(billetSDF, toolGrid)

      // billetSDF 就地更新，toolGrid 可释放


  function rasterizeToolSDF(toolSDF, voxelSize, halfWidth) -> FloatGrid:
      bbox = toolSDF.getBoundingBox()
      xform = Transform::createLinearTransform(voxelSize)

      grid = FloatGrid::create(halfWidth * voxelSize)
      grid->setTransform(xform)
      grid->setGridClass(GRID_LEVEL_SET)

      for each (i,j,k) in indexBBox(bbox, xform):
          worldPos = xform->indexToWorld(Coord(i,j,k))
          dist = toolSDF.eval(worldPos)
          if abs(dist) < halfWidth * voxelSize:
              grid->setValue(Coord(i,j,k), dist)

      return grid

  **优势**：代码极简（3 行核心逻辑），OpenVDB 自动处理窄带拓扑维护
  **劣势**：额外内存（临时 Grid），光栅化遍历开销

  #### 方案 B：直接逐体素 eval（后续优化方案）

  不创建临时 Grid，直接在毛坯窄带体素上求解析值：

  function cutBillet_DirectEval(billetSDF, toolSDF):
      bbox = toolSDF.getBoundingBox()
      indexBBox = billetSDF->transform().worldToIndex(bbox)

      // 仅遍历刀具包围盒内的活跃体素
      for each activeVoxel (i,j,k) in billetSDF within indexBBox:
          worldPos = billetSDF->transform().indexToWorld(Coord(i,j,k))
          toolDist = toolSDF.eval(worldPos)

          // SDF 布尔差集的数学等价：max(billet, -tool)
          billetSDF[i,j,k] = max(billetSDF[i,j,k], -toolDist)

      // 需要手动触发窄带拓扑重建
      tools::pruneLevelSet(billetSDF)
      // 可选：tools::renormalizeLevelSet(billetSDF) 修正 SDF 梯度

  **优势**：零额外内存，无临时对象分配，适合高频调用
  **劣势**：需手动维护窄带拓扑（pruneLevelSet），SDF 梯度可能需要重归一化

  #### 精度分析

  两种方案精度等价：

  误差来源：体素离散化（零等值面位置误差 ≤ d_v/2）
  单轨模式下：d_v = 0.5t → 最大误差 = d_v/2 = t/4 < t ✓

  光栅化不会引入额外误差——因为毛坯本身已经是离散化的 Grid，刀具 Grid 与毛坯 Grid 在同一离散空间中运算，精度瓶颈始终是 VoxelSize。

  #### 第一阶段决策

  采用**方案 A**（光栅化 + csgDifferenceSDF）：
  - 实现最快，验证逻辑正确性优先
  - 性能瓶颈出现后再切换方案 B

  ### 7.2 双轨模式（DUAL_TRACK）

  不光栅化为 Grid，直接在切削遍历中调用 `toolSDF.eval(p)` 对每个面元求精确距离：

  // 在 R4 CuttingEngine 中：
  // 宏观过滤：检查体素是否在刀具包围盒内
  if toolSDF.getBoundingBox().contains(voxelWorldPos):
      // 微观精确计算：对体素内每个面元求解析距离
      for each surfel in microGrid.getPoints(i,j,k):
          dist = toolSDF.eval(surfel.position)
          if dist <= 0:
              surfel.active = 0  // 位掩码翻转

  双轨模式下面元位置是连续坐标（精度 d_v），解析 SDF 也是连续函数，**不存在离散化误差**——精度仅受面元采样密度限制。
  ───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────

  8. 验收标准

  ┌────────────────────────────┬──────────────────────────────────────────────────────┐
  │ 检查项                     │ 方法                                                 │
  ├────────────────────────────┼──────────────────────────────────────────────────────┤
  │ 球头刀：A=B 时退化为球体   │ eval(球面上的点) ≈ 0，误差 < 1e-10                   │
  ├────────────────────────────┼──────────────────────────────────────────────────────┤
  │ 球头刀：线段中点处距离正确 │ 手算验证                                             │
  ├────────────────────────────┼──────────────────────────────────────────────────────┤
  │ 平底刀：静止时退化为圆柱   │ eval(圆柱面上的点) ≈ 0                               │
  ├────────────────────────────┼──────────────────────────────────────────────────────┤
  │ 包围盒完全包含扫掠体       │ 随机采样 10000 点，eval<0 的点全在 BBox 内           │
  ├────────────────────────────┼──────────────────────────────────────────────────────┤
  │ 无分支性能                 │ 同一批点的 evalBatch 耗时方差 < 5%（无分支预测抖动） │
  ├────────────────────────────┼──────────────────────────────────────────────────────┤
  │ 光栅化后 SDF 与解析值一致  │ 随机采样点，光栅化插值 vs eval() 偏差 < voxelSize    │
  └────────────────────────────┴──────────────────────────────────────────────────────┘

  ───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────

  9. 潜在风险点

  【不同见解】：当前算法假设刀具从 A 到 B 做瞬时直线运动（整段路径同时作用）。但实际 G 代码执行是连续的——刀具在 t=0 时在 A，t=1 时在 B。如果步长过大（A-B 距离 >>
  刀具半径），中间可能存在"跳过"的材料区域。

  缓解：

  1. 对于 3 轴直线插补，胶囊体 SDF 数学上精确覆盖了整条路径的扫掠体积，不存在跳过问题——这正是解析解的优势
  2. 但对于圆弧插补（G02/G03），直线胶囊体不再适用。第一阶段仅支持直线段（G01），圆弧需离散为短直线段序列

  后续扩展：圆弧扫掠体可用"圆弧胶囊体"（沿圆弧的闵可夫斯基和）建模，或将圆弧细分为足够短的直线段使误差 < d_v。

  ───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────

  修订记录

  ┌──────┬────────────┬──────────┐
  │ 版本 │ 日期       │ 修订内容 │
  ├──────┼────────────┼──────────┤
  │ v0.a │ 2026-06-01 │ 初稿创建 │
  | v0.b | 2026-06-01 | 第7节重构：区分方案A(光栅化+CSG)和方案B(直接eval)，补充精度等价性分析；第一阶段采用方案A |
  