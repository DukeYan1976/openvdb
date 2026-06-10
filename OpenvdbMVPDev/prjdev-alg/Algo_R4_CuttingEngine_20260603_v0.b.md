# R4 算法设计：实现刀具扫掠体 SDF 与毛坯计算模型的切削计算

  **文档编号**：`Algo_R4_CuttingEngine_20260603_v0.b.md`  
  **状态**：已修订（Spike 验证修正）  
  **作者**：Duke / Jarvas  
  **日期**：2026-06-03
  **状态**：初稿
  **作者**：Duke / Kiro
  **日期**：2026-06-01
  **前置依赖**：R2（BilletModel）、R3（ToolSweepSDF）
  **对应里程碑**：M4

  ---

  ## 1. 问题定义

  给定当前毛坯模型（IPWₙ）和一段刀具运动（ToolSweepSDF），执行材料剥离计算，输出更新后的 IPWₙ₊₁。

  根据 R1 的模式判定，切削引擎有两条执行路径：

  | 模式 | 切削策略 | 精度来源 |
  |------|----------|----------|
  | SINGLE_TRACK | SDF 布尔差集（Grid 级运算） | VoxelSize = d_v |
  | DUAL_TRACK | 宏观过滤 + 微观面元逐点距离计算 | 面元采样精度 = d_v |

  ---

  ## 2. 输入

  | 参数 | 类型 | 含义 |
  |------|------|------|
  | `billet` | BilletModel& | 当前毛坯（就地更新） |
  | `toolSDF` | ToolSweepSDF | R3 输出的刀具扫掠体解析 SDF |
  | `config` | ResolutionConfig | 分辨率配置 |

  ---

  ## 3. SINGLE_TRACK 切削算法（主路径）

  ### 3.1 流程

  function cut_SingleTrack(billet, toolSDF, config):
      voxelSize = config.d_v
      halfWidth = 3

      // Step 1: 光栅化刀具 SDF 为临时 FloatGrid
      toolGrid = rasterizeToolSDF(toolSDF, voxelSize, halfWidth)

      // Step 2: 执行 SDF 布尔差集（就地更新毛坯）
      tools::csgDifferenceSDF(*billet.sdfGrid, *toolGrid)

      // 完成。billet.sdfGrid 已更新为 IPWₙ₊₁

  ### 3.2 就这么简单？

  是的。单轨模式下，OpenVDB 的 `csgDifferenceSDF` 在内部完成：
  1. 拓扑合并（两个 Grid 的活跃体素取并集）
  2. 逐体素计算 `max(billet, -tool)`（SDF 差集的数学定义）
  3. 窄带重建（剪枝远离新表面的体素，维护窄带宽度）
  4. Tile 优化（完全被切除的区域收敛为背景值 Tile）

  **精度**：误差 ≤ d_v/2 = t/4 < t ✓

  ### 3.3 后续优化路径（方案 B）

  当性能成为瓶颈时，切换为直接 eval：

  function cut_SingleTrack_DirectEval(billet, toolSDF):
      indexBBox = billet.sdfGrid->transform().worldToIndex(toolSDF.getBoundingBox())
      accessor = billet.sdfGrid->getAccessor()

      for each activeVoxel (i,j,k) in billet.sdfGrid within indexBBox:
          worldPos = billet.sdfGrid->transform().indexToWorld(Coord(i,j,k))
          toolDist = toolSDF.eval(worldPos)

          oldVal = accessor.getValue(Coord(i,j,k))
          newVal = max(oldVal, -toolDist)

          if newVal != oldVal:
              accessor.setValue(Coord(i,j,k), newVal)

      tools::pruneLevelSet(*billet.sdfGrid)

  ---

  ## 4. DUAL_TRACK 切削算法（扩展路径）

  ### 4.1 整体流程

  function cut_DualTrack(billet, toolSDF, config):
      // Phase 1: 宏观过滤 — 确定受影响的体素集合
      affectedVoxels = macroFilter(billet.sdfGrid, toolSDF)

      // Phase 2: 微观切削 — 对受影响体素内的面元逐点计算
      for each voxelCoord in affectedVoxels:
          microCut(billet.microGrid, voxelCoord, toolSDF, config)

      // Phase 3: 宏观 SDF 同步更新
      updateMacroSDF(billet.sdfGrid, billet.microGrid, affectedVoxels)

  ### 4.2 Phase 1：宏观过滤

  利用 FloatGrid SDF 快速确定刀具影响范围：

  function macroFilter(sdfGrid, toolSDF) -> list<Coord>:
      toolBBox = toolSDF.getBoundingBox()
      indexBBox = sdfGrid->transform().worldToIndex(toolBBox)

      affected = []
      accessor = sdfGrid->getAccessor()

      for each (i,j,k) in indexBBox:
          sdfVal = accessor.getValue(Coord(i,j,k))

          // 仅处理窄带内的体素（表面附近）
          // SDF < 0 且在刀具包围盒内 → 可能被切削
          if sdfVal < 0:
              // 进一步检查：体素中心是否在刀具 SDF 内部
              worldPos = sdfGrid->transform().indexToWorld(Coord(i,j,k))
              if toolSDF.eval(worldPos) <= config.D_v:  // 留余量
                  affected.append(Coord(i,j,k))

      return affected

  **加速**：实际实现中可用 `tools::dilate` 对刀具包围盒做拓扑膨胀，直接获取候选叶节点集合，避免逐体素遍历。

  ### 4.3 Phase 2：微观切削

  对每个受影响体素内的面元执行精确距离计算：

  function microCut(microGrid, voxelCoord, toolSDF, config):
      // 检查该体素是否已有面元
      if not microGrid.hasPoints(voxelCoord):
          // 首次接触：注入面元（延迟分配触发）
          // 如果是 IPW₀ 原始表面，已有粗精度面元
          // 如果是内部体素首次暴露，需要注入精细面元
          injectSurfels_Cut(microGrid, voxelCoord, config, estimateNormal(...))

      // 遍历体素内所有活跃面元
      points = microGrid.getPoints(voxelCoord)
      for each surfel in points:
          if surfel not in group("active"):
              continue  // 已被切除，跳过

          // 精确距离计算（解析 SDF，无离散化误差）
          dist = toolSDF.eval(surfel.position)

          if dist <= 0:
              // 材料剥离：从 active group 中移除
              remove surfel from group("active")

      // 在切削边界注入新的精细面元（新暴露表面）
      if hasPartialCut(points):
          injectBoundarySurfels(microGrid, voxelCoord, toolSDF, config)

  ### 4.4 Phase 3：宏观 SDF 保守更新

  切削后更新 FloatGrid，采用**保守策略**：只要体素内有任何 active 面元，SDF 保持 ≤ 0。

  ```
  function updateMacroSDF(sdfGrid, microGrid, affectedVoxels, config):
      accessor = sdfGrid->getAccessor()

      for each voxelCoord in affectedVoxels:
          points = microGrid.getPoints(voxelCoord)
          activeCount = count(p for p in points if p in group("active"))

          if activeCount == 0:
              // 体素内材料完全被切除 → SDF 设为正值（空气）
              accessor.setValue(voxelCoord, config.D_v)

          else:
              // 保守策略：只要有任何活跃面元，SDF 保持原值不变（≤ 0）
              // 不设为 0，避免后续过滤误判内部体素为"表面"
              pass  // 不修改

      tools::pruneLevelSet(*sdfGrid)
  ```

  **理由**：之前将部分切削体素 SDF 设为 0，会让后续切削的宏观过滤
  误认为该体素"在表面"而跳过内部检查。保守策略确保：有材料的地方
  SDF 始终 ≤ 0，宏观过滤不会漏检。

  ### 4.5 切削边界面元注入（N² 表面采样）

  当体素被部分切削时，在刀具 SDF 零等值面与体素的交界处注入精细面元。
  **仅做 N² 级表面采样（非 N³ 体积采样）**，避免性能爆炸。

  ```
  function injectBoundarySurfels(microGrid, voxelCoord, toolSDF, config):
      D_v = config.D_v
      d_v = config.d_v
      N = config.N

      voxelCenter = microGrid->transform().indexToWorld(voxelCoord)

      // 估算刀具表面在该体素内的法向量（解析梯度或有限差分）
      toolNormal = toolSDF.gradient(voxelCenter)
      if length(toolNormal) < 1e-10:
          return  // 退化情况，跳过
      toolNormal = normalize(toolNormal)

      // 构建局部切平面坐标系 (u, v, toolNormal)
      u, v = buildTangentFrame(toolNormal)

      // 在切平面上按 d_v 间距做 N×N 采样（N² 级）
      for i in range(N):
          for j in range(N):
              // 切平面上的候选点
              offset_uv = ((i + 0.5 - N/2) * d_v) * u + ((j + 0.5 - N/2) * d_v) * v
              candidate = voxelCenter + offset_uv

              // 沿法向投影到刀具零等值面（一步牛顿投影）
              dist = toolSDF.eval(candidate)
              candidate = candidate - dist * toolNormal

              // 验证：投影后的点仍在该体素内？
              if not isInsideVoxel(candidate, voxelCoord, D_v):
                  continue

              appendPoint(microGrid, voxelCoord,
                         {pos: candidate, normal: toolNormal, precision: FINE})
              // active 通过 AttributeGroup 管理：groupWriteHandle("active")->set(offset, true)
  ```

  **复杂度**：N² 次 eval（而非 N³）。N=128 时 = 16384 次，可接受。

  ---

  ## 5. 统一接口

  ```cpp
  class CuttingEngine {
  public:
      // 执行单步切削
      void cut(BilletModel& billet, const ToolSweepSDF& toolSDF);

      // 批量切削（多段刀轨）
      void cutBatch(BilletModel& billet, const std::vector<ToolSweepSDF>& segments);

  private:
      void cut_SingleTrack(BilletModel& billet, const ToolSweepSDF& toolSDF);
      void cut_DualTrack(BilletModel& billet, const ToolSweepSDF& toolSDF);
  };

  // 实现
  void CuttingEngine::cut(BilletModel& billet, const ToolSweepSDF& toolSDF) {
      if (billet.isSingleTrack())
          cut_SingleTrack(billet, toolSDF);
      else
          cut_DualTrack(billet, toolSDF);
  }

  ───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────

  6. 批量切削优化

  对于连续多段刀轨（G01 序列），逐段调用 cut() 效率低下。批量优化策略：

  function cutBatch(billet, segments):
      if billet.isSingleTrack():
          // 方案1：逐段光栅化 + CSG（简单但慢）
          for seg in segments:
              cut_SingleTrack(billet, seg)

          // 方案2（后续优化）：合并多段为单个 Grid 再做一次 CSG
          // mergedGrid = union(rasterize(seg) for seg in segments)
          // csgDifferenceSDF(billet.sdfGrid, mergedGrid)

      else:  // DUAL_TRACK
          // 合并所有段的包围盒，一次性确定受影响体素
          mergedBBox = union(seg.getBoundingBox() for seg in segments)
          affectedVoxels = macroFilter(billet.sdfGrid, mergedBBox)

          for each voxelCoord in affectedVoxels:
              for seg in segments:
                  if seg.getBoundingBox().contains(voxelWorldPos):
                      microCut(billet.microGrid, voxelCoord, seg, config)

  ───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────

  7. 验收标准

  ┌─────────────────────────┬────────────────────────────────────────────────────┐
  │ 检查项                  │ 方法                                               │
  ├─────────────────────────┼────────────────────────────────────────────────────┤
  │ 单轨切削后 SDF 符号正确 │ 切削区域内采样点 SDF > 0（已变为空气）             │
  ├─────────────────────────┼────────────────────────────────────────────────────┤
  │ 切削体积精度            │ 理论扫掠体体积 vs 实际切除体素体积，偏差 < 5%      │
  ├─────────────────────────┼────────────────────────────────────────────────────┤
  │ 双轨面元剥离正确        │ 刀具内部面元移出 active group，外部面元保持 active group │
  ├─────────────────────────┼────────────────────────────────────────────────────┤
  │ 双轨宏观/微观一致性     │ SDF=正值的体素内无 active group 的面元                 │
  ├─────────────────────────┼────────────────────────────────────────────────────┤
  │ 多段切削累积正确        │ 10 段连续切削后，结果与一次性大扫掠体等价          │
  ├─────────────────────────┼────────────────────────────────────────────────────┤
  │ 可视化验证              │ 输出 .vdb，vdb_view 查看切削槽形状合理             │
  ├─────────────────────────┼────────────────────────────────────────────────────┤
  │ 性能基线                │ 单段切削（球头刀 R=5mm，步长 10mm）< 100ms（单轨） │
  └─────────────────────────┴────────────────────────────────────────────────────┘

  ───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────

  8. 潜在风险点

  【不同见解】：双轨模式 Phase 3 中，用面元活跃比例来更新宏观 SDF 是一种粗略近似。当体素被部分切削时，简单地将 SDF 设为 0 会丢失精确的距离信息。

  更精确的方案：从活跃面元点云重建局部 SDF（如用面元位置做最近点距离估算）。但这增加了计算复杂度。

  第一阶段决策：接受粗略近似。双轨模式下，精确几何由 PointDataGrid 面元承载，FloatGrid 仅用于快速过滤（不需要精确 SDF
  值，只需要正确的正/负/零判定）。后续如果过滤误判率过高，再引入精确重建。

  ───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────

  修订记录

  ┌──────┬────────────┬──────────┐
  │ 版本 │ 日期       │ 修订内容 │
  ├──────┼────────────┼──────────┤
  │ v0.a │ 2026-06-01 │ 初稿创建 │
  │ v0.b │ 2026-06-03 │ §4.4: `active` 标记由 uint8_t 属性改为 AttributeGroup（R6 Spike 验证） │