  # 第二阶段详细设计：DualGrid 双轨高精度切削

  **文档编号**：`Phase2_DualGrid_Design_20260601_v0.a.md`
  **状态**：初稿
  **作者**：Duke / Kiro
  **日期**：2026-06-01
  **前置**：第一阶段 MVP 完成（单轨验证通过）

  ---

  ## 1. 目标

  在单轨方案内存不可行的场景下（大件+高精度），实现 DualGrid 双轨架构：
  - FloatGrid(D_v) 做宏观快速过滤
  - PointDataGrid(D_v) 做微观精确切削（面元精度 d_v）
  - 全程监控两个 Grid 的内存使用

  **验收标准**：
  - 对于 500×500×200mm 毛坯 + t=0.01mm 精度，单轨需要 ~864GB 不可行
  - 双轨 D_v=0.64mm 时，FloatGrid 内存 < 100MB，PointDataGrid 按需增长
  - 切削后新表面精度达到 d_v=0.005mm 级别

  ---

  ## 2. 架构回顾（严格遵循原设计）

  ### 2.1 共享 Transform

  两个 Grid 共享同一个 Transform（VoxelSize = D_v）
  同一个索引 (i,j,k)：
    FloatGrid[i,j,k]     → SDF 标量值（精度 ~D_v）
    PointDataGrid[i,j,k] → 该体素内的面元点集合（精度 ~d_v）

  ### 2.2 面元密度

  N = D_v / d_v = 面元密度因子
  每个表面体素内约 N² 个面元（表面采样）
  IPW₀ 初始面元间距 = d_v_init ≥ 10×d_v（粗精度）
  切削后新表面面元间距 = d_v（仿真精度）

  ### 2.3 遍历规则

  以 FloatGrid 为主树遍历
  PointDataGrid 为从属树，按需触发延迟注入
  不存在"PointDataGrid 有数据但 FloatGrid 无对应体素"的情况

  ---

  ## 3. 数据结构设计

  ### 3.1 PointDataGrid 属性布局

  ```cpp
  // 每个点的属性（SoA 布局由 OpenVDB PointDataGrid 自动管理）
  struct SurfelAttributes {
      Vec3f position;    // 相对于体素中心的局部偏移（精度 d_v）
      Vec3f normal;      // 表面法向量
      uint8_t active;    // 1=材料存在, 0=已切除
      uint8_t precision; // 0=COARSE(IPW₀), 1=FINE(切削后)
  };

  3.2 内存监控结构

  struct MemoryStats {
      size_t floatGridBytes;      // FloatGrid 内存
      size_t pointGridBytes;      // PointDataGrid 内存
      size_t totalBytes;          // 总计
      size_t pointCount;          // 总面元数
      size_t activePointCount;    // 活跃面元数
      size_t leafNodeCount;       // 叶节点数

      void update(const FloatGrid::Ptr& sdf, const PointDataGrid::Ptr& pts);
      void print() const;
  };

  ───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────

  4. 模块设计

  4.1 DualGridBuilder（扩展 BilletBuilder）

  输入: ResolutionConfig(mode=DUAL_TRACK), origin, dims
  输出: BilletModel { sdfGrid(D_v), microGrid(D_v), ... }

  流程:
    1. 创建共享 Transform(D_v)
    2. 构建 FloatGrid SDF（长方体窄带，复用第一阶段代码）
    3. 创建 PointDataGrid（共享 Transform）
    4. 注册属性描述符（position, normal, active, precision）
    5. 为窄带表面体素注入 IPW₀ 粗面元（d_v_init = max(10*d_v, D_v)）
    6. 记录初始内存

  4.2 IPW₀ 面元注入

  对每个表面体素 (i,j,k)（|SDF| < halfWidth * D_v 且 SDF 零交叉）:
    1. 从 SDF 梯度估算法向量（长方体用解析法向）
    2. 在体素表面区域按 d_v_init 间距生成 N_init² 个面元
       N_init = max(2, floor(D_v / d_v_init))
    3. 面元位置 = 体素中心 + 局部偏移（投影到 SDF 零等值面）
    4. 标记 precision = COARSE

  4.3 DualGridCutter（扩展 CuttingEngine）

  输入: BilletModel(DUAL_TRACK), ToolSweepSDF
  流程:
    Phase 1 - 宏观过滤:
      遍历 FloatGrid 窄带体素
      筛选在刀具 BBox 内且 SDF ≤ 0 的体素 → affectedVoxels

    Phase 2 - 微观切削:
      for each voxelCoord in affectedVoxels:
        if PointDataGrid 该体素无数据:
          触发延迟注入（从 SDF 梯度生成面元）
        for each surfel in voxel:
          if surfel.active == 0: continue
          dist = toolSDF.eval(surfel.position_world)
          if dist <= 0:
            surfel.active = 0  // 剥离

    Phase 3 - 边界面元注入（N² 表面采样）:
      对部分切削的体素，在刀具零等值面上注入精细面元
      precision = FINE

    Phase 4 - 宏观 SDF 保守更新:
      activeCount == 0 → SDF = +D_v
      activeCount > 0  → 保持原值不变

    Phase 5 - 内存统计:
      更新 MemoryStats

  4.4 延迟注入触发

  function injectOnDemand(microGrid, voxelCoord, sdfGrid, config):
    // 该体素首次被刀具接触，需要生成面元
    // 判断是 IPW₀ 表面还是内部首次暴露

    sdfVal = sdfGrid.getValue(voxelCoord)
    if |sdfVal| < halfWidth * D_v:
      // 表面体素：应该在初始化时已注入（IPW₀）
      // 如果没有，补注入粗面元
      injectSurfels_IPW0(...)
    else:
      // 内部体素首次暴露：注入精细面元
      injectSurfels_Cut(...)

  ───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────

  5. 精度验证方案

  5.1 面元级精度

  对切削后的活跃面元，计算其到理论切削表面的距离:
    for each active surfel:
      dist_to_tool_surface = |toolSDF.eval(surfel.position)|
      assert dist_to_tool_surface < d_v  // 面元精度

  5.2 与单轨对比

  同一切削场景，分别用:
    - 单轨 FloatGrid(d_v=0.005mm) → 作为 ground truth（如果内存允许小件）
    - 双轨 FloatGrid(D_v=0.64mm) + PointDataGrid
  比较两者的切削表面偏差

  ───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────

  6. 内存监控要求

  每次切削操作后输出：

  [MemStats] FloatGrid: 45.2 MB | PointGrid: 12.8 MB | Total: 58.0 MB
  [MemStats] Points: 1,234,567 (active: 987,654) | Leaves: 3,456
  [MemStats] Delta: +2.1 MB (inject: +3.5 MB, prune: -1.4 MB)

  ───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────

  7. 潜在风险点

  【不同见解】：PointDataGrid 的 appendAttribute 和动态点注入在 OpenVDB
  中不是零成本操作。每次注入新面元需要重新分配叶节点内存。高频切削时（每段都触发注入），可能产生严重的内存碎片。

  缓解：

  1. 第二阶段先验证正确性，不追求性能
  2. 预分配策略：初始化时为所有窄带体素预留面元容量
  3. 后续引入 RCU 紧凑化

  ───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────

  修订记录

  ┌──────┬────────────┬──────────┐
  │ 版本 │ 日期       │ 修订内容 │
  ├──────┼────────────┼──────────┤
  │ v0.a │ 2026-06-01 │ 初稿创建 │
  └──────┴────────────┴──────────┘
