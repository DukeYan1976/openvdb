**文档编号**：`TaskPlan_Phase2_20260601_v0.a.md`
  **状态**：初稿
  **日期**：2026-06-01

  ---

  ## Sprint 6：DualGrid 毛坯构建

  | Task | 描述 | 测试先行 |
  |------|------|----------|
  | T6.1 | MemoryStats 工具类 | 先写测试 |
  | T6.2 | PointDataGrid 创建 + 属性注册 | 先写测试 |
  | T6.3 | IPW₀ 粗面元注入（表面体素） | 先写测试 |
  | T6.4 | 双轨 BilletModel 构建集成 | 先写测试 |

  ### T6.2 测试用例

  ```cpp
  TEST(DualGrid, CreatePointDataGrid_SharedTransform) {
      ResolutionConfig cfg = solveResolution(0.01, 5.0, 1.0, {500, 500, 200});
      ASSERT_EQ(cfg.mode, ResolutionConfig::DUAL_TRACK);

      auto billet = buildBillet(cfg, {0,0,0}, {500, 500, 200});
      ASSERT_NE(billet.sdfGrid, nullptr);
      ASSERT_NE(billet.microGrid, nullptr);

      // 共享 Transform 验证
      EXPECT_DOUBLE_EQ(billet.sdfGrid->voxelSize()[0], cfg.D_v);
      EXPECT_DOUBLE_EQ(billet.microGrid->voxelSize()[0], cfg.D_v);
  }

  TEST(DualGrid, IPW0_SurfelsInjected) {
      ResolutionConfig cfg;
      cfg.mode = ResolutionConfig::DUAL_TRACK;
      cfg.d_v = 0.1; cfg.D_v = 6.4; cfg.N = 64;

      auto billet = buildBillet(cfg, {0,0,0}, {30, 30, 20});

      // PointDataGrid 应有面元
      size_t pointCount = openvdb::points::pointCount(billet.microGrid->tree());
      EXPECT_GT(pointCount, 0u);

      // 面元数应远小于精细模式（粗精度）
      // 精细模式每表面体素 N²=4096 个，粗模式 ≤ 4 个
      size_t leafCount = billet.microGrid->tree().leafCount();
      double avgPerLeaf = static_cast<double>(pointCount) / leafCount;
      EXPECT_LT(avgPerLeaf, 100.0);  // 粗精度，每叶节点不多
  }

  TEST(DualGrid, MemoryStats_Reported) {
      ResolutionConfig cfg;
      cfg.mode = ResolutionConfig::DUAL_TRACK;
      cfg.d_v = 0.1; cfg.D_v = 6.4; cfg.N = 64;

      auto billet = buildBillet(cfg, {0,0,0}, {30, 30, 20});

      MemoryStats stats;
      stats.update(billet.sdfGrid, billet.microGrid);

      EXPECT_GT(stats.floatGridBytes, 0u);
      EXPECT_GT(stats.pointGridBytes, 0u);
      EXPECT_GT(stats.pointCount, 0u);
      EXPECT_EQ(stats.activePointCount, stats.pointCount); // 初始全活跃
      stats.print();  // 输出到 stdout 供人工检查
  }

  ───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────

  Sprint 7：DualGrid 切削引擎

  ┌──────┬─────────────────────────────────┬──────────┐
  │ Task │ 描述                            │ 测试先行 │
  ├──────┼─────────────────────────────────┼──────────┤
  │ T7.1 │ 宏观过滤（Phase 1）             │ 先写测试 │
  ├──────┼─────────────────────────────────┼──────────┤
  │ T7.2 │ 微观面元剥离（Phase 2）         │ 先写测试 │
  ├──────┼─────────────────────────────────┼──────────┤
  │ T7.3 │ 边界面元注入（Phase 3, N²采样） │ 先写测试 │
  ├──────┼─────────────────────────────────┼──────────┤
  │ T7.4 │ 宏观 SDF 保守更新（Phase 4）    │ 先写测试 │
  ├──────┼─────────────────────────────────┼──────────┤
  │ T7.5 │ 内存统计输出（Phase 5）         │ 先写测试 │
  └──────┴─────────────────────────────────┴──────────┘

  T7.2 测试用例

  TEST(DualGridCut, MicroCut_SurfelsDeactivated) {
      ResolutionConfig cfg;
      cfg.mode = ResolutionConfig::DUAL_TRACK;
      cfg.d_v = 0.1; cfg.D_v = 6.4; cfg.N = 64;

      auto billet = buildBillet(cfg, {0,0,0}, {30, 30, 20});
      size_t pointsBefore = openvdb::points::pointCount(billet.microGrid->tree());

      // 球头刀切入顶面
      ToolSweepSDF tool(ToolType::BALL_END, 5.0, 0, 30, {15,15,22}, {15,15,22});
      CuttingEngine engine;
      engine.cut(billet, tool);

      // 活跃面元应减少
      size_t activeAfter = countActivePoints(billet.microGrid);
      EXPECT_LT(activeAfter, pointsBefore);
  }

  TEST(DualGridCut, BoundaryInjection_FinePoints) {
      ResolutionConfig cfg;
      cfg.mode = ResolutionConfig::DUAL_TRACK;
      cfg.d_v = 0.1; cfg.D_v = 6.4; cfg.N = 64;

      auto billet = buildBillet(cfg, {0,0,0}, {30, 30, 20});
      size_t pointsBefore = openvdb::points::pointCount(billet.microGrid->tree());

      ToolSweepSDF tool(ToolType::BALL_END, 5.0, 0, 30, {5,15,22}, {25,15,22});
      CuttingEngine engine;
      engine.cut(billet, tool);

      // 总点数应增加（边界注入了精细面元）
      size_t pointsAfter = openvdb::points::pointCount(billet.microGrid->tree());
      EXPECT_GT(pointsAfter, pointsBefore);

      // 验证新注入的面元精度
      // 新面元到刀具表面的距离应 < d_v
      // (具体实现在 verifyBoundaryPrecision 辅助函数中)
  }

  TEST(DualGridCut, MemoryGrowth_Monitored) {
      ResolutionConfig cfg;
      cfg.mode = ResolutionConfig::DUAL_TRACK;
      cfg.d_v = 0.1; cfg.D_v = 6.4; cfg.N = 64;

      auto billet = buildBillet(cfg, {0,0,0}, {30, 30, 20});

      MemoryStats before, after;
      before.update(billet.sdfGrid, billet.microGrid);

      ToolSweepSDF tool(ToolType::BALL_END, 5.0, 0, 30, {5,15,22}, {25,15,22});
      CuttingEngine engine;
      engine.cut(billet, tool);

      after.update(billet.sdfGrid, billet.microGrid);

      printf("Memory delta: +%.2f MB\n",
             (after.totalBytes - before.totalBytes) / 1e6);

      // FloatGrid 内存应基本不变（仅值更新，无拓扑变化）
      EXPECT_NEAR(after.floatGridBytes, before.floatGridBytes,
                  before.floatGridBytes * 0.1);
  }

  ───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────

  Sprint 8：精度验证 + 对比

  ┌──────┬──────────────────────────────────────┬──────────┐
  │ Task │ 描述                                 │ 测试先行 │
  ├──────┼──────────────────────────────────────┼──────────┤
  │ T8.1 │ 面元级精度验证（到刀具表面距离）     │ 先写测试 │
  ├──────┼──────────────────────────────────────┼──────────┤
  │ T8.2 │ 双轨 vs 单轨精度对比（小件）         │ 先写测试 │
  ├──────┼──────────────────────────────────────┼──────────┤
  │ T8.3 │ 大件场景验证（单轨不可行，双轨可行） │ 先写测试 │
  └──────┴──────────────────────────────────────┴──────────┘

  T8.1 测试用例

  TEST(DualGridAccuracy, BoundarySurfels_WithinDv) {
      ResolutionConfig cfg;
      cfg.mode = ResolutionConfig::DUAL_TRACK;
      cfg.d_v = 0.1; cfg.D_v = 6.4; cfg.N = 64;

      auto billet = buildBillet(cfg, {0,0,0}, {30, 30, 20});
      ToolSweepSDF tool(ToolType::BALL_END, 5.0, 0, 30, {5,15,22}, {25,15,22});

      CuttingEngine engine;
      engine.cut(billet, tool);

      // 遍历所有 FINE 精度的活跃面元
      // 验证它们到刀具表面的距离 < d_v
      double maxError = 0;
      size_t fineCount = 0;
      forEachPoint(billet.microGrid, [&](const Vec3d& pos, uint8_t active, uint8_t prec) {
          if (active == 1 && prec == FINE) {
              double dist = std::abs(tool.eval(pos));
              maxError = std::max(maxError, dist);
              fineCount++;
          }
      });

      printf("Fine surfels: %zu, max error: %.6f mm (target: %.6f)\n",
             fineCount, maxError, cfg.d_v);
      EXPECT_LT(maxError, cfg.d_v);
  }

  ───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────

  执行顺序

  Sprint 6: T6.1 → T6.2 → T6.3 → T6.4
                                    ↓
  Sprint 7: T7.1 → T7.2 → T7.3 → T7.4 → T7.5
                                           ↓
  Sprint 8: T8.1 → T8.2 → T8.3

  ───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────

  TDD commit 节奏

  test(DualGrid): add failing tests for PointDataGrid creation  [RED]
  feat(DualGrid): implement shared Transform + attribute registration  [GREEN]
  refactor(DualGrid): extract MemoryStats utility  [REFACTOR]

  ───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────

  修订记录

  ┌──────┬────────────┬──────────┐
  │ 版本 │ 日期       │ 修订内容 │
  ├──────┼────────────┼──────────┤
  │ v0.a │ 2026-06-01 │ 初稿创建 │
  └──────┴────────────┴──────────┘
