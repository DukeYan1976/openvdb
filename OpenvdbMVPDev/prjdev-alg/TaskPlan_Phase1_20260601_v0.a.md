# 第一阶段 Task 实施方案（TDD 驱动）

**文档编号**：`TaskPlan_Phase1_20260601_v0.a.md`  
**状态**：初稿  
**作者**：Duke / Kiro  
**日期**：2026-06-01  
**方法论**：Test-Driven Development（红→绿→重构）  

---

## 1. 总体原则

1. **先写测试，再写实现**：每个 Task 先产出 GTest 用例（红），再写最小实现使其通过（绿），最后重构
2. **单轨优先**：SINGLE_TRACK 路径是第一阶段的主线，DUAL_TRACK 为可选扩展
3. **端到端验证**：最终 Task 必须串联 R1→R2→R3→R4 完成一次完整切削并输出 .vdb
4. **每个 Task 独立可编译、可测试**

---

## 2. Task 分解

### Sprint 1：基础设施 + R1

| Task | 描述 | 测试先行 | 产出 |
|------|------|----------|------|
| T1.1 | CMake 项目骨架搭建 | 编译空 GTest 通过 | `yggdrasil/CMakeLists.txt` |
| T1.2 | 公共类型定义 | — | `types/YggTypes.h` |
| T1.3 | ResolutionSolver — 单轨判定 | 先写测试 | `core/ResolutionSolver.h/.cpp` |
| T1.4 | ResolutionSolver — 双轨/Atlas 判定 | 先写测试 | 扩展 T1.3 |

#### T1.3 测试用例（红）

```cpp
TEST(ResolutionSolver, SingleTrack_LowPrecisionSmallPart) {
    auto cfg = solveResolution(1.0, 50.0, 20.0, {300, 300, 300});
    EXPECT_EQ(cfg.mode, ResolutionConfig::SINGLE_TRACK);
    EXPECT_DOUBLE_EQ(cfg.d_v, 0.5);
    EXPECT_EQ(cfg.N, 1);
}

TEST(ResolutionSolver, DualTrack_HighPrecisionLargePart) {
    auto cfg = solveResolution(0.01, 5.0, 1.0, {500, 500, 200});
    EXPECT_EQ(cfg.mode, ResolutionConfig::DUAL_TRACK);
    EXPECT_DOUBLE_EQ(cfg.d_v, 0.005);
    EXPECT_EQ(cfg.N, 128);
    EXPECT_DOUBLE_EQ(cfg.D_v, 0.64);
}

TEST(ResolutionSolver, Atlas_UltraPrecision) {
    auto cfg = solveResolution(0.001, 0.5, 0.1, {50, 50, 50});
    EXPECT_EQ(cfg.mode, ResolutionConfig::ATLAS_REGION);
}

TEST(ResolutionSolver, BoundaryCase_DupperLessThanDv) {
    auto cfg = solveResolution(10.0, 1.0, 0.5, {100, 100, 100});
    EXPECT_EQ(cfg.mode, ResolutionConfig::SINGLE_TRACK);
    EXPECT_EQ(cfg.N, 1);
}
```

---

### Sprint 2：R2 毛坯构建

| Task | 描述 | 测试先行 | 产出 |
|------|------|----------|------|
| T2.1 | BilletBuilder — 单轨 FloatGrid 构建 | 先写测试 | `core/BilletBuilder.h/.cpp` |
| T2.2 | BilletBuilder — SDF 正确性验证 | 先写测试 | 扩展 T2.1 |
| T2.3 | BilletBuilder — .vdb 序列化输出 | 先写测试 | 扩展 T2.1 |
| T2.4 | BilletBuilder — 双轨空壳构建（可选） | 先写测试 | 扩展 T2.1 |

#### T2.1 测试用例（红）

```cpp
TEST(BilletBuilder, SingleTrack_CreatesValidLevelSet) {
    ResolutionConfig cfg;
    cfg.mode = ResolutionConfig::SINGLE_TRACK;
    cfg.d_v = 0.5;  cfg.D_v = 0.5;  cfg.N = 1;

    auto billet = buildBillet(cfg, {0,0,0}, {20, 20, 10});
    ASSERT_NE(billet.sdfGrid, nullptr);
    EXPECT_EQ(billet.sdfGrid->getGridClass(), openvdb::GRID_LEVEL_SET);
    EXPECT_NEAR(billet.sdfGrid->voxelSize()[0], 0.5, 1e-10);
}

TEST(BilletBuilder, SingleTrack_SDFSignCorrect) {
    auto cfg = solveResolution(1.0, 50.0, 20.0, {20, 20, 10});
    auto billet = buildBillet(cfg, {0,0,0}, {20, 20, 10});
    auto accessor = billet.sdfGrid->getConstAccessor();
    auto& xform = billet.sdfGrid->transform();

    auto idxInside = xform.worldToIndexCellCentered({10, 10, 5});
    EXPECT_LT(accessor.getValue(idxInside), 0.0);

    auto idxOutside = xform.worldToIndexCellCentered({-5, 10, 5});
    EXPECT_GT(accessor.getValue(idxOutside), 0.0);
}

TEST(BilletBuilder, SingleTrack_SerializeDeserialize) {
    auto cfg = solveResolution(1.0, 50.0, 20.0, {20, 20, 10});
    auto billet = buildBillet(cfg, {0,0,0}, {20, 20, 10});

    openvdb::io::File file("/tmp/test_billet.vdb");
    file.write({billet.sdfGrid});
    file.close();

    file.open();
    auto grids = file.getGrids();
    EXPECT_EQ(grids->size(), 1);
    file.close();
    std::remove("/tmp/test_billet.vdb");
}
```

---

### Sprint 3：R3 刀具扫掠体 SDF

| Task | 描述 | 测试先行 | 产出 |
|------|------|----------|------|
| T3.1 | ToolSweepSDF — 球头刀胶囊体 eval | 先写测试 | `core/ToolSweepSDF.h/.cpp` |
| T3.2 | ToolSweepSDF — 平底刀扁平胶囊体 eval | 先写测试 | 扩展 T3.1 |
| T3.3 | ToolSweepSDF — 包围盒计算 | 先写测试 | 扩展 T3.1 |
| T3.4 | ToolSweepSDF — gradient 实现 | 先写测试 | 扩展 T3.1 |
| T3.5 | ToolSweepSDF — 光栅化为 FloatGrid | 先写测试 | 扩展 T3.1 |

#### T3.1 测试用例（红）

```cpp
TEST(ToolSweepSDF, BallEnd_StaticSphere) {
    ToolSweepSDF tool(BALL_END, 5.0, 0, 20, {0,0,0}, {0,0,0});
    EXPECT_NEAR(tool.eval({5, 0, 0}), 0.0, 1e-10);
    EXPECT_NEAR(tool.eval({0, 5, 0}), 0.0, 1e-10);
    EXPECT_LT(tool.eval({0, 0, 0}), 0.0);
    EXPECT_GT(tool.eval({10, 0, 0}), 0.0);
}

TEST(ToolSweepSDF, BallEnd_LinearSweep_Capsule) {
    ToolSweepSDF tool(BALL_END, 5.0, 0, 20, {0,0,0}, {20,0,0});
    EXPECT_NEAR(tool.eval({10, 5, 0}), 0.0, 1e-10);
    EXPECT_NEAR(tool.eval({-5, 0, 0}), 0.0, 1e-10);
    EXPECT_NEAR(tool.eval({25, 0, 0}), 0.0, 1e-10);
    EXPECT_LT(tool.eval({10, 0, 0}), 0.0);
}

TEST(ToolSweepSDF, BallEnd_BoundingBox) {
    ToolSweepSDF tool(BALL_END, 5.0, 0, 20, {0,0,0}, {20,0,0});
    auto bbox = tool.getBoundingBox();
    EXPECT_LE(bbox.min().x(), -5.0);
    EXPECT_GE(bbox.max().x(), 25.0);
}

TEST(ToolSweepSDF, Gradient_FiniteDifference) {
    ToolSweepSDF tool(BALL_END, 5.0, 0, 20, {0,0,0}, {20,0,0});
    auto grad = tool.gradient({10, 5, 0});
    auto norm = grad.unit();
    EXPECT_NEAR(norm.y(), 1.0, 0.01);
}
```

---

### Sprint 4：R4 切削引擎

| Task | 描述 | 测试先行 | 产出 |
|------|------|----------|------|
| T4.1 | CuttingEngine — 单轨 CSG 切削 | 先写测试 | `core/CuttingEngine.h/.cpp` |
| T4.2 | CuttingEngine — 切削后 SDF 符号验证 | 先写测试 | 扩展 T4.1 |
| T4.3 | CuttingEngine — 切削体积精度验证 | 先写测试 | 扩展 T4.1 |
| T4.4 | CuttingEngine — 多段连续切削 | 先写测试 | 扩展 T4.1 |
| T4.5 | CuttingEngine — 性能基准记录 | 先写测试 | `tests/bench_cutting.cpp` |

#### T4.1 测试用例（红）

```cpp
TEST(CuttingEngine, SingleTrack_BasicCut) {
    auto cfg = solveResolution(1.0, 50.0, 20.0, {20, 20, 10});
    auto billet = buildBillet(cfg, {0,0,0}, {20, 20, 10});
    ToolSweepSDF tool(BALL_END, 3.0, 0, 15, {5,10,0}, {15,10,0});

    CuttingEngine engine;
    engine.cut(billet, tool);

    auto accessor = billet.sdfGrid->getConstAccessor();
    auto& xform = billet.sdfGrid->transform();
    auto idx = xform.worldToIndexCellCentered({10, 10, 0});
    EXPECT_GT(accessor.getValue(idx), 0.0);
}

TEST(CuttingEngine, SingleTrack_UncutRegionUnchanged) {
    auto cfg = solveResolution(1.0, 50.0, 20.0, {20, 20, 10});
    auto billet = buildBillet(cfg, {0,0,0}, {20, 20, 10});
    auto& xform = billet.sdfGrid->transform();
    auto farIdx = xform.worldToIndexCellCentered({2, 2, 5});
    float before = billet.sdfGrid->getConstAccessor().getValue(farIdx);

    ToolSweepSDF tool(BALL_END, 3.0, 0, 15, {15,15,0}, {18,15,0});
    CuttingEngine engine;
    engine.cut(billet, tool);

    float after = billet.sdfGrid->getConstAccessor().getValue(farIdx);
    EXPECT_FLOAT_EQ(before, after);
}

TEST(CuttingEngine, SingleTrack_VolumeAccuracy) {
    auto cfg = solveResolution(0.5, 50.0, 20.0, {20, 20, 10});
    auto billet = buildBillet(cfg, {0,0,0}, {20, 20, 10});

    ToolSweepSDF tool(BALL_END, 2.0, 0, 15, {5,10,0}, {15,10,0});
    double theoreticalVolume = (4.0/3.0)*M_PI*8.0 + M_PI*4.0*10.0;

    double volBefore = computeVolume(billet.sdfGrid);
    CuttingEngine engine;
    engine.cut(billet, tool);
    double volAfter = computeVolume(billet.sdfGrid);
    double removedVolume = volBefore - volAfter;

    double surfaceArea = 2*M_PI*2.0*10.0 + 4*M_PI*4.0;
    double maxError = surfaceArea * cfg.d_v;
    EXPECT_NEAR(removedVolume, theoreticalVolume, maxError);
}

TEST(CuttingEngine, SingleTrack_MultiSegment) {
    auto cfg = solveResolution(0.5, 50.0, 20.0, {20, 20, 10});
    auto billet = buildBillet(cfg, {0,0,0}, {20, 20, 10});

    std::vector<ToolSweepSDF> segs = {
        {BALL_END, 2.0, 0, 15, {2,10,0}, {7,10,0}},
        {BALL_END, 2.0, 0, 15, {7,10,0}, {12,10,0}},
        {BALL_END, 2.0, 0, 15, {12,10,0}, {17,10,0}},
    };
    CuttingEngine engine;
    for (auto& seg : segs) engine.cut(billet, seg);

    auto billet2 = buildBillet(cfg, {0,0,0}, {20, 20, 10});
    ToolSweepSDF longSeg(BALL_END, 2.0, 0, 15, {2,10,0}, {17,10,0});
    engine.cut(billet2, longSeg);

    double vol1 = computeVolume(billet.sdfGrid);
    double vol2 = computeVolume(billet2.sdfGrid);
    EXPECT_NEAR(vol1, vol2, cfg.d_v * 100);
}
```

#### T4.5 性能基准

```cpp
TEST(CuttingBench, RecordPerformance) {
    CutPerformanceRecord record;
    auto cfg = solveResolution(0.1, 5.0, 1.0, {50, 50, 50});
    auto billet = buildBillet(cfg, {0,0,0}, {50, 50, 50});

    record.voxelSize = cfg.d_v;
    record.method = "CSG";
    record.toolType = "BALL";
    record.toolRadius = 5.0;
    record.segmentLength = 10.0;
    record.billetActiveVoxels = billet.sdfGrid->activeVoxelCount();

    ToolSweepSDF tool(BALL_END, 5.0, 0, 20, {10,25,0}, {20,25,0});

    auto t0 = std::chrono::high_resolution_clock::now();
    CuttingEngine engine;
    engine.cut(billet, tool);
    auto t1 = std::chrono::high_resolution_clock::now();

    record.time_total = std::chrono::duration<double, std::milli>(t1-t0).count();
    appendToCSV("benchmark_results.csv", record);
    EXPECT_LT(record.time_total, 5000.0);
}
```

---

### Sprint 5：端到端集成 + 可视化

| Task | 描述 | 测试先行 | 产出 |
|------|------|----------|------|
| T5.1 | 端到端集成测试：R1→R2→R3→R4 | 先写测试 | `tests/test_e2e.cpp` |
| T5.2 | 输出 .vdb 可视化验证 | 手动验证 | `examples/demo_cut.cpp` |
| T5.3 | 性能基准矩阵运行 | 自动化 | `bench/run_benchmarks.cpp` |

#### T5.1 端到端测试

```cpp
TEST(EndToEnd, FullPipeline_SingleTrack) {
    auto cfg = solveResolution(0.5, 10.0, 2.0, {30, 30, 15});
    ASSERT_EQ(cfg.mode, ResolutionConfig::SINGLE_TRACK);

    auto billet = buildBillet(cfg, {0,0,0}, {30, 30, 15});
    ASSERT_NE(billet.sdfGrid, nullptr);
    double volOriginal = computeVolume(billet.sdfGrid);

    ToolSweepSDF tool(BALL_END, 3.0, 0, 20, {5,15,0}, {25,15,0});
    CuttingEngine engine;
    engine.cut(billet, tool);

    double volAfter = computeVolume(billet.sdfGrid);
    EXPECT_LT(volAfter, volOriginal);
    EXPECT_GT(volAfter, 0.0);

    openvdb::io::File file("/tmp/e2e_result.vdb");
    file.write({billet.sdfGrid});
    file.close();
}
```

---

## 3. 执行顺序与依赖

```
T1.1 → T1.2 → T1.3 → T1.4
                 ↓
T2.1 → T2.2 → T2.3 → T2.4(可选)
                 ↓
T3.1 → T3.2 → T3.3 → T3.4 → T3.5
                              ↓
T4.1 → T4.2 → T4.3 → T4.4 → T4.5
                              ↓
T5.1 → T5.2 → T5.3
```

Sprint 2 和 Sprint 3 可并行开发。

---

## 4. 辅助工具函数

```cpp
double computeVolume(const openvdb::FloatGrid::Ptr& grid) {
    double voxelVol = std::pow(grid->voxelSize()[0], 3);
    size_t count = 0;
    for (auto iter = grid->cbeginValueOn(); iter; ++iter) {
        if (*iter < 0.0f) ++count;
    }
    return count * voxelVol;
}
```

---

## 5. TDD 节奏规范

每个 Task 的 commit 序列：

```
1. "test(R1): add failing tests for SingleTrack resolution"  [RED]
2. "feat(R1): implement solveResolution minimal"             [GREEN]
3. "refactor(R1): extract helper, add edge cases"            [REFACTOR]
```

---

## 修订记录

| 版本 | 日期 | 修订内容 |
|------|------|----------|
| v0.a | 2026-06-01 | 初稿创建 |
