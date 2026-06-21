# ModuleWorks vs 2D Region Cut Simulation API 功能与命名对照表

> 日期：2025-05-25  
> 目的：逐功能对比两套 API 的命名、签名风格和覆盖范围

---

## 1. 创建与生命周期

| 功能 | ModuleWorks | 本设计 | 备注 |
|------|-------------|--------|------|
| 创建仿真实例 | `mwMachSimVerifier::Create()` | `Simulator::create(config)` | MW 无参创建，配置后置；本设计配置前置不可变 |
| 创建刀轨计算实例 | `mwGeoLib(Units)` | *不适用（本设计仅覆盖仿真）* | — |
| 仿真结束 | `verifier->OnSimulationEnded()` | `simulator->finish()` | 语义相同，命名更简洁 |
| 仿真暂停 | `verifier->OnSimulationStopped()` | `simulator->pause()` | — |
| 完全重置 | *无显式 Reset（需重建实例）* | `simulator->reset()` | 本设计支持原地重置 |
| 析构 | `~mwMachSimVerifier()` | `~Simulator()` | — |

---

## 2. 精度与数据模型配置

| 功能 | ModuleWorks | 本设计 | 备注 |
|------|-------------|--------|------|
| 设置精度 | `SetPrecision(float)` | `SimConfig::precision` | MW 运行时可变；本设计创建时确定 |
| 计算推荐精度 | `CalculatePrecision(sampleCount, bbox)` | `SimConfig::recommendedPrecision(bbox, count)` | 静态辅助方法 |
| 获取当前精度 | `GetPrecision()` / `GetCurrentStockPrecision()` | `config.precision`（不可变） | 本设计无二义性 |
| 选择数据模型 | `ForceDataModel(MWV_FM_FIELD / MWV_FM_DEXELBLOCK)` | *内部自动选择* | 本设计隐藏实现细节 |
| 网格质量 | `SetMeshQuality(int)` / `SetMeshExportQuality(int)` | *不适用（2D 无网格）* | 3D 扩展时再引入 |

---

## 3. 毛坯/工件管理

| 功能 | ModuleWorks | 本设计 | 备注 |
|------|-------------|--------|------|
| 网格毛坯 | `SetMesh(MeshPtr, tolerance)` | `StockDef::fromRegion(region, units)` | 2D 用 Region 替代 Mesh |
| 多网格联合 | `SetMeshes(vector<MeshPtr>, tolerance)` | `StockDef::fromRegion(Region::boolean_union(...))` | 布尔运算前置 |
| 长方体毛坯 | `SetStockCube(min, max)` | `StockDef::rect(min, max, units)` | — |
| 圆柱体毛坯 | `SetStockCylinder(h, r, pos, axis)` | `StockDef::circle(center, radius, units)` | 2D 简化为圆 |
| 回转体毛坯 | `SetStockRevolved(profile, axis)` | `StockDef::fromContour(outer, units)` | 2D 直接用轮廓 |
| 空白空间（增材） | `SetStockEmptyCube(min, max)` | *暂不支持* | 增材场景后续扩展 |
| 通用接口 | `SetStock(mwvInitStockGeometry&)` | `simulator->setStock(StockDef)` | 统一入口 |
| 检查是否有毛坯 | `HasStock()` | `simulator->hasStock()` | — |
| 清除毛坯 | `ClearStock()` | `simulator->clearStock()` | — |
| 当前包围盒 | `GetCurrentStockBoundingBox()` | `simulator->stockBounds()` | — |
| 世界包围盒 | `GetWorldBoundingBox()` | `simulator->worldBounds()` | — |
| 验证毛坯尺寸 | `CanSetStock(min, max)` | *构造时校验，异常抛出* | 本设计 fail-fast |
| 重新仿真毛坯 | `ResimulateStock()` | *不需要（精度不可变）* | — |
| 修复毛坯 | `RepairStock()` | *不需要（2D 区域始终一致）* | — |
| 毛坯验证开关 | `SetCheckStock(bool)` | *始终校验* | — |

---

## 4. 布尔操作

| 功能 | ModuleWorks | 本设计 | 备注 |
|------|-------------|--------|------|
| 添加材料 | `BooleanUnify(mesh)` | `simulator->booleanAdd(region)` | — |
| 减去材料 | `BooleanSubtract(mesh)` | `simulator->booleanSubtract(region)` | — |
| 相交 | `BooleanIntersect(mesh)` | `simulator->booleanIntersect(region)` | — |
| 回转体布尔减 | `BooleanSubtract(contour, axis)` | *用 Region 直接表达* | 2D 无需特殊处理 |

---

## 5. 刀具定义

| 功能 | ModuleWorks | 本设计 | 备注 |
|------|-------------|--------|------|
| 球头铣刀 | `mwSphereMill(d, holder, shoulder, flute, arbor, units)` | `Tool::create(ToolProfile::ballmill(d), units)` | 本设计分离几何与组件 |
| 平底铣刀 | `mwEndMill(...)` | `Tool::create(ToolProfile::endmill(d), units)` | — |
| 圆角铣刀 | `mwBullMill(...)` | `Tool::create(ToolProfile::bullmill(d, r), units)` | — |
| 锥形铣刀 | `mwTaperMill(...)` | `Tool::create(ToolProfile::vmill(d, angle), units)` | — |
| 自定义截面 | `mwGenericTool(...)` | `Tool::create(ToolProfile::custom(contour), units)` | — |
| 完整组件（含刀柄） | 构造函数参数 `holder + arbor + shoulder` | `Tool::create(ToolAssembly{...}, units)` | 本设计可选组件 |
| 设置单刀具 | `SetTool(toolPtr, toolId)` | `simulator->setTool(tool)` | ID 自动分配 |
| 设置多刀具 | `SetTools(vector<SetToolParameters>)` | `simulator->setTools(span<Tool>)` | — |
| 添加刀具 | `AddTool(toolPtr, toolId)` | *setTools 覆盖* | 本设计简化 |
| 切换活跃刀具 | `SetCurrentCutTool(index)` | `simulator->setActiveTool(toolId)` | 用 ID 而非索引 |
| 获取当前刀具 | `GetCurrentCutTool()` | `simulator->activeTool()` | — |
| 刀具颜色 | `SetToolColor(...)` | *不在仿真 API 中（渲染层职责）* | 关注点分离 |
| 刀具可见性 | `SetToolVisibility(bool)` | *不在仿真 API 中* | — |

---

## 6. 刀具行为

| 功能 | ModuleWorks | 本设计 | 备注 |
|------|-------------|--------|------|
| 设置行为 | `SetToolBehavior(mwvToolBehavior)` | `simulator->setToolBehavior({cuts, collisionChecked, rapid})` | 本设计用 POD 结构体 |
| 快速移动模式 | `SetRapidMode(bool)` | `ToolBehavior::rapid = true` | 统一到行为结构体 |
| 切削部件选择 | `SetCuttingPart(edge, shaft, arbor, holder)` | `ToolBehavior::cuts` + `collisionChecked` | 简化为两个布尔 |
| 碰撞模式 | `SetMoveModes(edge, ncedge, arbor, holder)` | `ToolBehavior::collisionChecked` | 2D 简化 |

---

## 7. 容差配置

| 功能 | ModuleWorks | 本设计 | 备注 |
|------|-------------|--------|------|
| 切削容差 | `SetCuttingTolerance(float)` | `Tolerances::cutting` | 创建时确定 |
| 碰撞容差 | `SetCollisionTolerance(float)` | `Tolerances::collision` | — |
| 刃部碰撞容差 | `SetFluteCollisionTolerance(float)` | *统一为 collision* | 2D 简化 |
| 刀杆碰撞容差 | `SetShaftCollisionTolerance(float)` | *统一* | — |
| 刀柄碰撞容差 | `SetHolderCollisionTolerance(float)` | *统一* | — |
| 快速移动碰撞容差 | `SetRapidCollisionTolerance(float)` | *统一* | — |
| 快速加速容差 | `SetRapidAccelerationTolerance(float)` | *不需要（2D 无加速相位）* | — |
| 安全距离 | `SetSafetyDistance(float)` | *可通过 collision 容差覆盖* | — |
| 各部件安全距离 | `SetFlute/Shaft/Arbor/HolderSafetyDistance()` | *统一* | — |
| 薄壁容差 | `SetThinVolumeTolerance(float)` | *暂不支持* | 后续按需添加 |
| 过切阈值 | `SetGougeThreshold(float)` | `Tolerances::gouge` | — |
| 欠切阈值 | `SetExcessThreshold(float)` | `Tolerances::excess` | — |

---

## 8. 运动/切削模拟

| 功能 | ModuleWorks | 本设计 | 备注 |
|------|-------------|--------|------|
| 即时线性切削 | `Cut(startFrame, endFrame)` | `simulator->cut(start, end)` | — |
| 即时圆弧切削 | `CircularCut(start, end, center, normal)` | `simulator->arcCut(start, end, center, ccw)` | 2D 用 ccw 替代法线 |
| 即时角度圆弧 | `CircularSweepCut(start, angle, center, normal, h, r)` | `ArcMove(start, center, sweep_angle)` | — |
| 缓冲线性 | `BufferedCut(start, end)` | `simulator->buffer(LinearMove{...})` | — |
| 缓冲圆弧 | `BufferedCircularCut(start, end, center, normal)` | `simulator->buffer(ArcMove{...})` | — |
| 缓冲角度圆弧 | `BufferedCircularSweepCut(...)` | `simulator->buffer(ArcMove{start, center, angle})` | — |
| 批量执行 | `SimulateBufferedCuts(results)` | `simulator->simulate()` → `SimResult` | 本设计返回值而非输出参数 |
| 丢弃缓冲 | `DiscardBufferedCuts()` | `simulator->discardBuffer()` | — |
| 序列批量缓冲 | *逐个 BufferedCut* | `simulator->bufferSequence(MoveSequence)` | 本设计支持批量提交 |
| 螺纹切削 | `ThreadCut(start, end, pitch, offset, helix)` | *不适用（2D）* | 3D 扩展时添加 |
| 线切割 | `SetWirePlaneInterpolation(...)` | *不适用（2D）* | — |
| 多体协同运动 | `SimulateMotion(mwMotions, options, results)` | *暂不支持* | 多刀具场景后续扩展 |

---

## 9. 运动属性

| 功能 | ModuleWorks | 本设计 | 备注 |
|------|-------------|--------|------|
| 运动 ID | `SetMoveID(float)` | `simulator->setMoveId(int)` | 本设计用 int（更自然） |
| 操作 ID | `SetOperation(int)` | `simulator->setOperationId(int)` | — |
| 运动描述 | `SetMoveDescription(string)` | *不支持（轻量化）* | 调试信息由调用方管理 |
| 进给率 | *隐含在 CNCMove 中* | `simulator->setFeedRate(double)` | 显式设置 |
| 获取已仿真属性 | `GetSimulatedMoveParameters(vector&)` | `SimResult::perMove[i].moveId` | 结果中直接携带 |

---

## 10. 结果获取

| 功能 | ModuleWorks | 本设计 | 备注 |
|------|-------------|--------|------|
| 即时碰撞码 | `GetVerificationResult()` → `unsigned int` | `CutResult::code` | — |
| 详细验证结果 | `GetVerificationResult(VerificationResult&)` | `CutResult` 结构体 | 本设计一次返回全部 |
| 批量结果 | `SimulateBufferedCuts(vector<VerificationResult>&)` | `SimResult simulate()` | 返回值 vs 输出参数 |
| 材料去除标志 | `VerificationResult::DidRemoveMaterial` | `CutResult::materialRemoved` | — |
| 刀具 ID | `VerificationResult::ToolId` | `CutResult::toolId` | — |
| 运动 ID | `VerificationResult::MoveID` (float) | `CutResult::moveId` (int) | — |
| 强制正确材料标志 | `ForceCorrectMaterialRemovalFlag(bool)` | *始终正确* | 本设计不做此优化妥协 |

---

## 11. 碰撞码

| ModuleWorks | 本设计 | 含义 |
|-------------|--------|------|
| `VC_NO_ERROR` (0x0) | `CollisionCode::None` (0x0) | 无碰撞 |
| `VC_RAPIDEDGE_CRASH` (0x1) | `CollisionCode::RapidCrash` (0x10) | 快速移动碰撞 |
| `VC_NCEDGE_CRASH` (0x2) | `CollisionCode::FluteCrash` (0x1) | 刃部碰撞 |
| `VC_ARBOR_CRASH` (0x4) | `CollisionCode::ArborCrash` (0x20) | 刀杆碰撞 |
| `VC_HOLDER_CRASH` (0x8) | `CollisionCode::HolderCrash` (0x4) | 刀柄碰撞 |
| `VC_FLUTE_SAFETY_DISTANCE_CRASH` (0x10) | `CollisionCode::SafetyViolation` (0x8) | 安全距离违反 |
| `VC_SHAFT_SAFETY_DISTANCE_CRASH` (0x20) | `CollisionCode::SafetyViolation` (0x8) | 合并为统一安全违反 |
| `VC_ARBOR_SAFETY_DISTANCE_CRASH` (0x40) | `CollisionCode::SafetyViolation` (0x8) | — |
| `VC_HOLDER_SAFETY_DISTANCE_CRASH` (0x80) | `CollisionCode::SafetyViolation` (0x8) | — |

---

## 12. 目标形状与偏差分析

| 功能 | ModuleWorks | 本设计 | 备注 |
|------|-------------|--------|------|
| 设置目标 | `SetTarget(MeshPtr, tryClose)` | `simulator->setTarget(Region)` | 2D 用 Region |
| 多目标 | `SetTargets(vector<TargetData>, overlap)` | *单目标（2D 足够）* | 后续可扩展 |
| 清除目标 | `ClearTargets()` | `simulator->clearTarget()` | — |
| 过切列表 | `GetGougeList()` → `vector<GougeReport>` | `simulator->gougeList(maxCount)` | — |
| 欠切列表 | `GetExcessList()` → `vector<GougeReport>` | `simulator->excessList(maxCount)` | — |
| 偏差偏移 | `SetDeviationOffset(float)` | *不支持（直接修改目标）* | 更直观 |
| 报告排序模式 | `SetGougeReportMode(INCREASING/DECREASING/ALL)` | *默认按偏差降序* | 简化 |
| 最大报告数 | `SetMaxGougeExcessReportSize(int)` | `gougeList(maxCount)` 参数 | — |
| Z 方向过切 | `EnableZGouge(bool)` | *不适用（2D）* | — |
| 计算偏差 | `CalculateGougeExcess()` | *自动按需计算* | 惰性求值 |
| 目标可见性 | `SetTargetVisibility(bool)` | *不在仿真 API 中* | 渲染层职责 |
| 目标颜色 | `SetTargetColor(...)` | *不在仿真 API 中* | — |
| 偏差统计 | *无直接 API* | `simulator->deviationStats()` | 本设计新增 |

---

## 13. 分块检测

| 功能 | ModuleWorks | 本设计 | 备注 |
|------|-------------|--------|------|
| 标记分块 | `MarkChunks()` | `simulator->detectChunks()` | 返回完整 Chunk 信息 |
| 获取分块 ID | `GetChunks(vector<int>&)` | `detectChunks()` 返回值含 ID | — |
| 分块数量 | `GetNumberOfChunks()` | `detectChunks().size()` | — |
| 查找位置处分块 | `FindChunksAtPositions(positions, tol, out)` | *通过 Chunk::region.contains() 实现* | 调用方自行判断 |
| 删除分块 | `DeleteChunk(id)` | `simulator->deleteChunk(id)` | — |
| 批量删除 | `DeleteChunks(vector<int>)` | *循环调用 deleteChunk* | 简化 |
| 删除非接触块 | `DeleteChunksNotTouching(points, tol)` | `simulator->deleteChunksNotTouching(points, tol)` | — |
| 删除接触块 | `DeleteChunksTouching(points, tol)` | *不支持（少见场景）* | — |
| 保留最大块 | *需手动实现* | `simulator->keepLargestChunk()` | 本设计新增便捷方法 |
| 增量检测 | `EnableIncrementalChunkDetection(handler)` | `Observer::onChunksChanged()` | 统一到观察者 |
| 分块网格导出 | `GetChunkMesh(id)` | `Chunk::region` | 2D 直接返回 Region |

---

## 14. 工件导出

| 功能 | ModuleWorks | 本设计 | 备注 |
|------|-------------|--------|------|
| 完整网格 | `GetMesh(filename, tol, colors)` | `simulator->exportRegion()` | 2D 返回 Region |
| 局部网格（包围盒） | `GetMeshInBoundingBox(min, max)` | `simulator->exportRegionInBox(bbox)` | — |
| 局部网格（Z 以上） | `GetMeshAboveZLevel(z)` | *不适用（2D）* | — |
| 局部网格（平面间） | `GetMeshBetweenPlanes(normal, o1, o2)` | *不适用（2D）* | — |
| 导出到文件 | `ExportStockMesh(filename, format, options)` | `simulator->exportToSVG(filename)` | 2D 用 SVG |
| 原始毛坯网格 | `GetOriginalStockMesh()` | *通过 StockDef 保留* | — |
| 轮廓导出 | *无直接 API* | `simulator->exportContours()` | 本设计新增 |
| 面积查询 | *无直接 API* | `simulator->stockArea()` | 本设计新增 |

---

## 15. 回调与观察者

| 功能 | ModuleWorks | 本设计 | 备注 |
|------|-------------|--------|------|
| 进度回调 | `SetProgressHandler(mwMachSimVerifierProgressHandler*)` | `simulator->setProgressCallback(function<bool(float)>)` | lambda 替代抽象类 |
| 进度任务类型 | `SetJob(JobType)` | *不暴露（内部细节）* | — |
| 进度步数 | `SetStepCount(ulong)` | *不暴露* | — |
| 进度位置 | `SetPos(ulong)` → `ulong` | 回调参数 `float percent` | 归一化百分比 |
| 用户取消 | `IsAborted()` → `bool` | 回调返回 `false` 表示取消 | 更自然 |
| 运动完成回调 | `RegisterMoveFinishedHandler(handler*)` | `Observer::onMoveSimulated(result)` | 统一观察者 |
| 注销回调 | `UnregisterMoveFinishedHandler(handler*)` | `simulator->removeObserver(obs)` | — |
| 通知处理 | `SetNotificationHandler(handler)` | *不需要（异常/日志替代）* | — |
| 分块变化回调 | `EnableIncrementalChunkDetection(handler)` | `Observer::onChunksChanged(ids)` | — |
| 毛坯变化 | *无* | `Observer::onStockChanged()` | 本设计新增 |

---

## 16. 车削/主轴相关（2D 不适用）

| 功能 | ModuleWorks | 本设计 | 备注 |
|------|-------------|--------|------|
| 主轴模式 | `SetStockSpindleMode(ON/OFF/AUTO)` | *不适用* | 3D 扩展时考虑 |
| 主轴轴向 | `SetStockSpindleAxis(axis, position)` | *不适用* | — |
| 回转对称检测 | `GetStockIsRotationalSymmetric()` | *不适用* | — |
| 回转轮廓 | `GetRotationalProfileLinearized()` | *不适用* | — |

---

## 17. 历史与存储

| 功能 | ModuleWorks | 本设计 | 备注 |
|------|-------------|--------|------|
| 禁用切削历史 | `SetNoCutHistory(bool)` | *默认无历史（2D 区域即时更新）* | 本设计无此需求 |
| 保存仿真 | `SaveSimulation(...)` | *序列化 Region + 配置（后续）* | — |
| 加载仿真 | `LoadSimulation(...)` | *反序列化（后续）* | — |
| 恢复点 | `CreateRestorePoint() / RestoreToPoint()` | *通过 Region 快照实现* | — |
| GPU 模式 | `SetGpuMode(off/gpuOnly/preferGpu)` | *内部实现细节* | — |

---

## 18. 命名风格对比

| 维度 | ModuleWorks 风格 | 本设计风格 |
|------|-----------------|-----------|
| 类前缀 | `mw` / `mwv` / `MW_` | 无前缀，`sim::` 命名空间 |
| 方法命名 | `PascalCase`（`SetPrecision`, `GetMesh`） | `camelCase`（`setStock`, `exportRegion`） |
| 枚举命名 | `SCREAMING_CASE`（`MWV_FM_DEXELBLOCK`） | `PascalCase`（`Units::Millimeter`） |
| 常量命名 | `VC_NO_ERROR` | `CollisionCode::None` |
| 类型后缀 | `Ptr`（`MeshPtr`, `ToolPtr`） | 无后缀（值语义或 `unique_ptr`） |
| 参数传递 | 输出参数（`GetXxx(T& out)`） | 返回值（`T getXxx()`） |
| 配置方式 | 运行时 Set/Get 对 | 不可变 Config 对象 |
| 回调 | 抽象类指针 | `std::function` / Observer 接口 |
| 智能指针 | `misc::mwAutoPointer<T>` | `std::unique_ptr<T>` |
| 容器 | `std::vector<T>` + 输出参数 | `std::span<const T>` + 返回值 |

---

## 19. 功能覆盖度总结

| 功能域 | MW 方法数 | 本设计方法数 | 覆盖率 | 说明 |
|--------|----------|-------------|--------|------|
| 会话生命周期 | 3 | 4 | 100% | 新增 reset |
| 精度配置 | 6 | 2 | 100% | 合并为 Config |
| 毛坯管理 | 15 | 8 | 90% | 去掉修复/验证开关 |
| 布尔操作 | 5 | 3 | 100% | 去掉回转体特化 |
| 刀具定义 | 10+ 类 | 3 类 | 100% | 工厂方法统一 |
| 刀具管理 | 8 | 4 | 95% | 去掉颜色/可见性 |
| 刀具行为 | 6 | 1 | 100% | 合并为结构体 |
| 容差配置 | 12 | 1 结构体 | 85% | 合并，2D 简化 |
| 运动模拟 | 12 | 7 | 90% | 去掉螺纹/线切割 |
| 运动属性 | 5 | 3 | 80% | 去掉描述字段 |
| 结果获取 | 6 | 3 | 100% | 合并为返回值 |
| 偏差分析 | 12 | 5 | 85% | 去掉多目标/Z过切 |
| 分块检测 | 10 | 5 | 80% | 去掉接触删除 |
| 工件导出 | 8 | 5 | 90% | 2D 适配 |
| 回调/观察 | 6 | 4 | 100% | 统一观察者 |
| 车削/主轴 | 8 | 0 | 0% | 2D 不适用 |
| **总计** | **~130** | **~55** | **~85%** | API 表面积减少 58% |

---

*结论：本设计以约 55 个公开方法覆盖了 ModuleWorks ~130 个方法中 85% 的功能，API 表面积减少 58%。未覆盖的 15% 主要是车削/主轴（2D 不适用）和部件级容差细分（2D 简化）。*
