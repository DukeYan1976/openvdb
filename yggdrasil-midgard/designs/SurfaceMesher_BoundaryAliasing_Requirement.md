# SurfaceMesher 曲面边界走样改进需求

**日期**: 2026-07-19  
**模块**: `core/SurfaceMesher.h/cpp` / `app/windows/MicroGridLabWindow.cpp`  
**状态**: 已分析，待实现  
**优先级**: 中  

---

## 1. 问题现象

由 `SurfaceMesher` 从 `surfacePoints` 重建的三角形网格，在**曲面边界**处出现明显锯齿/走样（aliasing），而**平面切削边界**精度良好。见 `build/mesh_clip2.png`。

---

## 2. 根本原因

当前实现采用 **均匀体素网格 + 标准 Marching Cubes**，存在以下固有离散误差：

1. **体素网格与曲面边界不共面**
   - MC 顶点只能落在体素棱边上。
   - 平面边界（与 cube 面或直纹面平行）恰好与体素面对齐，离散不明显。
   - 曲面边界（球头/牛鼻刀圆弧与 cube 面相交）斜切体素，只能在体素棱角处“楼梯状”逼近真实交线。

2. **活跃体素区域为轴对齐盒状扩展**
   - 当前策略：点所在体素 + 26-邻域。
   - 曲面边界处表面点骤然结束，活跃体素外轮廓为方块堆砌，MC 只能在这些方块棱角上生成边界三角形。

3. **支撑半径 h 不改变边界精度**
   - `h = max(3·avgSpacing, 4·voxelSize)` 只影响隐式场光滑度与内部面片质量。
   - 边界精度由 `voxelSize = extent / resolution` 决定。

---

## 3. 影响范围

- 视觉渲染：曲面边界出现锯齿，影响观感。
- 下游测量/分析：若用 mesh 做距离、体积或曲率计算，边界误差会引入系统误差。
- 当前不是功能性 bug，而是重建精度问题。

---

## 4. 可选改进方案

| 方案 | 描述 | 预期效果 | 代价/风险 |
|------|------|----------|-----------|
| A. 提高 `resolution` 可配置 | 将默认 128 改为可配置（如 256/512），或在 UI 中暴露分辨率滑块。 | 线性减小锯齿，实现最简单。 | 内存与时间按 `resolution³` 增长；10k 点可能从 ~10ms 增至数百 ms。 |
| B. 精确裁剪到 cube 边界 | MC 重建后，用 cube 的 6 个面（`[0, cubeSize]³`）对 mesh 做精确平面裁剪。 | 外轮廓与 cube 边界完全对齐，消除 cube 相交处的锯齿。 | 需实现稳健的多边形裁剪，可能产生狭长三角形，需后续网格清理。 |
| C. 使用已知刀具扫掠面约束边界 | 利用 `ToolSweepSurface` 解析表达式，将 cube 边界处的 mesh 顶点投影到真实刀具-立方体交线上。 | 边界精度最高，可逼近计算精度。 | 破坏 `SurfaceMesher` 的模块独立性；需耦合刀具几何与 cutHistory。 |
| D. 自适应 Marching Cubes / Dual Contouring | 在边界/高曲率区域局部细分体素，平坦区域保持粗网格。 | 在相同内存下获得更高边界精度。 | 实现复杂度高，需重写稀疏体素与 MC 流程。 |

---

## 5. 建议实现路径

**短期（低投入）**：
- 方案 A：在 `buildSurfaceMesh` 中保持 `resolution` 参数，默认 128；在 UI 中增加 `Mesh Resolution` 输入框，允许用户临时提高到 256/512。
- 方案 B：实现一个独立的 `clipTriMeshByAxisAlignedBox(const TriMesh&, const Vec3d& min, const Vec3d& max)` 工具函数，在 `Output Mesh` 导出前对 mesh 做精确裁剪。

**长期（高投入）**：
- 方案 D：若后续对边界精度有硬性要求，可考虑自适应体素或 Dual Contouring。

---

## 6. 验收标准

- 在相同 surface points 下，曲面与 cube 相交处的边界锯齿肉眼可见减小。
- 单元测试通过（`tests/test_surfacemesher.cpp` 中的平面/球面测试）。
- 不破坏 `SurfaceMesher` 的零外部依赖设计（方案 A/B 满足；方案 C 不满足）。

---

## 7. 相关文件

- `core/SurfaceMesher.cpp` — MLS + 稀疏体素 + Marching Cubes 实现。
- `app/windows/MicroGridLabWindow.cpp` — `Cut Surface Mesh` / `Output Mesh` UI 入口。
- `tests/test_surfacemesher.cpp` — 重建质量单元测试。
- `build/mesh_clip2.png` — 问题现象截图。
