# Plan: IPW0 显示 + Debug 集成 (v2)

## 目标
1. 毛坯切换时自动构建 IPWState，使用 ipw.config 已有的 ToleranceConfig
2. Debug 窗口增加 BilletBuilderTest / Clear 按钮

## Step 1: IPWBuilder 集成到主循环
- `main.cpp` 渲染循环中，`st.tick()` 之前检查 `st.ipwDirty`
- 若 dirty，`IPWBuilder::build(st.ipw.billetDef, st.ipw.config)` → `st.ipw`
- 清 dirty 标志
- `main.cpp` + `#include "core/IPWBuilder.h"`

## Step 2: Debug 按钮 + 显示
- `DebugWindow.h`: + `drawIPWTest()`
- `DebugWindow.cpp`:
  - `"Billet Builder Test"` → 遍历 `ipw.microGrid->tree().cbeginLeaf()`
  - 每个 leaf: `getNodeBoundingBox()` → bbox 12 边 → `g_debugDisplay->drawLines()`
  - `"Clear"` → `g_debugDisplay->clear()`
  - + `#include "core/IPWBuilder.h"` + `"core/IDebugDisplay.h"`

## 数据流
```
SettingsWindow → billetDef 修改 → ipwDirty = true
                  ↓
main loop → IPWBuilder::build(billetDef, ipw.config) → ipw
                  ↓
Debug → BilletBuilderTest → microGrid leaf bboxes → g_debugDisplay
```

## 修改文件
- `app/main.cpp` — IPWBuilder::build 调用
- `app/windows/DebugWindow.h` — drawIPWTest()
- `app/windows/DebugWindow.cpp` — 按钮 + bbox 遍历 + drawLines
