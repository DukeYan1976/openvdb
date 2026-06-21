# Skill: RedGreen Loop

> 启动口令：`启动 RedGreen Loop` 或 `进入开发循环`

## 定义

基于 TDD 的三角色开发循环：**架构师（你）→ 审查者（Kiro）→ 编码 Agent**。

## 角色

| 角色 | 职责 |
|------|------|
| 架构师 | 提需求目标，最终验收 |
| Kiro（审查者） | 拆解任务→设计 GTest→输出编码提示词→审查代码 |
| 编码 Agent | 根据提示词实现，确保测试通过 |

## 循环

```
① Kiro → 提供 2~4 个候选目标（含优先级建议+依赖说明）
② 你 → 选定目标（可修改/合并/自定义）
③ Kiro → 输出：
   • 任务拆解
   • GTest 测试代码（RED：编译通过但测试失败）
   • 编码 Agent 提示词（含约束+禁忌）
   • 【潜在风险点】
④ 你 → 交给编码 Agent
⑤ Agent → 实现（GREEN：测试全过）
⑥ 你 → 代码交回 Kiro 审查
⑦ Kiro → PASS / REJECT + 修改意见
⑧ 你 → 验收（编译+测试+视觉）→ 下一轮
```

## 提示词模板

```markdown
# [任务名]

## 目标
一句话。

## 上下文
- 读取：[文件列表]
- 关键接口：[签名摘要]

## 测试先行
[完整 GTest 代码]

## 实现约束
- 允许修改：...
- 禁止修改：...
- 风格：namespace midgard, #pragma once, C++17
- 性能边界：...

## 验证标准
- `cmake --build build && ctest --test-dir build`
- 测试全绿 + 无 warning

## 禁忌
- 禁止 git push
- 禁止引入新第三方依赖
- 禁止修改 CMakeLists.txt 主结构（除非明确指定）
```

## 设计权威

- 算法/管线：`DETAIL_DESIGN_20260618_v1.3.md`
- UI/交互：`design_ui.md`
- 编码 Agent 基础 prompt：`KIRO_PROMPT.md`

## 当前队列（按依赖序）

1. 仿真引擎集成 — MacroCut+MicroCut 接入 tick()
2. SDF Mesh 实时渲染 — MacroGrid → MC → GPU
3. 刀路渲染 — PathRenderer 分色
4. 刀具跟随渲染

## 快捷指令

| 指令 | 含义 |
|------|------|
| `下一轮` | Kiro 输出下一个任务的 TDD 提示词 |
| `审查` | 提交代码给 Kiro 做 code review |
| `验收通过` | 标记当前轮完成，推进队列 |
| `插入任务 [描述]` | 在队列头部插入紧急任务 |
| `状态` | Kiro 汇报当前队列和进度 |
