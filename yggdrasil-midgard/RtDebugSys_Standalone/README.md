# RtDebugSys 编程指南 (V1.0)

`RtDebugSys` 是一个轻量级、异步、线程安全且零依赖的 C++17 运行时调试系统。它允许开发者在不重新编译程序的情况下，通过修改外部配置文件实时控制调试输出。

## 1. 核心特性
*   **零开销（关闭时）**：在宏级别进行分支预测优化，关闭时对算法性能影响几乎为零。
*   **分级控制 (Verbose Mode)**：支持 `Normal` (1) 和 `Verbose` (2) 两种级别，可精细化过滤海量诊断数据。
*   **异步日志**：日志写入由后台线程完成，不会阻塞算法的核心计算线程（支持 OpenMP/std::thread）。
*   **节流热重载**：配置文件检查频率限制在 500ms 一次，避免高频磁盘 I/O 拖垮算法。
*   **标签自愈（Auto-Discovery）**：代码中新写的调试标签若在配置文件中不存在，系统会自动将其添加并默认禁用。

---

## 2. 快速集成

### 第一步：引入源码
将 `RtDebugSys.h` 和 `RtDebugSys.cpp` 拷贝到您的项目中，并确保编译器支持 **C++17**。

### 第二步：初始化环境
在程序入口（如 `main` 函数）设置工作目录：

```cpp
#include "RtDebugSys.h"

int main() {
    // 设置存放 RTDebug.cfg 和日志文件的目录
    RtDebugSys::Debugger::GetInstance().SetWorkspace("./debug_output");
    // 全局启动调试系统 (1: Normal, 2: Verbose)
    RtDebugSys::Debugger::GetInstance().Activate(1);

    // ... 运行您的算法 ...
    return 0;
}
```

---

## 3. 常用宏 API

### `DEBUG_SECTION(Tag)`
**用途**：创建一个受控的代码块。只要 `Tag` 激活且主开关 >= 1 时，内部代码就会执行。

### `DEBUG_SECTION_VERBOSE(Tag)`
**用途**：创建一个**详细模式**受控块。只有当 `Tag` 激活且主开关设为 `2` (Verbose) 时，内部代码才会执行。
**场景**：极其高频的迭代日志、体素/点云细节输出。

```cpp
DEBUG_SECTION_VERBOSE(DeepIntersection) {
    // 只有在 RTDebug.cfg 第一行为 2 时才运行
    DEBUG_INFO_OUT("Checking intersection for voxel: " + v.id());
}
```

### `DEBUG_INFO_OUT(String)`
**用途**：输出带时间戳的异步日志。支持多线程并发调用。

### `CHECK_SECTION()`
**用途**：全局调试开关检查（只要主开关 > 0）。

---

## 4. 配置文件管理 (`RTDebug.cfg`)

系统启动后会自动在工作目录下生成 `RTDebug.cfg`。其格式如下：

```ini
2                # 第一行：主级别（0:关闭, 1:Normal, 2:Verbose）
AlgoStep1        # 激活标签
#AlgoStep2       # 禁用标签
```

---

## 5. 编译与测试
可以使用以下命令编译测试用例：
```bash
g++ -std=c++17 RtDebugSys.cpp main_test.cpp -o rtd_test
./rtd_test
```
