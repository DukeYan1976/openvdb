# 切削仿真时间调度与扫掠精度技术文档

## 文档信息
- **标题**: Machining Simulation Time Scheduling & Sweep Accuracy
- **版本**: 1.0
- **日期**: 2026-06-08
- **覆盖范围**: 时间驱动仿真 / 事件驱动仿真 / 一次性IPW计算
- **核心目标**: 确保扫掠构建时的路径精度，同时满足不同仿真模式的需求

---

## 1. 仿真模式总览

### 1.1 三种仿真模式对比

| 维度 | 时间驱动仿真 (Time-Driven) | 事件驱动仿真 (Event-Driven) | 一次性IPW计算 (In-Process Workpiece) |
|:---|:---|:---|:---|
| **输入** | G-code + 时间参数 | G-code + 几何事件 | 完整刀具路径 |
| **输出** | 每帧工件状态 | 关键事件点状态 | 最终工件几何 |
| **时间推进** | 固定/自适应Δt | 事件到事件 | 无时间概念，纯几何 |
| **精度控制** | CFL条件 + 自适应 | 事件边界精确 | 全局最优采样 |
| **性能目标** | 实时交互 (16ms/帧) | 准实时 (100ms/事件) | 离线批处理 (分钟级) |
| **典型应用** | 虚拟加工监控 | 碰撞检测/验证 | NC程序验证/成本估算 |
| **扫掠体构建** | 增量式，每步更新 | 段级精确构建 | 全局精确并集 |
| **内存模式** | 流式，只保留当前 | 段级缓存 | 全量加载 |

### 1.2 统一架构

```
┌─────────────────────────────────────────────────────────────────┐
│                    统一调度框架 (Unified Scheduler)                │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐   │
│  │ 模式选择器   │  │ 资源管理器   │  │ 性能监控器               │   │
│  │ Mode Selector│  │  Resource   │  │  Performance Monitor    │   │
│  └──────┬──────┘  │   Manager   │  └─────────────────────────┘   │
│         │         └──────┬──────┘                                  │
│         └────────────────┘                                          │
├─────────────────────────────────────────────────────────────────────┤
│                    三种模式实现                                       │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐    │
│  │ 时间驱动仿真     │  │ 事件驱动仿真     │  │ 一次性IPW计算   │    │
│  │ Time-Driven     │  │ Event-Driven    │  │ One-Shot IPW    │    │
│  │                 │  │                 │  │                 │    │
│  │ • Adaptive Δt   │  │ • Event Queue   │  │ • Global Optimal│    │
│  │ • Per-frame update│ • Phase-based   │  │ • Exact Boolean │    │
│  │ • Real-time target│ • Predictive    │  │ • Full Cache    │    │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘    │
├─────────────────────────────────────────────────────────────────────┤
│                    公共基础设施                                       │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌───────────┐ │
│  │ 路径解析器   │  │ 扫掠体构建器 │  │ 体素更新器   │  │ SDF管理器  │ │
│  │ Path Parser │  │Sweep Builder │  │ Voxel Updater│  │SDF Manager │ │
│  └─────────────┘  └─────────────┘  └─────────────┘  └───────────┘ │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 2. 时间驱动仿真 (Time-Driven Simulation)

### 2.1 核心特征
- **时间推进**: 按固定或自适应时间步长Δt推进
- **状态更新**: 每步更新刀具位姿和工件状态
- **实时目标**: 满足交互帧率（60fps = 16.67ms/帧）

### 2.2 自适应时间步长控制器

```cpp
namespace openvdb {
namespace cam {

/// @brief 自适应时间步长控制器
/// @details 根据速度、距离、曲率动态调整Δt，确保精度和性能
class AdaptiveTimeStepController {
public:
    struct Config {
        // 边界约束
        float min_dt = 0.0001f;        // 0.1ms (接触表面时)
        float max_dt = 0.016f;         // 16ms (一帧时间)
        float nominal_dt = 0.001f;      // 1ms (默认值)
        
        // CFL条件
        float cfl_factor = 1.0f;        // CFL系数
        float voxel_size = 0.01f;       // 目标体素尺寸
        
        // 距离因子
        float approach_threshold = 1.0f;    // 接近阈值 (mm)
        float contact_threshold = 0.01f;      // 接触阈值 (mm)
        
        // 曲率因子
        float curvature_threshold = 0.1f;    // 曲率阈值 (1/mm)
        
        // 稳定性
        float max_change_ratio = 2.0f;   // 步长最大变化率
        int history_size = 10;           // 历史窗口大小
    };
    
    struct State {
        float current_dt = 0.001f;
        float prev_dt = 0.001f;
        float avg_dt = 0.001f;
        std::deque<float> history;
        
        // 运行时统计
        int steps_taken = 0;
        int steps_clamped_by_min = 0;
        int steps_clamped_by_max = 0;
        float total_sim_time = 0.0f;
    };
    
    struct Context {
        float tool_speed;           // 刀具当前速度 (mm/s)
        float distance_to_surface;  // 到工件表面距离 (mm)
        float path_curvature;       // 路径曲率 (1/mm)
        bool is_cutting;            // 是否正在切削
        float frame_budget_ms;      // 当前帧剩余时间预算
    };

public:
    explicit AdaptiveTimeStepController(const Config& config = Config())
        : config_(config) {}
    
    /// @brief 计算下一步时间步长
    float computeNextDT(const Context& ctx) {
        // 1. 速度因子: dt ≤ voxel_size / speed
        float speed_dt = (ctx.tool_speed > 1e-6f)
            ? (config_.voxel_size * config_.cfl_factor / ctx.tool_speed)
            : config_.max_dt;
        
        // 2. 距离因子
        float distance_dt;
        if (ctx.distance_to_surface > config_.approach_threshold) {
            // 远离: 大步长
            distance_dt = config_.max_dt;
        } else if (ctx.distance_to_surface > config_.contact_threshold * 2) {
            // 接近: 中等步长
            distance_dt = config_.nominal_dt;
        } else if (ctx.distance_to_surface > 0) {
            // 非常接近: 小步长
            distance_dt = config_.min_dt * 2;
        } else {
            // 已接触: 严格步长
            distance_dt = config_.min_dt;
        }
        
        // 3. 曲率因子
        float curvature_dt = config_.nominal_dt;
        if (ctx.path_curvature > config_.curvature_threshold) {
            curvature_dt = config_.min_dt * (1.0f + config_.curvature_threshold / ctx.path_curvature);
        }
        
        // 4. 帧预算因子
        float budget_dt = ctx.frame_budget_ms * 0.001f * 0.8f;  // 留20%余量
        
        // 5. 综合: 取最严格的约束
        float proposed = std::min({speed_dt, distance_dt, curvature_dt, budget_dt});
        
        // 6. 限制变化率 (避免振荡)
        if (!state_.history.empty()) {
            float max_increase = state_.prev_dt * config_.max_change_ratio;
            float max_decrease = state_.prev_dt / config_.max_change_ratio;
            proposed = std::clamp(proposed, max_decrease, max_increase);
        }
        
        // 7. 全局边界
        proposed = std::clamp(proposed, config_.min_dt, config_.max_dt);
        
        // 8. 更新状态
        state_.prev_dt = state_.current_dt;
        state_.current_dt = proposed;
        state_.history.push_back(proposed);
        if (state_.history.size() > config_.history_size) {
            state_.history.pop_front();
        }
        
        // 计算滑动平均
        if (!state_.history.empty()) {
            state_.avg_dt = std::accumulate(state_.history.begin(), 
                                             state_.history.end(), 0.0f) 
                           / state_.history.size();
        }
        
        state_.steps_taken++;
        if (proposed <= config_.min_dt * 1.01f) state_.steps_clamped_by_min++;
        if (proposed >= config_.max_dt * 0.99f) state_.steps_clamped_by_max++;
        
        return proposed;
    }
    
    /// @brief 获取当前状态
    const State& getState() const { return state_; }
    
    /// @brief 重置
    void reset() { state_ = State(); }

private:
    Config config_;
    State state_;
};

} // namespace cam
} // namespace openvdb
```

### 2.3 时间驱动主循环

```cpp
class TimeDrivenSimulator {
public:
    struct FrameResult {
        float frame_time_ms;        // 实际帧耗时
        float simulation_time;       // 仿真时间推进量
        int steps_in_frame;         // 本帧内步数
        bool dropped_frame;         // 是否掉帧
    };
    
    struct Config {
        AdaptiveTimeStepController::Config dt_config;
        float target_fps = 60.0f;                    // 目标帧率
        float max_drop_rate = 0.1f;                  // 最大允许掉帧率
        bool enable_predictive_subdivide = true;     // 预测性细化
        bool enable_async_render = true;             // 异步渲染
    };

public:
    explicit TimeDrivenSimulator(const Config& config) : config_(config) {}
    
    /// @brief 初始化仿真
    void initialize(const Workpiece& workpiece, const Tool& tool) {
        // 构建初始体素网格
        grid_ = buildAdaptiveGrid(workpiece, config_.dt_config.voxel_size);
        
        // 初始化SDF
        sdf_manager_.initialize(grid_, workpiece);
        
        // 设置刀具
        current_tool_ = tool;
        
        // 重置控制器
        dt_controller_.reset();
        
        // 统计
        stats_ = FrameStats();
    }
    
    /// @brief 单帧仿真（时间驱动核心）
    FrameResult simulateFrame(const ToolPath& path, float frame_budget_ms) {
        auto frame_start = getHighResTime();
        FrameResult result;
        result.steps_in_frame = 0;
        
        float frame_sim_time = 0.0f;  // 本帧内仿真时间推进
        float remaining_budget = frame_budget_ms;
        
        // 帧内多步推进
        while (remaining_budget > 0.5f &&  // 至少0.5ms才值得做一步
               frame_sim_time < (1.0f / config_.target_fps)) {  // 推进不超过一帧
            
            auto step_start = getHighResTime();
            
            // 1. 获取当前上下文
            AdaptiveTimeStepController::Context ctx;
            ctx.tool_speed = current_tool_.getCurrentSpeed();
            ctx.distance_to_surface = queryDistanceToSurface(current_tool_.getPosition());
            ctx.path_curvature = path.getCurvatureAt(current_time_);
            ctx.is_cutting = (ctx.distance_to_surface <= 0);
            ctx.frame_budget_ms = remaining_budget;
            
            // 2. 计算时间步长
            float dt = dt_controller_.computeNextDT(ctx);
            
            // 3. 获取刀具位姿（插补）
            ToolPose pose = path.interpolate(current_time_ + dt);
            current_tool_.setPose(pose);
            
            // 4. 构建扫掠体（增量）
            SweptVolume sweep = buildIncrementalSweep(
                current_tool_, 
                path.getPose(current_time_),
                pose,
                dt
            );
            
            // 5. 查询受影响体素
            auto affected_voxels = grid_.queryRegion(sweep.getAABB());
            
            // 6. 预测性细化
            if (config_.enable_predictive_subdivide && ctx.is_cutting) {
                subdivideForPrecision(affected_voxels, config_.dt_config.voxel_size);
            }
            
            // 7. 执行切削
            executeCut(affected_voxels, sweep);
            
            // 8. 更新SDF
            sdf_manager_.fastUpdate(affected_voxels, current_tool_);
            
            // 9. 更新时间
            current_time_ += dt;
            frame_sim_time += dt;
            result.steps_in_frame++;
            
            // 10. 计算步耗时，更新预算
            auto step_end = getHighResTime();
            float step_time_ms = (step_end - step_start) * 1000.0f;
            remaining_budget -= step_time_ms;
            
            // 性能自适应：如果步耗时过长，调整策略
            if (step_time_ms > remaining_budget * 0.5f && result.steps_in_frame > 1) {
                // 紧急降精度
                emergencyDegrade();
                break;  // 结束本帧
            }
        }
        
        // 帧结束统计
        auto frame_end = getHighResTime();
        result.frame_time_ms = (frame_end - frame_start) * 1000.0f;
        result.simulation_time = frame_sim_time;
        result.dropped_frame = (result.frame_time_ms > (1.0f / config_.target_fps * 1000.0f));
        
        updateStats(result);
        
        return result;
    }
    
    /// @brief 获取当前工件表面（用于渲染）
    Mesh getCurrentSurface(SurfaceQuality quality = SurfaceQuality::FAST_PREVIEW) const {
        return extractor_.extract(grid_, quality);
    }
    
    /// @brief 是否完成
    bool isComplete(const ToolPath& path) const {
        return current_time_ >= path.getTotalDuration();
    }
    
    /// @brief 获取统计
    const FrameStats& getStats() const { return stats_; }

private:
    Config config_;
    AdaptiveTimeStepController dt_controller_;
    
    // 状态
    HybridGrid grid_;
    Tool current_tool_;
    float current_time_ = 0.0f;
    
    // 子系统
    AdaptiveSDFManager sdf_manager_;
    MultiQualityExtractor extractor_;
    
    // 统计
    FrameStats stats_;
    
    // 内部方法
    SweptVolume buildIncrementalSweep(const Tool& tool, 
                                       const ToolPose& from,
                                       const ToolPose& to,
                                       float dt);
    void executeCut(const std::vector<VoxelIndex>& voxels, const SweptVolume& sweep);
    void subdivideForPrecision(std::vector<VoxelIndex>& voxels, float target_size);
    void emergencyDegrade();
    void updateStats(const FrameResult& result);
};
```

### 2.4 增量扫掠体构建

```cpp
/// @brief 增量扫掠体（时间驱动专用）
/// @details 只构建当前步的扫掠体，而非全局
class IncrementalSweepBuilder {
public:
    struct Config {
        float min_samples = 2;          // 最少采样点数
        float max_samples = 100;        // 最多采样点数
        float samples_per_mm = 10.0f;   // 每mm采样数
        bool use_analytic_sweep = true;  // 对简单刀具使用解析扫掠
    };
    
    /// @brief 构建两姿态间的增量扫掠体
    SweptVolume build(const Tool& tool,
                      const ToolPose& from,
                      const ToolPose& to,
                      float dt,
                      const Config& config = Config()) {
        
        // 1. 计算运动参数
        Vec3 displacement = to.position - from.position;
        float distance = displacement.magnitude();
        float speed = distance / dt;
        
        // 2. 确定采样数
        int num_samples = std::clamp(
            static_cast<int>(distance * config.samples_per_mm),
            static_cast<int>(config.min_samples),
            static_cast<int>(config.max_samples)
        );
        
        // 3. 对简单刀具使用解析扫掠
        if (config.use_analytic_sweep && tool.isSimpleGeometry()) {
            return buildAnalyticSweep(tool, from, to, num_samples);
        }
        
        // 4. 通用：自适应采样扫掠
        return buildAdaptiveSweep(tool, from, to, num_samples);
    }

private:
    /// @brief 解析扫掠（球头刀直线运动 = 圆柱+半球）
    SweptVolume buildAnalyticSweep(const Tool& tool,
                                    const ToolPose& from,
                                    const ToolPose& to,
                                    int num_samples) {
        SweptVolume result;
        
        if (tool.type == ToolType::BALL_END) {
            // 球头刀扫掠体 = 圆柱（杆部） + 球面（端部）
            // 简化：用圆柱+两端半球近似
            
            float radius = tool.diameter * 0.5f;
            Vec3 axis = (to.position - from.position).normalized();
            float length = (to.position - from.position).magnitude();
            
            // 圆柱部分（侧刃）
            Cylinder side_cylinder(from.position, axis, radius, length);
            result.addPrimitive(side_cylinder);
            
            // 端部半球
            Sphere end_sphere(to.position, radius);
            result.addPrimitive(end_sphere);
            
            // 起始端半球（如果运动距离大于0）
            if (length > 1e-6f) {
                Sphere start_sphere(from.position, radius);
                result.addPrimitive(start_sphere);
            }
            
        } else if (tool.type == ToolType::FLAT_END) {
            // 平底刀扫掠体 = 圆柱（侧面） + 圆盘（底面扫掠）
            float radius = tool.diameter * 0.5f;
            Vec3 axis = (to.position - from.position).normalized();
            float length = (to.position - from.position).magnitude();
            
            // 侧面圆柱
            Cylinder side_cylinder(from.position, axis, radius, length);
            result.addPrimitive(side_cylinder);
            
            // 底面圆盘（扫掠成圆柱）
            if (length > 1e-6f) {
                Cylinder bottom_sweep(from.position, axis, radius, length);
                result.addPrimitive(bottom_sweep);
            }
        }
        
        // 计算AABB
        result.computeAABB();
        
        return result;
    }
    
    /// @brief 自适应采样扫掠
    SweptVolume buildAdaptiveSweep(const Tool& tool,
                                    const ToolPose& from,
                                    const ToolPose& to,
                                    int num_samples) {
        SweptVolume result;
        
        // 初始采样
        std::vector<ToolPose> samples;
        samples.push_back(from);
        
        // 递归细分
        subdivideSweep(tool, from, to, 0, 1, samples);
        
        samples.push_back(to);
        
        // 去重排序
        std::sort(samples.begin(), samples.end(), 
                  [](const ToolPose& a, const ToolPose& b) { return a.t < b.t; });
        
        // 构建扫掠体 = 各采样点刀具体积的并集
        for (const auto& pose : samples) {
            ToolPose temp_tool = tool;
            temp_tool.setPose(pose);
            result.addToolPose(temp_tool);
        }
        
        result.computeAABB();
        
        return result;
    }
    
    /// @brief 递归细分
    void subdivideSweep(const Tool& tool,
                        const ToolPose& p0,
                        const ToolPose& p1,
                        float t0, float t1,
                        std::vector<ToolPose>& samples) {
        
        float t_mid = (t0 + t1) * 0.5f;
        ToolPose p_mid = interpolate(p0, p1, 0.5f);
        
        // 线性插值中点
        ToolPose p_linear = interpolate(p0, p1, 0.5f);
        
        // 误差评估
        float error = evaluateSweepError(p_mid, p_linear, tool);
        
        if (error > target_precision_ && (t1 - t0) > min_param_delta_) {
            // 需要细分
            subdivideSweep(tool, p0, p_mid, t0, t_mid, samples);
            samples.push_back(p_mid);
            subdivideSweep(tool, p_mid, p1, t_mid, t1, samples);
        }
    }
    
    float evaluateSweepError(const ToolPose& actual, const ToolPose& linear, 
                             const Tool& tool) {
        // 位置误差
        float pos_error = (actual.position - linear.position).magnitude();
        
        // 方向误差（影响刀具轮廓）
        float orient_error = 1.0f - dot(actual.orientation, linear.orientation);
        
        // 包络误差 = 位置误差 + 刀具半径 × 方向误差
        float envelope_error = pos_error + tool.getRadius() * orient_error;
        
        return envelope_error;
    }
};
```

---

## 3. 事件驱动仿真 (Event-Driven Simulation)

### 3.1 核心特征
- **事件驱动**: 以关键几何事件为节点推进
- **精确边界**: 事件点精确处理，事件间快速推进
- **准实时**: 不需要每帧更新，适合验证和调试

### 3.2 事件定义与分类

```cpp
namespace openvdb {
namespace cam {

/// @brief 仿真事件类型
enum class SimulationEventType : uint8_t {
    // 路径事件
    PATH_SEGMENT_START = 0,     // 路径段开始
    PATH_SEGMENT_END,           // 路径段结束
    
    // 几何事件
    APPROACH_WORKPIECE,         // 接近工件（距离 < approach_threshold）
    CONTACT_WORKPIECE,          // 接触工件（距离 <= 0）
    LEAVE_WORKPIECE,            // 离开工件（距离 > 0）
    RETRACT_SAFE,               // 退刀到安全高度
    
    // 运动学事件
    CORNER_APPROACH,            // 接近拐角
    CORNER_LEAVE,               // 离开拐角
    ACCELERATION_START,         // 开始加速
    DECELERATION_START,         // 开始减速
    
    // 工艺事件
    TOOL_CHANGE,                // 换刀
    SPINDLE_START,              // 主轴启动
    SPINDLE_STOP,               // 主轴停止
    COOLANT_ON,                 // 冷却液开
    COOLANT_OFF,                // 冷却液关
    
    // 精度事件
    PRECISION_CHANGE,           // 精度要求变化
    SUBDIVIDE_REQUIRED,         // 需要局部细化
    COARSEN_ALLOWED,            // 允许粗化
    
    // 异常事件
    COLLISION_WARNING,          // 碰撞警告
    GOUGE_DETECTED,             // 过切检测
    EXCESSIVE_FORCE             // 切削力过大
};

/// @brief 仿真事件
struct SimulationEvent {
    SimulationEventType type;
    float time;                     // 预计发生时间（从路径开始）
    Vec3 position;                // 发生位置
    Vec3 orientation;             // 刀具方向
    
    // 事件参数
    union {
        struct { float approach_distance; } approach;
        struct { float contact_depth; } contact;
        struct { float corner_radius; float entry_angle; } corner;
        struct { int new_tool_id; } tool_change;
        struct { float new_precision; } precision;
    } params;
    
    float priority;               // 处理优先级（0-100）
    bool requires_exact;          // 是否需要精确处理（不能跳过）
    
    // 排序用
    bool operator<(const SimulationEvent& other) const {
        return time < other.time;
    }
};

} // namespace cam
} // namespace openvdb
```

### 3.3 事件预测器

```cpp
class EventPredictor {
public:
    struct Config {
        float approach_threshold = 1.0f;      // 接近阈值 (mm)
        float contact_threshold = 0.001f;     // 接触阈值 (mm)
        float corner_angle_threshold = 5.0f;  // 拐角角度阈值 (度)
        float safe_height = 10.0f;            // 安全高度 (mm)
    };
    
    /// @brief 预测路径段上的所有事件
    std::vector<SimulationEvent> predictEvents(const PathSegment& segment,
                                                const Workpiece& workpiece,
                                                const Tool& tool) {
        std::vector<SimulationEvent> events;
        
        // 1. 段开始事件
        events.push_back({
            SimulationEventType::PATH_SEGMENT_START,
            segment.start_time,
            segment.start_pos,
            segment.start_dir,
            {}, 1.0f, true
        });
        
        // 2. 预测接近/接触事件
        auto contact_events = predictContactEvents(segment, workpiece, tool);
        events.insert(events.end(), contact_events.begin(), contact_events.end());
        
        // 3. 预测拐角事件
        auto corner_events = predictCornerEvents(segment);
        events.insert(events.end(), corner_events.begin(), corner_events.end());
        
        // 4. 预测速度变化事件
        auto speed_events = predictSpeedEvents(segment);
        events.insert(events.end(), speed_events.begin(), speed_events.end());
        
        // 5. 段结束事件
        events.push_back({
            SimulationEventType::PATH_SEGMENT_END,
            segment.end_time,
            segment.end_pos,
            segment.end_dir,
            {}, 1.0f, true
        });
        
        // 按时间排序
        std::sort(events.begin(), events.end());
        
        // 合并重复事件
        mergeDuplicateEvents(events);
        
        return events;
    }

private:
    Config config_;
    
    /// @brief 预测接触事件
    std::vector<SimulationEvent> predictContactEvents(const PathSegment& segment,
                                                        const Workpiece& workpiece,
                                                        const Tool& tool) {
        std::vector<SimulationEvent> events;
        
        // 采样路径，检测与工件距离变化
        int samples = 100;
        float prev_distance = std::numeric_limits<float>::max();
        
        for (int i = 0; i <= samples; i++) {
            float t = static_cast<float>(i) / samples;
            ToolPose pose = segment.interpolate(t);
            
            // 查询到工件距离
            float distance = workpiece.distanceToSurface(pose.position, tool);
            
            // 检测接近事件
            if (prev_distance > config_.approach_threshold && 
                distance <= config_.approach_threshold) {
                // 二分查找精确接近时间
                float exact_t = findExactApproach(segment, workpiece, tool, 
                                                     (i-1.0f)/samples, t);
                ToolPose exact_pose = segment.interpolate(exact_t);
                
                events.push_back({
                    SimulationEventType::APPROACH_WORKPIECE,
                    segment.start_time + exact_t * segment.duration,
                    exact_pose.position,
                    exact_pose.orientation,
                    {.approach = {distance}},
                    10.0f, true
                });
            }
            
            // 检测接触事件
            if (prev_distance > 0 && distance <= 0) {
                float exact_t = findExactContact(segment, workpiece, tool,
                                                  (i-1.0f)/samples, t);
                ToolPose exact_pose = segment.interpolate(exact_t);
                
                events.push_back({
                    SimulationEventType::CONTACT_WORKPIECE,
                    segment.start_time + exact_t * segment.duration,
                    exact_pose.position,
                    exact_pose.orientation,
                    {.contact = {-distance}},  // 接触深度
                    20.0f, true  // 最高优先级
                });
            }
            
            // 检测离开事件
            if (prev_distance <= 0 && distance > 0) {
                float exact_t = findExactLeave(segment, workpiece, tool,
                                                (i-1.0f)/samples, t);
                ToolPose exact_pose = segment.interpolate(exact_t);
                
                events.push_back({
                    SimulationEventType::LEAVE_WORKPIECE,
                    segment.start_time + exact_t * segment.duration,
                    exact_pose.position,
                    exact_pose.orientation,
                    {}, 10.0f, true
                });
            }
            
            prev_distance = distance;
        }
        
        return events;
    }
    
    /// @brief 预测拐角事件
    std::vector<SimulationEvent> predictCornerEvents(const PathSegment& segment) {
        std::vector<SimulationEvent> events;
        
        if (segment.type != PathSegmentType::G01) {
            // 圆弧段本身就是曲线，分析曲率变化
            auto curvature_profile = segment.getCurvatureProfile();
            
            for (const auto& peak : curvature_profile.peaks) {
                if (peak.curvature > config_.corner_angle_threshold * M_PI / 180.0f) {
                    ToolPose pose = segment.interpolate(peak.t);
                    
                    events.push_back({
                        SimulationEventType::CORNER_APPROACH,
                        segment.start_time + peak.t * segment.duration,
                        pose.position,
                        pose.orientation,
                        {.corner = {1.0f / peak.curvature, 0.0f}},
                        8.0f, false
                    });
                }
            }
        }
        
        return events;
    }
    
    /// @brief 二分查找精确接触时间
    float findExactContact(const PathSegment& segment,
                           const Workpiece& workpiece,
                           const Tool& tool,
                           float t_low, float t_high) {
        float t_mid = (t_low + t_high) * 0.5f;
        ToolPose pose = segment.interpolate(t_mid);
        float distance = workpiece.distanceToSurface(pose.position, tool);
        
        if (std::abs(distance) < 1e-6f || (t_high - t_low) < 1e-6f) {
            return t_mid;
        }
        
        if (distance > 0) {
            return findExactContact(segment, workpiece, tool, t_mid, t_high);
        } else {
            return findExactContact(segment, workpiece, tool, t_low, t_mid);
        }
    }
};
```

### 3.4 事件驱动主循环

```cpp
class EventDrivenSimulator {
public:
    struct PhaseConfig {
        // 不同相位的精度配置
        float fast_forward_precision = 0.1f;    // 快速推进精度
        float approach_precision = 0.01f;       // 接近精度
        float cutting_precision = 0.005f;       // 切削精度
        float corner_precision = 0.002f;        // 拐角精度
    };
    
    struct EventResult {
        SimulationEventType processed_event;
        float event_time;
        float processing_time_ms;
        int substeps_taken;
        bool success;
    };

public:
    /// @brief 初始化并预处理路径
    void initialize(const ToolPath& path, const Workpiece& workpiece, 
                    const Tool& tool) {
        // 构建初始网格
        grid_ = buildAdaptiveGrid(workpiece, 0.1f);  // 粗初始化
        
        // 预处理：预测所有事件
        all_events_.clear();
        for (const auto& segment : path.segments) {
            auto segment_events = event_predictor_.predictEvents(segment, workpiece, tool);
            all_events_.insert(all_events_.end(), segment_events.begin(), segment_events.end());
        }
        
        // 全局排序
        std::sort(all_events_.begin(), all_events_.end());
        
        // 初始化状态
        current_time_ = 0.0f;
        current_event_idx_ = 0;
        current_tool_ = tool;
        
        // 预构建段级扫掠体（离线）
        if (config_.prebuild_sweeps) {
            prebuildSegmentSweeps(path, tool);
        }
    }
    
    /// @brief 处理下一个事件
    EventResult processNextEvent(const ToolPath& path) {
        if (current_event_idx_ >= all_events_.size()) {
            return {SimulationEventType::PATH_SEGMENT_END, current_time_, 0, 0, true};
        }
        
        auto start_time = getHighResTime();
        
        const auto& event = all_events_[current_event_idx_];
        EventResult result;
        result.processed_event = event.type;
        result.event_time = event.time;
        result.substeps_taken = 0;
        
        // 1. 快速推进到事件前（如果距离较远）
        if (event.time - current_time_ > 0.001f) {
            fastForward(current_time_, event.time, path);
        }
        
        // 2. 根据事件类型精确处理
        switch (event.type) {
            case SimulationEventType::APPROACH_WORKPIECE:
                result = handleApproach(event, path);
                break;
                
            case SimulationEventType::CONTACT_WORKPIECE:
                result = handleContact(event, path);
                break;
                
            case SimulationEventType::LEAVE_WORKPIECE:
                result = handleLeave(event, path);
                break;
                
            case SimulationEventType::CORNER_APPROACH:
                result = handleCorner(event, path);
                break;
                
            case SimulationEventType::PATH_SEGMENT_START:
            case SimulationEventType::PATH_SEGMENT_END:
                result = handleSegmentBoundary(event, path);
                break;
                
            default:
                result = handleGeneric(event, path);
                break;
        }
        
        // 3. 更新状态
        current_time_ = event.time;
        current_event_idx_++;
        
        auto end_time = getHighResTime();
        result.processing_time_ms = (end_time - start_time) * 1000.0f;
        
        return result;
    }
    
    /// @brief 处理到指定时间（用于外部控制）
    void simulateToTime(float target_time, const ToolPath& path) {
        while (current_event_idx_ < all_events_.size() &&
               all_events_[current_event_idx_].time <= target_time) {
            processNextEvent(path);
        }
        
        // 推进到精确时间
        if (current_time_ < target_time) {
            fastForward(current_time_, target_time, path);
            current_time_ = target_time;
        }
    }
    
    /// @brief 是否还有更多事件
    bool hasMoreEvents() const {
        return current_event_idx_ < all_events_.size();
    }
    
    /// @brief 获取当前进度
    float getProgress() const {
        return static_cast<float>(current_event_idx_) / all_events_.size();
    }

private:
    // 配置
    EventPredictor::Config predictor_config_;
    PhaseConfig phase_config_;
    bool prebuild_sweeps_ = true;
    
    // 状态
    HybridGrid grid_;
    Tool current_tool_;
    float current_time_ = 0.0f;
    size_t current_event_idx_ = 0;
    
    // 事件
    std::vector<SimulationEvent> all_events_;
    EventPredictor event_predictor_;
    
    // 预构建扫掠体
    std::unordered_map<int, SweptVolume> segment_sweeps_;
    
    // 事件处理器
    EventResult handleApproach(const SimulationEvent& event, const ToolPath& path);
    EventResult handleContact(const SimulationEvent& event, const ToolPath& path);
    EventResult handleLeave(const SimulationEvent& event, const ToolPath& path);
    EventResult handleCorner(const SimulationEvent& event, const ToolPath& path);
    EventResult handleSegmentBoundary(const SimulationEvent& event, const ToolPath& path);
    EventResult handleGeneric(const SimulationEvent& event, const ToolPath& path);
    
    // 快速推进
    void fastForward(float from_time, float to_time, const ToolPath& path) {
        // 使用大步长快速推进，不构建精确扫掠体
        // 只更新刀具位姿，不更新工件（假设无切削）
        
        float dt = to_time - from_time;
        ToolPose from_pose = path.interpolate(from_time);
        ToolPose to_pose = path.interpolate(to_time);
        
        // 验证：快速推进期间不应有接触
        #ifndef NDEBUG
        int check_samples = 10;
        for (int i = 0; i <= check_samples; i++) {
            float t = from_time + dt * i / check_samples;
            ToolPose pose = path.interpolate(t);
            float dist = queryDistanceToSurface(pose.position);
            assert(dist > 0 && "Contact detected during fast-forward!");
        }
        #endif
        
        current_tool_.setPose(to_pose);
    }
    
    // 精确切削处理（接触事件）
    EventResult handleContact(const SimulationEvent& event, const ToolPath& path) {
        EventResult result;
        result.substeps_taken = 0;
        
        // 1. 在接触点附近局部细化到最高精度
        subdivideRegion(event.position, phase_config_.cutting_precision);
        
        // 2. 从接触点开始，小步长精确仿真直到离开
        float sim_time = event.time;
        float max_sim_duration = 1.0f;  // 最多仿真1秒
        
        while (sim_time < event.time + max_sim_duration) {
            // 获取当前上下文
            ToolPose pose = path.interpolate(sim_time);
            current_tool_.setPose(pose);
            
            float dist = queryDistanceToSurface(pose.position);
            
            if (dist > 0) {
                // 已离开，结束
                break;
            }
            
            // 小步长精确仿真
            float dt = computePreciseDT(pose, phase_config_.cutting_precision);
            
            // 构建精确扫掠体
            ToolPose next_pose = path.interpolate(sim_time + dt);
            SweptVolume sweep = buildPreciseSweep(current_tool_, pose, next_pose);
            
            // 执行切削
            auto affected = grid_.queryRegion(sweep.getAABB());
            executeCut(affected, sweep);
            
            sim_time += dt;
            result.substeps_taken++;
        }
        
        result.success = true;
        return result;
    }
};
```

---

## 4. 一次性IPW计算 (One-Shot In-Process Workpiece)

### 4.1 核心特征
- **全局优化**: 一次性处理完整路径，构建最终工件
- **精确布尔**: 使用精确几何布尔运算，非体素近似
- **离线批处理**: 时间要求宽松，追求最高精度
- **成本估算**: 常用于加工时间、材料成本估算

### 4.2 IPW定义

```
IPW (In-Process Workpiece): 
    加工过程中任意时刻的工件状态
    
数学定义:
    IPW(t) = W_initial - ∪_{τ∈[0,t]} SV(τ)
    
其中:
    W_initial = 初始工件几何
    SV(τ) = 时刻τ的刀具扫掠体
    ∪ = 几何并集（布尔减）
```

### 4.3 全局扫掠体构建策略

```cpp
class OneShotIPWCalculator {
public:
    struct Config {
        // 精度控制
        float target_precision = 0.001f;      // 1μm目标（比实时高10倍）
        float merge_tolerance = 0.0001f;      // 合并容差
        
        // 计算策略
        bool use_exact_boolean = true;        // 使用精确布尔（vs 体素近似）
        bool use_parallel_sweep = true;       // 并行构建扫掠体
        bool use_hierarchical_merge = true;   // 层次合并（vs 线性合并）
        
        // 内存管理
        size_t max_memory_mb = 8192;          // 最大内存
        bool use_disk_swap = true;            // 磁盘交换
        
        // 输出
        bool output_intermediate = false;     // 输出中间状态
        int intermediate_steps = 10;          // 中间状态数
    };
    
    struct IPWResult {
        Mesh final_workpiece;                 // 最终工件
        float total_cut_volume;               // 总切削体积
        float total_cut_time;                 // 总切削时间
        std::vector<Mesh> intermediate_states; // 中间状态（可选）
        std::vector<float> state_times;        // 中间状态时间
        
        // 统计
        int num_sweeps_generated;              // 生成的扫掠体数
        int num_boolean_operations;            // 布尔运算数
        double total_compute_time_sec;         // 总计算时间
    };

public:
    explicit OneShotIPWCalculator(const Config& config = Config()) 
        : config_(config) {}
    
    /// @brief 计算最终IPW
    IPWResult calculate(const ToolPath& path, const Workpiece& initial,
                        const Tool& tool) {
        auto start_time = getHighResTime();
        IPWResult result;
        
        // 1. 路径分段与并行任务生成
        auto sweep_tasks = generateSweepTasks(path, tool);
        result.num_sweeps_generated = static_cast<int>(sweep_tasks.size());
        
        // 2. 并行构建所有扫掠体
        std::vector<SweptVolume> sweeps;
        if (config_.use_parallel_sweep) {
            sweeps = buildSweepsParallel(sweep_tasks);
        } else {
            sweeps = buildSweepsSequential(sweep_tasks);
        }
        
        // 3. 层次合并扫掠体（减少布尔运算次数）
        SweptVolume total_swept_volume;
        if (config_.use_hierarchical_merge) {
            total_swept_volume = hierarchicalMerge(sweeps);
        } else {
            total_swept_volume = linearMerge(sweeps);
        }
        
        // 4. 精确布尔减
        Mesh final_ipw;
        if (config_.use_exact_boolean) {
            final_ipw = exactBooleanSubtract(initial.mesh, total_swept_volume.toMesh());
        } else {
            // 体素近似（回退）
            final_ipw = voxelBooleanSubtract(initial, sweeps, config_.target_precision);
        }
        
        result.final_workpiece = final_ipw;
        result.num_boolean_operations = countBooleanOps();
        
        // 5. 计算统计
        result.total_cut_volume = computeCutVolume(initial, final_ipw);
        result.total_cut_time = path.getTotalDuration();
        
        // 6. 中间状态（可选）
        if (config_.output_intermediate) {
            result.intermediate_states = computeIntermediateStates(
                path, initial, tool, config_.intermediate_steps
            );
        }
        
        auto end_time = getHighResTime();
        result.total_compute_time_sec = end_time - start_time;
        
        return result;
    }
    
    /// @brief 增量式IPW更新（用于大路径分段处理）
    IPWResult calculateIncremental(const ToolPath& path, const Workpiece& initial,
                                    const Tool& tool, int batch_size = 100) {
        IPWResult result;
        Mesh current_ipw = initial.mesh;
        
        int num_batches = (path.segments.size() + batch_size - 1) / batch_size;
        
        for (int batch = 0; batch < num_batches; batch++) {
            int start_idx = batch * batch_size;
            int end_idx = std::min(start_idx + batch_size, 
                                    static_cast<int>(path.segments.size()));
            
            // 构建本批扫掠体
            std::vector<SweptVolume> batch_sweeps;
            for (int i = start_idx; i < end_idx; i++) {
                auto sweep = buildExactSweep(path.segments[i], tool);
                batch_sweeps.push_back(sweep);
            }
            
            // 合并本批
            auto batch_merged = hierarchicalMerge(batch_sweeps);
            
            // 布尔减更新IPW
            current_ipw = exactBooleanSubtract(current_ipw, batch_merged.toMesh());
            
            // 保存中间状态
            if (config_.output_intermediate) {
                result.intermediate_states.push_back(current_ipw);
                result.state_times.push_back(path.segments[end_idx - 1].end_time);
            }
            
            // 内存检查
            if (getCurrentMemoryMB() > config_.max_memory_mb && config_.use_disk_swap) {
                flushToDisk(current_ipw);
            }
        }
        
        result.final_workpiece = current_ipw;
        return result;
    }

private:
    Config config_;
    
    // 任务生成
    std::vector<SweepTask> generateSweepTasks(const ToolPath& path, const Tool& tool) {
        std::vector<SweepTask> tasks;
        
        for (const auto& segment : path.segments) {
            // 根据段类型选择构建策略
            if (segment.type == PathSegmentType::G01 && segment.isShort()) {
                // 短直线：单扫掠体
                tasks.push_back({segment, SweepBuildMethod::LINEAR});
            } else if (segment.type == PathSegmentType::G02 || 
                       segment.type == PathSegmentType::G03) {
                // 圆弧：解析扫掠
                tasks.push_back({segment, SweepBuildMethod::ANALYTIC_ARC});
            } else if (segment.hasComplexMotion()) {
                // 复杂运动：自适应采样
                tasks.push_back({segment, SweepBuildMethod::ADAPTIVE});
            } else {
                // 默认
                tasks.push_back({segment, SweepBuildMethod::STANDARD});
            }
        }
        
        return tasks;
    }
    
    // 层次合并（关键优化）
    SweptVolume hierarchicalMerge(const std::vector<SweptVolume>& sweeps) {
        if (sweeps.empty()) return SweptVolume();
        if (sweeps.size() == 1) return sweeps[0];
        
        // 层次合并：两两合并，减少总布尔运算次数
        // O(N) vs O(N log N) 的优化
        
        std::vector<SweptVolume> current_level = sweeps;
        
        while (current_level.size() > 1) {
            std::vector<SweptVolume> next_level;
            
            for (size_t i = 0; i < current_level.size(); i += 2) {
                if (i + 1 < current_level.size()) {
                    // 合并两个
                    auto merged = booleanUnion(current_level[i], current_level[i+1]);
                    next_level.push_back(merged);
                } else {
                    // 奇数个，直接传递
                    next_level.push_back(current_level[i]);
                }
            }
            
            current_level = std::move(next_level);
        }
        
        return current_level[0];
    }
    
    // 精确布尔减
    Mesh exactBooleanSubtract(const Mesh& workpiece, const Mesh& swept_volume) {
        // 使用CGAL、Open CASCADE或类似库
        // 这里用伪代码表示
        
        #ifdef USE_CGAL
        return cgalBooleanSubtract(workpiece, swept_volume);
        #elif defined(USE_OPENCASCADE)
        return occBooleanSubtract(workpiece, swept_volume);
        #else
        // 回退：体素近似
        return voxelBooleanSubtract(workpiece, swept_volume, config_.target_precision);
        #endif
    }
};
```

### 4.4 精确扫掠体构建（一次性专用）

```cpp
class ExactSweepBuilder {
public:
    /// @brief 精确圆弧扫掠（球头刀）
    /// @details 球头刀沿圆弧运动，扫掠体是环面段
    Mesh buildBallEndArcSweep(const ArcSegment& arc, float tool_radius) {
        // 环面参数
        float major_radius = arc.radius;           // 圆弧半径
        float minor_radius = tool_radius * 0.5f;   // 刀具半径（球头）
        
        // 构建完整环面
        Torus torus(arc.center, arc.normal, major_radius, minor_radius);
        
        // 裁剪到圆弧角度范围
        Mesh torus_mesh = torus.toMesh();
        
        // 角度裁剪
        float start_angle = arc.start_angle;
        float end_angle = arc.end_angle;
        
        if (arc.clockwise) {
            std::swap(start_angle, end_angle);
        }
        
        return clipMeshByAngleRange(torus_mesh, arc.center, arc.normal, 
                                     start_angle, end_angle);
    }
    
    /// @brief 精确直线扫掠（平底刀）
    Mesh buildFlatEndLinearSweep(const LineSegment& line, float tool_radius, 
                                  float tool_length) {
        // 平底刀直线扫掠 = 圆柱（侧面）+ 圆柱（底面扫掠）
        
        Vec3 direction = (line.end - line.start).normalized();
        float length = (line.end - line.start).magnitude();
        
        // 1. 侧面扫掠：圆柱
        Cylinder side_cylinder(line.start, direction, tool_radius, length);
        
        // 2. 底面扫掠：圆柱（底面圆盘沿路径扫掠）
        // 简化：底面扫掠体也是圆柱，半径=刀具半径，高度=路径长度
        Cylinder bottom_sweep(line.start, direction, tool_radius, length);
        
        // 3. 端部圆盘（起始和结束）
        Disk start_disk(line.start, -direction, tool_radius);
        Disk end_disk(line.end, direction, tool_radius);
        
        // 合并
        return booleanUnion({
            side_cylinder.toMesh(),
            bottom_sweep.toMesh(),
            start_disk.toMesh(),
            end_disk.toMesh()
        });
    }
    
    /// @brief 通用自适应精确扫掠
    Mesh buildAdaptiveExactSweep(const PathSegment& segment, const Tool& tool) {
        // 高精度自适应采样
        const int max_samples = 10000;
        const float target_error = 0.0001f;  // 0.1μm
        
        std::vector<ToolPose> samples;
        samples.push_back(segment.startPose());
        
        // 递归细分
        subdivideExact(segment, 0.0f, 1.0f, samples, target_error, max_samples);
        
        samples.push_back(segment.endPose());
        
        // 构建扫掠体 = 所有采样点刀具的凸包/并集
        Mesh sweep;
        for (const auto& pose : samples) {
            Tool temp_tool = tool;
            temp_tool.setPose(pose);
            Mesh tool_mesh = temp_tool.toMesh();
            sweep = booleanUnion(sweep, tool_mesh);
        }
        
        return sweep;
    }
    
private:
    void subdivideExact(const PathSegment& segment, float t0, float t1,
                        std::vector<ToolPose>& samples, float target_error,
                        int max_depth) {
        if (max_depth <= 0) return;
        
        ToolPose p0 = segment.interpolate(t0);
        ToolPose p1 = segment.interpolate(t1);
        ToolPose p_mid = segment.interpolate((t0 + t1) * 0.5f);
        
        // 线性插值
        ToolPose p_linear = interpolate(p0, p1, 0.5f);
        
        // 精确误差
        float error = (p_mid.position - p_linear.position).magnitude();
        
        // 方向误差（影响刀具轮廓）
        float orient_error = 1.0f - dot(p_mid.orientation, p_linear.orientation);
        error += orient_error * 0.1f;  // 方向权重
        
        if (error > target_error) {
            float t_mid = (t0 + t1) * 0.5f;
            subdivideExact(segment, t0, t_mid, samples, target_error, max_depth - 1);
            samples.push_back(p_mid);
            subdivideExact(segment, t_mid, t1, samples, target_error, max_depth - 1);
        }
    }
};
```

---

## 5. 统一调度框架

### 5.1 模式选择与运行时切换

```cpp
class UnifiedSimulationFramework {
public:
    enum class SimulationMode {
        TIME_DRIVEN,        // 时间驱动（实时交互）
        EVENT_DRIVEN,       // 事件驱动（准实时验证）
        ONE_SHOT_IPW        // 一次性IPW（离线批处理）
    };
    
    struct RuntimeConfig {
        SimulationMode mode;
        
        // 时间驱动配置
        TimeDrivenSimulator::Config time_config;
        
        // 事件驱动配置
        EventDrivenSimulator::PhaseConfig event_config;
        
        // IPW配置
        OneShotIPWCalculator::Config ipw_config;
        
        // 运行时切换
        bool allow_runtime_switch = true;
        float switch_threshold_fps = 30.0f;  // 低于此帧率可切换模式
    };
    
    struct UnifiedResult {
        Mesh workpiece;
        float simulation_time;
        float real_time;
        SimulationMode used_mode;
        bool mode_switched;
    };

public:
    explicit UnifiedSimulationFramework(const RuntimeConfig& config) 
        : config_(config) {
        // 初始化所有模式
        time_sim_ = std::make_unique<TimeDrivenSimulator>(config.time_config);
        event_sim_ = std::make_unique<EventDrivenSimulator>();
        ipw_calc_ = std::make_unique<OneShotIPWCalculator>(config.ipw_config);
    }
    
    /// @brief 统一仿真入口
    UnifiedResult simulate(const ToolPath& path, const Workpiece& workpiece,
                           const Tool& tool) {
        switch (config_.mode) {
            case SimulationMode::TIME_DRIVEN:
                return simulateTimeDriven(path, workpiece, tool);
                
            case SimulationMode::EVENT_DRIVEN:
                return simulateEventDriven(path, workpiece, tool);
                
            case SimulationMode::ONE_SHOT_IPW:
                return simulateOneShot(path, workpiece, tool);
        }
        
        return UnifiedResult{};
    }
    
    /// @brief 运行时模式切换（自适应）
    UnifiedResult simulateAdaptive(const ToolPath& path, const Workpiece& workpiece,
                                    const Tool& tool) {
        if (!config_.allow_runtime_switch) {
            return simulate(path, workpiece, tool);
        }
        
        // 先尝试时间驱动
        auto start = getHighResTime();
        auto result = simulateTimeDriven(path, workpiece, tool);
        auto time_driven_duration = getHighResTime() - start;
        
        // 检查性能
        float actual_fps = result.simulation_time / time_driven_duration;
        
        if (actual_fps < config_.switch_threshold_fps) {
            // 性能不足，切换到事件驱动
            result = simulateEventDriven(path, workpiece, tool);
            result.mode_switched = true;
        }
        
        return result;
    }

private:
    RuntimeConfig config_;
    
    std::unique_ptr<TimeDrivenSimulator> time_sim_;
    std::unique_ptr<EventDrivenSimulator> event_sim_;
    std::unique_ptr<OneShotIPWCalculator> ipw_calc_;
    
    UnifiedResult simulateTimeDriven(const ToolPath& path, 
                                      const Workpiece& workpiece,
                                      const Tool& tool) {
        time_sim_->initialize(workpiece, tool);
        
        auto start = getHighResTime();
        float sim_time = 0.0f;
        
        while (!time_sim_->isComplete(path)) {
            auto frame_result = time_sim_->simulateFrame(path, 16.0f);
            sim_time += frame_result.simulation_time;
            
            // 可以在这里插入渲染/交互
        }
        
        auto end = getHighResTime();
        
        return {
            time_sim_->getCurrentSurface(),
            sim_time,
            static_cast<float>(end - start),
            SimulationMode::TIME_DRIVEN,
            false
        };
    }
    
    UnifiedResult simulateEventDriven(const ToolPath& path,
                                       const Workpiece& workpiece,
                                       const Tool& tool) {
        event_sim_->initialize(path, workpiece, tool);
        
        auto start = getHighResTime();
        float sim_time = 0.0f;
        
        while (event_sim_->hasMoreEvents()) {
            auto event_result = event_sim_->processNextEvent(path);
            sim_time = event_result.event_time;
        }
        
        auto end = getHighResTime();
        
        return {
            Mesh{},  // 事件驱动不持续维护表面
            sim_time,
            static_cast<float>(end - start),
            SimulationMode::EVENT_DRIVEN,
            false
        };
    }
    
    UnifiedResult simulateOneShot(const ToolPath& path,
                                   const Workpiece& workpiece,
                                   const Tool& tool) {
        auto start = getHighResTime();
        
        auto result = ipw_calc_->calculate(path, workpiece, tool);
        
        auto end = getHighResTime();
        
        return {
            result.final_workpiece,
            result.total_cut_time,
            static_cast<float>(result.total_compute_time_sec),
            SimulationMode::ONE_SHOT_IPW,
            false
        };
    }
};
```

---

## 6. 扫掠精度保证机制总结

### 6.1 三种模式的精度策略对比

| 精度机制 | 时间驱动 | 事件驱动 | 一次性IPW |
|:---|:---|:---|:---|
| **时间步长** | 自适应Δt，CFL约束 | 事件边界精确 | 无时间，纯几何 |
| **采样密度** | 速度相关，动态调整 | 段级固定，事件点加密 | 全局最优，误差驱动 |
| **扫掠体构建** | 增量式，解析+采样 | 段级精确，预构建 | 全局精确，层次合并 |
| **布尔运算** | 体素更新（近似） | 段级体素更新 | 精确几何布尔 |
| **精度上限** | 体素尺寸（0.01mm） | 体素尺寸（0.01mm） | 浮点精度（~1μm） |
| **无遗漏保证** | CFL + 保守AABB | 段级全覆盖验证 | 精确布尔并集 |
| **亚体素精度** | SDF插值 | SDF插值 | 精确NURBS/曲面 |

### 6.2 关键精度公式

```
时间驱动精度:
    位置精度 = voxel_size × (0.5 ~ 1.0)  [体素离散化]
             + SDF插值误差  [亚体素修正]
             + CFL约束误差  [运动采样]
    
    总误差 ≈ 0.01mm (目标)

事件驱动精度:
    位置精度 = voxel_size × (0.5 ~ 1.0)
             + SDF插值误差
             + 事件定位误差  [二分查找精度]
    
    总误差 ≈ 0.01mm (目标)

一次性IPW精度:
    位置精度 = 几何布尔误差  [曲面求交精度]
             + 扫掠采样误差  [自适应细分阈值]
             + 合并误差  [布尔并集容差]
    
    总误差 ≈ 0.001mm (目标，10倍于实时)
```

### 6.3 无遗漏切削的数学保证

```
定理：无遗漏条件

对于时间驱动模式，如果满足：
    ∀t, dt:  max_speed(t) × dt ≤ voxel_size / √3

则刀具在一个时间步内移动的距离不超过体素对角线，
因此不会跳过任何体素。

证明：
    体素对角线 = voxel_size × √3
    CFL条件：speed × dt ≤ voxel_size
    保守条件：speed × dt ≤ voxel_size / √3 确保即使斜向运动也不遗漏

对于事件驱动模式，如果满足：
    每个PATH_SEGMENT事件都处理段内所有可能接触

则通过段级全覆盖验证保证无遗漏。

对于一次性IPW模式：
    精确布尔并集保证：∪ SV_i 覆盖所有切削区域
```

---

## 7. 性能-精度权衡决策表

| 应用场景 | 推荐模式 | 精度 | 性能 | 关键配置 |
|:---|:---|:---|:---|:---|
| 虚拟加工监控 | 时间驱动 | 0.01mm | 60fps | 自适应Δt, 预测细化 |
| 碰撞检测 | 事件驱动 | 0.01mm | 10fps | 事件预处理, 精确接触 |
| NC程序验证 | 一次性IPW | 0.001mm | 分钟级 | 精确布尔, 层次合并 |
| 成本估算 | 一次性IPW | 0.01mm | 秒级 | 体素近似, 批量处理 |
| 刀具路径优化 | 事件驱动 | 0.005mm | 1fps | 拐角精确, 力预测 |
| 数字孪生实时 | 时间驱动 | 0.05mm | 30fps | 粗体素, 快速预览 |

---

## 8. 附录

### 8.1 与现有文档的关系

| 本文档 | 关系 | 对应文档 |
|:---|:---|:---|
| 时间调度与扫掠精度 | 本技术文档 | `Time_Scheduling_Sweep_Accuracy.md` |
| GPU方案分析 | 基础分析 | `GPU_Voxel_Machining_Analysis.md` |
| CPU优化方案 | 扩展方案 | `CPU_Voxel_Machining_Analysis.md` |
| OpenVDB对比 | 对比分析 | `Paper_vs_OpenVDB_Comparison.md` |
| 改进提案 | 架构提案 | `OpenVDB-CAM_Proposal.md` |

### 8.2 关键术语表

| 术语 | 英文 | 定义 |
|:---|:---|:---|
| IPW | In-Process Workpiece | 加工过程中工件 |
| CFL条件 | Courant-Friedrichs-Lewy | 时间步长与空间步长稳定性约束 |
| 扫掠体 | Swept Volume | 刀具运动轨迹的体积并集 |
| 事件驱动 | Event-Driven | 以关键事件为节点推进仿真 |
| 时间驱动 | Time-Driven | 按固定/自适应时间步推进 |
| 自适应采样 | Adaptive Sampling | 根据误差动态调整采样密度 |
| 精确布尔 | Exact Boolean | 几何曲面精确求交运算 |
| 层次合并 | Hierarchical Merge | 树状结构合并减少运算次数 |

---

*本文档覆盖切削仿真的三种核心模式，提供统一的时间调度框架和扫掠精度保证机制。*
