# OpenVDB-CAM 技术提案

## 面向切削仿真的稀疏体素框架改进方案

**版本**: 1.0  
**日期**: 2026-06-08  
**提案人**: CAM-Architect  
**基础**: OpenVDB (DreamWorks) + GPU-accelerated voxel-based machining simulation 论文

---

## 1. 执行摘要

### 1.1 问题陈述
现有方案存在以下割裂：
- **OpenVDB**: 优秀的稀疏体素基础设施，但面向动画/VFX，缺乏切削语义，CPU-only
- **论文GPU方案**: 实时切削能力，但缺乏OpenVDB的成熟I/O、序列化和通用性
- **工业CAM软件**: 封闭生态，难以扩展

### 1.2 提案核心
**在OpenVDB基础上，注入切削专用语义，扩展GPU后端，形成统一的工业级切削仿真框架。**

### 1.3 预期成果
| 指标 | 现有OpenVDB | 现有论文 | OpenVDB-CAM目标 |
|:---|:---|:---|:---|
| 稀疏体素存储 | ✅ | ✅ | ✅ 继承并增强 |
| 切削语义 | ❌ | ✅ 基础 | ✅ 完整 |
| GPU加速 | ❌ | ✅ CUDA | ✅ CUDA/Metal/HIP |
| 工业精度 | ~0.1mm | 0.01mm | **0.005mm** |
| 超大数据集 | ✅ TB级 | ❌ 内存限制 | ✅ 延迟I/O + GPU |
| 标准格式 | ✅ .vdb | ❌ | ✅ .vdb + .cam |
| 开源生态 | ✅ Apache 2.0 | 未明确 | ✅ Apache 2.0 |

---

## 2. 架构设计

### 2.1 三层架构

```
┌─────────────────────────────────────────────────────────────┐
│                    Layer 3: CAM Application Layer              │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐   │
│  │ Tool Manager │  │ Path Parser │  │ Force/Thermal      │   │
│  │ (刀具定义)   │  │ (G-code/NC) │  │ Coupling (可选)    │   │
│  └─────────────┘  └─────────────┘  └─────────────────────┘   │
├─────────────────────────────────────────────────────────────┤
│                    Layer 2: OpenVDB-CAM Core                 │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐   │
│  │ Hybrid Tree │  │ CutVoxel    │  │ Incremental        │   │
│  │ (混合树结构) │  │ Value Type  │  │ Topology Manager   │   │
│  └─────────────┘  └─────────────┘  └─────────────────────┘   │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐   │
│  │ Adaptive SDF│  │ Multi-Quality│  │ Material Database  │   │
│  │ Manager     │  │ Extractor   │  │ (材料库)            │   │
│  └─────────────┘  └─────────────┘  └─────────────────────┘   │
├─────────────────────────────────────────────────────────────┤
│                    Layer 1: Hardware Abstraction             │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐   │
│  │ CPU Backend │  │ GPU Backend │  │ Unified Scheduler  │   │
│  │ (TBB/AVX2)  │  │ (CUDA/Metal)│  │ (自动选择)          │   │
│  └─────────────┘  └─────────────┘  └─────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

### 2.2 模块依赖关系

```
CAM Application
      │
      ├─→ Tool Manager ──→ Geometry Primitives (圆柱/球体/圆锥)
      │
      ├─→ Path Parser ──→ NC文件/G-code/CL数据
      │
      └─→ Force/Thermal ──→ 材料属性库 ──→ Material Database
              │
              ▼
      OpenVDB-CAM Core
              │
      ├─→ Hybrid Tree ──→ OpenVDB Tree (继承)
      │         │
      │         ├─→ B+Tree Nodes (CPU优化)
      │         └─→ Octree Nodes (GPU优化)
      │
      ├─→ CutVoxelValue ──→ TypedGrid (继承)
      │         │
      │         ├─→ sdf: float
      │         ├─→ volume_fraction: float
      │         └─→ state: uint8_t
      │
      ├─→ Incremental Topology ──→ OpenVDB Topology (扩展)
      │         │
      │         ├─→ 局部细化 (Subdivide)
      │         ├─→ 局部粗化 (Coarsen)
      │         └─→ 切削更新 (Cut)
      │
      ├─→ Adaptive SDF ──→ OpenVDB Level Set (扩展)
      │         │
      │         ├─→ FastUpdate (增量)
      │         └─→ FullReinit (全量)
      │
      └─→ Multi-Quality Extractor ──→ MarchingCubes/DualContouring
                │
                ├─→ FAST_PREVIEW
                ├─→ STANDARD
                ├─→ HIGH_PRECISION
                └─→ CMM_ACCURATE
                      │
                      ▼
              Hardware Abstraction
                      │
              ├─→ CPU Backend
              │         ├─→ TBB Task Scheduler
              │         ├─→ AVX2/AVX512 Kernels
              │         └─→ OpenMP Fallback
              │
              └─→ GPU Backend
                        ├─→ CUDA (NVIDIA)
                        ├─→ Metal (Apple)
                        └─→ HIP (AMD)
```

---

## 3. 核心接口设计

### 3.1 命名空间与模块划分

```cpp
namespace openvdb {
namespace cam {  // OpenVDB-CAM 扩展命名空间

// 基础类型
using Vec3f = math::Vec3<float>;
using Vec3d = math::Vec3<double>;
using AABB = math::BBox<Vec3d>;

// 前向声明
class Tool;
class ToolPath;
class CutVoxelValue;
class HybridTree;
class IncrementalTopologyManager;
class AdaptiveSDFManager;
class MultiQualityExtractor;
class UnifiedScheduler;

} // namespace cam
} // namespace openvdb
```

### 3.2 CutVoxelValue — 切削专用值类型

```cpp
namespace openvdb {
namespace cam {

/// @brief 切削仿真专用体素值类型
/// @details 融合OpenVDB值语义与论文切削物理
class CutVoxelValue {
public:
    // 状态枚举
    enum State : uint8_t {
        STATE_UNDEFINED = 0,    // 未初始化
        STATE_AIR       = 1,    // 空气/空洞
        STATE_MATERIAL  = 2,    // 完整材料
        STATE_MACHINED  = 3,    // 已切削（残留）
        STATE_TOOL      = 4,    // 刀具占据（临时）
        STATE_RESERVED  = 5     // 保留区域（夹具等）
    };

    // 构造函数
    CutVoxelValue() : sdf_(1.0f), volume_fraction_(1.0f), state_(STATE_MATERIAL) {}
    
    explicit CutVoxelValue(float sdf) 
        : sdf_(sdf), volume_fraction_(sdf > 0 ? 1.0f : 0.0f), 
          state_(sdf > 0 ? STATE_MATERIAL : STATE_AIR) {}
    
    CutVoxelValue(float sdf, float vf, State state)
        : sdf_(sdf), volume_fraction_(vf), state_(state) {}

    // OpenVDB兼容接口 —— 必须实现
    static CutVoxelValue zero() { return CutVoxelValue(1.0f, 1.0f, STATE_MATERIAL); }
    static CutVoxelValue background() { return CutVoxelValue(1.0f, 1.0f, STATE_MATERIAL); }
    
    // 类型特征（OpenVDB TypedGrid要求）
    using ValueType = CutVoxelValue;
    using FloatType = float;
    static constexpr bool IsVec = false;
    static constexpr bool IsFloat = true;
    static constexpr bool IsClass = true;
    
    // 核心访问器
    float sdf() const { return sdf_; }
    float volumeFraction() const { return volume_fraction_; }
    State state() const { return state_; }
    
    void setSdf(float sdf) { sdf_ = sdf; }
    void setVolumeFraction(float vf) { volume_fraction_ = math::Clamp(vf, 0.0f, 1.0f); }
    void setState(State s) { state_ = s; }

    // 切削操作（论文核心思想）
    /// @brief 执行切削，移除指定体积
    /// @param removed_volume 被移除的材料体积
    /// @param voxel_volume 体素总体积
    /// @return 实际移除的体积（可能受volume_fraction限制）
    float cut(float removed_volume, float voxel_volume);
    
    /// @brief 快速切削：完全移除
    void cutFully() { volume_fraction_ = 0.0f; state_ = STATE_AIR; sdf_ = -1.0f; }
    
    /// @brief 部分切削：按比例
    void cutByRatio(float ratio) {
        volume_fraction_ = math::Max(0.0f, volume_fraction_ - ratio);
        if (volume_fraction_ <= 0.0f) {
            state_ = STATE_AIR;
            sdf_ = -1.0f;
        }
    }

    // SDF更新（与刀具SDF取min）
    void updateSDF(float tool_sdf) {
        sdf_ = math::Min(sdf_, tool_sdf);
        if (sdf_ < 0 && state_ == STATE_MATERIAL) {
            state_ = STATE_MACHINED;
        }
    }

    // 状态查询
    bool isMaterial() const { return state_ == STATE_MATERIAL && volume_fraction_ > 0.99f; }
    bool isAir() const { return state_ == STATE_AIR || volume_fraction_ < 0.01f; }
    bool isMachined() const { return state_ == STATE_MACHINED || (volume_fraction_ > 0.01f && volume_fraction_ < 0.99f); }
    bool isActive() const { return volume_fraction_ > 0.01f; }  // OpenVDB Active语义
    
    // 插值支持（三线性插值）
    static CutVoxelValue lerp(const CutVoxelValue& a, const CutVoxelValue& b, float t) {
        return CutVoxelValue(
            math::Lerp(a.sdf_, b.sdf_, t),
            math::Lerp(a.volume_fraction_, b.volume_fraction_, t),
            (t < 0.5f) ? a.state_ : b.state_  // 状态取最近
        );
    }

    // 运算符（OpenVDB Tree操作要求）
    bool operator==(const CutVoxelValue& other) const {
        return sdf_ == other.sdf_ && volume_fraction_ == other.volume_fraction_ && state_ == other.state_;
    }
    bool operator!=(const CutVoxelValue& other) const { return !(*this == other); }
    
    // 输出
    friend std::ostream& operator<<(std::ostream& os, const CutVoxelValue& v) {
        os << "{sdf=" << v.sdf_ << ", vf=" << v.volume_fraction_ 
           << ", state=" << static_cast<int>(v.state_) << "}";
        return os;
    }

private:
    float sdf_;              // 有向距离场（+内部，-外部）
    float volume_fraction_;  // 材料体积分数 [0,1]
    State state_;            // 离散状态
};

// 特化ValueTraits（OpenVDB注册）
template<> struct ValueTraits<cam::CutVoxelValue> {
    using Type = cam::CutVoxelValue;
    using BidirectionalType = cam::CutVoxelValue;
    using NumericType = float;
    static constexpr bool IsVec = false;
    static constexpr bool IsFloat = true;
    static constexpr bool IsClass = true;
    static constexpr bool IsSparse = true;  // 支持稀疏存储
    static constexpr size_t Size = sizeof(CutVoxelValue);  // 9 bytes (padding to 12)
};

// 特化io::Compression（序列化支持）
template<> struct io::Compression<cam::CutVoxelValue> {
    static constexpr bool IsCompressed = true;
    static constexpr bool FixedSize = true;
};

} // namespace cam
} // namespace openvdb
```

### 3.3 HybridTree — 混合树结构

```cpp
namespace openvdb {
namespace cam {

/// @brief 混合树结构，自适应选择B+树或八叉树节点
/// @details 根据硬件类型、节点深度、活跃体素数动态选择最优节点类型
class HybridTree {
public:
    using Ptr = std::shared_ptr<HybridTree>;
    using ConstPtr = std::shared_ptr<const HybridTree>;
    
    // 节点类型枚举
    enum NodeType {
        NODE_BPLUS_INTERNAL,    // 大节点，32³，CPU缓存优化
        NODE_OCT_INTERNAL,      // 八叉节点，8子节点，GPU友好
        NODE_LEAF_CPU,          // 标准Leaf，8³，CPU优化
        NODE_LEAF_GPU,          // GPU线性布局，Morton序
        NODE_LEAF_UNIFIED       // 统一布局，自适应后端
    };
    
    // 配置
    struct Config {
        // 空间配置
        float voxel_size_min = 0.01f;      // 最小体素尺寸（目标精度）
        float voxel_size_max = 1.0f;       // 最大体素尺寸
        int max_depth = 10;               // 最大树深度
        
        // 硬件配置
        Backend preferred_backend = BACKEND_AUTO;
        bool allow_cpu_fallback = true;   // GPU失败时回退CPU
        
        // 节点配置
        int bplus_node_size = 32;         // B+树节点尺寸（32³）
        int leaf_node_size = 8;           // Leaf节点尺寸（8³）
        
        // 自适应阈值
        int bplus_threshold = 1000;       // 活跃体素>1000使用B+树
        int gpu_linear_threshold = 512;   // <512体素使用GPU线性
    };

    // 构造函数
    explicit HybridTree(const Config& config = Config());
    
    // 工厂方法
    static Ptr create(const Config& config = Config());
    static Ptr createFromOpenVDB(const openvdb::FloatGrid::Ptr& grid, const Config& config);
    
    // 核心操作
    
    /// @brief 初始化工件几何
    /// @param mesh 输入三角网格
    /// @param material_id 材料ID（查询材料库）
    void initializeWorkpiece(const Mesh& mesh, int material_id = 0);
    
    /// @brief 执行单步切削
    /// @param tool 当前刀具状态
    /// @param dt 时间步长
    /// @return 本步移除的材料体积
    float cutStep(const Tool& tool, float dt);
    
    /// @brief 批量切削（多步）
    /// @param path 刀具路径
    /// @param adaptive_dt 是否使用自适应时间步长
    /// @return 总移除体积
    float cutPath(const ToolPath& path, bool adaptive_dt = true);
    
    // 查询
    
    /// @brief 查询指定位置的体素值
    CutVoxelValue getValue(const Vec3d& world_pos) const;
    
    /// @brief 查询SDF
    float getSDF(const Vec3d& world_pos) const;
    
    /// @brief 查询体积分数
    float getVolumeFraction(const Vec3d& world_pos) const;
    
    /// @brief 查询表面法向（从SDF梯度）
    Vec3d getNormal(const Vec3d& world_pos) const;
    
    // 拓扑操作
    
    /// @brief 局部细化
    void subdivideRegion(const AABB& region, float target_voxel_size);
    
    /// @brief 局部粗化（释放内存）
    void coarsenRegion(const AABB& region, float min_voxel_size);
    
    /// @brief 平衡树（优化遍历性能）
    void balance();
    
    // 输出
    
    /// @brief 提取表面网格
    /// @param quality 提取质量级别
    Mesh extractSurface(SurfaceQuality quality = STANDARD) const;
    
    /// @brief 提取切屑几何
    Mesh extractChips() const;
    
    /// @brief 导出为OpenVDB标准格式
    openvdb::FloatGrid::Ptr toOpenVDB() const;
    
    /// @brief 导出为CAM专用格式
    void writeCAM(const std::string& filename) const;
    
    // 统计信息
    struct Stats {
        size_t total_voxels = 0;          // 总体素数（含所有层级）
        size_t active_voxels = 0;         // 活跃体素数
        size_t leaf_nodes = 0;            // Leaf节点数
        size_t internal_nodes = 0;       // Internal节点数
        size_t memory_bytes = 0;          // 内存占用
        int max_depth_reached = 0;       // 实际最大深度
        float avg_voxel_size = 0.0f;     // 平均体素尺寸
    };
    Stats getStats() const;
    
    // 后端访问
    Backend getActiveBackend() const;
    void forceBackend(Backend backend);  // 强制指定后端（调试用）

private:
    Config config_;
    
    // 根节点（多态）
    std::unique_ptr<HybridNode> root_;
    
    // 后端管理
    std::unique_ptr<UnifiedScheduler> scheduler_;
    Backend active_backend_;
    
    // 辅助管理器
    std::unique_ptr<IncrementalTopologyManager> topo_manager_;
    std::unique_ptr<AdaptiveSDFManager> sdf_manager_;
    std::unique_ptr<MultiQualityExtractor> extractor_;
    
    // 材料属性
    MaterialProperties material_props_;
    
    // 内部方法
    void autoSelectBackend();
    HybridNode* createOptimalNode(const AABB& bounds, int depth, int active_count);
};

} // namespace cam
} // namespace openvdb
```

### 3.4 Tool — 刀具定义（解析几何）

```cpp
namespace openvdb {
namespace cam {

/// @brief 刀具类型枚举
enum ToolType {
    TOOL_FLAT_END,       // 平底刀
    TOOL_BALL_END,       // 球头刀
    TOOL_BULL_NOSE,      // 圆鼻刀
    TOOL_DRILL,          // 钻头
    TOOL_TAPERED,        // 锥度刀
    TOOL_CUSTOM_MESH     // 自定义网格（从CAD导入）
};

/// @brief 刀具几何定义
/// @details 使用解析几何精确表示，避免离散化误差
class Tool {
public:
    using Ptr = std::shared_ptr<Tool>;
    
    // 工厂方法
    static Ptr createFlatEnd(float diameter, float length);
    static Ptr createBallEnd(float diameter, float length);
    static Ptr createBullNose(float diameter, float corner_radius, float length);
    static Ptr createFromMesh(const Mesh& mesh, float resolution = 0.01f);
    static Ptr createFromStandard(int iso_id);  // ISO标准刀具编号
    
    // 几何查询
    
    /// @brief 精确SDF查询
    /// @param point 世界坐标系中的点
    /// @return 点到刀具表面的有向距离（<0在内部）
    float sdf(const Vec3d& point) const;
    
    /// @brief 批量SDF查询（SIMD优化）
    void sdfBatch(const float* px, const float* py, const float* pz,
                  float* out, int count) const;
    
    /// @brief 刀具占用测试
    bool isInside(const Vec3d& point) const { return sdf(point) < 0; }
    
    /// @brief 计算刀具AABB
    AABB getAABB() const;
    AABB getAABB(const Vec3d& position, const Vec3d& direction) const;  // 考虑方向
    
    /// @brief 刀具扫掠体AABB（沿路径段）
    AABB getSweepAABB(const Vec3d& pos_start, const Vec3d& pos_end,
                      const Vec3d& dir_start, const Vec3d& dir_end) const;
    
    // 运动学
    
    /// @brief 设置当前位姿
    void setPose(const Vec3d& position, const Vec3d& direction);
    void setPose(const Transform& transform);
    
    /// @brief 获取当前位姿
    Vec3d getPosition() const;
    Vec3d getDirection() const;
    Transform getTransform() const;
    
    // 物理属性
    
    /// @brief 设置切削参数
    void setCuttingParameters(float spindle_speed, float feed_rate, float depth_of_cut);
    
    /// @brief 计算理论材料移除率
    float getMaterialRemovalRate() const;  // mm³/min
    
    // 刀具补偿
    
    /// @brief 设置刀具磨损补偿
    void setWearCompensation(float radius_offset, float length_offset);
    
    // 序列化
    void write(std::ostream& os) const;
    static Ptr read(std::istream& is);

private:
    ToolType type_;
    
    // 几何参数
    float diameter_ = 0.0f;
    float length_ = 0.0f;
    float corner_radius_ = 0.0f;
    float taper_angle_ = 0.0f;
    Mesh custom_mesh_;  // 自定义网格（体素化缓存）
    
    // 当前位姿
    Transform current_pose_;
    
    // 切削参数
    float spindle_speed_ = 0.0f;   // RPM
    float feed_rate_ = 0.0f;         // mm/min
    float depth_of_cut_ = 0.0f;    // mm
    
    // 补偿
    float radius_offset_ = 0.0f;
    float length_offset_ = 0.0f;
    
    // 解析SDF实现（私有）
    float sdfFlatEnd(const Vec3d& local_point) const;
    float sdfBallEnd(const Vec3d& local_point) const;
    float sdfBullNose(const Vec3d& local_point) const;
    float sdfCustomMesh(const Vec3d& world_point) const;
};

} // namespace cam
} // namespace openvdb
```

### 3.5 UnifiedScheduler — 统一硬件调度器

```cpp
namespace openvdb {
namespace cam {

/// @brief 硬件后端枚举
enum Backend {
    BACKEND_AUTO = 0,       // 自动选择
    BACKEND_CPU_SCALAR,     // CPU标量（回退）
    BACKEND_CPU_AVX2,       // CPU AVX2
    BACKEND_CPU_AVX512,     // CPU AVX-512
    BACKEND_GPU_CUDA,       // NVIDIA CUDA
    BACKEND_GPU_METAL,      // Apple Metal
    BACKEND_GPU_HIP,        // AMD HIP
    BACKEND_GPU_VULKAN      // Vulkan Compute（未来）
};

/// @brief 任务描述
struct TaskDesc {
    size_t voxel_count = 0;         // 估计体素数
    size_t memory_required = 0;      // 估计内存需求
    bool requires_realtime = false;   // 是否需要实时
    bool requires_double_precision = false;  // 是否需要双精度
    float target_latency_ms = 16.0f;  // 目标延迟（帧时间）
};

/// @brief 统一硬件调度器
/// @details 自动选择最优后端，支持运行时切换
class UnifiedScheduler {
public:
    using Ptr = std::shared_ptr<UnifiedScheduler>;
    
    // 系统信息
    struct SystemInfo {
        bool has_avx2 = false;
        bool has_avx512 = false;
        bool has_cuda = false;
        bool has_metal = false;
        bool has_hip = false;
        
        int cpu_cores = 0;
        size_t cpu_memory = 0;
        
        int gpu_count = 0;
        size_t gpu_memory[4] = {0};  // 最多4个GPU
        std::string gpu_name[4];
    };
    
    // 构造函数
    UnifiedScheduler();
    
    // 系统检测
    static SystemInfo detectSystem();
    
    // 调度决策
    Backend selectBackend(const TaskDesc& task) const;
    
    // 执行接口
    template<typename Kernel>
    void execute(Kernel& kernel, const TaskDesc& task);
    
    // 特定后端执行
    void executeCPU(KernelCPU& kernel, int num_threads = 0);
    void executeGPU(KernelGPU& kernel, int device_id = 0);
    
    // 内存管理
    void* allocate(size_t bytes, Backend backend);
    void free(void* ptr, Backend backend);
    void copy(void* dst, const void* src, size_t bytes, Backend dst_backend, Backend src_backend);
    
    // 性能监控
    struct PerformanceMetrics {
        double last_execution_time_ms = 0.0;
        double avg_execution_time_ms = 0.0;
        size_t total_executions = 0;
        Backend last_backend = BACKEND_AUTO;
    };
    PerformanceMetrics getMetrics() const;
    
    // 手动控制
    void forceBackend(Backend backend) { forced_backend_ = backend; }
    void clearForced() { forced_backend_ = BACKEND_AUTO; }

private:
    SystemInfo sys_info_;
    Backend forced_backend_ = BACKEND_AUTO;
    PerformanceMetrics metrics_;
    
    // 后端实例
    std::unique_ptr<CPUBackend> cpu_backend_;
    std::unique_ptr<GPUBackend> gpu_backend_;
    
    // 决策逻辑
    Backend autoSelect(const TaskDesc& task) const;
};

} // namespace cam
} // namespace openvdb
```

---

## 4. 核心算法伪代码

### 4.1 增量式拓扑更新

```cpp
// IncrementalTopologyManager::updateForCut
void IncrementalTopologyManager::updateForCut(
    const Tool& tool, 
    const ToolPath& path,
    float dt
) {
    // 1. 计算当前刀具位姿
    ToolPose pose = path.getPose(current_time_);
    tool.setPose(pose.position, pose.direction);
    
    // 2. 计算刀具影响区域
    AABB tool_aabb = tool.getAABB();
    AABB influence_region = tool_aabb;
    influence_region.expand(config_.safety_margin);
    
    // 3. 查询受影响节点（OpenVDB现有能力）
    std::vector<HybridNode*> affected_nodes;
    tree_->queryNodesInRegion(influence_region, affected_nodes);
    
    // 4. 局部细化（论文思想）
    for (auto* node : affected_nodes) {
        if (node->type() == NODE_LEAF_CPU || node->type() == NODE_LEAF_GPU) {
            // 已经是Leaf，检查是否需要进一步细化
            if (node->voxelSize() > config_.target_precision) {
                subdivideLeaf(node, config_.target_precision);
            }
        } else {
            // 递归到子节点
            refineForPrecision(node, tool_aabb, config_.target_precision);
        }
    }
    
    // 5. 收集所有需要更新的Leaf节点
    std::vector<LeafNode*> leaf_nodes;
    collectLeaves(affected_nodes, leaf_nodes);
    
    // 6. 调度器选择后端并执行切削
    TaskDesc task;
    task.voxel_count = estimateVoxelCount(leaf_nodes);
    task.memory_required = estimateMemory(leaf_nodes);
    task.requires_realtime = config_.realtime_mode;
    
    Backend backend = scheduler_->selectBackend(task);
    
    if (backend == BACKEND_GPU_CUDA || backend == BACKEND_GPU_METAL) {
        executeCutGPU(leaf_nodes, tool, backend);
    } else {
        executeCutCPU(leaf_nodes, tool, backend);
    }
    
    // 7. 局部粗化（释放远离切削区的内存）
    if (config_.enable_coarsening && frame_count_ % config_.coarsen_interval == 0) {
        AABB recent_cut_region = getRecentCutRegion();
        coarsenDistantRegions(recent_cut_region, config_.coarsen_distance);
    }
    
    // 8. 更新时间
    current_time_ += dt;
    frame_count_++;
}

// 局部细化递归
void IncrementalTopologyManager::refineForPrecision(
    HybridNode* node, 
    const AABB& target_region,
    float target_voxel_size
) {
    if (!node->bounds().intersects(target_region)) return;
    
    if (node->type() == NODE_LEAF_CPU || node->type() == NODE_LEAF_GPU) {
        // Leaf节点：检查是否需要细分
        if (node->voxelSize() > target_voxel_size * 2.0f) {
            subdivideLeaf(node, target_voxel_size);
        }
        return;
    }
    
    // 内部节点：递归到子节点
    for (int i = 0; i < node->childCount(); i++) {
        HybridNode* child = node->child(i);
        if (child) {
            refineForPrecision(child, target_region, target_voxel_size);
        }
    }
}

// CPU切削执行
void IncrementalTopologyManager::executeCutCPU(
    std::vector<LeafNode*>& leaves,
    const Tool& tool,
    Backend backend
) {
    // 根据后端选择SIMD级别
    int simd_width = (backend == BACKEND_CPU_AVX512) ? 16 :
                     (backend == BACKEND_CPU_AVX2) ? 8 : 1;
    
    #pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < static_cast<int>(leaves.size()); i++) {
        LeafNode* leaf = leaves[i];
        
        // 遍历Leaf内体素
        for (int vi = 0; vi < leaf->voxelCount(); vi += simd_width) {
            int batch_size = std::min(simd_width, leaf->voxelCount() - vi);
            
            // 收集位置
            Vec3d positions[16];  // 最大AVX-512
            for (int j = 0; j < batch_size; j++) {
                positions[j] = leaf->voxelPosition(vi + j);
            }
            
            // 批量SDF查询（SIMD）
            float tool_sdfs[16];
            tool.sdfBatch(positions, tool_sdfs, batch_size);
            
            // 更新体素
            for (int j = 0; j < batch_size; j++) {
                CutVoxelValue& voxel = leaf->value(vi + j);
                
                if (tool_sdfs[j] < 0) {
                    // 刀具内部：切削
                    float removed = estimateRemovedVolume(
                        positions[j], leaf->voxelSize(), tool
                    );
                    voxel.cut(removed, leaf->voxelVolume());
                }
                
                // 更新SDF（与刀具SDF取min）
                voxel.updateSDF(tool_sdfs[j]);
            }
        }
        
        // 标记Leaf已修改（用于后续粗化判断）
        leaf->setModified(true);
    }
}

// GPU切削执行
void IncrementalTopologyManager::executeCutGPU(
    std::vector<LeafNode*>& leaves,
    const Tool& tool,
    Backend backend
) {
    // 1. 准备GPU数据：将Leaf节点线性化为连续数组
    GPULinearData gpu_data;
    linearizeLeaves(leaves, gpu_data);
    
    // 2. 上传刀具参数（常量内存）
    uploadToolParams(tool);
    
    // 3. 启动Kernel
    if (backend == BACKEND_GPU_CUDA) {
        launchCutKernelCUDA(gpu_data);
    } else if (backend == BACKEND_GPU_METAL) {
        launchCutKernelMetal(gpu_data);
    }
    
    // 4. 下载结果
    downloadResults(gpu_data, leaves);
    
    // 5. 清理GPU内存
    freeGPUMemory(gpu_data);
}
```

### 4.2 自适应SDF管理

```cpp
// AdaptiveSDFManager::update
void AdaptiveSDFManager::update(
    const Tool& tool,
    float dt,
    const std::vector<LeafNode*>& affected_leaves
) {
    // 决策：快速更新 vs 完全重初始化
    bool need_full_reinit = false;
    
    if (steps_since_reinit_ > config_.max_steps_before_reinit) {
        need_full_reinit = true;
    }
    
    if (topology_changed_significantly_) {
        need_full_reinit = true;
    }
    
    if (checkSDFQuality() < config_.quality_threshold) {
        need_full_reinit = true;
    }
    
    if (need_full_reinit) {
        fullReinitialize();
        steps_since_reinit_ = 0;
        topology_changed_significantly_ = false;
    } else {
        fastUpdate(tool, affected_leaves);
        steps_since_reinit_++;
    }
}

// 快速更新（论文思想：增量min）
void AdaptiveSDFManager::fastUpdate(
    const Tool& tool,
    const std::vector<LeafNode*>& leaves
) {
    // 理论：SDF_new = min(SDF_old, SDF_tool)
    // 仅影响刀具附近，计算复杂度 O(N_active)
    
    for (auto* leaf : leaves) {
        for (int i = 0; i < leaf->voxelCount(); i++) {
            Vec3d pos = leaf->voxelPosition(i);
            float tool_sdf = tool.sdf(pos);
            
            CutVoxelValue& voxel = leaf->value(i);
            
            // 仅当刀具SDF更负时才更新（优化：避免不必要的写）
            if (tool_sdf < voxel.sdf()) {
                voxel.updateSDF(tool_sdf);
            }
        }
    }
}

// 完全重初始化（OpenVDB思想：Fast Sweeping）
void AdaptiveSDFManager::fullReinitialize() {
    // 从当前体积分数重建精确SDF
    // 使用Fast Sweeping Method（FSM）
    
    // 1. 初始化：材料内部 = +1，空气 = -1
    initializeFromVolumeFraction();
    
    // 2. 多方向扫描（6个方向）
    for (int sweep = 0; sweep < config_.fsm_sweeps; sweep++) {
        sweepXPositive();
        sweepXNegative();
        sweepYPositive();
        sweepYNegative();
        sweepZPositive();
        sweepZNegative();
    }
    
    // 3. 验证：检查SDF与体积分数一致性
    validateSDFConsistency();
}
```

### 4.3 多精度表面提取

```cpp
// MultiQualityExtractor::extract
Mesh MultiQualityExtractor::extract(
    const HybridTree& tree,
    SurfaceQuality quality
) const {
    switch (quality) {
        case FAST_PREVIEW:
            return extractFastPreview(tree);
        case STANDARD:
            return extractStandard(tree);
        case HIGH_PRECISION:
            return extractHighPrecision(tree);
        case CMM_ACCURATE:
            return extractCMMAccurate(tree);
        default:
            return extractStandard(tree);
    }
}

// 快速预览：标准Marching Cubes
Mesh MultiQualityExtractor::extractFastPreview(const HybridTree& tree) const {
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    
    // 遍历所有Leaf节点
    tree.foreachLeaf([&](const LeafNode& leaf) {
        // 标准MC：每个体素生成三角形
        for (int z = 0; z < leaf.sizeZ() - 1; z++) {
            for (int y = 0; y < leaf.sizeY() - 1; y++) {
                for (int x = 0; x < leaf.sizeX() - 1; x++) {
                    processCellMC(leaf, x, y, z, vertices, indices, false);
                }
            }
        }
    });
    
    return Mesh(vertices, indices);
}

// 标准质量：改进MC + SDF插值
Mesh MultiQualityExtractor::extractStandard(const HybridTree& tree) const {
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    
    tree.foreachLeaf([&](const LeafNode& leaf) {
        for (int z = 0; z < leaf.sizeZ() - 1; z++) {
            for (int y = 0; y < leaf.sizeY() - 1; y++) {
                for (int x = 0; x < leaf.sizeX() - 1; x++) {
                    // 改进：使用SDF插值定位精确表面
                    processCellMC(leaf, x, y, z, vertices, indices, true);
                }
            }
        }
    });
    
    return Mesh(vertices, indices);
}

// 高精度：Dual Contouring + 体积分数修正
Mesh MultiQualityExtractor::extractHighPrecision(const HybridTree& tree) const {
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    
    // Dual Contouring：在体素角点生成顶点
    // 优势：更好地保留锐利特征
    
    tree.foreachLeaf([&](const LeafNode& leaf) {
        for (int z = 0; z < leaf.sizeZ(); z++) {
            for (int y = 0; y < leaf.sizeY(); y++) {
                for (int x = 0; x < leaf.sizeX(); x++) {
                    // 检查角点是否为特征点
                    if (isFeaturePoint(leaf, x, y, z)) {
                        Vertex v = computeFeatureVertex(leaf, x, y, z);
                        vertices.push_back(v);
                    }
                }
            }
        }
    });
    
    // 生成四边形/三角形
    generateDualFaces(tree, vertices, indices);
    
    return Mesh(vertices, indices);
}

// CMM精度：子像素精确提取
Mesh MultiQualityExtractor::extractCMMAccurate(const HybridTree& tree) const {
    // 1. 先提取高分辨率网格
    Mesh high_res = extractHighPrecision(tree);
    
    // 2. 自适应细分：在曲率高的区域细分
    Mesh adaptive = adaptiveSubdivide(high_res, [](const Face& f) {
        return computeCurvature(f) > curvature_threshold_;
    });
    
    // 3. 投影到精确SDF表面
    projectToSDFSurface(adaptive, tree);
    
    // 4. 误差控制：确保顶点在SDF=0的等值面上
    for (auto& v : adaptive.vertices) {
        float sdf = tree.getSDF(v.position);
        if (std::abs(sdf) > 1e-6) {
            // 沿梯度方向修正到表面
            Vec3d normal = tree.getNormal(v.position);
            v.position -= normal * sdf;
        }
    }
    
    return adaptive;
}

// 改进的MC单元处理
void MultiQualityExtractor::processCellMC(
    const LeafNode& leaf,
    int x, int y, int z,
    std::vector<Vertex>& vertices,
    std::vector<uint32_t>& indices,
    bool use_sdf_interpolation
) {
    // 获取8个角点的SDF值
    float sdf[8];
    for (int i = 0; i < 8; i++) {
        int dx = (i & 1) ? 1 : 0;
        int dy = (i & 2) ? 1 : 0;
        int dz = (i & 4) ? 1 : 0;
        sdf[i] = leaf.value(x + dx, y + dy, z + dz).sdf();
    }
    
    // 确定MC表索引
    int mc_index = 0;
    for (int i = 0; i < 8; i++) {
        if (sdf[i] < 0) mc_index |= (1 << i);
    }
    
    if (mc_index == 0 || mc_index == 255) return;  // 全内部或全外部
    
    // 生成三角形
    int num_tris = MC_TriangleCount[mc_index];
    for (int t = 0; t < num_tris; t++) {
        int3 edges = MC_TriTable[mc_index][t];
        
        Vertex v[3];
        for (int e = 0; e < 3; e++) {
            int edge_idx = edges[e];
            
            if (use_sdf_interpolation) {
                // 改进：沿边线性插值找到精确SDF=0位置
                v[e] = interpolateVertexSDF(leaf, x, y, z, edge_idx, sdf);
            } else {
                // 标准：中点
                v[e] = interpolateVertexMidpoint(leaf, x, y, z, edge_idx);
            }
        }
        
        // 添加三角形
        uint32_t base = static_cast<uint32_t>(vertices.size());
        vertices.push_back(v[0]);
        vertices.push_back(v[1]);
        vertices.push_back(v[2]);
        indices.push_back(base);
        indices.push_back(base + 1);
        indices.push_back(base + 2);
    }
}

// SDF插值顶点定位
Vertex MultiQualityExtractor::interpolateVertexSDF(
    const LeafNode& leaf,
    int x, int y, int z,
    int edge_idx,
    const float sdf[8]
) const {
    // 边的两个端点
    int v1 = MC_EdgeVertices[edge_idx][0];
    int v2 = MC_EdgeVertices[edge_idx][1];
    
    float s1 = sdf[v1];
    float s2 = sdf[v2];
    
    // 线性插值找到SDF=0的位置
    float t = s1 / (s1 - s2);  // t ∈ [0,1]
    t = math::Clamp(t, 0.0f, 1.0f);
    
    // 计算世界坐标
    Vec3d p1 = leaf.voxelPosition(x + ((v1 & 1) ? 1 : 0),
                                   y + ((v1 & 2) ? 1 : 0),
                                   z + ((v1 & 4) ? 1 : 0));
    Vec3d p2 = leaf.voxelPosition(x + ((v2 & 1) ? 1 : 0),
                                   y + ((v2 & 2) ? 1 : 0),
                                   z + ((v2 & 4) ? 1 : 0));
    
    Vec3d pos = math::Lerp(p1, p2, t);
    
    // 计算法向（从SDF梯度）
    Vec3d normal = computeSDFGradient(leaf, pos);
    normal.normalize();
    
    return Vertex(pos, normal);
}
```

---

## 5. 实现路线图

### 5.1 Phase 1: 基础设施（3个月）

**目标**: 在OpenVDB中注册CutVoxelValue类型，实现基础序列化

| 任务 | 工作量 | 依赖 | 交付物 |
|:---|:---|:---|:---|
| CutVoxelValue类型实现 | 2周 | 无 | 值类型 + 测试 |
| TypedGrid特化 | 2周 | CutVoxelValue | CutVoxelGrid |
| 序列化/反序列化 | 2周 | TypedGrid | .vdb读写 |
| 基础工具类 | 2周 | 无 | Tool, ToolPath |
| 单元测试 | 2周 | 全部 | 测试覆盖率>80% |

**里程碑**: 可以创建、存储、读取CutVoxelGrid

### 5.2 Phase 2: 混合树与拓扑（3个月）

**目标**: 实现HybridTree，支持局部细化/粗化

| 任务 | 工作量 | 依赖 | 交付物 |
|:---|:---|:---|:---|
| HybridNode基类 | 2周 | Phase 1 | 多态节点 |
| B+TreeInternal实现 | 2周 | HybridNode | CPU优化节点 |
| OctreeInternal实现 | 2周 | HybridNode | GPU友好节点 |
| Leaf节点（CPU/GPU） | 2周 | HybridNode | 两种布局 |
| 局部细化算法 | 2周 | 全部节点 | Subdivide |
| 局部粗化算法 | 2周 | 全部节点 | Coarsen |
| 增量拓扑管理器 | 2周 | 细化/粗化 | IncrementalTopologyManager |
| 集成测试 | 2周 | 全部 | 性能基准 |

**里程碑**: 可以执行简单切削，内存占用可控

### 5.3 Phase 3: 硬件后端（2个月）

**目标**: 实现CPU AVX2/AVX512和GPU CUDA后端

| 任务 | 工作量 | 依赖 | 交付物 |
|:---|:---|:---|:---|
| UnifiedScheduler框架 | 2周 | Phase 2 | 调度器接口 |
| CPU AVX2后端 | 2周 | 调度器 | SIMD Kernel |
| CPU AVX512后端 | 1周 | AVX2 | 512-bit优化 |
| CUDA后端 | 3周 | 调度器 | GPU Kernel |
| 内存管理（统一） | 2周 | 全部后端 | 分配/释放/拷贝 |
| 后端自动选择 | 1周 | 全部 | 智能调度 |
| 性能对比测试 | 1周 | 全部 | 基准报告 |

**里程碑**: 自动选择最优后端，小规模CPU、大规模GPU

### 5.4 Phase 4: 算法增强（2个月）

**目标**: 实现自适应SDF、多精度提取、材料库

| 任务 | 工作量 | 依赖 | 交付物 |
|:---|:---|:---|:---|
| AdaptiveSDFManager | 2周 | Phase 2 | 增量/全量更新 |
| Fast Sweeping Method | 2周 | SDFManager | 重初始化 |
| MultiQualityExtractor | 3周 | Phase 2 | 4级质量提取 |
| Dual Contouring | 2周 | Extractor | 锐利特征保留 |
| 材料属性库 | 2周 | 无 | 常见材料数据库 |
| 力/热耦合接口 | 1周 | 材料库 | 扩展接口 |
| 集成验证 | 1周 | 全部 | 标准测试件 |

**里程碑**: 0.01mm精度验证通过

### 5.5 Phase 5: 工业验证（2个月）

**目标**: 与商业CAM对比，CMM测量验证

| 任务 | 工作量 | 依赖 | 交付物 |
|:---|:---|:---|:---|
| 标准测试件库 | 2周 | Phase 4 | NIST测试件 |
| 与Vericut对比 | 2周 | 测试件 | 精度对比报告 |
| 与NX CAM对比 | 2周 | 测试件 | 性能对比报告 |
| CMM测量实验 | 3周 | 实际加工 | 物理验证报告 |
| 文档与示例 | 2周 | 全部 | 用户手册 |
| 开源发布 | 1周 | 文档 | GitHub仓库 |

**里程碑**: 开源发布，精度验证报告

---

## 6. 风险评估与缓解

| 风险 | 概率 | 影响 | 缓解策略 |
|:---|:---|:---|:---|
| OpenVDB代码耦合度高 | 高 | 高 | 最小侵入式扩展，保持兼容 |
| GPU内存限制 | 中 | 高 | 动态粗化 + 延迟加载 |
| 精度验证失败 | 中 | 高 | 分阶段验证，及时调整 |
| 性能不达预期 | 中 | 中 | 多后端回退，持续优化 |
| 开源社区接受度 | 低 | 中 | 保持Apache 2.0，完整文档 |
| 商业软件诉讼 | 低 | 高 | 清洁室实现，专利审查 |

---

## 7. 预期成果与评估标准

### 7.1 技术成果

| 指标 | 当前最佳 | 目标 | 验证方法 |
|:---|:---|:---|:---|
| 精度 | 0.01mm (论文) | **0.005mm** | CMM测量 |
| 速度（1e6体素） | 5ms/步 (论文GPU) | **2ms/步** | 基准测试 |
| 速度（1e8体素） | 不可行 (CPU) | **50ms/步** | 基准测试 |
| 内存效率 | 10×节省 (OpenVDB) | **20×节省** | 内存分析 |
| 数据集规模 | TB级 (OpenVDB) | **TB级 + 实时** | 实际案例 |

### 7.2 开源成果

| 交付物 | 形式 | 许可 |
|:---|:---|:---|
| OpenVDB-CAM核心库 | C++库 | Apache 2.0 |
| Python绑定 | pyopenvdb扩展 | Apache 2.0 |
| 示例应用 | 命令行工具 | Apache 2.0 |
| 测试套件 | 自动化测试 | Apache 2.0 |
| 文档 | Markdown + Doxygen | CC-BY-SA |

---

## 8. 附录

### 8.1 与现有文档的关系

| 本文档 | 关系 | 对应文档 |
|:---|:---|:---|
| OpenVDB-CAM | 本提案 | `OpenVDB-CAM_Proposal.md` |
| GPU方案分析 | 基础分析 | `GPU_Voxel_Machining_Analysis.md` |
| CPU优化方案 | 扩展方案 | `CPU_Voxel_Machining_Analysis.md` |
| OpenVDB对比 | 对比分析 | `Paper_vs_OpenVDB_Comparison.md` |

### 8.2 参考资源

| 资源 | 链接 | 用途 |
|:---|:---|:---|
| OpenVDB官方 | https://www.openvdb.org | 基础库 |
| OpenVDB GitHub | https://github.com/AcademySoftwareFoundation/openvdb | 源码 |
| 论文原文 | `GPU_accelerated_voxel-based_machining_simulation.pdf` | 核心算法 |
| NIST测试件 | https://www.nist.gov/ctl/smart-manufacturing-systems | 验证标准 |
| ISO 10791 | 国际标准 | 机床精度标准 |

---

*本提案为OpenVDB-CAM项目的启动文档，后续将细化为设计文档、任务分解和代码实现。*
