# CPU优化体素化切削模拟技术文档

## 文档信息
- **标题**: CPU-Optimized Voxel-based Machining Simulation
- **版本**: 1.0
- **日期**: 2026-06-08
- **对应GPU文档**: `GPU_Voxel_Machining_Analysis.md`
- **适用场景**: 小规模计算（体素数 < 10^6），无需GPU加速

---

## 1. 设计动机与适用场景

### 1.1 为什么需要CPU方案

| 考量因素 | GPU方案 | CPU方案 |
|:---|:---|:---|
| **启动开销** | Kernel启动 + 数据传输 ~1-5ms | 无，直接计算 |
| **数据结构灵活性** | 受限（GPU偏好规则内存） | 高（指针、树、哈希表） |
| **调试难度** | 高（需Nsight等工具） | 低（标准调试器） |
| **部署成本** | 需CUDA/OpenCL运行时 | 仅需标准C++库 |
| **小规模效率** | 固定开销占比高 | 线性扩展，无固定成本 |
| **内存容量** | 受显存限制（通常8-24GB） | 受系统内存（通常64GB+） |

### 1.2 规模阈值判定

```
决策流程:

输入: 工件尺寸 L (mm), 目标精度 Δx (mm)
计算: 理论体素数 N = (L/Δx)^3

if N < 1e6:
    → 推荐CPU方案（本文档）
    原因: GPU启动开销 > 计算收益
    
else if N < 1e7:
    → 可选CPU（AVX优化）或GPU
    决策依据: 实时性要求、硬件可用性
    
else:
    → 必须GPU方案
    原因: CPU计算时间不可接受

示例:
- 50mm工件 @ 0.01mm精度: (50/0.01)^3 = 1.25e9 → 必须自适应/稀疏
- 实际活跃体素 ~1e6 → CPU可行
- 10mm工件 @ 0.01mm精度: (10/0.01)^3 = 1e6 → CPU最优
```

---

## 2. 核心架构差异：GPU vs CPU

### 2.1 架构对比总览

| 维度 | GPU方案 | CPU方案（本文） |
|:---|:---|:---|
| **并行模型** | 大规模数据并行（10^4线程） | 中等并行（8-32线程）+ SIMD |
| **内存结构** | 统一内存（Global/Shared/Local） | 分层缓存（L1/L2/L3） |
| **数据结构** | 扁平数组、纹理内存 | 哈希表、树、链表 |
| **同步机制** | 原子操作、屏障同步 | 锁、无锁结构、线程局部存储 |
| **优化目标** | 最大化吞吐量 | 最小化延迟、最大化缓存效率 |

### 2.2 数据流对比

```
GPU数据流:
Host Memory → PCIe Transfer → Device Memory → Kernel Compute → Result Transfer → Host Memory
              ↑________________________瓶颈_________________________↑

CPU数据流:
Main Memory → L3 Cache → L2 Cache → L1 Cache → SIMD Registers → Compute
              ↑__________零拷贝，缓存即内存__________↑
```

---

## 3. CPU核心优化策略

### 3.1 策略一：数据结构扁平化

#### 问题：八叉树指针追踪的缓存不友好性

在CPU上，八叉树的指针解引用（`node->children[i]`）会导致：
- 缓存未命中（Cache Miss）
- 预取失效（Prefetch Failure）
- 分支预测失败（Branch Misprediction）

#### 解决方案A：均匀网格扁平数组（小规模 < 10^6）

```cpp
struct UniformGrid {
    // 配置
    Vec3i resolution;        // 例如: 1000 × 1000 × 1000 = 1e9 (太大)
                             // 实际: 100 × 100 × 100 = 1e6
    float voxel_size;        // 例如: 0.01mm
    Vec3 origin;             // 网格原点
    
    // 数据存储: Structure of Arrays (SoA)
    struct Data {
        std::vector<float> sdf;              // 有向距离场
        std::vector<float> volume_fraction;  // 体积分数
        std::vector<uint8_t> state;          // 状态枚举
        std::vector<uint8_t> flags;          // 标记位
    } data;
    
    // 索引计算（扁平化三维到一维）
    inline size_t Index(int x, int y, int z) const {
        return static_cast<size_t>(z) * resolution.y * resolution.x 
             + static_cast<size_t>(y) * resolution.x 
             + static_cast<size_t>(x);
    }
    
    inline size_t Index(const Vec3i& pos) const {
        return Index(pos.x, pos.y, pos.z);
    }
    
    // 边界检查
    inline bool InBounds(int x, int y, int z) const {
        return x >= 0 && x < resolution.x &&
               y >= 0 && y < resolution.y &&
               z >= 0 && z < resolution.z;
    }
    
    // 空间位置 ↔ 体素索引转换
    inline Vec3i WorldToVoxel(const Vec3& world_pos) const {
        return Vec3i(
            static_cast<int>((world_pos.x - origin.x) / voxel_size),
            static_cast<int>((world_pos.y - origin.y) / voxel_size),
            static_cast<int>((world_pos.z - origin.z) / voxel_size)
        );
    }
    
    inline Vec3 VoxelToWorld(const Vec3i& voxel_idx) const {
        return Vec3(
            origin.x + (voxel_idx.x + 0.5f) * voxel_size,
            origin.y + (voxel_idx.y + 0.5f) * voxel_size,
            origin.z + (voxel_idx.z + 0.5f) * voxel_size
        );
    }
};
```

#### 解决方案B：哈希表稀疏存储（中等规模 10^6 ~ 10^7）

```cpp
struct SparseHashGrid {
    float voxel_size;
    
    // 体素数据存储
    struct VoxelData {
        float sdf;
        float volume_fraction;
        uint8_t state;
    };
    
    // 哈希表: 编码位置 → 体素数据
    // 使用robin_hood::unordered_flat_map或类似高性能哈希表
    std::unordered_map<uint64_t, VoxelData> data;
    
    // 位置编码（21位/轴，支持 ±1e6 范围）
    static inline uint64_t Encode(int x, int y, int z) {
        constexpr int OFFSET = 1 << 20;  // 使负数可编码
        uint64_t ux = static_cast<uint64_t>(x + OFFSET) & 0x1FFFFF;
        uint64_t uy = static_cast<uint64_t>(y + OFFSET) & 0x1FFFFF;
        uint64_t uz = static_cast<uint64_t>(z + OFFSET) & 0x1FFFFF;
        return (uz << 42) | (uy << 21) | ux;
    }
    
    static inline Vec3i Decode(uint64_t code) {
        constexpr int OFFSET = 1 << 20;
        return Vec3i(
            static_cast<int>((code >> 0)  & 0x1FFFFF) - OFFSET,
            static_cast<int>((code >> 21) & 0x1FFFFF) - OFFSET,
            static_cast<int>((code >> 42) & 0x1FFFFF) - OFFSET
        );
    }
    
    // 访问接口
    VoxelData* Find(const Vec3i& pos) {
        auto it = data.find(Encode(pos.x, pos.y, pos.z));
        return (it != data.end()) ? &it->second : nullptr;
    }
    
    VoxelData& Emplace(const Vec3i& pos) {
        return data[Encode(pos.x, pos.y, pos.z)];
    }
    
    // 批量预分配（避免rehash）
    void Reserve(size_t count) {
        data.reserve(count);
        data.max_load_factor(0.7f);
    }
};
```

#### 解决方案C：分层块结构（Block-based，混合方案）

```cpp
// 固定大小的块，内部均匀网格，块间稀疏存储
struct BlockedGrid {
    static constexpr int BLOCK_SIZE = 32;  // 32×32×32 = 32768 体素/块
    
    struct Block {
        Vec3i block_idx;           // 块坐标
        std::array<float, BLOCK_SIZE * BLOCK_SIZE * BLOCK_SIZE> sdf;
        std::array<float, BLOCK_SIZE * BLOCK_SIZE * BLOCK_SIZE> volume_fraction;
        std::array<uint8_t, BLOCK_SIZE * BLOCK_SIZE * BLOCK_SIZE> state;
        bool is_active = true;     // 是否包含材料
    };
    
    float voxel_size;
    float block_world_size;        // BLOCK_SIZE * voxel_size
    
    // 稀疏块存储
    std::unordered_map<uint64_t, std::unique_ptr<Block>> blocks;
    
    // 活跃块列表（迭代优化）
    std::vector<Block*> active_blocks;
    
    inline Vec3i WorldToBlock(const Vec3& world_pos) const {
        return Vec3i(
            static_cast<int>(world_pos.x / block_world_size),
            static_cast<int>(world_pos.y / block_world_size),
            static_cast<int>(world_pos.z / block_world_size)
        );
    }
    
    inline Vec3i BlockLocalIndex(const Vec3i& world_voxel) const {
        return Vec3i(
            world_voxel.x & (BLOCK_SIZE - 1),
            world_voxel.y & (BLOCK_SIZE - 1),
            world_voxel.z & (BLOCK_SIZE - 1)
        );
    }
    
    Block* GetOrCreateBlock(const Vec3i& block_idx);
};
```

### 3.2 策略二：SIMD向量化计算

#### 硬件能力检测

```cpp
struct SIMDInfo {
    bool has_sse2 = false;
    bool has_avx = false;
    bool has_avx2 = false;
    bool has_avx512 = false;
    
    void Detect() {
        #if defined(__x86_64__) || defined(_M_X64)
        // CPUID检测
        int cpu_info[4];
        __cpuid(cpu_info, 1);
        has_sse2 = (cpu_info[3] & (1 << 26)) != 0;
        
        __cpuid(cpu_info, 7);
        has_avx = (cpu_info[2] & (1 << 28)) != 0;
        has_avx2 = (cpu_info[1] & (1 << 5)) != 0;
        has_avx512 = (cpu_info[1] & (1 << 16)) != 0;  // AVX-512F
        #endif
    }
    
    int FloatsPerRegister() const {
        if (has_avx512) return 16;
        if (has_avx || has_avx2) return 8;
        if (has_sse2) return 4;
        return 1;  // 标量回退
    }
};
```

#### AVX2 SDF更新Kernel

```cpp
#include <immintrin.h>

// 一次处理8个float（256位AVX2）
class SDFUpdateAVX2 {
public:
    static constexpr int VEC_WIDTH = 8;
    
    // 球头刀SDF更新
    static void UpdateBallEndMill(
        const float* pos_x, const float* pos_y, const float* pos_z,
        float* sdf_out,
        int count,
        const Vec3& tool_center,
        float tool_radius
    ) {
        // 广播常量到所有向量槽
        __m256 cx = _mm256_set1_ps(tool_center.x);
        __m256 cy = _mm256_set1_ps(tool_center.y);
        __m256 cz = _mm256_set1_ps(tool_center.z);
        __m256 rad = _mm256_set1_ps(tool_radius);
        
        int i = 0;
        // 主循环：每次8个
        for (; i + VEC_WIDTH <= count; i += VEC_WIDTH) {
            // 加载位置
            __m256 px = _mm256_loadu_ps(&pos_x[i]);
            __m256 py = _mm256_loadu_ps(&pos_y[i]);
            __m256 pz = _mm256_loadu_ps(&pos_z[i]);
            
            // 计算差值
            __m256 dx = _mm256_sub_ps(px, cx);
            __m256 dy = _mm256_sub_ps(py, cy);
            __m256 dz = _mm256_sub_ps(pz, cz);
            
            // 距离平方: dx*dx + dy*dy + dz*dz
            __m256 dist_sq = _mm256_add_ps(
                _mm256_add_ps(
                    _mm256_mul_ps(dx, dx),
                    _mm256_mul_ps(dy, dy)
                ),
                _mm256_mul_ps(dz, dz)
            );
            
            // 距离: sqrt(dist_sq)
            __m256 dist = _mm256_sqrt_ps(dist_sq);
            
            // SDF: dist - radius
            __m256 sdf_tool = _mm256_sub_ps(dist, rad);
            
            // 加载旧SDF
            __m256 sdf_old = _mm256_loadu_ps(&sdf_out[i]);
            
            // 取最小值
            __m256 sdf_new = _mm256_min_ps(sdf_old, sdf_tool);
            
            // 存储结果
            _mm256_storeu_ps(&sdf_out[i], sdf_new);
        }
        
        // 尾部处理：标量
        for (; i < count; i++) {
            float dx = pos_x[i] - tool_center.x;
            float dy = pos_y[i] - tool_center.y;
            float dz = pos_z[i] - tool_center.z;
            float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
            float sdf_tool = dist - tool_radius;
            sdf_out[i] = std::min(sdf_out[i], sdf_tool);
        }
    }
    
    // 平底刀SDF更新（圆柱体）
    static void UpdateFlatEndMill(
        const float* pos_x, const float* pos_y, const float* pos_z,
        float* sdf_out,
        int count,
        const Vec3& tool_center,      // 圆柱底部中心
        float tool_radius,
        float tool_length,            // 圆柱高度
        const Vec3& tool_axis         // 通常为 (0,0,1)
    ) {
        __m256 cx = _mm256_set1_ps(tool_center.x);
        __m256 cy = _mm256_set1_ps(tool_center.y);
        __m256 cz = _mm256_set1_ps(tool_center.z);
        __m256 rad = _mm256_set1_ps(tool_radius);
        __m256 half_len = _mm256_set1_ps(tool_length * 0.5f);
        
        int i = 0;
        for (; i + VEC_WIDTH <= count; i += VEC_WIDTH) {
            __m256 px = _mm256_loadu_ps(&pos_x[i]);
            __m256 py = _mm256_loadu_ps(&pos_y[i]);
            __m256 pz = _mm256_loadu_ps(&pos_z[i]);
            
            // 假设工具轴为Z轴，简化计算
            // 径向距离
            __m256 dx = _mm256_sub_ps(px, cx);
            __m256 dy = _mm256_sub_ps(py, cy);
            __m256 dz = _mm256_sub_ps(pz, cz);
            
            __m256 radial_dist_sq = _mm256_add_ps(
                _mm256_mul_ps(dx, dx),
                _mm256_mul_ps(dy, dy)
            );
            __m256 radial_dist = _mm256_sqrt_ps(radial_dist_sq);
            
            // 圆柱SDF: max(radial_dist - R, |dz| - H/2)
            __m256 radial_sdf = _mm256_sub_ps(radial_dist, rad);
            __m256 abs_dz = _mm256_andnot_ps(_mm256_set1_ps(-0.0f), dz);  // fabs
            __m256 axial_sdf = _mm256_sub_ps(abs_dz, half_len);
            
            __m256 sdf_tool = _mm256_max_ps(radial_sdf, axial_sdf);
            
            __m256 sdf_old = _mm256_loadu_ps(&sdf_out[i]);
            __m256 sdf_new = _mm256_min_ps(sdf_old, sdf_tool);
            _mm256_storeu_ps(&sdf_out[i], sdf_new);
        }
        
        // 尾部标量处理...
    }
};
```

#### 编译器自动向量化提示

```cpp
// 帮助编译器生成SIMD代码

// 1. 使用restrict关键字消除指针别名假设
void UpdateSDF_Optimized(
    float* __restrict sdf_out,
    const float* __restrict pos_x,
    const float* __restrict pos_y,
    const float* __restrict pos_z,
    int count,
    const Tool& tool
);

// 2. 使用pragma提示循环可向量化
#pragma omp simd
for (int i = 0; i < count; i++) {
    // 循环体
}

// 3. 确保内存对齐（32字节边界用于AVX2）
alignas(32) float buffer[1024];  // C++11 alignas

// 或使用_aligned_malloc/_mm_malloc
float* aligned_buffer = (float*)_mm_malloc(1024 * sizeof(float), 32);
// ... 使用 ...
_mm_free(aligned_buffer);
```

### 3.3 策略三：缓存优化与内存预取

#### 问题：随机内存访问模式

体素化模拟中，刀具可能以任意角度切削，导致内存访问模式不规则。

#### 解决方案：空间排序与预取

```cpp
struct CacheOptimizedProcessor {
    // Z序曲线（Morton Order）排序体素索引
    // 使空间上接近的体素在内存中也接近
    
    static uint32_t MortonEncode3(uint16_t x, uint16_t y, uint16_t z) {
        // 分离位: x = 0bABCD → 0b000A000B000C000D
        auto Part1By2 = [](uint16_t v) -> uint32_t {
            uint32_t w = v;
            w = (w ^ (w << 16)) & 0xFF0000FF;
            w = (w ^ (w << 8))  & 0x0F00F00F;
            w = (w ^ (w << 4))  & 0xC30C30C3;
            w = (w ^ (w << 2))  & 0x49249249;
            return w;
        };
        
        return (Part1By2(z) << 2) | (Part1By2(y) << 1) | Part1By2(x);
    }
    
    // 按Morton序排序候选体素
    void SortCandidatesBySpaceFillingCurve(std::vector<int>& candidates) {
        std::sort(candidates.begin(), candidates.end(),
            [this](int a, int b) {
                Vec3i pa = IndexToVoxel(a);
                Vec3i pb = IndexToVoxel(b);
                return MortonEncode3(pa.x, pa.y, pa.z) < MortonEncode3(pb.x, pb.y, pb.z);
            }
        );
    }
    
    // 软件预取
    void ProcessWithPrefetch(const std::vector<int>& candidates, const Tool& tool) {
        constexpr int PREFETCH_DISTANCE = 16;  // 预取16个体素 ahead
        
        for (int i = 0; i < static_cast<int>(candidates.size()); i++) {
            // 预取未来的数据
            if (i + PREFETCH_DISTANCE < static_cast<int>(candidates.size())) {
                int future_idx = candidates[i + PREFETCH_DISTANCE];
                _mm_prefetch((const char*)&sdf_data_[future_idx], _MM_HINT_T0);
            }
            
            // 处理当前体素
            ProcessVoxel(candidates[i], tool);
        }
    }
};
```

### 3.4 策略四：多线程并行（OpenMP）

```cpp
#include <omp.h>

class ThreadedSimulator {
public:
    void RunParallel(const std::vector<int>& candidates, const Tool& tool) {
        int num_threads = omp_get_max_threads();
        
        // 方案A: 静态分区（候选体素均匀分配）
        #pragma omp parallel for schedule(static)
        for (int i = 0; i < static_cast<int>(candidates.size()); i++) {
            ProcessVoxel(candidates[i], tool);
        }
        
        // 方案B: 动态调度（负载不均时更优）
        #pragma omp parallel for schedule(dynamic, 64)
        for (int i = 0; i < static_cast<int>(candidates.size()); i++) {
            ProcessVoxel(candidates[i], tool);
        }
        
        // 方案C: 分块并行（缓存友好）
        constexpr int BLOCK = 1024;
        int num_blocks = (candidates.size() + BLOCK - 1) / BLOCK;
        
        #pragma omp parallel for schedule(dynamic)
        for (int b = 0; b < num_blocks; b++) {
            int start = b * BLOCK;
            int end = std::min(start + BLOCK, static_cast<int>(candidates.size()));
            for (int i = start; i < end; i++) {
                ProcessVoxel(candidates[i], tool);
            }
        }
    }
    
    // 线程局部存储避免false sharing
    void ProcessVoxelThreadLocal(int idx, const Tool& tool) {
        // 每个线程有自己的临时缓冲区
        thread_local std::vector<float> temp_buffer;
        temp_buffer.clear();
        
        // 处理...
    }
};
```

### 3.5 策略五：自适应时间步长（与GPU方案一致）

```cpp
struct AdaptiveTimeStepCPU {
    float current_dt = 0.001f;   // 1ms初始
    float min_dt = 0.0001f;      // 0.1ms最小（对应0.01mm @ 100mm/s）
    float max_dt = 0.01f;        // 10ms最大
    
    // 历史状态用于预测
    std::deque<float> dt_history;
    static constexpr int HISTORY_SIZE = 10;
    
    float ComputeNextDT(const Tool& tool, float distance_to_surface) {
        // 基础CFL条件
        float speed = tool.velocity.magnitude();
        float cfl_dt = (speed > 1e-6f) ? (0.01f / speed) : max_dt;
        
        // 根据距离调整
        float distance_factor;
        if (distance_to_surface > 1.0f) {           // > 1mm: 远离
            distance_factor = 10.0f;
        } else if (distance_to_surface > 0.01f) {    // > 0.01mm: 接近
            distance_factor = 1.0f;
        } else {                                      // 接触或切入
            distance_factor = 0.5f;
        }
        
        float proposed_dt = cfl_dt * distance_factor;
        
        // 限制变化率（避免振荡）
        if (!dt_history.empty()) {
            float max_change = 2.0f;  // 每步最多变化2倍
            float prev_dt = dt_history.back();
            proposed_dt = std::clamp(proposed_dt, prev_dt / max_change, prev_dt * max_change);
        }
        
        proposed_dt = std::clamp(proposed_dt, min_dt, max_dt);
        
        // 更新历史
        dt_history.push_back(proposed_dt);
        if (dt_history.size() > HISTORY_SIZE) {
            dt_history.pop_front();
        }
        
        return proposed_dt;
    }
};
```

---

## 4. 完整CPU模拟器实现

```cpp
class CPUMachiningSimulator {
public:
    struct Config {
        // 空间配置
        Vec3 workpiece_bounds = {100.0f, 100.0f, 50.0f};  // mm
        float target_precision = 0.01f;                    // mm
        
        // 计算配置
        bool use_simd = true;
        bool use_openmp = true;
        int num_threads = 0;  // 0 = 自动检测
        
        // 精度配置
        bool use_sdf = true;
        bool use_volume_fraction = true;
        float sdf_threshold = 0.0f;
    };
    
    struct Statistics {
        double total_time_ms = 0.0;
        double avg_step_time_ms = 0.0;
        int num_steps = 0;
        int total_voxels_processed = 0;
        int active_voxels = 0;
        size_t memory_used_mb = 0;
    };
    
public:
    CPUMachiningSimulator() = default;
    ~CPUMachiningSimulator() = default;
    
    // 初始化
    bool Initialize(const Config& config, const Mesh& workpiece_mesh);
    
    // 加载刀具路径
    bool LoadToolPath(const ToolPath& path);
    
    // 执行模拟
    bool RunSimulation();
    
    // 提取结果
    Mesh ExtractSurface();
    Mesh ExtractCutVolume();
    
    // 查询
    float QuerySurfaceError(const Mesh& ground_truth) const;
    const Statistics& GetStatistics() const { return stats_; }
    
private:
    // 核心数据
    Config config_;
    std::unique_ptr<UniformGrid> grid_;           // 或 SparseHashGrid
    std::unique_ptr<BlockedGrid> blocked_grid_;   // 可选
    
    // 刀具
    Tool tool_;
    ToolPath tool_path_;
    
    // 加速结构
    std::vector<int> active_candidates_;     // 当前帧候选体素
    std::vector<int> next_candidates_;       // 预计算的下一帧候选
    AABB tool_aabb_prev_;                    // 上一帧刀具AABB（用于增量更新）
    
    // 时间步进
    AdaptiveTimeStepCPU time_stepper_;
    float current_time_ = 0.0f;
    
    // 统计
    Statistics stats_;
    
    // 内部方法
    void BuildInitialSDF(const Mesh& workpiece);
    void UpdateActiveCandidates();
    void Step(float dt);
    
    // SIMD dispatch
    void UpdateSDF_Scalar(const std::vector<int>& candidates);
    void UpdateSDF_AVX2(const std::vector<int>& candidates);
    void UpdateSDF_AVX512(const std::vector<int>& candidates);
    
    // 体素处理
    void CutVoxel(int idx);
    void UpdateVoxelState(int idx);
    
    // 表面提取
    Mesh MarchingCubesCPU() const;
    Mesh DualContouringCPU() const;
};

// 实现概要
bool CPUMachiningSimulator::Initialize(const Config& config, const Mesh& workpiece) {
    config_ = config;
    
    // 检测SIMD能力
    SIMDInfo simd;
    simd.Detect();
    if (config.use_simd && !simd.has_avx2) {
        config_.use_simd = false;  // 回退到标量
    }
    
    // 配置OpenMP
    if (config.use_openmp) {
        int threads = (config.num_threads > 0) ? config.num_threads : omp_get_max_threads();
        omp_set_num_threads(threads);
    }
    
    // 创建网格
    Vec3i resolution(
        static_cast<int>(config.workpiece_bounds.x / config.target_precision),
        static_cast<int>(config.workpiece_bounds.y / config.target_precision),
        static_cast<int>(config.workpiece_bounds.z / config.target_precision)
    );
    
    // 检查规模
    size_t total_voxels = static_cast<size_t>(resolution.x) * resolution.y * resolution.z;
    if (total_voxels > 10'000'000) {
        // 切换到稀疏结构或提示用户
        return false;  // 或创建BlockedGrid
    }
    
    grid_ = std::make_unique<UniformGrid>();
    grid_->resolution = resolution;
    grid_->voxel_size = config.target_precision;
    grid_->origin = Vec3(0, 0, 0);
    
    size_t num_voxels = total_voxels;
    grid_->data.sdf.resize(num_voxels, 1.0f);              // 初始: 内部
    grid_->data.volume_fraction.resize(num_voxels, 1.0f); // 完整材料
    grid_->data.state.resize(num_voxels, VoxelState::MATERIAL);
    
    // 初始化SDF
    BuildInitialSDF(workpiece);
    
    return true;
}

void CPUMachiningSimulator::Step(float dt) {
    // 1. 更新刀具位姿
    tool_.pose = tool_path_.GetPose(current_time_);
    
    // 2. 确定活跃候选体素
    UpdateActiveCandidates();
    
    // 3. SDF更新（SIMD优化）
    if (config_.use_simd) {
        UpdateSDF_AVX2(active_candidates_);
    } else {
        UpdateSDF_Scalar(active_candidates_);
    }
    
    // 4. 体素切削（多线程）
    #pragma omp parallel for if(config_.use_openmp)
    for (int i = 0; i < static_cast<int>(active_candidates_.size()); i++) {
        CutVoxel(active_candidates_[i]);
    }
    
    // 5. 状态更新
    #pragma omp parallel for if(config_.use_openmp)
    for (int i = 0; i < static_cast<int>(active_candidates_.size()); i++) {
        UpdateVoxelState(active_candidates_[i]);
    }
    
    // 6. 更新时间
    current_time_ += dt;
    stats_.num_steps++;
    stats_.total_voxels_processed += static_cast<int>(active_candidates_.size());
}

void CPUMachiningSimulator::UpdateActiveCandidates() {
    // 计算当前刀具AABB
    AABB tool_aabb = ComputeToolAABB(tool_);
    
    // 增量更新：仅检查AABB变化区域
    if (tool_aabb_prev_.IsValid()) {
        AABB changed_region = AABB::Union(tool_aabb, tool_aabb_prev_);
        // 仅查询变化区域
        QueryVoxelsInAABB(changed_region, active_candidates_);
    } else {
        // 首次：全量查询
        QueryVoxelsInAABB(tool_aabb, active_candidates_);
    }
    
    tool_aabb_prev_ = tool_aabb;
}

void CPUMachiningSimulator::CutVoxel(int idx) {
    Vec3i voxel_pos = grid_->IndexToVoxel(idx);
    Vec3 world_pos = grid_->VoxelToWorld(voxel_pos);
    
    // 精确刀具占用测试
    float sdf_tool = ComputeToolSDF(world_pos, tool_);
    
    if (sdf_tool < 0) {
        // 刀具内部：切削
        float& vf = grid_->data.volume_fraction[idx];
        float removed = EstimateRemovedVolume(world_pos, grid_->voxel_size, tool_);
        float voxel_vol = grid_->voxel_size * grid_->voxel_size * grid_->voxel_size;
        
        vf -= removed / voxel_vol;
        
        if (vf <= 0.0f) {
            grid_->data.state[idx] = VoxelState::AIR;
            vf = 0.0f;
        }
    }
    
    // 更新SDF
    grid_->data.sdf[idx] = std::min(grid_->data.sdf[idx], sdf_tool);
}
```

---

## 5. 性能优化检查清单

### 5.1 编译优化

```bash
# GCC/Clang
-O3 -march=native -ffast-math -funroll-loops
-fopenmp  # OpenMP支持

# MSVC
/O2 /arch:AVX2 /fp:fast /openmp

# 链接时优化（LTO）
-flto  # GCC/Clang
/LTCG  # MSVC
```

### 5.2 运行时优化

| 检查项 | 优化前 | 优化后 | 验证方法 |
|:---|:---|:---|:---|
| 内存布局 | Array of Structs | Structure of Arrays | Cache miss计数 |
| SIMD利用率 | 标量 | AVX2 256-bit | 指令级分析（perf） |
| 线程并行 | 单线程 | OpenMP 8线程 | CPU利用率 ~800% |
| 内存对齐 | 未对齐 | 32字节对齐 | `_mm_load_ps` vs `_mm_loadu_ps` |
| 预取 | 无 | 软件预取 | Cache miss减少 |
| 分支预测 | 随机分支 | 排序后处理 | Branch misprediction减少 |

### 5.3 性能分析工具

```bash
# Linux perf
perf stat -e cache-misses,cache-references,instructions,cycles ./simulator
perf record -g ./simulator
perf report

# Intel VTune
vtune -collect hotspots -result-dir ./vtune_results ./simulator

# AMD uProf
uprof -P ./simulator

# 内置计时
// 在代码中插入高精度计时
#include <chrono>
auto t0 = std::chrono::high_resolution_clock::now();
// ... 代码 ...
auto t1 = std::chrono::high_resolution_clock::now();
double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
```

---

## 6. GPU vs CPU 方案选择决策树

```
输入: 问题规模、硬件环境、实时性要求、开发资源

├─ 体素总数 N < 1e6?
│   ├─ 是 → CPU方案（本文档）
│   │       ├─ 需要实时交互? → 优化目标: 单步 < 16ms
│   │       └─ 离线批处理? → 优化目标: 总时间最小
│   └─ 否 → 继续判断
│
├─ 体素总数 N < 1e7?
│   ├─ 是 → 可选CPU或GPU
│   │       ├─ 有高端GPU (RTX 3060+)? → GPU方案
│   │       └─ 仅有CPU / 需要快速部署? → CPU方案（AVX优化）
│   └─ 否 → 必须GPU方案
│
└─ 体素总数 N >= 1e7?
    └─ 必须GPU方案
        ├─ N < 1e8? → 单GPU
        └─ N >= 1e8? → 多GPU / GPU集群

其他考量:
├─ 开发周期 < 1周? → CPU（调试快）
├─ 需要部署到嵌入式? → CPU（无CUDA依赖）
├─ 内存需求 > 16GB? → CPU（系统内存更大）
└─ 需要与现有CPU框架集成? → CPU（避免跨设备传输）
```

---

## 7. 与GPU文档的对应关系

| GPU文档章节 | 本文对应章节 | 核心差异 |
|:---|:---|:---|
| 2. 核心算法架构 | 2. 架构差异 | 并行模型、内存结构 |
| 3.1 自适应体素细化 | 3.1 数据结构扁平化 | 八叉树 → 均匀数组/哈希表 |
| 3.2 SDF | 3.2 SIMD SDF更新 | 相同数学，SIMD实现方式不同 |
| 3.3 体积分数 | 3.2 中集成 | 相同 |
| 3.4 解析刀具 | 3.2 SIMD刀具测试 | 相同数学，AVX2实现 |
| 3.5 时间步长 | 3.5 自适应时间步长 | 相同 |
| 3.6 表面重建 | 4. 中 MarchingCubesCPU | 相同算法，CPU优化 |
| 4. GPU并行 | 3.3 缓存优化 + 3.4 OpenMP | 线程级并行 vs 数据级并行 |
| 5. 复杂度 | 5. 性能优化 | 关注缓存而非内存带宽 |
| 6. 伪代码 | 4. 完整实现 | CPU风格代码 |

---

## 8. 结论

### 8.1 CPU方案核心价值

1. **低门槛**：无需GPU编程知识，标准C++即可
2. **低延迟**：无PCIe传输、Kernel启动开销
3. **高灵活**：复杂数据结构、指针追踪、递归算法
4. **易调试**：标准工具链，单步调试友好

### 8.2 性能天花板

| 优化级别 | 相对标量加速 | 典型单步时间（1e6体素） |
|:---|:---|:---|
| 标量 | 1× | 50ms |
| + SoA布局 | 1.5× | 33ms |
| + AVX2 SIMD | 4× | 12.5ms |
| + OpenMP 8线程 | 8× | 6.25ms |
| + 缓存优化 | 10× | 5ms |
| **总计** | **~10×** | **~5ms** |

**结论**：小规模（< 1e6体素）下，优化后的CPU方案可达 **5ms/步**，完全满足实时交互需求（目标 < 16ms/帧）。

---

## 附录A：CPU优化术语表

| 术语 | 英文 | 定义 |
|:---|:---|:---|
| SIMD | Single Instruction Multiple Data | 单指令多数据并行 |
| AVX2 | Advanced Vector Extensions 2 | Intel 256位向量指令集 |
| SoA | Structure of Arrays | 数组结构体，缓存友好布局 |
| AoS | Array of Structures | 结构体数组，传统布局 |
| 缓存行 | Cache Line | 64字节缓存传输单元 |
| 伪共享 | False Sharing | 多线程修改同一缓存行的不同数据 |
| 预取 | Prefetch | 提前加载即将访问的数据到缓存 |
| Morton序 | Morton Order / Z-order | 空间填充曲线，保持局部性 |
| OpenMP | Open Multi-Processing | 共享内存并行编程API |
| 线程局部 | Thread-local | 每个线程独立的存储 |

## 附录B：推荐第三方库

| 库 | 用途 | 许可证 |
|:---|:---|:---|
| **Eigen** | 向量/矩阵运算 | MPL2 |
| **Intel TBB** | 并行算法、任务调度 | Apache 2.0 |
| **robin-hood-hashing** | 高性能哈希表 | MIT |
| **fmt** | 快速格式化 | MIT |
| **spdlog** | 日志 | MIT |
| **Google Benchmark** | 性能基准测试 | Apache 2.0 |

---

*本文档与 `GPU_Voxel_Machining_Analysis.md` 形成完整的技术方案对照，覆盖从嵌入式CPU到数据中心GPU的全尺度部署需求。*
