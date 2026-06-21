#pragma once
#include <cstddef>
#include <cstdint>

namespace midgard {

class IDebugDisplay {
public:
    virtual ~IDebugDisplay() = default;

    // data 为 packed float3: 3 * count 个 float
    virtual void drawLines(const float* data, size_t count, uint32_t color) = 0;
    virtual void drawPoints(const float* data, size_t count, uint32_t color) = 0;
    virtual void drawTriangles(const float* verts, const uint32_t* indices, size_t triCount, uint32_t color) = 0;
    virtual void clear() = 0;
};

extern IDebugDisplay* g_debugDisplay;

} // namespace midgard

// 编译隔离宏
#ifdef MIDGARD_DEV
  #define DEBUG_DRAW(...) do { if (midgard::g_debugDisplay) { __VA_ARGS__; } } while(0)
#else
  #define DEBUG_DRAW(...) ((void)0)
#endif
