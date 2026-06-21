#pragma once
#include <glad/glad.h>
#include <vector>
#include <cstdint>

namespace midgard {

// GPU 三角网格
struct GPUMesh {
    GLuint vao = 0, vbo = 0, ebo = 0;
    int count = 0;
    
    void init();
    void upload(const float* verts, size_t vBytes, 
                  const uint32_t* idx, size_t iBytes, int n);
    void draw();
    void cleanup();
};

// GPU 线段
struct GPULines {
    GLuint vao = 0, vbo = 0;
    int count = 0;
    
    void init();
    void upload(const float* data, size_t bytes, int vertCount);
    void draw();
    void cleanup();
};

// GPU 点云
struct GPUPoints {
    GLuint vao = 0, vbo = 0;
    int count = 0;
    
    void init();
    void upload(const float* data, size_t bytes, int vertCount);
    void draw(float pointSize);
    void cleanup();
};

} // namespace midgard
