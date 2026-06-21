#include "GPURenderers.h"
#include <glad/glad.h>

namespace midgard {

// GPUMesh
void GPUMesh::init() {
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glGenBuffers(1, &ebo);
}

void GPUMesh::upload(const float* verts, size_t vBytes, const uint32_t* idx, size_t iBytes, int n) {
    if (!vao) init();
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, vBytes, verts, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, iBytes, idx, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    count = n;
}

void GPUMesh::draw() {
    if (count > 0) {
        glBindVertexArray(vao);
        glDrawElements(GL_TRIANGLES, count, GL_UNSIGNED_INT, nullptr);
    }
}

void GPUMesh::cleanup() {
    if (ebo) { glDeleteBuffers(1, &ebo); ebo = 0; }
    if (vbo) { glDeleteBuffers(1, &vbo); vbo = 0; }
    if (vao) { glDeleteVertexArrays(1, &vao); vao = 0; }
    count = 0;
}

// GPULines
void GPULines::init() {
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
}

void GPULines::upload(const float* data, size_t bytes, int vertCount) {
    if (!vao) init();
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, bytes, data, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    count = vertCount;
}

void GPULines::draw() {
    if (count > 0) {
        glBindVertexArray(vao);
        glDrawArrays(GL_LINES, 0, count);
    }
}

void GPULines::cleanup() {
    if (vbo) { glDeleteBuffers(1, &vbo); vbo = 0; }
    if (vao) { glDeleteVertexArrays(1, &vao); vao = 0; }
    count = 0;
}

// GPUPoints
void GPUPoints::init() {
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
}

void GPUPoints::upload(const float* data, size_t bytes, int vertCount) {
    if (!vao) init();
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, bytes, data, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    count = vertCount;
}

void GPUPoints::draw(float pointSize) {
    if (count > 0) {
        glPointSize(pointSize);
        glBindVertexArray(vao);
        glDrawArrays(GL_POINTS, 0, count);
    }
}

void GPUPoints::cleanup() {
    if (vbo) { glDeleteBuffers(1, &vbo); vbo = 0; }
    if (vao) { glDeleteVertexArrays(1, &vao); vao = 0; }
    count = 0;
}

} // namespace midgard
