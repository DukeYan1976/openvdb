#pragma once
#include <glad/glad.h>

namespace midgard {

// 着色器程序管理
class ShaderProgram {
public:
    GLuint id = 0;

    bool load(const char* vertSrc, const char* fragSrc);
    void use() const;
    void setMat4(const char* name, const float* mat) const;
    void setMat3(const char* name, const float* mat) const;
    void setVec3(const char* name, float x, float y, float z) const;
    void setFloat(const char* name, float val) const;
    void cleanup();

private:
    GLuint compileShader(GLenum type, const char* src);
};

// 内置着色器源码
namespace Shaders {
    extern const char* meshVert;
    extern const char* meshFrag;
    extern const char* lineVert;
    extern const char* lineFrag;
    extern const char* pointVert;
    extern const char* pointFrag;
}

} // namespace midgard
