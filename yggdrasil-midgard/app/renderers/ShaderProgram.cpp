#include "ShaderProgram.h"
#include <cstdio>

namespace midgard {

// 内置着色器源码
namespace Shaders {

const char* meshVert = R"(
#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNorm;
uniform mat4 uMVP;
uniform mat3 uNormalMat;
out vec3 vNorm;
void main(){
    gl_Position = uMVP * vec4(aPos, 1.0);
    vNorm = normalize(uNormalMat * aNorm);
})";

const char* meshFrag = R"(
#version 330 core
in vec3 vNorm;
out vec4 fragColor;
uniform vec3 uLightDir;
uniform vec3 uColor;
uniform float uAlpha;
void main(){
    float diff = abs(dot(vNorm, uLightDir)) * 0.5 + 0.5;
    fragColor = vec4(uColor * diff, uAlpha);
})";

const char* lineVert = R"(
#version 330 core
layout(location=0) in vec3 aPos;
uniform mat4 uMVP;
void main(){ gl_Position = uMVP * vec4(aPos, 1.0); }
)";

const char* lineFrag = R"(
#version 330 core
out vec4 fragColor;
uniform vec3 uColor;
uniform float uAlpha;
void main(){ fragColor = vec4(uColor, uAlpha); }
)";

const char* pointVert = R"(
#version 330 core
layout(location=0) in vec3 aPos;
uniform mat4 uMVP;
uniform float uPointSize;
void main(){
    gl_Position = uMVP * vec4(aPos, 1.0);
    gl_PointSize = uPointSize;
}
)";

const char* pointFrag = R"(
#version 330 core
out vec4 fragColor;
uniform vec3 uColor;
uniform float uAlpha;
void main(){ fragColor = vec4(uColor, uAlpha); }
)";

} // namespace Shaders

bool ShaderProgram::load(const char* vertSrc, const char* fragSrc) {
    GLuint v = compileShader(GL_VERTEX_SHADER, vertSrc);
    GLuint f = compileShader(GL_FRAGMENT_SHADER, fragSrc);
    if (!v || !f) return false;

    id = glCreateProgram();
    glAttachShader(id, v);
    glAttachShader(id, f);
    glLinkProgram(id);

    GLint ok;
    glGetProgramiv(id, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetProgramInfoLog(id, 512, nullptr, log);
        printf("Link: %s\n", log);
        glDeleteProgram(id);
        id = 0;
    }

    glDeleteShader(v);
    glDeleteShader(f);
    return ok;
}

void ShaderProgram::use() const {
    glUseProgram(id);
}

void ShaderProgram::setMat4(const char* name, const float* mat) const {
    glUniformMatrix4fv(glGetUniformLocation(id, name), 1, GL_FALSE, mat);
}

void ShaderProgram::setMat3(const char* name, const float* mat) const {
    glUniformMatrix3fv(glGetUniformLocation(id, name), 1, GL_FALSE, mat);
}

void ShaderProgram::setVec3(const char* name, float x, float y, float z) const {
    glUniform3f(glGetUniformLocation(id, name), x, y, z);
}

void ShaderProgram::setFloat(const char* name, float val) const {
    glUniform1f(glGetUniformLocation(id, name), val);
}

void ShaderProgram::cleanup() {
    if (id) { glDeleteProgram(id); id = 0; }
}

GLuint ShaderProgram::compileShader(GLenum type, const char* src) {
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    GLint ok;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetShaderInfoLog(s, 512, nullptr, log);
        printf("Shader: %s\n", log);
        glDeleteShader(s);
        return 0;
    }
    return s;
}

} // namespace midgard
