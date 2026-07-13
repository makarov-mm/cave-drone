#pragma once
#include "GlLoader.h"
#include "Math.h"

class Shader
{
public:
    Shader() = default;
    ~Shader();
    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;

    bool Build(const char* vertexSrc, const char* fragmentSrc);
    void Use() const;

    GLint UniformLocation(const char* name) const;
    void SetMat4(const char* name, const Mat4& value) const;
    void SetVec3(const char* name, const Vec3& value) const;
    void SetFloat(const char* name, float value) const;

private:
    GLuint m_program = 0;

    static GLuint Compile(GLenum type, const char* src);
};
