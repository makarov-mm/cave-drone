#include "Shader.h"
#include <cstdio>

Shader::~Shader()
{
    if (m_program != 0 && glDeleteProgram != nullptr)
        glDeleteProgram(m_program);
}

GLuint Shader::Compile(GLenum type, const char* src)
{
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &src, nullptr);
    glCompileShader(shader);
    GLint ok = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (ok == 0)
    {
        char log[2048];
        glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
        char msg[2304];
        _snprintf_s(msg, _TRUNCATE, "Shader compile error:\n%s", log);
        MessageBoxA(nullptr, msg, "CaveDroneSim", MB_ICONERROR);
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

bool Shader::Build(const char* vertexSrc, const char* fragmentSrc)
{
    GLuint vs = Compile(GL_VERTEX_SHADER, vertexSrc);
    GLuint fs = Compile(GL_FRAGMENT_SHADER, fragmentSrc);
    if (vs == 0 || fs == 0)
        return false;

    m_program = glCreateProgram();
    glAttachShader(m_program, vs);
    glAttachShader(m_program, fs);
    glLinkProgram(m_program);
    glDeleteShader(vs);
    glDeleteShader(fs);

    GLint ok = 0;
    glGetProgramiv(m_program, GL_LINK_STATUS, &ok);
    if (ok == 0)
    {
        char log[2048];
        glGetProgramInfoLog(m_program, sizeof(log), nullptr, log);
        MessageBoxA(nullptr, log, "Shader link error", MB_ICONERROR);
        return false;
    }
    return true;
}

void Shader::Use() const
{
    glUseProgram(m_program);
}

GLint Shader::UniformLocation(const char* name) const
{
    return glGetUniformLocation(m_program, name);
}

void Shader::SetMat4(const char* name, const Mat4& value) const
{
    glUniformMatrix4fv(glGetUniformLocation(m_program, name), 1, GL_FALSE, value.m);
}

void Shader::SetVec3(const char* name, const Vec3& value) const
{
    glUniform3f(glGetUniformLocation(m_program, name), value.x, value.y, value.z);
}

void Shader::SetFloat(const char* name, float value) const
{
    glUniform1f(glGetUniformLocation(m_program, name), value);
}
