#include "Shader.h"
#include <format>

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
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

std::expected<void, std::string> Shader::Build(const char* vertexSrc, const char* fragmentSrc)
{
    auto compileChecked = [](GLenum type, const char* src)
        -> std::expected<GLuint, std::string>
    {
        GLuint shader = Compile(type, src);
        if (shader != 0)
            return shader;
        // Re-run to fetch the log for the error message
        shader = glCreateShader(type);
        glShaderSource(shader, 1, &src, nullptr);
        glCompileShader(shader);
        char log[2048] = {};
        glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
        glDeleteShader(shader);
        return std::unexpected(std::format(
            "{} shader compile error:\n{}",
            type == GL_VERTEX_SHADER ? "vertex" : "fragment", log));
    };

    auto vs = compileChecked(GL_VERTEX_SHADER, vertexSrc);
    if (!vs)
        return std::unexpected(vs.error());
    auto fs = compileChecked(GL_FRAGMENT_SHADER, fragmentSrc);
    if (!fs)
    {
        glDeleteShader(*vs);
        return std::unexpected(fs.error());
    }

    m_program = glCreateProgram();
    glAttachShader(m_program, *vs);
    glAttachShader(m_program, *fs);
    glLinkProgram(m_program);
    glDeleteShader(*vs);
    glDeleteShader(*fs);

    GLint ok = 0;
    glGetProgramiv(m_program, GL_LINK_STATUS, &ok);
    if (ok == 0)
    {
        char log[2048] = {};
        glGetProgramInfoLog(m_program, sizeof(log), nullptr, log);
        return std::unexpected(std::format("shader link error:\n{}", log));
    }
    return {};
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
