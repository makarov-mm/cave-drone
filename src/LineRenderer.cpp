#include "LineRenderer.h"

namespace
{
    const char* kVertexSrc = R"GLSL(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aColor;
uniform mat4 uViewProj;
out vec3 vColor;
void main()
{
    vColor = aColor;
    gl_Position = uViewProj * vec4(aPos, 1.0);
}
)GLSL";

    const char* kFragmentSrc = R"GLSL(
#version 330 core
in vec3 vColor;
out vec4 fragColor;
void main()
{
    fragColor = vec4(vColor, 1.0);
}
)GLSL";
}

bool LineRenderer::Init()
{
    if (!m_shader.Build(kVertexSrc, kFragmentSrc))
        return false;
    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &m_vbo);
    glBindVertexArray(m_vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    const GLsizei stride = 6 * sizeof(float);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(0));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(3 * sizeof(float)));
    glBindVertexArray(0);
    return true;
}

void LineRenderer::Shutdown()
{
    glDeleteBuffers(1, &m_vbo);
    glDeleteVertexArrays(1, &m_vao);
}

void LineRenderer::Begin()
{
    m_verts.clear();
}

void LineRenderer::AddLine(const Vec3& a, const Vec3& b, const Vec3& color)
{
    const float data[] = {
        a.x, a.y, a.z, color.x, color.y, color.z,
        b.x, b.y, b.z, color.x, color.y, color.z};
    m_verts.insert(m_verts.end(), data, data + 12);
}

void LineRenderer::AddPolyline(const std::vector<Vec3>& points, const Vec3& color)
{
    for (size_t i = 0; i + 1 < points.size(); ++i)
        AddLine(points[i], points[i + 1], color);
}

void LineRenderer::Draw(const Mat4& viewProj)
{
    if (m_verts.empty())
        return;
    m_shader.Use();
    m_shader.SetMat4("uViewProj", viewProj);
    glBindVertexArray(m_vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(m_verts.size() * sizeof(float)),
                 m_verts.data(), GL_DYNAMIC_DRAW);
    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(m_verts.size() / 6));
    glBindVertexArray(0);
}
