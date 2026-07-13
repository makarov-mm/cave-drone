#include "PointCloudRenderer.h"
#include "Config.h"

namespace
{
    const char* kVertexSrc = R"GLSL(
#version 330 core
layout(location = 0) in vec3 aPos;
uniform mat4 uViewProj;
uniform vec3 uCameraPos;
uniform float uWorldHeight;
out vec3 vColor;
out float vFog;
void main()
{
    float t = clamp(aPos.y / uWorldHeight, 0.0, 1.0);
    vec3 low = vec3(0.10, 0.38, 0.10);
    vec3 high = vec3(0.65, 0.95, 0.25);
    vColor = mix(low, high, t);

    float dist = length(aPos - uCameraPos);
    vFog = exp(-dist * 0.020);
    gl_Position = uViewProj * vec4(aPos, 1.0);
    gl_PointSize = clamp(160.0 / max(dist, 0.5), 1.0, 5.0);
}
)GLSL";

    const char* kFragmentSrc = R"GLSL(
#version 330 core
in vec3 vColor;
in float vFog;
out vec4 fragColor;
void main()
{
    fragColor = vec4(vColor * vFog, 1.0);
}
)GLSL";
}

bool PointCloudRenderer::Init()
{
    if (!m_shader.Build(kVertexSrc, kFragmentSrc))
        return false;
    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &m_vbo);
    glBindVertexArray(m_vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(GL_ARRAY_BUFFER, kCapacity * 3 * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), reinterpret_cast<void*>(0));
    glBindVertexArray(0);
    return true;
}

void PointCloudRenderer::Shutdown()
{
    glDeleteBuffers(1, &m_vbo);
    glDeleteVertexArrays(1, &m_vao);
}

void PointCloudRenderer::Clear()
{
    m_head = 0;
    m_count = 0;
}

void PointCloudRenderer::AddPoints(const std::vector<Vec3>& points)
{
    if (points.empty())
        return;

    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    size_t remaining = points.size();
    size_t src = 0;
    while (remaining > 0)
    {
        size_t slots = static_cast<size_t>(kCapacity - m_head);
        size_t batch = remaining < slots ? remaining : slots;
        glBufferSubData(GL_ARRAY_BUFFER,
                        static_cast<GLintptr>(m_head) * 3 * sizeof(float),
                        static_cast<GLsizeiptr>(batch * 3 * sizeof(float)),
                        &points[src].x);
        m_head = (m_head + static_cast<int>(batch)) % kCapacity;
        src += batch;
        remaining -= batch;
    }
    m_count += static_cast<int>(points.size());
    if (m_count > kCapacity)
        m_count = kCapacity;
}

void PointCloudRenderer::Draw(const Mat4& viewProj, const Vec3& cameraPos) const
{
    if (m_count == 0)
        return;
    m_shader.Use();
    m_shader.SetMat4("uViewProj", viewProj);
    m_shader.SetVec3("uCameraPos", cameraPos);
    m_shader.SetFloat("uWorldHeight", WORLD_NY * VOXEL_SIZE);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glBindVertexArray(m_vao);
    glDrawArrays(GL_POINTS, 0, m_count);
    glBindVertexArray(0);
    glDisable(GL_PROGRAM_POINT_SIZE);
}
