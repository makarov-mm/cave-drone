#include "DustRenderer.h"

namespace
{
    const char* kVertexSrc = R"GLSL(
#version 330 core
layout(location = 0) in vec3 aPos;
uniform mat4 uViewProj;
uniform vec3 uLightPos;
out float vBrightness;
void main()
{
    vec3 toLight = uLightPos - aPos;
    float dist = length(toLight);
    // Same falloff family as the rock headlight shading
    vBrightness = 1.2 / (1.0 + 0.6 * dist * dist);

    gl_Position = uViewProj * vec4(aPos, 1.0);
    float viewDist = gl_Position.w;
    gl_PointSize = clamp(7.0 / max(viewDist, 0.2), 1.0, 6.0);
}
)GLSL";

    const char* kFragmentSrc = R"GLSL(
#version 330 core
in float vBrightness;
out vec4 fragColor;
void main()
{
    // Soft round sprite
    vec2 d = gl_PointCoord - vec2(0.5);
    float falloff = clamp(1.0 - dot(d, d) * 4.0, 0.0, 1.0);
    vec3 warm = vec3(1.0, 0.95, 0.85);
    fragColor = vec4(warm * vBrightness * falloff * 0.55, 1.0);
}
)GLSL";
}

bool DustRenderer::Init()
{
    if (!m_shader.Build(kVertexSrc, kFragmentSrc))
        return false;
    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &m_vbo);
    glBindVertexArray(m_vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(GL_ARRAY_BUFFER, kParticleCount * 3 * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), reinterpret_cast<void*>(0));
    glBindVertexArray(0);

    m_particles.resize(kParticleCount);
    m_upload.resize(kParticleCount * 3);
    return true;
}

void DustRenderer::Shutdown()
{
    glDeleteBuffers(1, &m_vbo);
    glDeleteVertexArrays(1, &m_vao);
}

void DustRenderer::Respawn(Particle& particle, const Vec3& center)
{
    std::uniform_real_distribution<float> uni(-1.0f, 1.0f);
    Vec3 offset;
    do
    {
        offset = Vec3{uni(m_rng), uni(m_rng), uni(m_rng)};
    } while (Dot(offset, offset) > 1.0f);
    particle.position = center + offset * (kBubbleRadius * 0.95f);
    particle.velocity = Vec3{uni(m_rng), uni(m_rng) * 0.4f - 0.05f, uni(m_rng)} * 0.12f;
    particle.phase = (uni(m_rng) + 1.0f) * 3.1416f;
}

void DustRenderer::Update(float dt, const Vec3& droneCenter)
{
    m_time += dt;
    bool first = m_particles[0].phase == 0.0f && m_particles[0].velocity.x == 0.0f;
    for (Particle& particle : m_particles)
    {
        if (first)
            Respawn(particle, droneCenter);

        // Slow drift plus a gentle sinusoidal wobble
        Vec3 wobble{
            std::sin(m_time * 0.7f + particle.phase) * 0.03f,
            std::sin(m_time * 0.5f + particle.phase * 1.7f) * 0.02f,
            std::cos(m_time * 0.6f + particle.phase) * 0.03f};
        particle.position += (particle.velocity + wobble) * dt;

        Vec3 fromDrone = particle.position - droneCenter;
        if (Dot(fromDrone, fromDrone) > kBubbleRadius * kBubbleRadius)
            Respawn(particle, droneCenter);
    }

    for (int i = 0; i < kParticleCount; ++i)
    {
        m_upload[i * 3 + 0] = m_particles[i].position.x;
        m_upload[i * 3 + 1] = m_particles[i].position.y;
        m_upload[i * 3 + 2] = m_particles[i].position.z;
    }
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferSubData(GL_ARRAY_BUFFER, 0,
                    static_cast<GLsizeiptr>(m_upload.size() * sizeof(float)),
                    m_upload.data());
}

void DustRenderer::Draw(const Mat4& viewProj, const Vec3& lightPos) const
{
    m_shader.Use();
    m_shader.SetMat4("uViewProj", viewProj);
    m_shader.SetVec3("uLightPos", lightPos);

    glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_ONE, GL_ONE);   // additive: dust only brightens
    glDepthMask(GL_FALSE);         // rocks occlude dust, dust never occludes

    glBindVertexArray(m_vao);
    glDrawArrays(GL_POINTS, 0, kParticleCount);
    glBindVertexArray(0);

    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glDisable(GL_PROGRAM_POINT_SIZE);
}
