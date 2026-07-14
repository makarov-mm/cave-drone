#pragma once
#include "GlLoader.h"
#include "Shader.h"
#include <expected>
#include <string>
#include "Math.h"
#include <random>
#include <vector>

// Dust motes drifting through the FPV headlight beam. Particles live in a
// bubble around the focused drone and recycle when they drift out of it.
// Rendered as additive points whose brightness follows the same headlight
// falloff as the rock, so dust only shows where the light actually is,
// exactly like cave footage.
class DustRenderer
{
public:
    [[nodiscard]] std::expected<void, std::string> Init();
    void Shutdown();

    void Update(float dt, const Vec3& droneCenter);
    void Draw(const Mat4& viewProj, const Vec3& lightPos) const;

private:
    static constexpr int kParticleCount = 600;
    static constexpr float kBubbleRadius = 6.0f;

    struct Particle
    {
        Vec3 position;
        Vec3 velocity;
        float phase;
    };

    Shader m_shader;
    GLuint m_vao = 0;
    GLuint m_vbo = 0;
    std::vector<Particle> m_particles;
    std::vector<float> m_upload;
    std::mt19937 m_rng{20260713u};
    float m_time = 0.0f;
    bool m_seeded = false;

    void Respawn(Particle& particle, const Vec3& center);
};
