#pragma once
#include "GlLoader.h"
#include "Shader.h"
#include <expected>
#include <string>
#include "Drone.h"
#include "Math.h"
#include <vector>

// Procedural drone mesh: a central body, four arms and four two-blade
// props that spin according to the physics thrust demand.
class DroneRenderer
{
public:
    [[nodiscard]] std::expected<void, std::string> Init();
    void Shutdown();
    void Draw(const Mat4& viewProj, const Drone& drone) const;

private:
    Shader m_shader;
    GLuint m_bodyVao = 0, m_bodyVbo = 0;
    int m_bodyVertexCount = 0;
    GLuint m_propVao = 0, m_propVbo = 0;
    int m_propVertexCount = 0;

    static void AddBox(std::vector<float>& verts, const Vec3& center,
                       const Vec3& halfExtents, const Vec3& color);
};
