#pragma once
#include "GlLoader.h"
#include "Shader.h"
#include "Math.h"
#include <vector>

// Immediate-style colored line renderer (dynamic VBO, rebuilt per frame).
// Used for the planned path and the traveled trail.
class LineRenderer
{
public:
    bool Init();
    void Shutdown();

    void Begin();
    void AddLine(const Vec3& a, const Vec3& b, const Vec3& color);
    void AddPolyline(const std::vector<Vec3>& points, const Vec3& color);
    void Draw(const Mat4& viewProj);

private:
    Shader m_shader;
    GLuint m_vao = 0;
    GLuint m_vbo = 0;
    std::vector<float> m_verts;
};
