#include "DroneRenderer.h"

namespace
{
    const char* kVertexSrc = R"GLSL(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aColor;
uniform mat4 uMvp;
out vec3 vColor;
void main()
{
    vColor = aColor;
    gl_Position = uMvp * vec4(aPos, 1.0);
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

    Mat4 RotationY(float angle)
    {
        Mat4 r = Mat4::Identity();
        float c = std::cos(angle), s = std::sin(angle);
        r.m[0] = c;  r.m[8] = s;
        r.m[2] = -s; r.m[10] = c;
        return r;
    }

    void UploadMesh(GLuint& vao, GLuint& vbo, const std::vector<float>& verts)
    {
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(verts.size() * sizeof(float)),
                     verts.data(), GL_STATIC_DRAW);
        const GLsizei stride = 6 * sizeof(float);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(0));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(3 * sizeof(float)));
        glBindVertexArray(0);
    }
}

void DroneRenderer::AddBox(std::vector<float>& verts, const Vec3& center,
                           const Vec3& halfExtents, const Vec3& color)
{
    const Vec3 c = center;
    const Vec3 h = halfExtents;
    const float shades[6] = {0.80f, 0.80f, 1.00f, 0.55f, 0.70f, 0.70f};
    static const float corners[6][4][3] = {
        {{1,-1,-1},{1,1,-1},{1,1,1},{1,-1,1}},
        {{-1,-1,-1},{-1,-1,1},{-1,1,1},{-1,1,-1}},
        {{-1,1,-1},{1,1,-1},{1,1,1},{-1,1,1}},
        {{-1,-1,-1},{-1,-1,1},{1,-1,1},{1,-1,-1}},
        {{-1,-1,1},{1,-1,1},{1,1,1},{-1,1,1}},
        {{-1,-1,-1},{-1,1,-1},{1,1,-1},{1,-1,-1}}};
    static const int tris[6] = {0, 1, 2, 0, 2, 3};

    for (int face = 0; face < 6; ++face)
    {
        Vec3 shaded = color * shades[face];
        for (int i = 0; i < 6; ++i)
        {
            const float* p = corners[face][tris[i]];
            verts.push_back(c.x + p[0] * h.x);
            verts.push_back(c.y + p[1] * h.y);
            verts.push_back(c.z + p[2] * h.z);
            verts.push_back(shaded.x);
            verts.push_back(shaded.y);
            verts.push_back(shaded.z);
        }
    }
}

bool DroneRenderer::Init()
{
    if (!m_shader.Build(kVertexSrc, kFragmentSrc))
        return false;

    // Body: central block, canopy, four diagonal arms, motor pods
    std::vector<float> body;
    const Vec3 dark{0.16f, 0.17f, 0.19f};
    const Vec3 accent{0.85f, 0.45f, 0.10f};
    AddBox(body, {0.0f, 0.0f, 0.0f}, {0.14f, 0.05f, 0.20f}, dark);
    AddBox(body, {0.0f, 0.06f, 0.05f}, {0.08f, 0.035f, 0.10f}, accent);
    const float armX[4] = {0.30f, -0.30f, 0.30f, -0.30f};
    const float armZ[4] = {0.30f, 0.30f, -0.30f, -0.30f};
    for (int i = 0; i < 4; ++i)
    {
        Vec3 tip{armX[i], 0.0f, armZ[i]};
        AddBox(body, tip * 0.5f, {0.19f, 0.018f, 0.05f}, dark); // rough arm
        AddBox(body, {tip.x, 0.03f, tip.z}, {0.045f, 0.045f, 0.045f}, dark); // motor pod
    }
    m_bodyVertexCount = static_cast<int>(body.size() / 6);
    UploadMesh(m_bodyVao, m_bodyVbo, body);

    // Two-blade prop, drawn once per motor with its own spin
    std::vector<float> prop;
    const Vec3 blade{0.75f, 0.75f, 0.78f};
    AddBox(prop, {0.10f, 0.0f, 0.0f}, {0.10f, 0.004f, 0.022f}, blade);
    AddBox(prop, {-0.10f, 0.0f, 0.0f}, {0.10f, 0.004f, 0.022f}, blade);
    m_propVertexCount = static_cast<int>(prop.size() / 6);
    UploadMesh(m_propVao, m_propVbo, prop);

    return true;
}

void DroneRenderer::Shutdown()
{
    glDeleteBuffers(1, &m_bodyVbo);
    glDeleteVertexArrays(1, &m_bodyVao);
    glDeleteBuffers(1, &m_propVbo);
    glDeleteVertexArrays(1, &m_propVao);
}

void DroneRenderer::Draw(const Mat4& viewProj, const Drone& drone) const
{
    Mat4 model = Mat4::Translation(drone.Position()) * drone.Orientation().ToMat4();

    m_shader.Use();
    m_shader.SetMat4("uMvp", viewProj * model);
    glBindVertexArray(m_bodyVao);
    glDrawArrays(GL_TRIANGLES, 0, m_bodyVertexCount);

    const float armX[4] = {0.30f, -0.30f, 0.30f, -0.30f};
    const float armZ[4] = {0.30f, 0.30f, -0.30f, -0.30f};
    for (int i = 0; i < 4; ++i)
    {
        float dir = (i == 0 || i == 3) ? 1.0f : -1.0f;
        Mat4 local = Mat4::Translation({armX[i], 0.085f, armZ[i]}) *
                     RotationY(drone.RotorAngle() * 9.0f * dir + static_cast<float>(i) * 1.3f);
        m_shader.SetMat4("uMvp", viewProj * model * local);
        glBindVertexArray(m_propVao);
        glDrawArrays(GL_TRIANGLES, 0, m_propVertexCount);
    }
    glBindVertexArray(0);
}
