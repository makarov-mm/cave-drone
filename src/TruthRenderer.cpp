#include "TruthRenderer.h"
#include "Meshing.h"
#include "Config.h"

namespace
{
    const char* kVertexSrc = R"GLSL(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aUv;
uniform mat4 uViewProj;
out vec3 vNormal;
out vec2 vUv;
out vec3 vWorldPos;
void main()
{
    vNormal = aNormal;
    vUv = aUv;
    vWorldPos = aPos;
    gl_Position = uViewProj * vec4(aPos, 1.0);
}
)GLSL";

    const char* kFragmentSrc = R"GLSL(
#version 330 core
in vec3 vNormal;
in vec2 vUv;
in vec3 vWorldPos;
uniform vec3 uLightPos;
out vec4 fragColor;

// Cheap value noise for rock albedo variation
float hash(vec3 p)
{
    return fract(sin(dot(p, vec3(127.1, 311.7, 74.7))) * 43758.5453);
}

void main()
{
    // Rock albedo with per-voxel brightness variation
    float tint = 0.75 + 0.5 * hash(floor(vWorldPos * 2.01));
    vec3 albedo = vec3(0.42, 0.38, 0.33) * tint;

    // Headlight: point light at the drone with quadratic falloff
    vec3 toLight = uLightPos - vWorldPos;
    float dist = length(toLight);
    vec3 lightDir = toLight / max(dist, 1e-4);
    float diffuse = max(dot(vNormal, lightDir), 0.0);
    float attenuation = 14.0 / (1.0 + 0.6 * dist * dist);
    float lighting = diffuse * attenuation + 0.006;

    // Subtle voxel seams so scale reads on flat rock; fract restores the
    // per-voxel pattern on greedy-merged quads
    float b = 0.03;
    vec2 f = fract(vUv);
    float edge = (f.x < b || f.x > 1.0 - b || f.y < b || f.y > 1.0 - b) ? 0.85 : 1.0;

    vec3 color = albedo * lighting * edge;
    // Filmic-ish rolloff keeps the near rock from blowing out
    color = color / (color + vec3(1.0));
    fragColor = vec4(color, 1.0);
}
)GLSL";

    void UploadMesh(GLuint& vao, GLuint& vbo, const std::vector<float>& verts)
    {
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(verts.size() * sizeof(float)),
                     verts.data(), GL_STATIC_DRAW);
        const GLsizei stride = 8 * sizeof(float);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(0));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(3 * sizeof(float)));
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(6 * sizeof(float)));
        glBindVertexArray(0);
    }
}

std::expected<void, std::string> TruthRenderer::Init()
{
    return m_shader.Build(kVertexSrc, kFragmentSrc);
}

void TruthRenderer::FreeMeshes()
{
    for (ChunkMesh& mesh : m_meshes)
    {
        glDeleteBuffers(1, &mesh.vbo);
        glDeleteVertexArrays(1, &mesh.vao);
    }
    m_meshes.clear();
}

void TruthRenderer::Shutdown()
{
    FreeMeshes();
}

void TruthRenderer::Build(const World& world)
{
    FreeMeshes();

    const int cxCount = (WORLD_NX + CHUNK_SIZE - 1) / CHUNK_SIZE;
    const int cyCount = (WORLD_NY + CHUNK_SIZE - 1) / CHUNK_SIZE;
    const int czCount = (WORLD_NZ + CHUNK_SIZE - 1) / CHUNK_SIZE;

    std::vector<float> verts;
    std::vector<uint8_t> padded(PADDED_SIZE * PADDED_SIZE * PADDED_SIZE);
    for (int cy = 0; cy < cyCount; ++cy)
        for (int cz = 0; cz < czCount; ++cz)
            for (int cx = 0; cx < cxCount; ++cx)
            {
                verts.clear();
                for (int y = -1; y <= CHUNK_SIZE; ++y)
                    for (int z = -1; z <= CHUNK_SIZE; ++z)
                        for (int x = -1; x <= CHUNK_SIZE; ++x)
                            padded[PaddedIndex(x, y, z)] = world.IsSolid(
                                cx * CHUNK_SIZE + x,
                                cy * CHUNK_SIZE + y,
                                cz * CHUNK_SIZE + z) ? 1 : 0;

                Vec3 origin{cx * CHUNK_SIZE * VOXEL_SIZE,
                            cy * CHUNK_SIZE * VOXEL_SIZE,
                            cz * CHUNK_SIZE * VOXEL_SIZE};
                GreedyMeshChunk(padded, origin, verts);
                if (verts.empty())
                    continue;

                ChunkMesh mesh;
                UploadMesh(mesh.vao, mesh.vbo, verts);
                mesh.vertexCount = static_cast<int>(verts.size() / 8);
                const float s = CHUNK_SIZE * VOXEL_SIZE;
                mesh.aabbMin = Vec3{cx * s, cy * s, cz * s};
                mesh.aabbMax = mesh.aabbMin + Vec3{s, s, s};
                m_meshes.push_back(mesh);
            }
}

void TruthRenderer::Draw(const Mat4& viewProj, const Vec3& lightPos) const
{
    m_shader.Use();
    m_shader.SetMat4("uViewProj", viewProj);
    m_shader.SetVec3("uLightPos", lightPos);

    const Frustum frustum = Frustum::FromViewProj(viewProj);
    for (const ChunkMesh& mesh : m_meshes)
    {
        if (!frustum.IntersectsAabb(mesh.aabbMin, mesh.aabbMax))
            continue;
        glBindVertexArray(mesh.vao);
        glDrawArrays(GL_TRIANGLES, 0, mesh.vertexCount);
    }
    glBindVertexArray(0);
}
