#include "VoxelRenderer.h"
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
uniform vec3 uCameraPos;
uniform float uWorldMinY;
uniform float uWorldHeight;
out vec4 fragColor;
void main()
{
    // Height gradient between two greens, like a lidar point-cloud palette
    float t = clamp((vWorldPos.y - uWorldMinY) / uWorldHeight, 0.0, 1.0);
    vec3 low = vec3(0.10, 0.34, 0.08);
    vec3 high = vec3(0.55, 0.85, 0.20);
    vec3 base = mix(low, high, t);

    // Face shading: ceilings dark, floors bright
    float shade = 0.62 + 0.38 * clamp(vNormal.y * 0.5 + 0.5, 0.0, 1.0);
    shade *= 0.85 + 0.15 * abs(vNormal.x);

    // Per-voxel grid edges; uv is in voxel units so fract restores the
    // grid on greedy-merged quads
    float b = 0.055;
    vec2 f = fract(vUv);
    float edge = (f.x < b || f.x > 1.0 - b || f.y < b || f.y > 1.0 - b) ? 0.45 : 1.0;

    // Exponential distance fog into cave darkness
    float dist = length(vWorldPos - uCameraPos);
    float fog = exp(-dist * 0.030);

    vec3 color = base * shade * edge;
    vec3 fogColor = vec3(0.01, 0.02, 0.01);
    fragColor = vec4(mix(fogColor, color, fog), 1.0);
}
)GLSL";
}

bool VoxelRenderer::Init()
{
    return m_shader.Build(kVertexSrc, kFragmentSrc);
}

void VoxelRenderer::Shutdown()
{
    for (auto& [key, mesh] : m_meshes)
    {
        glDeleteBuffers(1, &mesh.vbo);
        glDeleteVertexArrays(1, &mesh.vao);
    }
    m_meshes.clear();
}

void VoxelRenderer::RebuildChunk(const VoxelMap& map, const VoxelMap::Chunk& chunk, uint64_t key)
{
    std::vector<float> verts;
    verts.reserve(4096);

    const int ox = chunk.cx * CHUNK_SIZE;
    const int oy = chunk.cy * CHUNK_SIZE;
    const int oz = chunk.cz * CHUNK_SIZE;

    // Padded occupancy: chunk interior read directly, the one-cell border
    // ring resolved through the map so faces on chunk boundaries cull right
    uint8_t padded[PADDED_SIZE * PADDED_SIZE * PADDED_SIZE];
    for (int y = -1; y <= CHUNK_SIZE; ++y)
        for (int z = -1; z <= CHUNK_SIZE; ++z)
            for (int x = -1; x <= CHUNK_SIZE; ++x)
            {
                bool interior = x >= 0 && x < CHUNK_SIZE &&
                                y >= 0 && y < CHUNK_SIZE &&
                                z >= 0 && z < CHUNK_SIZE;
                CellState state = interior
                    ? VoxelMap::Classify(chunk.cells[(y * CHUNK_SIZE + z) * CHUNK_SIZE + x])
                    : map.Get(ox + x, oy + y, oz + z);
                padded[PaddedIndex(x, y, z)] = state == CellState::Occupied ? 1 : 0;
            }

    Vec3 origin{ox * VOXEL_SIZE, oy * VOXEL_SIZE, oz * VOXEL_SIZE};
    GreedyMeshChunk(padded, origin, verts);

    auto it = m_meshes.find(key);
    if (verts.empty())
    {
        if (it != m_meshes.end())
        {
            glDeleteBuffers(1, &it->second.vbo);
            glDeleteVertexArrays(1, &it->second.vao);
            m_meshes.erase(it);
        }
        return;
    }

    if (it == m_meshes.end())
    {
        ChunkMesh mesh;
        glGenVertexArrays(1, &mesh.vao);
        glGenBuffers(1, &mesh.vbo);
        mesh.cx = chunk.cx;
        mesh.cy = chunk.cy;
        mesh.cz = chunk.cz;
        it = m_meshes.emplace(key, mesh).first;
    }

    ChunkMesh& mesh = it->second;
    glBindVertexArray(mesh.vao);
    glBindBuffer(GL_ARRAY_BUFFER, mesh.vbo);
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

    mesh.vertexCount = static_cast<int>(verts.size() / 8);
}

void VoxelRenderer::UpdateMeshes(VoxelMap& map, int budget)
{
    auto& dirty = map.DirtyChunks();
    auto it = dirty.begin();
    while (it != dirty.end() && budget > 0)
    {
        uint64_t key = *it;
        auto chunkIt = map.Chunks().find(key);
        if (chunkIt != map.Chunks().end())
        {
            RebuildChunk(map, *chunkIt->second, key);
            --budget;
        }
        it = dirty.erase(it);
    }
}

void VoxelRenderer::Draw(const Mat4& viewProj, const Vec3& cameraPos) const
{
    m_shader.Use();
    m_shader.SetMat4("uViewProj", viewProj);
    m_shader.SetVec3("uCameraPos", cameraPos);
    m_shader.SetFloat("uWorldMinY", 0.0f);
    m_shader.SetFloat("uWorldHeight", WORLD_NY * VOXEL_SIZE);

    // Frustum culling: skip chunks entirely outside the view volume
    const Frustum frustum = Frustum::FromViewProj(viewProj);
    const float chunkWorldSize = CHUNK_SIZE * VOXEL_SIZE;

    m_drawnChunks = 0;
    for (const auto& [key, mesh] : m_meshes)
    {
        Vec3 mn{mesh.cx * chunkWorldSize, mesh.cy * chunkWorldSize, mesh.cz * chunkWorldSize};
        Vec3 mx = mn + Vec3{chunkWorldSize, chunkWorldSize, chunkWorldSize};
        if (!frustum.IntersectsAabb(mn, mx))
            continue;
        glBindVertexArray(mesh.vao);
        glDrawArrays(GL_TRIANGLES, 0, mesh.vertexCount);
        ++m_drawnChunks;
    }
    glBindVertexArray(0);
}
