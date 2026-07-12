#pragma once
#include "GlLoader.h"
#include "Shader.h"
#include "VoxelMap.h"
#include "Math.h"
#include <unordered_map>
#include <vector>

// Builds one interleaved VBO per chunk (position, normal, face UV) and
// redraws only chunks whose contents changed. Hidden faces between two
// occupied cells are culled during meshing, so vertex counts stay small.
class VoxelRenderer
{
public:
    bool Init();
    void Shutdown();

    // Rebuilds up to `budget` dirty chunk meshes per call
    void UpdateMeshes(VoxelMap& map, int budget);

    void Draw(const Mat4& viewProj, const Vec3& cameraPos) const;

    int MeshedChunkCount() const { return static_cast<int>(m_meshes.size()); }
    int DrawnChunkCount() const { return m_drawnChunks; }

private:
    struct ChunkMesh
    {
        GLuint vao = 0;
        GLuint vbo = 0;
        int vertexCount = 0;
        int cx = 0, cy = 0, cz = 0;
    };

    Shader m_shader;
    std::unordered_map<uint64_t, ChunkMesh> m_meshes;
    mutable int m_drawnChunks = 0;

    void RebuildChunk(const VoxelMap& map, const VoxelMap::Chunk& chunk, uint64_t key);
};
