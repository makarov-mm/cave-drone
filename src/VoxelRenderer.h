#pragma once
#include "GlLoader.h"
#include "Shader.h"
#include <expected>
#include <string>
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
    [[nodiscard]] std::expected<void, std::string> Init();
    void Shutdown();

    // Rebuilds up to `budget` dirty chunk meshes per call. `now` stamps
    // rebuilt chunks so freshly updated geometry glows and cools down.
    void UpdateMeshes(VoxelMap& map, int budget, double now);

    // fogDensity: 0.030 for the in-scene view; the rotating overview uses
    // a much lower value so a distant camera does not fog the map to black.
    // pulseOrigin is the focused drone: a scan wave expands from it. Fresh
    // chunks glow (rebuild recency) and cool down over a second and a half.
    // effectStrength scales rim, pulse and glow (1 main view, subtler inset).
    void Draw(const Mat4& viewProj, const Vec3& cameraPos, float fogDensity,
              double now, const Vec3& pulseOrigin, float effectStrength) const;

    // World-space AABB of everything meshed so far. False while empty.
    bool ComputeBounds(Vec3& outMin, Vec3& outMax) const;

    int MeshedChunkCount() const { return static_cast<int>(m_meshes.size()); }
    int DrawnChunkCount() const { return m_drawnChunks; }

private:
    struct ChunkMesh
    {
        GLuint vao = 0;
        GLuint vbo = 0;
        int vertexCount = 0;
        int cx = 0, cy = 0, cz = 0;
        double rebuiltAt = -100.0;
    };

    Shader m_shader;
    std::unordered_map<uint64_t, ChunkMesh> m_meshes;
    mutable int m_drawnChunks = 0;

    void RebuildChunk(const VoxelMap& map, const VoxelMap::Chunk& chunk, uint64_t key);
};
