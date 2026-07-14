#pragma once
#include "Math.h"
#include "Config.h"
#include <cstdint>
#include <span>
#include <vector>

// Greedy meshing shared by the observed-map renderer and the ground-truth
// renderer. Coplanar visible faces with the same normal are merged into
// maximal rectangles (Lysenko-style sweep), which collapses flat walls and
// floors into a handful of large quads instead of thousands of unit faces.
//
// Input is a padded occupancy buffer: the chunk itself plus a one-cell
// border from neighboring chunks, so faces on chunk boundaries are culled
// correctly. Vertex format stays position(3), normal(3), uv(2); the uv of
// a merged quad spans (0,0)..(w,h) in voxel units, and the fragment shaders
// use fract(uv) to keep drawing the per-voxel grid on merged surfaces.

constexpr int PADDED_SIZE = CHUNK_SIZE + 2;

// x, y, z in [-1, CHUNK_SIZE]
inline int PaddedIndex(int x, int y, int z)
{
    return ((y + 1) * PADDED_SIZE + (z + 1)) * PADDED_SIZE + (x + 1);
}

void GreedyMeshChunk(std::span<const uint8_t> paddedSolid, const Vec3& chunkOrigin,
                     std::vector<float>& outVerts);
