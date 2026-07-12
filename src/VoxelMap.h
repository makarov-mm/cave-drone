#pragma once
#include "Config.h"
#include "Math.h"
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <unordered_set>

enum class CellState : uint8_t
{
    Unknown = 0,
    Free = 1,
    Occupied = 2
};

// Sparse chunked probabilistic occupancy grid. Each cell stores an integer
// log-odds value (OctoMap style): lidar returns add LOGODDS_HIT, rays
// passing through add LOGODDS_MISS. Classification into Unknown / Free /
// Occupied is derived from thresholds, so spurious returns from a noisy
// sensor appear briefly and then dissolve as contradicting evidence
// accumulates. Chunks are allocated lazily, so memory grows only with what
// the drone has actually observed; the scheme scales to unbounded worlds.
class VoxelMap
{
public:
    struct Chunk
    {
        int cx, cy, cz;
        int8_t cells[CHUNK_SIZE * CHUNK_SIZE * CHUNK_SIZE] = {};
    };

    static uint64_t PackKey(int x, int y, int z)
    {
        const uint64_t bias = 1u << 20;
        return ((static_cast<uint64_t>(x) + bias) << 42) |
               ((static_cast<uint64_t>(y) + bias) << 21) |
                (static_cast<uint64_t>(z) + bias);
    }

    static CellState Classify(int8_t logOdds)
    {
        if (logOdds >= LOGODDS_OCC_THRESHOLD) return CellState::Occupied;
        if (logOdds <= LOGODDS_FREE_THRESHOLD) return CellState::Free;
        return CellState::Unknown;
    }

    CellState Get(int x, int y, int z) const
    {
        int cx = FloorDiv(x), cy = FloorDiv(y), cz = FloorDiv(z);
        auto it = m_chunks.find(PackKey(cx, cy, cz));
        if (it == m_chunks.end())
            return CellState::Unknown;
        return Classify(it->second->cells[LocalIndex(x, y, z)]);
    }

    // Bayesian-style evidence updates from the sensor
    void IntegrateHit(int x, int y, int z)  { Integrate(x, y, z, LOGODDS_HIT); }
    void IntegrateMiss(int x, int y, int z) { Integrate(x, y, z, LOGODDS_MISS); }

    // Non-sensor override: the planner gives up on a frontier the lidar
    // cannot resolve by saturating the cell to occupied.
    void ForceOccupied(int x, int y, int z) { Integrate(x, y, z, 127); }

    bool IsFree(int x, int y, int z) const { return Get(x, y, z) == CellState::Free; }

    // Free cell whose entire 26-neighborhood is known free: safe to fly
    // through. This keeps the planned path centerline at least one full
    // voxel away from any wall in every direction, which absorbs the
    // controller's overshoot in turns.
    bool IsSafe(int x, int y, int z) const
    {
        if (!IsFree(x, y, z)) return false;
        for (int dy = -1; dy <= 1; ++dy)
            for (int dz = -1; dz <= 1; ++dz)
                for (int dx = -1; dx <= 1; ++dx)
                {
                    if (dx == 0 && dy == 0 && dz == 0)
                        continue;
                    if (!IsFree(x + dx, y + dy, z + dz))
                        return false;
                }
        return true;
    }

    // Free cell bordering unexplored space
    bool IsFrontier(int x, int y, int z) const
    {
        if (!IsFree(x, y, z)) return false;
        return Get(x + 1, y, z) == CellState::Unknown || Get(x - 1, y, z) == CellState::Unknown ||
               Get(x, y + 1, z) == CellState::Unknown || Get(x, y - 1, z) == CellState::Unknown ||
               Get(x, y, z + 1) == CellState::Unknown || Get(x, y, z - 1) == CellState::Unknown;
    }

    // Unknown cell that is part of a locally thick unknown region (at least
    // 3 unknown face neighbors). Sensor outliers create isolated unknown
    // cells inside well-observed space; those produce phantom frontiers that
    // would make the drone chase noise. Genuinely unexplored space is thick.
    bool IsThickUnknown(int x, int y, int z) const
    {
        if (Get(x, y, z) != CellState::Unknown) return false;
        int count = 0;
        count += Get(x + 1, y, z) == CellState::Unknown;
        count += Get(x - 1, y, z) == CellState::Unknown;
        count += Get(x, y + 1, z) == CellState::Unknown;
        count += Get(x, y - 1, z) == CellState::Unknown;
        count += Get(x, y, z + 1) == CellState::Unknown;
        count += Get(x, y, z - 1) == CellState::Unknown;
        return count >= 3;
    }

    // Frontier worth flying to: free cell bordering a thick unknown region
    bool IsExplorationFrontier(int x, int y, int z) const
    {
        if (!IsFree(x, y, z)) return false;
        return IsThickUnknown(x + 1, y, z) || IsThickUnknown(x - 1, y, z) ||
               IsThickUnknown(x, y + 1, z) || IsThickUnknown(x, y - 1, z) ||
               IsThickUnknown(x, y, z + 1) || IsThickUnknown(x, y, z - 1);
    }

    const std::unordered_map<uint64_t, std::unique_ptr<Chunk>>& Chunks() const { return m_chunks; }
    std::unordered_set<uint64_t>& DirtyChunks() { return m_dirty; }

    // Distance along a unit direction to the first observed-occupied cell,
    // capped at maxDist. Used by the chase camera so discovered walls do not
    // block the view of the drone. Undiscovered geometry is not rendered,
    // so it intentionally does not occlude.
    float OccludedDistance(const Vec3& origin, const Vec3& dir, float maxDist) const;

    int FreeCount() const { return m_freeCount; }
    int OccupiedCount() const { return m_occupiedCount; }

    void Clear();

private:
    std::unordered_map<uint64_t, std::unique_ptr<Chunk>> m_chunks;
    std::unordered_set<uint64_t> m_dirty;
    int m_freeCount = 0;
    int m_occupiedCount = 0;

    static int FloorDiv(int v)
    {
        return v >= 0 ? v / CHUNK_SIZE : (v - CHUNK_SIZE + 1) / CHUNK_SIZE;
    }

    static int LocalIndex(int x, int y, int z)
    {
        int lx = x - FloorDiv(x) * CHUNK_SIZE;
        int ly = y - FloorDiv(y) * CHUNK_SIZE;
        int lz = z - FloorDiv(z) * CHUNK_SIZE;
        return (ly * CHUNK_SIZE + lz) * CHUNK_SIZE + lx;
    }

    Chunk& GetOrCreateChunk(int cx, int cy, int cz);
    void Integrate(int x, int y, int z, int delta);
    void MarkDirtyAround(int x, int y, int z);
};
