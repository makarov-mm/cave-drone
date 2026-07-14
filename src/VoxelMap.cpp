#include "VoxelMap.h"
#include "VoxelDda.h"

VoxelMap::Chunk& VoxelMap::GetOrCreateChunk(int cx, int cy, int cz)
{
    uint64_t key = PackKey(cx, cy, cz);
    auto it = m_chunks.find(key);
    if (it == m_chunks.end())
    {
        auto chunk = std::make_unique<Chunk>();
        chunk->cx = cx;
        chunk->cy = cy;
        chunk->cz = cz;
        it = m_chunks.emplace(key, std::move(chunk)).first;
    }
    return *it->second;
}

void VoxelMap::Integrate(int x, int y, int z, int delta)
{
    int cx = FloorDiv(x), cy = FloorDiv(y), cz = FloorDiv(z);
    Chunk& chunk = GetOrCreateChunk(cx, cy, cz);
    int8_t& cell = chunk.cells[LocalIndex(x, y, z)];

    CellState oldState = Classify(cell);
    int value = static_cast<int>(cell) + delta;
    value = value < LOGODDS_MIN ? LOGODDS_MIN : (value > LOGODDS_MAX ? LOGODDS_MAX : value);
    cell = static_cast<int8_t>(value);
    CellState newState = Classify(cell);

    if (oldState == newState)
        return;

    if (oldState == CellState::Free) --m_freeCount;
    if (oldState == CellState::Occupied) --m_occupiedCount;
    if (newState == CellState::Free) ++m_freeCount;
    if (newState == CellState::Occupied) ++m_occupiedCount;

    // Meshes depend on classification only, so mark dirty on transitions
    MarkDirtyAround(x, y, z);
}

void VoxelMap::MarkDirtyAround(int x, int y, int z)
{
    // The cell's own chunk, plus neighbor chunks if the cell sits on a border
    // (their meshes depend on this cell's state for face culling).
    int cx = FloorDiv(x), cy = FloorDiv(y), cz = FloorDiv(z);
    m_dirty.insert(PackKey(cx, cy, cz));

    int lx = x - cx * CHUNK_SIZE;
    int ly = y - cy * CHUNK_SIZE;
    int lz = z - cz * CHUNK_SIZE;
    if (lx == 0) m_dirty.insert(PackKey(cx - 1, cy, cz));
    if (lx == CHUNK_SIZE - 1) m_dirty.insert(PackKey(cx + 1, cy, cz));
    if (ly == 0) m_dirty.insert(PackKey(cx, cy - 1, cz));
    if (ly == CHUNK_SIZE - 1) m_dirty.insert(PackKey(cx, cy + 1, cz));
    if (lz == 0) m_dirty.insert(PackKey(cx, cy, cz - 1));
    if (lz == CHUNK_SIZE - 1) m_dirty.insert(PackKey(cx, cy, cz + 1));
}

void VoxelMap::Clear()
{
    m_chunks.clear();
    m_dirty.clear();
    m_freeCount = 0;
    m_occupiedCount = 0;
}

float VoxelMap::OccludedDistance(const Vec3& origin, const Vec3& dir, float maxDist) const
{
    for (VoxelDda dda(origin, dir); dda.EnterT() <= maxDist; dda.Step())
        if (Get(dda.X(), dda.Y(), dda.Z()) == CellState::Occupied)
            return dda.EnterT();
    return maxDist;
}
