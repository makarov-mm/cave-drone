#include "VoxelMap.h"

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
    const float inv = 1.0f / VOXEL_SIZE;
    int ix = static_cast<int>(std::floor(origin.x * inv));
    int iy = static_cast<int>(std::floor(origin.y * inv));
    int iz = static_cast<int>(std::floor(origin.z * inv));

    int stepX = dir.x > 0.0f ? 1 : -1;
    int stepY = dir.y > 0.0f ? 1 : -1;
    int stepZ = dir.z > 0.0f ? 1 : -1;

    auto boundary = [](float o, float d, int i, int step)
    {
        float edge = (step > 0 ? static_cast<float>(i + 1) : static_cast<float>(i)) * VOXEL_SIZE;
        return (edge - o) / d;
    };

    float tMaxX = std::fabs(dir.x) > 1e-8f ? boundary(origin.x, dir.x, ix, stepX) : 1e30f;
    float tMaxY = std::fabs(dir.y) > 1e-8f ? boundary(origin.y, dir.y, iy, stepY) : 1e30f;
    float tMaxZ = std::fabs(dir.z) > 1e-8f ? boundary(origin.z, dir.z, iz, stepZ) : 1e30f;

    float tDeltaX = std::fabs(dir.x) > 1e-8f ? VOXEL_SIZE / std::fabs(dir.x) : 1e30f;
    float tDeltaY = std::fabs(dir.y) > 1e-8f ? VOXEL_SIZE / std::fabs(dir.y) : 1e30f;
    float tDeltaZ = std::fabs(dir.z) > 1e-8f ? VOXEL_SIZE / std::fabs(dir.z) : 1e30f;

    float t = 0.0f;
    while (t <= maxDist)
    {
        if (Get(ix, iy, iz) == CellState::Occupied)
            return t;

        if (tMaxX < tMaxY && tMaxX < tMaxZ)
        {
            t = tMaxX; tMaxX += tDeltaX; ix += stepX;
        }
        else if (tMaxY < tMaxZ)
        {
            t = tMaxY; tMaxY += tDeltaY; iy += stepY;
        }
        else
        {
            t = tMaxZ; tMaxZ += tDeltaZ; iz += stepZ;
        }
    }
    return maxDist;
}
