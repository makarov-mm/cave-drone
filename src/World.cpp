#include "World.h"
#include "Noise.h"
#include <random>
#include <queue>
#include <array>

void World::Generate(uint32_t seed)
{
    m_solid.assign(static_cast<size_t>(WORLD_NX) * WORLD_NY * WORLD_NZ, 1);

    // Carve organic chambers with fBm noise
    Noise noise(seed);
    const float freq = 0.055f;
    for (int y = 3; y < WORLD_NY - 3; ++y)
        for (int z = 2; z < WORLD_NZ - 2; ++z)
            for (int x = 2; x < WORLD_NX - 2; ++x)
            {
                float n = noise.Fbm(x * freq, y * freq * 1.8f, z * freq, 4);
                // Bias: fewer voids near floor and ceiling
                float edge = 1.0f - std::fabs(2.0f * y / WORLD_NY - 1.0f);
                if (n + edge * 0.12f > 0.60f)
                    m_solid[Index(x, y, z)] = 0;
            }

    // Guarantee a start chamber in the middle
    const float cx = WORLD_NX * 0.5f;
    const float cy = WORLD_NY * 0.5f;
    const float cz = WORLD_NZ * 0.5f;
    CarveSphere(cx, cy, cz, 6.0f);

    // Carve a tunnel network so the noise chambers connect
    CarveTunnels(seed);

    // Remove voids unreachable from the start chamber, so every
    // frontier the drone discovers is actually attainable.
    KeepMainCavity(static_cast<int>(cx), static_cast<int>(cy), static_cast<int>(cz));

    m_startPos = Vec3(cx * VOXEL_SIZE, cy * VOXEL_SIZE, cz * VOXEL_SIZE);
}

void World::CarveSphere(float cx, float cy, float cz, float radius)
{
    int r = static_cast<int>(std::ceil(radius));
    for (int dy = -r; dy <= r; ++dy)
        for (int dz = -r; dz <= r; ++dz)
            for (int dx = -r; dx <= r; ++dx)
            {
                if (static_cast<float>(dx * dx + dy * dy + dz * dz) > radius * radius)
                    continue;
                int x = static_cast<int>(cx) + dx;
                int y = static_cast<int>(cy) + dy;
                int z = static_cast<int>(cz) + dz;
                if (x < 2 || y < 3 || z < 2 || x >= WORLD_NX - 2 || y >= WORLD_NY - 3 || z >= WORLD_NZ - 2)
                    continue;
                m_solid[Index(x, y, z)] = 0;
            }
}

void World::CarveTunnels(uint32_t seed)
{
    std::mt19937 rng(seed * 7919u + 17u);
    std::uniform_real_distribution<float> uni(0.0f, 1.0f);

    const int walkerCount = 14;
    for (int w = 0; w < walkerCount; ++w)
    {
        float px = WORLD_NX * 0.5f;
        float py = WORLD_NY * 0.5f;
        float pz = WORLD_NZ * 0.5f;

        float yaw = uni(rng) * 6.2831853f;
        float pitch = (uni(rng) - 0.5f) * 0.5f;
        float radius = 1.8f + uni(rng) * 1.2f;

        const int steps = 550;
        for (int s = 0; s < steps; ++s)
        {
            yaw += (uni(rng) - 0.5f) * 0.45f;
            pitch += (uni(rng) - 0.5f) * 0.25f;
            pitch = Clamp(pitch, -0.5f, 0.5f);

            px += std::cos(yaw) * std::cos(pitch);
            py += std::sin(pitch) * 0.6f;
            pz += std::sin(yaw) * std::cos(pitch);

            px = Clamp(px, 6.0f, WORLD_NX - 7.0f);
            py = Clamp(py, 6.0f, WORLD_NY - 7.0f);
            pz = Clamp(pz, 6.0f, WORLD_NZ - 7.0f);

            CarveSphere(px, py, pz, radius);
            if (uni(rng) < 0.01f)
                radius = 1.8f + uni(rng) * 1.4f;
        }
    }
}

void World::KeepMainCavity(int startX, int startY, int startZ)
{
    std::vector<uint8_t> reachable(m_solid.size(), 0);
    std::queue<std::array<int, 3>> queue;
    queue.push({startX, startY, startZ});
    reachable[Index(startX, startY, startZ)] = 1;

    const int dirs[6][3] = {{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
    while (!queue.empty())
    {
        auto [x, y, z] = queue.front();
        queue.pop();
        for (const int* d : dirs)
        {
            int nx = x + d[0], ny = y + d[1], nz = z + d[2];
            if (IsSolid(nx, ny, nz))
                continue;
            size_t idx = Index(nx, ny, nz);
            if (reachable[idx] != 0)
                continue;
            reachable[idx] = 1;
            queue.push({nx, ny, nz});
        }
    }

    for (size_t i = 0; i < m_solid.size(); ++i)
        if (m_solid[i] == 0 && reachable[i] == 0)
            m_solid[i] = 1;
}

float World::HitDistance(const Vec3& origin, const Vec3& dir, float maxDist) const
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
        if (IsSolid(ix, iy, iz))
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
    return maxDist + 1.0f;
}
