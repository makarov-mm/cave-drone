#pragma once
#include "Math.h"
#include "Config.h"
#include <vector>
#include <cstdint>

// Ground-truth environment. The drone never reads it directly:
// it only perceives the world through simulated lidar raycasts.
class World
{
public:
    void Generate(uint32_t seed);

    bool IsSolid(int ix, int iy, int iz) const
    {
        if (ix < 0 || iy < 0 || iz < 0 || ix >= WORLD_NX || iy >= WORLD_NY || iz >= WORLD_NZ)
            return true;
        return m_solid[Index(ix, iy, iz)] != 0;
    }

    // Amanatides-Woo voxel traversal. Calls visitFree for every empty cell
    // along the ray. Returns true on hit and writes the hit cell coordinates.
    template <typename FreeVisitor>
    bool Raycast(const Vec3& origin, const Vec3& dir, float maxDist,
                 FreeVisitor&& visitFree, int& hitX, int& hitY, int& hitZ) const;

    // Distance to the first solid cell along a unit direction, or a value
    // greater than maxDist if nothing is hit. Used by the lidar to obtain
    // the true range before noise is applied.
    float HitDistance(const Vec3& origin, const Vec3& dir, float maxDist) const;

    Vec3 StartPosition() const { return m_startPos; }

private:
    std::vector<uint8_t> m_solid;
    Vec3 m_startPos;

    static size_t Index(int ix, int iy, int iz)
    {
        return (static_cast<size_t>(iy) * WORLD_NZ + iz) * WORLD_NX + ix;
    }

    void CarveSphere(float cx, float cy, float cz, float radius);
    void CarveTunnels(uint32_t seed);
    void KeepMainCavity(int startX, int startY, int startZ);
};

template <typename FreeVisitor>
bool World::Raycast(const Vec3& origin, const Vec3& dir, float maxDist,
                    FreeVisitor&& visitFree, int& hitX, int& hitY, int& hitZ) const
{
    const float inv = 1.0f / VOXEL_SIZE;
    int ix = static_cast<int>(std::floor(origin.x * inv));
    int iy = static_cast<int>(std::floor(origin.y * inv));
    int iz = static_cast<int>(std::floor(origin.z * inv));

    int stepX = dir.x > 0.0f ? 1 : -1;
    int stepY = dir.y > 0.0f ? 1 : -1;
    int stepZ = dir.z > 0.0f ? 1 : -1;

    auto boundary = [inv](float o, float d, int i, int step)
    {
        float edge = (step > 0 ? static_cast<float>(i + 1) : static_cast<float>(i)) * VOXEL_SIZE;
        return d != 0.0f ? (edge - o) / d : 1e30f;
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
        {
            hitX = ix; hitY = iy; hitZ = iz;
            return true;
        }
        visitFree(ix, iy, iz);

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
    return false;
}
