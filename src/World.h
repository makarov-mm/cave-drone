#pragma once
#include "Math.h"
#include "Config.h"
#include "VoxelDda.h"
#include <concepts>
#include <cstdint>
#include <vector>

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

    // Voxel traversal along a unit direction. Calls visitFree for every
    // empty cell along the ray. Returns true on hit and writes the hit
    // cell coordinates.
    template <std::invocable<int, int, int> FreeVisitor>
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

template <std::invocable<int, int, int> FreeVisitor>
bool World::Raycast(const Vec3& origin, const Vec3& dir, float maxDist,
                    FreeVisitor&& visitFree, int& hitX, int& hitY, int& hitZ) const
{
    for (VoxelDda dda(origin, dir); dda.EnterT() <= maxDist; dda.Step())
    {
        if (IsSolid(dda.X(), dda.Y(), dda.Z()))
        {
            hitX = dda.X();
            hitY = dda.Y();
            hitZ = dda.Z();
            return true;
        }
        visitFree(dda.X(), dda.Y(), dda.Z());
    }
    return false;
}
