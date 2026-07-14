#pragma once
#include "Math.h"
#include "Config.h"
#include <cmath>
#include <limits>

// Amanatides-Woo traversal over the global voxel grid. One implementation
// shared by the ground-truth raycast, the observed-map occlusion query and
// the lidar evidence integration, which previously each carried their own
// copy of this stepping logic.
//
// Usage:
//   for (VoxelDda dda(origin, unitDir); dda.EnterT() <= maxDist; dda.Step())
//       visit(dda.X(), dda.Y(), dda.Z());
class VoxelDda
{
public:
    VoxelDda(const Vec3& origin, const Vec3& dir)
    {
        // Parenthesized to survive TUs where windows.h defined a max macro
        constexpr float kInfinity = (std::numeric_limits<float>::max)();
        const float inv = 1.0f / VOXEL_SIZE;
        m_x = static_cast<int>(std::floor(origin.x * inv));
        m_y = static_cast<int>(std::floor(origin.y * inv));
        m_z = static_cast<int>(std::floor(origin.z * inv));

        m_stepX = dir.x > 0.0f ? 1 : -1;
        m_stepY = dir.y > 0.0f ? 1 : -1;
        m_stepZ = dir.z > 0.0f ? 1 : -1;

        auto boundary = [](float o, float d, int i, int step)
        {
            float edge = (step > 0 ? static_cast<float>(i + 1)
                                   : static_cast<float>(i)) * VOXEL_SIZE;
            return (edge - o) / d;
        };

        m_tMaxX = std::fabs(dir.x) > 1e-8f ? boundary(origin.x, dir.x, m_x, m_stepX) : kInfinity;
        m_tMaxY = std::fabs(dir.y) > 1e-8f ? boundary(origin.y, dir.y, m_y, m_stepY) : kInfinity;
        m_tMaxZ = std::fabs(dir.z) > 1e-8f ? boundary(origin.z, dir.z, m_z, m_stepZ) : kInfinity;

        m_tDeltaX = std::fabs(dir.x) > 1e-8f ? VOXEL_SIZE / std::fabs(dir.x) : kInfinity;
        m_tDeltaY = std::fabs(dir.y) > 1e-8f ? VOXEL_SIZE / std::fabs(dir.y) : kInfinity;
        m_tDeltaZ = std::fabs(dir.z) > 1e-8f ? VOXEL_SIZE / std::fabs(dir.z) : kInfinity;
    }

    [[nodiscard]] int X() const { return m_x; }
    [[nodiscard]] int Y() const { return m_y; }
    [[nodiscard]] int Z() const { return m_z; }

    // Ray parameter where the current cell was entered / will be left
    [[nodiscard]] float EnterT() const { return m_tEnter; }
    [[nodiscard]] float ExitT() const
    {
        return m_tMaxX < m_tMaxY ? (m_tMaxX < m_tMaxZ ? m_tMaxX : m_tMaxZ)
                                 : (m_tMaxY < m_tMaxZ ? m_tMaxY : m_tMaxZ);
    }

    void Step()
    {
        m_tEnter = ExitT();
        if (m_tMaxX < m_tMaxY && m_tMaxX < m_tMaxZ)
        {
            m_tMaxX += m_tDeltaX;
            m_x += m_stepX;
        }
        else if (m_tMaxY < m_tMaxZ)
        {
            m_tMaxY += m_tDeltaY;
            m_y += m_stepY;
        }
        else
        {
            m_tMaxZ += m_tDeltaZ;
            m_z += m_stepZ;
        }
    }

private:
    int m_x, m_y, m_z;
    int m_stepX, m_stepY, m_stepZ;
    float m_tMaxX, m_tMaxY, m_tMaxZ;
    float m_tDeltaX, m_tDeltaY, m_tDeltaZ;
    float m_tEnter = 0.0f;
};
