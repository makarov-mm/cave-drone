#include "Planner.h"
#include "Config.h"
#include <queue>
#include <unordered_map>
#include <algorithm>

namespace
{
    const int kDirs[6][3] = {{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};

    void UnpackKey(uint64_t key, int& x, int& y, int& z)
    {
        const int bias = 1 << 20;
        x = static_cast<int>((key >> 42) & 0x1FFFFF) - bias;
        y = static_cast<int>((key >> 21) & 0x1FFFFF) - bias;
        z = static_cast<int>(key & 0x1FFFFF) - bias;
    }
}

void Planner::WorldToCell(const Vec3& p, int& x, int& y, int& z)
{
    x = static_cast<int>(std::floor(p.x / VOXEL_SIZE));
    y = static_cast<int>(std::floor(p.y / VOXEL_SIZE));
    z = static_cast<int>(std::floor(p.z / VOXEL_SIZE));
}

Vec3 Planner::CellCenter(int x, int y, int z)
{
    return Vec3(
        (static_cast<float>(x) + 0.5f) * VOXEL_SIZE,
        (static_cast<float>(y) + 0.5f) * VOXEL_SIZE,
        (static_cast<float>(z) + 0.5f) * VOXEL_SIZE);
}

bool Planner::IsGoal(const VoxelMap& map, int x, int y, int z)
{
    // A safe cell adjacent to an exploration frontier: close enough that
    // the lidar will reveal the unknown space behind it. The thick-unknown
    // requirement inside IsExplorationFrontier filters out phantom
    // frontiers created by sensor outliers.
    for (const int* d : kDirs)
        if (map.IsExplorationFrontier(x + d[0], y + d[1], z + d[2]))
            return true;
    return map.IsExplorationFrontier(x, y, z);
}

bool Planner::FindNearestSafe(const VoxelMap& map, int& x, int& y, int& z) const
{
    if (map.IsSafe(x, y, z))
        return true;
    for (int r = 1; r <= 5; ++r)
        for (int dy = -r; dy <= r; ++dy)
            for (int dz = -r; dz <= r; ++dz)
                for (int dx = -r; dx <= r; ++dx)
                    if (map.IsSafe(x + dx, y + dy, z + dz))
                    {
                        x += dx; y += dy; z += dz;
                        return true;
                    }
    return false;
}

template <typename GoalPredicate>
bool Planner::Bfs(const VoxelMap& map, const Vec3& startWorld, GoalPredicate&& isGoal,
                  std::vector<Vec3>& outPath, bool& exhausted)
{
    outPath.clear();
    exhausted = false;

    int sx, sy, sz;
    WorldToCell(startWorld, sx, sy, sz);
    if (!FindNearestSafe(map, sx, sy, sz))
        return false;

    std::queue<uint64_t> frontier;
    std::unordered_map<uint64_t, uint64_t> parent;
    parent.reserve(65536);

    uint64_t startKey = VoxelMap::PackKey(sx, sy, sz);
    frontier.push(startKey);
    parent[startKey] = startKey;

    uint64_t goalKey = 0;
    bool found = false;
    int expansions = 0;

    while (!frontier.empty() && expansions < PLANNER_MAX_EXPANSIONS)
    {
        uint64_t key = frontier.front();
        frontier.pop();
        ++expansions;

        int x, y, z;
        UnpackKey(key, x, y, z);

        if (isGoal(x, y, z))
        {
            goalKey = key;
            found = true;
            break;
        }

        for (const int* d : kDirs)
        {
            int nx = x + d[0], ny = y + d[1], nz = z + d[2];
            if (!map.IsSafe(nx, ny, nz))
                continue;
            uint64_t nkey = VoxelMap::PackKey(nx, ny, nz);
            if (parent.find(nkey) != parent.end())
                continue;
            parent[nkey] = key;
            frontier.push(nkey);
        }
    }

    if (!found)
    {
        exhausted = frontier.empty() && expansions < PLANNER_MAX_EXPANSIONS;
        return false;
    }

    // Reconstruct grid path
    std::vector<Vec3> path;
    uint64_t key = goalKey;
    while (true)
    {
        int x, y, z;
        UnpackKey(key, x, y, z);
        path.push_back(CellCenter(x, y, z));
        uint64_t p = parent[key];
        if (p == key)
            break;
        key = p;
    }
    std::reverse(path.begin(), path.end());
    path.front() = startWorld;

    Smooth(map, path);
    outPath = std::move(path);
    return true;
}

bool Planner::Replan(const VoxelMap& map, const Vec3& startWorld, std::vector<Vec3>& outPath,
                     const std::vector<Vec3>& claimedGoals)
{
    const float claimSq = CLAIM_RADIUS * CLAIM_RADIUS;
    auto unclaimedGoal = [this, &map, &claimedGoals, claimSq](int x, int y, int z)
    {
        if (!IsGoal(map, x, y, z))
            return false;
        Vec3 c = CellCenter(x, y, z);
        if (IsBlacklisted(c))
            return false;
        for (const Vec3& g : claimedGoals)
        {
            Vec3 d = c - g;
            if (Dot(d, d) < claimSq)
                return false;
        }
        return true;
    };

    bool exhausted = false;
    bool found = Bfs(map, startWorld, unclaimedGoal, outPath, exhausted);

    if (!found && !claimedGoals.empty())
    {
        // Every remaining frontier may simply be claimed by other agents;
        // retry with claims lifted (blacklist stays) so completion detection
        // stays correct and an otherwise idle drone still contributes.
        found = Bfs(map, startWorld,
                    [this, &map](int x, int y, int z)
                    {
                        return IsGoal(map, x, y, z) && !IsBlacklisted(CellCenter(x, y, z));
                    },
                    outPath, exhausted);
    }

    // Sensor noise makes cell classification flicker, which can briefly
    // pinch off a narrow passage and exhaust the search prematurely.
    // Declare completion only after several consecutive exhausted replans.
    if (!found && exhausted)
    {
        if (++m_exhaustStreak >= 12)
            m_complete = true;
    }
    else
    {
        m_exhaustStreak = 0;
    }
    return found;
}

bool Planner::PlanToPoint(const VoxelMap& map, const Vec3& startWorld,
                          const Vec3& goalWorld, std::vector<Vec3>& outPath)
{
    int gx, gy, gz;
    WorldToCell(goalWorld, gx, gy, gz);
    if (!FindNearestSafe(map, gx, gy, gz))
        return false;

    bool exhausted = false;
    bool found = Bfs(map, startWorld,
                     [gx, gy, gz](int x, int y, int z)
                     { return x == gx && y == gy && z == gz; },
                     outPath, exhausted);
    if (found)
        outPath.back() = goalWorld;
    return found;
}

void Planner::Advance(double dt)
{
    m_time += dt;
    for (size_t i = 0; i < m_blacklist.size();)
    {
        if (m_blacklist[i].expiresAt <= m_time)
        {
            m_blacklist[i] = m_blacklist.back();
            m_blacklist.pop_back();
        }
        else
        {
            ++i;
        }
    }
}

void Planner::BlacklistGoal(const Vec3& goalWorld)
{
    m_blacklist.push_back({goalWorld, m_time + BLACKLIST_TTL});
}

bool Planner::IsBlacklisted(const Vec3& p) const
{
    const float rSq = BLACKLIST_RADIUS * BLACKLIST_RADIUS;
    for (const BlacklistEntry& entry : m_blacklist)
    {
        Vec3 d = p - entry.position;
        if (Dot(d, d) < rSq)
            return true;
    }
    return false;
}

bool Planner::LineOfSight(const VoxelMap& map, const Vec3& a, const Vec3& b) const
{
    Vec3 delta = b - a;
    float dist = Length(delta);
    if (dist < 1e-4f)
        return true;
    Vec3 dir = delta * (1.0f / dist);
    const float step = VOXEL_SIZE * 0.25f;
    for (float t = 0.0f; t <= dist; t += step)
    {
        Vec3 p = a + dir * t;
        int x, y, z;
        WorldToCell(p, x, y, z);
        if (!map.IsSafe(x, y, z))
            return false;
    }
    return true;
}

void Planner::Smooth(const VoxelMap& map, std::vector<Vec3>& path) const
{
    if (path.size() < 3)
        return;
    std::vector<Vec3> result;
    result.push_back(path.front());
    size_t i = 0;
    while (i + 1 < path.size())
    {
        size_t best = i + 1;
        for (size_t j = path.size() - 1; j > i + 1; --j)
        {
            if (LineOfSight(map, path[i], path[j]))
            {
                best = j;
                break;
            }
        }
        result.push_back(path[best]);
        i = best;
    }
    path = std::move(result);
}

bool Planner::GoalStillWorthwhile(const VoxelMap& map, const Vec3& goalWorld) const
{
    int x, y, z;
    WorldToCell(goalWorld, x, y, z);
    return IsGoal(map, x, y, z) && !IsBlacklisted(goalWorld);
}

bool Planner::PathStillValid(const VoxelMap& map, const std::vector<Vec3>& path, size_t fromIndex) const
{
    if (path.empty())
        return false;
    size_t end = std::min(path.size() - 1, fromIndex + 4);
    for (size_t i = fromIndex; i < end; ++i)
        if (!LineOfSight(map, path[i], path[i + 1]))
            return false;
    return true;
}
