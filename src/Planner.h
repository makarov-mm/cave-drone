#pragma once
#include "Math.h"
#include "VoxelMap.h"
#include <vector>

// Frontier-based exploration. A breadth-first search over known-safe cells
// finds the nearest cell adjacent to a frontier (free space bordering the
// unknown). The raw grid path is then shortened with line-of-sight
// shortcutting so the drone flies smooth diagonals instead of stair-steps.
class Planner
{
public:
    // Returns true if a path was found. Path is in world coordinates.
    // claimedGoals are other agents' current targets: frontiers within
    // CLAIM_RADIUS of a claim are skipped so the fleet spreads out. If
    // every remaining frontier is claimed, the search retries unclaimed,
    // which keeps completion detection correct and lets idle drones assist.
    bool Replan(const VoxelMap& map, const Vec3& startWorld, std::vector<Vec3>& outPath,
                const std::vector<Vec3>& claimedGoals = {});

    // Plans a path through known-safe space to a specific point (e.g. home
    // after exploration completes). Returns false if unreachable.
    bool PlanToPoint(const VoxelMap& map, const Vec3& startWorld,
                     const Vec3& goalWorld, std::vector<Vec3>& outPath);

    // Checks that the upcoming portion of the path is still traversable
    // given what the map knows now.
    bool PathStillValid(const VoxelMap& map, const std::vector<Vec3>& path, size_t fromIndex) const;

    // Goal hysteresis: true while the current goal still borders an
    // exploration frontier. Agents keep their goal instead of re-picking
    // the nearest one every replan cycle; without this, symmetric claims
    // make fleet members swap goals back and forth and oscillate.
    bool GoalStillWorthwhile(const VoxelMap& map, const Vec3& goalWorld) const;

    // Advances the planner clock and prunes expired blacklist entries
    void Advance(double dt);

    // Called when the drone reached a goal but the frontier behind it never
    // resolves (grazing angles, occlusion). The position is excluded from
    // goal selection for BLACKLIST_TTL seconds. The shared map itself stays
    // truthful: another drone approaching from a different angle can still
    // resolve the same frontier, and no corridor can be sealed off by a
    // poisoned map.
    void BlacklistGoal(const Vec3& goalWorld);

    bool ExplorationComplete() const { return m_complete; }
    void Reset()
    {
        m_complete = false;
        m_exhaustStreak = 0;
        m_time = 0.0;
        m_blacklist.clear();
    }

private:
    struct BlacklistEntry
    {
        Vec3 position;
        double expiresAt;
    };

    bool m_complete = false;
    int m_exhaustStreak = 0;
    double m_time = 0.0;
    std::vector<BlacklistEntry> m_blacklist;

    bool IsBlacklisted(const Vec3& p) const;

    static void WorldToCell(const Vec3& p, int& x, int& y, int& z);
    static Vec3 CellCenter(int x, int y, int z);
    static bool IsGoal(const VoxelMap& map, int x, int y, int z);
    // Shared BFS over known-safe cells. Returns true if a cell satisfying
    // the goal predicate was found; sets exhausted when the whole safe
    // region was expanded without finding one.
    template <typename GoalPredicate>
    bool Bfs(const VoxelMap& map, const Vec3& startWorld, GoalPredicate&& isGoal,
             std::vector<Vec3>& outPath, bool& exhausted);
    bool LineOfSight(const VoxelMap& map, const Vec3& a, const Vec3& b) const;
    void Smooth(const VoxelMap& map, std::vector<Vec3>& path) const;
    bool FindNearestSafe(const VoxelMap& map, int& x, int& y, int& z) const;
};
