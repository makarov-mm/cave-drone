#pragma once
#include "Math.h"
#include "VoxelMap.h"
#include <concepts>
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

    // Plans a path to a specific point (e.g. home after exploration).
    // First attempts full-clearance space; if that region does not connect
    // to the goal (noise-frozen cells can sever safe connectivity in a
    // corridor no drone will scan again), falls back to a degraded plan
    // through merely-free cells. relaxedUsed reports which tier succeeded;
    // relaxed paths must be validated with the matching relaxed check.
    bool PlanToPoint(const VoxelMap& map, const Vec3& startWorld,
                     const Vec3& goalWorld, std::vector<Vec3>& outPath,
                     bool& relaxedUsed);

    // Checks that the upcoming portion of the path is still traversable
    // given what the map knows now.
    bool PathStillValid(const VoxelMap& map, const std::vector<Vec3>& path,
                        size_t fromIndex, bool relaxed = false) const;

    // Straight segment through known-free cells only. Deliberately weaker
    // than the safety predicate used for planning: the polyline itself
    // guarantees full clearance, and this check merely stops the path
    // follower from dragging the pursuit chord through cells the map
    // already classifies as occupied or unknown. Requiring full clearance
    // here would freeze the follower whenever the drone brushes past a
    // wall, since it samples the drone's own position.
    [[nodiscard]] static bool ChordIsFree(const VoxelMap& map, const Vec3& a, const Vec3& b);

    // Reflex probe: every cell from the start point along dir for dist
    // meters must be known free, including the very next cell. Unlike
    // ChordIsFree this does not skip the first cell, because its job is to
    // veto motion into a wall that is already adjacent.
    [[nodiscard]] static bool ProbeIsFree(const VoxelMap& map, const Vec3& from,
                                          const Vec3& dir, float dist);

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
    template <std::predicate<int, int, int> GoalPredicate>
    bool Bfs(const VoxelMap& map, const Vec3& startWorld, GoalPredicate&& isGoal,
             std::vector<Vec3>& outPath, bool& exhausted, bool relaxed = false);
    bool LineOfSight(const VoxelMap& map, const Vec3& a, const Vec3& b) const;
    void Smooth(const VoxelMap& map, std::vector<Vec3>& path) const;
    // Snaps a cell to the nearest safe one. When chordFrom is given, the
    // chosen cell must additionally be reachable from that point by a
    // straight free chord: picking a safe cell diagonally across a wall
    // corner produces a first path leg the chord gate rightfully refuses,
    // which deadlocks the follower against the wall.
    bool FindNearestSafe(const VoxelMap& map, int& x, int& y, int& z,
                         const Vec3* chordFrom = nullptr) const;
};
