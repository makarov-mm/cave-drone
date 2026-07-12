// Headless smoke test: runs the full multi-agent sense-plan-act loop
// without rendering. Not part of the Windows build. Compile:
//   g++ -O2 -std=c++17 tests/HeadlessTest.cpp src/World.cpp src/VoxelMap.cpp \
//       src/Lidar.cpp src/Planner.cpp src/Drone.cpp src/Meshing.cpp -Isrc -o headless_test
// Optional arguments: [agentCount] [seed] [simSeconds]
#include "../src/World.h"
#include "../src/VoxelMap.h"
#include "../src/Lidar.h"
#include "../src/Planner.h"
#include "../src/Drone.h"
#include "../src/Config.h"
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <vector>

struct TestAgent
{
    Drone drone;
    Lidar lidar{0};
    Planner planner;
    std::vector<Vec3> path;
    double replanTimer = 0.0;
    double stuckTimer = 0.0;
    Vec3 home;
    bool atHome = false;
};

static Vec3 SeparationFor(const std::vector<TestAgent>& agents, size_t self)
{
    Vec3 push;
    for (size_t i = 0; i < agents.size(); ++i)
    {
        if (i == self)
            continue;
        Vec3 delta = agents[self].drone.Position() - agents[i].drone.Position();
        float d = Length(delta);
        if (d < 1e-4f || d > SEPARATION_DIST)
            continue;
        push += delta * (SEPARATION_GAIN * (SEPARATION_DIST - d) / (SEPARATION_DIST * d));
    }
    float len = Length(push);
    if (len > DRONE_CRUISE_SPEED)
        push *= DRONE_CRUISE_SPEED / len;
    return push;
}

static std::vector<Vec3> ClaimsAgainst(const std::vector<TestAgent>& agents, size_t self)
{
    std::vector<Vec3> claims;
    for (size_t i = 0; i < agents.size(); ++i)
        if (i != self && !agents[i].path.empty())
            claims.push_back(agents[i].path.back());
    return claims;
}

int main(int argc, char** argv)
{
    const int agentCount = argc > 1 ? std::atoi(argv[1]) : 3;
    const uint32_t seed = argc > 2 ? static_cast<uint32_t>(std::atoi(argv[2])) : 1u;
    const double simSeconds = argc > 3 ? std::atof(argv[3]) : 240.0;

    World world;
    world.Generate(seed);

    VoxelMap map;
    std::vector<TestAgent> agents(agentCount);
    const Vec3 center = world.StartPosition();
    for (int i = 0; i < agentCount; ++i)
    {
        float angle = 6.2831853f * static_cast<float>(i) / agentCount;
        agents[i].home = center + Vec3{std::cos(angle) * 1.5f, 0.0f, std::sin(angle) * 1.5f};
        agents[i].drone.Reset(agents[i].home);
        agents[i].lidar = Lidar(seed * 31u + static_cast<uint32_t>(i) * 977u + 12345u);
    }

    const double dt = 1.0 / 60.0;
    int collisions = 0;
    int replans = 0;
    bool explorationDone = false;
    double doneAt = 0.0;

    auto physicsStep = [&](TestAgent& agent, size_t idx)
    {
        Vec3 separation = SeparationFor(agents, idx);
        float remaining = static_cast<float>(dt);
        while (remaining > 0.0f)
        {
            float st = remaining > 1.0f / 240.0f ? 1.0f / 240.0f : remaining;
            agent.drone.Update(st, agent.path, separation);
            remaining -= st;
        }
        const Vec3 p = agent.drone.Position();
        if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z))
        {
            std::printf("FAIL: NaN position\n");
            std::exit(1);
        }
        int cx = static_cast<int>(std::floor(p.x / VOXEL_SIZE));
        int cy = static_cast<int>(std::floor(p.y / VOXEL_SIZE));
        int cz = static_cast<int>(std::floor(p.z / VOXEL_SIZE));
        if (world.IsSolid(cx, cy, cz))
            ++collisions;
    };

    // --- Exploration phase
    for (double t = 0.0; t < simSeconds && !explorationDone; t += dt)
    {
        for (TestAgent& agent : agents)
        {
            agent.lidar.Scan(agent.drone.Position(), world, map);
            agent.planner.Advance(dt);
        }

        for (size_t i = 0; i < agents.size(); ++i)
        {
            TestAgent& agent = agents[i];
            agent.replanTimer -= dt;
            bool blocked = !agent.path.empty() &&
                !agent.planner.PathStillValid(map, agent.path, agent.drone.PathProgress());
            bool goalDone = !agent.path.empty() &&
                !agent.planner.GoalStillWorthwhile(map, agent.path.back());
            bool wantReplan = agent.path.empty() || blocked ||
                              (agent.replanTimer <= 0.0 && goalDone);
            if (wantReplan && !agent.planner.ExplorationComplete())
            {
                if (agent.planner.Replan(map, agent.drone.Position(), agent.path,
                                         ClaimsAgainst(agents, i)))
                {
                    agent.drone.ResetPathProgress();
                    ++replans;
                }
                else
                {
                    agent.path.clear();
                }
                agent.replanTimer = REPLAN_INTERVAL;
            }

            bool atGoal = !agent.path.empty() &&
                Length(agent.path.back() - agent.drone.Position()) < 1.0f &&
                agent.drone.Speed() < 0.4f;
            agent.stuckTimer = atGoal ? agent.stuckTimer + dt : 0.0;
            if (agent.stuckTimer > 1.5)
            {
                agent.planner.BlacklistGoal(agent.path.empty() ? agent.drone.Position()
                                                               : agent.path.back());
                agent.path.clear();
                agent.replanTimer = 0.0;
                agent.stuckTimer = 0.0;
            }
        }

        bool allComplete = true;
        for (const TestAgent& agent : agents)
            allComplete = allComplete && agent.planner.ExplorationComplete();
        if (allComplete)
        {
            explorationDone = true;
            doneAt = t;
        }

        for (size_t i = 0; i < agents.size(); ++i)
            physicsStep(agents[i], i);

        if (static_cast<int>(t * 60.0) % 1800 == 0)
            std::printf("t=%5.0fs  free=%7d occ=%7d\n", t, map.FreeCount(), map.OccupiedCount());
    }

    std::printf("exploration: agents=%d free=%d occ=%d replans=%d collisionFrames=%d %s\n",
                agentCount, map.FreeCount(), map.OccupiedCount(), replans, collisions,
                explorationDone ? "COMPLETE" : "(time limit)");

    if (collisions > 0)
    {
        std::printf("FAIL: drone entered solid voxels\n");
        return 1;
    }
    if (map.FreeCount() < 20000 * agentCount)
    {
        std::printf("FAIL: exploration made too little progress\n");
        return 1;
    }

    // --- Return-to-home through the shared map
    bool allHome = false;
    for (double t = 0.0; t < 180.0 && !allHome; t += dt)
    {
        for (TestAgent& agent : agents)
            agent.lidar.Scan(agent.drone.Position(), world, map);

        allHome = true;
        for (TestAgent& agent : agents)
        {
            if (agent.atHome)
                continue;
            if (Length(agent.home - agent.drone.Position()) < 1.2f)
            {
                agent.atHome = true;
                agent.path.clear();
                continue;
            }
            allHome = false;
            agent.replanTimer -= dt;
            bool blocked = !agent.path.empty() &&
                !agent.planner.PathStillValid(map, agent.path, agent.drone.PathProgress());
            if (agent.replanTimer <= 0.0 || blocked || agent.path.empty())
            {
                if (agent.planner.PlanToPoint(map, agent.drone.Position(), agent.home, agent.path))
                    agent.drone.ResetPathProgress();
                else
                    agent.path.clear();
                agent.replanTimer = REPLAN_INTERVAL * 2.0;
            }
        }

        for (size_t i = 0; i < agents.size(); ++i)
            physicsStep(agents[i], i);
    }

    if (!allHome)
    {
        std::printf("FAIL: not all drones returned home within 180 s\n");
        return 1;
    }
    if (collisions > 0)
    {
        std::printf("FAIL: collision during return flight\n");
        return 1;
    }
    std::printf("return to home: all %d drones reached their pads\n", agentCount);
    if (explorationDone)
        std::printf("full coverage in %.0f s\n", doneAt);
    std::printf("PASS\n");
    return 0;
}
