#pragma once
#include "Window.h"
#include "Camera.h"
#include "World.h"
#include "VoxelMap.h"
#include "Lidar.h"
#include "Planner.h"
#include "Drone.h"
#include "VoxelRenderer.h"
#include "TruthRenderer.h"
#include "LineRenderer.h"
#include "DroneRenderer.h"
#include <vector>

// One autonomous agent of the fleet. All agents integrate their lidar
// evidence into the same shared map (an ideal-communication model of map
// sharing) and coordinate through goal claims in the planner.
struct AgentState
{
    Drone drone;
    Lidar lidar{0};
    Planner planner;
    std::vector<Vec3> path;
    std::vector<Vec3> trail;
    double replanTimer = 0.0;
    double stuckTimer = 0.0;
    Vec3 home;
    bool atHome = false;
};

class App
{
public:
    bool Init();
    int Run();

private:
    Window m_window;
    Camera m_camera;
    World m_world;
    VoxelMap m_map;
    std::vector<AgentState> m_agents;
    VoxelRenderer m_voxelRenderer;
    TruthRenderer m_truthRenderer;
    LineRenderer m_lineRenderer;
    DroneRenderer m_droneRenderer;

    double m_trailTimer = 0.0;
    double m_titleTimer = 0.0;
    uint32_t m_seed = 1u;
    int m_focus = 0;
    bool m_paused = false;
    bool m_splitView = true;

    enum class Phase { Exploring, Returning, Home };
    Phase m_phase = Phase::Exploring;

    void ResetSimulation(uint32_t seed);
    void UpdateSimulation(double dt);
    void UpdateAgentExploring(AgentState& agent, double dt);
    void UpdateAgentReturning(AgentState& agent, double dt);
    Vec3 SeparationFor(const AgentState& agent) const;
    std::vector<Vec3> ClaimsAgainst(const AgentState& agent) const;
    void HandleInput(double dt);
    void Render();
    void RenderMapPane(int x, int y, int w, int h);
    void RenderFpvPane(int x, int y, int w, int h);
    void UpdateTitle(double fps);
};
