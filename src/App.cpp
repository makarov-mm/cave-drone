#include "App.h"
#include "Config.h"
#include <cstdio>
#include <cmath>

namespace
{
    // Per-agent colors: flight trail and planned path
    const Vec3 kTrailColors[] = {
        {0.85f, 0.45f, 0.10f}, {0.85f, 0.20f, 0.55f}, {0.90f, 0.85f, 0.20f},
        {0.30f, 0.60f, 0.95f}, {0.60f, 0.90f, 0.60f}};
    const Vec3 kPathColors[] = {
        {0.15f, 0.85f, 0.95f}, {0.95f, 0.55f, 0.95f}, {0.95f, 0.95f, 0.60f},
        {0.55f, 0.75f, 1.00f}, {0.70f, 1.00f, 0.70f}};
    constexpr int kColorCount = 5;
}

bool App::Init()
{
    if (!m_window.Create("CaveDroneSim", 1440, 900))
        return false;

    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glClearColor(0.01f, 0.02f, 0.01f, 1.0f);

    if (!m_voxelRenderer.Init() || !m_truthRenderer.Init() ||
        !m_lineRenderer.Init() || !m_droneRenderer.Init())
        return false;

    ResetSimulation(m_seed);
    return true;
}

void App::ResetSimulation(uint32_t seed)
{
    m_seed = seed;
    m_world.Generate(seed);
    m_truthRenderer.Build(m_world);
    m_map.Clear();
    m_voxelRenderer.Shutdown();

    // Spawn the fleet in a ring inside the start chamber
    m_agents.clear();
    m_agents.resize(DRONE_COUNT);
    const Vec3 center = m_world.StartPosition();
    for (int i = 0; i < DRONE_COUNT; ++i)
    {
        AgentState& agent = m_agents[i];
        float angle = 6.2831853f * static_cast<float>(i) / DRONE_COUNT;
        agent.home = center + Vec3{std::cos(angle) * 1.5f, 0.0f, std::sin(angle) * 1.5f};
        agent.drone.Reset(agent.home);
        agent.lidar = Lidar(seed * 31u + static_cast<uint32_t>(i) * 977u + 12345u);
        agent.planner.Reset();
    }

    m_camera.Snap(center);
    m_focus = 0;
    m_phase = Phase::Exploring;
    m_trailTimer = 0.0;
}

std::vector<Vec3> App::ClaimsAgainst(const AgentState& agent) const
{
    std::vector<Vec3> claims;
    for (const AgentState& other : m_agents)
    {
        if (&other == &agent || other.path.empty())
            continue;
        claims.push_back(other.path.back());
    }
    return claims;
}

Vec3 App::SeparationFor(const AgentState& agent) const
{
    // Velocity-space repulsion: agents inside SEPARATION_DIST push each
    // other apart, so crossing paths do not lead to visual mid-air merges.
    Vec3 push;
    for (const AgentState& other : m_agents)
    {
        if (&other == &agent)
            continue;
        Vec3 delta = agent.drone.Position() - other.drone.Position();
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

void App::UpdateAgentExploring(AgentState& agent, double dt)
{
    agent.replanTimer -= dt;
    bool pathBlocked = !agent.path.empty() &&
        !agent.planner.PathStillValid(m_map, agent.path, agent.drone.PathProgress());
    bool goalDone = !agent.path.empty() &&
        !agent.planner.GoalStillWorthwhile(m_map, agent.path.back());
    bool wantReplan = agent.path.empty() || pathBlocked ||
                      (agent.replanTimer <= 0.0 && goalDone);
    if (wantReplan && !agent.planner.ExplorationComplete())
    {
        if (agent.planner.Replan(m_map, agent.drone.Position(), agent.path, ClaimsAgainst(agent)))
            agent.drone.ResetPathProgress();
        else
            agent.path.clear();
        agent.replanTimer = REPLAN_INTERVAL;
    }

    // Livelock guard: the drone sits at a goal but the frontier behind it
    // never resolves (lidar can only see it at grazing angles). Give up on
    // those cells so exploration moves elsewhere.
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

void App::UpdateAgentReturning(AgentState& agent, double dt)
{
    if (agent.atHome)
        return;

    if (Length(agent.home - agent.drone.Position()) < 1.0f && agent.drone.Speed() < 0.5f)
    {
        agent.atHome = true;
        agent.path.clear();
        return;
    }

    agent.replanTimer -= dt;
    bool pathBlocked = !agent.path.empty() &&
        !agent.planner.PathStillValid(m_map, agent.path, agent.drone.PathProgress());
    if (agent.replanTimer <= 0.0 || pathBlocked || agent.path.empty())
    {
        if (agent.planner.PlanToPoint(m_map, agent.drone.Position(), agent.home, agent.path))
            agent.drone.ResetPathProgress();
        else
            agent.path.clear(); // hold position, retry next interval
        agent.replanTimer = REPLAN_INTERVAL * 2.0;
    }
}

void App::UpdateSimulation(double dt)
{
    // Sense: every agent integrates noisy evidence into the shared map
    for (AgentState& agent : m_agents)
    {
        agent.lidar.Scan(agent.drone.Position(), m_world, m_map);
        agent.planner.Advance(dt);
    }

    // Plan
    if (m_phase == Phase::Exploring)
    {
        for (AgentState& agent : m_agents)
            UpdateAgentExploring(agent, dt);

        bool allComplete = true;
        for (const AgentState& agent : m_agents)
            allComplete = allComplete && agent.planner.ExplorationComplete();
        if (allComplete)
        {
            m_phase = Phase::Returning;
            for (AgentState& agent : m_agents)
            {
                agent.path.clear();
                agent.replanTimer = 0.0;
            }
        }
    }
    else if (m_phase == Phase::Returning)
    {
        bool allHome = true;
        for (AgentState& agent : m_agents)
        {
            UpdateAgentReturning(agent, dt);
            allHome = allHome && agent.atHome;
        }
        if (allHome)
            m_phase = Phase::Home;
    }

    // Act: fixed-substep physics per agent, with mutual separation
    for (AgentState& agent : m_agents)
    {
        Vec3 separation = SeparationFor(agent);
        const float subStep = 1.0f / 240.0f;
        float remaining = static_cast<float>(dt);
        while (remaining > 0.0f)
        {
            float step = remaining > subStep ? subStep : remaining;
            agent.drone.Update(step, agent.path, separation);
            remaining -= step;
        }
    }

    // Flight trails
    m_trailTimer -= dt;
    if (m_trailTimer <= 0.0)
    {
        for (AgentState& agent : m_agents)
        {
            agent.trail.push_back(agent.drone.Position());
            if (agent.trail.size() > 3000)
                agent.trail.erase(agent.trail.begin(), agent.trail.begin() + 500);
        }
        m_trailTimer = 0.1;
    }
}

void App::HandleInput(double dt)
{
    InputState& input = m_window.Input();

    if (input.mouseDown)
        m_camera.Rotate(static_cast<float>(input.mouseDeltaX) * -0.008f,
                        static_cast<float>(input.mouseDeltaY) * 0.008f);
    if (input.wheelDelta != 0)
        m_camera.Zoom(static_cast<float>(input.wheelDelta));

    if (input.keysPressed[VK_SPACE])
        m_paused = !m_paused;
    if (input.keysPressed['R'])
        ResetSimulation(m_seed * 1664525u + 1013904223u);
    if (input.keysPressed['C'])
        m_camera.ToggleMode();
    if (input.keysPressed['V'])
        m_splitView = !m_splitView;
    for (int i = 0; i < DRONE_COUNT && i < 9; ++i)
        if (input.keysPressed['1' + i])
            m_focus = i;

    (void)dt;
}

void App::Render()
{
    int w = m_window.Width() > 0 ? m_window.Width() : 1;
    int h = m_window.Height() > 0 ? m_window.Height() : 1;
    glViewport(0, 0, w, h);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (m_splitView)
    {
        // Exyn-style layout: onboard camera view on top, reconstructed map
        // below, with a thin divider between the panes.
        int paneH = (h - 2) / 2;
        RenderFpvPane(0, h - paneH, w, paneH);
        glClear(GL_DEPTH_BUFFER_BIT);
        RenderMapPane(0, 0, w, paneH);
    }
    else
    {
        RenderMapPane(0, 0, w, h);
    }
}

void App::RenderMapPane(int x, int y, int w, int h)
{
    glViewport(x, y, w, h);
    Mat4 proj = Mat4::Perspective(1.05f, static_cast<float>(w) / static_cast<float>(h), 0.1f, 300.0f);
    Mat4 viewProj = proj * m_camera.View();

    m_voxelRenderer.Draw(viewProj, m_camera.Eye());

    m_lineRenderer.Begin();
    for (size_t i = 0; i < m_agents.size(); ++i)
    {
        const AgentState& agent = m_agents[i];
        const Vec3& trailColor = kTrailColors[i % kColorCount];
        const Vec3& pathColor = kPathColors[i % kColorCount];

        m_lineRenderer.AddPolyline(agent.trail, trailColor);
        if (!agent.path.empty())
        {
            m_lineRenderer.AddPolyline(agent.path, pathColor);
            const Vec3& g = agent.path.back();
            const float s = 0.4f;
            m_lineRenderer.AddLine(g - Vec3{s, 0, 0}, g + Vec3{s, 0, 0}, pathColor);
            m_lineRenderer.AddLine(g - Vec3{0, s, 0}, g + Vec3{0, s, 0}, pathColor);
            m_lineRenderer.AddLine(g - Vec3{0, 0, s}, g + Vec3{0, 0, s}, pathColor);
        }
    }
    m_lineRenderer.Draw(viewProj);

    for (const AgentState& agent : m_agents)
        m_droneRenderer.Draw(viewProj, agent.drone);
}

void App::RenderFpvPane(int x, int y, int w, int h)
{
    glViewport(x, y, w, h);

    // Onboard camera of the focused drone: mounted slightly above and ahead
    // of the body center, using the physical orientation, so the picture
    // banks into turns exactly like real FPV footage.
    const Drone& focused = m_agents[m_focus].drone;
    const Quat& q = focused.Orientation();
    Vec3 forward = q.Rotate(Vec3{0.0f, 0.0f, 1.0f});
    Vec3 up = q.Rotate(Vec3{0.0f, 1.0f, 0.0f});
    Vec3 eye = focused.Position() + forward * 0.18f + up * 0.06f;

    Mat4 proj = Mat4::Perspective(1.35f, static_cast<float>(w) / static_cast<float>(h), 0.05f, 120.0f);
    Mat4 view = Mat4::LookAt(eye, eye + forward, up);

    m_truthRenderer.Draw(proj * view, focused.Position());

    // Other fleet members are physically present in the cave, so the
    // onboard camera sees them when they cross its view
    for (size_t i = 0; i < m_agents.size(); ++i)
        if (static_cast<int>(i) != m_focus)
            m_droneRenderer.Draw(proj * view, m_agents[i].drone);
}

void App::UpdateTitle(double fps)
{
    char title[352];
    const char* status = "exploring";
    if (m_phase == Phase::Returning) status = "returning home";
    else if (m_phase == Phase::Home) status = "mission complete";
    if (m_paused) status = "paused";
    const char* camMode = m_camera.GetMode() == Camera::Mode::Chase ? "chase" : "orbit";
    _snprintf_s(title, _TRUNCATE,
        "CaveDroneSim | %s | %d drones, fpv %d | cam %s | free %d occ %d | chunks %d/%d | %.0f fps | 1-%d focus, V split, C camera, Space pause, R new world",
        status, DRONE_COUNT, m_focus + 1, camMode,
        m_map.FreeCount(), m_map.OccupiedCount(),
        m_voxelRenderer.DrawnChunkCount(), m_voxelRenderer.MeshedChunkCount(),
        fps, DRONE_COUNT);
    m_window.SetTitle(title);
}

int App::Run()
{
    LARGE_INTEGER freq, prev, now;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&prev);

    while (m_window.PumpMessages())
    {
        QueryPerformanceCounter(&now);
        double dt = static_cast<double>(now.QuadPart - prev.QuadPart) / static_cast<double>(freq.QuadPart);
        prev = now;
        if (dt > 0.1) dt = 0.1;

        HandleInput(dt);
        if (!m_paused)
            UpdateSimulation(dt);

        m_voxelRenderer.UpdateMeshes(m_map, MESH_REBUILDS_PER_FRAME);
        const Drone& focused = m_agents[m_focus].drone;
        m_camera.Update(static_cast<float>(dt), focused.Position(), focused.Velocity(), m_map);

        Render();
        m_window.SwapBuffers();
        m_window.EndFrameInput();

        m_titleTimer -= dt;
        if (m_titleTimer <= 0.0)
        {
            UpdateTitle(dt > 1e-6 ? 1.0 / dt : 0.0);
            m_titleTimer = 0.25;
        }
    }
    return 0;
}
