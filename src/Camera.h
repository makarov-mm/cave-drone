#pragma once
#include "Math.h"
#include "VoxelMap.h"

// Two camera modes:
//
// Chase (default): sits behind the drone and smoothly aligns with the
// direction of flight. A DDA raycast against the observed map pulls the
// camera in when a discovered wall would block the view of the drone;
// the pull-in is fast, the release back to full distance is slow, so
// there is no popping. Undiscovered geometry is not rendered and thus
// intentionally does not occlude.
//
// Orbit: free mouse-controlled orbit around the drone, same occlusion
// handling.
class Camera
{
public:
    enum class Mode { Chase, Orbit };

    void Update(float dt, const Vec3& dronePos, const Vec3& droneVel, const VoxelMap& map)
    {
        // Smoothed look-at target slightly above the drone
        Vec3 desiredTarget = dronePos + Vec3{0.0f, 0.25f, 0.0f};
        float targetBlend = 1.0f - std::exp(-8.0f * dt);
        m_target = Lerp(m_target, desiredTarget, targetBlend);

        Vec3 eyeDir;
        float wantDist;
        if (m_mode == Mode::Chase)
        {
            // Follow direction tracks the velocity, but only when actually
            // flying: while hovering the camera keeps its last heading.
            float speed = Length(droneVel);
            if (speed > 0.6f)
            {
                float dirBlend = 1.0f - std::exp(-2.5f * dt);
                m_followDir = Normalize(Lerp(m_followDir, droneVel * (1.0f / speed), dirBlend));
            }
            eyeDir = Normalize(m_followDir * -1.0f + Vec3{0.0f, 0.5f, 0.0f});
            wantDist = m_chaseDist;
        }
        else
        {
            eyeDir = Vec3{
                std::cos(m_pitch) * std::sin(m_yaw),
                std::sin(m_pitch),
                std::cos(m_pitch) * std::cos(m_yaw)};
            wantDist = m_orbitDist;
        }

        // Occlusion: never place the camera behind a discovered wall
        float occluded = map.OccludedDistance(m_target, eyeDir, wantDist);
        float allowed = Clamp(occluded - 0.4f, 0.8f, wantDist);
        if (allowed < m_curDist)
            m_curDist += (allowed - m_curDist) * (1.0f - std::exp(-16.0f * dt)); // fast pull-in
        else
            m_curDist += (allowed - m_curDist) * (1.0f - std::exp(-2.0f * dt));  // slow release

        m_eye = m_target + eyeDir * m_curDist;
    }

    void ToggleMode()
    {
        if (m_mode == Mode::Chase)
        {
            // Hand over smoothly: start the orbit where the chase cam looks
            m_mode = Mode::Orbit;
            Vec3 dir = Normalize(m_eye - m_target);
            m_pitch = std::asin(Clamp(dir.y, -1.0f, 1.0f));
            m_yaw = std::atan2(dir.x, dir.z);
            m_orbitDist = m_curDist;
        }
        else
        {
            m_mode = Mode::Chase;
        }
    }

    Mode GetMode() const { return m_mode; }

    void Rotate(float dYaw, float dPitch)
    {
        if (m_mode != Mode::Orbit)
            return;
        m_yaw += dYaw;
        m_pitch = Clamp(m_pitch + dPitch, -1.45f, 1.45f);
    }

    void Zoom(float steps)
    {
        float& dist = m_mode == Mode::Chase ? m_chaseDist : m_orbitDist;
        dist = Clamp(dist * std::pow(0.88f, steps), 2.0f, 60.0f);
    }

    Vec3 Eye() const { return m_eye; }

    Mat4 View() const
    {
        return Mat4::LookAt(m_eye, m_target, Vec3{0.0f, 1.0f, 0.0f});
    }

    void Snap(const Vec3& target)
    {
        m_target = target;
        m_followDir = Vec3{0.0f, 0.0f, 1.0f};
        m_curDist = m_chaseDist;
        m_eye = m_target + Normalize(m_followDir * -1.0f + Vec3{0.0f, 0.5f, 0.0f}) * m_curDist;
    }

private:
    Mode m_mode = Mode::Chase;
    Vec3 m_target;
    Vec3 m_eye;
    Vec3 m_followDir{0.0f, 0.0f, 1.0f};
    float m_chaseDist = 7.0f;
    float m_orbitDist = 14.0f;
    float m_curDist = 7.0f;
    float m_yaw = 0.7f;
    float m_pitch = 0.45f;
};
