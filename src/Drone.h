#pragma once
#include "Math.h"
#include <vector>

// Simplified quadrotor dynamics. The drone is a point mass with an attitude:
// thrust always acts along the body up axis, so the vehicle must physically
// bank to accelerate sideways, which produces realistic tilting into turns.
// Control cascade: pursuit point -> desired velocity -> desired specific
// force -> desired attitude + thrust magnitude -> first-order attitude
// tracking -> translational integration with quadratic drag.
class Drone
{
public:
    void Reset(const Vec3& position);

    // Advances physics. Path is the current planned polyline (world
    // coords). separation is an extra velocity command from drone-drone
    // repulsion, computed by the fleet coordinator.
    void Update(float dt, const std::vector<Vec3>& path, const Vec3& separation = Vec3{});

    const Vec3& Position() const { return m_position; }
    const Vec3& Velocity() const { return m_velocity; }
    const Quat& Orientation() const { return m_orientation; }
    float Speed() const { return Length(m_velocity); }

    // Index of the path segment currently being tracked
    size_t PathProgress() const { return m_segmentIndex; }
    void ResetPathProgress() { m_segmentIndex = 0; m_segmentT = 0.0f; }

    float RotorAngle() const { return m_rotorAngle; }

private:
    Vec3 m_position;
    Vec3 m_velocity;
    Quat m_orientation;
    Vec3 m_hoverTarget;
    size_t m_segmentIndex = 0;
    float m_segmentT = 0.0f;
    float m_yaw = 0.0f;
    float m_rotorAngle = 0.0f;
    float m_rotorSpeed = 0.0f;

    Vec3 ComputePursuitPoint(const std::vector<Vec3>& path, float& distToEnd);
};
