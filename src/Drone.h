#pragma once
#include "Math.h"
#include <functional>
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

    // Segment test against the observed map, supplied by the coordinator.
    // Must return true when the straight chord between the two points lies
    // through known-free cells.
    using ChordClear = std::function<bool(const Vec3&, const Vec3&)>;

    // Reflex probe against the observed map: no occupied cell from the
    // point along the unit direction within the distance.
    using ProbeClear = std::function<bool(const Vec3&, const Vec3&, float)>;

    // Advances physics. Path is the current planned polyline (world
    // coords). separation is an extra velocity command from drone-drone
    // repulsion. chordClear bounds the pursuit point: the planner only
    // validates the polyline itself, and an unconstrained lookahead cuts
    // corners at sharp bends beyond the one-voxel safety margin, which is
    // exactly how the drone used to clip walls the map already knew about.
    void Update(float dt, const std::vector<Vec3>& path, const Vec3& separation,
                const ChordClear& chordClear, const ProbeClear& probeClear);

    const Vec3& Position() const { return m_position; }
    const Vec3& Velocity() const { return m_velocity; }
    const Quat& Orientation() const { return m_orientation; }
    float Speed() const { return Length(m_velocity); }

    // Index of the path segment currently being tracked
    size_t PathProgress() const { return m_segmentIndex; }

    // True when the reflex brake has been firing for a sustained period:
    // the follower is grinding against known geometry and cannot make
    // progress on its own. The coordinator should discard the current path
    // (and blacklist the goal while exploring) and replan.
    [[nodiscard]] bool ReflexSaturated() const { return m_reflexTime > 1.0f; }
    void ResetPathProgress() { m_segmentIndex = 0; m_segmentT = 0.0f; }

    float RotorAngle() const { return m_rotorAngle; }

private:
    Vec3 m_position;
    Vec3 m_velocity;
    Quat m_orientation;
    Vec3 m_hoverTarget;
    bool m_anchorLatched = false;
    Vec3 m_anchor;
    size_t m_segmentIndex = 0;
    float m_segmentT = 0.0f;
    float m_yaw = 0.0f;
    float m_rotorAngle = 0.0f;
    float m_rotorSpeed = 0.0f;
    float m_reflexTime = 0.0f;
    float m_carefulTime = 0.0f;

    Vec3 ComputePursuitPoint(const std::vector<Vec3>& path, float& distToEnd,
                             const ChordClear& chordClear);
    void ApplyReflexAndIntegrate(float dt, Vec3 desiredVel, const ProbeClear& probeClear);
};
