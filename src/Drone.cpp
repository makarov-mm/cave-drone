#include "Drone.h"
#include "Config.h"

void Drone::Reset(const Vec3& position)
{
    m_position = position;
    m_velocity = Vec3();
    m_orientation = Quat();
    m_hoverTarget = position;
    m_segmentIndex = 0;
    m_segmentT = 0.0f;
    m_yaw = 0.0f;
    m_rotorAngle = 0.0f;
    m_rotorSpeed = 30.0f;
}

Vec3 Drone::ComputePursuitPoint(const std::vector<Vec3>& path, float& distToEnd)
{
    // Advance a monotonic parameter along the polyline while the pursuit
    // point stays within lookahead distance of the drone.
    while (m_segmentIndex + 1 < path.size())
    {
        Vec3 a = path[m_segmentIndex];
        Vec3 b = path[m_segmentIndex + 1];
        Vec3 seg = b - a;
        float segLen = Length(seg);
        if (segLen < 1e-5f)
        {
            ++m_segmentIndex;
            m_segmentT = 0.0f;
            continue;
        }
        Vec3 point = a + seg * m_segmentT;
        if (Length(point - m_position) < DRONE_LOOKAHEAD)
        {
            m_segmentT += (VOXEL_SIZE * 0.25f) / segLen;
            if (m_segmentT >= 1.0f)
            {
                ++m_segmentIndex;
                m_segmentT = 0.0f;
            }
            continue;
        }
        break;
    }

    Vec3 pursuit;
    if (m_segmentIndex + 1 < path.size())
    {
        Vec3 a = path[m_segmentIndex];
        Vec3 b = path[m_segmentIndex + 1];
        pursuit = a + (b - a) * m_segmentT;
    }
    else
    {
        pursuit = path.back();
    }

    // Remaining distance: pursuit point to path end, approximated by segments
    distToEnd = Length(path.back() - m_position);
    return pursuit;
}

void Drone::Update(float dt, const std::vector<Vec3>& path, const Vec3& separation)
{
    const Vec3 up{0.0f, 1.0f, 0.0f};

    // 1. Desired velocity from pursuit point
    Vec3 desiredVel;
    if (!path.empty())
    {
        float distToEnd = 0.0f;
        Vec3 pursuit = ComputePursuitPoint(path, distToEnd);
        Vec3 toPursuit = pursuit - m_position;
        float d = Length(toPursuit);
        float speed = DRONE_CRUISE_SPEED;
        speed = std::fmin(speed, 1.8f * distToEnd + 0.15f); // brake near goal

        // Brake into turns: scale speed by how well the current velocity
        // aligns with the desired direction, like a pilot slowing before
        // a sharp corner.
        if (Speed() > 0.5f && d > 1e-4f)
        {
            float align = Dot(m_velocity * (1.0f / Speed()), toPursuit * (1.0f / d));
            speed *= 0.35f + 0.65f * Clamp(align, 0.0f, 1.0f);
        }
        if (d > 1e-4f)
            desiredVel = toPursuit * (speed / std::fmax(d, DRONE_LOOKAHEAD * 0.5f));
        m_hoverTarget = m_position;
    }
    else
    {
        // Position hold
        desiredVel = (m_hoverTarget - m_position) * 1.5f;
    }
    desiredVel += separation;

    // 2. Desired specific force (velocity P loop + gravity compensation)
    Vec3 accCmd = (desiredVel - m_velocity) * DRONE_VEL_GAIN;
    Vec3 horiz{accCmd.x, 0.0f, accCmd.z};
    float h = Length(horiz);
    if (h > DRONE_MAX_HORIZ_ACC)
    {
        horiz *= DRONE_MAX_HORIZ_ACC / h;
        accCmd.x = horiz.x;
        accCmd.z = horiz.z;
    }
    Vec3 forceCmd = accCmd + up * GRAVITY;
    float f = Length(forceCmd);
    if (f > DRONE_MAX_TOTAL_ACC)
        forceCmd *= DRONE_MAX_TOTAL_ACC / f;
    if (forceCmd.y < 1.0f)
        forceCmd.y = 1.0f; // keep thrust vector pointing generally up

    // 3. Desired attitude: body up along the force command, yaw along travel
    Vec3 desiredUp = Normalize(forceCmd);
    Vec3 velFlat{m_velocity.x, 0.0f, m_velocity.z};
    if (Length(velFlat) > 0.4f)
        m_yaw = std::atan2(velFlat.x, velFlat.z);
    Vec3 heading{std::sin(m_yaw), 0.0f, std::cos(m_yaw)};
    Vec3 right = Normalize(Cross(desiredUp, heading));
    if (Length(right) < 1e-4f)
        right = Vec3{1.0f, 0.0f, 0.0f};
    Vec3 forward = Cross(right, desiredUp);
    Quat desired = Quat::FromBasis(right, desiredUp, forward);

    // 4. First-order attitude tracking (motor/ESC lag abstraction)
    float blend = 1.0f - std::exp(-DRONE_ATTITUDE_RATE * dt);
    m_orientation = Quat::Nlerp(m_orientation, desired, blend);

    // 5. Thrust acts along the actual body up axis
    Vec3 bodyUp = m_orientation.Rotate(up);
    float thrust = Dot(forceCmd, bodyUp);
    thrust = Clamp(thrust, 0.0f, DRONE_MAX_TOTAL_ACC);

    Vec3 drag = m_velocity * (-DRONE_DRAG * Length(m_velocity));
    Vec3 accel = bodyUp * thrust - up * GRAVITY + drag;

    m_velocity += accel * dt;
    m_position += m_velocity * dt;

    // Rotor animation: spin rate follows thrust demand
    m_rotorSpeed = 25.0f + thrust * 4.0f;
    m_rotorAngle += m_rotorSpeed * dt;
    if (m_rotorAngle > 6.2831853f)
        m_rotorAngle -= 6.2831853f;
}
