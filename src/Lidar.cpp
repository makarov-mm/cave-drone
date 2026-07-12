#include "Lidar.h"
#include "Config.h"

void Lidar::Scan(const Vec3& origin, const World& world, VoxelMap& map)
{
    std::uniform_real_distribution<float> uni(0.0f, 1.0f);
    std::normal_distribution<float> gauss(0.0f, LIDAR_NOISE_SIGMA);
    const float phase = uni(m_rng);
    const float golden = 2.399963f; // golden angle in radians

    for (int i = 0; i < LIDAR_RAYS_PER_FRAME; ++i)
    {
        if (uni(m_rng) < LIDAR_DROPOUT_PROB)
            continue;

        // Fibonacci sphere with random per-frame phase offset
        float t = (static_cast<float>(i) + phase) / static_cast<float>(LIDAR_RAYS_PER_FRAME);
        float y = 1.0f - 2.0f * t;
        float r = std::sqrt(std::fmax(0.0f, 1.0f - y * y));
        float a = golden * (static_cast<float>(i) + phase * 31.0f);
        Vec3 dir{r * std::cos(a), y, r * std::sin(a)};

        float trueRange = world.HitDistance(origin, dir, LIDAR_RANGE);
        bool hasReturn = trueRange <= LIDAR_RANGE;

        float measured;
        if (uni(m_rng) < LIDAR_OUTLIER_PROB)
        {
            // Spurious short return somewhere in front of the true surface
            float limit = hasReturn ? trueRange : LIDAR_RANGE;
            measured = 0.4f + uni(m_rng) * std::fmax(0.1f, limit - 0.4f);
            hasReturn = true;
        }
        else if (hasReturn)
        {
            measured = Clamp(trueRange + gauss(m_rng), 0.05f, LIDAR_RANGE);
        }
        else
        {
            measured = LIDAR_RANGE; // no return: free space up to max range
        }

        IntegrateRay(origin, dir, measured, hasReturn, map);
    }
}

void Lidar::IntegrateRay(const Vec3& origin, const Vec3& dir,
                         float measured, bool hasReturn, VoxelMap& map)
{
    const float inv = 1.0f / VOXEL_SIZE;
    int ix = static_cast<int>(std::floor(origin.x * inv));
    int iy = static_cast<int>(std::floor(origin.y * inv));
    int iz = static_cast<int>(std::floor(origin.z * inv));

    int stepX = dir.x > 0.0f ? 1 : -1;
    int stepY = dir.y > 0.0f ? 1 : -1;
    int stepZ = dir.z > 0.0f ? 1 : -1;

    auto boundary = [](float o, float d, int i, int step)
    {
        float edge = (step > 0 ? static_cast<float>(i + 1) : static_cast<float>(i)) * VOXEL_SIZE;
        return (edge - o) / d;
    };

    float tMaxX = std::fabs(dir.x) > 1e-8f ? boundary(origin.x, dir.x, ix, stepX) : 1e30f;
    float tMaxY = std::fabs(dir.y) > 1e-8f ? boundary(origin.y, dir.y, iy, stepY) : 1e30f;
    float tMaxZ = std::fabs(dir.z) > 1e-8f ? boundary(origin.z, dir.z, iz, stepZ) : 1e30f;

    float tDeltaX = std::fabs(dir.x) > 1e-8f ? VOXEL_SIZE / std::fabs(dir.x) : 1e30f;
    float tDeltaY = std::fabs(dir.y) > 1e-8f ? VOXEL_SIZE / std::fabs(dir.y) : 1e30f;
    float tDeltaZ = std::fabs(dir.z) > 1e-8f ? VOXEL_SIZE / std::fabs(dir.z) : 1e30f;

    // Walk cells; the cell containing the measured endpoint gets a hit
    // update, cells before it get a miss update, with one exception: a cell
    // that the ray merely clips with a short chord right in front of the
    // return gets nothing. Rays grazing a wall at a shallow angle clip
    // solid corner cells that way, and integrating misses there would
    // slowly erode real geometry. Head-on rays traverse the last free cell
    // with a long chord and keep integrating misses normally.
    const float missMargin = hasReturn ? 3.0f * LIDAR_NOISE_SIGMA : 0.0f;
    const float shortChord = VOXEL_SIZE * 0.5f;
    float tEnter = 0.0f;
    for (;;)
    {
        float tExit = tMaxX < tMaxY ? (tMaxX < tMaxZ ? tMaxX : tMaxZ)
                                    : (tMaxY < tMaxZ ? tMaxY : tMaxZ);

        if (hasReturn && measured >= tEnter && measured < tExit)
        {
            map.IntegrateHit(ix, iy, iz);
            return;
        }
        if (tEnter >= measured)
            return;
        bool cornerClip = tExit > measured - missMargin && tExit - tEnter < shortChord;
        if (!cornerClip)
            map.IntegrateMiss(ix, iy, iz);

        if (tMaxX < tMaxY && tMaxX < tMaxZ)
        {
            tMaxX += tDeltaX; ix += stepX;
        }
        else if (tMaxY < tMaxZ)
        {
            tMaxY += tDeltaY; iy += stepY;
        }
        else
        {
            tMaxZ += tDeltaZ; iz += stepZ;
        }
        tEnter = tExit;
    }
}
