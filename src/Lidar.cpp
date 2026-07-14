#include "Lidar.h"
#include "Config.h"
#include "VoxelDda.h"
#include <numbers>

void Lidar::Scan(const Vec3& origin, const World& world, VoxelMap& map,
                 std::vector<Vec3>* hitSink)
{
    std::uniform_real_distribution<float> uni(0.0f, 1.0f);
    std::normal_distribution<float> gauss(0.0f, LIDAR_NOISE_SIGMA);
    const float phase = uni(m_rng);
    constexpr float golden = 2.399963f; // golden angle in radians

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

        if (hasReturn && hitSink != nullptr)
            hitSink->push_back(origin + dir * measured);
        IntegrateRay(origin, dir, measured, hasReturn, map);
    }
}

void Lidar::IntegrateRay(const Vec3& origin, const Vec3& dir,
                         float measured, bool hasReturn, VoxelMap& map)
{
    // Walk cells; the cell containing the measured endpoint gets a hit
    // update, cells before it get a miss update, with one exception: a cell
    // that the ray merely clips with a short chord right in front of the
    // return gets nothing. Rays grazing a wall at a shallow angle clip
    // solid corner cells that way, and integrating misses there would
    // slowly erode real geometry. Head-on rays traverse the last free cell
    // with a long chord and keep integrating misses normally.
    const float missMargin = hasReturn ? 3.0f * LIDAR_NOISE_SIGMA : 0.0f;
    const float shortChord = VOXEL_SIZE * 0.5f;

    for (VoxelDda dda(origin, dir);; dda.Step())
    {
        const float tEnter = dda.EnterT();
        const float tExit = dda.ExitT();

        if (hasReturn && measured >= tEnter && measured < tExit)
        {
            map.IntegrateHit(dda.X(), dda.Y(), dda.Z());
            return;
        }
        if (tEnter >= measured)
            return;
        bool cornerClip = tExit > measured - missMargin && tExit - tEnter < shortChord;
        if (!cornerClip)
            map.IntegrateMiss(dda.X(), dda.Y(), dda.Z());
    }
}
