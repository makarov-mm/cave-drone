#pragma once
#include "Math.h"
#include "World.h"
#include "VoxelMap.h"
#include <random>
#include <vector>

// Simulated spinning lidar with a realistic noise model:
// - Gaussian range noise on every return
// - occasional dropouts (ray returns nothing)
// - occasional spurious short returns (dust, multipath)
//
// Each frame casts a batch of rays in a Fibonacci-sphere pattern with a
// random phase, so coverage fills in over time like a real rotating
// scanner. Evidence is integrated into the log-odds map along the
// *measured* range, not the true one, exactly like a real fusion pipeline:
// phantom voxels from outliers appear and later dissolve under
// contradicting evidence.
class Lidar
{
public:
    explicit Lidar(uint32_t seed) : m_rng(seed) {}

    // hitSink, if given, receives the measured hit points of this scan
    // (for point-cloud visualization)
    void Scan(const Vec3& origin, const World& world, VoxelMap& map,
              std::vector<Vec3>* hitSink = nullptr);

private:
    std::mt19937 m_rng;

    void IntegrateRay(const Vec3& origin, const Vec3& dir,
                      float measured, bool hasReturn, VoxelMap& map);
};
