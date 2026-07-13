#pragma once
#include "GlLoader.h"
#include "Shader.h"
#include "Math.h"
#include <vector>

// Renders the raw lidar returns as an accumulating point cloud, the way
// real SLAM tooling visualizes a scan. Points live in a fixed-capacity GPU
// ring buffer: new returns overwrite the oldest ones, uploads happen only
// for the ranges written this frame. Color comes from a height palette in
// the shader; point size attenuates with distance.
class PointCloudRenderer
{
public:
    bool Init();
    void Shutdown();

    void AddPoints(const std::vector<Vec3>& points);
    void Clear();

    void Draw(const Mat4& viewProj, const Vec3& cameraPos) const;

    int PointCount() const { return m_count; }

private:
    static constexpr int kCapacity = 400000;

    Shader m_shader;
    GLuint m_vao = 0;
    GLuint m_vbo = 0;
    int m_head = 0;   // next write slot in the ring
    int m_count = 0;  // valid points, up to kCapacity
};
