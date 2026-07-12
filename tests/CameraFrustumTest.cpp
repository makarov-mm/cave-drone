// Native logic test for the frustum math and the chase camera occlusion.
// Compile:
//   g++ -O2 -std=c++17 tests/CameraFrustumTest.cpp src/VoxelMap.cpp -Isrc -o camera_test
#include "../src/Math.h"
#include "../src/VoxelMap.h"
#include "../src/Camera.h"
#include <cstdio>

static int failures = 0;
#define CHECK(cond, msg) do { \
    if (!(cond)) { std::printf("FAIL: %s\n", msg); ++failures; } \
    else { std::printf("ok:   %s\n", msg); } } while (0)

int main()
{
    // --- Frustum: camera at origin looking down -Z
    Mat4 proj = Mat4::Perspective(1.05f, 16.0f / 9.0f, 0.1f, 300.0f);
    Mat4 view = Mat4::LookAt({0, 0, 0}, {0, 0, -1}, {0, 1, 0});
    Frustum f = Frustum::FromViewProj(proj * view);

    CHECK(f.IntersectsAabb({-1, -1, -6}, {1, 1, -4}), "box in front is visible");
    CHECK(!f.IntersectsAabb({-1, -1, 4}, {1, 1, 6}), "box behind camera is culled");
    CHECK(!f.IntersectsAabb({-102, -1, -6}, {-100, 1, -4}), "box far left is culled");
    CHECK(!f.IntersectsAabb({-1, -1, -500}, {1, 1, -400}), "box beyond far plane is culled");
    CHECK(f.IntersectsAabb({-50, -50, -60}, {50, 50, -1}), "large box spanning frustum is visible");

    // --- Occlusion raycast: wall of occupied cells at x = 10 (cells 20..)
    VoxelMap map;
    for (int y = 0; y < 20; ++y)
        for (int z = 0; z < 20; ++z)
            map.ForceOccupied(20, y, z);

    float d = map.OccludedDistance({5.0f, 5.0f, 5.0f}, {1, 0, 0}, 30.0f);
    CHECK(d > 4.9f && d < 5.1f, "raycast hits wall at expected distance");
    d = map.OccludedDistance({5.0f, 5.0f, 5.0f}, {-1, 0, 0}, 30.0f);
    CHECK(d >= 30.0f, "raycast away from wall reaches maxDist");

    // --- Chase camera: without occlusion sits behind at full distance,
    //     with a wall behind the drone it pulls in
    Camera cam;
    cam.Snap({5.0f, 5.0f, 5.0f});
    Vec3 vel{0.0f, 0.0f, 2.0f}; // flying +Z, camera should sit at -Z side
    VoxelMap empty;
    for (int i = 0; i < 300; ++i)
        cam.Update(1.0f / 60.0f, {5.0f, 5.0f, 5.0f}, vel, empty);
    Vec3 eye = cam.Eye();
    CHECK(eye.z < 5.0f, "chase camera sits behind flight direction");
    float fullDist = Length(eye - Vec3{5.0f, 5.25f, 5.0f});
    CHECK(fullDist > 6.0f, "no occlusion: full chase distance");

    // Wall crossing the camera ray behind the drone
    VoxelMap walled;
    for (int y = 0; y < 40; ++y)
        for (int x = 0; x < 40; ++x)
            walled.ForceOccupied(x, y, 4); // wall at z = 2.0m
    for (int i = 0; i < 300; ++i)
        cam.Update(1.0f / 60.0f, {5.0f, 5.0f, 5.0f}, vel, walled);
    float pulledDist = Length(cam.Eye() - Vec3{5.0f, 5.25f, 5.0f});
    CHECK(pulledDist < fullDist * 0.75f, "occluding wall pulls camera in");
    CHECK(cam.Eye().z > 2.5f, "camera stays on the near side of the wall");

    std::printf("%s\n", failures == 0 ? "ALL PASS" : "FAILURES");
    return failures == 0 ? 0 : 1;
}
