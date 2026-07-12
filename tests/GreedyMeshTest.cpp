// Correctness test for the greedy mesher. For a set of occupancy patterns
// it reconstructs the emitted quads and rasterizes them into per-direction
// coverage grids, then verifies against a naive face enumeration:
// every visible face is covered exactly once, no invisible face is covered.
// Also reports the compression ratio versus naive per-face meshing.
// Compile:
//   g++ -O2 -std=c++17 tests/GreedyMeshTest.cpp src/Meshing.cpp -Isrc -o greedy_test
#include "../src/Meshing.h"
#include <cstdio>
#include <cstring>
#include <cmath>
#include <random>
#include <vector>

static int failures = 0;
static const int N = CHUNK_SIZE;

static bool At(const uint8_t* padded, int x, int y, int z)
{
    return padded[PaddedIndex(x, y, z)] != 0;
}

// coverage[axis][signIdx][slice][b][a]
using Coverage = std::vector<int>;
static int CoverIndex(int axis, int signIdx, int slice, int b, int a)
{
    return (((axis * 2 + signIdx) * N + slice) * N + b) * N + a;
}

static bool CheckPattern(const uint8_t* padded, const char* name, bool report)
{
    std::vector<float> verts;
    GreedyMeshChunk(padded, Vec3{0, 0, 0}, verts);

    if (verts.size() % (6 * 8) != 0)
    {
        std::printf("FAIL %s: vertex count not a multiple of one quad\n", name);
        return false;
    }
    int quadCount = static_cast<int>(verts.size() / (6 * 8));

    Coverage cover(3 * 2 * N * N * N, 0);

    for (int q = 0; q < quadCount; ++q)
    {
        const float* v = verts.data() + q * 6 * 8;
        // Normal from the first vertex
        float nx = v[3], ny = v[4], nz = v[5];
        int axis = std::fabs(nx) > 0.5f ? 0 : (std::fabs(ny) > 0.5f ? 1 : 2);
        float s = (axis == 0 ? nx : (axis == 1 ? ny : nz));
        int signIdx = s > 0.0f ? 1 : 0;
        int uAxis = (axis + 1) % 3;
        int vAxis = (axis + 2) % 3;

        // Rect extents over the 6 vertices
        float mn[3] = {1e9f, 1e9f, 1e9f}, mx[3] = {-1e9f, -1e9f, -1e9f};
        for (int i = 0; i < 6; ++i)
            for (int c = 0; c < 3; ++c)
            {
                float p = v[i * 8 + c];
                if (p < mn[c]) mn[c] = p;
                if (p > mx[c]) mx[c] = p;
            }
        if (std::fabs(mx[axis] - mn[axis]) > 1e-4f)
        {
            std::printf("FAIL %s: quad not planar along its normal\n", name);
            return false;
        }
        int plane = static_cast<int>(std::lround(mn[axis] / VOXEL_SIZE));
        int slice = signIdx == 1 ? plane - 1 : plane;
        int a0 = static_cast<int>(std::lround(mn[uAxis] / VOXEL_SIZE));
        int w = static_cast<int>(std::lround((mx[uAxis] - mn[uAxis]) / VOXEL_SIZE));
        int b0 = static_cast<int>(std::lround(mn[vAxis] / VOXEL_SIZE));
        int h = static_cast<int>(std::lround((mx[vAxis] - mn[vAxis]) / VOXEL_SIZE));
        if (w < 1 || h < 1 || slice < 0 || slice >= N)
        {
            std::printf("FAIL %s: degenerate quad rect\n", name);
            return false;
        }
        for (int b = b0; b < b0 + h; ++b)
            for (int a = a0; a < a0 + w; ++a)
                ++cover[CoverIndex(axis, signIdx, slice, b, a)];
    }

    // Naive enumeration of visible faces
    int naiveFaces = 0;
    for (int axis = 0; axis < 3; ++axis)
    {
        int uAxis = (axis + 1) % 3;
        int vAxis = (axis + 2) % 3;
        for (int signIdx = 0; signIdx < 2; ++signIdx)
        {
            int sign = signIdx == 1 ? 1 : -1;
            for (int slice = 0; slice < N; ++slice)
                for (int b = 0; b < N; ++b)
                    for (int a = 0; a < N; ++a)
                    {
                        int c[3];
                        c[axis] = slice; c[uAxis] = a; c[vAxis] = b;
                        int n[3] = {c[0], c[1], c[2]};
                        n[axis] += sign;
                        bool visible = At(padded, c[0], c[1], c[2]) &&
                                       !At(padded, n[0], n[1], n[2]);
                        naiveFaces += visible ? 1 : 0;
                        int got = cover[CoverIndex(axis, signIdx, slice, b, a)];
                        int want = visible ? 1 : 0;
                        if (got != want)
                        {
                            std::printf(
                                "FAIL %s: coverage %d != %d at axis %d sign %d slice %d a %d b %d\n",
                                name, got, want, axis, sign, slice, a, b);
                            return false;
                        }
                    }
        }
    }

    if (report)
        std::printf("ok:   %-22s faces %6d -> quads %5d (%.1fx)\n",
                    name, naiveFaces, quadCount,
                    quadCount > 0 ? static_cast<double>(naiveFaces) / quadCount : 1.0);
    else
        std::printf("ok:   %s\n", name);
    return true;
}

int main()
{
    std::vector<uint8_t> padded(PADDED_SIZE * PADDED_SIZE * PADDED_SIZE);

    auto set = [&padded](int x, int y, int z, uint8_t v)
    {
        padded[PaddedIndex(x, y, z)] = v;
    };
    auto clear = [&padded]() { std::fill(padded.begin(), padded.end(), 0); };

    // Empty chunk
    clear();
    if (!CheckPattern(padded.data(), "empty", false)) ++failures;

    // Single voxel
    clear();
    set(7, 7, 7, 1);
    if (!CheckPattern(padded.data(), "single voxel", true)) ++failures;

    // Full chunk with empty surroundings: 6 faces of N*N each
    clear();
    for (int y = 0; y < N; ++y)
        for (int z = 0; z < N; ++z)
            for (int x = 0; x < N; ++x)
                set(x, y, z, 1);
    if (!CheckPattern(padded.data(), "full chunk", true)) ++failures;

    // Flat plate: best case for merging
    clear();
    for (int z = 0; z < N; ++z)
        for (int x = 0; x < N; ++x)
            set(x, 5, z, 1);
    if (!CheckPattern(padded.data(), "flat plate", true)) ++failures;

    // L-shaped slab: forces non-trivial rectangle decomposition
    clear();
    for (int z = 0; z < N; ++z)
        for (int x = 0; x < N; ++x)
            if (x < 8 || z < 8)
                set(x, 3, z, 1);
    if (!CheckPattern(padded.data(), "L-shaped slab", true)) ++failures;

    // Random fills, including a random border ring from "neighbor chunks"
    for (uint32_t seed = 1; seed <= 5; ++seed)
    {
        std::mt19937 rng(seed);
        std::uniform_real_distribution<float> uni(0.0f, 1.0f);
        clear();
        for (int y = -1; y <= N; ++y)
            for (int z = -1; z <= N; ++z)
                for (int x = -1; x <= N; ++x)
                    set(x, y, z, uni(rng) < 0.3f ? 1 : 0);
        char name[64];
        std::snprintf(name, sizeof(name), "random 30%% seed %u", seed);
        if (!CheckPattern(padded.data(), name, true)) ++failures;
    }

    std::printf("%s\n", failures == 0 ? "ALL PASS" : "FAILURES");
    return failures == 0 ? 0 : 1;
}
