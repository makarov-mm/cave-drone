#include "Meshing.h"

namespace
{
    void EmitQuad(std::vector<float>& out, const Vec3& p,
                  const Vec3& uVec, const Vec3& vVec, const Vec3& normal,
                  float w, float h)
    {
        const Vec3 corners[4] = {p, p + uVec, p + uVec + vVec, p + vVec};
        const float uvs[4][2] = {{0.0f, 0.0f}, {w, 0.0f}, {w, h}, {0.0f, h}};
        static const int tris[6] = {0, 1, 2, 0, 2, 3};
        for (int i = 0; i < 6; ++i)
        {
            int c = tris[i];
            out.push_back(corners[c].x);
            out.push_back(corners[c].y);
            out.push_back(corners[c].z);
            out.push_back(normal.x);
            out.push_back(normal.y);
            out.push_back(normal.z);
            out.push_back(uvs[c][0]);
            out.push_back(uvs[c][1]);
        }
    }
}

void GreedyMeshChunk(const uint8_t* paddedSolid, const Vec3& chunkOrigin,
                     std::vector<float>& outVerts)
{
    const int N = CHUNK_SIZE;
    auto at = [paddedSolid](int x, int y, int z)
    {
        return paddedSolid[PaddedIndex(x, y, z)] != 0;
    };

    uint8_t mask[CHUNK_SIZE * CHUNK_SIZE];

    for (int axis = 0; axis < 3; ++axis)
    {
        const int uAxis = (axis + 1) % 3;
        const int vAxis = (axis + 2) % 3;

        for (int sign = -1; sign <= 1; sign += 2)
        {
            for (int slice = 0; slice < N; ++slice)
            {
                // Visibility mask for this slice: solid cell whose neighbor
                // along the face normal is not solid
                for (int b = 0; b < N; ++b)
                    for (int a = 0; a < N; ++a)
                    {
                        int c[3];
                        c[axis] = slice; c[uAxis] = a; c[vAxis] = b;
                        int n[3] = {c[0], c[1], c[2]};
                        n[axis] += sign;
                        mask[b * N + a] =
                            at(c[0], c[1], c[2]) && !at(n[0], n[1], n[2]);
                    }

                // Greedy rectangle merge
                for (int b = 0; b < N; ++b)
                    for (int a = 0; a < N;)
                    {
                        if (mask[b * N + a] == 0)
                        {
                            ++a;
                            continue;
                        }
                        int w = 1;
                        while (a + w < N && mask[b * N + a + w] != 0)
                            ++w;
                        int h = 1;
                        for (bool grow = true; grow && b + h < N;)
                        {
                            for (int k = 0; k < w; ++k)
                                if (mask[(b + h) * N + a + k] == 0)
                                {
                                    grow = false;
                                    break;
                                }
                            if (grow)
                                ++h;
                        }

                        float p[3];
                        p[axis] = (slice + (sign > 0 ? 1 : 0)) * VOXEL_SIZE;
                        p[uAxis] = a * VOXEL_SIZE;
                        p[vAxis] = b * VOXEL_SIZE;
                        float uv3[3] = {0.0f, 0.0f, 0.0f};
                        uv3[uAxis] = static_cast<float>(w) * VOXEL_SIZE;
                        float vv3[3] = {0.0f, 0.0f, 0.0f};
                        vv3[vAxis] = static_cast<float>(h) * VOXEL_SIZE;
                        float nrm[3] = {0.0f, 0.0f, 0.0f};
                        nrm[axis] = static_cast<float>(sign);

                        EmitQuad(outVerts,
                                 chunkOrigin + Vec3{p[0], p[1], p[2]},
                                 Vec3{uv3[0], uv3[1], uv3[2]},
                                 Vec3{vv3[0], vv3[1], vv3[2]},
                                 Vec3{nrm[0], nrm[1], nrm[2]},
                                 static_cast<float>(w), static_cast<float>(h));

                        for (int bb = 0; bb < h; ++bb)
                            for (int aa = 0; aa < w; ++aa)
                                mask[(b + bb) * N + a + aa] = 0;
                        a += w;
                    }
            }
        }
    }
}
