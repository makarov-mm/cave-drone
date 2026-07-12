#pragma once
#include <cstdint>
#include <cmath>

// Deterministic 3D value noise with fractal Brownian motion.
class Noise
{
public:
    explicit Noise(uint32_t seed) : m_seed(seed) {}

    // Returns fBm noise in roughly [0, 1]
    float Fbm(float x, float y, float z, int octaves) const
    {
        float sum = 0.0f;
        float amp = 0.5f;
        float freq = 1.0f;
        for (int i = 0; i < octaves; ++i)
        {
            sum += amp * Value(x * freq, y * freq, z * freq);
            freq *= 2.03f;
            amp *= 0.5f;
        }
        return sum;
    }

private:
    uint32_t m_seed;

    static float Smooth(float t) { return t * t * (3.0f - 2.0f * t); }

    float Hash(int x, int y, int z) const
    {
        uint32_t h = m_seed;
        h ^= static_cast<uint32_t>(x) * 0x8da6b343u;
        h ^= static_cast<uint32_t>(y) * 0xd8163841u;
        h ^= static_cast<uint32_t>(z) * 0xcb1ab31fu;
        h = (h ^ (h >> 13)) * 0x5bd1e995u;
        h ^= h >> 15;
        return static_cast<float>(h & 0xFFFFFFu) / static_cast<float>(0xFFFFFFu);
    }

    float Value(float x, float y, float z) const
    {
        int ix = static_cast<int>(std::floor(x));
        int iy = static_cast<int>(std::floor(y));
        int iz = static_cast<int>(std::floor(z));
        float fx = Smooth(x - static_cast<float>(ix));
        float fy = Smooth(y - static_cast<float>(iy));
        float fz = Smooth(z - static_cast<float>(iz));

        float c000 = Hash(ix, iy, iz),         c100 = Hash(ix + 1, iy, iz);
        float c010 = Hash(ix, iy + 1, iz),     c110 = Hash(ix + 1, iy + 1, iz);
        float c001 = Hash(ix, iy, iz + 1),     c101 = Hash(ix + 1, iy, iz + 1);
        float c011 = Hash(ix, iy + 1, iz + 1), c111 = Hash(ix + 1, iy + 1, iz + 1);

        float x00 = c000 + (c100 - c000) * fx;
        float x10 = c010 + (c110 - c010) * fx;
        float x01 = c001 + (c101 - c001) * fx;
        float x11 = c011 + (c111 - c011) * fx;
        float y0 = x00 + (x10 - x00) * fy;
        float y1 = x01 + (x11 - x01) * fy;
        return y0 + (y1 - y0) * fz;
    }
};
