#pragma once
#include <cmath>

struct Vec3
{
    float x = 0.0f, y = 0.0f, z = 0.0f;

    Vec3() = default;
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3& b) const { return {x + b.x, y + b.y, z + b.z}; }
    Vec3 operator-(const Vec3& b) const { return {x - b.x, y - b.y, z - b.z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3& operator+=(const Vec3& b) { x += b.x; y += b.y; z += b.z; return *this; }
    Vec3& operator-=(const Vec3& b) { x -= b.x; y -= b.y; z -= b.z; return *this; }
    Vec3& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }
};

inline float Dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 Cross(const Vec3& a, const Vec3& b)
{
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline float Length(const Vec3& v) { return std::sqrt(Dot(v, v)); }
inline Vec3 Normalize(const Vec3& v)
{
    float len = Length(v);
    return len > 1e-8f ? v * (1.0f / len) : Vec3{0.0f, 0.0f, 0.0f};
}
inline Vec3 Lerp(const Vec3& a, const Vec3& b, float t) { return a + (b - a) * t; }
inline float Clamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Column-major 4x4 matrix, m[col * 4 + row]
struct Mat4
{
    float m[16] = {};

    static Mat4 Identity()
    {
        Mat4 r;
        r.m[0] = r.m[5] = r.m[10] = r.m[15] = 1.0f;
        return r;
    }

    static Mat4 Perspective(float fovYRad, float aspect, float zNear, float zFar)
    {
        Mat4 r;
        float f = 1.0f / std::tan(fovYRad * 0.5f);
        r.m[0] = f / aspect;
        r.m[5] = f;
        r.m[10] = (zFar + zNear) / (zNear - zFar);
        r.m[11] = -1.0f;
        r.m[14] = (2.0f * zFar * zNear) / (zNear - zFar);
        return r;
    }

    static Mat4 LookAt(const Vec3& eye, const Vec3& center, const Vec3& up)
    {
        Vec3 f = Normalize(center - eye);
        Vec3 s = Normalize(Cross(f, up));
        Vec3 u = Cross(s, f);
        Mat4 r = Identity();
        r.m[0] = s.x;  r.m[4] = s.y;  r.m[8]  = s.z;
        r.m[1] = u.x;  r.m[5] = u.y;  r.m[9]  = u.z;
        r.m[2] = -f.x; r.m[6] = -f.y; r.m[10] = -f.z;
        r.m[12] = -Dot(s, eye);
        r.m[13] = -Dot(u, eye);
        r.m[14] = Dot(f, eye);
        return r;
    }

    static Mat4 Ortho(float left, float right, float bottom, float top, float zNear, float zFar)
    {
        Mat4 r = Identity();
        r.m[0] = 2.0f / (right - left);
        r.m[5] = 2.0f / (top - bottom);
        r.m[10] = -2.0f / (zFar - zNear);
        r.m[12] = -(right + left) / (right - left);
        r.m[13] = -(top + bottom) / (top - bottom);
        r.m[14] = -(zFar + zNear) / (zFar - zNear);
        return r;
    }

    static Mat4 Translation(const Vec3& t)
    {
        Mat4 r = Identity();
        r.m[12] = t.x; r.m[13] = t.y; r.m[14] = t.z;
        return r;
    }

    Mat4 operator*(const Mat4& b) const
    {
        Mat4 r;
        for (int c = 0; c < 4; ++c)
            for (int row = 0; row < 4; ++row)
            {
                float sum = 0.0f;
                for (int k = 0; k < 4; ++k)
                    sum += m[k * 4 + row] * b.m[c * 4 + k];
                r.m[c * 4 + row] = sum;
            }
        return r;
    }
};

struct Quat
{
    float w = 1.0f, x = 0.0f, y = 0.0f, z = 0.0f;

    static Quat FromAxisAngle(const Vec3& axis, float angleRad)
    {
        float h = angleRad * 0.5f;
        float s = std::sin(h);
        return {std::cos(h), axis.x * s, axis.y * s, axis.z * s};
    }

    // Build quaternion from an orthonormal basis (columns: right, up, forward)
    static Quat FromBasis(const Vec3& r, const Vec3& u, const Vec3& f)
    {
        float m00 = r.x, m01 = u.x, m02 = f.x;
        float m10 = r.y, m11 = u.y, m12 = f.y;
        float m20 = r.z, m21 = u.z, m22 = f.z;
        float trace = m00 + m11 + m22;
        Quat q;
        if (trace > 0.0f)
        {
            float s = std::sqrt(trace + 1.0f) * 2.0f;
            q.w = 0.25f * s;
            q.x = (m21 - m12) / s;
            q.y = (m02 - m20) / s;
            q.z = (m10 - m01) / s;
        }
        else if (m00 > m11 && m00 > m22)
        {
            float s = std::sqrt(1.0f + m00 - m11 - m22) * 2.0f;
            q.w = (m21 - m12) / s;
            q.x = 0.25f * s;
            q.y = (m01 + m10) / s;
            q.z = (m02 + m20) / s;
        }
        else if (m11 > m22)
        {
            float s = std::sqrt(1.0f + m11 - m00 - m22) * 2.0f;
            q.w = (m02 - m20) / s;
            q.x = (m01 + m10) / s;
            q.y = 0.25f * s;
            q.z = (m12 + m21) / s;
        }
        else
        {
            float s = std::sqrt(1.0f + m22 - m00 - m11) * 2.0f;
            q.w = (m10 - m01) / s;
            q.x = (m02 + m20) / s;
            q.y = (m12 + m21) / s;
            q.z = 0.25f * s;
        }
        return q;
    }

    Quat Normalized() const
    {
        float n = std::sqrt(w * w + x * x + y * y + z * z);
        if (n < 1e-8f) return {};
        float inv = 1.0f / n;
        return {w * inv, x * inv, y * inv, z * inv};
    }

    Vec3 Rotate(const Vec3& v) const
    {
        Vec3 qv{x, y, z};
        Vec3 t = Cross(qv, v) * 2.0f;
        return v + t * w + Cross(qv, t);
    }

    static Quat Nlerp(const Quat& a, const Quat& b, float t)
    {
        float d = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
        float sign = d < 0.0f ? -1.0f : 1.0f;
        Quat r{
            a.w + (b.w * sign - a.w) * t,
            a.x + (b.x * sign - a.x) * t,
            a.y + (b.y * sign - a.y) * t,
            a.z + (b.z * sign - a.z) * t};
        return r.Normalized();
    }

    Mat4 ToMat4() const
    {
        Mat4 r = Mat4::Identity();
        float xx = x * x, yy = y * y, zz = z * z;
        float xy = x * y, xz = x * z, yz = y * z;
        float wx = w * x, wy = w * y, wz = w * z;
        r.m[0] = 1.0f - 2.0f * (yy + zz);
        r.m[1] = 2.0f * (xy + wz);
        r.m[2] = 2.0f * (xz - wy);
        r.m[4] = 2.0f * (xy - wz);
        r.m[5] = 1.0f - 2.0f * (xx + zz);
        r.m[6] = 2.0f * (yz + wx);
        r.m[8] = 2.0f * (xz + wy);
        r.m[9] = 2.0f * (yz - wx);
        r.m[10] = 1.0f - 2.0f * (xx + yy);
        return r;
    }
};

// View frustum extracted from a combined view-projection matrix
// (Gribb-Hartmann method). Used for chunk AABB culling.
struct Frustum
{
    struct Plane { float a, b, c, d; };
    Plane planes[6];

    static Frustum FromViewProj(const Mat4& m)
    {
        auto row = [&m](int i, float* out)
        {
            out[0] = m.m[i];
            out[1] = m.m[4 + i];
            out[2] = m.m[8 + i];
            out[3] = m.m[12 + i];
        };
        float r0[4], r1[4], r2[4], r3[4];
        row(0, r0); row(1, r1); row(2, r2); row(3, r3);

        Frustum f;
        auto set = [](Plane& p, const float* a, const float* b, float sign)
        {
            p.a = a[0] + sign * b[0];
            p.b = a[1] + sign * b[1];
            p.c = a[2] + sign * b[2];
            p.d = a[3] + sign * b[3];
        };
        set(f.planes[0], r3, r0, 1.0f);  // left
        set(f.planes[1], r3, r0, -1.0f); // right
        set(f.planes[2], r3, r1, 1.0f);  // bottom
        set(f.planes[3], r3, r1, -1.0f); // top
        set(f.planes[4], r3, r2, 1.0f);  // near
        set(f.planes[5], r3, r2, -1.0f); // far
        return f;
    }

    bool IntersectsAabb(const Vec3& mn, const Vec3& mx) const
    {
        for (const Plane& p : planes)
        {
            // Positive vertex: the AABB corner farthest along the plane normal
            float px = p.a > 0.0f ? mx.x : mn.x;
            float py = p.b > 0.0f ? mx.y : mn.y;
            float pz = p.c > 0.0f ? mx.z : mn.z;
            if (p.a * px + p.b * py + p.c * pz + p.d < 0.0f)
                return false;
        }
        return true;
    }
};
