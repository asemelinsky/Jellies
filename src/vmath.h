// Small math helpers on top of raymath: 3x3 matrices and quaternion rotation
// extraction used by shape matching.
#pragma once
#include <cmath>
#include "raylib.h"
#include "raymath.h"

inline float Dot(Vector3 a, Vector3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vector3 Cross(Vector3 a, Vector3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline float Len(Vector3 a) { return std::sqrt(Dot(a, a)); }
inline float Len2(Vector3 a) { return Dot(a, a); }
inline Vector3 Norm(Vector3 a) { float l = Len(a); return l > 1e-12f ? a * (1.0f / l) : Vector3{0, 0, 0}; }

// Column-major 3x3 matrix
struct M3 {
    Vector3 c[3];
    static M3 Zero() { return {{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}}; }
    static M3 Identity() { return {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}}; }
    Vector3 operator*(Vector3 v) const { return c[0] * v.x + c[1] * v.y + c[2] * v.z; }
    // this += a * b^T
    void AddOuter(Vector3 a, Vector3 b) {
        c[0] += a * b.x;
        c[1] += a * b.y;
        c[2] += a * b.z;
    }
};

inline M3 Inverse(const M3& m, bool* ok = nullptr) {
    Vector3 r0 = Cross(m.c[1], m.c[2]);
    Vector3 r1 = Cross(m.c[2], m.c[0]);
    Vector3 r2 = Cross(m.c[0], m.c[1]);
    float det = Dot(m.c[0], r0);
    if (std::fabs(det) < 1e-12f) {
        if (ok) *ok = false;
        return M3::Identity();
    }
    if (ok) *ok = true;
    float inv = 1.0f / det;
    // rows of the inverse are r0, r1, r2 -> transpose into columns
    M3 res;
    res.c[0] = {r0.x * inv, r1.x * inv, r2.x * inv};
    res.c[1] = {r0.y * inv, r1.y * inv, r2.y * inv};
    res.c[2] = {r0.z * inv, r1.z * inv, r2.z * inv};
    return res;
}

struct Quat {
    float x = 0, y = 0, z = 0, w = 1;
};

inline Quat QMul(Quat a, Quat b) {
    return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

inline Quat QNormalize(Quat q) {
    float l = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (l < 1e-12f) return Quat{};
    float i = 1.0f / l;
    return {q.x * i, q.y * i, q.z * i, q.w * i};
}

inline Quat QAxisAngle(Vector3 axis, float angle) {
    float s = std::sin(angle * 0.5f);
    return {axis.x * s, axis.y * s, axis.z * s, std::cos(angle * 0.5f)};
}

inline M3 QToM3(Quat q) {
    float xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
    float xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
    float wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
    M3 m;
    m.c[0] = {1 - 2 * (yy + zz), 2 * (xy + wz), 2 * (xz - wy)};
    m.c[1] = {2 * (xy - wz), 1 - 2 * (xx + zz), 2 * (yz + wx)};
    m.c[2] = {2 * (xz + wy), 2 * (yz - wx), 1 - 2 * (xx + yy)};
    return m;
}

// Müller et al. 2016, "A Robust Method to Extract the Rotational Part of
// Deformations". Warm-started with the previous frame's rotation.
inline void ExtractRotation(const M3& A, Quat& q, int iterations) {
    for (int i = 0; i < iterations; i++) {
        M3 R = QToM3(q);
        Vector3 num = Cross(R.c[0], A.c[0]) + Cross(R.c[1], A.c[1]) + Cross(R.c[2], A.c[2]);
        float den = std::fabs(Dot(R.c[0], A.c[0]) + Dot(R.c[1], A.c[1]) + Dot(R.c[2], A.c[2])) + 1e-9f;
        Vector3 omega = num * (1.0f / den);
        float w = Len(omega);
        if (w < 1e-7f) break;
        q = QNormalize(QMul(QAxisAngle(omega * (1.0f / w), w), q));
    }
}

inline Quat QRandom() {
    // Uniform random rotation (Shoemake)
    float u1 = GetRandomValue(0, 100000) / 100000.0f;
    float u2 = GetRandomValue(0, 100000) / 100000.0f * 2.0f * PI;
    float u3 = GetRandomValue(0, 100000) / 100000.0f * 2.0f * PI;
    float a = std::sqrt(1 - u1), b = std::sqrt(u1);
    return {a * std::sin(u2), a * std::cos(u2), b * std::sin(u3), b * std::cos(u3)};
}

inline float RandF(float lo, float hi) { return lo + (hi - lo) * (GetRandomValue(0, 1000000) / 1000000.0f); }
