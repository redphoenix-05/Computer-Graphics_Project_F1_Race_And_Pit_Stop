// ============================================================================
//  math3d.h  -  Minimal linear-algebra library for the F1 simulation
//  Column-major 4x4 matrices (OpenGL convention), right-handed coordinates:
//      +X forward (car), +Y up, +Z right (car).
//  All geometric transformations used by the project (translation, rotation,
//  scaling, projection, view) are expressed as 4x4 transformation matrices.
// ============================================================================
#pragma once
#include <cmath>
#include <cstring>
#include <algorithm>

#ifndef PI_F
#define PI_F 3.14159265358979323846f
#endif

struct vec2 { float x = 0, y = 0; };

struct vec3 {
    float x = 0, y = 0, z = 0;
    vec3() {}
    vec3(float a, float b, float c) : x(a), y(b), z(c) {}
    explicit vec3(float a) : x(a), y(a), z(a) {}
    vec3 operator+(const vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    vec3 operator-(const vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    vec3 operator/(float s) const { return {x / s, y / s, z / s}; }
    vec3 operator*(const vec3& o) const { return {x * o.x, y * o.y, z * o.z}; }
    vec3 operator-() const { return {-x, -y, -z}; }
    vec3& operator+=(const vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    vec3& operator-=(const vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    vec3& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }
};
inline vec3 operator*(float s, const vec3& v) { return v * s; }
inline float dot(const vec3& a, const vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline vec3 cross(const vec3& a, const vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline float length(const vec3& v) { return std::sqrt(dot(v, v)); }
inline vec3 normalize(const vec3& v) {
    float l = length(v);
    return l > 1e-12f ? v / l : vec3(0, 1, 0);
}
inline vec3 mix(const vec3& a, const vec3& b, float t) { return a + (b - a) * t; }
inline float mixf(float a, float b, float t) { return a + (b - a) * t; }
inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline float smoothstepf(float a, float b, float x) {
    float t = clampf((x - a) / (b - a), 0.f, 1.f);
    return t * t * (3.f - 2.f * t);
}
inline float sgn(float v) { return v < 0 ? -1.f : 1.f; }
inline float radians(float d) { return d * PI_F / 180.f; }
inline float wrapAngle(float a) {
    while (a > PI_F) a -= 2 * PI_F;
    while (a < -PI_F) a += 2 * PI_F;
    return a;
}

struct mat4 {
    float m[16];  // column-major: m[col*4 + row]
    mat4() { std::memset(m, 0, sizeof(m)); m[0] = m[5] = m[10] = m[15] = 1.f; }
    float& at(int r, int c) { return m[c * 4 + r]; }
    float at(int r, int c) const { return m[c * 4 + r]; }

    mat4 operator*(const mat4& b) const {
        mat4 r;
        for (int c = 0; c < 4; c++)
            for (int rr = 0; rr < 4; rr++) {
                float s = 0;
                for (int k = 0; k < 4; k++) s += m[k * 4 + rr] * b.m[c * 4 + k];
                r.m[c * 4 + rr] = s;
            }
        return r;
    }
    vec3 point(const vec3& p) const {  // transform a position (w = 1)
        return {m[0] * p.x + m[4] * p.y + m[8] * p.z + m[12],
                m[1] * p.x + m[5] * p.y + m[9] * p.z + m[13],
                m[2] * p.x + m[6] * p.y + m[10] * p.z + m[14]};
    }
    vec3 dir(const vec3& p) const {  // transform a direction (w = 0)
        return {m[0] * p.x + m[4] * p.y + m[8] * p.z,
                m[1] * p.x + m[5] * p.y + m[9] * p.z,
                m[2] * p.x + m[6] * p.y + m[10] * p.z};
    }
    vec3 translation() const { return {m[12], m[13], m[14]}; }
};

// ---- Geometric transformation matrices ------------------------------------
inline mat4 translate(const vec3& t) {
    mat4 r; r.m[12] = t.x; r.m[13] = t.y; r.m[14] = t.z; return r;
}
inline mat4 translate(float x, float y, float z) { return translate(vec3(x, y, z)); }
inline mat4 scale(const vec3& s) {
    mat4 r; r.m[0] = s.x; r.m[5] = s.y; r.m[10] = s.z; return r;
}
inline mat4 scale(float x, float y, float z) { return scale(vec3(x, y, z)); }
inline mat4 scale(float s) { return scale(vec3(s, s, s)); }
inline mat4 rotX(float a) {
    mat4 r; float c = std::cos(a), s = std::sin(a);
    r.at(1, 1) = c; r.at(1, 2) = -s; r.at(2, 1) = s; r.at(2, 2) = c; return r;
}
inline mat4 rotY(float a) {
    mat4 r; float c = std::cos(a), s = std::sin(a);
    r.at(0, 0) = c; r.at(0, 2) = s; r.at(2, 0) = -s; r.at(2, 2) = c; return r;
}
inline mat4 rotZ(float a) {
    mat4 r; float c = std::cos(a), s = std::sin(a);
    r.at(0, 0) = c; r.at(0, 1) = -s; r.at(1, 0) = s; r.at(1, 1) = c; return r;
}
// Matrix whose columns are the given basis vectors + origin
inline mat4 basis(const vec3& x, const vec3& y, const vec3& z, const vec3& o) {
    mat4 r;
    r.m[0] = x.x; r.m[1] = x.y; r.m[2] = x.z;
    r.m[4] = y.x; r.m[5] = y.y; r.m[6] = y.z;
    r.m[8] = z.x; r.m[9] = z.y; r.m[10] = z.z;
    r.m[12] = o.x; r.m[13] = o.y; r.m[14] = o.z;
    return r;
}
// Frame whose local +Y axis points along 'dir', origin at 'o'
inline mat4 frameY(const vec3& o, const vec3& dir) {
    vec3 y = normalize(dir);
    vec3 helper = std::fabs(y.y) < 0.95f ? vec3(0, 1, 0) : vec3(1, 0, 0);
    vec3 x = normalize(cross(helper, y));
    vec3 z = cross(x, y);
    // x,y,z must be right handed: x × y = z  ->  recompute
    z = cross(x, y);
    return basis(x, y, z, o);
}
inline mat4 perspective(float fovyRad, float aspect, float zn, float zf) {
    mat4 r; std::memset(r.m, 0, sizeof(r.m));
    float f = 1.f / std::tan(fovyRad * 0.5f);
    r.m[0] = f / aspect; r.m[5] = f;
    r.m[10] = (zf + zn) / (zn - zf); r.m[11] = -1.f;
    r.m[14] = 2.f * zf * zn / (zn - zf);
    return r;
}
inline mat4 lookAt(const vec3& eye, const vec3& at, const vec3& up) {
    vec3 f = normalize(at - eye);
    vec3 s = normalize(cross(f, up));
    vec3 u = cross(s, f);
    mat4 r;
    r.at(0, 0) = s.x; r.at(0, 1) = s.y; r.at(0, 2) = s.z;
    r.at(1, 0) = u.x; r.at(1, 1) = u.y; r.at(1, 2) = u.z;
    r.at(2, 0) = -f.x; r.at(2, 1) = -f.y; r.at(2, 2) = -f.z;
    r.at(0, 3) = -dot(s, eye); r.at(1, 3) = -dot(u, eye); r.at(2, 3) = dot(f, eye);
    return r;
}
// Matrix for transforming normals (cofactor of the upper-left 3x3; row-major
// 3x3 returned as column-major float[9] ready for glUniformMatrix3fv)
inline void normalMatrix(const mat4& M, float out[9]) {
    vec3 a(M.m[0], M.m[1], M.m[2]), b(M.m[4], M.m[5], M.m[6]), c(M.m[8], M.m[9], M.m[10]);
    vec3 n0 = cross(b, c), n1 = cross(c, a), n2 = cross(a, b);
    out[0] = n0.x; out[1] = n0.y; out[2] = n0.z;
    out[3] = n1.x; out[4] = n1.y; out[5] = n1.z;
    out[6] = n2.x; out[7] = n2.y; out[8] = n2.z;
}
// Inverse of a general 4x4 matrix (used for the sky shader)
inline mat4 inverse(const mat4& A) {
    const float* m = A.m; float inv[16];
    inv[0] = m[5]*m[10]*m[15]-m[5]*m[11]*m[14]-m[9]*m[6]*m[15]+m[9]*m[7]*m[14]+m[13]*m[6]*m[11]-m[13]*m[7]*m[10];
    inv[4] = -m[4]*m[10]*m[15]+m[4]*m[11]*m[14]+m[8]*m[6]*m[15]-m[8]*m[7]*m[14]-m[12]*m[6]*m[11]+m[12]*m[7]*m[10];
    inv[8] = m[4]*m[9]*m[15]-m[4]*m[11]*m[13]-m[8]*m[5]*m[15]+m[8]*m[7]*m[13]+m[12]*m[5]*m[11]-m[12]*m[7]*m[9];
    inv[12] = -m[4]*m[9]*m[14]+m[4]*m[10]*m[13]+m[8]*m[5]*m[14]-m[8]*m[6]*m[13]-m[12]*m[5]*m[10]+m[12]*m[6]*m[9];
    inv[1] = -m[1]*m[10]*m[15]+m[1]*m[11]*m[14]+m[9]*m[2]*m[15]-m[9]*m[3]*m[14]-m[13]*m[2]*m[11]+m[13]*m[3]*m[10];
    inv[5] = m[0]*m[10]*m[15]-m[0]*m[11]*m[14]-m[8]*m[2]*m[15]+m[8]*m[3]*m[14]+m[12]*m[2]*m[11]-m[12]*m[3]*m[10];
    inv[9] = -m[0]*m[9]*m[15]+m[0]*m[11]*m[13]+m[8]*m[1]*m[15]-m[8]*m[3]*m[13]-m[12]*m[1]*m[11]+m[12]*m[3]*m[9];
    inv[13] = m[0]*m[9]*m[14]-m[0]*m[10]*m[13]-m[8]*m[1]*m[14]+m[8]*m[2]*m[13]+m[12]*m[1]*m[10]-m[12]*m[2]*m[9];
    inv[2] = m[1]*m[6]*m[15]-m[1]*m[7]*m[14]-m[5]*m[2]*m[15]+m[5]*m[3]*m[14]+m[13]*m[2]*m[7]-m[13]*m[3]*m[6];
    inv[6] = -m[0]*m[6]*m[15]+m[0]*m[7]*m[14]+m[4]*m[2]*m[15]-m[4]*m[3]*m[14]-m[12]*m[2]*m[7]+m[12]*m[3]*m[6];
    inv[10] = m[0]*m[5]*m[15]-m[0]*m[7]*m[13]-m[4]*m[1]*m[15]+m[4]*m[3]*m[13]+m[12]*m[1]*m[7]-m[12]*m[3]*m[5];
    inv[14] = -m[0]*m[5]*m[14]+m[0]*m[6]*m[13]+m[4]*m[1]*m[14]-m[4]*m[2]*m[13]-m[12]*m[1]*m[6]+m[12]*m[2]*m[5];
    inv[3] = -m[1]*m[6]*m[11]+m[1]*m[7]*m[10]+m[5]*m[2]*m[11]-m[5]*m[3]*m[10]-m[9]*m[2]*m[7]+m[9]*m[3]*m[6];
    inv[7] = m[0]*m[6]*m[11]-m[0]*m[7]*m[10]-m[4]*m[2]*m[11]+m[4]*m[3]*m[10]+m[8]*m[2]*m[7]-m[8]*m[3]*m[6];
    inv[11] = -m[0]*m[5]*m[11]+m[0]*m[7]*m[9]+m[4]*m[1]*m[11]-m[4]*m[3]*m[9]-m[8]*m[1]*m[7]+m[8]*m[3]*m[5];
    inv[15] = m[0]*m[5]*m[10]-m[0]*m[6]*m[9]-m[4]*m[1]*m[10]+m[4]*m[2]*m[9]+m[8]*m[1]*m[6]-m[8]*m[2]*m[5];
    float det = m[0]*inv[0] + m[1]*inv[4] + m[2]*inv[8] + m[3]*inv[12];
    mat4 r;
    if (std::fabs(det) < 1e-20f) return r;
    det = 1.f / det;
    for (int i = 0; i < 16; i++) r.m[i] = inv[i] * det;
    return r;
}
