// ============================================================================
//  gfx.h  -  Rendering layer
//    * GLSL shaders: Gouraud (per-vertex lighting) and Phong (per-fragment)
//    * MeshBuilder: procedural 3D modelling (boxes, cylinders, spheres, lathes)
//    * Mesh / StaticBatch (spatially chunked + frustum culled)
//    * Renderer: lights, materials, shading-mode selection
//    * Hud: 2D overlay (bitmap-font text, rectangles, lines)
// ============================================================================
#pragma once
#define GLFW_INCLUDE_NONE
#include <glad/gl.h>
#include <GLFW/glfw3.h>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include <map>
#include <unordered_map>
#include <functional>
#include "math3d.h"
#include "font_data.h"

// ---------------------------------------------------------------------------
//  Vertex format and procedural mesh builder
// ---------------------------------------------------------------------------
struct Vertex { float p[3], n[3], c[3], m[4]; };   // 13 floats

struct Mat {            // surface material baked into vertices
    vec3 c{0.5f, 0.5f, 0.5f};
    float spec = 0.1f, shin = 16.f, emis = 0.f, tex = 0.f;
};
inline Mat MT(vec3 c, float spec = 0.1f, float shin = 16.f, float emis = 0.f, float tex = 0.f) {
    Mat m; m.c = c; m.spec = spec; m.shin = shin; m.emis = emis; m.tex = tex; return m;
}

static inline vec3 xformNormal(const mat4& M, const vec3& n) {
    vec3 a(M.m[0], M.m[1], M.m[2]), b(M.m[4], M.m[5], M.m[6]), c(M.m[8], M.m[9], M.m[10]);
    return normalize(cross(b, c) * n.x + cross(c, a) * n.y + cross(a, b) * n.z);
}

struct ProfilePt { float r, z; vec3 c; bool sharp; };
struct LoftSec { float cy, hh, hw; };

struct MeshBuilder {
    std::vector<Vertex> v;
    size_t size() const { return v.size(); }

    void vert(const vec3& p, const vec3& n, const Mat& m) {
        Vertex x;
        x.p[0] = p.x; x.p[1] = p.y; x.p[2] = p.z;
        x.n[0] = n.x; x.n[1] = n.y; x.n[2] = n.z;
        x.c[0] = m.c.x; x.c[1] = m.c.y; x.c[2] = m.c.z;
        x.m[0] = m.spec; x.m[1] = m.shin / 128.f; x.m[2] = m.emis; x.m[3] = m.tex;
        v.push_back(x);
    }
    void tri(const vec3& a, const vec3& b, const vec3& c, const vec3& n, const Mat& m) {
        vert(a, n, m); vert(b, n, m); vert(c, n, m);
    }
    void quadN(const vec3& a, const vec3& b, const vec3& c, const vec3& d, const vec3& n, const Mat& m) {
        tri(a, b, c, n, m); tri(a, c, d, n, m);
    }
    void quadSmooth(const vec3& a, const vec3& b, const vec3& c, const vec3& d,
                    const vec3& na, const vec3& nb, const vec3& nc, const vec3& nd, const Mat& m) {
        vert(a, na, m); vert(b, nb, m); vert(c, nc, m);
        vert(a, na, m); vert(c, nc, m); vert(d, nd, m);
    }
    // flat quad whose normal is flipped, if necessary, to face away from 'inside'
    void quadOut(const vec3& a, const vec3& b, const vec3& c, const vec3& d, const vec3& inside, const Mat& m) {
        vec3 n = normalize(cross(b - a, c - a));
        vec3 ctr = (a + b + c + d) * 0.25f;
        if (dot(n, ctr - inside) < 0) n = -n;
        quadN(a, b, c, d, n, m);
    }
    // quad lying on a surface with an explicit upward normal
    void quadUp(const vec3& a, const vec3& b, const vec3& c, const vec3& d, const Mat& m) {
        quadN(a, b, c, d, vec3(0, 1, 0), m);
    }

    // ---- primitives (all take a transformation matrix) --------------------
    void box(const mat4& T, const vec3& h, const Mat& m, const Mat* top = nullptr) {
        static const int ax[3][3] = {{0, 1, 2}, {1, 2, 0}, {2, 0, 1}};
        for (int i = 0; i < 3; i++)
            for (int s = -1; s <= 1; s += 2) {
                float hv[3] = {h.x, h.y, h.z};
                vec3 n; float* np = &n.x; np[ax[i][0]] = (float)s;
                vec3 u, w; (&u.x)[ax[i][1]] = hv[ax[i][1]]; (&w.x)[ax[i][2]] = hv[ax[i][2]];
                vec3 c = n * hv[ax[i][0]];
                vec3 p0 = c - u - w, p1 = c + u - w, p2 = c + u + w, p3 = c - u + w;
                vec3 nt = xformNormal(T, n);
                const Mat& mm = (top && i == 1 && s == 1) ? *top : m;
                vec3 a = T.point(p0), b = T.point(p1), cc = T.point(p2), d = T.point(p3);
                // orient so (b-a)x(c-a) agrees with the normal (not required, but tidy)
                tri(a, b, cc, nt, mm); tri(a, cc, d, nt, mm);
            }
    }
    void boxAt(const vec3& c, const vec3& h, const Mat& m, const Mat* top = nullptr) {
        box(translate(c), h, m, top);
    }
    // Truncated cone / cylinder along local Y from -h to +h (r0 bottom, r1 top)
    void cyl(const mat4& T, float r0, float r1, float h, int seg, const Mat& m, bool capTop = true, bool capBot = true) {
        float dr = r1 - r0;
        for (int i = 0; i < seg; i++) {
            float a0 = 2 * PI_F * i / seg, a1 = 2 * PI_F * (i + 1) / seg;
            float c0 = std::cos(a0), s0 = std::sin(a0), c1 = std::cos(a1), s1 = std::sin(a1);
            vec3 n0 = normalize(vec3(2 * h * c0, -dr, 2 * h * s0)), n1 = normalize(vec3(2 * h * c1, -dr, 2 * h * s1));
            vec3 b0 = T.point({r0 * c0, -h, r0 * s0}), b1 = T.point({r0 * c1, -h, r0 * s1});
            vec3 t0 = T.point({r1 * c0, h, r1 * s0}), t1 = T.point({r1 * c1, h, r1 * s1});
            quadSmooth(b0, b1, t1, t0, xformNormal(T, n0), xformNormal(T, n1), xformNormal(T, n1), xformNormal(T, n0), m);
            if (capTop && r1 > 0) tri(T.point({0, h, 0}), t1, t0, xformNormal(T, {0, 1, 0}), m);
            if (capBot && r0 > 0) tri(T.point({0, -h, 0}), b0, b1, xformNormal(T, {0, -1, 0}), m);
        }
    }
    void sphere(const mat4& T, float r, int seg, int stacks, const Mat& m) {
        for (int j = 0; j < stacks; j++) {
            float p0 = PI_F * j / stacks, p1 = PI_F * (j + 1) / stacks;
            for (int i = 0; i < seg; i++) {
                float a0 = 2 * PI_F * i / seg, a1 = 2 * PI_F * (i + 1) / seg;
                auto P = [&](float p, float a) { return vec3(std::sin(p) * std::cos(a), std::cos(p), std::sin(p) * std::sin(a)); };
                vec3 q00 = P(p0, a0), q01 = P(p0, a1), q10 = P(p1, a0), q11 = P(p1, a1);
                quadSmooth(T.point(q00 * r), T.point(q10 * r), T.point(q11 * r), T.point(q01 * r),
                           xformNormal(T, q00), xformNormal(T, q10), xformNormal(T, q11), xformNormal(T, q01), m);
            }
        }
    }
    // Cylinder / cone between two world points (used for suspension arms, halo, limbs ...)
    void tube(const vec3& a, const vec3& b, float r0, float r1, int seg, const Mat& m, bool caps = true) {
        vec3 d = b - a; float len = length(d);
        if (len < 1e-5f) return;
        mat4 T = frameY((a + b) * 0.5f, d);
        cyl(T, r0, r1, len * 0.5f, seg, m, caps, caps);
    }
    // Rectangular loft along X between two cross sections (nose cones, sidepods ...)
    void loft(const mat4& T, float x0, float x1, LoftSec s0, LoftSec s1, const Mat& m, const Mat* top = nullptr) {
        auto P = [&](float x, const LoftSec& s, float ys, float zs) { return T.point({x, s.cy + ys * s.hh, zs * s.hw}); };
        vec3 in = T.point({(x0 + x1) * 0.5f, (s0.cy + s1.cy) * 0.5f, 0});
        // top, bottom, left(-z), right(+z), back(x0), front(x1)
        quadOut(P(x0, s0, 1, -1), P(x1, s1, 1, -1), P(x1, s1, 1, 1), P(x0, s0, 1, 1), in, top ? *top : m);
        quadOut(P(x0, s0, -1, -1), P(x1, s1, -1, -1), P(x1, s1, -1, 1), P(x0, s0, -1, 1), in, m);
        quadOut(P(x0, s0, -1, -1), P(x1, s1, -1, -1), P(x1, s1, 1, -1), P(x0, s0, 1, -1), in, m);
        quadOut(P(x0, s0, -1, 1), P(x1, s1, -1, 1), P(x1, s1, 1, 1), P(x0, s0, 1, 1), in, m);
        quadOut(P(x0, s0, -1, -1), P(x0, s0, 1, -1), P(x0, s0, 1, 1), P(x0, s0, -1, 1), in, m);
        quadOut(P(x1, s1, -1, -1), P(x1, s1, 1, -1), P(x1, s1, 1, 1), P(x1, s1, -1, 1), in, m);
    }
    // Surface of revolution about local Z (wheels, tyres, rims).  Profile is (radius, z).
    void lathe(const mat4& T, const std::vector<ProfilePt>& pr, int seg, float spec, float shin, float tex = 0.f) {
        int K = (int)pr.size() - 1;
        std::vector<vec3> sn(K);   // segment normals in (r,z) plane stored as (nr,nz,0)
        for (int k = 0; k < K; k++) {
            float dr = pr[k + 1].r - pr[k].r, dz = pr[k + 1].z - pr[k].z;
            float l = std::sqrt(dr * dr + dz * dz); if (l < 1e-9f) l = 1;
            sn[k] = vec3(dz / l, -dr / l, 0);
        }
        auto ptN = [&](int k, int segIdx) {   // normal at profile point k as seen from segment segIdx
            if (pr[k].sharp) return sn[segIdx];
            vec3 a = (k > 0) ? sn[k - 1] : sn[0], b = (k < K) ? sn[k] : sn[K - 1];
            vec3 s = a + b; float l = std::sqrt(s.x * s.x + s.y * s.y); if (l < 1e-9f) return sn[segIdx];
            return vec3(s.x / l, s.y / l, 0);
        };
        for (int k = 0; k < K; k++) {
            vec3 na = ptN(k, k), nb = ptN(k + 1, k);
            for (int i = 0; i < seg; i++) {
                float a0 = 2 * PI_F * i / seg, a1 = 2 * PI_F * (i + 1) / seg;
                float c0 = std::cos(a0), s0 = std::sin(a0), c1 = std::cos(a1), s1 = std::sin(a1);
                auto V = [&](float r, float z, float c, float s) { return T.point({r * c, r * s, z}); };
                auto N = [&](const vec3& n, float c, float s) { return xformNormal(T, {n.x * c, n.x * s, n.y}); };
                Mat ma = MT(pr[k].c, spec, shin, 0, tex), mb = MT(pr[k + 1].c, spec, shin, 0, tex);
                vert(V(pr[k].r, pr[k].z, c0, s0), N(na, c0, s0), ma);
                vert(V(pr[k + 1].r, pr[k + 1].z, c0, s0), N(nb, c0, s0), mb);
                vert(V(pr[k + 1].r, pr[k + 1].z, c1, s1), N(nb, c1, s1), mb);
                vert(V(pr[k].r, pr[k].z, c0, s0), N(na, c0, s0), ma);
                vert(V(pr[k + 1].r, pr[k + 1].z, c1, s1), N(nb, c1, s1), mb);
                vert(V(pr[k].r, pr[k].z, c1, s1), N(na, c1, s1), ma);
            }
        }
    }

};
