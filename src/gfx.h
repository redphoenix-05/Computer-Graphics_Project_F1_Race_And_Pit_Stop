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
//  Shaders
// ---------------------------------------------------------------------------
static const char* GLSL_LIGHTING = R"GLSL(
#define MAXL 16
uniform vec3 uCamPos;
uniform vec3 uSunDir;   // direction TOWARDS the sun
uniform vec3 uSunCol;
uniform vec3 uSkyAmb;
uniform vec3 uGndAmb;
uniform int  uNL;
uniform vec3 uLPos[MAXL];
uniform vec3 uLCol[MAXL];
uniform vec3 uLDir[MAXL];
uniform vec4 uLPar[MAXL];   // radius, cosInner, cosOuter, isSpot
uniform vec3 uFogCol;
uniform float uFogDen;

// Blinn-Phong illumination: hemispheric ambient + directional sun +
// up to 16 point / spot lights + emission.  Used per-vertex by the Gouraud
// program and per-fragment by the Phong program.
vec3 lightModel(vec3 P, vec3 N, vec3 col, float spec, float shin, float emis) {
    col = col * col;                       // sRGB-ish -> linear
    vec3 V = normalize(uCamPos - P);
    vec3 res = mix(uGndAmb, uSkyAmb, N.y * 0.5 + 0.5) * col;
    float ndl = max(dot(N, uSunDir), 0.0);
    res += uSunCol * col * ndl;
    if (ndl > 0.0 && spec > 0.0) {
        vec3 H = normalize(uSunDir + V);
        res += uSunCol * spec * pow(max(dot(N, H), 0.0), shin);
    }
    for (int i = 0; i < uNL; i++) {
        vec3 L = uLPos[i] - P;
        float d = length(L);
        L /= max(d, 0.001);
        float att = clamp(1.0 - d / uLPar[i].x, 0.0, 1.0);
        att *= att;
        if (uLPar[i].w > 0.5) {
            float cd = dot(-L, uLDir[i]);
            att *= smoothstep(uLPar[i].z, uLPar[i].y, cd);
        }
        float nl = max(dot(N, L), 0.0);
        res += uLCol[i] * att * col * nl;
        if (nl > 0.0 && spec > 0.0) {
            vec3 H = normalize(L + V);
            res += uLCol[i] * att * spec * pow(max(dot(N, H), 0.0), shin);
        }
    }
    res += col * emis * 1.8;
    return res;
}
vec3 finish(vec3 c, float dist) {
    float f = 1.0 - exp(-dist * uFogDen);
    c = mix(c, uFogCol * uFogCol, clamp(f, 0.0, 1.0));
    c = vec3(1.0) - exp(-c * 1.25);       // soft tone mapping
    return pow(c, vec3(1.0 / 2.2));       // gamma
}
)GLSL";

static const char* GLSL_ATTRIBS = R"GLSL(
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNrm;
layout(location=2) in vec3 aCol;
layout(location=3) in vec4 aMat;   // specular, shininess/128, emission, detail-texture amount
uniform mat4 uModel;
uniform mat3 uNrmMat;
uniform mat4 uVP;
uniform vec3 uTint;
uniform float uEmis;
)GLSL";

static const char* GLSL_DETAIL = R"GLSL(
uniform sampler2D uNoise;
float detail(vec3 w, float amt) {
    if (amt <= 0.0) return 1.0;
    float n = texture(uNoise, w.xz * 0.31).r * 0.55 + texture(uNoise, w.xz * 0.043).r * 0.45;
    return 1.0 + (n - 0.5) * 2.0 * amt;
}
)GLSL";

// ---- Gouraud: lighting evaluated in the VERTEX shader, colour interpolated
static const char* GOURAUD_VS_BODY = R"GLSL(
out vec3 vLit; out vec3 vW; out float vTex; out float vDist;
void main() {
    vec4 w = uModel * vec4(aPos, 1.0);
    vec3 N = normalize(uNrmMat * aNrm);
    vec3 col = aCol * uTint;
    vLit = lightModel(w.xyz, N, col, aMat.x, aMat.y * 128.0, aMat.z + uEmis);
    vW = w.xyz; vTex = aMat.w; vDist = length(uCamPos - w.xyz);
    gl_Position = uVP * w;
}
)GLSL";
static const char* GOURAUD_FS_BODY = R"GLSL(
in vec3 vLit; in vec3 vW; in float vTex; in float vDist;
uniform float uAlpha;
out vec4 FragColor;
void main() {
    vec3 c = vLit * detail(vW, vTex);
    FragColor = vec4(finish(c, vDist), uAlpha);
}
)GLSL";

// ---- Phong: normals interpolated, lighting evaluated per FRAGMENT
static const char* PHONG_VS_BODY = R"GLSL(
out vec3 vW; out vec3 vN; out vec3 vC; out vec4 vM;
void main() {
    vec4 w = uModel * vec4(aPos, 1.0);
    vW = w.xyz; vN = uNrmMat * aNrm; vC = aCol * uTint; vM = aMat;
    gl_Position = uVP * w;
}
)GLSL";
static const char* PHONG_FS_BODY = R"GLSL(
in vec3 vW; in vec3 vN; in vec3 vC; in vec4 vM;
uniform float uAlpha; uniform float uEmis;
out vec4 FragColor;
void main() {
    vec3 N = normalize(vN);
    vec3 col = vC * detail(vW, vM.w);
    vec3 c = lightModel(vW, N, col, vM.x, vM.y * 128.0, vM.z + uEmis);
    FragColor = vec4(finish(c, length(uCamPos - vW)), uAlpha);
}
)GLSL";

// ---------------------------------------------------------------------------
//  Shader program wrapper
// ---------------------------------------------------------------------------
struct Shader {
    GLuint id = 0;
    std::unordered_map<std::string, GLint> cache;
    static GLuint compile(GLenum type, const std::string& src, const char* name) {
        GLuint s = glCreateShader(type);
        const char* c = src.c_str();
        glShaderSource(s, 1, &c, nullptr);
        glCompileShader(s);
        GLint ok = 0; glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
        if (!ok) {
            char log[2048]; glGetShaderInfoLog(s, 2048, nullptr, log);
            fprintf(stderr, "Shader compile error (%s):\n%s\n", name, log);
            exit(1);
        }
        return s;
    }
    void build(const std::string& vs, const std::string& fs, const char* name) {
        GLuint v = compile(GL_VERTEX_SHADER, vs, name), f = compile(GL_FRAGMENT_SHADER, fs, name);
        id = glCreateProgram();
        glAttachShader(id, v); glAttachShader(id, f);
        glLinkProgram(id);
        GLint ok = 0; glGetProgramiv(id, GL_LINK_STATUS, &ok);
        if (!ok) { char log[2048]; glGetProgramInfoLog(id, 2048, nullptr, log); fprintf(stderr, "Link error (%s): %s\n", name, log); exit(1); }
        glDeleteShader(v); glDeleteShader(f);
    }
    GLint u(const char* n) {
        auto it = cache.find(n);
        if (it != cache.end()) return it->second;
        GLint l = glGetUniformLocation(id, n);
        cache[n] = l; return l;
    }
    void use() const { glUseProgram(id); }
    void f1(const char* n, float a) { glUniform1f(u(n), a); }
    void i1(const char* n, int a) { glUniform1i(u(n), a); }
    void f2(const char* n, float a, float b) { glUniform2f(u(n), a, b); }
    void f3(const char* n, const vec3& v) { glUniform3f(u(n), v.x, v.y, v.z); }
    void f4(const char* n, float a, float b, float c, float d) { glUniform4f(u(n), a, b, c, d); }
    void m4(const char* n, const mat4& m) { glUniformMatrix4fv(u(n), 1, GL_FALSE, m.m); }
};

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
    // ---- smooth lofted bodies --------------------------------------------------------
    // Cross sections are super-ellipses (exponent n: 2 = ellipse, large = boxier);
    // vertex normals are averaged from the neighbouring grid points -> smooth shading.
    struct LoftSt { float x, cy, cz, hw, hh, n, bot; };
    typedef std::function<Mat(const vec3&, const vec3&)> ColFn;
    void loftSmooth(const mat4& T, const std::vector<LoftSt>& st, int ring, const ColFn& cf, bool capA = true, bool capB = true) {
        int S = (int)st.size();
        std::vector<vec3> P(S * ring), N(S * ring);
        for (int i = 0; i < S; i++)
            for (int k = 0; k < ring; k++) {
                float a = 2.f * PI_F * k / ring, c = std::cos(a), sn = std::sin(a), ex = 2.f / st[i].n;
                float px = sgn(c) * std::pow(std::fabs(c), ex), py = sgn(sn) * std::pow(std::fabs(sn), ex);
                if (py < 0) py *= st[i].bot;
                P[i * ring + k] = vec3(st[i].x, st[i].cy + py * st[i].hh, st[i].cz + px * st[i].hw);
            }
        for (int i = 0; i < S; i++)
            for (int k = 0; k < ring; k++) {
                vec3 t1 = P[std::min(i + 1, S - 1) * ring + k] - P[std::max(i - 1, 0) * ring + k];
                vec3 t2 = P[i * ring + (k + 1) % ring] - P[i * ring + (k + ring - 1) % ring];
                vec3 ctr(st[i].x, st[i].cy, st[i].cz), out = P[i * ring + k] - ctr;
                vec3 n = cross(t1, t2);
                if (length(n) < 1e-7f) n = (length(out) > 1e-6f) ? out : vec3(1, 0, 0);
                n = normalize(n);
                if (dot(n, out) < 0) n = -n;
                N[i * ring + k] = n;
            }
        auto V = [&](int i, int k) {
            k %= ring;
            const vec3& p = P[i * ring + k]; const vec3& n = N[i * ring + k];
            Mat m = cf(p, n);
            vert(T.point(p), xformNormal(T, n), m);
        };
        for (int i = 0; i + 1 < S; i++)
            for (int k = 0; k < ring; k++) {
                V(i, k); V(i + 1, k); V(i + 1, k + 1);
                V(i, k); V(i + 1, k + 1); V(i, k + 1);
            }
        auto cap = [&](int i, float dirx) {
            vec3 c(st[i].x, st[i].cy, st[i].cz); vec3 n(dirx, 0, 0);
            Mat m = cf(c, n);
            for (int k = 0; k < ring; k++) {
                vert(T.point(c), xformNormal(T, n), m);
                vert(T.point(P[i * ring + k]), xformNormal(T, n), m);
                vert(T.point(P[i * ring + (k + 1) % ring]), xformNormal(T, n), m);
            }
        };
        if (capA) cap(0, st[0].x > st[S - 1].x ? 1.f : -1.f);
        if (capB) cap(S - 1, st[0].x > st[S - 1].x ? -1.f : 1.f);
    }
    // Extruded convex polygon in the XY plane (end plates, floor edges): z in [zc-th/2, zc+th/2]
    void extrudePoly(const mat4& T, const std::vector<vec3>& pts, float zc, float th, const Mat& m) {
        int n = (int)pts.size();
        vec3 c(0, 0, 0); for (auto& p : pts) c += p; c = c / (float)n;
        for (int s = -1; s <= 1; s += 2) {
            float z = zc + s * th * 0.5f; vec3 nn = xformNormal(T, vec3(0, 0, (float)s));
            for (int i = 0; i < n; i++)
                tri(T.point({c.x, c.y, z}), T.point({pts[i].x, pts[i].y, z}), T.point({pts[(i + 1) % n].x, pts[(i + 1) % n].y, z}), nn, m);
        }
        for (int i = 0; i < n; i++) {
            const vec3 &a = pts[i], &b = pts[(i + 1) % n];
            vec3 e = b - a; vec3 nrm = normalize(vec3(e.y, -e.x, 0));
            if (dot(nrm, vec3((a.x + b.x) * 0.5f - c.x, (a.y + b.y) * 0.5f - c.y, 0)) < 0) nrm = -nrm;
            quadN(T.point({a.x, a.y, zc - th * 0.5f}), T.point({b.x, b.y, zc - th * 0.5f}), T.point({b.x, b.y, zc + th * 0.5f}),
                  T.point({a.x, a.y, zc + th * 0.5f}), xformNormal(T, nrm), m);
        }
    }
    // Curved wing element spanning z in [-span, span]; y rises towards the tips (lift * (z/span)^2)
    void wingElement(const mat4& T, float xc, float chord, float y0, float lift, float pitch, float span, float th, int segs, const Mat& m) {
        for (int i = 0; i < segs; i++) {
            float z0 = -span + 2 * span * i / segs, z1 = -span + 2 * span * (i + 1) / segs;
            auto Y = [&](float z) { float q = z / span; return y0 + lift * q * q; };
            vec3 le0(xc + chord * 0.5f, Y(z0), z0), te0(xc - chord * 0.5f, Y(z0) + pitch * chord, z0);
            vec3 le1(xc + chord * 0.5f, Y(z1), z1), te1(xc - chord * 0.5f, Y(z1) + pitch * chord, z1);
            vec3 d(0, th, 0);
            vec3 nTop = normalize(cross(le1 - le0, te0 - le0)); if (nTop.y < 0) nTop = -nTop;
            quadN(T.point(le0 + d), T.point(le1 + d), T.point(te1 + d), T.point(te0 + d), xformNormal(T, nTop), m);
            quadN(T.point(le0), T.point(le1), T.point(te1), T.point(te0), xformNormal(T, -nTop), m);
            quadN(T.point(le0), T.point(le1), T.point(le1 + d), T.point(le0 + d), xformNormal(T, vec3(1, 0, 0)), m);
            quadN(T.point(te0), T.point(te1), T.point(te1 + d), T.point(te0 + d), xformNormal(T, vec3(-1, 0, 0)), m);
        }
        for (int s = -1; s <= 1; s += 2) {   // closing end faces
            float z = s * span, q = 1.f;
            vec3 le(xc + chord * 0.5f, y0 + lift * q * q, z), te(xc - chord * 0.5f, y0 + lift * q * q + pitch * chord, z), d(0, th, 0);
            quadN(T.point(le), T.point(te), T.point(te + d), T.point(le + d), xformNormal(T, vec3(0, 0, (float)s)), m);
        }
    }
    void append(const MeshBuilder& o, const mat4& T) {
        for (auto x : o.v) {
            vec3 p = T.point({x.p[0], x.p[1], x.p[2]});
            vec3 n = xformNormal(T, {x.n[0], x.n[1], x.n[2]});
            x.p[0] = p.x; x.p[1] = p.y; x.p[2] = p.z; x.n[0] = n.x; x.n[1] = n.y; x.n[2] = n.z;
            v.push_back(x);
        }
    }
};

// ---------------------------------------------------------------------------
//  GPU mesh
// ---------------------------------------------------------------------------
struct Mesh {
    GLuint vao = 0, vbo = 0; GLsizei count = 0;
    vec3 bmin{1e9f, 1e9f, 1e9f}, bmax{-1e9f, -1e9f, -1e9f};
    void upload(const MeshBuilder& mb) {
        count = (GLsizei)mb.v.size();
        for (auto& x : mb.v) {
            bmin.x = std::min(bmin.x, x.p[0]); bmin.y = std::min(bmin.y, x.p[1]); bmin.z = std::min(bmin.z, x.p[2]);
            bmax.x = std::max(bmax.x, x.p[0]); bmax.y = std::max(bmax.y, x.p[1]); bmax.z = std::max(bmax.z, x.p[2]);
        }
        if (!vao) { glGenVertexArrays(1, &vao); glGenBuffers(1, &vbo); }
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, mb.v.size() * sizeof(Vertex), mb.v.data(), GL_STATIC_DRAW);
        GLsizei st = sizeof(Vertex);
        glEnableVertexAttribArray(0); glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, st, (void*)0);
        glEnableVertexAttribArray(1); glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, st, (void*)(3 * 4));
        glEnableVertexAttribArray(2); glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, st, (void*)(6 * 4));
        glEnableVertexAttribArray(3); glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, st, (void*)(9 * 4));
        glBindVertexArray(0);
    }
    void draw() const { if (count) { glBindVertexArray(vao); glDrawArrays(GL_TRIANGLES, 0, count); } }
};
