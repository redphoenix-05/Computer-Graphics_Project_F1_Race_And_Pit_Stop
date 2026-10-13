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

static const char* SKY_VS = R"GLSL(
#version 330 core
uniform mat4 uInvVP;
out vec3 vDir;
void main() {
    vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
    vec4 ndc = vec4(p * 2.0 - 1.0, 1.0, 1.0);
    vec4 w = uInvVP * ndc;
    vDir = w.xyz / w.w;
    gl_Position = vec4(ndc.xy, 0.9999, 1.0);
}
)GLSL";
static const char* SKY_FS = R"GLSL(
#version 330 core
in vec3 vDir;
uniform vec3 uCamPos, uSunDir, uSunCol, uZenith, uHorizon, uFogCol, uCloud;
uniform float uStars, uTime;
out vec4 FragColor;
float hash(vec2 p) { return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453); }
float vnoise(vec2 p) {
    vec2 i = floor(p), f = fract(p); f = f * f * (3.0 - 2.0 * f);
    return mix(mix(hash(i), hash(i + vec2(1, 0)), f.x), mix(hash(i + vec2(0, 1)), hash(i + vec2(1, 1)), f.x), f.y);
}
float fbm(vec2 p) { float a = 0.5, s = 0.0; for (int i = 0; i < 5; i++) { s += a * vnoise(p); p *= 2.03; a *= 0.5; } return s; }
void main() {
    vec3 d = normalize(vDir - uCamPos);
    float h = clamp(d.y, 0.0, 1.0);
    vec3 col = mix(uHorizon, uZenith, pow(h, 0.5));
    if (d.y < 0.0) col = mix(uHorizon, uFogCol, clamp(-d.y * 8.0, 0.0, 1.0));
    float sd = max(dot(d, uSunDir), 0.0);
    col += uSunCol * (pow(sd, 900.0) * 6.0 + pow(sd, 10.0) * 0.28);
    if (d.y > 0.01) {
        vec2 q = d.xz / (d.y + 0.12) * 1.3 + vec2(uTime * 0.01, 0.0);
        float n = fbm(q);
        float cl = smoothstep(0.52, 0.80, n) * clamp(d.y * 4.0, 0.0, 1.0);
        col = mix(col, uCloud * (0.75 + 0.5 * uSunCol / (1.0 + uSunCol)) , cl * 0.75);
        if (uStars > 0.0) {
            vec2 sp = floor(d.xz / (d.y + 0.3) * 160.0);
            float s = step(0.9965, hash(sp)) * uStars * (1.0 - cl);
            col += vec3(s);
        }
    }
    col = pow(col, vec3(1.0 / 2.2) * 1.0);
    FragColor = vec4(col, 1.0);
}
)GLSL";


// ---- Decals: text / marking quads textured with the font atlas, lit like the body they sit on
static const char* DECAL_VS_BODY = R"GLSL(
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNrm;
layout(location=2) in vec2 aUV;
layout(location=3) in vec4 aCol;
uniform mat4 uModel; uniform mat3 uNrmMat; uniform mat4 uVP;
out vec3 vW; out vec3 vN; out vec2 vUV; out vec4 vC;
void main() {
    vec4 w = uModel * vec4(aPos, 1.0);
    vW = w.xyz; vN = uNrmMat * aNrm; vUV = aUV; vC = aCol;
    gl_Position = uVP * w;
}
)GLSL";
static const char* DECAL_FS_BODY = R"GLSL(
in vec3 vW; in vec3 vN; in vec2 vUV; in vec4 vC;
uniform sampler2D uFont;
out vec4 FragColor;
void main() {
    float a = texture(uFont, vUV).r * vC.a;
    if (a < 0.02) discard;
    vec3 N = normalize(vN);
    if (!gl_FrontFacing) N = -N;
    vec3 c = lightModel(vW, N, vC.rgb, 0.35, 40.0, 0.0);
    FragColor = vec4(finish(c, length(uCamPos - vW)), a);
}
)GLSL";

static const char* SHADOW_VS = R"GLSL(
#version 330 core
layout(location=0) in vec3 aPos;
uniform mat4 uModel, uVP;
out vec2 vL;
void main() { vL = aPos.xz; gl_Position = uVP * uModel * vec4(aPos, 1.0); }
)GLSL";
static const char* SHADOW_FS = R"GLSL(
#version 330 core
in vec2 vL; uniform float uAlpha; out vec4 FragColor;
void main() { float a = (1.0 - smoothstep(0.25, 1.0, length(vL))) * uAlpha; FragColor = vec4(0.0, 0.0, 0.0, a); }
)GLSL";

static const char* HUD_VS = R"GLSL(
#version 330 core
layout(location=0) in vec2 aPos; layout(location=1) in vec2 aUV; layout(location=2) in vec4 aCol;
uniform vec2 uScreen; out vec2 vUV; out vec4 vCol;
void main() { vUV = aUV; vCol = aCol; gl_Position = vec4(aPos.x / uScreen.x * 2.0 - 1.0, 1.0 - aPos.y / uScreen.y * 2.0, 0.0, 1.0); }
)GLSL";
static const char* HUD_FS = R"GLSL(
#version 330 core
in vec2 vUV; in vec4 vCol; uniform sampler2D uTex; out vec4 FragColor;
void main() { FragColor = vec4(vCol.rgb, vCol.a * texture(uTex, vUV).r); }
)GLSL";

static const char* BLIT_VS = R"GLSL(
#version 330 core
out vec2 vUV; uniform vec4 uRect; // x0,y0,x1,y1 in NDC
void main() {
    vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);   // 0..2
    vec2 q = vec2(gl_VertexID == 1 || gl_VertexID == 3 ? 1.0 : 0.0, gl_VertexID >= 2 ? 1.0 : 0.0);
    vUV = vec2(q.x, 1.0 - q.y);
    gl_Position = vec4(mix(uRect.xy, uRect.zw, q), 0.0, 1.0);
}
)GLSL";
static const char* BLIT_FS = R"GLSL(
#version 330 core
in vec2 vUV; uniform sampler2D uTex; out vec4 FragColor;
void main() { FragColor = vec4(texture(uTex, vUV).rgb, 1.0); }
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


// ---------------------------------------------------------------------------
//  Decals: text and marking quads (sponsor names, race numbers, tyre lettering)
// ---------------------------------------------------------------------------
struct DecalVertex { float p[3], n[3], uv[2], c[4]; };
struct DecalMesh {
    GLuint vao = 0, vbo = 0; GLsizei count = 0;
    void upload(const std::vector<DecalVertex>& v) {
        count = (GLsizei)v.size();
        if (!vao) { glGenVertexArrays(1, &vao); glGenBuffers(1, &vbo); }
        glBindVertexArray(vao); glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, v.size() * sizeof(DecalVertex), v.data(), GL_STATIC_DRAW);
        GLsizei st = sizeof(DecalVertex);
        glEnableVertexAttribArray(0); glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, st, (void*)0);
        glEnableVertexAttribArray(1); glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, st, (void*)12);
        glEnableVertexAttribArray(2); glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, st, (void*)24);
        glEnableVertexAttribArray(3); glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, st, (void*)32);
    }
    void draw() const { if (count) { glBindVertexArray(vao); glDrawArrays(GL_TRIANGLES, 0, count); } }
};
// surf(u, v, pos, normal): maps 2D layout coordinates (metres) onto the surface the decal sits on
typedef std::function<void(float, float, vec3&, vec3&)> SurfFn;
struct DecalBuilder {
    std::vector<DecalVertex> v;
    void glyphQuad(const vec3 p[4], const vec3& n, float u0, float v0, float u1, float v1, const vec3& col, float a) {
        float uv[4][2] = {{u0, v1}, {u1, v1}, {u1, v0}, {u0, v0}};   // p0 bottom-left, p1 bottom-right, p2 top-right, p3 top-left
        int idx[6] = {0, 1, 2, 0, 2, 3};
        for (int k = 0; k < 6; k++) {
            DecalVertex d; int i = idx[k];
            d.p[0] = p[i].x; d.p[1] = p[i].y; d.p[2] = p[i].z; d.n[0] = n.x; d.n[1] = n.y; d.n[2] = n.z;
            d.uv[0] = uv[i][0]; d.uv[1] = uv[i][1]; d.c[0] = col.x; d.c[1] = col.y; d.c[2] = col.z; d.c[3] = a;
            v.push_back(d);
        }
    }
    // text with its baseline-left corner at (u0, v0); glyph height h (metres); returns width
    float text(const std::string& str, float u0, float v0, float h, const vec3& col, const SurfFn& surf, float a = 1.f, float spacing = 0.92f) {
        float w = h * 0.5f * spacing * 1.f;   // advance
        float gw = h * 0.5f;
        for (size_t i = 0; i < str.size(); i++) {
            int ch = (unsigned char)str[i]; if (ch < 32 || ch > 126) ch = '?';
            if (ch == ' ') continue;
            int idx = ch - 32, cx = idx % FONT_COLS, cy = idx / FONT_COLS;
            float tu0 = (cx * FONT_CW + 1.5f) / FONT_W, tv0 = (cy * FONT_CH + 3.f) / FONT_H;
            float tu1 = (cx * FONT_CW + FONT_CW - 1.5f) / FONT_W, tv1 = (cy * FONT_CH + FONT_CH - 3.f) / FONT_H;
            float ua = u0 + i * w, ub = ua + gw;
            vec3 p[4], n0, n1, n2, n3, nn;
            surf(ua, v0, p[0], n0); surf(ub, v0, p[1], n1); surf(ub, v0 + h, p[2], n2); surf(ua, v0 + h, p[3], n3);
            nn = normalize(n0 + n1 + n2 + n3);
            glyphQuad(p, nn, tu0, tv0, tu1, tv1, col, a);
        }
        return str.size() * w;
    }
    float width(const std::string& s, float h) const { return s.size() * h * 0.5f * 0.92f; }
    // solid coloured strip (uses the white block of the atlas), subdivided so it follows curved surfaces
    void bar(float u0, float v0, float w, float h, const vec3& col, const SurfFn& surf, int seg = 4, float a = 1.f) {
        float tu = 16.f / FONT_W, tv = 304.f / FONT_H;
        for (int i = 0; i < seg; i++) {
            float ua = u0 + w * i / seg, ub = u0 + w * (i + 1) / seg;
            vec3 p[4], n0, n1, n2, n3;
            surf(ua, v0, p[0], n0); surf(ub, v0, p[1], n1); surf(ub, v0 + h, p[2], n2); surf(ua, v0 + h, p[3], n3);
            glyphQuad(p, normalize(n0 + n1 + n2 + n3), tu, tv, tu, tv, col, a);
        }
    }
};

struct Frustum {
    float pl[6][4];
    void set(const mat4& M) {
        auto row = [&](int i, float* r) { r[0] = M.m[i]; r[1] = M.m[4 + i]; r[2] = M.m[8 + i]; r[3] = M.m[12 + i]; };
        float r0[4], r1[4], r2[4], r3[4]; row(0, r0); row(1, r1); row(2, r2); row(3, r3);
        for (int i = 0; i < 4; i++) {
            pl[0][i] = r3[i] + r0[i]; pl[1][i] = r3[i] - r0[i];
            pl[2][i] = r3[i] + r1[i]; pl[3][i] = r3[i] - r1[i];
            pl[4][i] = r3[i] + r2[i]; pl[5][i] = r3[i] - r2[i];
        }
    }
    bool visible(const vec3& mn, const vec3& mx) const {
        for (int i = 0; i < 6; i++) {
            const float* p = pl[i];
            vec3 q(p[0] > 0 ? mx.x : mn.x, p[1] > 0 ? mx.y : mn.y, p[2] > 0 ? mx.z : mn.z);
            if (p[0] * q.x + p[1] * q.y + p[2] * q.z + p[3] < 0) return false;
        }
        return true;
    }
};

// Spatially chunked static geometry: each grid cell becomes one VBO that is
// frustum- and distance-culled every frame.
struct StaticBatch {
    float cell = 220.f;
    std::map<long long, MeshBuilder> cells;
    std::vector<Mesh> chunks;
    size_t tris = 0;
    MeshBuilder& at(float x, float z) {
        long long ix = (long long)std::floor(x / cell), iz = (long long)std::floor(z / cell);
        return cells[(ix + 100000) * 200000LL + (iz + 100000)];
    }
    MeshBuilder& at(const vec3& p) { return at(p.x, p.z); }
    void finalize() {
        chunks.clear();
        for (auto& kv : cells) {
            if (kv.second.v.empty()) continue;
            chunks.emplace_back();
            chunks.back().upload(kv.second);
            tris += kv.second.v.size() / 3;
        }
        cells.clear();
    }
};

// ---------------------------------------------------------------------------
//  Lighting presets / light sources
// ---------------------------------------------------------------------------
struct LightSrc { vec3 pos, col, dir; float radius = 30, cosIn = 0.9f, cosOut = 0.8f; bool spot = false; };

struct Preset {
    const char* name;
    vec3 sunDir, sunCol, skyAmb, gndAmb, zenith, horizon, fog;
    float fogDen, lamp, stars;
    vec3 cloud;
};

enum ShadeMode { SH_HYBRID = 0, SH_GOURAUD = 1, SH_PHONG = 2 };

static GLuint makeNoiseTexture() {
    const int S = 256;
    std::vector<unsigned char> d(S * S);
    auto hashv = [](int x, int y, int seed) {
        unsigned h = (unsigned)(x * 374761393 + y * 668265263 + seed * 69069);
        h = (h ^ (h >> 13)) * 1274126177u; h ^= h >> 16; return (h & 0xffff) / 65535.f;
    };
    for (int y = 0; y < S; y++)
        for (int x = 0; x < S; x++) {
            float sum = 0, amp = 0.5f, tot = 0;
            for (int o = 0; o < 5; o++) {
                int f = 4 << o;
                float fx = (float)x / S * f, fy = (float)y / S * f;
                int ix = (int)fx, iy = (int)fy; float tx = fx - ix, ty = fy - iy;
                tx = tx * tx * (3 - 2 * tx); ty = ty * ty * (3 - 2 * ty);
                auto H = [&](int a, int b) { return hashv(a % f, b % f, o); };
                float v = mixf(mixf(H(ix, iy), H(ix + 1, iy), tx), mixf(H(ix, iy + 1), H(ix + 1, iy + 1), tx), ty);
                sum += v * amp; tot += amp; amp *= 0.5f;
            }
            d[y * S + x] = (unsigned char)(clampf(sum / tot, 0, 1) * 255);
        }
    GLuint t; glGenTextures(1, &t); glBindTexture(GL_TEXTURE_2D, t);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, S, S, 0, GL_RED, GL_UNSIGNED_BYTE, d.data());
    glGenerateMipmap(GL_TEXTURE_2D);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    return t;
}

// ---------------------------------------------------------------------------
//  Renderer
// ---------------------------------------------------------------------------
struct Renderer {
    Shader gouraud, phong, sky, shadow, blit, decal;
    GLuint fontTex = 0;
    GLuint noiseTex = 0, emptyVao = 0;
    Mesh shadowQuad;
    ShadeMode mode = SH_HYBRID;
    bool wire = false;
    Shader* cur = nullptr;
    mat4 VP; vec3 camPos; Frustum fr;
    std::vector<LightSrc> lights;
    const Preset* pre = nullptr;
    float time = 0;
    int drawCalls = 0;

    void init() {
        std::string vsA = std::string("#version 330 core\n") + GLSL_LIGHTING + GLSL_ATTRIBS;
        std::string fsA = std::string("#version 330 core\n") + GLSL_LIGHTING + GLSL_DETAIL;
        std::string vsD = std::string("#version 330 core\n") + GLSL_LIGHTING + DECAL_VS_BODY;
        std::string fsD = std::string("#version 330 core\n") + GLSL_LIGHTING + DECAL_FS_BODY;
        gouraud.build(vsA + GOURAUD_VS_BODY, fsA + GOURAUD_FS_BODY, "gouraud");
        phong.build(vsA + PHONG_VS_BODY, fsA + PHONG_FS_BODY, "phong");
        decal.build(vsD, fsD, "decal");
        sky.build(SKY_VS, SKY_FS, "sky");
        shadow.build(SHADOW_VS, SHADOW_FS, "shadow");
        blit.build(BLIT_VS, BLIT_FS, "blit");
        noiseTex = makeNoiseTexture();
        glGenVertexArrays(1, &emptyVao);
        MeshBuilder q;
        q.quadUp({-1, 0, -1}, {-1, 0, 1}, {1, 0, 1}, {1, 0, -1}, MT(vec3(0)));
        shadowQuad.upload(q);
        for (Shader* s : {&gouraud, &phong}) { s->use(); s->i1("uNoise", 0); }
        decal.use(); decal.i1("uFont", 1);
        glActiveTexture(GL_TEXTURE0); glBindTexture(GL_TEXTURE_2D, noiseTex);
    }

    void beginFrame(const mat4& view, const mat4& proj, const vec3& cam, const Preset& p, float t) {
        VP = proj * view; camPos = cam; pre = &p; time = t; drawCalls = 0;
        fr.set(VP);
        for (Shader* s : {&gouraud, &phong, &decal}) {
            s->use();
            s->m4("uVP", VP); s->f3("uCamPos", cam);
            s->f3("uSunDir", normalize(p.sunDir)); s->f3("uSunCol", p.sunCol);
            s->f3("uSkyAmb", p.skyAmb); s->f3("uGndAmb", p.gndAmb);
            s->f3("uFogCol", p.fog); s->f1("uFogDen", p.fogDen);
            if (s != &decal) s->f1("uAlpha", 1.f);
        }
        glActiveTexture(GL_TEXTURE0); glBindTexture(GL_TEXTURE_2D, noiseTex);
        cur = nullptr;
    }
    void uploadLights() {
        int n = std::min((int)lights.size(), 16);
        float pos[48], col[48], dir[48], par[64];
        for (int i = 0; i < n; i++) {
            const LightSrc& L = lights[i];
            pos[i * 3] = L.pos.x; pos[i * 3 + 1] = L.pos.y; pos[i * 3 + 2] = L.pos.z;
            col[i * 3] = L.col.x; col[i * 3 + 1] = L.col.y; col[i * 3 + 2] = L.col.z;
            dir[i * 3] = L.dir.x; dir[i * 3 + 1] = L.dir.y; dir[i * 3 + 2] = L.dir.z;
            par[i * 4] = L.radius; par[i * 4 + 1] = L.cosIn; par[i * 4 + 2] = L.cosOut; par[i * 4 + 3] = L.spot ? 1.f : 0.f;
        }
        for (Shader* s : {&gouraud, &phong, &decal}) {
            s->use(); s->i1("uNL", n);
            if (n) {
                glUniform3fv(s->u("uLPos"), n, pos); glUniform3fv(s->u("uLCol"), n, col);
                glUniform3fv(s->u("uLDir"), n, dir); glUniform4fv(s->u("uLPar"), n, par);
            }
        }
        cur = nullptr;
    }
    Shader* pick(bool hero) {
        if (mode == SH_GOURAUD) return &gouraud;
        if (mode == SH_PHONG) return &phong;
        return hero ? &phong : &gouraud;
    }
    void draw(const Mesh& m, const mat4& model, const vec3& tint = vec3(1, 1, 1), float emis = 0.f,
              bool hero = true, float alpha = 1.f) {
        Shader* s = pick(hero);
        if (s != cur) { s->use(); cur = s; }
        s->m4("uModel", model);
        float nm[9]; normalMatrix(model, nm);
        glUniformMatrix3fv(s->u("uNrmMat"), 1, GL_FALSE, nm);
        s->f3("uTint", tint); s->f1("uEmis", emis); s->f1("uAlpha", alpha);
        m.draw(); drawCalls++;
    }
    void drawDecal(const DecalMesh& m, const mat4& model) {
        decal.use();
        decal.m4("uModel", model);
        float nm[9]; normalMatrix(model, nm);
        glUniformMatrix3fv(decal.u("uNrmMat"), 1, GL_FALSE, nm);
        glActiveTexture(GL_TEXTURE1); glBindTexture(GL_TEXTURE_2D, fontTex); glActiveTexture(GL_TEXTURE0);
        glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE); glEnable(GL_POLYGON_OFFSET_FILL); glPolygonOffset(-2.f, -6.f);
        m.draw(); drawCalls++;
        glDisable(GL_POLYGON_OFFSET_FILL); glDepthMask(GL_TRUE); glDisable(GL_BLEND);
        cur = nullptr;
    }
    void drawBatch(const StaticBatch& b, bool hero, float maxDist, float alpha = 1.f) {
        mat4 I;
        for (auto& c : b.chunks) {
            if (!fr.visible(c.bmin, c.bmax)) continue;
            vec3 cp(clampf(camPos.x, c.bmin.x, c.bmax.x), clampf(camPos.y, c.bmin.y, c.bmax.y), clampf(camPos.z, c.bmin.z, c.bmax.z));
            if (length(cp - camPos) > maxDist) continue;
            draw(c, I, vec3(1, 1, 1), 0.f, hero, alpha);
        }
    }
    void drawSky(const mat4& view, const mat4& proj) {
        glDisable(GL_DEPTH_TEST); glDepthMask(GL_FALSE);
        sky.use();
        sky.m4("uInvVP", inverse(proj * view));
        sky.f3("uCamPos", camPos);
        sky.f3("uSunDir", normalize(pre->sunDir)); sky.f3("uSunCol", pre->sunCol);
        sky.f3("uZenith", pre->zenith); sky.f3("uHorizon", pre->horizon); sky.f3("uFogCol", pre->fog * pre->fog);
        sky.f1("uStars", pre->stars); sky.f1("uTime", time); sky.f3("uCloud", pre->cloud);
        glBindVertexArray(emptyVao); glDrawArrays(GL_TRIANGLES, 0, 3);
        glDepthMask(GL_TRUE); glEnable(GL_DEPTH_TEST);
        cur = nullptr;
    }
    void drawBlob(const vec3& pos, float yaw, float sx, float sz, float alpha) {
        shadow.use(); shadow.m4("uVP", VP);
        mat4 M = translate(pos) * rotY(-yaw) * scale(sx, 1, sz);
        shadow.m4("uModel", M); shadow.f1("uAlpha", alpha);
        shadowQuad.draw(); cur = nullptr;
    }
};

// ---------------------------------------------------------------------------
//  HUD / 2D overlay
// ---------------------------------------------------------------------------
struct Hud {
    Shader sh; GLuint vao = 0, vbo = 0, tex = 0;
    std::vector<float> buf; int W = 1, H = 1;
    void init() {
        sh.build(HUD_VS, HUD_FS, "hud");
        glGenTextures(1, &tex); glBindTexture(GL_TEXTURE_2D, tex);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, FONT_W, FONT_H, 0, GL_RED, GL_UNSIGNED_BYTE, FONT_DATA);
        glGenerateMipmap(GL_TEXTURE_2D);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glGenVertexArrays(1, &vao); glGenBuffers(1, &vbo);
        glBindVertexArray(vao); glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glEnableVertexAttribArray(0); glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 32, (void*)0);
        glEnableVertexAttribArray(1); glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 32, (void*)8);
        glEnableVertexAttribArray(2); glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, 32, (void*)16);
    }
    void begin(int w, int h) { W = w; H = h; buf.clear(); }
    void quad(float x0, float y0, float x1, float y1, float u0, float v0, float u1, float v1, const float* c) {
        float q[6][4] = {{x0, y0, u0, v0}, {x1, y0, u1, v0}, {x1, y1, u1, v1}, {x0, y0, u0, v0}, {x1, y1, u1, v1}, {x0, y1, u0, v1}};
        for (auto& p : q) { buf.insert(buf.end(), {p[0], p[1], p[2], p[3], c[0], c[1], c[2], c[3]}); }
    }
    void rect(float x, float y, float w, float h, float r, float g, float b, float a = 1.f) {
        float c[4] = {r, g, b, a};
        float u = 16.f / FONT_W, v = 304.f / FONT_H;
        quad(x, y, x + w, y + h, u, v, u, v, c);
    }
    void rectc(float x, float y, float w, float h, const vec3& c, float a = 1.f) { rect(x, y, w, h, c.x, c.y, c.z, a); }
    // thick line = rotated quad
    void line(float x0, float y0, float x1, float y1, float th, float r, float g, float b, float a = 1.f) {
        float dx = x1 - x0, dy = y1 - y0, l = std::sqrt(dx * dx + dy * dy); if (l < 1e-4f) return;
        float nx = -dy / l * th * 0.5f, ny = dx / l * th * 0.5f;
        float c[4] = {r, g, b, a}; float u = 16.f / FONT_W, v = 304.f / FONT_H;
        float q[6][2] = {{x0 + nx, y0 + ny}, {x1 + nx, y1 + ny}, {x1 - nx, y1 - ny}, {x0 + nx, y0 + ny}, {x1 - nx, y1 - ny}, {x0 - nx, y0 - ny}};
        for (auto& p : q) buf.insert(buf.end(), {p[0], p[1], u, v, c[0], c[1], c[2], c[3]});
    }
    // Glyph cell is snapped to whole pixels so text stays crisp at any UI scale.
    static int cellW(float sc) { return std::max(4, (int)std::lround(8.0f * sc)); }
    static int cellH(float sc) { return std::max(7, (int)std::lround(16.0f * sc)); }
    float textW(const std::string& s, float sc) const { return (float)(s.size() * cellW(sc)); }
    void text(float x, float y, float sc, const vec3& col, const std::string& s, float a = 1.f) {
        float c[4] = {col.x, col.y, col.z, a};
        int cw = cellW(sc), ch = cellH(sc);
        float x0 = std::floor(x + 0.5f), y0 = std::floor(y + 0.5f);
        for (size_t i = 0; i < s.size(); i++) {
            int chr = (unsigned char)s[i]; if (chr < 32 || chr > 126) chr = '?';
            int idx = chr - 32, cx = idx % FONT_COLS, cy = idx / FONT_COLS;
            float u0 = (float)(cx * FONT_CW) / FONT_W, v0 = (float)(cy * FONT_CH) / FONT_H;
            float u1 = (float)(cx * FONT_CW + FONT_CW) / FONT_W, v1 = (float)(cy * FONT_CH + FONT_CH) / FONT_H;
            quad(x0 + (float)(i * cw), y0, x0 + (float)((i + 1) * cw), y0 + ch, u0, v0, u1, v1, c);
        }
    }
    void textShadow(float x, float y, float sc, const vec3& col, const std::string& s, float a = 1.f) {
        { float o = std::max(1.f, std::floor(sc * 1.0f + 0.5f)); text(x + o, y + o, sc, vec3(0, 0, 0), s, a * 0.75f); }
        text(x, y, sc, col, s, a);
    }
    void textC(float cx, float y, float sc, const vec3& col, const std::string& s, float a = 1.f) {
        textShadow(cx - textW(s, sc) * 0.5f, y, sc, col, s, a);
    }
    void flush() {
        if (buf.empty()) return;
        glDisable(GL_DEPTH_TEST); glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        GLint pm[2]; glGetIntegerv(GL_POLYGON_MODE, pm);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        sh.use(); sh.f2("uScreen", (float)W, (float)H);
        glActiveTexture(GL_TEXTURE0); glBindTexture(GL_TEXTURE_2D, tex);
        glBindVertexArray(vao); glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, buf.size() * 4, buf.data(), GL_STREAM_DRAW);
        glDrawArrays(GL_TRIANGLES, 0, (GLsizei)(buf.size() / 8));
        glPolygonMode(GL_FRONT_AND_BACK, (GLenum)pm[0]);
        glDisable(GL_BLEND); glEnable(GL_DEPTH_TEST);
    }
};
