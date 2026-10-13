// ============================================================================
//  raytrace.h  -  Optional ray-tracing demonstration (key T)
//  A separate CPU ray tracer: a simplified F1 car built from primitives
//  (boxes, cylinders, spheres), a chequered reflective ground plane, two
//  light sources with hard shadows, Blinn-Phong shading and recursive
//  reflections.  The image is rendered progressively (time-sliced, several
//  anti-aliasing passes) so the application stays responsive.
// ============================================================================
#pragma once
#include "gfx.h"

struct RTMat { vec3 col; float refl, spec, shin; bool checker; };
struct RTPrim { int type; vec3 a, b; float r; int mat; };   // 0 sphere(a,r)  1 box(a=min,b=max)  2 cylinderZ(a=centre,r,b.z=half length)  3 plane y=0
struct RTHit { float t; vec3 p, n; int mat; };

struct RayTracer {
    static const int W = 512, H = 288;
    std::vector<RTPrim> prims; std::vector<RTMat> mats;
    std::vector<float> acc; std::vector<int> rowPass; std::vector<unsigned char> rgb;
    int row = 0, pass = 0, maxPass = 8; float azim = 0.75f, elev = 0.28f; bool done = false;
    GLuint tex = 0; double elapsed = 0; unsigned seed = 1;
    vec3 lightPos[2] = {vec3(-6, 9, 5), vec3(7, 6, -6)};

    void init() {
        mats = {
            {vec3(0.85f, 0.05f, 0.06f), 0.22f, 0.9f, 90.f, false},   // 0 car paint (red)
            {vec3(0.04f, 0.04f, 0.05f), 0.05f, 0.2f, 20.f, false},   // 1 rubber / carbon
            {vec3(0.85f, 0.85f, 0.88f), 0.55f, 0.9f, 120.f, false},  // 2 metal rims
            {vec3(0.5f), 0.28f, 0.4f, 40.f, true},                   // 3 checker floor
            {vec3(0.9f, 0.9f, 0.95f), 0.85f, 1.0f, 200.f, false},    // 4 chrome sphere
            {vec3(0.95f, 0.8f, 0.1f), 0.18f, 0.8f, 80.f, false},     // 5 yellow
            {vec3(0.1f, 0.3f, 0.85f), 0.35f, 0.9f, 100.f, false}};   // 6 blue glossy sphere
        auto box = [&](vec3 c, vec3 h, int m) { prims.push_back({1, c - h, c + h, 0, m}); };
        auto cyl = [&](vec3 c, float r, float hl, int m) { prims.push_back({2, c, vec3(0, 0, hl), r, m}); };
        auto sph = [&](vec3 c, float r, int m) { prims.push_back({0, c, vec3(), r, m}); };
        prims.push_back({3, vec3(), vec3(), 0, 3});
        // car (X forward, Z lateral), scaled so wheels sit on y = 0
        float s = 1.f;
        box(vec3(0.0f, 0.42f, 0) * s, vec3(1.25f, 0.2f, 0.42f) * s, 0);        // tub
        box(vec3(1.9f, 0.26f, 0) * s, vec3(0.75f, 0.1f, 0.2f) * s, 0);         // nose
        box(vec3(-1.0f, 0.62f, 0) * s, vec3(0.9f, 0.22f, 0.2f) * s, 0);        // engine cover
        box(vec3(-0.15f, 0.34f, 0.72f) * s, vec3(0.85f, 0.2f, 0.26f) * s, 0);  // sidepods
        box(vec3(-0.15f, 0.34f, -0.72f) * s, vec3(0.85f, 0.2f, 0.26f) * s, 0);
        box(vec3(-2.05f, 0.95f, 0) * s, vec3(0.2f, 0.02f, 0.62f) * s, 1);      // rear wing
        box(vec3(-2.05f, 0.75f, 0.62f) * s, vec3(0.28f, 0.28f, 0.015f) * s, 5);
        box(vec3(-2.05f, 0.75f, -0.62f) * s, vec3(0.28f, 0.28f, 0.015f) * s, 5);
        box(vec3(-2.0f, 0.6f, 0) * s, vec3(0.04f, 0.26f, 0.05f) * s, 1);
        box(vec3(2.55f, 0.1f, 0) * s, vec3(0.25f, 0.015f, 0.95f) * s, 1);      // front wing
        box(vec3(2.55f, 0.18f, 0.95f) * s, vec3(0.28f, 0.1f, 0.015f) * s, 5);
        box(vec3(2.55f, 0.18f, -0.95f) * s, vec3(0.28f, 0.1f, 0.015f) * s, 5);
        box(vec3(0.0f, 0.12f, 0) * s, vec3(1.9f, 0.025f, 0.7f) * s, 1);        // floor
        sph(vec3(0.15f, 0.78f, 0) * s, 0.15f * s, 5);                           // helmet
        cyl(vec3(1.55f, 0.34f, 0.9f), 0.34f, 0.16f, 1); cyl(vec3(1.55f, 0.34f, -0.9f), 0.34f, 0.16f, 1);   // wheels
        cyl(vec3(-1.45f, 0.36f, 0.92f), 0.36f, 0.2f, 1); cyl(vec3(-1.45f, 0.36f, -0.92f), 0.36f, 0.2f, 1);
        cyl(vec3(1.55f, 0.34f, 1.06f), 0.2f, 0.01f, 2); cyl(vec3(1.55f, 0.34f, -1.06f), 0.2f, 0.01f, 2);   // rim faces
        cyl(vec3(-1.45f, 0.36f, 1.13f), 0.21f, 0.01f, 2); cyl(vec3(-1.45f, 0.36f, -1.13f), 0.21f, 0.01f, 2);
        sph(vec3(-1.4f, 0.9f, 3.6f), 0.9f, 4);                                  // chrome sphere
        sph(vec3(3.4f, 0.6f, -3.1f), 0.6f, 6);                                  // blue sphere
        sph(vec3(-3.5f, 0.45f, -2.6f), 0.45f, 5);
        acc.assign(W * H * 3, 0.f); rowPass.assign(H, 0); rgb.assign(W * H * 3, 0);
        glGenTextures(1, &tex); glBindTexture(GL_TEXTURE_2D, tex);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, W, H, 0, GL_RGB, GL_UNSIGNED_BYTE, rgb.data());
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    }
    void reset() { std::fill(acc.begin(), acc.end(), 0.f); std::fill(rowPass.begin(), rowPass.end(), 0); row = 0; pass = 0; done = false; elapsed = 0; }
    float frand() { seed = seed * 1664525u + 1013904223u; return (seed >> 8) / 16777216.f; }

    bool hit(const RTPrim& p, const vec3& o, const vec3& d, float tmax, RTHit& h) const {
        float t = tmax;
        if (p.type == 0) {
            vec3 oc = o - p.a; float b = dot(oc, d), c = dot(oc, oc) - p.r * p.r, disc = b * b - c;
            if (disc < 0) return false;
            float sq = std::sqrt(disc), t0 = -b - sq; if (t0 < 1e-3f) t0 = -b + sq;
            if (t0 < 1e-3f || t0 > tmax) return false;
            h.t = t0; h.p = o + d * t0; h.n = normalize(h.p - p.a); h.mat = p.mat; return true;
        } else if (p.type == 1) {
            float t0 = 1e-3f, t1 = tmax; int ax = -1; float sg = 1;
            const float* oo = &o.x; const float* dd = &d.x; const float* mn = &p.a.x; const float* mx = &p.b.x;
            for (int i = 0; i < 3; i++) {
                if (std::fabs(dd[i]) < 1e-9f) { if (oo[i] < mn[i] || oo[i] > mx[i]) return false; continue; }
                float ta = (mn[i] - oo[i]) / dd[i], tb = (mx[i] - oo[i]) / dd[i]; float s = -1;
                if (ta > tb) { std::swap(ta, tb); s = 1; }
                if (ta > t0) { t0 = ta; ax = i; sg = s; }
                if (tb < t1) t1 = tb;
                if (t0 > t1) return false;
            }
            if (ax < 0) return false;
            h.t = t0; h.p = o + d * t0; vec3 n; (&n.x)[ax] = sg; h.n = n; h.mat = p.mat; return true;
        } else if (p.type == 2) {
            float best = 1e30f; vec3 bn;
            float ox = o.x - p.a.x, oy = o.y - p.a.y;
            float a = d.x * d.x + d.y * d.y, b = ox * d.x + oy * d.y, c = ox * ox + oy * oy - p.r * p.r;
            if (a > 1e-9f) {
                float disc = b * b - a * c;
                if (disc >= 0) {
                    float sq = std::sqrt(disc);
                    for (int k = 0; k < 2; k++) {
                        float tt = (-b + (k ? sq : -sq)) / a;
                        if (tt > 1e-3f && tt < best && tt < tmax) {
                            float z = o.z + d.z * tt - p.a.z;
                            if (std::fabs(z) <= p.b.z) { best = tt; bn = normalize(vec3(ox + d.x * tt, oy + d.y * tt, 0)); }
                        }
                    }
                }
            }
            if (std::fabs(d.z) > 1e-9f)
                for (int k = -1; k <= 1; k += 2) {
                    float tt = (p.a.z + k * p.b.z - o.z) / d.z;
                    if (tt > 1e-3f && tt < best && tt < tmax) {
                        float x = ox + d.x * tt, y = oy + d.y * tt;
                        if (x * x + y * y <= p.r * p.r) { best = tt; bn = vec3(0, 0, (float)k); }
                    }
                }
            if (best >= 1e29f) return false;
            h.t = best; h.p = o + d * best; h.n = bn; h.mat = p.mat; return true;
        } else {
            if (d.y > -1e-6f) return false;
            float tt = -o.y / d.y; if (tt < 1e-3f || tt > tmax) return false;
            h.t = tt; h.p = o + d * tt; h.n = vec3(0, 1, 0); h.mat = p.mat; (void)t; return true;
        }
    }
    bool closest(const vec3& o, const vec3& d, RTHit& h, float tmax = 1e9f) const {
        bool any = false; RTHit tmp;
        for (const RTPrim& p : prims) if (hit(p, o, d, tmax, tmp)) { tmax = tmp.t; h = tmp; any = true; }
        return any;
    }
    bool occluded(const vec3& o, const vec3& d, float dist) const {
        RTHit tmp;
        for (const RTPrim& p : prims) if (p.type != 3 && hit(p, o, d, dist, tmp)) return true;
        return false;
    }
    vec3 sky(const vec3& d) const {
        float t = clampf(d.y * 1.2f + 0.2f, 0.f, 1.f);
        return mix(vec3(0.75f, 0.85f, 0.98f), vec3(0.18f, 0.38f, 0.8f), t);
    }
    vec3 trace(const vec3& o, const vec3& d, int depth) const {
        RTHit h;
        if (!closest(o, d, h)) return sky(d);
        const RTMat& m = mats[h.mat];
        vec3 col = m.col;
        if (m.checker) { int cx = (int)std::floor(h.p.x), cz = (int)std::floor(h.p.z); col = ((cx + cz) & 1) ? vec3(0.9f) : vec3(0.12f); col = mix(col, vec3(0.5f), clampf((h.t - 22.f) / 40.f, 0.f, 1.f)); }
        vec3 n = h.n; if (dot(n, d) > 0) n = -n;
        vec3 res = col * 0.18f;
        for (int i = 0; i < 2; i++) {
            vec3 L = lightPos[i] - h.p; float dist = length(L); L = L / dist;
            if (occluded(h.p + n * 1e-3f, L, dist)) continue;
            float nl = std::max(0.f, dot(n, L));
            vec3 H = normalize(L - d);
            float sp = m.spec * std::pow(std::max(0.f, dot(n, H)), m.shin);
            res += col * (nl * 0.85f) + vec3(sp);
        }
        if (depth > 0 && m.refl > 0.01f) {
            vec3 r = d - n * (2.f * dot(d, n));
            res = res * (1.f - m.refl) + trace(h.p + n * 1e-3f, normalize(r), depth - 1) * m.refl;
        }
        return res;
    }
    void step(double budget) {
        if (done) return;
        double t0 = glfwGetTime();
        vec3 target(0, 0.5f, 0);
        float R = 9.5f;
        vec3 eye = target + vec3(std::cos(azim) * std::cos(elev), std::sin(elev), std::sin(azim) * std::cos(elev)) * R;
        vec3 f = normalize(target - eye), rt = normalize(cross(f, vec3(0, 1, 0))), up = cross(rt, f);
        float tn = std::tan(radians(36.f)), asp = (float)W / H;
        while (row < H) {
            for (int x = 0; x < W; x++) {
                float jx = pass == 0 ? 0.5f : frand(), jy = pass == 0 ? 0.5f : frand();
                float u = ((x + jx) / W * 2.f - 1.f) * tn * asp, v = (1.f - (row + jy) / H * 2.f) * tn;
                vec3 d = normalize(f + rt * u + up * v);
                vec3 c = trace(eye, d, 3);
                for (int k = 0; k < 3; k++) acc[(row * W + x) * 3 + k] += (&c.x)[k];
            }
            rowPass[row]++;
            for (int x = 0; x < W; x++)
                for (int k = 0; k < 3; k++) {
                    float v = acc[(row * W + x) * 3 + k] / rowPass[row];
                    v = std::pow(clampf(v, 0.f, 1.f), 1.f / 2.2f);
                    rgb[(row * W + x) * 3 + k] = (unsigned char)(v * 255.f);
                }
            row++;
            if (glfwGetTime() - t0 > budget) break;
        }
        if (row >= H) { row = 0; pass++; if (pass >= maxPass) done = true; }
        elapsed += glfwGetTime() - t0;
        glBindTexture(GL_TEXTURE_2D, tex);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, W, H, GL_RGB, GL_UNSIGNED_BYTE, rgb.data());
    }
    bool saveBMP(const char* path) const {
        FILE* f = fopen(path, "wb"); if (!f) return false;
        int rowB = (W * 3 + 3) & ~3, sz = 54 + rowB * H;
        unsigned char hd[54] = {'B', 'M'}; auto w32 = [&](int o, int v) { for (int i = 0; i < 4; i++) hd[o + i] = (v >> (8 * i)) & 255; };
        w32(2, sz); w32(10, 54); w32(14, 40); w32(18, W); w32(22, H); hd[26] = 1; hd[28] = 24; w32(34, rowB * H);
        fwrite(hd, 1, 54, f);
        std::vector<unsigned char> line(rowB, 0);
        for (int y = H - 1; y >= 0; y--) {
            for (int x = 0; x < W; x++) { line[x * 3] = rgb[(y * W + x) * 3 + 2]; line[x * 3 + 1] = rgb[(y * W + x) * 3 + 1]; line[x * 3 + 2] = rgb[(y * W + x) * 3]; }
            fwrite(line.data(), 1, rowB, f);
        }
        fclose(f); return true;
    }
};
