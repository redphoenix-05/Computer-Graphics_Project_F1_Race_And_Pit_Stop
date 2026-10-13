// ============================================================================
//  track.h  -  Circuit geometry, racing-line speed profile and pit-lane layout
//
//  The circuit is a closed Catmull-Rom spline through hand-placed control
//  points, resampled to uniform arc length.  s = distance along the circuit
//  (s = 0 is the start/finish line).  Lateral offset d is measured to the
//  RIGHT of the direction of travel.  The pit lane runs parallel to the main
//  straight on the right-hand side; because the main straight lies along the
//  world X axis (z = 0) all pit buildings can be placed in world coordinates.
// ============================================================================
#pragma once
#include <vector>
#include <unordered_map>
#include "math3d.h"

// ---------------------------------------------------------------------------
//  Pit-lane constants (world X == distance from the start line on the straight)
// ---------------------------------------------------------------------------
namespace PitC {
constexpr float ENTRY_X0 = -330.f, ENTRY_X1 = -215.f;   // entry ramp
constexpr float EXIT_X0 = 125.f, EXIT_X1 = 235.f;       // exit ramp
constexpr float WALL_X0 = -205.f, WALL_X1 = 135.f;      // pit wall extent
constexpr float WALL_D = 12.3f;                         // pit wall lateral position
constexpr float LANE_IN = 12.9f, LANE_OUT = 27.0f;      // pit-lane road edges
constexpr float FAST_D = 16.5f, WORK_D = 22.5f;         // car paths: fast / working lane
constexpr float LIMIT = 27.8f;                          // 100 km/h pit-lane speed limit (m/s)
constexpr int NBOX = 5;
constexpr float BOX_X[NBOX] = {-56.f, -28.f, 0.f, 28.f, 56.f};
constexpr int PLAYER_BOX = 2;
constexpr float GARAGE_W = 26.f, GARAGE_DEPTH = 15.f;
constexpr float COMMIT_X0 = -470.f, COMMIT_X1 = -255.f; // window where the pit entry may be taken
}

inline float pitW(float u) {   // 0 on the plain track, 1 in the pit-lane zone
    return smoothstepf(PitC::ENTRY_X0, PitC::ENTRY_X1, u) * (1.f - smoothstepf(PitC::EXIT_X0, PitC::EXIT_X1, u));
}
inline float pitPathOffset(float u) {   // lateral offset of the car's pit path
    using namespace PitC;
    float w = pitW(u);
    float base = FAST_D * w;
    float box = smoothstepf(BOX_X[PLAYER_BOX] - 75.f, BOX_X[PLAYER_BOX] - 28.f, u) *
                (1.f - smoothstepf(BOX_X[PLAYER_BOX] + 28.f, BOX_X[PLAYER_BOX] + 75.f, u));
    return base + (WORK_D - FAST_D) * box;
}
inline float pitInner(float u) { return 7.f + (PitC::LANE_IN - 7.f) * pitW(u); }
inline float pitOuter(float u) { return 7.f + (PitC::LANE_OUT - 7.f) * pitW(u); }

struct Proj { int i = 0; float s = 0, d = 0; };

struct Track {
    static constexpr float HALFW = 7.0f;
    int N = 0; float L = 0, ds = 1;
    std::vector<vec3> p, t, n;      // centre, tangent, right normal
    std::vector<float> k;           // signed curvature (+ = right turn)
    std::vector<float> ang;         // heading angle psi, forward = (cos psi, sin psi) in (x,z)
    vec3 bmin, bmax, center;
    std::unordered_map<long long, std::vector<int>> grid;

    int wrap(int i) const { return ((i % N) + N) % N; }
    float wrapS(float s) const { s = std::fmod(s, L); return s < 0 ? s + L : s; }
    float sRel(float s) const { s = wrapS(s); return s > L * 0.5f ? s - L : s; }

    void build() {
        static const float P[][2] = {
            {-500, 0}, {-440, 0}, {-380, 0}, {-300, 0}, {-100, 0}, {100, 0}, {300, 0}, {400, 0}, {440, 0}, {480, 0},
            {535, 8}, {582, 55}, {592, 115}, {578, 185}, {602, 250}, {580, 315}, {540, 380}, {470, 418}, {385, 425},
            {290, 415}, {200, 428}, {115, 462}, {45, 497}, {-35, 490}, {-100, 515}, {-185, 545}, {-265, 520},
            {-290, 465}, {-250, 420}, {-180, 395}, {-110, 420}, {-70, 380}, {-95, 320}, {-180, 300}, {-290, 310},
            {-360, 300}, {-405, 255}, {-440, 200}, {-480, 170}, {-540, 150}, {-585, 105}, {-590, 45}, {-560, 8}};
        const int NP = (int)(sizeof(P) / sizeof(P[0]));
        std::vector<vec3> dense;
        const int per = 60;
        for (int i = 0; i < NP; i++) {
            auto G = [&](int j) { j = (j % NP + NP) % NP; return vec3(P[j][0], 0, P[j][1]); };
            vec3 p0 = G(i - 1), p1 = G(i), p2 = G(i + 1), p3 = G(i + 2);
            for (int q = 0; q < per; q++) {
                float tt = (float)q / per;
                vec3 r = (p1 * 2.f + (p2 - p0) * tt + (p0 * 2.f - p1 * 5.f + p2 * 4.f - p3) * (tt * tt) +
                          (p1 * 3.f - p0 - p2 * 3.f + p3) * (tt * tt * tt)) * 0.5f;
                dense.push_back(r);
            }
        }
        // arc length parameterisation
        std::vector<float> cum(dense.size() + 1, 0.f);
        for (size_t i = 0; i < dense.size(); i++)
            cum[i + 1] = cum[i] + length(dense[(i + 1) % dense.size()] - dense[i]);
        float total = cum.back();
        N = (int)(total / 2.f); L = total; ds = L / N;
        std::vector<vec3> raw(N);
        size_t j = 0;
        for (int i = 0; i < N; i++) {
            float s = ds * i;
            while (j + 1 < cum.size() - 1 && cum[j + 1] < s) j++;
            float f = (s - cum[j]) / std::max(cum[j + 1] - cum[j], 1e-6f);
            raw[i] = mix(dense[j], dense[(j + 1) % dense.size()], f);
        }
        // rotate so that sample 0 is the start line (x = 0 on the main straight)
        int best = 0; float bd = 1e9f;
        for (int i = 0; i < N; i++) {
            vec3 tt = raw[(i + 1) % N] - raw[(i + N - 1) % N];
            if (tt.x > 0 && std::fabs(raw[i].z) < 5.f && std::fabs(raw[i].x) < bd) { bd = std::fabs(raw[i].x); best = i; }
        }
        p.resize(N);
        for (int i = 0; i < N; i++) p[i] = raw[(i + best) % N];
        // tangent / normal / curvature
        t.resize(N); n.resize(N); k.resize(N); ang.resize(N);
        for (int i = 0; i < N; i++) {
            vec3 d = p[(i + 1) % N] - p[(i + N - 1) % N]; d.y = 0;
            t[i] = normalize(d);
            n[i] = vec3(-t[i].z, 0, t[i].x);
            ang[i] = std::atan2(t[i].z, t[i].x);
        }
        std::vector<float> kr(N);
        for (int i = 0; i < N; i++) kr[i] = wrapAngle(ang[(i + 1) % N] - ang[(i + N - 1) % N]) / (2 * ds);
        for (int i = 0; i < N; i++) {      // box smoothing
            float s = 0; for (int q = -5; q <= 5; q++) s += kr[wrap(i + q)];
            k[i] = s / 11.f;
        }
        bmin = vec3(1e9f, 0, 1e9f); bmax = vec3(-1e9f, 0, -1e9f);
        for (auto& q : p) { bmin.x = std::min(bmin.x, q.x); bmin.z = std::min(bmin.z, q.z); bmax.x = std::max(bmax.x, q.x); bmax.z = std::max(bmax.z, q.z); }
        center = (bmin + bmax) * 0.5f;
        for (int i = 0; i < N; i++) grid[cellKey(p[i].x, p[i].z)].push_back(i);
    }

    static long long cellKey(float x, float z) {
        return ((long long)std::floor(x / 60.f) + 5000) * 10000LL + ((long long)std::floor(z / 60.f) + 5000);
    }
    // distance from a world point to the circuit centre line (spatial hash)
    float distToTrack(float x, float z) const {
        float best = 1e9f;
        int cx = (int)std::floor(x / 60.f), cz = (int)std::floor(z / 60.f);
        for (int a = -2; a <= 2; a++)
            for (int b = -2; b <= 2; b++) {
                auto it = grid.find(((long long)(cx + a) + 5000) * 10000LL + ((long long)(cz + b) + 5000));
                if (it == grid.end()) continue;
                for (int i : it->second) {
                    float dx = p[i].x - x, dz = p[i].z - z;
                    best = std::min(best, std::sqrt(dx * dx + dz * dz));
                }
            }
        return best;
    }

    // interpolated point at arc length s and lateral offset d (to the right)
    vec3 pos(float s, float d = 0) const {
        s = wrapS(s);
        float f = s / ds; int i = (int)f; float u = f - i;
        int a = wrap(i), b = wrap(i + 1);
        vec3 c = mix(p[a], p[b], u);
        vec3 nn = normalize(mix(n[a], n[b], u));
        return c + nn * d;
    }
    vec3 tangent(float s) const {
        float f = wrapS(s) / ds; int i = (int)f; float u = f - i;
        return normalize(mix(t[wrap(i)], t[wrap(i + 1)], u));
    }
    vec3 normal(float s) const { vec3 tt = tangent(s); return vec3(-tt.z, 0, tt.x); }
    float curvature(float s) const {
        float f = wrapS(s) / ds; int i = (int)f; float u = f - i;
        return mixf(k[wrap(i)], k[wrap(i + 1)], u);
    }
    float heading(float s) const { vec3 tt = tangent(s); return std::atan2(tt.z, tt.x); }

    // Project a world position on the circuit; hint = previous sample (or -1 for global search)
    Proj project(const vec3& q, int hint, int window = 40) const {
        int bi = 0; float bd = 1e18f;
        auto test = [&](int i) {
            float dx = p[i].x - q.x, dz = p[i].z - q.z, dd = dx * dx + dz * dz;
            if (dd < bd) { bd = dd; bi = i; }
        };
        if (hint < 0) for (int i = 0; i < N; i++) test(i);
        else for (int o = -window; o <= window; o++) test(wrap(hint + o));
        Proj r; r.i = bi;
        vec3 dv = q - p[bi];
        r.s = wrapS(bi * ds + dot(dv, t[bi]));
        r.d = dot(dv, n[bi]);
        return r;
    }

    // Maximum speed along a line offset by 'lateral' from the centre line.
    // Corner speed solves v^2 = R * aLat(v) with aLat(v) = a0 + aeroK*v^2 (downforce),
    // followed by forward (acceleration) and backward (braking) passes.
    std::vector<float> speedProfile(float lateral, float a0, float aeroK, float vmax, float aAcc, float aBrk) const {
        std::vector<float> v(N);
        for (int i = 0; i < N; i++) {
            float ko = k[i] / std::max(0.2f, 1.f - lateral * k[i]);
            float R = 1.f / std::max(std::fabs(ko), 1e-4f);
            float den = 1.f - aeroK * R;
            v[i] = den <= 0.05f ? vmax : std::min(vmax, std::sqrt(a0 * R / den));
        }
        for (int pass = 0; pass < 3; pass++) {
            for (int i = 2 * N; i > 0; i--) {           // braking (backwards)
                int a = wrap(i - 1), b = wrap(i);
                v[a] = std::min(v[a], std::sqrt(v[b] * v[b] + 2 * aBrk * ds));
            }
            for (int i = 0; i < 2 * N; i++) {           // acceleration (forwards)
                int a = wrap(i), b = wrap(i + 1);
                v[b] = std::min(v[b], std::sqrt(v[a] * v[a] + 2 * aAcc * ds));
            }
        }
        return v;
    }
    float sample(const std::vector<float>& v, float s) const {
        float f = wrapS(s) / ds; int i = (int)f; float u = f - i;
        return mixf(v[wrap(i)], v[wrap(i + 1)], u);
    }

    // ---- barriers / limits -------------------------------------------------
    float leftLimit(float) const { return -14.f; }
    // right limit measured from the centre line.  inLane: car is on the pit-lane
    // side of the pit wall.
    float rightLimit(float u, bool inLane) const {
        using namespace PitC;
        if (u > WALL_X0 && u < WALL_X1) return inLane ? LANE_OUT - 1.2f : WALL_D - 0.9f;
        return 14.f + (LANE_OUT - 1.2f - 14.f) * pitW(u);
    }
    float rightLimitVisual(float u) const { return 14.f + (PitC::LANE_OUT + 0.4f - 14.f) * pitW(u); }
};
