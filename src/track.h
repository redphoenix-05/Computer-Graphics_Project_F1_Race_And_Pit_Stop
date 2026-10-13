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

    }
};
