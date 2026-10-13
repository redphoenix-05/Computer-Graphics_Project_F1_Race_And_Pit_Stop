// ============================================================================
//  world.h  -  Procedural construction of the racing venue
//  Everything static is generated once at start-up into spatially chunked
//  batches (ground / road / pit / markings / environment / equipment / glass).
// ============================================================================
#pragma once
#include "pit.h"

struct World {
    StaticBatch ground, road, pitL, marks, env, hero, glass;
    Mesh hills;
    std::vector<LightSrc> lamps;           // trackside flood lights (pool, nearest ones are used)
    std::vector<LightSrc> garageSpots;     // 5 garage spot lights
    std::vector<vec3> camSpots;            // TV camera platforms
    std::vector<vec3> lensPos;             // pit-light lens centres  (3 clusters x 3 lenses: red, amber, green)
    std::vector<vec3> startLampPos;        // start-light positions
    vec3 towerTop;                         // broadcast camera on the race-control tower
    std::vector<vec3> poles;               // (x, z, radius) of upright obstacles near the pit lane - used for a start-up clearance check
    const Track* T = nullptr;
    float stat = 0;

    static unsigned hashu(unsigned x) { x ^= x >> 16; x *= 0x7feb352dU; x ^= x >> 15; x *= 0x846ca68bU; x ^= x >> 16; return x; }
    static float rnd(unsigned a, unsigned b = 0, unsigned c = 0) { return (hashu((a * 73856093u) ^ (b * 19349663u) ^ (c * 83492791u + 12345u)) & 0xffffff) / 16777215.f; }

    // ---- helpers ---------------------------------------------------------------
    template <class FA, class FB, class FM>
    void strip(StaticBatch& B, int i0, int i1, FA dA, FB dB, float y, FM mf, int nLat) {
        for (int i = i0; i < i1; i++) {
            int a = T->wrap(i), b = T->wrap(i + 1);
            float ua = T->sRel(a * T->ds), ub = T->sRel(b * T->ds);
            Mat m = mf(i);
            MeshBuilder& mb = B.at(T->p[a]);
            for (int j = 0; j < nLat; j++) {
                float f0 = (float)j / nLat, f1 = (float)(j + 1) / nLat;
                float a0 = mixf(dA(ua), dB(ua), f0), a1 = mixf(dA(ua), dB(ua), f1);
                float b0 = mixf(dA(ub), dB(ub), f0), b1 = mixf(dA(ub), dB(ub), f1);
                vec3 p00 = T->p[a] + T->n[a] * a0, p01 = T->p[a] + T->n[a] * a1;
                vec3 p10 = T->p[b] + T->n[b] * b0, p11 = T->p[b] + T->n[b] * b1;
                p00.y = p01.y = p10.y = p11.y = y;
                mb.quadUp(p00, p01, p11, p10, m);
            }
        }
    }
    // vertical wall along the circuit between lateral d0..d0+thick, from y0 to y1
    template <class FD, class FM>
    void wall(StaticBatch& B, int i0, int i1, FD dFn, float thick, float y0, float y1, FM mf, const Mat* topMat = nullptr) {
        for (int i = i0; i < i1; i++) {
            int a = T->wrap(i), b = T->wrap(i + 1);
            float ua = T->sRel(a * T->ds), ub = T->sRel(b * T->ds);
            Mat m = mf(i);
            float da = dFn(ua), db = dFn(ub);
            vec3 pa = T->p[a] + T->n[a] * da, pb = T->p[b] + T->n[b] * db;
            vec3 qa = T->p[a] + T->n[a] * (da + thick), qb = T->p[b] + T->n[b] * (db + thick);
            MeshBuilder& mb = B.at(pa);
            auto V = [&](vec3 p, float y) { p.y = y; return p; };
            vec3 nIn = T->n[a] * -1.f, nOut = T->n[a];
            mb.quadN(V(pa, y0), V(pb, y0), V(pb, y1), V(pa, y1), nIn, m);                 // track-facing face
            mb.quadN(V(qa, y0), V(qb, y0), V(qb, y1), V(qa, y1), nOut, m);                // outer face
            mb.quadN(V(pa, y1), V(pb, y1), V(qb, y1), V(qa, y1), vec3(0, 1, 0), topMat ? *topMat : m);   // top
        }
    }
    static mat4 frame(const vec3& t, const vec3& n, const vec3& o) { return basis(t, vec3(0, 1, 0), n, o); }
    void rectOutline(float x0, float z0, float x1, float z1, float w, const Mat& m) {
        MeshBuilder& mb = marks.at(vec3((x0 + x1) * 0.5f, 0, (z0 + z1) * 0.5f));
        auto Q = [&](float a, float b, float c, float d) { mb.quadUp({a, 0.f, b}, {a, 0.f, d}, {c, 0.f, d}, {c, 0.f, b}, m); };
        Q(x0, z0, x1, z0 + w); Q(x0, z1 - w, x1, z1); Q(x0, z0, x0 + w, z1); Q(x1 - w, z0, x1, z1);
    }
    void digit7(MeshBuilder& mb, const mat4& M, int d, const Mat& m) {
        static const int mask[10] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};
        const float hx[3] = {0.5f, 0.5f, 0.5f};
        (void)hx;
        // a b c d e f g
        struct S { float x, y, w, h; };
        S seg[7] = {{0, 0.5f, 0.5f, 0.09f}, {0.25f, 0.25f, 0.09f, 0.5f}, {0.25f, -0.25f, 0.09f, 0.5f}, {0, -0.5f, 0.5f, 0.09f},
                    {-0.25f, -0.25f, 0.09f, 0.5f}, {-0.25f, 0.25f, 0.09f, 0.5f}, {0, 0, 0.5f, 0.09f}};
        for (int s = 0; s < 7; s++) if (mask[d] & (1 << s))
            mb.box(M * translate(seg[s].x, seg[s].y, 0), vec3(seg[s].w * 0.5f, seg[s].h * 0.5f, 0.012f), m);
    }

    // ---- main build --------------------------------------------------------------
    void build(const Track& tr) {
        T = &tr;
        buildGround(); buildRoad(); buildBarriers(); buildPit(); buildStands(); buildFurniture(); buildScenery(); buildHills();
        for (StaticBatch* b : {&ground, &road, &pitL, &marks, &env, &hero, &glass}) { b->finalize(); stat += (float)b->tris; }
    }

    void buildGround() {
        Mat g1 = MT(vec3(0.20f, 0.44f, 0.15f), 0.f, 8.f, 0.f, 0.8f), g2 = MT(vec3(0.24f, 0.48f, 0.17f), 0.f, 8.f, 0.f, 0.8f);
        float cell = 55.f; int x0 = -30, x1 = 30, z0 = -20, z1 = 30;
        for (int ix = x0; ix < x1; ix++)
            for (int iz = z0; iz < z1; iz++) {
                float x = ix * cell, z = iz * cell;
                float r = rnd(ix + 100, iz + 100);
                Mat m = r > 0.5f ? g1 : g2;
                m.c = m.c * (0.9f + 0.2f * rnd(ix + 7, iz + 9));
                ground.at(x, z).quadUp({x, -0.25f, z}, {x, -0.25f, z + cell}, {x + cell, -0.25f, z + cell}, {x + cell, -0.25f, z}, m);
            }
        // mown grass bands alongside the circuit
        Mat ma = MT(vec3(0.27f, 0.55f, 0.20f), 0.f, 8.f, 0.f, 0.6f), mb2 = MT(vec3(0.22f, 0.47f, 0.17f), 0.f, 8.f, 0.f, 0.6f);
        for (int side = -1; side <= 1; side += 2) {
            auto d0 = [&](float u) { return side < 0 ? -42.f : (14.f + 27.f * pitW(u)); };
            auto d1 = [&](float u) { return side < 0 ? -14.f : 42.f + 10.f * pitW(u); };
            strip(ground, 0, T->N, d0, d1, -0.2f, [&](int i) { return ((i / 7) & 1) ? ma : mb2; }, 1);
        }
    }

    void build(const Track& tr) { T = &tr; buildGround(); }
};
