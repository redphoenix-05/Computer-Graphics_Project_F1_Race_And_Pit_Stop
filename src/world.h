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

    void buildRoad() {
        int N = T->N;
        Mat asph = MT(vec3(0.21f, 0.21f, 0.23f), 0.15f, 20.f, 0.f, 0.55f);
        Mat asph2 = MT(vec3(0.19f, 0.19f, 0.21f), 0.15f, 20.f, 0.f, 0.55f);
        strip(road, 0, N, [](float) { return -7.f; }, [](float) { return 7.f; }, 0.f,
              [&](int i) { return ((i / 4) & 1) ? asph : asph2; }, 8);
        // run-off areas: paved / gravel on the outside of corners
        Mat pave = MT(vec3(0.42f, 0.43f, 0.46f), 0.05f, 10.f, 0.f, 0.5f), grav = MT(vec3(0.70f, 0.64f, 0.48f), 0.f, 8.f, 0.f, 0.9f);
        for (int side = -1; side <= 1; side += 2) {
            auto d0 = [&](float) { return side < 0 ? -14.f : 7.f; };
            auto d1 = [&](float u) { return side < 0 ? -7.f : mixf(14.f, PitC::LANE_IN - 0.1f, pitW(u)); };
            strip(road, 0, N, d0, d1, -0.01f, [&](int i) {
                float k = T->k[i];
                bool outer = (std::fabs(k) > 1.f / 170.f) && ((k > 0 ? -1 : 1) == side);
                return outer ? grav : pave; }, 4);
        }
        // kerbs on the inside of corners (red/white) + edge lines + start/finish
        Mat red = MT(vec3(0.85f, 0.08f, 0.08f), 0.2f, 20.f), wht = MT(vec3(0.92f, 0.92f, 0.92f), 0.2f, 20.f);
        Mat line = MT(vec3(0.9f, 0.9f, 0.9f), 0.1f, 10.f);
        for (int side = -1; side <= 1; side += 2) {
            auto d0 = [&](float) { return side < 0 ? -8.2f : 7.f; };
            auto d1 = [&](float) { return side < 0 ? -7.f : 8.2f; };
            for (int i = 0; i < N; i++) {
                float k = T->k[i];
                bool inner = std::fabs(k) > 1.f / 240.f && ((k > 0 ? 1 : -1) == side);
                if (!inner) continue;
                strip(marks, i, i + 1, d0, d1, 0.01f, [&](int) { return (i & 1) ? red : wht; }, 1);
            }
            auto l0 = [&](float) { return side < 0 ? -6.9f : 6.5f; };
            auto l1 = [&](float) { return side < 0 ? -6.5f : 6.9f; };
            strip(marks, 0, N, l0, l1, 0.01f, [&](int) { return line; }, 1);
        }
        // start / finish chequered line (2 rows) across track and pit lane, grid slots
        Mat blk = MT(vec3(0.03f), 0.1f, 10.f);
        for (int row = 0; row < 2; row++)
            for (int c = 0; c < 28; c++) {
                float z0 = -7.f + c * 0.5f + (row == 0 ? 0.f : 0.f);
                float x0 = -0.5f + row * 0.5f;
                const Mat& m = ((c + row) & 1) ? blk : wht;
                marks.at(0, 0).quadUp({x0, 0.f, z0}, {x0, 0.f, z0 + 0.5f}, {x0 + 0.5f, 0.f, z0 + 0.5f}, {x0 + 0.5f, 0.f, z0}, m);
            }
        for (int k = 0; k < 5; k++) {   // grid slots
            float x = -10.f - 9.f * k, z = (k % 2 == 0) ? -2.6f : 2.6f;
            rectOutline(x - 3.2f, z - 1.2f, x + 0.6f, z + 1.2f, 0.14f, wht);
        }
    }

    void buildBarriers() {
        int N = T->N;
        Mat conc = MT(vec3(0.75f, 0.75f, 0.74f), 0.1f, 12.f, 0.f, 0.3f), concTop = MT(vec3(0.85f, 0.85f, 0.84f), 0.1f, 12.f);
        Mat ads[6] = {MT(vec3(0.85f, 0.1f, 0.1f), 0.2f, 16.f), MT(vec3(0.1f, 0.3f, 0.8f), 0.2f, 16.f), MT(vec3(0.95f, 0.85f, 0.1f), 0.2f, 16.f),
                      MT(vec3(0.1f, 0.65f, 0.35f), 0.2f, 16.f), MT(vec3(0.95f, 0.95f, 0.95f), 0.2f, 16.f), MT(vec3(0.95f, 0.5f, 0.05f), 0.2f, 16.f)};
        Mat tire = MT(vec3(0.04f, 0.04f, 0.045f), 0.1f, 12.f), tireW = MT(vec3(0.9f, 0.9f, 0.9f), 0.1f, 12.f), tireR = MT(vec3(0.85f, 0.1f, 0.1f), 0.1f, 12.f);
        Mat post = MT(vec3(0.25f, 0.27f, 0.3f), 0.5f, 40.f), net = MT(vec3(0.12f, 0.14f, 0.16f), 0.1f, 10.f);
        for (int side = -1; side <= 1; side += 2) {
            auto lim = [&](float u) { return side < 0 ? -14.f : T->rightLimitVisual(u); };
            for (int i = 0; i < N; i++) {
                float u = T->sRel(i * T->ds);
                // skip right barrier inside the pit zone (pit wall / garages take over) except at the ramps
                bool inPitZone = (side > 0 && u > PitC::WALL_X0 && u < PitC::WALL_X1);
                if (inPitZone) continue;
                float k = T->k[i];
                bool outerCorner = std::fabs(k) > 1.f / 190.f && ((k > 0 ? -1 : 1) == side);
                float d = (side < 0) ? -14.f : T->rightLimitVisual(u);
                int a = i;
                vec3 pa = T->p[a] + T->n[a] * d;
                if (outerCorner) {
                    // tyre walls: stacks every 1.2 m (every second sample gets one stack)
                    if (i % 2 == 0) {
                        vec3 c = pa + T->n[a] * (side * 0.6f);
                        float rr = rnd(i, side + 3);
                        MeshBuilder& mb = env.at(c);
                        for (int lv = 0; lv < 3; lv++) {
                            const Mat& m = (lv == 1) ? ((i / 8) % 3 == 0 ? tireR : tireW) : tire;
                            mb.cyl(translate(c.x, 0.17f + lv * 0.34f, c.z), 0.52f, 0.52f, 0.17f, 8, m, lv == 2, false);
                        }
                        if (rr > 0.5f) {
                            vec3 c2 = pa + T->n[a] * (side * 1.65f);
                            for (int lv = 0; lv < 2; lv++)
                                mb.cyl(translate(c2.x, 0.17f + lv * 0.34f, c2.z), 0.52f, 0.52f, 0.17f, 8, tire, lv == 1, false);
                        }
                    }
                } else {
                    wall(env, i, i + 1, lim, 0.5f, 0.0f, 1.0f, [&](int) { return conc; }, &concTop);
                    // advertising panels on the track-facing side, one per 6 m
                    if (i % 3 == 0 && std::fabs(k) < 1.f / 300.f) {
                        int ai = (int)(rnd(i / 3, side + 5) * 6) % 6;
                        float dd = lim(u) - 0.02f * side * 0 - 0.03f;
                        vec3 p0 = T->p[a] + T->n[a] * dd, p1 = T->pos(i * T->ds + 5.6f, 0) ; (void)p1;
                        int b2 = T->wrap(i + 3);
                        vec3 q1 = T->p[b2] + T->n[b2] * (side < 0 ? -14.03f : T->rightLimitVisual(T->sRel(b2 * T->ds)) - 0.03f);
                        vec3 q0 = T->p[a] + T->n[a] * (side < 0 ? -14.03f : T->rightLimitVisual(u) - 0.03f);
                        MeshBuilder& mb = env.at(q0);
                        vec3 nn = T->n[a] * (float)-1;
                        auto V = [&](vec3 p, float y) { p.y = y; return p; };
                        mb.quadN(V(q0, 0.12f), V(q1, 0.12f), V(q1, 0.88f), V(q0, 0.88f), nn, ads[ai]);
                        vec3 m0 = mix(q0, q1, 0.3f), m1 = mix(q0, q1, 0.7f);
                        mb.quadN(V(m0, 0.30f), V(m1, 0.30f), V(m1, 0.70f), V(m0, 0.70f), nn, ads[(ai + 4) % 6]);
                        (void)p0;
                    }
                }
                // catch fence behind the barrier (posts every 6 m)
                if (i % 3 == 0) {
                    vec3 c = pa + T->n[a] * (side * 1.4f);
                    env.at(c).boxAt({c.x, 2.2f, c.z}, {0.06f, 2.2f, 0.06f}, post);
                }
            }
            // translucent fence net
            auto fd = [&](float u) { return side < 0 ? -15.4f : T->rightLimitVisual(u) + 1.4f; };
            for (int i = 0; i < N; i += 1) {
                float u = T->sRel(i * T->ds);
                if (side > 0 && u > PitC::WALL_X0 - 5.f && u < PitC::WALL_X1 + 5.f) continue;
                int a = T->wrap(i), b = T->wrap(i + 1);
                vec3 pa = T->p[a] + T->n[a] * fd(u), pb = T->p[b] + T->n[b] * fd(T->sRel(b * T->ds));
                auto V = [&](vec3 p, float y) { p.y = y; return p; };
                glass.at(pa).quadN(V(pa, 0.8f), V(pb, 0.8f), V(pb, 4.3f), V(pa, 4.3f), T->n[a] * (float)-side, net);
            }
            (void)lim;
        }
    }

    void buildPit() {
        using namespace PitC;
        // pit-lane road (entry ramp, lane, exit ramp)
        Mat lane = MT(vec3(0.30f, 0.30f, 0.32f), 0.15f, 20.f, 0.f, 0.45f), lane2 = MT(vec3(0.27f, 0.27f, 0.29f), 0.15f, 20.f, 0.f, 0.45f);
        int iA = (int)((T->L + ENTRY_X0) / T->ds), iB = (int)((EXIT_X1 + 40.f) / T->ds);
        strip(pitL, iA, T->N + iB, [](float u) { return pitInner(u); }, [](float u) { return pitOuter(u); }, 0.f,
              [&](int i) { return ((i / 4) & 1) ? lane : lane2; }, 6);
        // painted lines: lane divider, speed-limit edge lines, entry & exit lines
        Mat wht = MT(vec3(0.92f), 0.1f, 10.f), yel = MT(vec3(0.95f, 0.8f, 0.1f), 0.1f, 10.f);
        int jA = (int)((T->L + WALL_X0 + 10.f) / T->ds), jB = (int)((WALL_X1 - 10.f) / T->ds);
        strip(marks, jA, T->N + jB, [](float) { return 19.4f; }, [](float) { return 19.65f; }, 0.01f, [&](int) { return wht; }, 1);
        strip(marks, jA, T->N + jB, [](float) { return LANE_IN + 0.15f; }, [](float) { return LANE_IN + 0.4f; }, 0.01f, [&](int) { return yel; }, 1);
        strip(marks, jA, T->N + jB, [](float) { return LANE_OUT - 0.4f; }, [](float) { return LANE_OUT - 0.15f; }, 0.01f, [&](int) { return wht; }, 1);
        for (int k = 0; k < NBOX; k++) {
            float bx = BOX_X[k];
            rectOutline(bx - 3.6f, WORK_D - 2.0f, bx + 3.6f, WORK_D + 2.0f, 0.12f, wht);
            if (k == PLAYER_BOX) {
                // stop marks for the player's box
                MeshBuilder& mb = marks.at(vec3(bx, 0, WORK_D));
                mb.quadUp({bx + 1.15f, 0.f, WORK_D - 1.3f}, {bx + 1.15f, 0.f, WORK_D + 1.3f}, {bx + 1.35f, 0.f, WORK_D + 1.3f}, {bx + 1.35f, 0.f, WORK_D - 1.3f}, yel);
                mb.quadUp({bx - 0.1f, 0.f, WORK_D - 0.1f}, {bx - 0.1f, 0.f, WORK_D + 0.1f}, {bx + 0.1f, 0.f, WORK_D + 0.1f}, {bx + 0.1f, 0.f, WORK_D - 0.1f}, yel);
            }
        }
        // pit-lane entry / exit cross lines + chequered line over the pit lane at x = 0
        marks.at(ENTRY_X1, 15.f).quadUp({ENTRY_X1 - 5.f, 0.f, 12.9f}, {ENTRY_X1 - 5.f, 0.f, 27.f}, {ENTRY_X1 - 4.6f, 0.f, 27.f}, {ENTRY_X1 - 4.6f, 0.f, 12.9f}, wht);
        Mat blk = MT(vec3(0.03f), 0.1f, 10.f);
        for (int row = 0; row < 2; row++)
            for (int c = 0; c < 28; c++) {
                float z0 = LANE_IN + c * 0.5f; float x0 = -0.5f + row * 0.5f;
                if (z0 + 0.5f > LANE_OUT) break;
                marks.at(0, 20.f).quadUp({x0, 0.f, z0}, {x0, 0.f, z0 + 0.5f}, {x0 + 0.5f, 0.f, z0 + 0.5f}, {x0 + 0.5f, 0.f, z0}, ((c + row) & 1) ? blk : wht);
            }
        // ---- pit wall -----------------------------------------------------------
        Mat conc = MT(vec3(0.72f, 0.72f, 0.71f), 0.1f, 12.f, 0.f, 0.3f), top = MT(vec3(0.85f), 0.1f, 12.f);
        int wA = (int)((T->L + WALL_X0) / T->ds), wB = (int)(WALL_X1 / T->ds);
        wall(env, wA, T->N + wB, [](float) { return WALL_D - 0.3f; }, 0.7f, 0.0f, 1.05f, [&](int) { return conc; }, &top);
        // pit-wall command posts: desk, canopy, monitors (one per team)
        for (int k = 0; k < NBOX; k++) {
            float bx = BOX_X[k];
            Mat team = MT(LIVERIES[k].main, 0.5f, 40.f);
            MeshBuilder& e = hero.at(vec3(bx, 0, 12.6f));
            e.boxAt({bx, 1.45f, 12.35f}, {1.4f, 0.4f, 0.35f}, MT(vec3(0.12f, 0.12f, 0.14f), 0.4f, 30.f));   // console
            for (int m = -1; m <= 1; m += 2)
                e.boxAt({bx + m * 0.7f, 1.95f, 12.2f}, {0.45f, 0.28f, 0.03f}, MT(vec3(0.3f, 0.7f, 0.95f), 0.6f, 60.f, 0.9f));
            for (int px = -1; px <= 1; px += 2) { e.tube({bx + px * 1.5f, 1.05f, 12.5f}, {bx + px * 1.5f, 3.3f, 12.5f}, 0.04f, 0.04f, 6, MT(vec3(0.5f))); poles.push_back(vec3(bx + px * 1.5f, 12.5f, 0.04f)); }
            e.boxAt({bx, 3.4f, 12.5f}, {1.9f, 0.05f, 1.1f}, team);
            e.boxAt({bx, 3.46f, 12.5f}, {1.9f, 0.012f, 0.4f}, MT(vec3(0.95f), 0.3f, 20.f));
        }
        // amber warning beacons on the pit wall next to the player's box
        // (drawn dynamically in drawLamps) - static housings:
        float bxp = BOX_X[PLAYER_BOX];
        for (int s = -1; s <= 1; s += 2) hero.at(vec3(bxp + s * 6.f, 0, 12.5f)).boxAt({bxp + s * 6.f, 1.25f, 12.65f}, {0.15f, 0.22f, 0.15f}, MT(vec3(0.15f), 0.3f, 30.f));
        // garage frontage fence / low wall between ramps and garages
        Mat fence = MT(vec3(0.55f, 0.57f, 0.6f), 0.4f, 30.f);
        for (int k = 0; k < 2; k++) {
            float xa = (k == 0) ? WALL_X0 : BOX_X[NBOX - 1] + 13.5f, xb = (k == 0) ? BOX_X[0] - 13.5f : WALL_X1;
            env.at(vec3((xa + xb) * 0.5f, 0, LANE_OUT + 0.3f)).boxAt({(xa + xb) * 0.5f, 0.6f, LANE_OUT + 0.3f}, {(xb - xa) * 0.5f, 0.6f, 0.25f}, MT(vec3(0.7f), 0.1f, 12.f));
            (void)fence;
        }
        // ---- garages -------------------------------------------------------------
        for (int k = 0; k < NBOX; k++) buildGarage(k);
        // ---- race-control tower + timing tower ---------------------------------------
        buildTowers();
        // ---- pit signage: entry / exit boards and 100 km/h limit signs ----------------
        Mat yb = MT(vec3(0.97f, 0.82f, 0.08f), 0.2f, 20.f), blkm = MT(vec3(0.04f), 0.1f, 10.f);
        for (int q = 0; q < 2; q++) {
            // boards stand on the track side, outside the pit-lane ramps (never in the car path)
            float x = (q == 0) ? ENTRY_X0 - 15.f : EXIT_X1 + 25.f;
            float z = 10.8f;
            MeshBuilder& e = env.at(vec3(x, 0, z));
            e.tube({x, 0, z + 2.5f}, {x, 3.6f, z + 2.5f}, 0.07f, 0.07f, 6, MT(vec3(0.4f)));
            e.tube({x, 0, z - 2.5f}, {x, 3.6f, z - 2.5f}, 0.07f, 0.07f, 6, MT(vec3(0.4f)));
            poles.push_back(vec3(x, z + 2.5f, 0.07f)); poles.push_back(vec3(x, z - 2.5f, 0.07f));
            e.boxAt({x, 3.4f, z}, {0.08f, 0.9f, 2.9f}, yb);
            for (int c = 0; c < 3; c++) {      // arrow chevrons
                float zc = z + (c - 1) * 1.7f;
                e.box(translate(x - 0.1f, 3.4f, zc) * rotX(0.6f), vec3(0.02f, 0.1f, 0.5f), blkm);
                e.box(translate(x - 0.1f, 3.4f, zc) * rotX(-0.6f), vec3(0.02f, 0.1f, 0.5f), blkm);
            }
        }
        for (int q = 0; q < 2; q++) {   // speed limit "100" signs
            float x = (q == 0) ? ENTRY_X1 + 22.f : EXIT_X0 - 20.f;
            float z = WALL_D + 0.05f;                      // on top of the pit wall, 3+ m from the fast-lane path
            MeshBuilder& e = hero.at(vec3(x, 0, z));
            e.tube({x, 1.0f, z}, {x, 3.0f, z}, 0.05f, 0.05f, 6, MT(vec3(0.4f)));
            poles.push_back(vec3(x, z, 0.05f));
            e.cyl(translate(x - 0.06f, 3.45f, z) * rotZ(PI_F * 0.5f), 0.75f, 0.75f, 0.03f, 20, MT(vec3(0.85f, 0.08f, 0.08f), 0.3f, 30.f));
            e.cyl(translate(x - 0.09f, 3.45f, z) * rotZ(PI_F * 0.5f), 0.58f, 0.58f, 0.03f, 20, MT(vec3(0.97f), 0.3f, 30.f));
            mat4 D = translate(x - 0.13f, 3.45f, z) * rotY(-PI_F * 0.5f) * scale(0.36f);
            digit7(e, D * translate(-0.78f, 0, 0), 1, MT(vec3(0.03f), 0.1f, 10.f));
            digit7(e, D * translate(0.0f, 0, 0), 0, MT(vec3(0.03f), 0.1f, 10.f));
            digit7(e, D * translate(0.78f, 0, 0), 0, MT(vec3(0.03f), 0.1f, 10.f));
        }
        // ---- pit-light gantry (static structure) -------------------------------------
        float gx = BOX_X[PLAYER_BOX] + 9.f;
        Mat steel = MT(vec3(0.35f, 0.36f, 0.38f), 0.6f, 50.f), housing = MT(vec3(0.05f), 0.3f, 30.f);
        MeshBuilder& g = hero.at(vec3(gx, 0, 20.f));
        g.tube({gx, 0, 12.9f}, {gx, 5.8f, 12.9f}, 0.14f, 0.14f, 8, steel);
        g.tube({gx, 0, 26.6f}, {gx, 5.8f, 26.6f}, 0.14f, 0.14f, 8, steel);
        poles.push_back(vec3(gx, 12.9f, 0.14f)); poles.push_back(vec3(gx, 26.6f, 0.14f));
        g.boxAt({gx, 5.9f, 19.75f}, {0.25f, 0.16f, 7.1f}, steel);
        float zs[3] = {13.4f, 19.75f, 26.2f}; float ys[3] = {3.4f, 5.1f, 3.4f};
        lensPos.clear();
        for (int c = 0; c < 3; c++) {
            g.boxAt({gx, ys[c], zs[c]}, {0.18f, 0.62f, 0.26f}, housing);
            for (int l = 0; l < 3; l++) lensPos.push_back(vec3(gx - 0.2f, ys[c] + 0.4f - l * 0.4f, zs[c]));   // red, amber, green
        }
    }

    void buildGarage(int k) {
        using namespace PitC;
        float cx = BOX_X[k], z0 = LANE_OUT + 0.4f, z1 = z0 + GARAGE_DEPTH, hw = GARAGE_W * 0.5f, H = 6.4f;
        const Livery& L = LIVERIES[k];
        Mat wallM = MT(vec3(0.80f, 0.80f, 0.82f), 0.15f, 20.f, 0.f, 0.15f), team = MT(L.main, 0.5f, 40.f), sec = MT(L.second, 0.5f, 40.f);
        Mat floorM = MT(vec3(0.58f, 0.58f, 0.6f), 0.35f, 40.f, 0.f, 0.3f), dark = MT(vec3(0.07f, 0.07f, 0.08f), 0.4f, 30.f);
        MeshBuilder& e = env.at(vec3(cx, 0, (z0 + z1) * 0.5f));
        MeshBuilder& h = hero.at(vec3(cx, 0, (z0 + z1) * 0.5f));
        // floor slab (apron + interior), walls (open front), roof, fascia
        e.boxAt({cx, 0.0f, (z0 - 0.4f + z1) * 0.5f}, {hw, 0.05f, (z1 - z0 + 0.4f) * 0.5f}, floorM);
        e.boxAt({cx, H * 0.5f, z1 + 0.2f}, {hw, H * 0.5f, 0.2f}, wallM);                      // back wall
        for (int s = -1; s <= 1; s += 2) e.boxAt({cx + s * hw, H * 0.5f, (z0 + z1) * 0.5f}, {0.25f, H * 0.5f, (z1 - z0) * 0.5f}, wallM);
        e.boxAt({cx, H + 0.2f, (z0 + z1) * 0.5f + 0.7f}, {hw + 0.3f, 0.2f, (z1 - z0) * 0.5f + 1.4f}, MT(vec3(0.5f, 0.52f, 0.55f), 0.3f, 30.f));   // roof + canopy
        e.boxAt({cx, H - 0.6f, z0 - 0.5f}, {hw, 0.9f, 0.18f}, team);                         // fascia
        e.boxAt({cx, H - 0.1f, z0 - 0.5f}, {hw, 0.08f, 0.2f}, sec);
        e.boxAt({cx, H - 0.6f, z0 - 0.7f}, {hw * 0.55f, 0.35f, 0.03f}, MT(vec3(1), 0.3f, 20.f, 0.9f));   // lit team sign
        // back wall decoration (team colour panel + emissive logo block)
        e.boxAt({cx, 3.2f, z1 - 0.05f}, {hw - 0.6f, 2.1f, 0.05f}, team);
        h.boxAt({cx, 3.4f, z1 - 0.12f}, {3.2f, 0.8f, 0.04f}, MT(sec.c, 0.4f, 40.f, 0.55f));
        // interior: tool cabinets, monitors, tyre racks, carts
        for (int s = -1; s <= 1; s += 2) {
            h.boxAt({cx + s * (hw - 1.3f), 0.9f, z1 - 1.2f}, {0.9f, 0.9f, 0.6f}, MT(vec3(0.75f, 0.1f, 0.1f), 0.5f, 40.f));
            h.boxAt({cx + s * (hw - 3.2f), 0.9f, z1 - 1.2f}, {0.8f, 0.9f, 0.6f}, MT(vec3(0.15f), 0.5f, 40.f));
            h.boxAt({cx + s * (hw - 1.1f), 2.4f, z1 - 0.5f}, {0.9f, 0.5f, 0.05f}, MT(vec3(0.3f, 0.75f, 1.0f), 0.8f, 80.f, 0.9f));   // monitor
            // tyre rack with tyres
            float rx = cx + s * (hw - 2.0f), rz = z0 + 4.0f;
            h.boxAt({rx, 0.12f, rz}, {0.7f, 0.06f, 1.9f}, dark);
            for (int q = 0; q < 4; q++)
                for (int lv = 0; lv < 3; lv++)
                    h.cyl(translate(rx, 0.18f + lv * 0.36f + 0.06f, rz - 1.4f + q * 0.95f) * rotX(PI_F * 0.5f) * rotY(0), 0.34f, 0.34f, 0.17f, 12,
                          lv == 1 ? MT(COMPOUNDS[(k + q) % 3].col, 0.1f, 10.f) : MT(vec3(0.05f), 0.2f, 20.f));
        }
        h.boxAt({cx, 0.55f, z1 - 3.0f}, {1.4f, 0.55f, 0.45f}, MT(vec3(0.2f), 0.5f, 40.f));        // equipment cart
        h.boxAt({cx + 0.6f, 0.15f, z0 + 3.6f}, {0.5f, 0.15f, 0.5f}, MT(vec3(0.9f, 0.8f, 0.1f), 0.4f, 30.f));
        // ceiling light strips (emissive) and light spot
        for (int q = -1; q <= 1; q += 2) e.boxAt({cx + q * 4.0f, H - 0.08f, (z0 + z1) * 0.5f}, {0.25f, 0.05f, 5.0f}, MT(vec3(1.0f, 0.97f, 0.9f), 0.1f, 10.f, 1.0f));
        LightSrc s; s.pos = vec3(cx, H - 0.5f, (z0 + z1) * 0.5f - 1.0f); s.dir = normalize(vec3(0, -1, -0.5f));
        s.col = vec3(1.7f, 1.6f, 1.45f); s.radius = 30.f; s.cosIn = 0.62f; s.cosOut = 0.30f; s.spot = true;
        garageSpots.push_back(s);
        // roof equipment
        e.boxAt({cx - 6.f, H + 0.8f, z1 - 4.f}, {1.2f, 0.5f, 0.9f}, MT(vec3(0.6f), 0.3f, 30.f));
        e.boxAt({cx + 5.f, H + 0.7f, z1 - 6.f}, {1.0f, 0.4f, 0.8f}, MT(vec3(0.55f), 0.3f, 30.f));
    }

    void buildTowers() {
        using namespace PitC;
        MeshBuilder& e = env.at(vec3(0, 0, 55.f));
        Mat conc = MT(vec3(0.62f, 0.64f, 0.68f), 0.2f, 20.f, 0.f, 0.1f);
        float tx = 20.f, tz = 56.f;
        // race-control tower: stacked floors with glass bands and emissive window strips
        for (int f = 0; f < 7; f++) {
            float y = 2.0f + f * 4.2f;
            e.boxAt({tx, y, tz}, {17.f, 0.35f, 8.5f}, conc);
            e.boxAt({tx, y + 2.1f, tz - 8.4f}, {16.8f, 1.6f, 0.06f}, MT(vec3(0.25f, 0.5f, 0.75f), 0.9f, 100.f, f > 1 ? 0.25f : 0.f));
        }
        e.boxAt({tx - 17.f, 15.f, tz}, {0.5f, 15.f, 8.5f}, conc);
        e.boxAt({tx + 17.f, 15.f, tz}, {0.5f, 15.f, 8.5f}, conc);
        e.boxAt({tx, 15.f, tz + 8.5f}, {17.f, 15.f, 0.4f}, conc);
        e.boxAt({tx, 31.5f, tz}, {18.f, 0.5f, 9.f}, MT(vec3(0.4f), 0.3f, 30.f));
        e.boxAt({tx, 33.f, tz - 2.f}, {8.f, 1.1f, 4.f}, conc);
        e.boxAt({tx, 33.1f, tz - 6.05f}, {7.8f, 0.9f, 0.05f}, MT(vec3(0.25f, 0.55f, 0.8f), 0.9f, 100.f, 0.35f));
        e.tube({tx + 6.f, 34.f, tz}, {tx + 6.f, 46.f, tz}, 0.12f, 0.06f, 6, MT(vec3(0.7f)));
        e.tube({tx - 6.f, 34.f, tz}, {tx - 6.f, 42.f, tz}, 0.10f, 0.05f, 6, MT(vec3(0.7f)));
        // big LED board on the tower front
        e.boxAt({tx, 26.f, tz - 8.7f}, {10.f, 2.2f, 0.12f}, MT(vec3(0.1f, 0.12f, 0.15f), 0.4f, 40.f));
        for (int c = 0; c < 10; c++)
            e.boxAt({tx - 9.f + c * 2.0f, 26.f, tz - 8.82f}, {0.75f, 1.9f, 0.04f}, MT(LIVERIES[c % 5].main * 0.9f + vec3(0.1f), 0.3f, 30.f, 0.75f));
        towerTop = vec3(tx, 34.5f, tz - 6.f);
        // timing tower at x = -110
        float mx = -120.f, mz = 48.f;
        e.boxAt({mx, 20.f, mz}, {3.f, 20.f, 3.f}, conc);
        for (int r = 0; r < 5; r++) {
            e.boxAt({mx, 26.f + r * 2.4f, mz - 3.05f}, {2.6f, 1.0f, 0.05f}, MT(vec3(0.05f), 0.3f, 30.f));
            e.boxAt({mx - 1.8f, 26.f + r * 2.4f, mz - 3.12f}, {0.5f, 0.55f, 0.03f}, MT(LIVERIES[r].main, 0.3f, 30.f, 0.85f));
            e.boxAt({mx + 0.3f, 26.f + r * 2.4f, mz - 3.12f}, {1.7f, 0.12f, 0.03f}, MT(vec3(0.95f), 0.3f, 30.f, 0.9f));
        }
        e.boxAt({mx, 41.f, mz}, {3.6f, 0.5f, 3.6f}, MT(vec3(0.3f), 0.3f, 30.f));
        camSpots.push_back(vec3(mx, 41.8f, mz - 2.f));
    }

    void buildStands() {
        struct St { float x, z, len; int rows; bool fixedSide; int side; float dist; };
        std::vector<St> list = {
            {0.f, 0.f, 520.f, 14, true, -1, 20.f}, {560.f, 60.f, 150.f, 10, false, 0, 22.f}, {600.f, 250.f, 120.f, 10, false, 0, 22.f},
            {300.f, 415.f, 200.f, 9, false, 0, 22.f}, {-270.f, 480.f, 110.f, 9, false, 0, 22.f}, {-420.f, 240.f, 120.f, 8, false, 0, 22.f},
            {-580.f, 80.f, 100.f, 8, false, 0, 22.f}};
        int idx = 0;
        for (auto& st : list) {
            Proj pr = T->project(vec3(st.x, 0, st.z), -1);
            int side = st.side;
            if (!st.fixedSide) {   // choose the side with more free space
                float ca = T->distToTrack(T->p[pr.i].x + T->n[pr.i].x * 90.f, T->p[pr.i].z + T->n[pr.i].z * 90.f);
                float cb = T->distToTrack(T->p[pr.i].x - T->n[pr.i].x * 90.f, T->p[pr.i].z - T->n[pr.i].z * 90.f);
                side = (ca >= cb) ? 1 : -1;
            }
            buildStand(pr.s, st.len, side, st.dist, st.rows, idx++);
        }
    }

    void buildStand(float sc, float len, int side, float dist, int rows, int id) {
        Mat conc = MT(vec3(0.62f, 0.62f, 0.64f), 0.1f, 12.f, 0.f, 0.2f);
        Mat seatM[3] = {MT(vec3(0.15f, 0.3f, 0.75f), 0.2f, 20.f), MT(vec3(0.8f, 0.12f, 0.12f), 0.2f, 20.f), MT(vec3(0.9f, 0.9f, 0.9f), 0.2f, 20.f)};
        Mat roofM = MT(vec3(0.85f, 0.85f, 0.88f), 0.4f, 40.f), pole = MT(vec3(0.4f), 0.5f, 40.f);
        float segL = 6.f; int nseg = (int)(len / segL);
        for (int sg = 0; sg < nseg; sg++) {
            float s = sc - len * 0.5f + (sg + 0.5f) * segL;
            vec3 t = T->tangent(s), n = T->normal(s), up(0, 1, 0);
            vec3 o = n * (float)side;
            vec3 c = T->pos(s, side * dist);
            if (T->distToTrack(c.x + o.x * 12.f, c.z + o.z * 12.f) < 12.f) continue;     // something in the way
            mat4 M = basis(t * (float)side, up, o, c);
            MeshBuilder& mb = env.at(c);
            // concrete terraces + coloured seats + spectators
            for (int r = 0; r < rows; r++) {
                float y = 0.55f * (r + 1), zc = 0.5f + r * 0.9f;
                mb.box(M * translate(0, y * 0.5f, zc + 0.45f), vec3(segL * 0.5f, y * 0.5f, 0.45f), conc);
                const Mat& sm = seatM[(id + sg / 3 + r / 4) % 3];
                mb.box(M * translate(0, y + 0.1f, zc + 0.2f), vec3(segL * 0.5f, 0.1f, 0.25f), sm);
                for (int q = 0; q < 6; q++) {
                    float rr = rnd(id * 1000 + sg, r, q);
                    if (rr < 0.35f) continue;
                    float cx = -segL * 0.5f + (q + 0.5f) * 1.0f;
                    vec3 col(0.2f + 0.8f * rnd(sg, r, q + 50), 0.2f + 0.8f * rnd(sg, r, q + 90), 0.2f + 0.8f * rnd(sg + 7, r, q + 20));
                    mb.box(M * translate(cx, y + 0.55f, zc + 0.35f), vec3(0.2f, 0.4f, 0.17f), MT(col, 0.05f, 8.f));
                    mb.box(M * translate(cx, y + 1.05f, zc + 0.35f), vec3(0.1f, 0.1f, 0.1f), MT(vec3(0.85f, 0.65f, 0.5f) * (0.7f + 0.4f * rnd(sg, r, q + 3)), 0.05f, 8.f));
                }
            }
            // roof over the upper rows, supported by posts every 3 segments
            float top = 0.55f * rows;
            if (rows >= 9) {
                mb.box(M * translate(0, top + 4.2f, rows * 0.9f - 3.2f), vec3(segL * 0.5f, 0.12f, 4.6f), roofM);
                if (sg % 3 == 0) {
                    mb.box(M * translate(-segL * 0.5f, top * 0.5f + 2.1f, rows * 0.9f - 6.5f), vec3(0.1f, top * 0.5f + 2.1f, 0.1f), pole);
                    mb.box(M * translate(-segL * 0.5f, top * 0.5f + 2.1f, rows * 0.9f + 0.5f), vec3(0.1f, top * 0.5f + 2.1f, 0.1f), pole);
                }
            }
            // flags on top
            if (sg % 4 == 0) {
                vec3 base = M.point({0, top, rows * 0.9f + 1.2f});
                MeshBuilder& f = env.at(base);
                f.tube(base, base + vec3(0, 7.f, 0), 0.05f, 0.04f, 6, pole);
                vec3 col = LIVERIES[(id + sg) % 5].main;
                vec3 p0 = base + vec3(0, 7.f, 0), p1 = base + vec3(0, 5.8f, 0), dirw = M.dir({1, 0, 0}) * 1.6f;
                f.quadN(p0, p0 + dirw, p1 + dirw, p1, o, MT(col, 0.1f, 10.f, 0.1f));
            }
            // front hoarding
            mb.box(M * translate(0, 0.5f, -0.1f), vec3(segL * 0.5f, 0.5f, 0.05f), MT(LIVERIES[(sg / 2 + id) % 5].main, 0.3f, 30.f));
        }
    }

    void buildFurniture() {
        int N = T->N;
        Mat pole = MT(vec3(0.45f, 0.46f, 0.5f), 0.6f, 50.f), headM = MT(vec3(1.0f, 0.97f, 0.88f), 0.2f, 20.f, 1.0f), hm = MT(vec3(0.12f), 0.3f, 30.f);
        Mat white = MT(vec3(0.95f), 0.3f, 30.f), orange = MT(vec3(0.95f, 0.45f, 0.05f), 0.3f, 30.f);
        // start gantry
        {
            float gx = 4.f;
            MeshBuilder& e = hero.at(vec3(gx, 0, 0));
            for (int s = -1; s <= 1; s += 2) {
                e.boxAt({gx, 4.5f, s * 10.f}, {0.35f, 4.5f, 0.35f}, MT(vec3(0.25f, 0.26f, 0.3f), 0.5f, 40.f));
                e.boxAt({gx, 0.2f, s * 10.f}, {0.9f, 0.2f, 0.9f}, MT(vec3(0.5f), 0.3f, 30.f));
            }
            e.boxAt({gx, 9.2f, 0}, {0.6f, 0.7f, 10.6f}, MT(vec3(0.2f, 0.21f, 0.25f), 0.5f, 40.f));
            e.boxAt({gx - 0.65f, 9.2f, 0}, {0.03f, 0.55f, 10.f}, MT(vec3(0.05f), 0.3f, 30.f));
            for (int c = 0; c < 6; c++)
                e.boxAt({gx - 0.7f, 9.2f, -8.5f + c * 3.4f}, {0.02f, 0.4f, 1.4f}, MT(LIVERIES[c % 5].main, 0.3f, 30.f, 0.5f));
            for (int c = 0; c < 5; c++) {
                float z = -3.2f + c * 1.6f;
                e.boxAt({gx - 0.45f, 8.55f, z}, {0.12f, 0.35f, 0.6f}, hm);
                startLampPos.push_back(vec3(gx - 0.62f, 8.55f, z));
            }
        }
        // lamp posts along the main straight (both sides) and sparsely around the circuit
        auto addLamp = [&](float s, int side, float dist, bool) {
            vec3 base = T->pos(s, side * dist);
            vec3 inward = T->normal(s) * (float)(-side);
            vec3 head = base + inward * 2.6f; head.y = 11.5f;
            MeshBuilder& e = env.at(base);
            e.tube(base, base + vec3(0, 11.2f, 0), 0.16f, 0.1f, 8, pole);
            e.tube(base + vec3(0, 11.0f, 0), head, 0.07f, 0.06f, 6, pole);
            e.boxAt(head + vec3(0, -0.12f, 0), {0.55f, 0.1f, 0.3f}, headM);
            LightSrc L; L.pos = head + vec3(0, -0.3f, 0); L.dir = normalize(inward * 0.35f + vec3(0, -1, 0)); L.spot = true;
            L.col = vec3(2.2f, 2.0f, 1.7f); L.radius = 55.f; L.cosIn = 0.62f; L.cosOut = 0.2f;
            lamps.push_back(L);
        };
        for (float x = -440.f; x <= 400.f; x += 55.f) addLamp(T->wrapS(x), -1, 17.5f, true);
        for (float s = 100.f; s < T->L - 560.f; s += 150.f) addLamp(s + 560.f > T->L ? s : s + 560.f, (int)(s / 150.f) % 2 ? 1 : -1, 17.f, false);
        // marshal posts + TV camera platforms every ~220 m
        int cnt = 0;
        for (float s = 80.f; s < T->L; s += 215.f, cnt++) {
            float u = T->sRel(s);
            if (u > -520.f && u < 260.f) continue;
            int side = (cnt % 2) ? 1 : -1;
            vec3 p = T->pos(s, side * 19.f); vec3 t = T->tangent(s), n = T->normal(s);
            MeshBuilder& e = env.at(p);
            mat4 M = basis(t * (float)side, vec3(0, 1, 0), n * (float)side, p);
            e.box(M * translate(0, 1.1f, 0), vec3(1.4f, 1.1f, 1.0f), white);
            e.box(M * translate(0, 2.3f, 0), vec3(1.6f, 0.1f, 1.2f), orange);
            e.box(M * translate(0, 1.4f, -1.01f), vec3(1.0f, 0.4f, 0.02f), MT(vec3(0.2f, 0.4f, 0.6f), 0.8f, 80.f));
            e.tube(p + vec3(1.6f, 0, 0) * 0.f + n * (float)side * 0.f, p + vec3(0, 4.5f, 0), 0.0f, 0.0f, 3, white, false);
            if (cnt % 3 == 1) {      // TV camera tower
                vec3 q = T->pos(s + 40.f, side * 20.f);
                MeshBuilder& cm = env.at(q);
                for (int a = -1; a <= 1; a += 2) for (int b = -1; b <= 1; b += 2)
                    cm.tube(q + vec3(a * 1.0f, 0, b * 1.0f), q + vec3(a * 0.7f, 4.8f, b * 0.7f), 0.06f, 0.06f, 6, pole);
                cm.boxAt(q + vec3(0, 4.9f, 0), {1.0f, 0.06f, 1.0f}, MT(vec3(0.3f), 0.4f, 30.f));
                cm.boxAt(q + vec3(0, 5.5f, 0), {0.35f, 0.3f, 0.5f}, hm);
                camSpots.push_back(q + vec3(0, 6.4f, 0));
            }
        }
        // distance (brake) boards before corners: 300 / 200 / 100 m
        std::vector<float> v = T->speedProfile(0, 24.f, 22.f / 6400.f, 88.f, 11.f, 38.f);
        for (int i = 10; i < N - 10; i++) {
            bool lm = v[i] < 62.f;
            for (int q = -25; q <= 25; q++) if (v[T->wrap(i + q)] < v[i]) lm = false;
            if (!lm) continue;
            // find where the corner starts (curvature threshold) going backwards
            int j = i; while (std::fabs(T->k[T->wrap(j)]) > 1.f / 260.f && j > i - 120) j--;
            float sEnt = T->wrapS(j * T->ds);
            for (int q = 1; q <= 3; q++) {
                float s = T->wrapS(sEnt - q * 100.f);
                float u = T->sRel(s);
                if (u > -560.f && u < 380.f) continue;
                int side = 1;
                vec3 p = T->pos(s, side * 16.2f); vec3 t = T->tangent(s), n = T->normal(s);
                MeshBuilder& e = env.at(p);
                mat4 M = basis(t, vec3(0, 1, 0), n, p);
                e.tube(p, p + vec3(0, 2.1f, 0), 0.04f, 0.04f, 6, pole);
                e.box(M * translate(0, 2.3f, 0), vec3(0.04f, 0.8f, 0.8f), white);
                for (int b = 0; b < q; b++) e.box(M * translate(0.05f, 2.3f, -0.55f + (3 - q) * 0.0f + b * 0.55f - (q - 1) * 0.275f), vec3(0.03f, 0.7f, 0.12f), MT(vec3(0.85f, 0.06f, 0.06f), 0.2f, 20.f));
            }
        }
    }

    void buildScenery() {
        // trees
        Mat trunk = MT(vec3(0.30f, 0.20f, 0.12f), 0.05f, 8.f), leaf1 = MT(vec3(0.10f, 0.32f, 0.12f), 0.05f, 8.f, 0.f, 0.5f), leaf2 = MT(vec3(0.16f, 0.40f, 0.14f), 0.05f, 8.f, 0.f, 0.5f);
        float sp = 36.f;
        for (float x = -1100.f; x < 1100.f; x += sp)
            for (float z = -700.f; z < 1000.f; z += sp) {
                unsigned a = (unsigned)((x + 2000) / sp), b = (unsigned)((z + 2000) / sp);
                float px = x + (rnd(a, b, 1) - 0.5f) * sp * 0.9f, pz = z + (rnd(a, b, 2) - 0.5f) * sp * 0.9f;
                if (rnd(a, b, 3) > 0.5f) continue;
                if (T->distToTrack(px, pz) < 48.f) continue;
                if (px > -230.f && px < 250.f && pz > 5.f && pz < 100.f) continue;
                if (px > -200.f && px < 200.f && pz > -80.f && pz < 5.f) continue;
                float h = 5.f + 5.f * rnd(a, b, 4);
                MeshBuilder& e = env.at(px, pz);
                e.cyl(translate(px, 1.2f, pz), 0.3f, 0.2f, 1.2f, 6, trunk, false, false);
                const Mat& lm = rnd(a, b, 5) > 0.5f ? leaf1 : leaf2;
                if (rnd(a, b, 6) > 0.4f) {      // conifer
                    e.cyl(translate(px, 2.4f + h * 0.25f, pz), 2.0f + 0.06f * h, 0.05f, h * 0.28f, 7, lm, false, false);
                    e.cyl(translate(px, 2.4f + h * 0.55f, pz), 1.5f + 0.04f * h, 0.05f, h * 0.26f, 7, lm, false, false);
                    e.cyl(translate(px, 2.4f + h * 0.8f, pz), 1.0f, 0.02f, h * 0.2f, 7, lm, false, false);
                } else {                         // broad-leaf
                    e.sphere(translate(px, 2.6f + h * 0.4f, pz), 1.8f + 0.18f * h, 7, 5, lm);
                }
            }
        // billboards / sponsor towers behind some stands are omitted; flags along the main straight
        for (float x = -380.f; x <= 380.f; x += 40.f) {
            vec3 p(x, 0, -27.f);
            MeshBuilder& e = env.at(p);
            e.tube(p, p + vec3(0, 9.f, 0), 0.07f, 0.05f, 6, MT(vec3(0.7f)));
            vec3 col = LIVERIES[((int)(x / 40.f) + 20) % 5].main;
            e.quadN(p + vec3(0, 9.f, 0), p + vec3(0.1f, 9.f, 2.0f), p + vec3(0.1f, 7.6f, 2.0f), p + vec3(0, 7.6f, 0), vec3(-1, 0, 0), MT(col, 0.1f, 10.f, 0.1f));
        }
    }

    void buildHills() {
        MeshBuilder b;
        int seg = 160, rings = 5;
        float r0 = 1900.f, r1 = 2900.f;
        auto height = [&](float ang, float rr) {
            float h = 60.f + 70.f * std::sin(ang * 5.f + 1.f) + 45.f * std::sin(ang * 11.f + 2.f) + 25.f * std::sin(ang * 23.f);
            h = std::max(25.f, h) * (1.f - (rr - r0) / (r1 - r0) * 0.5f);
            return h;
        };
        vec3 cc = T->center;
        for (int r = 0; r < rings; r++)
            for (int i = 0; i < seg; i++) {
                float a0 = 2 * PI_F * i / seg, a1 = 2 * PI_F * (i + 1) / seg;
                float ra = mixf(r0, r1, (float)r / rings), rb = mixf(r0, r1, (float)(r + 1) / rings);
                auto P = [&](float a, float rr, int rl) {
                    float hgt = (rl == 0) ? -5.f : height(a, rr) * (rl == 1 ? 1.f : 1.f);
                    float t = (float)rl / rings;
                    (void)t;
                    return vec3(cc.x + std::cos(a) * rr, hgt, cc.z + std::sin(a) * rr);
                };
                vec3 p00 = P(a0, ra, r), p01 = P(a1, ra, r), p10 = P(a0, rb, r + 1), p11 = P(a1, rb, r + 1);
                if (r == 0) { p00.y = p01.y = 4.f; }
                p10.y = height(a0, rb); p11.y = height(a1, rb);
                float h = (p10.y + p11.y) * 0.5f;
                vec3 col = mix(vec3(0.22f, 0.40f, 0.22f), vec3(0.45f, 0.45f, 0.48f), clampf((h - 60.f) / 120.f, 0.f, 1.f));
                vec3 nrm = normalize(cross(p10 - p00, p01 - p00));
                if (nrm.y < 0) nrm = -nrm;
                b.quadN(p00, p10, p11, p01, nrm, MT(col, 0.f, 8.f, 0.f, 0.3f));
            }
        hills.upload(b);
    }

    // ---- dynamic emissive parts: pit lights, start lights -------------------------
    void drawLamps(Renderer& r, const Prims& pr, const PitStop& pit, int startLights, bool lightsOut) {
        auto lens = [&](const vec3& p, const vec3& col, bool on) {
            r.draw(pr.sphere, translate(p) * scale(0.15f), on ? col : col * 0.12f, on ? 1.0f : 0.f, true);
        };
        bool red = pit.redLamp(), green = pit.greenLamp(), amber = pit.amberLamp();
        for (size_t i = 0; i + 2 < lensPos.size(); i += 3) {
            lens(lensPos[i], vec3(1.0f, 0.05f, 0.04f), red);
            lens(lensPos[i + 1], vec3(1.0f, 0.65f, 0.05f), amber);
            lens(lensPos[i + 2], vec3(0.05f, 1.0f, 0.15f), green);
        }
        // amber beacons on the pit wall
        float bxp = BOX_X_PLAYER();
        for (int s = -1; s <= 1; s += 2)
            r.draw(pr.sphere, translate(bxp + s * 6.f, 1.55f, 12.65f) * scale(0.2f, 0.16f, 0.2f), vec3(1.0f, 0.6f, 0.05f), amber ? 1.f : 0.05f, true);
        // start lights
        for (int i = 0; i < (int)startLampPos.size(); i++) {
            bool on = !lightsOut && i < startLights;
            r.draw(pr.sphere, translate(startLampPos[i]) * scale(0.22f, 0.22f, 0.22f), on ? vec3(1.0f, 0.03f, 0.02f) : vec3(0.25f, 0.02f, 0.02f), on ? 1.1f : 0.f, true);
        }
    }
    static float BOX_X_PLAYER() { return PitC::BOX_X[PitC::PLAYER_BOX]; }
};
