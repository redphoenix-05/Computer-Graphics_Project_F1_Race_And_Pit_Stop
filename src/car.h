// ============================================================================
//  car.h  -  Hierarchical scene graph, Formula 1 car models, vehicle dynamics
//            and the waypoint-following driver used by AI cars / auto mode
// ============================================================================
#pragma once
#include <memory>
#include <functional>
#include "gfx.h"
#include "track.h"

// ---------------------------------------------------------------------------
//  Scene-graph node (hierarchical modelling): world = parent.world * local
// ---------------------------------------------------------------------------
struct Node {
    mat4 local, world, override_;
    const Mesh* mesh = nullptr;
    const DecalMesh* decal = nullptr;
    vec3 tint{1, 1, 1};
    float emis = 0;
    bool visible = true, hero = true, useOverride = false;
    std::vector<Node*> kids;

    void update(const mat4& parent) {
        world = useOverride ? override_ : parent * local;
        for (Node* k : kids) k->update(world);
    }
    void draw(Renderer& r) const {
        if (!visible) return;
        if (mesh) r.draw(*mesh, world, tint, emis, hero);
        if (decal) r.drawDecal(*decal, world);
        for (const Node* k : kids) k->draw(r);
    }
};
struct NodePool {
    std::vector<std::unique_ptr<Node>> items;
    Node* make(Node* parent = nullptr, const Mesh* mesh = nullptr) {
        items.emplace_back(new Node());
        Node* n = items.back().get();
        n->mesh = mesh;
        if (parent) parent->kids.push_back(n);
        return n;
    }
};

// ---------------------------------------------------------------------------
//  Shared primitive meshes (unit sized, tinted at draw time)
// ---------------------------------------------------------------------------
struct Prims {
    Mesh cube, cyl, sphere, sphereGloss, cone, cubeGloss, cylGloss, ring;
    void build() {
        MeshBuilder b;
        b.box(mat4(), vec3(0.5f), MT(vec3(1), 0.15f, 16)); cube.upload(b);
        b.v.clear(); b.box(mat4(), vec3(0.5f), MT(vec3(1), 0.8f, 64)); cubeGloss.upload(b);
        b.v.clear(); b.cyl(mat4(), 1, 1, 0.5f, 20, MT(vec3(1), 0.15f, 16)); cyl.upload(b);
        b.v.clear(); b.cyl(mat4(), 1, 1, 0.5f, 20, MT(vec3(1), 0.8f, 64)); cylGloss.upload(b);
        b.v.clear(); b.sphere(mat4(), 1.f, 20, 14, MT(vec3(1), 0.15f, 16)); sphere.upload(b);
        b.v.clear(); b.sphere(mat4(), 1.f, 20, 14, MT(vec3(1), 0.9f, 80)); sphereGloss.upload(b);
        b.v.clear(); b.cyl(mat4(), 1, 0.f, 0.5f, 20, MT(vec3(1), 0.1f, 16)); cone.upload(b);
    }
};

// ---------------------------------------------------------------------------
//  Livery and model construction
// ---------------------------------------------------------------------------
struct Livery {
    const char* name;
    vec3 main, second, accent;      // body colour, stripe / trim colour, accent (helmet, wing flap)
    vec3 nose, airbox, plate, wing; // nose cone, airbox, end plates, wing elements
    const char* sponsor; vec3 sponsorCol; const char* num; vec3 numCol; float gloss;
};
static const Livery LIVERIES[5] = {
    {"AURORA RED",    vec3(0.80f, 0.05f, 0.06f), vec3(0.95f), vec3(0.95f, 0.80f, 0.10f),
     vec3(0.80f, 0.05f, 0.06f), vec3(0.80f, 0.05f, 0.06f), vec3(0.06f), vec3(0.05f), "NEXUS", vec3(0.97f), "16", vec3(0.97f), 0.6f},
    {"MERIDIAN BLUE", vec3(0.045f, 0.07f, 0.24f), vec3(0.86f, 0.08f, 0.08f), vec3(0.98f, 0.80f, 0.10f),
     vec3(0.98f, 0.78f, 0.08f), vec3(0.98f, 0.78f, 0.08f), vec3(0.045f, 0.07f, 0.24f), vec3(0.05f, 0.06f, 0.14f), "OMNICORE", vec3(0.95f), "1", vec3(0.045f, 0.07f, 0.24f), 0.7f},
    {"APEX ORANGE",   vec3(0.98f, 0.45f, 0.04f), vec3(0.15f, 0.55f, 0.95f), vec3(0.25f, 0.65f, 0.95f),
     vec3(0.98f, 0.45f, 0.04f), vec3(0.98f, 0.45f, 0.04f), vec3(0.98f, 0.45f, 0.04f), vec3(0.05f), "AETHER", vec3(0.04f, 0.05f, 0.08f), "4", vec3(0.04f, 0.05f, 0.08f), 0.6f},
    {"SILVER ARROW",  vec3(0.028f, 0.03f, 0.036f), vec3(0.0f, 0.78f, 0.70f), vec3(0.0f, 0.78f, 0.70f),
     vec3(0.035f, 0.037f, 0.043f), vec3(0.86f, 0.06f, 0.08f), vec3(0.03f), vec3(0.04f), "ARCTIC OIL", vec3(0.96f), "44", vec3(0.96f), 0.9f},
    {"VERDE GREEN",   vec3(0.03f, 0.42f, 0.28f), vec3(0.78f, 0.95f, 0.15f), vec3(0.78f, 0.95f, 0.15f),
     vec3(0.03f, 0.42f, 0.28f), vec3(0.03f, 0.42f, 0.28f), vec3(0.78f, 0.95f, 0.15f), vec3(0.04f), "VERDANT", vec3(0.97f), "14", vec3(0.78f, 0.95f, 0.15f), 0.6f},
};
struct Compound { const char* name; vec3 col; };
static const Compound COMPOUNDS[3] = {{"SOFT", vec3(0.90f, 0.08f, 0.08f)}, {"MEDIUM", vec3(0.95f, 0.80f, 0.08f)}, {"HARD", vec3(0.92f, 0.92f, 0.92f)}};

typedef MeshBuilder::LoftSt St;
// ---- station tables of the smooth body parts (x, cy, cz, half-width, half-height, exponent, bottom factor)
static std::vector<St> stChassis() {
    return {{2.98f, 0.20f, 0, 0.020f, 0.020f, 2.2f, 1.f}, {2.80f, 0.205f, 0, 0.050f, 0.045f, 2.4f, 0.9f}, {2.45f, 0.215f, 0, 0.085f, 0.075f, 2.5f, 0.85f},
            {2.00f, 0.240f, 0, 0.140f, 0.115f, 2.6f, 0.8f}, {1.55f, 0.300f, 0, 0.200f, 0.160f, 2.7f, 0.8f}, {1.15f, 0.370f, 0, 0.260f, 0.220f, 2.8f, 0.85f},
            {0.70f, 0.450f, 0, 0.310f, 0.270f, 3.0f, 0.85f}, {0.15f, 0.470f, 0, 0.330f, 0.290f, 3.0f, 0.85f}, {-0.35f, 0.460f, 0, 0.300f, 0.270f, 3.0f, 0.85f},
            {-0.80f, 0.440f, 0, 0.220f, 0.220f, 3.0f, 0.8f}};
}
static std::vector<St> stAirbox() {
    return {{0.12f, 0.99f, 0, 0.10f, 0.11f, 2.2f, 1.f}, {-0.15f, 0.97f, 0, 0.15f, 0.17f, 2.4f, 0.9f}, {-0.60f, 0.92f, 0, 0.19f, 0.22f, 2.5f, 0.85f},
            {-1.20f, 0.76f, 0, 0.19f, 0.20f, 2.5f, 0.8f}, {-1.80f, 0.60f, 0, 0.14f, 0.13f, 2.5f, 0.8f}, {-2.30f, 0.50f, 0, 0.09f, 0.08f, 2.4f, 0.8f}};
}
static std::vector<St> stSidepod(float s) {
    return {{0.85f, 0.40f, s * 0.50f, 0.13f, 0.15f, 2.4f, 0.8f}, {0.55f, 0.40f, s * 0.56f, 0.20f, 0.20f, 2.5f, 0.75f}, {0.15f, 0.40f, s * 0.60f, 0.25f, 0.24f, 2.6f, 0.7f},
            {-0.40f, 0.38f, s * 0.58f, 0.24f, 0.22f, 2.6f, 0.55f}, {-1.00f, 0.33f, s * 0.47f, 0.17f, 0.17f, 2.5f, 0.45f}, {-1.60f, 0.28f, s * 0.34f, 0.11f, 0.11f, 2.4f, 0.4f},
            {-2.05f, 0.26f, s * 0.22f, 0.06f, 0.06f, 2.3f, 0.4f}};
}
static St stAt(const std::vector<St>& v, float x);
static std::vector<St> splitAt(std::vector<St> v, float x, float eps = 0.012f) {
    for (size_t i = 0; i + 1 < v.size(); i++)
        if (x <= v[i].x && x >= v[i + 1].x) {
            St a = stAt(v, x + eps), b = stAt(v, x - eps);
            v.insert(v.begin() + i + 1, b); v.insert(v.begin() + i + 1, a); break;
        }
    return v;
}
static St stAt(const std::vector<St>& v, float x) {   // linear interpolation between stations (x decreasing)
    if (x >= v.front().x) return v.front();
    for (size_t i = 0; i + 1 < v.size(); i++)
        if (x <= v[i].x && x >= v[i + 1].x) {
            float t = (v[i].x - x) / (v[i].x - v[i + 1].x); St r;
            r.x = x; r.cy = mixf(v[i].cy, v[i + 1].cy, t); r.cz = mixf(v[i].cz, v[i + 1].cz, t); r.hw = mixf(v[i].hw, v[i + 1].hw, t);
            r.hh = mixf(v[i].hh, v[i + 1].hh, t); r.n = mixf(v[i].n, v[i + 1].n, t); r.bot = mixf(v[i].bot, v[i + 1].bot, t); return r;
        }
    return v.back();
}
// z of the outer side surface at (x, y) for a loft (side s = +1 right / -1 left)
static float sideZ(const std::vector<St>& v, float s, float x, float y) {
    St a = stAt(v, x);
    float hh = a.hh * (y < a.cy ? a.bot : 1.f);
    float q = std::min(0.999f, std::fabs(y - a.cy) / hh);
    return a.cz + s * a.hw * std::pow(std::max(0.f, 1.f - std::pow(q, a.n)), 1.f / a.n);
}
static float topY(const std::vector<St>& v, float x, float z) {
    St a = stAt(v, x);
    float q = std::min(0.999f, std::fabs(z - a.cz) / std::max(a.hw, 1e-3f));
    return a.cy + a.hh * std::pow(std::max(0.f, 1.f - std::pow(q, a.n)), 1.f / a.n);
}

// Chassis, aerodynamics, suspension and cockpit (without wheels / helmet)
inline void buildCarBody(MeshBuilder& b, const Livery& L) {
    float G = L.gloss;
    Mat carbon = MT(vec3(0.06f, 0.06f, 0.07f), 0.55f, 40.f), dark = MT(vec3(0.015f, 0.015f, 0.02f), 0.2f, 20.f);
    Mat floorM = MT(vec3(0.03f, 0.03f, 0.035f), 0.12f, 14.f);
    Mat metal = MT(vec3(0.55f, 0.56f, 0.58f), 0.9f, 90.f), ti = MT(vec3(0.22f, 0.23f, 0.25f), 0.7f, 60.f);
    Mat wingM = MT(L.wing, 0.5f, 45.f), accM = MT(L.accent, 0.5f, 45.f), plateM = MT(L.plate, 0.6f, 55.f);
    mat4 I;
    // ---- monocoque + nose (smooth loft) ------------------------------------------------
    auto cfBody = [&](const vec3& p, const vec3& n) {
        vec3 c = L.main;
        if (p.x > 1.95f) c = L.nose;
        else if (n.y > 0.6f && std::fabs(p.z) < 0.055f && p.x > 0.0f) c = L.second;      // centre line stripe
        return MT(c, 0.45f + 0.4f * G, 50.f + 60.f * G);
    };
    b.loftSmooth(I, splitAt(stChassis(), 1.95f), 28, cfBody);
    // ---- airbox + engine cover ----------------------------------------------------------
    auto cfAir = [&](const vec3& p, const vec3& n) {
        vec3 c = (p.x > -0.85f) ? L.airbox : L.main;
        if (p.x <= -0.85f && n.y > 0.6f && std::fabs(p.z) < 0.04f) c = L.second;
        return MT(c, 0.45f + 0.4f * G, 50.f + 60.f * G);
    };
    b.loftSmooth(I, splitAt(stAirbox(), -0.85f), 24, cfAir);
    b.sphere(translate(0.125f, 0.99f, 0) * scale(0.025f, 0.095f, 0.095f), 1.f, 14, 10, dark);        // intake mouth
    for (int i = 0; i < 6; i++) b.boxAt({-1.05f - i * 0.12f, 0.835f - i * 0.03f, 0}, {0.03f, 0.003f, 0.12f - i * 0.012f}, dark);   // exit louvres
    // ---- sidepods ------------------------------------------------------------------------
    for (int sd = -1; sd <= 1; sd += 2) {
        float s = (float)sd;
        auto cfPod = [&](const vec3& p, const vec3& n) {
            vec3 c = L.main;
            if (n.y > 0.55f && n.y < 0.80f && p.x < 0.55f) c = L.second;                          // shoulder stripe
            return MT(c, 0.45f + 0.4f * G, 50.f + 60.f * G);
        };
        b.loftSmooth(I, stSidepod(s), 24, cfPod);
        b.sphere(translate(0.86f, 0.40f, s * 0.50f) * scale(0.025f, 0.125f, 0.105f), 1.f, 14, 10, dark);   // inlet
        for (int i = 0; i < 5; i++) b.boxAt({-0.05f - i * 0.15f, 0.595f - i * 0.014f, s * (0.56f - i * 0.02f)}, {0.045f, 0.004f, 0.10f}, dark);   // louvres
        // floor edge, bargeboards
        b.boxAt({0.45f, 0.080f, s * 0.86f}, {1.1f, 0.010f, 0.07f}, floorM);
        b.boxAt({0.95f, 0.22f, s * 0.70f}, {0.35f, 0.14f, 0.008f}, carbon);
        b.boxAt({1.15f, 0.15f, s * 0.78f}, {0.30f, 0.07f, 0.008f}, carbon);
        // mirror + stalk
        b.sphere(translate(1.02f, 0.72f, s * 0.53f) * scale(0.06f, 0.035f, 0.085f), 1.f, 12, 8, MT(L.main, 0.6f, 60.f));
        b.tube({0.96f, 0.64f, s * 0.30f}, {1.01f, 0.71f, s * 0.48f}, 0.010f, 0.010f, 6, carbon);
    }
    // ---- floor, diffuser, rear -----------------------------------------------------------
    b.boxAt({0.0f, 0.070f, 0}, {2.3f, 0.012f, 0.80f}, floorM);
    b.box(translate(-2.45f, 0.155f, 0) * rotZ(0.13f), vec3(0.50f, 0.010f, 0.64f), floorM);
    for (int i = -2; i <= 2; i++) b.boxAt({-2.5f, 0.17f, i * 0.2f}, {0.45f, 0.06f, 0.006f}, carbon);
    b.loftSmooth(I, {{-1.9f, 0.37f, 0, 0.10f, 0.13f, 2.4f, 0.8f}, {-2.45f, 0.335f, 0, 0.075f, 0.11f, 2.4f, 0.8f}, {-2.95f, 0.31f, 0, 0.06f, 0.07f, 2.4f, 0.8f}},
                  14, [&](const vec3&, const vec3&) { return carbon; });
    b.tube({-2.2f, 0.50f, 0.05f}, {-3.0f, 0.46f, 0.05f}, 0.045f, 0.04f, 10, metal);
    b.boxAt({-2.97f, 0.345f, 0}, {0.012f, 0.04f, 0.085f}, MT(vec3(1.0f, 0.05f, 0.03f), 0.2f, 20.f, 1.0f));   // rain light
    // ---- front wing: 4 curved elements + shaped end plates -----------------------------------
    b.wingElement(I, 2.90f, 0.34f, 0.050f, 0.070f, 0.12f, 0.96f, 0.012f, 14, wingM);
    b.wingElement(I, 2.80f, 0.26f, 0.100f, 0.120f, 0.22f, 0.96f, 0.012f, 14, carbon);
    b.wingElement(I, 2.71f, 0.22f, 0.150f, 0.160f, 0.30f, 0.96f, 0.012f, 14, wingM);
    b.wingElement(I, 2.63f, 0.18f, 0.200f, 0.200f, 0.38f, 0.96f, 0.012f, 14, accM);
    for (int sd = -1; sd <= 1; sd += 2) {
        float s = (float)sd;
        b.extrudePoly(I, {{3.07f, 0.035f, 0}, {2.50f, 0.035f, 0}, {2.45f, 0.20f, 0}, {2.55f, 0.37f, 0}, {2.80f, 0.34f, 0}, {3.00f, 0.22f, 0}}, s * 0.975f, 0.013f, plateM);
        b.boxAt({2.80f, 0.040f, s * 0.935f}, {0.26f, 0.004f, 0.045f}, carbon);                       // foot plate
        b.tube({2.50f, 0.27f, s * 0.07f}, {2.78f, 0.115f, s * 0.12f}, 0.016f, 0.016f, 6, carbon);   // nose pylons
    }
    b.tube({2.97f, 0.20f, 0}, {3.04f, 0.20f, 0}, 0.008f, 0.004f, 6, metal);
    // ---- rear wing: main plane, DRS flap, end plates, beam wing -------------------------------
    b.wingElement(I, -2.78f, 0.36f, 1.000f, 0.f, 0.05f, 0.56f, 0.014f, 4, wingM);
    b.wingElement(I, -2.84f, 0.26f, 1.140f, 0.f, 0.22f, 0.56f, 0.014f, 4, accM);
    b.wingElement(I, -2.88f, 0.22f, 0.500f, 0.f, 0.15f, 0.38f, 0.012f, 4, carbon);
    for (int sd = -1; sd <= 1; sd += 2) {
        float s = (float)sd;
        b.extrudePoly(I, {{-2.52f, 0.55f, 0}, {-3.10f, 0.62f, 0}, {-3.13f, 1.31f, 0}, {-2.64f, 1.27f, 0}, {-2.52f, 0.98f, 0}}, s * 0.575f, 0.014f, plateM);
        b.boxAt({-2.82f, 1.30f, s * 0.575f}, {0.30f, 0.008f, 0.012f}, accM);
    }
    b.boxAt({-2.62f, 0.78f, 0}, {0.04f, 0.22f, 0.025f}, carbon);
    b.boxAt({-2.78f, 1.04f, 0}, {0.05f, 0.04f, 0.04f}, carbon);                                        // DRS actuator
    // ---- cockpit: opening, headrest, halo ------------------------------------------------------
    b.boxAt({0.45f, 0.748f, 0}, {0.42f, 0.006f, 0.17f}, dark);
    b.boxAt({-0.05f, 0.80f, 0}, {0.13f, 0.10f, 0.15f}, carbon);
    b.tube({1.00f, 0.68f, 0}, {0.66f, 0.99f, 0}, 0.015f, 0.015f, 8, ti);
    for (int sd = -1; sd <= 1; sd += 2) {
        float s = (float)sd;
        b.tube({0.66f, 0.99f, 0}, {0.32f, 1.00f, s * 0.27f}, 0.013f, 0.013f, 8, ti);
        b.tube({0.32f, 1.00f, s * 0.27f}, {-0.05f, 0.78f, s * 0.26f}, 0.013f, 0.013f, 8, ti);
    }
    // ---- suspension, brake drums, axles ----------------------------------------------------------
    for (int w = 0; w < 4; w++) {
        float x = WHEEL_X[w], y = WHEEL_R[w], zz = WHEEL_Z[w], s = zz > 0 ? 1.f : -1.f;
        float dx = (w < 2) ? 0.22f : 0.25f;
        vec3 hubU(x, y + 0.08f, s * 0.60f), hubL(x, y - 0.09f, s * 0.60f);
        b.tube({x + dx, y + 0.06f, s * 0.24f}, hubU, 0.014f, 0.014f, 6, carbon);
        b.tube({x - dx, y + 0.06f, s * 0.24f}, hubU, 0.014f, 0.014f, 6, carbon);
        b.tube({x + dx, y - 0.10f, s * 0.24f}, hubL, 0.016f, 0.016f, 6, carbon);
        b.tube({x - dx, y - 0.10f, s * 0.24f}, hubL, 0.016f, 0.016f, 6, carbon);
        b.tube({x, y + 0.24f, s * 0.58f}, {x, y + 0.24f, s * 0.20f}, 0.012f, 0.012f, 6, metal);
        b.cyl(translate(x, y, s * 0.64f) * rotX(PI_F * 0.5f), 0.19f, 0.19f, 0.045f, 20, MT(vec3(0.08f, 0.08f, 0.09f), 0.5f, 40.f));   // brake drum
        b.tube({x, y, s * 0.40f}, {x, y, s * 0.72f}, 0.050f, 0.050f, 10, MT(vec3(0.35f, 0.35f, 0.37f), 0.8f, 60.f));              // axle stub
    }
}

