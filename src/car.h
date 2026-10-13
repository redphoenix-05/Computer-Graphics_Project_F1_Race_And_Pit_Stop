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

static const float WHEEL_X[4] = {1.75f, 1.75f, -1.85f, -1.85f};
static const float WHEEL_Z[4] = {-0.80f, 0.80f, -0.80f, 0.80f};   // FL, FR, RL, RR
static const float WHEEL_R[4] = {0.34f, 0.34f, 0.36f, 0.36f};
static const float WHEEL_W[4] = {0.305f, 0.305f, 0.40f, 0.40f};

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

// Complete wheel assembly: low-profile tyre (lathe) with coloured compound ring, wheel cover, nut.
// Axis = local Z, outer face = +Z.
inline void buildWheel(MeshBuilder& b, float R, float W, const vec3& stripe) {
    float hw = W * 0.5f;
    vec3 rub(0.035f, 0.035f, 0.04f), rub2(0.06f, 0.06f, 0.065f);
    std::vector<ProfilePt> pr;
    auto side = [&](float sg) {
        std::vector<ProfilePt> h;
        h.push_back({0.222f, sg * hw * 0.93f, rub, true});
        h.push_back({0.236f, sg * hw, rub, false});
        h.push_back({0.238f, sg * hw, stripe, true});
        h.push_back({0.252f, sg * hw, stripe, true});
        h.push_back({0.254f, sg * hw, rub, true});
        h.push_back({R - 0.030f, sg * hw, rub, false});
        h.push_back({R - 0.010f, sg * hw * 0.92f, rub, false});
        h.push_back({R - 0.002f, sg * hw * 0.70f, rub2, false});
        return h;
    };
    std::vector<ProfilePt> a = side(-1.f), c = side(1.f);
    for (auto& q : a) pr.push_back(q);
    float g[3] = {-0.28f, 0.f, 0.28f};
    for (int i = 0; i < 3; i++) {
        pr.push_back({R, g[i] * W - 0.02f, rub2, true});
        pr.push_back({R - 0.006f, g[i] * W - 0.012f, rub, true});
        pr.push_back({R - 0.006f, g[i] * W + 0.012f, rub, true});
        pr.push_back({R, g[i] * W + 0.02f, rub2, true});
    }
    for (int i = (int)c.size() - 1; i >= 0; i--) pr.push_back(c[i]);
    b.lathe(mat4(), pr, 44, 0.04f, 10.f);
    // rim barrel + domed wheel cover (2022-style closed wheel)
    vec3 rimC(0.50f, 0.51f, 0.54f), cover(0.07f, 0.07f, 0.085f);
    std::vector<ProfilePt> rim = {
        {0.0f, hw * 0.80f, cover, true}, {0.10f, hw * 0.78f, cover, false}, {0.18f, hw * 0.72f, cover, false},
        {0.206f, hw * 0.66f, cover, false}, {0.2086f, hw * 0.64f, rimC, true}, {0.2286f, hw * 0.60f, rimC, true},
        {0.2286f, -hw * 0.88f, rimC, true}};
    b.lathe(mat4(), rim, 36, 0.55f, 40.f);
    // cooling slots in the cover
    for (int i = 0; i < 12; i++) {
        float a0 = 2 * PI_F * i / 12;
        b.box(translate(0, 0, hw * 0.79f) * rotZ(a0) * translate(0.14f, 0, 0), vec3(0.03f, 0.006f, 0.004f), MT(vec3(0.01f), 0.2f, 10.f));
    }
    b.cyl(translate(0, 0, hw * 0.83f) * rotX(PI_F * 0.5f), 0.040f, 0.040f, 0.014f, 12, MT(vec3(0.95f, 0.85f, 0.10f), 0.8f, 64.f));
    b.cyl(translate(0, 0, hw * 0.86f) * rotX(PI_F * 0.5f), 0.020f, 0.020f, 0.010f, 8, MT(vec3(0.25f, 0.25f, 0.27f), 0.8f, 64.f));
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

// ---- decals on the body (sponsor names, race numbers) ----------------------------------------------
inline void buildCarDecals(DecalBuilder& d, const Livery& L) {
    std::vector<St> pod = stSidepod(1.f), pod2 = stSidepod(-1.f), air = stAirbox(), ch = stChassis();
    // sidepod sponsor text (both sides, reads front-to-back on each side)
    for (int sd = -1; sd <= 1; sd += 2) {
        float s = (float)sd; const std::vector<St>& P = sd > 0 ? pod : pod2;
        float h = 0.19f, wtxt = d.width(L.sponsor, h), xc = 0.18f, x0 = xc - s * wtxt * 0.5f;
        SurfFn f = [&, s, x0](float u, float v, vec3& pos, vec3& nrm) {
            float x = x0 + s * u, y = v, z = sideZ(P, s, x, y) + s * 0.02f;
            float e = 0.01f;
            float gx = (sideZ(P, s, x + e, y) - sideZ(P, s, x - e, y)) / (2 * e), gy = (sideZ(P, s, x, y + e) - sideZ(P, s, x, y - e)) / (2 * e);
            pos = vec3(x, y, z); nrm = normalize(vec3(-gx, -gy, 1.f) * s);
        };
        d.text(L.sponsor, 0, 0.345f, h, L.sponsorCol, f);
        d.bar(0, 0.318f, wtxt, 0.014f, L.second, f, 10);     // trim line under the text
        // race number on the airbox flank
        std::string num = L.num; float hn = 0.26f, wn = d.width(num, hn), xn = -0.62f - s * 0 + (s > 0 ? -wn * 0.5f : wn * 0.5f);
        SurfFn g = [&, s, xn](float u, float v, vec3& pos, vec3& nrm) {
            float x = xn + s * u, y = v, z = sideZ(air, s, x, y) + s * 0.02f;
            float e = 0.01f;
            float gx = (sideZ(air, s, x + e, y) - sideZ(air, s, x - e, y)) / (2 * e), gy = (sideZ(air, s, x, y + e) - sideZ(air, s, x, y - e)) / (2 * e);
            pos = vec3(x, y, z); nrm = normalize(vec3(-gx, -gy, 1.f) * s);
        };
        d.text(num, 0, 0.78f, hn, L.numCol, g);
    }
    // race number on the nose (top), reads from behind
    {
        std::string num = L.num; float h = 0.17f, w = d.width(num, h);
        SurfFn f = [&](float u, float v, vec3& pos, vec3& nrm) {
            float x = 1.62f + v, z = u - w * 0.5f;
            pos = vec3(x, topY(ch, x, z) + 0.012f, z);
            float e = 0.01f; float gx = (topY(ch, x + e, z) - topY(ch, x - e, z)) / (2 * e), gz = (topY(ch, x, z + e) - topY(ch, x, z - e)) / (2 * e);
            nrm = normalize(vec3(-gx, 1.f, -gz));
        };
        d.text(num, 0, 0, h, L.numCol, f);
    }
    // sponsor on the upper rear-wing flap (reads from behind) and on the front-wing main plane (reads from the front)
    {
        std::string t = L.sponsor; float h = 0.11f, w = d.width(t, h);
        vec3 col = L.sponsorCol; if (length(col) > 1.5f && L.accent.x > 0.9f) col = vec3(0.97f);
        SurfFn f = [&](float u, float v, vec3& pos, vec3& nrm) {
            float x = -2.76f - 0.03f + v * -1.f * -1.f; (void)x;
            float xx = -2.77f + v; float y = 1.140f + 0.014f + 0.22f * (-2.84f + 0.13f - xx) + 0.004f;
            pos = vec3(xx, y, u - w * 0.5f); nrm = normalize(vec3(0.22f, 1.f, 0));
        };
        d.text(t, 0, 0.0f, h * 0.9f, vec3(0.97f), f, 0.95f);
        for (int sd = -1; sd <= 1; sd += 2) {
            float s = (float)sd;
            SurfFn g = [&, s](float u, float v, vec3& pos, vec3& nrm) {
                float xx = 3.07f - v, z = s * (0.18f) - s * u;                       // reads towards the car centre-line
                float q = z / 0.96f; float y = 0.050f + 0.070f * q * q + 0.012f + 0.12f * (3.07f - xx) + 0.004f; (void)y;
                float yy = 0.050f + 0.070f * q * q + 0.012f + 0.12f * (2.90f + 0.17f - xx) + 0.003f;
                pos = vec3(xx, yy, z); nrm = normalize(vec3(0.12f, 1.f, 0));
            };
            d.text("NEXUS", 0, 0.015f, 0.105f, vec3(0.97f, 0.97f, 0.97f), g, 0.95f);
        }
    }
}
// text on the tyre side wall (arc around the wheel, outer face +Z)
inline void buildWheelDecal(DecalBuilder& d, float R, float W) {
    float hw = W * 0.5f, r = (0.256f + R - 0.034f) * 0.5f, h = (R - 0.034f) - 0.258f;
    h = std::min(h, 0.052f);
    std::string t = "GRIPTEC";
    float dth = (h * 0.5f * 0.92f) / r; float th0 = PI_F * 0.5f + dth * (t.size() - 1) * 0.5f;
    for (size_t i = 0; i < t.size(); i++) {
        float th = th0 - dth * i;
        vec3 c(r * std::cos(th), r * std::sin(th), hw + 0.0015f), er(std::cos(th), std::sin(th), 0), ea(std::sin(th), -std::cos(th), 0);
        float w = h * 0.5f;
        vec3 p[4] = {c - ea * (w * 0.5f) - er * (h * 0.5f), c + ea * (w * 0.5f) - er * (h * 0.5f), c + ea * (w * 0.5f) + er * (h * 0.5f), c - ea * (w * 0.5f) + er * (h * 0.5f)};
        int ch = (unsigned char)t[i]; int idx = ch - 32, cx = idx % FONT_COLS, cy = idx / FONT_COLS;
        float tu0 = (cx * FONT_CW + 1.5f) / FONT_W, tv0 = (cy * FONT_CH + 3.f) / FONT_H, tu1 = (cx * FONT_CW + FONT_CW - 1.5f) / FONT_W, tv1 = (cy * FONT_CH + FONT_CH - 3.f) / FONT_H;
        d.glyphQuad(p, vec3(0, 0, 1), tu0, tv0, tu1, tv1, vec3(0.92f), 0.95f);
    }
    // small second line "P" mark opposite side (bottom)
    float th = -PI_F * 0.5f;
    vec3 c(r * std::cos(th), r * std::sin(th), hw + 0.0015f), er(std::cos(th), std::sin(th), 0), ea(std::sin(th), -std::cos(th), 0);
    float w = h * 0.5f;
    vec3 p[4] = {c - ea * (w * 0.5f) - er * (h * 0.5f), c + ea * (w * 0.5f) - er * (h * 0.5f), c + ea * (w * 0.5f) + er * (h * 0.5f), c - ea * (w * 0.5f) + er * (h * 0.5f)};
    int idx = 'P' - 32, cx = idx % FONT_COLS, cy = idx / FONT_COLS;
    d.glyphQuad(p, vec3(0, 0, 1), (cx * FONT_CW + 1.5f) / FONT_W, (cy * FONT_CH + 3.f) / FONT_H, (cx * FONT_CW + FONT_CW - 1.5f) / FONT_W, (cy * FONT_CH + FONT_CH - 3.f) / FONT_H, vec3(0.92f), 0.95f);
}

// ---------------------------------------------------------------------------
//  Car model assets (shared between frames; one body mesh per livery)
// ---------------------------------------------------------------------------
struct CarAssets {
    Mesh body[5];
    Mesh helmet[5];
    Mesh steerWheel;
    Mesh wheel[2][3];           // [front/rear][compound]
    DecalMesh bodyDecal[5], wheelDecal[2];
    void build() {
        for (int i = 0; i < 5; i++) {
            MeshBuilder b; buildCarBody(b, LIVERIES[i]); body[i].upload(b);
            DecalBuilder d; buildCarDecals(d, LIVERIES[i]); bodyDecal[i].upload(d.v);
            MeshBuilder h;
            h.sphere(translate(0.30f, 0.87f, 0), 0.135f, 20, 14, MT(LIVERIES[i].accent, 0.9f, 90.f));
            h.box(translate(0.415f, 0.89f, 0), vec3(0.04f, 0.04f, 0.10f), MT(vec3(0.02f, 0.02f, 0.05f), 0.95f, 120.f));
            h.box(translate(0.20f, 0.66f, 0) * rotZ(-0.2f), vec3(0.14f, 0.12f, 0.17f), MT(vec3(0.85f, 0.85f, 0.85f), 0.3f, 30.f));
            helmet[i].upload(h);
        }
        MeshBuilder s;
        s.box(mat4(), vec3(0.02f, 0.055f, 0.15f), MT(vec3(0.05f, 0.05f, 0.06f), 0.5f, 40.f));
        s.box(translate(0.025f, 0.0f, 0), vec3(0.008f, 0.04f, 0.08f), MT(vec3(0.1f, 0.85f, 0.2f), 0.2f, 20.f, 0.8f));
        for (int i = -1; i <= 1; i += 2) s.box(translate(0.0f, 0.0f, i * 0.17f), vec3(0.03f, 0.07f, 0.03f), MT(vec3(0.1f, 0.1f, 0.1f)));
        steerWheel.upload(s);
        for (int f = 0; f < 2; f++) {
            for (int c = 0; c < 3; c++) {
                MeshBuilder w; buildWheel(w, f == 0 ? 0.34f : 0.36f, f == 0 ? 0.305f : 0.40f, COMPOUNDS[c].col);
                wheel[f][c].upload(w);
            }
            DecalBuilder d; buildWheelDecal(d, f == 0 ? 0.34f : 0.36f, f == 0 ? 0.305f : 0.40f); wheelDecal[f].upload(d.v);
        }
    }
};

// ---------------------------------------------------------------------------
//  Car state, hierarchy and dynamics
// ---------------------------------------------------------------------------
enum CarCtrl { CTRL_AUTO = 0, CTRL_MANUAL = 1, CTRL_PIT = 2, CTRL_GRID = 3 };

struct Car {
    int id = 0; bool player = false; const Livery* liv = nullptr;
    // kinematic state
    vec3 pos; float heading = 0, speed = 0, steer = 0, steerIn = 0;
    float latAcc = 0, longAcc = 0, yawRate = 0, lift = 0;
    float spin[4] = {0, 0, 0, 0};
    // race state
    int idx = -1, lapsDone = 0; float s = 0, d = 0, prevS = 0, lapStart = 0, lastLap = 0, bestLap = 0;
    bool inLane = false; float hitCooldown = 0, distTotal = 0;
    float wear = 0;                  // tyre wear 0..1
    int compound = 1;                // 0 soft 1 medium 2 hard
    CarCtrl ctrl = CTRL_AUTO;
    // driver (AI)
    float lineBase = 0, lineCur = 0, lineTarget = 0, passHold = 0, phase = 0, skill = 1.f;
    std::vector<float> prof;
    float thr = 0, brk = 0;
    // hierarchy
    Node* root = nullptr; Node* tilt = nullptr; Node* bodyN = nullptr; Node* helmetN = nullptr; Node* steerN = nullptr;
    Node* hub[4] = {}; Node* wheelN[4] = {};

    static constexpr float WB = 3.6f;   // wheelbase

    float progress() const { return lapsDone * 0 + distTotal; }
    vec3 forward() const { return vec3(std::cos(heading), 0, std::sin(heading)); }
    vec3 rightV() const { return vec3(-std::sin(heading), 0, std::cos(heading)); }

    void buildHierarchy(NodePool& pool, const CarAssets& A) {
        root = pool.make();
        tilt = pool.make(root);
        bodyN = pool.make(tilt, &A.body[id]);
        pool.make(bodyN)->decal = &A.bodyDecal[id];
        helmetN = pool.make(tilt, &A.helmet[id]);
        steerN = pool.make(tilt, &A.steerWheel);
        for (int w = 0; w < 4; w++) {
            hub[w] = pool.make(tilt);
            wheelN[w] = pool.make(hub[w], &A.wheel[w < 2 ? 0 : 1][compound]);
            pool.make(wheelN[w])->decal = &A.wheelDecal[w < 2 ? 0 : 1];
        }
    }
    void setCompound(const CarAssets& A, int c) {
        compound = c;
        for (int w = 0; w < 4; w++) wheelN[w]->mesh = &A.wheel[w < 2 ? 0 : 1][c];
    }
    // update the transforms of the node hierarchy from the kinematic state
    void updateNodes() {
        root->local = translate(pos.x, pos.y + lift, pos.z) * rotY(-heading);
        float pitch = clampf(longAcc * 0.0011f, -0.06f, 0.05f), roll = clampf(-latAcc * 0.0009f, -0.05f, 0.05f);
        tilt->local = rotX(roll) * rotZ(pitch);
        steerN->local = translate(0.78f, 0.72f, 0) * rotZ(0.9f) * rotX(steer * 6.f);
        for (int w = 0; w < 4; w++) {
            hub[w]->local = translate(WHEEL_X[w], WHEEL_R[w], WHEEL_Z[w]) * (w < 2 ? rotY(-steer) : mat4());
            wheelN[w]->local = rotZ(spin[w]) * (WHEEL_Z[w] < 0 ? rotY(PI_F) : mat4());
        }
        root->update(mat4());
    }
    // world matrix a wheel would have if it were mounted (used by the pit animation)
    mat4 mountedWheelMatrix(int w) const { return hub[w]->world * (WHEEL_Z[w] < 0 ? rotY(PI_F) : mat4()); }

    // ---- dynamics ----------------------------------------------------------
    static float aLatMax(float v) { return 24.f + 22.f * std::min(1.f, (v * v) / 6400.f); }
    static float steerLimit(float v) {      // largest useful wheel angle at speed v
        v = std::max(v, 1.f);
        return clampf(1.12f * WB * aLatMax(v) / (v * v), 0.035f, 0.52f);
    }
    // surface: 0 = asphalt/kerb, 1 = runoff
    void dynamics(float dt, float thr_, float brk_, float steerTarget, float surfaceGrip, bool allowReverse) {
        thr = thr_; brk = brk_;
        float v = speed;
        float gripF = (1.f - 0.14f * wear) * surfaceGrip;
        float a = 0;
        float vr = std::max(0.f, v);
        a += thr * 13.5f * (1.f - (vr * vr) / (105.f * 105.f)) * (0.9f + 0.1f * gripF);
        if (brk > 0) {
            if (v > 0.6f) a -= brk * 46.f * std::min(1.f, gripF + 0.1f);
            else if (allowReverse && thr < 0.05f) a -= brk * 8.f;
        }
        a -= 0.0006f * v * std::fabs(v) + (v > 0.05f ? 0.25f : (v < -0.05f ? -0.25f : 0.f));
        if (surfaceGrip < 0.99f) a -= sgn(v) * 5.5f * std::min(1.f, std::fabs(v) / 20.f);
        float v0 = v;
        v += a * dt;
        if (v0 > 0 && v < 0 && brk > 0.05f && thr < 0.05f && !allowReverse) v = 0;
        if (v0 > 0 && v < 0 && thr > 0.05f) v = 0;
        if (v < -12.f) v = -12.f;
        if (std::fabs(v) < 0.08f && thr < 0.02f && brk < 0.02f) v = 0;
        longAcc = (v - v0) / std::max(dt, 1e-4f);
        // steering with slew-rate limit
        float rate = 2.4f * dt;
        steer += clampf(steerTarget - steer, -rate, rate);
        float lim = steerLimit(std::fabs(v)) * 1.25f;
        float dl = clampf(steer, -lim, lim);
        float omega = v / WB * std::tan(dl);
        float ay = v * omega, ayMax = aLatMax(std::fabs(v)) * gripF;
        if (std::fabs(ay) > ayMax && std::fabs(v) > 0.5f) {
            float ex = std::fabs(ay) - ayMax;
            omega = sgn(omega) * ayMax / std::fabs(v);
            v -= sgn(v) * std::min(std::fabs(v), ex * 0.12f * dt * 8.f);     // understeer scrub
            ay = sgn(ay) * ayMax;
        }
        speed = v; yawRate = omega; latAcc = ay;
        heading = wrapAngle(heading + omega * dt);
        pos += forward() * (speed * dt);
        for (int w = 0; w < 4; w++) spin[w] = std::fmod(spin[w] - speed * dt / WHEEL_R[w], 2 * PI_F);
        // tyre wear: longer/harder driving wears faster
        if (ctrl != CTRL_PIT && ctrl != CTRL_GRID)
            wear = std::min(1.f, wear + dt * (0.0016f + 0.00004f * std::fabs(ay) + 0.00005f * std::fabs(std::min(a, 0.f))) * (std::fabs(v) / 60.f));
    }
};

// ---------------------------------------------------------------------------
//  Racing driver: pure-pursuit steering on the spline + speed profile, with simple
//  racecraft - finds a car ahead, chooses a free passing lane, uses the slipstream
//  and returns to its own line afterwards.  Each car's pace drifts over time so
//  the running order really changes during the race.
// ---------------------------------------------------------------------------
inline void driverControl(const Track& tr, Car& c, const std::vector<Car>& cars, float dt, float time, float& thr, float& brk, float& steerTarget) {
    auto signedGap = [&](const Car& o) { float g = tr.wrapS(o.s - c.s); return g > tr.L * 0.5f ? g - tr.L : g; };
    const Car* lead = nullptr; float leadGap = 1e9f;
    for (const Car& o : cars) {
        if (o.id == c.id || o.ctrl == CTRL_PIT || o.ctrl == CTRL_GRID) continue;
        float g = signedGap(o) - 5.0f;                           // bumper-to-bumper
        if (g < -3.f || g > 42.f) continue;
        if (std::fabs(o.d - c.lineCur) < 2.7f && g < leadGap) { lead = &o; leadGap = g; }
    }
    if (lead) {
        float side = (lead->d > 0.f) ? -1.f : 1.f;
        float cand = lead->d + side * 3.1f;
        if (std::fabs(cand) > 5.0f) { side = -side; cand = lead->d + side * 3.1f; }
        cand = clampf(cand, -5.0f, 5.0f);
        bool free = true;
        for (const Car& o : cars) {
            if (o.id == c.id || o.id == lead->id || o.ctrl == CTRL_PIT || o.ctrl == CTRL_GRID) continue;
            if (std::fabs(signedGap(o)) < 13.f && std::fabs(o.d - cand) < 2.4f) free = false;
        }
        if (free) { c.lineTarget = cand; c.passHold = 2.6f; }
    } else if (c.passHold > 0.f) c.passHold -= dt;
    else c.lineTarget = c.lineBase;
    c.lineCur += clampf(c.lineTarget - c.lineCur, -2.4f * dt, 2.4f * dt);

    // steering: pure pursuit towards a point on the current lateral line
    float Ld = clampf(8.f + 0.5f * std::max(0.f, c.speed), 10.f, 48.f);
    vec3 tgt = tr.pos(c.s + Ld, c.lineCur);
    vec3 to = tgt - c.pos;
    float alpha = wrapAngle(std::atan2(to.z, to.x) - c.heading);
    steerTarget = std::atan2(2.f * Car::WB * std::sin(alpha), Ld);

    // speed: profile of the centre line, adjusted for the radius of the chosen lane
    float look = 4.f + c.speed * 0.15f, sl = c.s + look;
    float fac = std::sqrt(clampf(1.f - c.lineCur * tr.curvature(sl), 0.6f, 1.4f));
    float pace = c.skill * (1.f + 0.035f * std::sin(time * 0.13f + c.phase));
    float vt = tr.sample(c.prof, sl) * fac * pace;
    if (lead && leadGap < 38.f) vt *= 1.035f;                    // slipstream
    if (lead && leadGap < 11.f && std::fabs(lead->d - c.d) < 2.3f) vt = std::min(vt, std::max(8.f, lead->speed + (leadGap - 4.f) * 0.5f));
    float err = vt - c.speed;
    thr = clampf(err * 0.45f, 0.f, 1.f);
    brk = err < -1.0f ? clampf((-err - 1.f) * 0.30f, 0.f, 1.f) : 0.f;
    if (c.speed < vt * 0.3f) brk = 0;
}
