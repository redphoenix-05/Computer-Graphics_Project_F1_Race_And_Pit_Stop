// ============================================================================
//  pit.h  -  Interactive pit-stop simulation
//
//  * PitStage state machine (entry -> lane -> box -> raise -> tyres -> lower -> exit)
//  * 8 animated crew members built as hierarchical humanoids (torso, head,
//    shoulder/elbow/hip/knee joints) driven by key-framed paths
//  * 2 hydraulic jacks, 4 old wheels (detach + fly off) and 4 new wheels
//    (carried / delivered and attached), wheel guns and the red/amber/green pit lights
//  The stop is user-driven: each stage waits for the right key (R, C, D, Q).
// ============================================================================
#pragma once
#include "car.h"

enum PitStage {
    PS_NONE = 0, PS_ENTRY, PS_LANE, PS_AT_BOX, PS_RAISING, PS_RAISED, PS_CHANGING,
    PS_CHANGED, PS_LOWERING, PS_LOWERED, PS_EXITING
};
static const char* PIT_STAGE_NAME[] = {"RACING", "PIT ENTRY", "PIT LANE", "AT PIT BOX", "RAISING CAR", "CAR RAISED",
                                       "TYRE CHANGE", "TYRES DONE", "LOWERING CAR", "READY TO RELEASE", "PIT EXIT"};

// ---------------------------------------------------------------------------
//  Humanoid crew member (hierarchical model)
// ---------------------------------------------------------------------------
struct Pose {
    float lSh = 0, rSh = 0, lAbd = 0, rAbd = 0, lEl = 0, rEl = 0, lHip = 0, rHip = 0, lKn = 0, rKn = 0;
    float crouch = 0, lean = 0;
};
enum Act { A_IDLE, A_RUN, A_CARRY, A_GUN, A_UP, A_JACK, A_SIGN };
struct Key { float t; vec3 pos; float yaw; Act act; };

struct Crew {
    Node *root, *pelvis, *lSh, *rSh, *lEl, *rEl, *lHip, *rHip, *lKn, *rKn, *hand, *gun;
    vec3 pos, prev; float yaw = 0, phase = 0, speed = 0;
    Act act = A_IDLE; bool gunning = false;
    vec3 standby; float standYaw = 0;
    std::vector<Key> keys;
    const char* role = "";

    void build(NodePool& P, const Prims& pr, const vec3& suit, const vec3& stripe, const vec3& helm, bool withGun) {
        root = P.make(); pelvis = P.make(root);
        auto part = [&](Node* parent, const Mesh* m, const mat4& l, const vec3& tint) {
            Node* n = P.make(parent, m); n->local = l; n->tint = tint; return n;
        };
        part(pelvis, &pr.cube, translate(0, 0.30f, 0) * scale(0.26f, 0.56f, 0.44f), suit);
        part(pelvis, &pr.cube, translate(0.132f, 0.36f, 0) * scale(0.012f, 0.10f, 0.40f), stripe);
        part(pelvis, &pr.cube, translate(0.0f, 0.02f, 0) * scale(0.27f, 0.10f, 0.40f), stripe * 0.6f);
        part(pelvis, &pr.sphereGloss, translate(0, 0.74f, 0) * scale(0.145f, 0.155f, 0.145f), helm);
        part(pelvis, &pr.cubeGloss, translate(0.12f, 0.745f, 0) * scale(0.07f, 0.05f, 0.15f), vec3(0.03f, 0.03f, 0.05f));
        part(pelvis, &pr.cyl, translate(0, 0.60f, 0) * scale(0.06f, 0.08f, 0.06f), vec3(0.8f, 0.6f, 0.5f));
        auto arm = [&](float z, Node*& sh, Node*& el) {
            sh = P.make(pelvis); sh->local = translate(0, 0.50f, z);
            part(sh, &pr.cyl, translate(0, -0.15f, 0) * scale(0.052f, 0.30f, 0.052f), suit);
            el = P.make(sh); el->local = translate(0, -0.30f, 0);
            part(el, &pr.cyl, translate(0, -0.14f, 0) * scale(0.045f, 0.28f, 0.045f), suit);
            part(el, &pr.sphere, translate(0, -0.30f, 0) * scale(0.052f), vec3(0.08f, 0.08f, 0.09f));
        };
        arm(-0.25f, lSh, lEl); arm(0.25f, rSh, rEl);
        hand = P.make(rEl); hand->local = translate(0, -0.30f, 0);
        gun = P.make(hand);
        gun->visible = withGun;
        part(gun, &pr.cube, translate(0, -0.20f, 0) * scale(0.07f, 0.34f, 0.07f), vec3(0.20f, 0.22f, 0.25f));
        part(gun, &pr.cyl, translate(0, -0.40f, 0) * scale(0.055f, 0.06f, 0.055f), vec3(0.85f, 0.85f, 0.2f));
        part(gun, &pr.cube, translate(0.0f, -0.05f, 0.0f) * scale(0.06f, 0.15f, 0.12f), vec3(0.9f, 0.25f, 0.1f));
        auto leg = [&](float z, Node*& hip, Node*& kn) {
            hip = P.make(pelvis); hip->local = translate(0, 0.0f, z);
            part(hip, &pr.cyl, translate(0, -0.215f, 0) * scale(0.075f, 0.43f, 0.075f), suit * 0.8f);
            kn = P.make(hip); kn->local = translate(0, -0.43f, 0);
            part(kn, &pr.cyl, translate(0, -0.215f, 0) * scale(0.06f, 0.43f, 0.06f), suit * 0.8f);
            part(kn, &pr.cube, translate(0.05f, -0.45f, 0) * scale(0.24f, 0.08f, 0.11f), vec3(0.05f, 0.05f, 0.06f));
        };
        leg(-0.10f, lHip, lKn); leg(0.10f, rHip, rKn);
    }
    void setPose(const Pose& p) {
        pelvis->local = translate(0, 0.88f - p.crouch, 0) * rotZ(-p.lean);
        lSh->local = translate(0, 0.50f, -0.25f) * rotX(p.lAbd) * rotZ(p.lSh);
        rSh->local = translate(0, 0.50f, 0.25f) * rotX(-p.rAbd) * rotZ(p.rSh);
        lEl->local = translate(0, -0.30f, 0) * rotZ(p.lEl);
        rEl->local = translate(0, -0.30f, 0) * rotZ(p.rEl);
        lHip->local = translate(0, 0, -0.10f) * rotZ(p.lHip);
        rHip->local = translate(0, 0, 0.10f) * rotZ(p.rHip);
        lKn->local = translate(0, -0.43f, 0) * rotZ(-p.lKn);
        rKn->local = translate(0, -0.43f, 0) * rotZ(-p.rKn);
    }
    // follow key-framed path; t = time since keys were set
    void follow(float t, float dt, float time) {
        if (!keys.empty()) {
            size_t k = 0;
            while (k + 1 < keys.size() && t >= keys[k + 1].t) k++;
            if (k + 1 >= keys.size()) { pos = keys.back().pos; yaw = keys.back().yaw; act = keys.back().act; }
            else {
                const Key &a = keys[k], &b = keys[k + 1];
                float u = smoothstepf(a.t, b.t, t);
                pos = mix(a.pos, b.pos, u);
                yaw = a.yaw + wrapAngle(b.yaw - a.yaw) * smoothstepf(a.t, std::min(b.t, a.t + 0.35f), t);
                act = a.act;
            }
        }
        float dist = length(pos - prev);
        speed = dt > 1e-5f ? dist / dt : 0.f;
        prev = pos;
        phase += dist * 2.6f;
        // ---- build the pose --------------------------------------------------
        Pose p;
        bool moving = speed > 0.45f;
        float sw = std::sin(phase);
        float amp = clampf(speed / 3.2f, 0.35f, 1.25f);
        if (moving) {
            p.lHip = sw * 0.85f * amp; p.rHip = -sw * 0.85f * amp;
            p.lKn = std::max(0.f, -sw) * 1.15f * amp; p.rKn = std::max(0.f, sw) * 1.15f * amp;
            p.lean = 0.14f * amp;
        }
        auto crouch = [&](float a) { p.lHip = p.rHip = 0.9f * a; p.lKn = p.rKn = 1.4f * a; p.crouch = 0.216f * a; };
        float jit = gunning ? std::sin(time * 55.f) * 0.035f : 0.f;
        switch (act) {
            case A_IDLE: case A_RUN:
                if (moving) { p.lSh = -sw * 0.9f * amp; p.rSh = sw * 0.9f * amp; p.lEl = p.rEl = 0.9f; }
                else { p.lSh = 0.05f; p.rSh = -0.05f; p.lAbd = p.rAbd = 0.08f; }
                break;
            case A_CARRY:
                p.lSh = p.rSh = 1.15f; p.lEl = p.rEl = 0.75f; p.lAbd = p.rAbd = 0.15f;
                if (!moving) p.lean = -0.05f;
                break;
            case A_GUN:
                if (moving) { p.lSh = -sw * 0.9f * amp; p.rSh = sw * 0.9f * amp; p.lEl = p.rEl = 0.9f; }
                else {
                    crouch(0.85f); p.lean = 0.35f;
                    p.lSh = 1.40f + jit; p.rSh = 1.45f - jit; p.lEl = 0.35f; p.rEl = 0.25f;
                }
                break;
            case A_UP:
                p.lSh = p.rSh = 2.85f; p.lAbd = p.rAbd = 0.25f; p.lEl = p.rEl = 0.1f;
                if (moving) { p.lean = 0.05f; }
                break;
            case A_JACK:
                if (!moving) { crouch(0.55f); p.lean = 0.25f; }
                p.lSh = 0.95f + jit * 3.f; p.rSh = 1.0f - jit * 3.f; p.lEl = 0.6f; p.rEl = 0.5f;
                break;
            case A_SIGN:
                p.rSh = 2.75f; p.rEl = 0.15f; p.lSh = moving ? -sw * 0.8f * amp : 0.1f; p.lEl = 0.4f;
                break;
        }
        setPose(p);
        root->local = translate(pos) * rotY(-yaw);
    }
};

// ---------------------------------------------------------------------------
//  Pit stop system
// ---------------------------------------------------------------------------
struct PitStop {
    // ---- geometry -----------------------------------------------------------
    float bx = PitC::BOX_X[PitC::PLAYER_BOX], wz = PitC::WORK_D;
    std::vector<vec3> path; std::vector<float> pathU; int pi = 0;   // pit path + its lane-x coordinate
    // ---- state ----------------------------------------------------------------
    PitStage stage = PS_NONE; float t = 0;            // time within the stage
    bool armed = false;                                // pit requested via P
    CarCtrl resumeCtrl = CTRL_AUTO;
    bool autoMode = false;                 // Y = automatic pit stop, U = manual (key-by-key) pit stop
    float animK = 1.f, tReal = 0.f;        // animation speed-up in auto mode, un-scaled stage time
    int stops = 0; float stopTime = 0, lastStop = 0, totalT = 0, exitAt = -100.f;
    std::string message; float messageT = 0;
    // ---- objects ------------------------------------------------------------------
    NodePool pool; Crew crew[8];
    Node *oldW[4], *newW[4];
    Node *jackN[2], *jackRam[2], *jackSaddle[2];
    Mesh jackBase, lamp;
    float lift = 0; float jackZ[2] = {26.4f, 26.4f};
    float liftK = 0;
    bool carWheelsNew[4] = {false, false, false, false};
    const CarAssets* assets = nullptr;
    int oldCompound = 1, newCompound = 2;
    float flash = 0;

    enum { FJ = 0, RJ, FL, FR, RL, RR, TD, EH };
    // staged / toss positions (world, XZ)
    vec3 stagedNew(int w) const {
        float out = WHEEL_Z[w] > 0 ? 1.f : -1.f;
        float x = bx + WHEEL_X[w] + (w < 2 ? 1.15f : -1.15f);
        return vec3(x, 0, wz + out * (out > 0 ? 2.6f : 3.1f));
    }
    vec3 tossSpot(int w) const {
        float out = WHEEL_Z[w] > 0 ? 1.f : -1.f;
        return vec3(bx + WHEEL_X[w] + (w < 2 ? 0.2f : -0.2f), 0, wz + out * (out > 0 ? 3.7f : 4.4f));
    }
    vec3 hub(int w, float lf) const { return vec3(bx + WHEEL_X[w], WHEEL_R[w] + lf, wz + WHEEL_Z[w]); }
    vec3 workSpot(int w) const { float out = WHEEL_Z[w] > 0 ? 1.f : -1.f; return vec3(bx + WHEEL_X[w], 0, wz + WHEEL_Z[w] + out * 0.95f); }
    float workYaw(int w) const { return WHEEL_Z[w] > 0 ? -PI_F * 0.5f : PI_F * 0.5f; }
    float tFetch(int w) const { static const float tf[4] = {2.75f, 2.45f, 3.35f, 2.55f}; return tf[w]; }   // FL FR RL RR

    // ----------------------------------------------------------------------------
    void build(const Prims& pr, const CarAssets& A, const Track& tr) {
        assets = &A;
        // pit-lane path (car follows this in the guided stages)
        float u0 = PitC::COMMIT_X0 - 40.f, u1 = PitC::EXIT_X1 + 60.f;
        for (float u = u0; u <= u1; u += 2.f) {
            float sRel = u;                       // x == arc length on the straight
            path.push_back(tr.pos(sRel, pitPathOffset(u)));
            pathU.push_back(u);
        }
        // crew
        vec3 suit = LIVERIES[PitC::PLAYER_BOX].main * 0.9f, stripe(0.95f);
        vec3 helm[8] = {vec3(0.95f, 0.85f, 0.1f), vec3(0.95f, 0.85f, 0.1f), vec3(0.2f, 0.45f, 0.95f), vec3(0.95f, 0.5f, 0.1f),
                        vec3(0.15f, 0.8f, 0.3f), vec3(0.85f, 0.2f, 0.85f), vec3(0.95f), vec3(0.95f, 0.25f, 0.2f)};
        const char* roles[8] = {"FRONT JACK", "REAR JACK", "WHEEL FL", "WHEEL FR", "WHEEL RL", "WHEEL RR", "TYRE DELIVERY", "EQUIPMENT"};
        vec3 sb[8] = {{bx + 4.6f, 0, 26.2f}, {bx - 4.6f, 0, 26.2f}, {bx + 1.6f, 0, 14.4f}, {bx + 1.6f, 0, 26.2f},
                      {bx - 1.6f, 0, 14.4f}, {bx - 1.6f, 0, 26.2f}, {bx + 7.5f, 0, 26.2f}, {bx + 6.5f, 0, 14.4f}};
        for (int i = 0; i < 8; i++) {
            bool gunner = (i >= 2 && i <= 5);
            crew[i].build(pool, pr, suit, stripe, helm[i], gunner);
            crew[i].role = roles[i];
            crew[i].standby = sb[i];
            crew[i].standYaw = (sb[i].z < 20.f) ? PI_F * 0.5f : -PI_F * 0.5f;
        }
        for (int w = 0; w < 4; w++) {
            oldW[w] = pool.make(nullptr, &A.wheel[w < 2 ? 0 : 1][oldCompound]); oldW[w]->visible = false;
            newW[w] = pool.make(nullptr, &A.wheel[w < 2 ? 0 : 1][newCompound]);
            pool.make(oldW[w])->decal = &A.wheelDecal[w < 2 ? 0 : 1];
            pool.make(newW[w])->decal = &A.wheelDecal[w < 2 ? 0 : 1];
        }
        // hydraulic jacks
        MeshBuilder jb;
        Mat steel = MT(vec3(0.75f, 0.12f, 0.1f), 0.5f, 40.f), dk = MT(vec3(0.08f, 0.08f, 0.09f), 0.4f, 30.f);
        jb.boxAt({0, 0.06f, 0}, {0.32f, 0.05f, 0.46f}, steel);
        jb.boxAt({0, 0.12f, 0}, {0.10f, 0.03f, 0.10f}, dk);
        for (int sx = -1; sx <= 1; sx += 2) for (int sz = -1; sz <= 1; sz += 2)
            jb.cyl(translate(sx * 0.26f, 0.045f, sz * 0.38f) * rotX(PI_F * 0.5f), 0.045f, 0.045f, 0.03f, 10, dk);
        jackBase.upload(jb);
        MeshBuilder hnd;   // lever handle (points along +X in local space, rotated per jack)
        hnd.tube({0, 0.14f, 0}, {1.7f, 0.95f, 0}, 0.022f, 0.022f, 8, dk);
        hnd.tube({1.55f, 0.88f, -0.14f}, {1.55f, 0.88f, 0.14f}, 0.026f, 0.026f, 8, MT(vec3(0.1f, 0.1f, 0.1f)));
        lamp.upload(hnd);
        for (int j = 0; j < 2; j++) {
            jackN[j] = pool.make(); jackN[j]->mesh = &jackBase;
            Node* h = pool.make(jackN[j], &lamp);
            h->local = (j == 0) ? mat4() : rotY(PI_F);        // handle points away from the car
            h->tint = vec3(1);
            jackRam[j] = pool.make(jackN[j], &pr.cylGloss); jackRam[j]->tint = vec3(0.7f, 0.72f, 0.75f);
            jackSaddle[j] = pool.make(jackN[j], &pr.cube); jackSaddle[j]->tint = vec3(0.1f, 0.1f, 0.12f);
        }
        resetCrew();
    }

    void resetCrew() {
        for (int i = 0; i < 8; i++) {
            crew[i].pos = crew[i].prev = crew[i].standby; crew[i].yaw = crew[i].standYaw;
            crew[i].keys.clear(); crew[i].act = A_IDLE; crew[i].gunning = false;
        }
        for (int w = 0; w < 4; w++) {
            carWheelsNew[w] = false;
            oldW[w]->visible = false; newW[w]->visible = true;
        }
        lift = 0; jackZ[0] = jackZ[1] = 26.6f;
    }

    // ----------------------------------------------------------------------------
    //  Public control (keys)
    // ----------------------------------------------------------------------------
    void say(const std::string& s, float dur = 3.0f) { message = s; messageT = dur; }
    bool inEntryWindow(float u) const { return u >= PitC::COMMIT_X0 && u <= PitC::COMMIT_X1; }

    // mode: 0 = keep the current pit mode (P key), 1 = automatic (Y key), 2 = manual (U key)
    void request(Car& c, const Track& tr, int mode) {
        bool prevAuto = autoMode;
        if (mode == 1) autoMode = true; else if (mode == 2) autoMode = false;
        if (stage != PS_NONE) {                       // already in the pit: Y / U may switch mode on the fly
            if (mode == 1) say("AUTOMATIC pit stop - the crew handles everything");
            else if (mode == 2) say("MANUAL pit stop - use R, C, D, Q");
            else say("Already in the pit procedure");
            return;
        }
        if (armed && autoMode == prevAuto) { armed = false; say("Pit request cancelled"); return; }
        armed = true;
        std::string m = autoMode ? "AUTOMATIC" : "MANUAL";
        if (inEntryWindow(tr.sRel(c.s))) say(m + " PIT STOP - entering the pit lane");
        else say(m + " PIT STOP REQUESTED - take the pit entry on the main straight", 4.f);
    }
    void keyRaise() {
        if (stage == PS_AT_BOX) setStage(PS_RAISING);
        else say(stage == PS_NONE ? "R: no pit stop in progress" : "R: car must be stopped in the pit box first");
    }
    void keyChange() {
        if (stage == PS_RAISED) setStage(PS_CHANGING);
        else say(stage < PS_RAISED ? "C: raise the car first (R)" : "C: tyres cannot be changed now");
    }
    void keyLower() {
        if (stage == PS_CHANGED) setStage(PS_LOWERING);
        else say(stage < PS_CHANGED ? "D: finish the tyre change first (C)" : "D: car is not raised");
    }
    void keyExit() {
        if (stage == PS_LOWERED) setStage(PS_EXITING);
        else say(stage < PS_LOWERED ? "Q: lower the car first (D)" : "Q: not in the pit box");
    }

    void keysFor(int i, std::vector<Key> k) { crew[i].keys = std::move(k); }
    Key K(float tt, vec3 p, float yaw, Act a) { return Key{tt, p, yaw, a}; }

    void setStage(PitStage s) {
        stage = s; t = 0; tReal = 0;
        auto cur = [&](int i, Act a) { return K(0, crew[i].pos, crew[i].yaw, a); };
        if (s == PS_RAISING) {
            say("Jacks going in...", 1.5f);
            keysFor(FJ, {cur(FJ, A_IDLE), K(0.25f, vec3(bx + 3.9f, 0, 26.2f), PI_F, A_RUN), K(0.9f, vec3(bx + 3.9f, 0, wz), PI_F, A_JACK)});
            keysFor(RJ, {cur(RJ, A_IDLE), K(0.25f, vec3(bx - 3.9f, 0, 26.2f), 0, A_RUN), K(0.9f, vec3(bx - 3.9f, 0, wz), 0, A_JACK)});
        } else if (s == PS_CHANGING) {
            say("Changing tyres...", 1.5f);
            for (int w = 0; w < 4; w++) {
                int id = (w == 0) ? FL : (w == 1) ? FR : (w == 2) ? RL : RR;
                vec3 W = workSpot(w); float yw = workYaw(w);
                float out = WHEEL_Z[w] > 0 ? 1.f : -1.f;
                std::vector<Key> k;
                k.push_back(cur(id, A_IDLE));
                k.push_back(K(0.9f, W, yw, A_GUN));             // approach
                k.push_back(K(2.4f, W, yw, A_GUN));             // loosen + remove old wheel
                float tf = tFetch(w);
                vec3 st = stagedNew(w); vec3 S = st + vec3(0, 0, out * 0.95f);
                k.push_back(K(tf, S, yw, A_RUN));               // fetch new wheel
                const int NK = 6;
                for (int q = 1; q <= NK; q++) {                 // carry along the wheel's arc
                    float u = (float)q / NK;
                    vec3 hp = mix(st, vec3(hub(w, 0.3f).x, 0, hub(w, 0.3f).z), smoothstepf(0, 1, u));
                    k.push_back(K(tf + 0.9f * u, hp + vec3(0, 0, out * 0.95f), yw, A_CARRY));
                }
                k.push_back(K(tf + 0.95f, W, yw, A_GUN));      // tighten
                k.push_back(K(tf + 1.75f, W, yw, A_GUN));
                k.push_back(K(tf + 1.80f, W, yw, A_UP));        // thumbs up
                keysFor(id, k);
            }
            // tyre delivery: rack -> front of car -> left-side wheels
            vec3 rack(bx + 8.8f, 0, 27.0f);
            vec3 stFL = stagedNew(0), stRL = stagedNew(2);
            keysFor(TD, {cur(TD, A_IDLE), K(0.5f, rack, PI_F * 0.5f, A_RUN), K(0.7f, rack, PI_F * 0.5f, A_CARRY),
                         K(1.9f, vec3(bx + 8.8f, 0, 19.2f), PI_F, A_CARRY),
                         K(2.5f, vec3(stFL.x, 0, 18.4f), PI_F, A_CARRY), K(2.8f, vec3(stFL.x, 0, 18.4f), PI_F, A_CARRY),
                         K(3.3f, vec3(stRL.x, 0, 18.4f), PI_F, A_CARRY), K(3.6f, vec3(stRL.x, 0, 18.4f), PI_F, A_IDLE)});
            keysFor(FJ, {cur(FJ, A_JACK), K(10, crew[FJ].pos, crew[FJ].yaw, A_JACK)});
            keysFor(RJ, {cur(RJ, A_JACK), K(10, crew[RJ].pos, crew[RJ].yaw, A_JACK)});
            for (int w = 0; w < 4; w++) { oldW[w]->visible = false; carWheelsNew[w] = false; newW[w]->visible = true; }
        } else if (s == PS_LOWERING) {
            say("Lowering car...", 1.5f);
            for (int w = 0; w < 4; w++) {
                int id = (w == 0) ? FL : (w == 1) ? FR : (w == 2) ? RL : RR;
                float out = WHEEL_Z[w] > 0 ? 1.f : -1.f;
                vec3 back = crew[id].pos + vec3(0, 0, out * 1.1f);
                keysFor(id, {cur(id, A_UP), K(0.25f, crew[id].pos, crew[id].yaw, A_UP), K(1.0f, back, workYaw(w), A_IDLE)});
            }
            keysFor(TD, {cur(TD, A_IDLE), K(10, crew[TD].pos, crew[TD].yaw, A_IDLE)});
            keysFor(FJ, {cur(FJ, A_JACK), K(0.7f, crew[FJ].pos, PI_F, A_JACK), K(1.7f, vec3(bx + 3.9f, 0, 26.2f), PI_F, A_JACK)});
            keysFor(RJ, {cur(RJ, A_JACK), K(0.7f, crew[RJ].pos, 0, A_JACK), K(1.7f, vec3(bx - 3.9f, 0, 26.2f), 0, A_JACK)});
        } else if (s == PS_EXITING) {
            say("RELEASED - GO GO GO!", 2.0f);
            for (int i = 0; i < 8; i++) {
                float d = length(crew[i].standby - crew[i].pos);
                float dur = std::max(0.9f, d / 4.0f);
                std::vector<Key> k{cur(i, A_IDLE)};
                k.push_back(K(0.3f + dur, crew[i].standby, crew[i].standYaw, A_RUN));
                k.push_back(K(0.4f + dur, crew[i].standby, crew[i].standYaw, A_IDLE));
                keysFor(i, k);
            }
        }
    }

    // ----------------------------------------------------------------------------
    //  Per-frame update for the player's car while it is in the pit procedure.
    //  Returns true while the pit system has control of the car.
    // ----------------------------------------------------------------------------
    bool guidedControl(Car& c, const Track& /*tr*/, float /*dt*/, float& thr, float& brk, float& steerT) {
        // advance along path
        int bi = pi; float bd = 1e18f;
        for (int j = pi; j < (int)path.size() && j < pi + 40; j++) {
            float dx = path[j].x - c.pos.x, dz = path[j].z - c.pos.z, dd = dx * dx + dz * dz;
            if (dd < bd) { bd = dd; bi = j; }
        }
        pi = bi;
        float Ld = clampf(6.f + 0.45f * std::fabs(c.speed), 7.f, 30.f);
        int j = pi; float acc = 0;
        while (j + 1 < (int)path.size() && acc < Ld) { acc += length(path[j + 1] - path[j]); j++; }
        vec3 tg = path[j];
        if (stage == PS_LANE && j + 1 >= (int)path.size()) tg = path.back();
        vec3 to = tg - c.pos;
        float alpha = wrapAngle(std::atan2(to.z, to.x) - c.heading);
        steerT = std::atan2(2.f * Car::WB * std::sin(alpha), Ld);
        float vt = PitC::LIMIT;
        if (stage == PS_ENTRY || stage == PS_LANE) {
            float rem = bx - c.pos.x;
            // braking curve: the pit-lane limiter applies from the pit-entry line (end of the entry ramp)
            float toLine = std::max(0.f, (PitC::ENTRY_X1 - 5.f) - c.pos.x);
            vt = std::sqrt(PitC::LIMIT * PitC::LIMIT + 2.f * 30.f * toLine);
            vt = std::min(vt, std::sqrt(2.f * 9.f * std::max(0.f, rem - 0.25f)) + 0.6f);
        } else if (stage == PS_EXITING) {
            float ux = c.pos.x;
            vt = (ux < PitC::EXIT_X0 + 10.f) ? PitC::LIMIT : 90.f;
            vt = std::min(vt, 3.f + tReal * 14.f);          // gentle launch
        }
        float err = vt - c.speed;
        thr = clampf(err * 0.5f, 0.f, 1.f);
        brk = err < -0.5f ? clampf((-err - 0.5f) * 0.35f, 0.f, 1.f) : 0.f;
        return true;
    }

    // call every simulation step; handles stage transitions and animation.
    // Returns pit control state for the car in 'ctrlOut'.
    void update(Car& c, const Track& tr, float dt, float time, const Car* /*unused*/ = nullptr) {
        totalT += dt;
        if (messageT > 0) messageT -= dt;
        animK = 1.f;
        if (autoMode && stage >= PS_AT_BOX && stage <= PS_LOWERED) animK = 3.2f;     // whole stop in about 3 s
        else if (autoMode && stage == PS_EXITING) animK = 2.0f;
        t += dt * animK; tReal += dt; flash += dt;
        float u = tr.sRel(c.s);
        // --- arming: commit to the pit entry --------------------------------
        if (stage == PS_NONE && armed && c.ctrl != CTRL_GRID && inEntryWindow(u) && std::fabs(c.d) < 12.f) {
            stage = PS_ENTRY; t = 0; armed = false; resumeCtrl = c.ctrl; c.ctrl = CTRL_PIT; pi = 0;
            resetCrew();
            oldCompound = c.compound; newCompound = (c.compound + 1) % 3;
            for (int w = 0; w < 4; w++) {
                oldW[w]->mesh = &assets->wheel[w < 2 ? 0 : 1][oldCompound];
                newW[w]->mesh = &assets->wheel[w < 2 ? 0 : 1][newCompound];
                c.wheelN[w]->visible = true;
            }
            // start of path search: nearest path point
            float bd = 1e18f;
            for (int j = 0; j < (int)path.size(); j++) { float dd = length(path[j] - c.pos); if (dd < bd) { bd = dd; pi = j; } }
            say("PIT ENTRY - pit lane speed limiter ON");
            stopTime = 0;
        }
        if (stage == PS_ENTRY && c.pos.x > PitC::ENTRY_X1 + 5.f) { stage = PS_LANE; t = 0; say("Pit lane - approaching your box"); }
        if (stage == PS_ENTRY || stage == PS_LANE) {
            if ((c.pos.x > bx - 0.3f && c.speed < 0.6f) || c.pos.x >= bx - 0.05f) {
                // arrived: snap exactly into the box
                c.pos = vec3(bx, 0, wz); c.heading = 0; c.speed = 0; c.steer = 0;
                stage = PS_AT_BOX; t = 0; tReal = 0; stopTime = 0; stops++;
                say(autoMode ? "AT PIT BOX - automatic stop" : "AT PIT BOX - press R to raise the car", 5.f);
            }
        }
        if (stage >= PS_AT_BOX && stage < PS_EXITING) stopTime += dt;

        // --- stage animation ---------------------------------------------------
        float liftTarget = lift;
        if (stage == PS_RAISING) {
            lift = 0.30f * smoothstepf(0.9f, 1.9f, t);
            jackZ[0] = jackZ[1] = mixf(26.6f, wz, smoothstepf(0.25f, 0.9f, t));
            if (t >= 1.95f) { setStage(PS_RAISED); say(autoMode ? "" : "CAR RAISED - press C to change tyres", 5.f); }
        } else if (stage == PS_RAISED) {
            lift = 0.30f; jackZ[0] = jackZ[1] = wz;
        } else if (stage == PS_CHANGING) {
            lift = 0.30f; jackZ[0] = jackZ[1] = wz;
            animateWheels(c, t);
            if (t >= 5.45f) {
                stage = PS_CHANGED; t = 0; tReal = 0; c.wear = 0; say(autoMode ? "" : "TYRES CHANGED - press D to lower the car", 5.f);
            }
        } else if (stage == PS_CHANGED) {
            lift = 0.30f;
            animateWheels(c, 99.f);
        } else if (stage == PS_LOWERING) {
            lift = 0.30f * (1.f - smoothstepf(0.0f, 0.7f, t));
            jackZ[0] = jackZ[1] = mixf(wz, 26.6f, smoothstepf(0.7f, 1.7f, t));
            if (t >= 1.9f) { stage = PS_LOWERED; t = 0; tReal = 0; lastStop = stopTime; say(autoMode ? "" : "GREEN LIGHT - press Q to exit the pit", 5.f); }
        } else if (stage == PS_LOWERED) {
            lift = 0; jackZ[0] = jackZ[1] = 26.6f;
        } else if (stage == PS_EXITING) {
            lift = 0; jackZ[0] = jackZ[1] = 26.6f;
            if (c.pos.x >= PitC::EXIT_X1 + 10.f) {
                stage = PS_NONE; c.ctrl = resumeCtrl; t = 0; c.inLane = false; exitAt = totalT;
                say("Back on track - race resumed", 3.f);
            }
        }
        c.lift = lift;
        // wheel gun vibration while working
        bool gun = (stage == PS_CHANGING);
        for (int i = 2; i < 6; i++) crew[i].gunning = gun && ((t > 0.9f && t < 1.8f) || t > 4.0f);
        crew[FJ].gunning = crew[RJ].gunning = (stage == PS_RAISING && t > 0.9f) || (stage == PS_LOWERING && t < 0.7f);
        // --- automatic pit stop (Y): the crew performs every stage without key presses
        if (autoMode && tReal > 0.10f) {
            if (stage == PS_AT_BOX) setStage(PS_RAISING);
            else if (stage == PS_RAISED) setStage(PS_CHANGING);
            else if (stage == PS_CHANGED) setStage(PS_LOWERING);
            else if (stage == PS_LOWERED) setStage(PS_EXITING);
        }
        // --- animate crew -------------------------------------------------------
        for (int i = 0; i < 8; i++) crew[i].follow(t, dt * animK, time);
        for (int i = 0; i < 8; i++) crew[i].root->update(mat4());
        // jacks
        for (int j = 0; j < 2; j++) {
            float sx = (j == 0) ? 2.35f : -2.35f;
            jackN[j]->local = translate(bx + sx, 0, jackZ[j]);
            float contact = (j == 0) ? 0.150f : 0.235f;   // nose / gearbox underside heights
            float ramTop = contact + lift;
            float ramLen = std::max(0.02f, ramTop - 0.12f);
            jackRam[j]->local = translate(0, 0.12f + ramLen * 0.5f, 0) * scale(0.04f, ramLen, 0.04f);
            jackSaddle[j]->local = translate(0, ramTop + 0.012f, 0) * scale(0.18f, 0.025f, 0.24f);
            jackN[j]->update(mat4());
        }
        (void)liftTarget;
    }

    // wheel removal / delivery / mounting animation (t = time since the C key)
    void animateWheels(Car& c, float tt) {
        for (int w = 0; w < 4; w++) {
            float out = WHEEL_Z[w] > 0 ? 1.f : -1.f;
            mat4 flip = (WHEEL_Z[w] < 0) ? rotY(PI_F) : mat4();
            float tr0 = 1.8f, tr1 = 2.4f;
            float tf = tFetch(w);
            // old wheel -----------------------------------------------------
            if (tt < tr0) {
                c.wheelN[w]->visible = true; oldW[w]->visible = false;
                if (tt > 0.9f) c.spin[w] += std::sin(tt * 90.f) * 0.004f;     // vibration while loosening
            } else {
                c.wheelN[w]->visible = false; oldW[w]->visible = true;
                float u = clampf((tt - tr0) / (tr1 - tr0), 0.f, 1.f);
                vec3 h = hub(w, 0.30f), toss = tossSpot(w);
                float e = u * (2.f - u);
                float hx = mixf(h.x, toss.x, e), hz = mixf(h.z, toss.z, e);
                float hy = mixf(h.y, WHEEL_W[w] * 0.5f, u * u) + 0.9f * 4.f * u * (1.f - u);
                float tilt = (PI_F * 0.5f) * u * u * out * -1.f;
                oldW[w]->override_ = translate(hx, hy, hz) * rotX(tilt) * flip * rotZ(u * 9.f * (1.f - u));
                oldW[w]->useOverride = true;
                oldW[w]->update(mat4());
            }
            // new wheel -----------------------------------------------------
            if (carWheelsNew[w] || tt >= tf + 0.9f) {
                if (!carWheelsNew[w]) {
                    carWheelsNew[w] = true;
                    c.setCompound(*assets, newCompound); c.spin[w] = 0;
                }
                c.wheelN[w]->visible = true; newW[w]->visible = false;
                continue;
            }
            vec3 st = stagedNew(w);
            float h0 = WHEEL_R[w];
            if (tt < tf) {
                // waiting (right wheels) or on the delivery stack (left wheels)
                if (out < 0) {
                    // left wheels ride on the tyre-delivery man's stack until placed
                    vec3 tp = crew[TD].pos;
                    float place = (w == 0) ? 2.55f : 3.15f;   // when each wheel is put down
                    if (tt < 0.7f) {
                        vec3 rack(bx + 8.8f, 0, 27.0f);
                        newW[w]->override_ = translate(rack.x + (w == 0 ? 0.0f : 0.05f), 0.20f + (w == 0 ? 0.0f : 0.42f), rack.z + 0.9f) * rotX(PI_F * 0.5f) * flip;
                    } else if (tt < place) {
                        newW[w]->override_ = translate(tp.x + 0.45f, 1.0f + (w == 0 ? 0.0f : 0.42f), tp.z) * rotX(PI_F * 0.5f) * flip;
                    } else {
                        float u = smoothstepf(place, place + 0.3f, tt);
                        vec3 from(tp.x + 0.45f, 1.0f + (w == 0 ? 0.0f : 0.42f), tp.z);
                        vec3 to(st.x, h0, st.z);
                        vec3 pp = mix(from, to, u);
                        newW[w]->override_ = translate(pp) * rotX((PI_F * 0.5f) * (1.f - u)) * flip;
                    }
                } else {
                    newW[w]->override_ = translate(st.x, h0, st.z) * flip;
                }
            } else {
                float u = smoothstepf(0.f, 0.9f, tt - tf);
                vec3 to = hub(w, 0.30f);
                vec3 pp = mix(vec3(st.x, h0, st.z), to, u);
                pp.y += 0.35f * 4.f * u * (1.f - u);
                float jitter = (tt - tf > 0.9f) ? std::sin(tt * 80.f) * 0.003f : 0.f;
                newW[w]->override_ = translate(pp) * flip * rotZ(jitter);
            }
            newW[w]->useOverride = true; newW[w]->visible = true;
            newW[w]->update(mat4());
        }
    }

    // Reset all wheel nodes when no stop is going on (before a new stop)
    void idleWheels(Car& c) {
        for (int w = 0; w < 4; w++) {
            if (stage == PS_NONE) {
                c.wheelN[w]->visible = true;
                vec3 st = stagedNew(w);
                mat4 flip = (WHEEL_Z[w] < 0) ? rotY(PI_F) : mat4();
                if (WHEEL_Z[w] > 0) newW[w]->override_ = translate(st.x, WHEEL_R[w], st.z) * flip;
                else newW[w]->override_ = translate(bx + 8.8f, 0.20f + (w == 0 ? 0.f : 0.42f), 27.9f) * rotX(PI_F * 0.5f) * flip;
                newW[w]->useOverride = true; newW[w]->visible = true; newW[w]->update(mat4());
            }
        }
    }

    void draw(Renderer& r) {
        for (int i = 0; i < 8; i++) crew[i].root->draw(r);
        for (int w = 0; w < 4; w++) { oldW[w]->draw(r); newW[w]->draw(r); }
        for (int j = 0; j < 2; j++) jackN[j]->draw(r);
    }

    // dynamic point lights belonging to the pit-light gantry
    void lights(std::vector<LightSrc>& out, float lampScale) const {
        bool red = (stage >= PS_AT_BOX && stage <= PS_LOWERING);
        bool green = (stage == PS_LOWERED || stage == PS_EXITING);
        bool amber = ((int)(flash * 5.f) % 2 == 0) && (stage >= PS_RAISING && stage <= PS_LOWERING);
        LightSrc L; L.radius = 16.f; L.spot = false;
        L.pos = vec3(bx + 9.f, 4.4f, 18.f);
        if (red) { L.col = vec3(1.8f, 0.12f, 0.08f) * lampScale; out.push_back(L); }
        else if (green) { L.col = vec3(0.1f, 1.6f, 0.2f) * lampScale; out.push_back(L); }
        if (amber) { LightSrc A = L; A.pos = vec3(bx - 3.f, 2.2f, 12.6f); A.col = vec3(2.2f, 1.2f, 0.1f) * lampScale; A.radius = 14.f; out.push_back(A); }
    }
    bool redLamp() const { return stage >= PS_AT_BOX && stage <= PS_LOWERING; }
    bool greenLamp() const { return stage == PS_LOWERED || stage == PS_EXITING; }
    bool amberLamp() const { return ((int)(flash * 5.f) % 2 == 0) && (stage >= PS_RAISING && stage <= PS_LOWERING); }
    int hudStep() const {   // 0..8 : number of completed steps in the report's 8-step list
        switch (stage) {
            case PS_NONE: return armed ? 0 : 0;
            case PS_ENTRY: return 0;
            case PS_LANE: return 1;
            case PS_AT_BOX: return 2;
            case PS_RAISING: case PS_RAISED: return 3;
            case PS_CHANGING: case PS_CHANGED: return 4;
            case PS_LOWERING: case PS_LOWERED: return 5;
            case PS_EXITING: return 6;
        }
        return 0;
    }
};
