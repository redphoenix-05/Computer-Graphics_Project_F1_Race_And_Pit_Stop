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

};
