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

};
