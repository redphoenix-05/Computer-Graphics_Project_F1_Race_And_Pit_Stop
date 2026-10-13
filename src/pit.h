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


struct Crew {};
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

};
