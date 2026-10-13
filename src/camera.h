// ============================================================================
//  camera.h  -  Nine camera views: cockpit, chase, trackside, overhead, pit,
//               broadcast, low chase, helicopter, head-on.
// ============================================================================
#pragma once
#include "world.h"

struct CamView { vec3 eye, at, up{0, 1, 0}; float fov = 60.f, znear = 0.4f; std::string name; };

struct CameraSystem {
    int mode = 2;                 // 1..9
    int pitView = 0;              // sub-view of the pit camera (key 5 cycles)
    float overH = 260.f;          // overhead height (PageUp / PageDown / +/-)
    float chaseDist = 11.f;
    vec3 chaseEye; bool chaseInit = false;
    int trackIdx = -1;
    std::vector<vec3> spots;      // fixed trackside camera positions
    float shake = 0;

    void init(const Track& tr, const World& w) {
        spots = w.camSpots;
        for (float s = 40.f; s < tr.L; s += 170.f) {
            int side = ((int)(s / 170.f) % 2) ? 1 : -1;
            float u = tr.sRel(s);
            if (side > 0 && u > -520.f && u < 400.f) side = -1;
            vec3 p = tr.pos(s, side * 24.f); p.y = 3.2f;
            spots.push_back(p);
        }
    }

    CamView compute(int forceMode, const Car& c, const Track& tr, const World& w, const PitStop& pit, float dt, float time) {
        CamView v;
        int m = forceMode ? forceMode : mode;
        vec3 fwd = c.forward(), up(0, 1, 0);
        vec3 cp = c.pos + vec3(0, c.lift, 0);
        if (m == 1) {        // ---- cockpit: first person from the driver's eyes
            vec3 eye = c.tilt->world.point({0.26f, 1.12f, 0});
            float la = c.steer * 0.3f;
            vec3 look = c.tilt->world.dir(vec3(std::cos(la), 0.f, std::sin(la)));
            v.eye = eye;
            v.at = eye + look * 40.f + vec3(0, -0.5f, 0);
            v.up = c.tilt->world.dir({0, 1, 0});
            v.fov = 78.f; v.znear = 0.08f; v.name = "COCKPIT";
        } else if (m == 2) { // ---- chase: smoothed third-person
            float dist = chaseDist + std::fabs(c.speed) * 0.035f;
            vec3 want = cp - fwd * dist + vec3(0, 3.5f + std::fabs(c.speed) * 0.01f, 0);
            if (!chaseInit) { chaseEye = want; chaseInit = true; }
            chaseEye = chaseEye + (want - chaseEye) * (1.f - std::exp(-dt * 7.f));
            v.eye = chaseEye + vec3(0, 0, 0); v.at = cp + fwd * 6.f + vec3(0, 0.9f, 0);
            v.fov = 62.f + clampf(c.speed * 0.12f, 0.f, 10.f); v.name = "CHASE";
        } else if (m == 3 || (m == 6 && false)) {   // ---- trackside: nearest fixed camera, zoom on the car
            v = trackside(c, "TRACKSIDE");
        } else if (m == 4) { // ---- overhead: top-down, zoom out to see the whole circuit
            float k = smoothstepf(380.f, 1100.f, overH);
            vec3 centre = mix(vec3(c.pos.x, 0, c.pos.z), tr.center, k);
            v.eye = vec3(centre.x, overH, centre.z + 0.001f); v.at = centre; v.up = vec3(0, 0, -1);
            v.fov = 50.f; v.znear = 2.f; v.name = "OVERHEAD";
        } else if (m == 5) { // ---- pit camera (3 angles)
            float bx = PitC::BOX_X[PitC::PLAYER_BOX], wz = PitC::WORK_D;
            float sway = std::sin(time * 0.35f);
            vec3 tgt(bx, 0.9f, wz);
            if (pitView == 0) { v.eye = vec3(bx - 8.5f + sway * 1.2f, 3.6f, 14.0f); v.at = tgt; v.name = "PIT CAMERA - PIT WALL"; v.fov = 52.f; }
            else if (pitView == 1) { v.eye = vec3(bx + 9.5f, 4.6f + sway * 0.4f, 31.5f); v.at = tgt + vec3(0, 0.3f, -1.f); v.name = "PIT CAMERA - GARAGE HIGH"; v.fov = 55.f; }
            else { v.eye = vec3(bx + 8.5f, 1.5f, 17.5f + sway); v.at = vec3(bx + 0.5f, 0.9f, wz); v.name = "PIT CAMERA - FRONT LOW"; v.fov = 58.f; }
            v.znear = 0.3f;
        } else if (m == 6) { // ---- broadcast: automatic director
            bool inPit = pit.stage >= PS_AT_BOX && pit.stage <= PS_LOWERED;
            if (inPit) {
                CamView p = compute(5, c, tr, w, pit, dt, time);
                int sub = ((int)(time / 4.f)) % 3;
                float bx = PitC::BOX_X[PitC::PLAYER_BOX], wz = PitC::WORK_D;
                float sway = std::sin(time * 0.35f);
                vec3 tgt(bx, 0.9f, wz);
                if (sub == 0) { v.eye = vec3(bx - 8.5f + sway * 1.2f, 3.6f, 14.0f); v.fov = 52.f; }
                else if (sub == 1) { v.eye = vec3(bx + 9.5f, 4.6f, 31.5f); v.fov = 55.f; }
                else { v.eye = vec3(bx + 8.5f, 1.5f, 17.5f + sway); v.fov = 58.f; }
                v.at = tgt; v.znear = 0.3f; v.name = "BROADCAST - PIT STOP"; (void)p;
            } else {
                int shot = ((int)(time / 7.f)) % 5;
                if (shot == 0 || shot == 4) { v = trackside(c, "BROADCAST - TRACKSIDE"); }
                else if (shot == 1) {   // helicopter orbit
                    float a = time * 0.25f;
                    v.eye = cp + vec3(std::cos(a) * 65.f, 38.f, std::sin(a) * 65.f); v.at = cp + fwd * 8.f;
                    v.fov = 45.f; v.name = "BROADCAST - HELICOPTER";
                } else if (shot == 2) { // head-on shot from the circuit ahead
                    vec3 p = tr.pos(c.s + 70.f + c.speed * 0.4f, 0); p.y = 1.3f;
                    v.eye = p + tr.normal(c.s + 70.f) * 6.f; v.at = cp; v.fov = clampf(2.f * std::atan(5.f / std::max(8.f, length(v.eye - cp))) * 57.3f, 12.f, 60.f);
                    v.name = "BROADCAST - HEAD ON";
                } else {                // low rear quarter
                    v.eye = cp - fwd * 6.5f + c.rightV() * 2.4f + vec3(0, 0.75f, 0); v.at = cp + fwd * 5.f + vec3(0, 0.4f, 0);
                    v.fov = 65.f; v.name = "BROADCAST - LOW CHASE";
                }
            }
        } else if (m == 7) { // ---- low chase: low rear-quarter view
            v.eye = cp - fwd * 6.5f + c.rightV() * 2.4f + vec3(0, 0.75f, 0);
            v.at = cp + fwd * 5.f + vec3(0, 0.4f, 0);
            v.fov = 65.f; v.name = "LOW CHASE";
        } else if (m == 8) { // ---- helicopter: orbiting aerial view
            float a = time * 0.25f;
            v.eye = cp + vec3(std::cos(a) * 65.f, 38.f, std::sin(a) * 65.f);
            v.at = cp + fwd * 8.f;
            v.fov = 45.f; v.name = "HELICOPTER";
        } else if (m == 9) { // ---- head-on: looking back at oncoming car
            vec3 p = tr.pos(c.s + 70.f + c.speed * 0.4f, 0); p.y = 1.3f;
            v.eye = p + tr.normal(c.s + 70.f) * 6.f;
            v.at = cp;
            v.fov = clampf(2.f * std::atan(5.f / std::max(8.f, length(v.eye - cp))) * 57.3f, 12.f, 60.f);
            v.name = "HEAD ON";
        } else {
            v = compute(2, c, tr, w, pit, dt, time);
        }
        return v;
    }
    CamView trackside(const Car& c, const char* name) {
        CamView v; vec3 cp = c.pos + vec3(0, 0.6f, 0);
        int best = 0; float bd = 1e18f;
        for (int i = 0; i < (int)spots.size(); i++) { float d = length(spots[i] - cp); if (d < bd) { bd = d; best = i; } }
        if (trackIdx < 0 || trackIdx >= (int)spots.size()) trackIdx = best;
        float cur = length(spots[trackIdx] - cp);
        if (cur > 260.f || (bd < cur * 0.55f && cur > 60.f)) trackIdx = best;
        v.eye = spots[trackIdx]; v.at = cp + c.forward() * 3.f;
        float dist = std::max(10.f, length(v.eye - cp));
        v.fov = clampf(2.f * std::atan(7.f / dist) * 57.2958f, 9.f, 55.f);
        v.name = name; v.znear = 0.5f;
        return v;
    }
    static float hash1(float x) { float s = std::sin(x * 12.9898f) * 43758.5453f; return s - std::floor(s); }
};
