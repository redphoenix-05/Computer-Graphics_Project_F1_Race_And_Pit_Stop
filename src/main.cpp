// ============================================================================
//  Real-Time 3D Formula 1 Racing and Interactive Pit Stop Simulation
//  Computer Graphics course project  -  C++17 / OpenGL 3.3 core / GLFW / GLAD
//
//  Source layout (single translation unit, header-only modules):
//      math3d.h    vectors, 4x4 matrices, transformations
//      gfx.h       shaders (Gouraud + Phong), mesh builder, renderer, HUD
//      track.h     circuit spline, speed profile, pit-lane geometry
//      car.h       hierarchical F1 models, vehicle dynamics, driver AI
//      pit.h       interactive pit stop (crew, jacks, wheels, state machine)
//      world.h     procedural venue: road, barriers, garages, stands ...
//      camera.h    nine camera views
//      raytrace.h  optional ray-tracing demonstration
// ============================================================================
#include "camera.h"
#include "raytrace.h"
#include "audio.h"
#include <chrono>
#include <deque>
#include <fstream>
#include <sstream>

// ---------------------------------------------------------------------------
//  Lighting presets (L key): day / dusk / night
// ---------------------------------------------------------------------------
static const Preset PRESETS[3] = {
    {"DAY", vec3(0.45f, 0.75f, 0.35f), vec3(2.5f, 2.35f, 2.1f), vec3(0.50f, 0.62f, 0.85f), vec3(0.26f, 0.28f, 0.22f),
     vec3(0.07f, 0.20f, 0.60f), vec3(0.58f, 0.70f, 0.88f), vec3(0.76f, 0.84f, 0.92f), 0.00050f, 0.f, 0.f, vec3(0.95f)},
    {"DUSK", vec3(0.85f, 0.20f, 0.30f), vec3(2.3f, 1.05f, 0.45f), vec3(0.36f, 0.30f, 0.42f), vec3(0.16f, 0.12f, 0.12f),
     vec3(0.04f, 0.07f, 0.22f), vec3(0.95f, 0.45f, 0.20f), vec3(0.70f, 0.48f, 0.40f), 0.00065f, 0.55f, 0.15f, vec3(0.75f, 0.40f, 0.30f)},
    {"NIGHT", vec3(-0.3f, 0.55f, 0.45f), vec3(0.16f, 0.20f, 0.38f), vec3(0.07f, 0.09f, 0.18f), vec3(0.025f, 0.03f, 0.05f),
     vec3(0.002f, 0.005f, 0.03f), vec3(0.02f, 0.04f, 0.10f), vec3(0.08f, 0.11f, 0.20f), 0.00075f, 1.0f, 1.0f, vec3(0.05f, 0.06f, 0.12f)}};

enum RaceState { RS_COUNT = 0, RS_RACE };

struct Game;
static Game* G = nullptr;

struct Game {
    // ---- subsystems -------------------------------------------------------
    Track tr; World world; Prims prims; CarAssets assets; NodePool pool; PitStop pit;
    Renderer R; Hud hud; CameraSystem cam; RayTracer rt; AudioSystem audio;
    std::vector<Car> cars;
    GLFWwindow* win = nullptr;
    int fbw = 1280, fbh = 720;
    // ---- state ----------------------------------------------------------------
    RaceState race = RS_COUNT; float raceT = 0, simTime = 0, realTime = 0, goDelay = 1.4f;
    int startLights = 0; bool lightsOut = false;
    CarCtrl driveMode = CTRL_AUTO;
    bool paused = false, showHelp = true, showHud = true, rtMode = false, fullscreen = false;
    int camMode = 2, preset = 0; float helpTimer = 14.f;
    float steerIn = 0, shakeT = 0;
    int restarts = 0;
    float fps = 60, fpsAcc = 0; int fpsN = 0;
    std::string toast; float toastT = 0;
    bool keyOverride[512] = {};          // scripted key holds (test harness)
    float holdUntil[512] = {};
    std::deque<int> keyQueue;
    // test harness
    bool testMode = false; double testTime = 0;
    struct Cmd { double t; std::string c, a, b; bool done; };
    std::vector<Cmd> script;
    // diagnostics (test harness)
    bool viewOverride = false; vec3 ovEye, ovAt; float ovFov = 40.f;
    struct Diag { vec3 prevPos; float prevH = 0, prevV = 0; int posJump = 0, hdgJump = 0, spdJump = 0, hits = 0; float maxD = 0; } diag[5];
    std::vector<int> lastOrder; int orderChanges = 0; float orderT = 0;

    Car& player() { return cars[PitC::PLAYER_BOX]; }

    // ------------------------------------------------------------------------
    //  initialisation
    // ------------------------------------------------------------------------
    void init() {
        double t0 = glfwGetTime();
        R.init(); hud.init(); R.fontTex = hud.tex; prims.build(); assets.build();
        printf("  shaders + car models: %.2fs\n", glfwGetTime() - t0);
        tr.build();
        printf("  track: %.2fs\n", glfwGetTime() - t0);
        world.build(tr);
        printf("  world: %.2fs\n", glfwGetTime() - t0);
        pit.build(prims, assets, tr);
        // safety check: the guided pit path must keep clear of every upright obstacle near the pit lane
        int bad = 0;
        for (const vec3& pl : world.poles)
            for (const vec3& q : pit.path) {
                float d = std::sqrt((q.x - pl.x) * (q.x - pl.x) + (q.z - pl.y) * (q.z - pl.y));
                if (d < pl.z + 1.6f) { bad++; printf("WARNING: pit path passes %.2f m from an obstacle at (%.1f, %.1f)\n", d, pl.x, pl.y); break; }
            }
        printf("  pit path clearance check: %zu obstacles, %d conflicts\n", world.poles.size(), bad);
        cars.resize(5);
        static const float offs[5] = {-2.6f, 2.6f, -2.6f, 2.6f, -2.6f};
        static const float skill[5] = {0.996f, 0.994f, 0.992f, 0.990f, 0.988f};
        for (int i = 0; i < 5; i++) {
            Car& c = cars[i];
            c.id = i; c.player = (i == PitC::PLAYER_BOX); c.liv = &LIVERIES[i];
            c.lineBase = c.lineCur = c.lineTarget = offs[i]; c.skill = skill[i]; c.phase = 1.9f * i;
            c.prof = tr.speedProfile(0.f, 24.f * 0.935f, 22.f / 6400.f, 86.f, 10.5f, 34.f);
            c.buildHierarchy(pool, assets);
        }
        rt.init();
        cam.init(tr, world);
        audio.init();
        resetRace();
        printf("Track length %.0f m | static triangles %.0f | start-up OK\n", tr.L, world.stat);
    }

    void resetRace() {
        static const float offs[5] = {-2.6f, 2.6f, -2.6f, 2.6f, -2.6f};
        for (int i = 0; i < 5; i++) {
            Car& c = cars[i];
            c.lineCur = c.lineTarget = c.lineBase; c.passHold = 0;
            c.pos = vec3(-10.f - 9.f * i, 0, offs[i]); c.heading = 0; c.speed = 0; c.steer = 0;
            c.latAcc = c.longAcc = c.yawRate = 0; c.lift = 0; c.wear = 0; c.lapsDone = 0; c.lastLap = c.bestLap = 0;
            c.idx = -1; c.hitCooldown = 0; c.inLane = false; c.ctrl = CTRL_GRID;
            for (int w = 0; w < 4; w++) c.spin[w] = 0;
            c.setCompound(assets, 1);
            Proj p = tr.project(c.pos, -1); c.idx = p.i; c.s = c.prevS = p.s; c.d = p.d;
            c.distTotal = c.s - tr.L;
        }
        pit.stage = PS_NONE; pit.armed = false; pit.resetCrew(); pit.message.clear(); pit.messageT = 0; pit.stops = 0; pit.pi = 0;
        for (int w = 0; w < 4; w++) { pit.oldW[w]->visible = false; }
        race = RS_COUNT; raceT = 0; startLights = 0; lightsOut = false; steerIn = 0;
        goDelay = 0.9f + 1.2f * World::rnd(restarts++, 77);
        cam.chaseInit = false;
    }

    bool isDown(int key) {
        if (key < 0 || key >= 512) return false;
        if (testMode) return keyOverride[key] && holdUntil[key] > testTime;
        return glfwGetKey(win, key) == GLFW_PRESS;
    }

    void setMode(CarCtrl m) {
        driveMode = m;
        Car& p = player();
        if (p.ctrl == CTRL_AUTO || p.ctrl == CTRL_MANUAL) p.ctrl = m;
        else if (p.ctrl == CTRL_PIT) pit.resumeCtrl = m;
        say(m == CTRL_AUTO ? "AUTOMATIC DRIVING MODE" : "MANUAL DRIVING MODE  -  arrows to drive");
    }
    void say(const std::string& s, float d = 2.2f) { toast = s; toastT = d; }

    // ------------------------------------------------------------------------
    //  keyboard
    // ------------------------------------------------------------------------
    void onKey(int key) {
        if (key == GLFW_KEY_ESCAPE) { glfwSetWindowShouldClose(win, 1); return; }
        if (rtMode) {
            if (key == GLFW_KEY_T) { rtMode = false; }
            else if (key == GLFW_KEY_S) { say(rt.saveBMP("raytrace.bmp") ? "Saved raytrace.bmp" : "Could not save image"); }
            else if (key == GLFW_KEY_SPACE) rt.reset();
            return;
        }
        switch (key) {
            case GLFW_KEY_A: setMode(CTRL_AUTO); break;
            case GLFW_KEY_M: setMode(CTRL_MANUAL); break;
            case GLFW_KEY_P: pit.request(player(), tr, 0); break;
            case GLFW_KEY_Y: pit.request(player(), tr, 1); break;
            case GLFW_KEY_U: pit.request(player(), tr, 2); break;
            case GLFW_KEY_R: pit.keyRaise(); break;
            case GLFW_KEY_C: pit.keyChange(); break;
            case GLFW_KEY_D: pit.keyLower(); break;
            case GLFW_KEY_Q: pit.keyExit(); break;
            case GLFW_KEY_1: camMode = 1; break;
            case GLFW_KEY_2: camMode = 2; break;
            case GLFW_KEY_3: camMode = 3; break;
            case GLFW_KEY_4: camMode = 4; break;
            case GLFW_KEY_5: if (camMode == 5) cam.pitView = (cam.pitView + 1) % 3; camMode = 5; break;
            case GLFW_KEY_6: camMode = 6; break;
            case GLFW_KEY_7: camMode = 7; break;
            case GLFW_KEY_8: camMode = 8; break;
            case GLFW_KEY_9: camMode = 9; break;
            case GLFW_KEY_V: audio.toggleMute(); say(audio.isMuted() ? "AUDIO: MUTED" : "AUDIO: ENABLED"); break;
            case GLFW_KEY_G: R.mode = SH_GOURAUD; say("GOURAUD SHADING (per-vertex lighting) - all objects"); break;
            case GLFW_KEY_H: R.mode = SH_PHONG; say("PHONG SHADING (per-fragment lighting) - all objects"); break;
            case GLFW_KEY_J: R.mode = SH_HYBRID; say("HYBRID: Gouraud environment + Phong cars / tyres / pit equipment"); break;
            case GLFW_KEY_L: preset = (preset + 1) % 3; say(std::string("LIGHTING: ") + PRESETS[preset].name); break;
            case GLFW_KEY_W: R.wire = !R.wire; say(R.wire ? "WIREFRAME ON" : "WIREFRAME OFF"); break;
            case GLFW_KEY_SPACE: paused = !paused; break;
            case GLFW_KEY_N: resetRace(); say("RACE RESTARTED"); break;
            case GLFW_KEY_T: rtMode = true; rt.reset(); break;
            case GLFW_KEY_F1: showHelp = !showHelp; helpTimer = 0; break;
            case GLFW_KEY_TAB: showHud = !showHud; break;
            case GLFW_KEY_F: toggleFullscreen(); break;
            case GLFW_KEY_PAGE_UP: case GLFW_KEY_EQUAL: case GLFW_KEY_KP_ADD: cam.overH = clampf(cam.overH * 1.2f, 60.f, 1500.f); cam.chaseDist = clampf(cam.chaseDist + 1.f, 6.f, 30.f); break;
            case GLFW_KEY_PAGE_DOWN: case GLFW_KEY_MINUS: case GLFW_KEY_KP_SUBTRACT: cam.overH = clampf(cam.overH / 1.2f, 60.f, 1500.f); cam.chaseDist = clampf(cam.chaseDist - 1.f, 6.f, 30.f); break;
            default: break;
        }
    }
    void toggleFullscreen() {
        static int wx, wy, ww, wh;
        fullscreen = !fullscreen;
        if (fullscreen) {
            glfwGetWindowPos(win, &wx, &wy); glfwGetWindowSize(win, &ww, &wh);
            GLFWmonitor* m = glfwGetPrimaryMonitor(); const GLFWvidmode* vm = glfwGetVideoMode(m);
            glfwSetWindowMonitor(win, m, 0, 0, vm->width, vm->height, vm->refreshRate);
        } else glfwSetWindowMonitor(win, nullptr, wx, wy, ww, wh, 0);
    }

    // ------------------------------------------------------------------------
    //  simulation
    // ------------------------------------------------------------------------
    void trackLimits(Car& c, float dt) {
        float u = tr.sRel(c.s);
        bool inWall = (u > PitC::WALL_X0 && u < PitC::WALL_X1);
        if (!inWall) c.inLane = c.d > PitC::WALL_D + 0.4f;
        float hdg = tr.heading(c.s);
        float sinD = std::sin(wrapAngle(c.heading - hdg));
        float ext = 0.95f + 2.6f * std::min(1.f, std::fabs(sinD));
        float right = tr.rightLimit(u, c.inLane), left = tr.leftLimit(u);
        if (inWall && c.inLane) left = PitC::LANE_IN + 0.6f;
        vec3 n = tr.n[c.idx];
        bool hit = false; float over = 0;
        if (c.d + ext > right) { over = c.d + ext - right; c.pos -= n * over; hit = true; }
        else if (c.d - ext < left) { over = left - (c.d - ext); c.pos += n * over; hit = true; }
        if (hit) {
            float vn = dot(c.forward() * c.speed, n);
            if (c.hitCooldown <= 0 && std::fabs(vn) > 0.5f) {
                c.speed *= 0.62f; c.hitCooldown = 0.35f;
                if (c.player) shakeT = clampf(std::fabs(vn) / 25.f, 0.2f, 1.f);
            }
            c.heading = wrapAngle(c.heading + wrapAngle(hdg - c.heading) * 0.12f);
            c.steer *= 0.5f;
        }
        c.hitCooldown -= dt;
    }

    void carCollisions() {
        for (size_t a = 0; a < cars.size(); a++)
            for (size_t b = a + 1; b < cars.size(); b++) {
                Car &A = cars[a], &B = cars[b];
                if (A.ctrl == CTRL_PIT || B.ctrl == CTRL_PIT || A.ctrl == CTRL_GRID || B.ctrl == CTRL_GRID) continue;
                vec3 dd = B.pos - A.pos; dd.y = 0;
                if (length(dd) > 6.5f) continue;
                float lon = dot(A.forward(), dd), lat = dot(A.rightV(), dd);
                bool touched = false;
                for (int i = -1; i <= 1; i += 2)
                    for (int j = -1; j <= 1; j += 2) {
                        vec3 pa = A.pos + A.forward() * (1.5f * i), pb = B.pos + B.forward() * (1.5f * j);
                        vec3 dlt = pb - pa; dlt.y = 0; float d = length(dlt);
                        if (d < 2.1f && d > 1e-4f) {
                            vec3 nn = dlt / d; float push = (2.1f - d) * 0.35f;
                            A.pos -= nn * push; B.pos += nn * push; touched = true;
                        }
                    }
                if (touched && std::fabs(lat) < 1.8f && std::fabs(lon) > 1.0f) {    // nose-to-tail contact: the rear car loses speed
                    Car& rear = lon > 0 ? A : B; Car& front = lon > 0 ? B : A;
                    rear.speed = std::min(rear.speed, front.speed * 0.99f);
                }
                if (touched && (A.player || B.player)) shakeT = std::max(shakeT, 0.3f);
            }
    }

    void stepRace(float dt) {
        simTime += dt;
        if (race == RS_COUNT) {
            raceT += dt;
            float t0 = 1.6f;
            startLights = raceT < t0 ? 0 : std::min(5, (int)((raceT - t0) / 0.85f) + 1);
            if (startLights >= 5 && raceT > t0 + 4 * 0.85f + goDelay) {
                lightsOut = true; race = RS_RACE;
                for (Car& c : cars) { c.ctrl = c.player ? driveMode : CTRL_AUTO; c.lapStart = simTime; }
                say("LIGHTS OUT AND AWAY WE GO!", 2.5f);
            }
        }
        // projection / lap logic
        for (Car& c : cars) {
            Proj p = tr.project(c.pos, c.idx, 45);
            c.idx = p.i; c.prevS = c.s; c.s = p.s; c.d = p.d;
            if (c.prevS > tr.L * 0.9f && c.s < tr.L * 0.1f) {
                c.lapsDone++;
                if (c.lapsDone >= 2) { c.lastLap = simTime - c.lapStart; if (c.bestLap <= 0 || c.lastLap < c.bestLap) c.bestLap = c.lastLap; }
                c.lapStart = simTime;
            } else if (c.prevS < tr.L * 0.1f && c.s > tr.L * 0.9f) c.lapsDone--;
            c.distTotal = c.lapsDone * tr.L + ((c.lapsDone == 0 && c.s > tr.L * 0.5f) ? c.s - tr.L : c.s);
        }
        pit.update(player(), tr, dt, simTime);
        for (Car& c : cars) {
            float thr = 0, brk = 0, steerT = 0; float grip = (std::fabs(c.d) > 7.6f && c.ctrl != CTRL_PIT) ? 0.72f : 1.f;
            switch (c.ctrl) {
                case CTRL_GRID: c.speed = 0; c.thr = 0; c.brk = 1; c.latAcc = c.longAcc = 0; continue;
                case CTRL_AUTO: driverControl(tr, c, cars, dt, simTime, thr, brk, steerT); break;
                case CTRL_MANUAL: {
                    thr = isDown(GLFW_KEY_UP) ? 1.f : 0.f; brk = isDown(GLFW_KEY_DOWN) ? 1.f : 0.f;
                    float target = (isDown(GLFW_KEY_RIGHT) ? 1.f : 0.f) - (isDown(GLFW_KEY_LEFT) ? 1.f : 0.f);
                    if (target != 0) { float rate = (target * steerIn < 0 ? 4.5f : 2.2f) * dt; steerIn += clampf(target - steerIn, -rate, rate); }
                    else steerIn += clampf(-steerIn, -3.5f * dt, 3.5f * dt);
                    steerT = steerIn * Car::steerLimit(std::fabs(c.speed));
                    break;
                }
                case CTRL_PIT:
                    if (pit.stage >= PS_AT_BOX && pit.stage <= PS_LOWERED) {
                        c.speed = 0; c.thr = 0; c.brk = 0; c.latAcc = c.longAcc = 0; c.steer = 0; continue;
                    }
                    pit.guidedControl(c, tr, dt, thr, brk, steerT);
                    break;
            }
            c.dynamics(dt, thr, brk, steerT, grip, c.ctrl == CTRL_MANUAL);
            if (c.ctrl != CTRL_PIT) trackLimits(c, dt);
        }
        carCollisions();
        if (testMode) {
            for (Car& c : cars) {
                Diag& d = diag[c.id];
                if (c.ctrl != CTRL_GRID && c.ctrl != CTRL_PIT) {
                    float step = length(c.pos - d.prevPos), exp = std::fabs(c.speed) * dt;
                    if (std::fabs(step - exp) > 0.35f && simTime > 0.1f) d.posJump++;
                    if (std::fabs(wrapAngle(c.heading - d.prevH)) > 0.05f) d.hdgJump++;
                    if (std::fabs(c.speed - d.prevV) > 2.5f) d.spdJump++;
                    d.maxD = std::max(d.maxD, std::fabs(c.d));
                }
                d.prevPos = c.pos; d.prevH = c.heading; d.prevV = c.speed;
            }
            orderT += dt;
            if (orderT > 0.5f && race == RS_RACE) {
                orderT = 0; std::vector<int> o = {0, 1, 2, 3, 4};
                std::sort(o.begin(), o.end(), [&](int a, int b) { return cars[a].distTotal > cars[b].distTotal; });
                if (!lastOrder.empty() && o != lastOrder) orderChanges++;
                lastOrder = o;
            }
        }
    }

    // ------------------------------------------------------------------------
    //  frame update
    // ------------------------------------------------------------------------
    CamView view;
    void update(float dt) {
        realTime += dt;
        if (toastT > 0) toastT -= dt;
        if (helpTimer > 0) { helpTimer -= dt; if (helpTimer <= 0) showHelp = false; }
        shakeT = std::max(0.f, shakeT - dt * 1.5f);
        if (!paused && !rtMode) {
            static float acc = 0; acc += std::min(dt, 0.05f);
            const float H = 1.f / 120.f; int n = 0;
            while (acc >= H && n < 12) { stepRace(H); acc -= H; n++; }
        }
        for (Car& c : cars) c.updateNodes();
        pit.idleWheels(player());
        player().helmetN->visible = (camMode != 1);
        view = cam.compute(camMode, player(), tr, world, pit, dt, realTime);
        if (viewOverride) { view.eye = ovEye; view.at = ovAt; view.fov = ovFov; view.up = vec3(0, 1, 0); view.znear = 0.1f; view.name = "FREE"; }
        if (shakeT > 0) {
            float a = shakeT * 0.25f;
            view.eye += vec3(std::sin(realTime * 91.f), std::sin(realTime * 77.f + 1.f), std::sin(realTime * 63.f + 2.f)) * a;
        }
        if (rtMode) {
            if (isDown(GLFW_KEY_LEFT)) { rt.azim -= dt * 1.2f; rt.reset(); }
            if (isDown(GLFW_KEY_RIGHT)) { rt.azim += dt * 1.2f; rt.reset(); }
            if (isDown(GLFW_KEY_UP)) { rt.elev = clampf(rt.elev + dt * 0.6f, 0.05f, 1.2f); rt.reset(); }
            if (isDown(GLFW_KEY_DOWN)) { rt.elev = clampf(rt.elev - dt * 0.6f, 0.05f, 1.2f); rt.reset(); }
            rt.step(0.012);
        }
        audio.update(player().speed, player().thr, player().brk, player().latAcc, player().ctrl, pit.stage, shakeT, paused);
    }

    // ------------------------------------------------------------------------
    //  rendering
    // ------------------------------------------------------------------------
    void gatherLights() {
        const Preset& P = PRESETS[preset];
        R.lights.clear();
        pit.lights(R.lights, 1.0f);
        for (const LightSrc& g : world.garageSpots) { LightSrc l = g; l.col = g.col * mixf(0.45f, 1.f, P.lamp); R.lights.push_back(l); }
        if (P.lamp > 0.05f) {
            std::vector<std::pair<float, int>> d;
            for (int i = 0; i < (int)world.lamps.size(); i++) d.push_back({length(world.lamps[i].pos - view.eye), i});
            std::sort(d.begin(), d.end());
            for (int i = 0; i < (int)d.size() && R.lights.size() < 16; i++) {
                LightSrc l = world.lamps[d[i].second]; l.col = l.col * P.lamp; R.lights.push_back(l);
            }
        }
        R.uploadLights();
    }

    void drawScene() {
        const Preset& P = PRESETS[preset];
        float aspect = (float)fbw / std::max(1, fbh);
        mat4 proj = perspective(radians(view.fov), aspect, view.znear, 4500.f);
        mat4 vm = lookAt(view.eye, view.at, view.up);
        R.beginFrame(vm, proj, view.eye, P, realTime);
        glViewport(0, 0, fbw, fbh);
        glClearColor(P.horizon.x, P.horizon.y, P.horizon.z, 1);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glPolygonMode(GL_FRONT_AND_BACK, R.wire ? GL_LINE : GL_FILL);
        if (!R.wire) R.drawSky(vm, proj);
        glEnable(GL_DEPTH_TEST); glDepthFunc(GL_LEQUAL);
        gatherLights();
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(0.f, 0.f);  R.drawBatch(world.ground, false, 4000.f);
        glPolygonOffset(-1.f, -2.f); R.drawBatch(world.road, false, 3000.f);
        glPolygonOffset(-1.f, -4.f); R.drawBatch(world.pitL, false, 3000.f);
        glPolygonOffset(-1.f, -7.f); R.drawBatch(world.marks, false, 2500.f);
        glDisable(GL_POLYGON_OFFSET_FILL);
        { mat4 I; R.draw(world.hills, I, vec3(1), 0.f, false); }
        R.drawBatch(world.env, false, 1500.f);
        R.drawBatch(world.hero, true, 900.f);
        // dynamic objects: cars, pit crew, equipment, lamps
        for (Car& c : cars) c.root->draw(R);
        pit.draw(R);
        world.drawLamps(R, prims, pit, startLights, lightsOut);
        // marker discs above cars for the overhead view
        if (view.name == "OVERHEAD") {
            for (Car& c : cars) {
                float rr = std::max(3.f, cam.overH * 0.012f);
                R.draw(prims.cyl, translate(c.pos.x, 2.2f, c.pos.z) * scale(rr, 0.3f, rr), c.liv->main, 0.6f, true);
            }
        }
        // blob shadows + transparent parts
        glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE); glEnable(GL_POLYGON_OFFSET_FILL); glPolygonOffset(-1.f, -12.f);
        if (!R.wire) {
            float sh = 0.55f * clampf(P.sunCol.x * 0.5f, 0.2f, 1.f);
            for (Car& c : cars) R.drawBlob(vec3(c.pos.x, 0.02f, c.pos.z) - normalize(vec3(P.sunDir.x, 0, P.sunDir.z)) * 0.6f, c.heading, 3.3f, 1.35f, sh);
            for (int i = 0; i < 8; i++) R.drawBlob(vec3(pit.crew[i].pos.x, 0.02f, pit.crew[i].pos.z), 0, 0.55f, 0.55f, sh * 0.8f);
        }
        glDisable(GL_POLYGON_OFFSET_FILL);
        R.drawBatch(world.glass, false, 500.f, 0.38f);
        glDepthMask(GL_TRUE); glDisable(GL_BLEND);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

    static std::string fmtTime(float t) {
        if (t <= 0) return "--:--.---";
        char b[32]; snprintf(b, sizeof b, "%d:%06.3f", (int)(t / 60), std::fmod(t, 60.f)); return b;
    }

    void drawHud() {
        hud.begin(fbw, fbh);
        float ui = std::max(0.8f, fbh / 720.f);
        const vec3 white(1), yellow(1, 0.85f, 0.2f), green(0.3f, 1, 0.4f), red(1, 0.3f, 0.25f), cyan(0.4f, 0.9f, 1.f), grey(0.75f);
        Car& p = player();
        if (rtMode) {
            float W = (float)fbw, H = (float)fbh;
            hud.rect(0, 0, W, 44 * ui, 0, 0, 0, 0.6f);
            hud.textShadow(14 * ui, 8 * ui, 1.5f * ui, white, "RAY-TRACING DEMONSTRATION  (CPU, shadows + reflections)");
            char b[200]; snprintf(b, sizeof b, "pass %d/%d  |  arrows: orbit camera  |  SPACE: re-render  |  S: save raytrace.bmp  |  T: back to simulation", std::min(rt.pass + 1, rt.maxPass), rt.maxPass);
            hud.textShadow(14 * ui, H - 30 * ui, 1.1f * ui, yellow, b);
            if (toastT > 0) hud.textC(fbw * 0.5f, 60 * ui, 1.3f * ui, cyan, toast);
            hud.flush(); return;
        }
        if (!showHud) { hud.flush(); return; }
        // ---- speed / gear / pedals (bottom centre) -----------------------------
        float cx = fbw * 0.5f, by = fbh - 118 * ui;
        hud.rect(cx - 190 * ui, by - 8 * ui, 380 * ui, 118 * ui, 0, 0, 0, 0.45f);
        char buf[160];
        snprintf(buf, sizeof buf, "%3d", (int)std::fabs(p.speed * 3.6f));
        hud.textShadow(cx - 160 * ui, by + 4 * ui, 4.2f * ui, white, buf);
        hud.textShadow(cx - 160 * ui + 3 * 42 * ui + 4 * ui, by + 56 * ui, 1.2f * ui, grey, "KM/H");
        int gear = p.speed < -0.5f ? -1 : std::min(8, 1 + (int)(std::fabs(p.speed) / 11.5f));
        float span = 11.5f, rpm = clampf((std::fmod(std::fabs(p.speed), span) / span) * 0.5f + 0.5f + p.thr * 0.05f, 0.f, 1.f);
        if (std::fabs(p.speed) < 0.5f) rpm = 0.25f + p.thr * 0.5f;
        if (gear < 0) snprintf(buf, sizeof buf, "R"); else snprintf(buf, sizeof buf, "%d", gear);
        hud.textShadow(cx + 70 * ui, by + 4 * ui, 4.2f * ui, yellow, buf);
        hud.textShadow(cx + 70 * ui, by + 56 * ui, 1.2f * ui, grey, "GEAR");
        for (int i = 0; i < 20; i++) {
            float f = i / 20.f; vec3 c = f < 0.6f ? green : (f < 0.85f ? yellow : red);
            hud.rectc(cx - 170 * ui + i * 17.5f * ui, by + 80 * ui, 14 * ui, 10 * ui, c, rpm > f ? 1.f : 0.18f);
        }
        hud.rectc(cx + 125 * ui, by + 6 * ui, 10 * ui, 60 * ui, vec3(0.2f), 0.6f);
        hud.rectc(cx + 125 * ui, by + 66 * ui - 60 * ui * p.thr, 10 * ui, 60 * ui * p.thr, green);
        hud.rectc(cx + 145 * ui, by + 6 * ui, 10 * ui, 60 * ui, vec3(0.2f), 0.6f);
        hud.rectc(cx + 145 * ui, by + 66 * ui - 60 * ui * p.brk, 10 * ui, 60 * ui * p.brk, red);
        // tyre wear bar
        snprintf(buf, sizeof buf, "TYRES %-6s", COMPOUNDS[p.compound].name);
        hud.textShadow(cx - 170 * ui, by + 94 * ui, 1.0f * ui, COMPOUNDS[p.compound].col, buf);
        float life = 1.f - p.wear;
        hud.rectc(cx - 28 * ui, by + 97 * ui, 128 * ui, 9 * ui, vec3(0.2f), 0.7f);
        hud.rectc(cx - 28 * ui, by + 97 * ui, 128 * ui * life, 9 * ui, life > 0.5f ? green : (life > 0.25f ? yellow : red));
        snprintf(buf, sizeof buf, "%3d%%", (int)(life * 100)); hud.textShadow(cx + 104 * ui, by + 94 * ui, 1.0f * ui, white, buf);
        // ---- race info (top left) ---------------------------------------------------
        float lx = 14 * ui, ly = 12 * ui;
        hud.rect(lx - 6 * ui, ly - 4 * ui, 300 * ui, 196 * ui, 0, 0, 0, 0.45f);
        int lap = std::max(1, p.lapsDone);
        snprintf(buf, sizeof buf, "LAP %d", lap); hud.textShadow(lx, ly, 2.2f * ui, white, buf);
        snprintf(buf, sizeof buf, "LAST %s", fmtTime(p.lastLap).c_str()); hud.textShadow(lx, ly + 34 * ui, 1.0f * ui, grey, buf);
        snprintf(buf, sizeof buf, "BEST %s", fmtTime(p.bestLap).c_str()); hud.textShadow(lx + 150 * ui, ly + 34 * ui, 1.0f * ui, grey, buf);
        std::vector<int> order = {0, 1, 2, 3, 4};
        std::sort(order.begin(), order.end(), [&](int a, int b) { return cars[a].distTotal > cars[b].distTotal; });
        for (int r = 0; r < 5; r++) {
            Car& c = cars[order[r]];
            float y = ly + 58 * ui + r * 26 * ui;
            hud.rectc(lx, y, 10 * ui, 18 * ui, c.liv->main);
            hud.rectc(lx + 10 * ui, y, 4 * ui, 18 * ui, c.liv->second);
            std::string nm = c.liv->name; nm.resize(14, ' ');
            snprintf(buf, sizeof buf, "%d %s", r + 1, nm.c_str());
            hud.textShadow(lx + 22 * ui, y, 1.15f * ui, c.player ? yellow : white, buf);
            if (r > 0) { snprintf(buf, sizeof buf, "+%3.0fm", cars[order[0]].distTotal - c.distTotal); hud.textShadow(lx + 236 * ui, y, 1.0f * ui, grey, buf); }
        }
        // ---- minimap (top right) --------------------------------------------------------
        {
            float mw = 250 * ui, mh = 150 * ui, mx = fbw - mw - 14 * ui, my = 12 * ui;
            hud.rect(mx - 4 * ui, my - 4 * ui, mw + 8 * ui, mh + 8 * ui, 0, 0, 0, 0.5f);
            float sx = (mw - 16 * ui) / (tr.bmax.x - tr.bmin.x), sz = (mh - 16 * ui) / (tr.bmax.z - tr.bmin.z), sc = std::min(sx, sz);
            auto M = [&](const vec3& q, float& x, float& y) { x = mx + 8 * ui + (q.x - tr.bmin.x) * sc; y = my + 8 * ui + (q.z - tr.bmin.z) * sc; };
            for (int i = 0; i < tr.N; i += 7) {
                float x0, y0, x1, y1; M(tr.p[i], x0, y0); M(tr.p[(i + 7) % tr.N], x1, y1);
                hud.line(x0, y0, x1, y1, 3.f * ui, 0.85f, 0.85f, 0.88f, 0.95f);
            }
            for (size_t i = 0; i + 8 < pit.path.size(); i += 8) {
                float x0, y0, x1, y1; M(pit.path[i], x0, y0); M(pit.path[i + 8], x1, y1);
                float u = pit.pathU[i]; if (pitPathOffset(u) > 3.f) hud.line(x0, y0, x1, y1, 2.f * ui, 0.3f, 0.6f, 1.f, 0.95f);
            }
            { float x0, y0, x1, y1; M(tr.pos(0, -8), x0, y0); M(tr.pos(0, 8), x1, y1); hud.line(x0, y0, x1, y1, 2.f * ui, 1, 0.2f, 0.2f, 1); }
            for (Car& c : cars) {
                float x, y; M(c.pos, x, y); float r = (c.player ? 5.5f : 4.f) * ui;
                if (c.player) hud.rect(x - r - 1.5f * ui, y - r - 1.5f * ui, 2 * r + 3 * ui, 2 * r + 3 * ui, 1, 1, 1, 1);
                hud.rectc(x - r, y - r, 2 * r, 2 * r, c.liv->main);
            }
        }
        // ---- right-bottom: status ---------------------------------------------------------
        {
            float rx = fbw - 14 * ui, y = fbh - 100 * ui;
            auto R_ = [&](const std::string& s, const vec3& c) { hud.textShadow(rx - hud.textW(s, 1.05f * ui), y, 1.05f * ui, c, s); y += 20 * ui; };
            R_(std::string("MODE: ") + (p.ctrl == CTRL_PIT ? "PIT (guided)" : (driveMode == CTRL_AUTO ? "AUTOMATIC" : "MANUAL")), driveMode == CTRL_AUTO ? cyan : green);
            R_(std::string("CAM: ") + view.name, white);
            R_(std::string("AUDIO: ") + (audio.isMuted() ? "MUTED (V)" : "ON (V)"), audio.isMuted() ? grey : green);
            const char* sm = R.mode == SH_GOURAUD ? "GOURAUD" : (R.mode == SH_PHONG ? "PHONG" : "HYBRID (G env + P cars)");
            R_(std::string("SHADING: ") + sm, yellow);
            R_(std::string("LIGHT: ") + PRESETS[preset].name + (R.wire ? "  WIREFRAME" : ""), grey);
            snprintf(buf, sizeof buf, "%.0f FPS", fps); R_(buf, grey);
        }
        // ---- pit panel ----------------------------------------------------------------------
        bool pitActive = pit.stage != PS_NONE || (pit.totalT - pit.exitAt) < 3.5f;
        if (pitActive || pit.armed) {
            float px = 14 * ui, py = 218 * ui;
            hud.rect(px - 6 * ui, py - 4 * ui, 300 * ui, 270 * ui, 0, 0, 0, 0.5f);
            hud.textShadow(px, py, 1.3f * ui, yellow, std::string(pit.armed && pit.stage == PS_NONE ? "PIT REQUESTED" : "PIT STOP") + (pit.autoMode ? " [AUTO]" : " [MANUAL]"));
            static const char* steps[8] = {"PIT ENTRY", "PIT LANE MOVEMENT", "PIT BOX ARRIVAL", "CAR RAISING", "TYRE REPLACEMENT", "CAR LOWERING", "PIT EXIT", "RACE RESUMPTION"};
            int cur = 0;
            switch (pit.stage) {
                case PS_ENTRY: cur = 1; break; case PS_LANE: cur = 2; break; case PS_AT_BOX: cur = 3; break;
                case PS_RAISING: case PS_RAISED: cur = 4; break; case PS_CHANGING: case PS_CHANGED: cur = 5; break;
                case PS_LOWERING: case PS_LOWERED: cur = 6; break; case PS_EXITING: cur = 7; break;
                default: cur = (pit.totalT - pit.exitAt) < 3.5f ? 9 : 0;
            }
            bool waiting = pit.stage == PS_AT_BOX || pit.stage == PS_RAISED || pit.stage == PS_CHANGED || pit.stage == PS_LOWERED;
            for (int i = 0; i < 8; i++) {
                float y = py + 26 * ui + i * 22 * ui;
                bool done = (i + 1) < cur, now = (i + 1) == cur;
                vec3 c = done ? green : (now ? yellow : vec3(0.5f));
                snprintf(buf, sizeof buf, "%s %d %s", done ? "[x]" : (now ? "[>]" : "[ ]"), i + 1, steps[i]);
                hud.textShadow(px, y, 1.0f * ui, c, buf, (now && waiting && ((int)(realTime * 3) % 2)) ? 0.6f : 1.f);
            }
            if (pit.stage >= PS_AT_BOX && pit.stage <= PS_LOWERED) {
                snprintf(buf, sizeof buf, "STOP TIME %.1fs", pit.stopTime); hud.textShadow(px, py + 26 * ui + 8 * 22 * ui + 4 * ui, 1.15f * ui, white, buf);
            } else if (pit.lastStop > 0 && pit.stage == PS_NONE) {
                snprintf(buf, sizeof buf, "LAST STOP %.1fs", pit.lastStop); hud.textShadow(px, py + 26 * ui + 8 * 22 * ui + 4 * ui, 1.15f * ui, white, buf);
            }
        }
        // ---- big prompts -----------------------------------------------------------------------
        const char* prompt = nullptr;
        if (!pit.autoMode) switch (pit.stage) {
            case PS_AT_BOX: prompt = "PRESS  R  -  RAISE THE CAR"; break;
            case PS_RAISED: prompt = "PRESS  C  -  CHANGE TYRES"; break;
            case PS_CHANGED: prompt = "PRESS  D  -  LOWER THE CAR"; break;
            case PS_LOWERED: prompt = "GREEN LIGHT  -  PRESS  Q  TO EXIT THE PIT"; break;
            case PS_CHANGING: prompt = "CHANGING TYRES..."; break;
            case PS_RAISING: prompt = "RAISING CAR..."; break;
            case PS_LOWERING: prompt = "LOWERING CAR..."; break;
            default: break;
        }
        if (prompt) hud.textC(fbw * 0.5f, fbh * 0.18f, 2.0f * ui, (pit.stage == PS_LOWERED) ? green : yellow, prompt, (int)(realTime * 2.5f) % 2 ? 0.75f : 1.f);
        else if (pit.armed && pit.stage == PS_NONE) hud.textC(fbw * 0.5f, fbh * 0.18f, 1.6f * ui, yellow, pit.autoMode ? "BOX BOX BOX  -  AUTOMATIC PIT STOP" : "BOX BOX BOX  -  MANUAL PIT STOP", 1.f);
        else if (pit.autoMode && pit.stage >= PS_AT_BOX && pit.stage <= PS_LOWERED) hud.textC(fbw * 0.5f, fbh * 0.18f, 2.0f * ui, yellow, "AUTOMATIC PIT STOP...", 1.f);
        if (race == RS_COUNT) {
            hud.textC(fbw * 0.5f, fbh * 0.30f, 2.4f * ui, startLights >= 5 ? red : white, startLights >= 5 ? "WAIT FOR LIGHTS OUT" : "FORMATION - GET READY");
            for (int i = 0; i < 5; i++) {
                float x = fbw * 0.5f + (i - 2) * 46 * ui;
                hud.rectc(x - 17 * ui, fbh * 0.38f, 34 * ui, 34 * ui, i < startLights ? vec3(1, 0.05f, 0.05f) : vec3(0.15f, 0.02f, 0.02f), 0.95f);
            }
        }
        // ---- messages --------------------------------------------------------------------------------
        if (pit.messageT > 0 && !pit.message.empty()) hud.textC(fbw * 0.5f, fbh * 0.25f, 1.3f * ui, white, pit.message, clampf(pit.messageT, 0.f, 1.f));
        if (toastT > 0) hud.textC(fbw * 0.5f, fbh * 0.07f, 1.3f * ui, cyan, toast, clampf(toastT * 2.f, 0.f, 1.f));
        if (paused) { hud.rect(0, 0, (float)fbw, (float)fbh, 0, 0, 0, 0.35f); hud.textC(fbw * 0.5f, fbh * 0.42f, 3.f * ui, white, "PAUSED"); hud.textC(fbw * 0.5f, fbh * 0.42f + 60 * ui, 1.2f * ui, grey, "SPACE to resume"); }
        // ---- help ----------------------------------------------------------------------------------------
        if (showHelp) {
            static const char* L[] = {
                "CONTROLS", "A  automatic driving         M  manual driving", "UP / DOWN  accelerate / brake-reverse", "LEFT / RIGHT  steer",
                "Y  AUTO pit stop (whole stop ~3 s)   U  MANUAL pit stop", "P  request pit (last chosen mode)", "manual: R raise   C tyres   D lower   Q exit pit",
                "1 Cockpit  2 Chase  3 Trackside  4 Overhead", "5 Pit (repeat = angles)  6 Broadcast", "7 Low Chase  8 Helicopter  9 Head On", "G Gouraud  H Phong  J Hybrid  W Wireframe",
                "L day/dusk/night   V audio mute", "PgUp/PgDn zoom   SPACE pause", "N restart   T ray tracing", "F fullscreen  TAB hide HUD  F1 help", "ESC exit"};
            float hx = fbw - 500 * ui, hy = 190 * ui; float hh = (float)(sizeof(L) / sizeof(L[0])) * 21 * ui + 14 * ui;
            hud.rect(hx - 8 * ui, hy - 6 * ui, 494 * ui, hh, 0, 0, 0, 0.55f);
            for (size_t i = 0; i < sizeof(L) / sizeof(L[0]); i++) hud.textShadow(hx, hy + i * 21 * ui, 0.98f * ui, i == 0 ? yellow : white, L[i]);
        }
        hud.flush();
    }

    void render() {
        if (rtMode) {
            glViewport(0, 0, fbw, fbh); glClearColor(0.05f, 0.05f, 0.07f, 1); glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glDisable(GL_DEPTH_TEST);
            float asp = (float)RayTracer::W / RayTracer::H, sa = (float)fbw / fbh;
            float w, h; if (sa > asp) { w = asp / sa * 0.9f; h = 0.9f; } else { w = 0.95f; h = sa / asp * 0.95f; }
            R.blit.use(); R.blit.f4("uRect", -w, -h, w, h);
            glActiveTexture(GL_TEXTURE0); glBindTexture(GL_TEXTURE_2D, rt.tex); R.blit.i1("uTex", 0);
            glBindVertexArray(R.emptyVao); glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
            glEnable(GL_DEPTH_TEST);
        } else drawScene();
        drawHud();
    }

    void savePPM(const char* path) {
        std::vector<unsigned char> px(fbw * fbh * 3);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadBuffer(GL_BACK);
        glReadPixels(0, 0, fbw, fbh, GL_RGB, GL_UNSIGNED_BYTE, px.data());
        FILE* f = fopen(path, "wb"); if (!f) return;
        fprintf(f, "P6\n%d %d\n255\n", fbw, fbh);
        for (int y = fbh - 1; y >= 0; y--) fwrite(&px[y * fbw * 3], 1, fbw * 3, f);
        fclose(f);
    }

    // ------------------------------------------------------------------------
    //  scripted test harness (used to verify the build headlessly)
    // ------------------------------------------------------------------------
    static int keyFromName(const std::string& n) {
        if (n.size() == 1) { char c = n[0]; if (c >= 'a' && c <= 'z') c -= 32; return c; }
        if (n == "UP") return GLFW_KEY_UP;
        if (n == "DOWN") return GLFW_KEY_DOWN;
        if (n == "LEFT") return GLFW_KEY_LEFT;
        if (n == "RIGHT") return GLFW_KEY_RIGHT;
        if (n == "SPACE") return GLFW_KEY_SPACE;
        if (n == "F1") return GLFW_KEY_F1;
        if (n == "TAB") return GLFW_KEY_TAB;
        if (n == "PGUP") return GLFW_KEY_PAGE_UP;
        if (n == "PGDN") return GLFW_KEY_PAGE_DOWN;
        return 0;
    }
    void loadScript(const char* path) {
        std::ifstream f(path); std::string line;
        while (std::getline(f, line)) {
            std::istringstream ss(line); Cmd c; c.done = false;
            if (!(ss >> c.t >> c.c)) continue;
            ss >> c.a; std::getline(ss, c.b); script.push_back(c);
        }
        testMode = true;
    }
    void runScript(double dt, bool& wantShot, std::string& shotPath, bool& quit) {
        testTime += dt; wantShot = false;
        for (Cmd& c : script) {
            if (c.done || c.t > testTime) continue;
            c.done = true;
            if (c.c == "key") onKey(keyFromName(c.a));
            else if (c.c == "hold") { int k = keyFromName(c.a); keyOverride[k] = true; holdUntil[k] = (float)(testTime + atof(c.b.c_str())); }
            else if (c.c == "shot") { wantShot = true; shotPath = c.a; }
            else if (c.c == "quit") quit = true;
            else if (c.c == "tele") {   // tele <sRel> <speed>: place the player on the centre line
                Car& p = player(); float s = tr.wrapS((float)atof(c.a.c_str()));
                p.pos = tr.pos(s, 0); p.heading = tr.heading(s); p.speed = (float)atof(c.b.c_str()); p.idx = -1;
                Proj pr = tr.project(p.pos, -1); p.idx = pr.i; p.s = p.prevS = pr.s; p.d = pr.d; p.steer = 0;
            }
            else if (c.c == "view") {   // view ex ey ez tx ty tz fov   (arguments packed in c.a / c.b by the loader)
                float v[7] = {0}; std::istringstream is(c.a + " " + c.b); for (int i = 0; i < 7; i++) is >> v[i];
                viewOverride = true; ovEye = vec3(v[0], v[1], v[2]); ovAt = vec3(v[3], v[4], v[5]); ovFov = v[6];
            }
            else if (c.c == "noview") viewOverride = false;
            else if (c.c == "diag") {
                printf("[diag t=%.0f] running-order changes: %d | order:", testTime, orderChanges);
                for (int id : lastOrder) { printf(" %s", cars[id].liv->name); }
                printf("\n");
                for (Car& q : cars) { Diag& d = diag[q.id]; printf("   %-14s laps=%d best=%.1fs  pos-jumps=%d heading-jumps=%d speed-jumps=%d  max|d|=%.1f\n", q.liv->name, q.lapsDone, q.bestLap, d.posJump, d.hdgJump, d.spdJump, d.maxD); }
            }
            else if (c.c == "skip") { race = RS_RACE; lightsOut = true; startLights = 5; for (Car& q : cars) q.ctrl = q.player ? driveMode : CTRL_AUTO; }
            else if (c.c == "print") {
                Car& p = player();
                printf("[t=%.1f] player s=%.1f d=%.2f v=%.1f ctrl=%d stage=%s lap=%d lastLap=%.1f stopTime=%.2f wear=%.2f pos=(%.1f,%.1f) lift=%.2f\n", testTime, p.s, p.d, p.speed, (int)p.ctrl, PIT_STAGE_NAME[pit.stage], p.lapsDone, p.lastLap, pit.lastStop, p.wear, p.pos.x, p.pos.z, p.lift);
                for (Car& q : cars) printf("   car %d %-14s lap=%d s=%.0f v=%.1f d=%.2f\n", q.id, q.liv->name, q.lapsDone, q.s, q.speed, q.d);
            }
        }
    }
};

// ---------------------------------------------------------------------------
static void keyCb(GLFWwindow*, int key, int, int action, int) { if (action == GLFW_PRESS && G) G->keyQueue.push_back(key); }
static void scrollCb(GLFWwindow*, double, double dy) {
    if (!G) return;
    G->cam.overH = clampf(G->cam.overH * (dy > 0 ? 0.88f : 1.14f), 60.f, 1500.f);
    G->cam.chaseDist = clampf(G->cam.chaseDist - (float)dy, 6.f, 30.f);
}
static void sizeCb(GLFWwindow*, int w, int h) { if (G) { G->fbw = w; G->fbh = h; } }

int main(int argc, char** argv) {
    int W = 1280, H = 720; const char* scriptPath = nullptr;
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        if (a == "--script" && i + 1 < argc) scriptPath = argv[++i];
        else if (a == "--size" && i + 1 < argc) sscanf(argv[++i], "%dx%d", &W, &H);
    }
    setvbuf(stdout, nullptr, _IONBF, 0);
    if (!glfwInit()) { fprintf(stderr, "Failed to initialise GLFW\n"); return 1; }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3); glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif
    glfwWindowHint(GLFW_SAMPLES, scriptPath ? 0 : 4);
    GLFWwindow* win = glfwCreateWindow(W, H, "Real-Time 3D Formula 1 Racing and Interactive Pit Stop Simulation", nullptr, nullptr);
    if (!win) { fprintf(stderr, "Failed to create an OpenGL 3.3 window\n"); glfwTerminate(); return 1; }
    glfwMakeContextCurrent(win);
    glfwSwapInterval(scriptPath ? 0 : 1);
    if (!gladLoadGL((GLADloadfunc)glfwGetProcAddress)) { fprintf(stderr, "Failed to load OpenGL\n"); return 1; }
    printf("OpenGL %s | %s\n", glGetString(GL_VERSION), glGetString(GL_RENDERER));
    glEnable(GL_MULTISAMPLE);

    Game* g = new Game(); G = g; g->win = win;
    glfwGetFramebufferSize(win, &g->fbw, &g->fbh);
    glfwSetKeyCallback(win, keyCb); glfwSetScrollCallback(win, scrollCb); glfwSetFramebufferSizeCallback(win, sizeCb);
    g->init();
    if (scriptPath) g->loadScript(scriptPath);

    double last = glfwGetTime();
    bool quit = false;
    while (!glfwWindowShouldClose(win) && !quit) {
        glfwPollEvents();
        double now = glfwGetTime(); float dt = (float)(now - last); last = now;
        dt = std::min(dt, 0.1f);
        if (scriptPath) {
            const double TD = 1.0 / 30.0; bool shot = false; std::string path;
            g->runScript(TD, shot, path, quit);
            while (!g->keyQueue.empty()) { g->onKey(g->keyQueue.front()); g->keyQueue.pop_front(); }
            g->update((float)TD);
            if (shot) { g->render(); g->savePPM(path.c_str()); glfwSwapBuffers(win); }
            continue;
        }
        while (!g->keyQueue.empty()) { g->onKey(g->keyQueue.front()); g->keyQueue.pop_front(); }
        g->update(dt);
        g->fpsAcc += dt; g->fpsN++;
        if (g->fpsAcc > 0.5f) { g->fps = g->fpsN / g->fpsAcc; g->fpsAcc = 0; g->fpsN = 0; }
        g->render();
        glfwSwapBuffers(win);
    }
    glfwDestroyWindow(win); glfwTerminate();
    return 0;
}
