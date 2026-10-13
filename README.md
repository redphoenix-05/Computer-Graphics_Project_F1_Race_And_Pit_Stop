# Real-Time 3D Formula 1 Racing and Interactive Pit Stop Simulation

Computer Graphics course project — **C++17 · OpenGL 3.3 Core · GLFW · GLAD** (Windows / MinGW / MSYS2 UCRT64).

Five Formula 1 cars (one player car + four autonomous cars) race on a 3.45 km F1-style circuit with a full pit lane.
The player can drive manually or let the car drive itself, and can perform a **fully interactive pit stop**
(guided pit entry → jacks → four-tyre change by an animated 8-man crew → release → race resumption).

---

## How to build and run

```cmd
run.bat
```
or
```cmd
g++ -std=c++17 -O2 src/main.cpp src/glad.c -Iinclude -Llib -lglfw3dll -lopengl32 -lglu32 -lgdi32 -o main.exe
main.exe
```
or `mingw32-make run`, or the VS Code build task (`Ctrl+Shift+B`).
`glfw3.dll` must sit next to `main.exe` (it already does). A prebuilt `main.exe` is included.

Requires an OpenGL 3.3 capable GPU/driver.

---

## Controls (exactly as in the project report)

| Key | Function |
|---|---|
| **A** | Automatic driving mode |
| **M** | Manual driving mode |
| **↑ / ↓** | Accelerate / Brake (reverse when stopped) |
| **← / →** | Steer left / right |
| **Y** | Request an **automatic** pit stop - the crew does the whole stop by itself (about 3 s stationary) |
| **U** | Request a **manual** pit stop - you press R, C, D, Q step by step |
| **P** | Request pit entry using the last chosen mode (default: manual); press again to cancel |
| **R** | Raise the car with the hydraulic jacks (only when stopped in the pit box) |
| **C** | Perform the tyre replacement (only when the car is raised) |
| **D** | Lower the car (only after the tyres are changed) |
| **Q** | Exit the pit and resume racing (only after the car is lowered / green light) |
| **1–9** | Cockpit · Chase · Trackside · Overhead · Pit · Broadcast · Low Chase · Helicopter · Head-on camera |
| **G** | Gouraud shading (all objects) |
| **H** | Phong shading (all objects) |
| **V** | Toggle procedural audio (mute / unmute) |
| **Space** | Pause / resume |
| **N** | Restart the race (new start-light sequence) |
| **T** | Open the optional ray-tracing demonstration |
| **Esc** | Exit the application |

Extra keys: **J** hybrid shading (Gouraud environment + Phong cars/tyres/pit equipment — the report's default scheme),
**L** day / dusk / night lighting, **W** wireframe, **V** procedural sound toggle (engine RPM, tyre screech, pit guns), **5** (repeat) cycles three pit-camera angles,
**PgUp / PgDn** or mouse wheel zoom (overhead height / chase distance), **F** fullscreen, **TAB** hide HUD, **F1** help.
Inside the ray-tracing demo: arrows orbit the camera, **Space** re-renders, **S** saves `raytrace.bmp`, **T** returns.

### How a pit stop works
1. Press **Y** (automatic stop) or **U** (manual stop) - or **P** to reuse the last mode - on any part of the lap, in automatic or manual driving.
   The HUD shows *PIT REQUESTED*. When the car reaches the pit-entry window on the main straight (after the last corner) it commits to the
   pit lane and follows the pit-lane path; the 100 km/h limiter starts at the pit-entry line. Pressing the key again before the entry cancels the request
   (pressing the *other* key switches the mode; Y / U also switch the mode during a stop that is already underway).
2. The car stops exactly in its pit box.
3. **Automatic (Y):** raise -> tyre change -> lower -> release happen on their own; the car is stationary for about **3 seconds**.
   **Manual (U):** each step waits for you:
   * **R** - jack operators slide the jacks under the nose/gearbox and lift the car 30 cm.
   * **C** - the crew runs in: wheel guns loosen the nuts, old tyres are flung away, the tyre-delivery man brings new wheels, new wheels are carried
     and attached, tightened, thumbs-up. The compound changes (e.g. MEDIUM -> HARD) and tyre wear resets.
   * **D** - the jacks retract and the car is lowered; the pit lights switch from red to **green**.
   * **Q** - the car is released, the crew clears the area, the car follows the pit lane to the exit and merges back onto the circuit.
4. The AI cars keep racing the whole time. The pit lane is short (about 340 m of pit wall), so a complete automatic stop costs roughly 12 s of lap time.

In manual mode the keys only work at the correct stage (e.g. **C** before **R** shows a hint instead of doing anything). Red/amber/green pit lights on the gantry signal the stop (no sign holder needed).

---

## Realism notes (v2)
* **Cars** are modelled after modern ground-effect F1 cars: smooth lofted monocoque with a slim tapered nose, undercut sidepods with louvres,
  airbox with intake, halo, 4-element curved front wing and DRS-style rear wing with shaped end-plates, diffuser, wheel covers,
  brake drums and coloured compound rings. Team liveries are inspired by black/teal, navy/yellow/red, papaya, red and green cars.
  Sponsor names (fictional), race numbers and tyre side-wall lettering are real text *decals* built from the font atlas.
* **Racing**: AI cars have time-varying pace, use the slipstream, choose a free passing lane and overtake - the running order changes
  continuously. Lane spacing and collision handling were reworked (no more jitter between neighbouring cars).
* **Pit lane**: all signs/poles are kept clear of the car path; a start-up check (`pit path clearance check` in the console)
  verifies this against every upright obstacle.
* **HUD text** uses a 40 px font atlas with mip-mapping and pixel-snapped glyph placement, so it stays sharp at any window size.

## Project structure

```
src/
  main.cpp      application: window, input, race control, collisions, rendering pipeline, HUD
  math3d.h      vec3 / mat4, translate / rotate / scale, perspective, lookAt, inverse
  gfx.h         GLSL shaders (Gouraud + Phong), MeshBuilder, chunked static batches, frustum culling, HUD font renderer
  track.h       circuit spline, curvature-based speed profile, pit-lane geometry
  car.h         hierarchical scene graph, F1 car + wheel models, vehicle dynamics, waypoint driver
  pit.h         pit-stop state machine, humanoid crew (hierarchical), jacks, wheel animation, pit lights
  world.h       procedural venue: road, kerbs, barriers, fences, garages, grandstands, towers, scenery
  camera.h      the six camera views
  raytrace.h    CPU ray tracer for the optional demonstration
  font_data.h   embedded bitmap font atlas
  glad.c        GLAD loader (unchanged)
tests/          scripted test scenarios (see below)
```
Everything is header-only modules included by `main.cpp`, so the original single-file build commands keep working.

---

## Computer graphics concepts → where they are implemented

| Concept (report §4) | Implementation |
|---|---|
| **3D modelling** | `MeshBuilder` in `gfx.h` builds boxes, cylinders/cones, spheres, lofts and *surfaces of revolution* (tyres, rims). Cars: monocoque, nose, airbox, shark-fin, sidepods, front/rear wings, halo, mirrors, suspension, diffuser (`buildCarBody` in `car.h`). |
| **Hierarchical modelling** | `Node` scene graph (`car.h`): car → body-tilt → body / helmet / steering wheel / 4 × (hub → steer → spin → wheel). Wheels are separate nodes so they can be detached and replaced. Crew members are articulated humanoids: pelvis → torso/head, shoulder → elbow → hand (+ wheel gun), hip → knee (`pit.h`). The jack, ram and saddle form a small hierarchy too. |
| **Translation** | Cars along the circuit, crew paths, jack sliding, car lift, tyre carrying. |
| **Rotation** | Wheel spin, front-wheel steering, car heading, body roll/pitch, crew joints, steering wheel. |
| **Scaling** | Unit primitive meshes scaled into limbs, tyres, signs, shadows; non-uniform scale handled with the correct normal matrix (cofactor matrix, `normalMatrix()`). |
| **Lighting & materials** | Blinn-Phong with hemispheric ambient, a directional sun, and up to 16 point/spot lights: garage spotlights, trackside flood lights, red/green/amber pit warning lights. Per-vertex material (colour, specular, shininess, emission) for car paint, rubber, metal, asphalt, glass, concrete. Day / dusk / night presets. |
| **Gouraud shading** | `GOURAUD_VS_BODY`: lighting evaluated per vertex and interpolated. |
| **Phong shading** | `PHONG_*`: normals interpolated, lighting evaluated per fragment. |
| **Shading switch** | **G** all Gouraud · **H** all Phong · **J** hybrid (environment Gouraud; cars, tyres, rims, pit equipment Phong). |
| **Motion & animation** | Frame-rate-independent fixed 120 Hz simulation step; wheel rotation, steering, body pitch/roll, crew key-frame paths with smooth interpolation and walk cycles, jack lifting, flying tyres, flashing pit lights, start-light sequence. |
| **Keyboard interaction** | `Game::onKey` + polled arrow keys; pit keys are gated by the pit-stop stage. |
| **Ray tracing (optional)** | `raytrace.h`: spheres, boxes, capped cylinders, chequered reflective floor, 2 lights, hard shadows, 3-bounce reflections, progressive anti-aliasing; time-sliced so the UI never freezes. |
| **Collision handling** | Barrier / pit-wall / pit-lane limits (with bounce and speed loss), car-vs-car circle tests, off-track grip loss. |
| **Waypoint driving** | Pure-pursuit steering on the spline centre line (+ lane offset) with a curvature/braking/acceleration-limited speed profile (`Track::speedProfile`); gap keeping behind other cars. |

### The venue (report §3.3)
Asphalt with painted edge lines, red-and-white kerbs, paved and gravel run-off, tyre walls on the outside of corners, concrete barriers
with advertising panels, catch fences, brake-distance boards (300/200/100 m), grandstands with thousands of spectators and flags,
race-control tower and timing tower, TV camera platforms and marshal posts, a pit wall with team consoles, **5 team garages** with
equipment (tool cabinets, tyre racks with tyres, monitors, carts, lights), pit entry/exit signs, 100 km/h limit signs, pit-light gantry,
start gantry with 5 start lights, chequered start line, grid slots, trees and distant hills. Static geometry is split into spatial chunks
that are frustum- and distance-culled.

### Cameras (report §3.6)
1 Cockpit (driver's eyes with halo & steering) · 2 Chase (smoothed, speed-dependent) · 3 Trackside (nearest fixed TV camera, zooms on the car) ·
4 Overhead (follows the car; zoom out to see the whole circuit) · 5 Pit (3 angles) · 6 Broadcast (automatic director: trackside, helicopter,
head-on, low chase; cuts to the pit cameras during a stop).

---

## Notes

* The simulation prioritises convincing movement and responsive controls over full vehicle physics, as stated in the report's scope.
* Lap ≈ 60 s for the AI cars. Tyre wear slowly reduces grip, so a pit stop is worthwhile.
* If your GPU is slow, press **G** (cheaper per-vertex lighting) or run in a smaller window.

### Automated test harness
`main.exe --script tests/pit_stop_test.txt --size 1280x720` runs the simulation with virtual time, injects key presses and saves
screenshots (`shot file.ppm`). Script lines are `<time> <command> [args]` with commands `key X`, `hold X seconds`, `shot file`,
`tele <s> <speed>`, `view ex ey ez tx ty tz fov`, `diag`, `skip`, `print`, `quit`. It was used to verify the full pit-stop sequence, lap behaviour, collisions and all cameras.
