# Project handoff: drift game (read this first)

The owner wants their own drift game with better physics and visuals than Assetto Corsa:
record real/Forza tracks, drive them in AC *and* in their own Unreal game, and eventually
load AC cars/tracks into it. Work branch: `claude/drift-simulator-map-gen-k0sgp9`
(push only there; no PRs unless asked). The rest of the repo root (`src/`, `index.html`,
vite files) is an unrelated web app.

## Working style the owner asked for
- Batch changes; only hand over a new build when a meaningful, tested batch is ready.
- Give exact click-by-click steps (they're new to GitHub/Unreal). Check instructions are current.
- Ask before creating files in their Google Drive. Never commit AC mod assets (Kunos/mod content).
- Don't build a `data.acd` decryptor (Kunos/mod-author protection). Packed AC cars are unpacked by
  the user in Content Manager ("Unpack data"); the importer should guide them, and maybe launch CM.

## Layout
- `drift-sim/` – **Drift Lab** (Godot 4.4.1, GDScript). Garage/car builder, track editor,
  Forza Data Out recorder (`scripts/mapgen/forza_*.gd`), AC export (`scripts/ac/`), Unreal export
  (`scripts/unreal/unreal_track_exporter.gd`). Tests: `godot --headless --path drift-sim -s tests/run_tests.gd`
  (258 passing). Tools: `tools/export_ac_tracks.gd`, `tools/export_car_specs.gd`, `tools/build_hd_textures.py`.
- `unreal/DriftCore/` – engine-independent C++17 drift physics (no exceptions/RTTI). Own rigid body
  at 480 Hz, raycast suspension, combined-slip tyres, engine/turbo/clutch/gearbox/diffs, wall
  contacts, FFB. Frame: right-handed, +X right, +Y up, -Z forward, metres. Host implements
  `drift::World` (Raycast, BodyContacts). Build/test: `cmake -S unreal/DriftCore -B build && cmake --build build && ctest --test-dir build`.
- `unreal/DriftGame/` – **UE 5.5 project** (owner has UE 5.5.4 + VS 2022). `ADriftCarPawn` runs
  DriftCore (`DriftUE.h` axis map: UE.X=-z, UE.Y=x, UE.Z=y, x100 cm), `FDriftUnrealWorld` does line
  traces/overlaps, surface from mesh name (ROAD/KERB/GRASS/DIRT/GRAVEL/SAND/WALL) or component tag.
  Enhanced Input built in C++ (Forza-style pad + keyboard), HUD, chase/cockpit cams, placeholder box body.
  Cars: `Content/DriftData/Cars/*.json` (Drift Lab builds; default `outback98_wrx_rwd_drift`, the RWD EJ205 Legacy).
  `Scripts/build_track_level.py` imports `Tracks/<id>/<id>.glb` (from Drift Lab "Export for Unreal..."),
  sets complex collision, builds Poly Haven world-projected PBR materials, sky/sun/fog, PlayerStart from
  START/START_AHEAD markers, sets default map. Run in the Output Log (Cmd mode): `py "<full path>\build_track_level.py"`.
- CI: `.github/workflows/drift-lab.yml` (tests + Windows build + AC tracks; artifact DriftLab-windows),
  `.github/workflows/drift-core.yml` (DriftCore on GCC + MSVC).

## Status (where we left off)
- The owner uses `C:\Dev\claude\unreal\DriftGame` (the git checkout); the `C:\Dev\claude-claude-...`
  folders are old copies. Shinamoyama (FH6 recording, `user://tracks/forza_zone.json`, camber 75%)
  drives. Fixed: inverted road winding, checkerboard (roughness sampler must be Masks), car riding on
  the merged trees' auto collision box. DX12 + SM6 on (Nanite).
- AC cars: `tools/export_unreal_car.gd -- <ac car dir> <unreal project>` then `Scripts/build_car.py`
  (selects it via Content/DriftData/SelectedCar.txt). Test car: BDC Street v4 Miata in
  `...\DriftGame 5.5\test ars\bdc_streetspec_miata_v4` (data unpacked into its data/). glTF import is a
  +90 deg yaw off DriftUE (fixed on the car meshes). AC STEER_RATIO is signed - use its magnitude.
- G920 (Xbox version): ffb_helper.exe (built locally with MSVC + SDL3 3.2.8 VC package into
  Binaries/Win64 - not in git) streams "DLIN" controls (SDL) and runs FFB through raw DirectInput:
  SDL haptics never moved the G920; Logitech's driver only accepts a 1-axis cartesian effect on
  DIJOFS_X with direction +1. The game starts/stops the helper. Mapping: Config/DefaultGame.ini
  [DriftWheel] (G920 Xbox: A=0 handbrake, X=2 reset, shifter R=11, gears 1-6 = 12-17). Cameras
  chase/cockpit/hood (C or Y). T = FFB test. HUD top lines show wheel/pedal/button readout.
- FFB works on the owner's G920 (Logitech takes the push side from the effect direction, not the
  magnitude sign; controls are read from the same DirectInput device, axes by name/type).
- AC comparison (roadmap step 3, started): `Record AC Telemetry.bat` (ac_telemetry.exe, AC shared
  memory -> Documents\DriftLab\telemetry\ac_*.csv); DriftGame writes dg_*.csv in the same layout/signs;
  `drift-sim/tools/compare_telemetry.py` compares them. Owner's AC FFB: gain 70%, filter 4%, min 0,
  road 50, kerb 20, slip 9, ABS 21, near-linear g920.lut. DriftCore FFB now follows AC's measured
  force-vs-front-slip curve. Findings: importer had used tyres.ini's first compound (fixed: default
  compound + DY0/DY1 at 3.5 kN; peak slip angle = 0.75x FRICTION_LIMIT_ANGLE, calibrated to AC front slip vs
  lateral g), handbrake per wheel, CG height = radius + BASEY (was a 0.45 m guess; Miata 0.175), AC TRACK
  kept over model wheel pivots, sliding friction along slip velocity. Understeer ~0.6-1.0 deg/g (AC 0.55). Next:
  no camber model (BDC front static camber -4.6 deg) - add camber + roll geometry from AC data.
- AC replay scorer (`unreal/DriftCore/tools/ac_replay.cpp`, built with DriftCore): starts DriftCore in AC's
  recorded state for 0.25 s windows with the recorded inputs. BDC Miata/Brooklyn: RMS 2.7 deg (baseline 8),
  gain 0.93. AC shared memory updates at the frame rate (~64 Hz), not 100 Hz: old CSVs have fake t.
- Graphics presets Low/Medium/High/Epic/VR (`DriftGraphics`, F5, `drift.Graphics N`).
- Multiplayer (listen server): owner simulates own car, FDriftNetState replicated ~30 Hz (COND_SkipOwner),
  remote cars interpolated 100 ms behind. No car-to-car collision yet. Host/Join .bat files in DriftGame/.
  Tested with two -game -RenderOffscreen processes (logs: LogDriftNet Verbose). Pawn must NOT AutoPossess.
- Headless UE runs fail with DXGI 887A002B (access denied) when the owner's display is off/away:
  add -RenderOffscreen.
- Headless UE testing: `UnrealEditor.exe <uproject> -ExecCmds="py <script>"` (not -ExecutePythonScript,
  which quits) with a slate post-tick callback driving PIE; HighResShot for screenshots. Compiling
  needs the owner's editor closed (Live Coding blocks UBT).

## Roadmap agreed with the owner
1. Done: Shinamoyama drivable, AC Miata model + physics (AC-replay matched), G920 + FFB, graphics
   presets, multiplayer (listen server, car contact), VR prep (OpenXR; untested on the Quest 2), layered
   road materials. Next: owner test drive of this batch; road realism (texture filtering, Nanite
   displacement, tyre marks, lighting), driver body/arms, SDL gamepads (PS4/PS5/Switch/8BitDo),
   Steam sessions instead of IP join, camber model if replay shows a need.
2. AC importers: AC track (.kn5, Drift Lab already reads kn5) -> Unreal; AC car model + physics
   (Drift Lab `scripts/ac/ac_car_importer.gd` already reads unpacked data/*.ini) -> DriftCore spec.
3. AC telemetry recorder + side-by-side comparison (same car/corner in AC vs DriftGame: slip angle,
   yaw rate, speed) to tune DriftCore to match AC, then exceed it (tyre temp/wear, tyre relaxation
   length, suspension geometry, richer FFB) - every upgrade verified against the graphs.
4. Later ideas (not active): own-city tracks from OSM / Gaussian splats (AKASO EK7000 footage),
   traffic, road/roundabout maker.

### Owner's vision (2026-10-17) - priority order
- Product: a modern, buy-download-play drift sim (no modding needed) with physics realistic enough that
  sim min-maxers (friends Corey, John) prefer it; drift first, then rally stages / GT tracks (Monza, Spa)
  reuse the same "just realistic" physics. Multiplayer with friends is the point. VR that looks good and
  runs well (AC/ACC run badly in VR).
- Physics tuning philosophy (owner agreed): make each component physically right (geometry, mass, springs,
  published tyre data) and validate with standard manoeuvres (skidpad, step steer, sine sweep, braking,
  power oversteer) and ideally real-car phone GPS/IMU logs; AC telemetry is a cross-check, not the target
  (matching one log can hide cancelling errors). Tyres must have real ground contact (not soft-body FEA).
- Cameras: cockpit, cockpit-forward (wheel + dash), hood, bumper/grille, two chase, sideways drift cam;
  NeckFX (head-tracking plugin) support.
- Player models/skins (cel-shaded characters OK, e.g. Goku animated at 12-18 fps), rewind, photo mode.
- Later: AI car builder from service manual + spec sheet; soft-body crash damage (BeamNG-like);
  more Forza telemetry / FH5 & FH Japan drift zones; procedural drift-map generator (needs a model of
  what makes drift corner sequences fun).
- Test map: Brooklyn Park (not the personal Forza tracks).

### Chrono evaluation (started 2026-10-17)
- Chrono 10.0.0 at `C:\Dev\chrono` (local patch: ChPac02Tire.cpp slip clamp removed so Fy/Fx fall off
  past the peak), built Release core+vehicle in `C:\Dev\chrono_build` (Eigen 3.4.0 + CMake 4.4.4 in
  `C:\Dev\tools`). `drift-sim/tools/ac_to_chrono.py` converts an AC car + Drift Lab spec into Chrono JSON
  (double wishbones from AC hardpoints, static camber, Pac02 .tir scaled to AC grip) in `C:\Dev\chrono_cars`
  (not in git). `unreal/ChronoLab` (`chrono_lab perf|skidpad|replay|diag|reset`) - build dir `C:\Dev\chrono_lab_build`.
- Patch: `unreal/ChronoLab/chrono-10.0.0-pac02-falloff.patch` (apply to a fresh Chrono checkout).
- Solver: BARZILAIBORWEIN (100 it) for stepping - clutches/brakes are boxed (friction) constraints that a
  direct solver ignores (SPARSE_LU locked the clutch). SPARSE_LU only for the replay state reset (exact
  assembly). Reset must: re-aim the steering actuator + POSITION assembly, set wheel spins (+ about vehicle
  +Y = forward) and every ShaftBodyRotation shaft, iterate VELOCITY assembly + vehicle Synchronize (the
  gearbox driveshaft is kinematic), set gearbox input = output / ratio, and never rewind ChTime.
- Miata: 3.5x real time at 1 kHz on one core (~0.29 ms/step). Skidpad understeer 0.13 deg/g (AC 0.55).
  Steering lock calibrated to AC (`--calibrate=`; rack is non-linear). AC replay (Brooklyn, 1671 windows,
  glitch-filtered): Chrono untuned RMS 1.69 deg, corr 0.98, gain 1.07 (rotates ~1 deg more), speed -1 km/h
  per window at full gas; DriftCore (tuned to AC) RMS 1.11, corr 0.99, gain 1.02. Missing in the Chrono car:
  rolling resistance (old-format .tir), turbo boost map, camber effect on grip (PDY3=0), relaxation length.
- AC importer mass fixed: TOTALMASS includes the driver, + fuel x 0.74 (was -75 kg).
- In the game (2026-10-17): `unreal/DriftChrono` = DriftChrono.dll (C API `include/drift_chrono.h`, DriftCore
  frame; build `cmake -S unreal/DriftChrono -B C:/Dev/drift_chrono_build -DChrono_DIR=C:/Dev/chrono_build/cmake`,
  test `dc_smoke <ChronoCars dir> <car>`). DriftGame loads it at runtime (`DriftChronoBackend.cpp`) from
  Binaries/Win64 with Chrono_core.dll + Chrono_vehicle.dll (copied there, not in git); car files in
  Content/DriftData/ChronoCars/<spec id>/ (ac_to_chrono.py output, gitignored). Falls back to DriftCore if
  missing. DriftCore runs the driver aids (`Vehicle::ControlStep`) and takes Chrono's state
  (`ApplyExternal`) so HUD/telemetry/net/visuals are unchanged. F6 toggles physics; [DriftPhysics] Backend,
  [DriftChrono] FfbRackForce (rack N for full FFB). FFB = Chrono steering-rack load. Body contacts: walls/cars
  only (floors are the tyres' job) as penalty springs. Chrono neutral (1e20 ratio) blows up: DLL keeps a gear
  and opens the clutch. Pac02 patch also adds transient slip (relaxation length from the .tir PTX/PTY): the
  steady-state slip made free-rolling front wheels oscillate below ~15 km/h (24 kN rack spikes).
- Headless UE tests: `drift.AutoThrottle/AutoSteer/PhysicsLog/NoWheel/UseDriftCore` cvars (set NoWheel before
  Play so tests never drive the owner's G920). Headless editor runs ~3 fps (RenderOffscreen).

## Forza / track inference notes
FH packets 324 bytes, dash block at 244. SurfaceRumble per wheel at 148-160 (0 tarmac, 0.12 rough,
0.24 kerb). Forza world is mirrored vs Godot (mirror x). Track inference (`forza_session.gd`) bins
wheel contacts in 1 m slices for edges/kerbs/shoulders, regresses height vs lateral for camber.
