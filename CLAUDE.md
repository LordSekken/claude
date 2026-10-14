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
- Not yet confirmed by the owner: that FFB is felt and its direction (T must pull left first, else
  FfbInvert=True), steering weight, odd RPM/clutch behaviour they reported (check pedal readout).
- Headless UE runs fail with DXGI 887A002B (access denied) when the owner's display is off/away:
  add -RenderOffscreen.
- Headless UE testing: `UnrealEditor.exe <uproject> -ExecCmds="py <script>"` (not -ExecutePythonScript,
  which quits) with a slate post-tick callback driving PIE; HighResShot for screenshots. Compiling
  needs the owner's editor closed (Live Coding blocks UBT).

## Roadmap agreed with the owner
1. Done: Shinamoyama drivable, AC Miata model + physics, G920 + FFB (owner to verify). Next: graphics
   presets + VR preset (Quest 2 + 3070 Ti, OpenXR), ACC-level road detail (close-up stones, Nanite
   displacement), driver body/arms (paddles vs shifter vs handbrake), SDL gamepads (PS4/PS5/Switch/
   8BitDo), gamepad steering polish (owner: low priority, wheel first).
2. AC importers: AC track (.kn5, Drift Lab already reads kn5) -> Unreal; AC car model + physics
   (Drift Lab `scripts/ac/ac_car_importer.gd` already reads unpacked data/*.ini) -> DriftCore spec.
3. AC telemetry recorder + side-by-side comparison (same car/corner in AC vs DriftGame: slip angle,
   yaw rate, speed) to tune DriftCore to match AC, then exceed it (tyre temp/wear, tyre relaxation
   length, suspension geometry, richer FFB) - every upgrade verified against the graphs.
4. Later ideas (not active): own-city tracks from OSM / Gaussian splats (AKASO EK7000 footage),
   traffic, road/roundabout maker.

## Forza / track inference notes
FH packets 324 bytes, dash block at 244. SurfaceRumble per wheel at 148-160 (0 tarmac, 0.12 rough,
0.24 kerb). Forza world is mirrored vs Godot (mirror x). Track inference (`forza_session.gd`) bins
wheel contacts in 1 m slices for edges/kerbs/shoulders, regresses height vs lateral for camber.
