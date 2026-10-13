# DriftGame (Unreal Engine 5)

Your own drift game: **DriftCore** physics (C++, `DriftCore/`, tested in CI) driving in
**Unreal Engine 5** visuals, on tracks made or recorded in **Drift Lab**.

## One-time setup (Windows)

1. Install **Unreal Engine 5.4 or newer** (Epic Games Launcher → Unreal Engine → Library).
2. Install **Visual Studio 2022** (Community is fine) with the workload
   **"Game development with C++"** (it adds the Unreal tools and Windows SDK).
3. Get this repo on your PC (GitHub Desktop → clone, or Code → Download ZIP).
4. Right-click `unreal/DriftGame/DriftGame.uproject` → **Switch Unreal Engine version…** → pick yours.
5. Double-click `DriftGame.uproject`. It says the DriftGame module is missing → **Yes** to rebuild.
   The first build takes a few minutes. If it fails, send the log from
   `DriftGame/Saved/Logs/` and the error text.

## Put a track in

1. In Drift Lab, open the track (Track Editor, or the Forza recorder after building from a session)
   → **Export for Unreal…** → choose `unreal/DriftGame/Tracks/`.
2. In Unreal: **Window → Output Log**. Switch the box at the bottom from `Cmd` to `Python` and run
   `py "Scripts/build_track_level.py"` (it builds every folder in `Tracks/`. Or pass one folder path).
   It downloads the photo-scanned textures (CC0, Poly Haven) the first time, so it needs internet.
3. Press **Play**.

## Controls

| | Gamepad | Keyboard |
|---|---|---|
| Steer | Left stick | A / D |
| Throttle / Brake | RT / LT | W / S |
| Handbrake | A | Space |
| Shift up / down | B / X | E / Q |
| Camera (chase / cockpit) | Y | C |
| Reset car | View | R |
| Auto / manual gears | Right stick click | M |

## Cars

`Content/DriftData/Cars/*.json` are Drift Lab builds (exported by `drift-sim/tools/export_car_specs.gd`,
also included in every Drift Lab build as `car_specs/`). Select the `DriftCarPawn`, or make a Blueprint
child of it, and set **Car Spec** (default: `outback98_wrx_rwd_drift`, the RWD EJ205 Legacy).
The body is a placeholder box for now: in a Blueprint child you can swap the Body mesh for a real car model.

## How it fits together

- `DriftCore/` – the whole car simulation in plain C++ (tyres, suspension, engine, diffs, its own
  rigid body at 480 Hz, force feedback). Unreal only answers "where's the ground under this wheel"
  and "is the body hitting something" (`Source/DriftGame/DriftUnrealWorld.cpp`).
- The surface under each wheel comes from the mesh name (`ROAD_*`, `KERB_*`, `GRASS_*`, `DIRT_*`…)
  or a component tag with one of those words, so hand-built levels work too.
