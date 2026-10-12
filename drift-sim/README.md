# Drift Lab

A drift simulator for Windows (Godot 4). It is built around three things:

- **Modular, real-world cars.** A car is a *chassis + engine + transmission + drivetrain + tune*, all swappable. Put a WRX motor in a '98 Legacy Outback, convert it to RWD, weld the diff, add an angle kit, and the weight, balance, power and gearing all follow.
- **Tracks from pictures.** Trace a satellite/map screenshot or a sketch offline. Or hand *any* photo of a place to Claude, which designs a drift course from it, checks it, looks at a render of what it built and refines it.
- **Forza-style drift scoring.** Chains, multipliers, transitions, clipping points, wall proximity, judged zones with star ratings.

Out of the box it plays with keyboard, Xbox-style controllers and steering wheels with **force feedback** (e.g. a Logitech G920), from a **first-person cockpit**, on a monitor or in **VR** (Quest 2 etc. via OpenXR).

---

## Getting it

**Windows build:** every push builds it. Open the repo's **Actions** tab, pick the latest *"Drift Lab - test & Windows build"* run, and download the **DriftLab-windows** artifact. Unzip it and keep the files together:
- `DriftLab.exe` is the game.
- `Drift Lab VR.bat` starts it in VR.
- `ffb_helper.exe` + `SDL3.dll` handle wheel force feedback.

**From source:** install [Godot 4.4](https://godotengine.org/download) (standard build, not .NET). Open `drift-sim/project.godot` and press F5.

## Controls

| | Keyboard | Xbox controller | Wheel (G920 etc.) |
|---|---|---|---|
| Steer | A/D or ←/→ | Left stick | Wheel |
| Throttle / brake | W/S or ↑/↓ | RT / LT | Pedals |
| Handbrake | Space | A | Bind any button or axis |
| Clutch | Shift | – | Clutch pedal |
| Shift up / down | E / Q | B / X | Paddles or H-shifter |
| Camera | C | Y | – |
| Reset car | R | Back/View | – |
| Pause | Esc | Start/Menu | – |

The menus work with a controller too (D-pad/stick + A).

**Setting up a wheel (G920):** go to *Settings & Controls*, tick *Use a steering wheel*, then click **Bind** for each control and move it once: turn the wheel right, press each pedal, press each paddle. Drift Lab records which axis moved and how far, so any pedal direction or range works. Set the wheel rotation to match G HUB (900° is a good default for drifting). In G HUB, turn **off** the centering spring.

**Force feedback:** you feel self-aligning torque from the front tyres. The wheel goes light as the fronts pass their grip limit and snaps back through a transition. Kerbs rumble, grass and gravel shake, and wall hits kick. Settings has strength and damping sliders and a **Test** button that pulls the wheel left, then right; the status line next to it shows whether a wheel was found, and **Open log** shows `%TEMP%\\DriftLab_ffb.log` (every device the helper saw and why it did or didn't use it). Close other force-feedback tools while playing; only one app can drive the wheel's motor at a time. Godot can't drive DirectInput force feedback itself, so the game streams forces to `ffb_helper.exe` (SDL3, source in `native/ffb/`). It cuts the force within 250 ms if the game stops, and exits when the game closes. Xbox controllers get rumble instead.

**Cameras & VR:** on a monitor the hood cam is the default (C cycles chase / close chase / hood / cockpit; your choice is remembered). VR always puts you in the cockpit. On a monitor the view glances into the drift automatically; Z/X or the right stick look around manually. Numpad 8/2 moves the seat forward/back and 9/3 up/down.

For **Quest 2**: connect with Quest Link, Air Link or Virtual Desktop (OpenXR runtime active), then run `Drift Lab VR.bat`. Pick your car and track on the PC monitor and press Drive; you're then in the driver's seat. V or right-stick click recentres your view. Score and popups float in front of the car. In VR, Start pauses, and while paused Back resets the car and Y quits to the menu.

**Assists** (Settings): auto clutch and automatic gearbox are on by default for couch play. *Steering assist* only applies to pad/keyboard: when you let go of the stick, the front wheels settle toward the direction of travel, the way caster does on a real car. That makes counter-steer manageable without a wheel. Set it to 0 for raw steering.

## Cars: the Garage

Pick a **chassis**, **engine**, **transmission** and **drivetrain** (stock / RWD / AWD / FWD), then tune:
boost, ECU/bolt-ons, rev limit, final drive, diff (open / clutch LSD with power/coast/preload / welded), AWD split, springs, dampers, anti-roll bars, ride height, steering lock (angle kit), Ackermann, brake bias, hydraulic handbrake, tyre compound and size per axle, and weight reduction. The stats panel updates live: hp/torque curve peaks, mass, weight distribution, hp/tonne, 0-100, gear speeds. Warnings flag things like a gearbox that won't survive the torque.

Swaps are physical: changing the engine changes mass and front/rear balance (by where the engine sits relative to the front axle). A drivetrain conversion adds or removes the front diff, prop shaft and transfer case.

**Included parts** (real published specs; suspension rates and CG heights are estimates):
- Chassis: 1998 Subaru Legacy Outback (BG9), Nissan 240SX/Silvia S14, BMW E36 328i, Toyota AE86, Mazda MX-5 NA / NB / NC / ND2
- Engines: Subaru EJ25D, EJ205 (02-05 WRX), EJ257 (STI), Nissan KA24DE, SR20DET, Toyota 2JZ-GTE, 4A-GE, BMW M52B28, Mazda B6ZE, BP-ZE, BP-Z3, BP turbo (Mazdaspeed), LF-VE (NC), SKYACTIV-G 2.0 (ND2), Chevy LS3
- Gearboxes: Subaru TY754 5MT, TY856 6MT (STI), Nissan FS5W71C, BMW ZF S5D, Toyota T50, R154, Mazda M5OD, NB/NC/ND 6-speeds

**Miatas:** BDC v6 NA and NB recreations (435 bhp / 504 Nm / 1110 kg from BDC's published specs, CD009 6-speed, locked diff, ~62° lock; they use the BDC models from your AC install if present), NA 1.8 welded grassroots, NA BP-turbo with NB 6-speed, NB Mazdaspeed drift spec, NC welded street drifter, ND2 club drift, and the NA LS3 swap.

**Featured: '98 Outback, EJ205 swap, RWD drift build.** The budget Subaru drift recipe: 02-05 WRX EJ205 + STI 6-speed, DCCD locked and front axles pulled for RWD, welded R180, ~1.15 bar on a tune (≈266 hp), coilovers ~95 mm lower than Outback height, 58° angle kit, hydro handbrake, interior stripped. Compare it with the bone-stock Outback, which (correctly) refuses to slide on the handbrake because the viscous centre diff ties the rear wheels to the front.

### Ask the Garage agent (Claude)
Type something like:

> My '98 Legacy Outback with a WRX engine, converted to RWD, with a realistic street/drift tune

Claude checks the parts library. If something is missing, it researches the real specs (web search; give it your VIN and it decodes it through the free NHTSA vPIC database) and creates the part. It then assembles and evaluates the build with the game's own physics code, adjusts until the build is realistic, and saves it. *Tweak current build with Claude* edits whatever you have loaded ("make it a 2JZ instead", "softer rear, more lock").

Needs an Anthropic API key (Settings, or the `ANTHROPIC_API_KEY` environment variable). It uses `claude-opus-5-5`. Requests opt into server-side fallback, so a declined request is retried on a fallback model automatically. The key is stored only on your PC.

## Tracks

**Built in:** *Kanto Lot* (flowing club circuit), *Minami Tech* (tight, walls close, night), *Usui Downhill* (point-to-point touge with guardrails), *Practice Lot* (big open pad).

**Track editor:** click to add points, drag to move, Shift+click to insert, right-click to delete. Set width, run-off surface, walls, kerbs, time of day and per-point elevation/width. Add drift zones and clipping points, then validate, save, test-drive, or export to Assetto Corsa. The validator catches crossing sections, corners too tight for the road width, and unrealistic grades.

**Photo → Track:**
- *Trace offline* (no internet): works on top-down images (satellite/map screenshots, drone shots of a lot) and on sketches. It segments the asphalt (or ink, or a colour you click), thins it to a centreline, finds the loop, and scales to metres from the road's pixel width. You can also turn a lot into an open pad.
- *Generate with Claude*: works on **any** picture, including a photo taken standing in the place. Claude works out the scale from things like lanes and parking bays and can run the tracer itself for exact geometry. It submits a track, gets back validation results plus a rendered top-down preview, compares that to your photo, and iterates.
- Open the result in the editor (with your photo as a background layer) to fine-tune.

## Record a track from Forza (Data Out telemetry)

Rebuild your favourite Forza Horizon / Motorsport drift zones by driving them:
1. In Forza: **Settings → HUD and Gameplay → Data Out: ON**, IP `127.0.0.1` (or this PC's IP if Forza runs on an Xbox), port `5300`.
2. Drift Lab → **Record from Forza** → **Listen**. The live line shows packets arriving, speed and position.
3. Record three passes from the same start to the same end: hugging the **left edge** (slowly, tyres on the edge of the tarmac), hugging the **right edge**, then your **driving line**. Press **Record** / **Stop** for each.
4. **Make track** builds the real layout from the car's world position (~60 samples/s):
   - the road centre and width come from the two edges (not from your drift line), matched along the road so hairpins don't snap to the other side of the hill;
   - camber comes from the height difference between the edges; elevation from every pass;
   - kerbs go exactly where Forza reported a tyre on a rumble strip;
   - edge points where the outer tyres were in the grass/dirt (much more surface rumble than the inner tyres) are ignored, so running wide doesn't widen the road.
   - passes don't need to be perfect: edges can be recorded in several overlapping pieces or driven backwards, and turnarounds/overshoots are cut off; hairpins keep their walls out of the other leg.
   Then **Open in Track Editor** (walls, zones, clips), **Save + Drive**, or **Export for Assetto Corsa**.

**Record whole session (raw)** saves every telemetry packet of a session (e.g. 20 minutes of Rivals laps, wipeouts included) to `%APPDATA%\DriftLab\recordings\raw\*.fzr` (~1.2 MB/min) so the track can be analysed again later without re-driving.

Recordings are saved to `%APPDATA%\DriftLab\recordings` so they can be shared or rebuilt later. The game's packet layout and its axis handedness are detected from the physics (speed vs. velocity, lateral g in corners), not hard-coded, so newer games (e.g. Forza Horizon 6) with a shifted layout are found by scanning the packet.

## Scoring (Forza-style)

- Points accrue while sliding (over 20 km/h and 10°). The rate grows with speed and angle.
- The **multiplier** (×1-×5) builds with sustained drifting, **transitions** (switching sides), **clipping points** (yellow poles: get the bumper inside the ring) and **proximity** (tail within ~1.2 m of a wall without touching).
- The chain **banks** after 2 s of not drifting. **Wrecking** into a wall, going **off track** or **spinning out** loses the unbanked chain.
- **Drift zones** (orange on the minimap) are scored separately with 1-3 stars.

## Audio

All sound is synthesised live from the simulation, so every engine/turbo/tune sounds different without sample packs:
- **Engine:** firing-frequency harmonics from cylinder count and rpm, shaped by load. The limiter's fuel cut bounces audibly, and flat-4 Subarus get the unequal-length boxer burble.
- **Turbo:** whine with boost, plus a blow-off on lift.
- **Tyres:** squeal driven by slip angle and wheelspin past the tyre's peak.
- Gravel/grass rumble, wind, and wall thuds.

Volume is in Settings.

## Assetto Corsa compatibility

| Direction | What works |
|---|---|
| **AC car → Drift Lab** | *Garage → Import AC car* reads an **unpacked** car data folder: car.ini, engine.ini + power.lut, turbo, drivetrain.ini, suspensions.ini, tyres.ini, brakes.ini. It produces a chassis, engine and gearbox part plus a build, so imported cars can be engine-swapped like any other. Packed cars (`data.acd`) need unpacking first; Content Manager can do that. |
| **AC car models → Drift Lab** | Pick **3D model (from AC)** and a skin in the Garage. Drift Lab reads the car's `.kn5` (and skin textures) straight from your Assetto Corsa install at runtime: nothing is copied or redistributed. The real wheels spin and steer on the simulated wheels, the real steering wheel turns, wheelbase/track/wheel size come from the model, and the cockpit view sits behind the model's steering wheel. AC is auto-detected in Steam libraries, or set its folder in Settings. Some special shaders (CSP car paint, normal maps) render simplified. |
| **AC track → Drift Lab** | *Load track model* opens AC `.kn5` tracks directly (or .glb / .gltf / .fbx). AC physics naming is honoured: `1ROAD`, `1KERB`, `1GRASS`, `1SAND`, `*WALL*`… become surfaces and walls, and the car spawns at `AC_START_0`. |
| **Drift Lab track → AC** | *Track editor → Export for Assetto Corsa* writes a ready-to-drive track folder: `<track>.kn5` written directly (road, kerbs, run-off and walls with AC physics names, textures, `AC_START/PIT/HOTLAP/TIME` markers), `ui/ui_track.json`, outline/preview, `map.png` + `data/map.ini`, and `data/surfaces.ini`. With the HD texture pack (the `hd_textures` folder shipped next to `DriftLab.exe`) the road, grass, dirt, gravel and walls use photo-scanned 4K/2K textures with normal maps (CC0, from [Poly Haven](https://polyhaven.com) - see `hd_textures/CREDITS.txt`), and `extension/ext_config.ini` turns on Custom Shaders Patch 3D grass and rain puddles. Copy it into `assettocorsa/content/tracks` and pick it in Content Manager. No AI line yet, so use practice, free roam or a server without AI. |

Cars without an AC model use a simple body sized from the chassis dimensions.

## How the physics works

Fixed 240 Hz simulation with 4 drivetrain sub-steps (960 Hz):
- **Tyres:** normalised Magic-Formula-style curve with a defined peak slip angle and ratio. Grip falls off past the peak, more for lateral sliding than for wheelspin. Combined slip means wheelspin eats lateral grip. Grip changes with load, compound, width and surface (asphalt, kerb, grass, gravel, dirt, sand). Wheel spin uses an implicit solve, so it's stable at high torque.
- **Suspension:** raycast spring/damper per corner (separate bump/rebound), anti-roll bars, bump stops, load transfer.
- **Drivetrain:** engine torque curve with engine braking, turbo spool/lag and boost, a fuel-cut limiter, then the clutch (you can clutch-kick), the gearbox with shift time, then the diffs. Diffs are open, clutch LSD (preload + power/coast ramps) or welded. AWD has a torque split and centre coupling.
- **Steering:** per-wheel Ackermann blend, steering lock per car/angle kit, and a wheel-to-road ratio from your wheel's rotation.

It aims for *believable and consistent* rather than AC-level detail. There's no tyre temperature or wear yet, aero is drag only, and suspension geometry (camber, caster) isn't modelled.

## Roadmap / known limits

- Force feedback tuning against more wheels (tested in code and under Wine; needs real-wheel feel tuning).
- VR: in-headset menus (for now menus are on the monitor).
- Ghosts/replays, tandem AI, online leaderboards.
- Tyre temperature, camber/caster effects.

## Development

```
cd drift-sim
godot --headless --path . --import
godot --headless --path . -s tests/run_tests.gd        # 142 checks: tyres, builds, tracks, tracer, scoring, AC import/export, physics, game smoke
xvfb-run godot --path . --rendering-method gl_compatibility -s tests/screenshots.gd -- /tmp/shots
```

Layout: `scripts/vehicle` (tyres, car builder, vehicle physics), `scripts/scoring`, `scripts/track` (spec, builder, rasteriser, external loader), `scripts/mapgen` (image tracer), `scripts/ai` (Claude client + Garage/Track agents), `scripts/ac` (AC import/export), `scripts/ui`, `scripts/game`; data in `data/parts`, `data/cars`, `tracks`. User data goes to `%APPDATA%\DriftLab` on Windows.
