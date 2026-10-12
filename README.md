# Traffic Sandbox

A SimCity-style traffic playground built on your real city's roads. Load the
actual street network from OpenStreetMap, reshape it like a game editor, fill
it with simulated cars, and get a traffic-engineering report card for the
spots you care about.

## What it does

**Build**
- **Load your city** — pan/zoom to an area and pull its real roads (lanes,
  speed limits, one-ways, existing roundabouts and traffic signals) from
  OpenStreetMap. Choose *All streets* for a neighborhood, or *Major roads only*
  for a whole city (up to ~400 km²).
- **Edit roads** — tap a road to select it, then:
  - drag the white dots to move its **start/end points** (drop one on another
    intersection to connect them),
  - **Make curve** and drag the blue handles to bend it (Bézier curve),
  - change lanes, speed limit, road class, one-way / flip direction,
  - split it in the middle, or delete it.
- **Edit intersections** — tap one to switch its control (priority, signal
  with adjustable cycle, all-way stop, uncontrolled), **Build roundabout**
  (auto-generates a 1- or 2-lane ring and trims every approach to meet it),
  join a pass-through node, or delete it.
- **Draw new roads** — tap point to point; taps snap to intersections and
  split roads they land on.
- **Focus zones** — draw rectangles around the areas you want to study.
- Undo/redo (Ctrl+Z / Ctrl+Shift+Z), auto-save in the browser, export/import
  project files.

**Simulate**
- Agent-based cars (Intelligent Driver Model car-following) enter and leave at
  the edges of the loaded area plus some local trips, and choose the fastest
  route given current congestion — so they discover your new roads.
- Intersections use signals, stops, roundabout yield-to-circulating, and
  side-street / left-turn yielding with a conflict check, so you'll see real
  queues and even gridlock.
- Roads color by speed, intersections get **Level of Service** grades (A–F),
  and each focus zone gets its own report card. Save a "before", make a fix,
  and watch the % change.

**Roundabout sketch** — the original detailed drawing tool (curbs, splitter
islands, spiral widening) for presenting a single intersection design;
exports GeoJSON.

## Running it

```bash
npm install
npm run dev
```

## Notes / limits

- Map tiles, geocoding and road loading use public OpenStreetMap services
  directly from the browser — no backend or API key. The Overpass API can be
  slow or busy; retry if a load fails.
- The simulation is built for "what if" exploration, not for stamped
  engineering studies: no lane changing, simplified two-phase signals, and
  demand is synthetic (not real counts). For permit-grade work, export your
  ideas and model them in SUMO, Vissim, or Synchro.
