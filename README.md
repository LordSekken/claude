# Intersection Editor

> This repo also contains **[Drift Lab](drift-sim/README.md)**, a Windows drift simulator with modular real-world cars, photo-to-track generation and Forza-style scoring, in `drift-sim/`.

A lightweight browser tool for looking at a real intersection and redesigning
it — e.g. dropping a two-lane spiral roundabout onto Shaw & Pioneer — without
needing full civil-engineering CAD software.

## What it does

1. **Find the intersection** — search an address (via OpenStreetMap
   Nominatim), or just click the spot on the map.
2. **Pull the real geometry** — fetches the actual road centerlines near that
   point from OpenStreetMap (via the Overpass API) so you're designing over
   the real intersection, not a blank canvas.
3. **Design a roundabout** — generates a 1- or 2-lane roundabout (island,
   circulatory lanes, lane divider, approach/exit lanes, splitter islands) as
   live, editable map layers. Tune lane count, island radius, lane width, and
   "spiral bulge" (how much the outer curb widens near each approach — the
   characteristic look of a multi-lane spiral roundabout). Drag any
   line/vertex to fine-tune it by hand.
4. **Export** — download the design as GeoJSON for use elsewhere.

## Running it

```bash
npm install
npm run dev
```

## Notes

- Map tiles, geocoding, and road-fetching all call public OpenStreetMap
  services directly from the browser — no backend or API key required.
- The roundabout geometry is a practical visual/editable approximation
  (concentric rings + a Gaussian widening near each leg), not a
  survey-grade spiral curve — good for exploring a design, not for stamped
  engineering drawings.
