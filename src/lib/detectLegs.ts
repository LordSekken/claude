import * as turf from "@turf/turf";
import type { RoadWay } from "./overpass";
import type { RoundaboutLeg } from "./roundabout";

/**
 * Infer the roundabout's approach legs from real road geometry: for each way
 * passing near the intersection, look outward from the closest point to find
 * which direction(s) it heads (a through road contributes two legs, a
 * dead-end / T stub contributes one), then de-duplicate directions that are
 * close in bearing (multiple OSM ways for the same physical road).
 */
export function detectLegs(
  center: [number, number],
  roads: RoadWay[],
  legLength = 40,
  probeMeters = 25
): RoundaboutLeg[] {
  const centerPt = turf.point(center);
  const candidates: { angleDeg: number; name?: string }[] = [];

  for (const road of roads) {
    if (road.coords.length < 2) continue;
    const line = turf.lineString(road.coords);
    const nearest = turf.nearestPointOnLine(line, centerPt);
    const idx = nearest.properties.index ?? 0;

    const probeOutward = (fromIdx: number, dir: 1 | -1) => {
      let i = fromIdx;
      let acc = 0;
      let prev = road.coords[Math.min(Math.max(i, 0), road.coords.length - 1)];
      while (i + dir >= 0 && i + dir < road.coords.length) {
        i += dir;
        const cur = road.coords[i];
        acc += turf.distance(turf.point(prev), turf.point(cur), { units: "kilometers" }) * 1000;
        prev = cur;
        if (acc >= probeMeters) break;
      }
      const end = road.coords[i];
      const bearing = turf.bearing(centerPt, turf.point(end));
      candidates.push({ angleDeg: (bearing + 360) % 360, name: road.tags.name });
    };

    probeOutward(idx, -1);
    if (idx + 1 < road.coords.length) probeOutward(idx, 1);
  }

  const legs: RoundaboutLeg[] = [];
  const usedAngles: number[] = [];
  const angleDist = (a: number, b: number) => Math.abs(((a - b + 540) % 360) - 180);

  for (const c of candidates) {
    if (usedAngles.some((a) => angleDist(a, c.angleDeg) < 20)) continue;
    usedAngles.push(c.angleDeg);
    legs.push({ angleDeg: c.angleDeg, length: legLength, name: c.name });
  }

  return legs;
}
