import * as turf from "@turf/turf";
import type { Feature, FeatureCollection, LineString, Polygon } from "geojson";

export interface RoundaboutLeg {
  /** Compass bearing (deg, 0 = north) the approach road points outward along. */
  angleDeg: number;
  /** How far out to draw the approach stub, in meters. */
  length: number;
  /** Label, e.g. the street name — carried through for export only. */
  name?: string;
}

export interface RoundaboutParams {
  center: [number, number]; // [lon, lat]
  islandRadius: number; // meters, radius of the (non-drivable) central island
  laneWidth: number; // meters per lane
  laneCount: 1 | 2;
  /** Extra outer-curb widening near each leg (meters) — gives the circulatory
   *  roadway its "spiral" character instead of a perfect circle. 0 = plain circle. */
  spiralBulge: number;
  legs: RoundaboutLeg[];
}

export const DEFAULT_LEGS: RoundaboutLeg[] = [
  { angleDeg: 0, length: 40, name: "North leg" },
  { angleDeg: 90, length: 40, name: "East leg" },
  { angleDeg: 180, length: 40, name: "South leg" },
  { angleDeg: 270, length: 40, name: "West leg" },
];

export function defaultRoundaboutParams(
  center: [number, number],
  laneCount: 1 | 2 = 2,
  legs: RoundaboutLeg[] = DEFAULT_LEGS
): RoundaboutParams {
  return {
    center,
    islandRadius: laneCount === 2 ? 9 : 6,
    laneWidth: 3.5,
    laneCount,
    spiralBulge: laneCount === 2 ? 3 : 1.5,
    legs,
  };
}

function toLngLat(center: [number, number], xMeters: number, yMeters: number): [number, number] {
  const bearing = (Math.atan2(xMeters, yMeters) * 180) / Math.PI;
  const distanceKm = Math.hypot(xMeters, yMeters) / 1000;
  if (distanceKm === 0) return center;
  const pt = turf.destination(center, distanceKm, bearing, { units: "kilometers" });
  return pt.geometry.coordinates as [number, number];
}

function angleDiffDeg(a: number, b: number): number {
  let d = ((a - b + 180) % 360 + 360) % 360 - 180;
  return d;
}

/** Outer-curb radius at a given bearing: base radius plus a bump near each leg,
 *  producing the asymmetric "spiral" widening seen at real multi-lane roundabouts. */
function outerRadiusAt(params: RoundaboutParams, angleDeg: number): number {
  const base = params.islandRadius + params.laneWidth * params.laneCount;
  if (params.spiralBulge <= 0) return base;
  const sigma = 22; // degrees, width of the bulge around each leg
  let bulge = 0;
  for (const leg of params.legs) {
    const d = angleDiffDeg(angleDeg, leg.angleDeg);
    bulge += params.spiralBulge * Math.exp(-(d * d) / (2 * sigma * sigma));
  }
  return base + bulge;
}

function ring(
  center: [number, number],
  radiusFn: (angleDeg: number) => number,
  steps = 96
): [number, number][] {
  const pts: [number, number][] = [];
  for (let i = 0; i <= steps; i++) {
    const angle = (360 * i) / steps;
    const r = radiusFn(angle);
    const rad = (angle * Math.PI) / 180;
    pts.push(toLngLat(center, r * Math.sin(rad), r * Math.cos(rad)));
  }
  return pts;
}

export interface RoundaboutDesign {
  island: Feature<Polygon>;
  circulatoryOuter: Feature<LineString>;
  laneDividers: Feature<LineString>[];
  approachLanes: Feature<LineString>[];
  splitterIslands: Feature<Polygon>[];
}

export function generateRoundabout(params: RoundaboutParams): RoundaboutDesign {
  const { center, islandRadius, laneWidth, laneCount } = params;

  const island = turf.polygon([ring(center, () => islandRadius)], {
    kind: "island",
  }) as Feature<Polygon>;

  const outerCoords = ring(center, (a) => outerRadiusAt(params, a));
  const circulatoryOuter = turf.lineString(outerCoords, {
    kind: "circulatory-outer",
  }) as Feature<LineString>;

  const laneDividers: Feature<LineString>[] = [];
  if (laneCount === 2) {
    const dividerCoords = ring(center, (a) => {
      const outer = outerRadiusAt(params, a);
      return (islandRadius + outer) / 2;
    });
    laneDividers.push(
      turf.lineString(dividerCoords, { kind: "lane-divider" }) as Feature<LineString>
    );
  }

  const approachLanes: Feature<LineString>[] = [];
  const splitterIslands: Feature<Polygon>[] = [];

  for (const leg of params.legs) {
    const mergeRadius = outerRadiusAt(params, leg.angleDeg);
    const halfWidth = (laneWidth * laneCount) / 2;
    const rad = (leg.angleDeg * Math.PI) / 180;
    const perpRad = rad + Math.PI / 2;

    // Entry lanes: from the stub's outer edge, converging onto the circulatory ring.
    for (let i = 0; i < laneCount; i++) {
      const offset = -halfWidth + laneWidth * (i + 0.5);
      const stubX = leg.length * Math.sin(rad) + offset * Math.sin(perpRad);
      const stubY = leg.length * Math.cos(rad) + offset * Math.cos(perpRad);
      const mergeX = mergeRadius * Math.sin(rad) + offset * 0.35 * Math.sin(perpRad);
      const mergeY = mergeRadius * Math.cos(rad) + offset * 0.35 * Math.cos(perpRad);
      approachLanes.push(
        turf.lineString(
          [toLngLat(center, stubX, stubY), toLngLat(center, mergeX, mergeY)],
          { kind: "entry-lane", leg: leg.name ?? `${leg.angleDeg}deg`, laneIndex: i }
        ) as Feature<LineString>
      );
    }

    // A small triangular splitter island between the entry and exit lanes,
    // separating inbound/outbound traffic on the approach.
    const tipX = (islandRadius + laneWidth * 0.3) * Math.sin(rad);
    const tipY = (islandRadius + laneWidth * 0.3) * Math.cos(rad);
    const baseOffset = halfWidth * 0.15;
    const base1X = leg.length * 0.5 * Math.sin(rad) + baseOffset * Math.sin(perpRad);
    const base1Y = leg.length * 0.5 * Math.cos(rad) + baseOffset * Math.cos(perpRad);
    const base2X = leg.length * 0.5 * Math.sin(rad) - baseOffset * Math.sin(perpRad);
    const base2Y = leg.length * 0.5 * Math.cos(rad) - baseOffset * Math.cos(perpRad);
    splitterIslands.push(
      turf.polygon(
        [[
          toLngLat(center, tipX, tipY),
          toLngLat(center, base1X, base1Y),
          toLngLat(center, base2X, base2Y),
          toLngLat(center, tipX, tipY),
        ]],
        { kind: "splitter-island", leg: leg.name ?? `${leg.angleDeg}deg` }
      ) as Feature<Polygon>
    );
  }

  return { island, circulatoryOuter, laneDividers, approachLanes, splitterIslands };
}

export function roundaboutToFeatureCollection(design: RoundaboutDesign): FeatureCollection {
  const features: Feature<Polygon | LineString>[] = [
    design.island,
    design.circulatoryOuter,
    ...design.laneDividers,
    ...design.approachLanes,
    ...design.splitterIslands,
  ];
  return turf.featureCollection(features) as FeatureCollection;
}
