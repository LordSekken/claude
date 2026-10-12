/** Local planar coordinates in meters: x = east, y = north, relative to a network origin. */
export type Vec = [number, number];

export const add = (a: Vec, b: Vec): Vec => [a[0] + b[0], a[1] + b[1]];
export const sub = (a: Vec, b: Vec): Vec => [a[0] - b[0], a[1] - b[1]];
export const scale = (a: Vec, k: number): Vec => [a[0] * k, a[1] * k];
export const len = (a: Vec) => Math.hypot(a[0], a[1]);
export const dist = (a: Vec, b: Vec) => Math.hypot(a[0] - b[0], a[1] - b[1]);
export const lerp = (a: Vec, b: Vec, t: number): Vec => [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t];
export const norm = (a: Vec): Vec => {
  const l = len(a);
  return l === 0 ? [0, 0] : [a[0] / l, a[1] / l];
};

// Equirectangular projection — accurate to well under 1% across a city-sized area.
const M_PER_DEG_LAT = 110540;
const M_PER_DEG_LON_EQ = 111320;

export function project(origin: [number, number], lonLat: [number, number]): Vec {
  const k = Math.cos((origin[1] * Math.PI) / 180);
  return [(lonLat[0] - origin[0]) * M_PER_DEG_LON_EQ * k, (lonLat[1] - origin[1]) * M_PER_DEG_LAT];
}

export function unproject(origin: [number, number], p: Vec): [number, number] {
  const k = Math.cos((origin[1] * Math.PI) / 180);
  return [origin[0] + p[0] / (M_PER_DEG_LON_EQ * k), origin[1] + p[1] / M_PER_DEG_LAT];
}

export function cubic(p0: Vec, c1: Vec, c2: Vec, p1: Vec, t: number): Vec {
  const u = 1 - t;
  const a = u * u * u, b = 3 * u * u * t, c = 3 * u * t * t, d = t * t * t;
  return [a * p0[0] + b * c1[0] + c * c2[0] + d * p1[0], a * p0[1] + b * c1[1] + c * c2[1] + d * p1[1]];
}

export function sampleCubic(p0: Vec, c1: Vec, c2: Vec, p1: Vec, steps = 20): Vec[] {
  const pts: Vec[] = [];
  for (let i = 0; i <= steps; i++) pts.push(cubic(p0, c1, c2, p1, i / steps));
  return pts;
}

export function polylineLength(pts: Vec[]): number {
  let l = 0;
  for (let i = 1; i < pts.length; i++) l += dist(pts[i - 1], pts[i]);
  return l;
}

/** The point a given fraction (0..1) of the way along a polyline. */
export function pointAlong(pts: Vec[], f: number): Vec {
  const target = polylineLength(pts) * f;
  let acc = 0;
  for (let i = 1; i < pts.length; i++) {
    const d = dist(pts[i - 1], pts[i]);
    if (acc + d >= target) return lerp(pts[i - 1], pts[i], d === 0 ? 0 : (target - acc) / d);
    acc += d;
  }
  return pts[pts.length - 1];
}

/** Nearest point on a polyline: returns the point, its distance, and the segment index it falls on. */
export function nearestOnPolyline(pts: Vec[], p: Vec): { point: Vec; d: number; seg: number; t: number } {
  let best = { point: pts[0], d: Infinity, seg: 0, t: 0 };
  for (let i = 1; i < pts.length; i++) {
    const a = pts[i - 1], b = pts[i];
    const ab = sub(b, a);
    const l2 = ab[0] * ab[0] + ab[1] * ab[1];
    const t = l2 === 0 ? 0 : Math.max(0, Math.min(1, ((p[0] - a[0]) * ab[0] + (p[1] - a[1]) * ab[1]) / l2));
    const q = lerp(a, b, t);
    const d = dist(p, q);
    if (d < best.d) best = { point: q, d, seg: i - 1, t };
  }
  return best;
}

/** Proper intersection test for segments ab and cd (touching endpoints don't count). */
export function segmentsCross(a: Vec, b: Vec, c: Vec, d: Vec): boolean {
  const o = (p: Vec, q: Vec, r: Vec) => (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0]);
  const d1 = o(c, d, a), d2 = o(c, d, b), d3 = o(a, b, c), d4 = o(a, b, d);
  return d1 * d2 < 0 && d3 * d4 < 0;
}

/**
 * Similarity transform (rotate + uniform scale + translate) that maps segment
 * (a0→b0) onto (a1→b1). Used to carry a road's interior shape along when one of
 * its end nodes is dragged, so the curve bends naturally instead of kinking.
 */
export function segmentTransform(a0: Vec, b0: Vec, a1: Vec, b1: Vec): (p: Vec) => Vec {
  const v0 = sub(b0, a0), v1 = sub(b1, a1);
  const l0 = len(v0);
  if (l0 < 1e-6) return (p) => add(sub(p, a0), a1);
  const k = len(v1) / l0;
  const ang = Math.atan2(v1[1], v1[0]) - Math.atan2(v0[1], v0[0]);
  const cs = Math.cos(ang) * k, sn = Math.sin(ang) * k;
  return (p) => {
    const q = sub(p, a0);
    return [a1[0] + q[0] * cs - q[1] * sn, a1[1] + q[0] * sn + q[1] * cs];
  };
}
