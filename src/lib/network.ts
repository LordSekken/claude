import {
  type Vec,
  add,
  sub,
  scale,
  dist,
  norm,
  sampleCubic,
  nearestOnPolyline,
  polylineLength,
  segmentTransform,
} from "./geom";

/**
 * Intersection control:
 *  - auto:   priority by road class (side streets yield to the bigger road, lefts yield to oncoming)
 *  - signal: fixed-time two-phase traffic light
 *  - stop:   all-way stop
 *  - yield:  every approach yields to anyone already approaching (like an unsigned junction)
 */
export type NodeControl = "auto" | "signal" | "stop" | "yield";

export interface RoadNode {
  id: string;
  p: Vec;
  control: NodeControl;
  /** Signal cycle length in seconds (only used when control = signal). */
  cycle?: number;
}

export interface RoadEdge {
  id: string;
  a: string;
  b: string;
  /** Lanes per direction (for a one-way: total lanes, all heading a→b). */
  lanes: number;
  oneway: boolean;
  /** Speed limit, m/s. */
  speed: number;
  /** Road class rank — higher = more important (motorway 7 … living street 1). */
  cls: number;
  name?: string;
  roundabout?: boolean;
  /** If both set, the road follows a cubic Bézier between its end nodes. */
  c1?: Vec;
  c2?: Vec;
  /** Otherwise: interior polyline points (from OSM geometry). */
  shape?: Vec[];
}

export interface FocusArea {
  id: string;
  name: string;
  min: Vec;
  max: Vec;
}

export interface Network {
  /** [lon, lat] that local (0,0) maps to. */
  origin: [number, number];
  nodes: Record<string, RoadNode>;
  edges: Record<string, RoadEdge>;
  focusAreas: FocusArea[];
  nextId: number;
}

export const MPH = 0.44704;

export function emptyNetwork(origin: [number, number]): Network {
  return { origin, nodes: {}, edges: {}, focusAreas: [], nextId: 1 };
}

export function cloneNetwork(n: Network): Network {
  return structuredClone(n);
}

export function newId(net: Network, prefix: string): string {
  return `${prefix}${net.nextId++}`;
}

export function edgePoints(net: Network, e: RoadEdge): Vec[] {
  const a = net.nodes[e.a].p, b = net.nodes[e.b].p;
  if (e.c1 && e.c2) return sampleCubic(a, e.c1, e.c2, b, Math.max(8, Math.min(32, Math.round(dist(a, b) / 6))));
  return [a, ...(e.shape ?? []), b];
}

export function edgeLength(net: Network, e: RoadEdge): number {
  return polylineLength(edgePoints(net, e));
}

export function incidentEdges(net: Network, nodeId: string): RoadEdge[] {
  return Object.values(net.edges).filter((e) => e.a === nodeId || e.b === nodeId);
}

// ---------------------------------------------------------------------------
// Editing operations. All mutate `net` in place; callers clone first for undo.
// ---------------------------------------------------------------------------

/** Move a node, carrying the shape / curve handles of every attached road along with it. */
export function moveNode(net: Network, nodeId: string, to: Vec) {
  const node = net.nodes[nodeId];
  for (const e of incidentEdges(net, nodeId)) {
    const a0 = net.nodes[e.a].p, b0 = net.nodes[e.b].p;
    const a1 = e.a === nodeId ? to : a0;
    const b1 = e.b === nodeId ? to : b0;
    const f = segmentTransform(a0, b0, a1, b1);
    if (e.shape) e.shape = e.shape.map(f);
    if (e.c1) e.c1 = f(e.c1);
    if (e.c2) e.c2 = f(e.c2);
  }
  node.p = to;
}

/** Turn a road into an adjustable Bézier curve, fitted to its current shape. */
export function makeCurve(net: Network, e: RoadEdge) {
  if (e.c1 && e.c2) return;
  const pts = edgePoints(net, e);
  const a = pts[0], b = pts[pts.length - 1];
  const L = polylineLength(pts);
  // Tangents from the first / last ~20% of the existing geometry.
  const probe = (fromStart: boolean): Vec => {
    const seq = fromStart ? pts : [...pts].reverse();
    let acc = 0;
    for (let i = 1; i < seq.length; i++) {
      acc += dist(seq[i - 1], seq[i]);
      if (acc >= L * 0.2) return norm(sub(seq[i], seq[0]));
    }
    return norm(sub(seq[seq.length - 1], seq[0]));
  };
  const chord = dist(a, b);
  e.c1 = add(a, scale(probe(true), chord / 3));
  e.c2 = add(b, scale(probe(false), chord / 3));
  delete e.shape;
}

export function straighten(e: RoadEdge) {
  delete e.c1;
  delete e.c2;
  delete e.shape;
}

/** Convert a curve to plain interior points (needed before splitting/trimming). */
function flattenToShape(net: Network, e: RoadEdge) {
  if (e.c1 && e.c2) {
    const pts = edgePoints(net, e);
    e.shape = pts.slice(1, -1);
    delete e.c1;
    delete e.c2;
  }
}

/** Split a road at the point nearest `at`, inserting a new node. Returns the new node id. */
export function splitEdge(net: Network, edgeId: string, at: Vec): string {
  const e = net.edges[edgeId];
  flattenToShape(net, e);
  const pts = edgePoints(net, e);
  const near = nearestOnPolyline(pts, at);
  const nodeId = newId(net, "n");
  net.nodes[nodeId] = { id: nodeId, p: near.point, control: "auto" };
  const first = pts.slice(1, near.seg + 1);
  const second = pts.slice(near.seg + 1, -1);
  const e2: RoadEdge = { ...structuredClone(e), id: newId(net, "e"), a: nodeId, b: e.b, shape: second };
  e.b = nodeId;
  e.shape = first;
  net.edges[e2.id] = e2;
  return nodeId;
}

export function deleteEdge(net: Network, edgeId: string) {
  const e = net.edges[edgeId];
  if (!e) return;
  delete net.edges[edgeId];
  for (const n of [e.a, e.b]) {
    if (incidentEdges(net, n).length === 0) delete net.nodes[n];
  }
}

export function deleteNode(net: Network, nodeId: string) {
  for (const e of incidentEdges(net, nodeId)) delete net.edges[e.id];
  delete net.nodes[nodeId];
}

/** Remove a pass-through node (exactly two roads) and join them into one. */
export function joinAtNode(net: Network, nodeId: string): boolean {
  const inc = incidentEdges(net, nodeId);
  if (inc.length !== 2) return false;
  const [e1, e2] = inc;
  flattenToShape(net, e1);
  flattenToShape(net, e2);
  // Orient e1 to end at node, e2 to start at node.
  const p1 = e1.b === nodeId ? edgePoints(net, e1) : [...edgePoints(net, e1)].reverse();
  const p2 = e2.a === nodeId ? edgePoints(net, e2) : [...edgePoints(net, e2)].reverse();
  const start = e1.b === nodeId ? e1.a : e1.b;
  const end = e2.a === nodeId ? e2.b : e2.a;
  if (start === end) return false;
  if ((e1.oneway || e2.oneway) && !(e1.b === nodeId && e2.a === nodeId)) return false; // would break one-way flow
  e1.a = start;
  e1.b = end;
  e1.shape = [...p1.slice(1, -1), p1[p1.length - 1], ...p2.slice(1, -1)];
  e1.lanes = Math.max(e1.lanes, e2.lanes);
  delete net.edges[e2.id];
  delete net.nodes[nodeId];
  return true;
}

/** Merge node `from` into node `into` (connects two road ends dropped on each other). */
export function mergeNodes(net: Network, from: string, into: string) {
  if (from === into) return;
  for (const e of incidentEdges(net, from)) {
    if (e.a === from) e.a = into;
    if (e.b === from) e.b = into;
    if (e.a === e.b) delete net.edges[e.id];
  }
  delete net.nodes[from];
}

export function reverseEdge(e: RoadEdge) {
  [e.a, e.b] = [e.b, e.a];
  if (e.shape) e.shape = [...e.shape].reverse();
  if (e.c1 && e.c2) [e.c1, e.c2] = [e.c2, e.c1];
}

/** Find a node within `tol` meters of p. */
export function nodeNear(net: Network, p: Vec, tol: number): RoadNode | null {
  let best: RoadNode | null = null, bd = tol;
  for (const n of Object.values(net.nodes)) {
    const d = dist(n.p, p);
    if (d < bd) { bd = d; best = n; }
  }
  return best;
}

export function edgeNear(net: Network, p: Vec, tol: number): RoadEdge | null {
  let best: RoadEdge | null = null, bd = tol;
  for (const e of Object.values(net.edges)) {
    const d = nearestOnPolyline(edgePoints(net, e), p).d;
    if (d < bd) { bd = d; best = e; }
  }
  return best;
}

/** Resolve a click into a node: snap to an existing node, split a nearby road, or make a new node. */
export function nodeAt(net: Network, p: Vec, tol: number): string {
  const n = nodeNear(net, p, tol);
  if (n) return n.id;
  const e = edgeNear(net, p, tol * 0.7);
  if (e) return splitEdge(net, e.id, p);
  const id = newId(net, "n");
  net.nodes[id] = { id, p, control: "auto" };
  return id;
}

export function addRoad(net: Network, a: string, b: string, template?: Partial<RoadEdge>): string {
  const id = newId(net, "e");
  net.edges[id] = { id, a, b, lanes: 1, oneway: false, speed: 30 * MPH, cls: 2, ...template };
  return id;
}

// ---------------------------------------------------------------------------
// Auto roundabout
// ---------------------------------------------------------------------------

/** Trim a polyline that starts at the center so it begins where it first leaves the circle of radius r. */
function trimFromCenter(pts: Vec[], center: Vec, r: number): Vec[] | null {
  for (let i = 1; i < pts.length; i++) {
    if (dist(pts[i], center) >= r) {
      // Bisect along segment i-1 → i for the exact crossing.
      let lo = 0, hi = 1;
      const a = pts[i - 1], b = pts[i];
      for (let k = 0; k < 30; k++) {
        const m = (lo + hi) / 2;
        const q: Vec = [a[0] + (b[0] - a[0]) * m, a[1] + (b[1] - a[1]) * m];
        if (dist(q, center) < r) lo = m; else hi = m;
      }
      const cross: Vec = [a[0] + (b[0] - a[0]) * hi, a[1] + (b[1] - a[1]) * hi];
      // Need some road left beyond the ring for an approach.
      if (polylineLength([cross, ...pts.slice(i)]) < 5) return null;
      return [cross, ...pts.slice(i)];
    }
  }
  return null;
}

export interface RoundaboutOptions {
  radius: number; // meters, to the circulating lane centerline
  lanes: 1 | 2;
  speed: number; // m/s
}

/**
 * Replace an intersection node with a roundabout: a ring of one-way, counter-
 * clockwise (right-hand traffic) curved roads, with each approach trimmed back to
 * meet the ring. Returns the ring node ids, or an error message.
 */
export function makeRoundabout(net: Network, nodeId: string, opt: RoundaboutOptions): string[] | string {
  const center = net.nodes[nodeId].p;
  const inc = incidentEdges(net, nodeId);
  if (inc.length < 2) return "Pick an intersection with at least 2 roads.";
  const { radius: r } = opt;

  const legs: { edge: RoadEdge; angle: number; pts: Vec[]; outward: boolean }[] = [];
  for (const e of inc) {
    flattenToShape(net, e);
    const outward = e.a === nodeId; // edge points away from the center
    const raw = edgePoints(net, e);
    const fromCenter = outward ? raw : [...raw].reverse();
    const trimmed = trimFromCenter(fromCenter, center, r);
    if (!trimmed) return `A road at this intersection is shorter than the roundabout radius (${r} m).`;
    const d = sub(trimmed[0], center);
    legs.push({ edge: e, angle: Math.atan2(d[1], d[0]), pts: trimmed, outward });
  }
  legs.sort((x, y) => x.angle - y.angle);

  const maxCls = Math.max(...inc.map((e) => e.cls));
  const ring: { id: string; angle: number }[] = [];

  for (const leg of legs) {
    const id = newId(net, "n");
    const p = add(center, [Math.cos(leg.angle) * r, Math.sin(leg.angle) * r]);
    net.nodes[id] = { id, p, control: "auto" };
    ring.push({ id, angle: leg.angle });
    const e = leg.edge;
    const pts = [p, ...leg.pts.slice(1)];
    if (leg.outward) {
      e.a = id;
      e.shape = pts.slice(1, -1);
    } else {
      e.b = id;
      e.shape = [...pts].reverse().slice(1, -1);
    }
  }

  // Fill big gaps so no ring arc exceeds ~100° (keeps Bézier arcs accurate).
  const filled: { id: string; angle: number }[] = [];
  for (let i = 0; i < ring.length; i++) {
    const cur = ring[i];
    const next = ring[(i + 1) % ring.length];
    let gap = next.angle - cur.angle;
    if (gap <= 0) gap += Math.PI * 2;
    filled.push(cur);
    const extra = Math.ceil(gap / (Math.PI * 100 / 180)) - 1;
    for (let k = 1; k <= extra; k++) {
      const ang = cur.angle + (gap * k) / (extra + 1);
      const id = newId(net, "n");
      net.nodes[id] = { id, p: add(center, [Math.cos(ang) * r, Math.sin(ang) * r]), control: "auto" };
      filled.push({ id, angle: ang });
    }
  }

  for (let i = 0; i < filled.length; i++) {
    const cur = filled[i];
    const next = filled[(i + 1) % filled.length];
    let gap = next.angle - cur.angle;
    if (gap <= 0) gap += Math.PI * 2;
    const k = (4 / 3) * Math.tan(gap / 4) * r;
    const t1: Vec = [-Math.sin(cur.angle), Math.cos(cur.angle)];
    const t2: Vec = [-Math.sin(next.angle), Math.cos(next.angle)];
    const p1 = net.nodes[cur.id].p, p2 = net.nodes[next.id].p;
    const id = newId(net, "e");
    net.edges[id] = {
      id,
      a: cur.id,
      b: next.id,
      lanes: opt.lanes,
      oneway: true,
      speed: opt.speed,
      cls: maxCls,
      roundabout: true,
      name: "Roundabout",
      c1: add(p1, scale(t1, k)),
      c2: sub(p2, scale(t2, k)),
    };
  }

  delete net.nodes[nodeId];
  return filled.map((f) => f.id);
}

/** Recommended ring radius for a node, based on its biggest approach. */
export function suggestRoundaboutRadius(net: Network, nodeId: string): number {
  const inc = incidentEdges(net, nodeId);
  const lanes = Math.max(1, ...inc.map((e) => e.lanes));
  const shortest = Math.min(...inc.map((e) => edgeLength(net, e)));
  const ideal = lanes >= 2 ? 22 : 15;
  return Math.max(8, Math.min(ideal, Math.floor(shortest - 6)));
}

export const inRect = (p: Vec, a: FocusArea) =>
  p[0] >= a.min[0] && p[0] <= a.max[0] && p[1] >= a.min[1] && p[1] <= a.max[1];
