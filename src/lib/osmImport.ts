import { project, type Vec } from "./geom";
import { MPH, emptyNetwork, newId, type Network, type NodeControl, type RoadEdge } from "./network";

const OVERPASS_ENDPOINT = "https://overpass-api.de/api/interpreter";

const CLASS_RANK: Record<string, number> = {
  motorway: 7, trunk: 6, primary: 5, secondary: 4, tertiary: 3,
  unclassified: 2, residential: 2, living_street: 1,
};

const DEFAULT_SPEED_MPH: Record<string, number> = {
  motorway: 65, trunk: 55, primary: 45, secondary: 40, tertiary: 35,
  unclassified: 30, residential: 25, living_street: 15,
};

const DEFAULT_LANES: Record<string, number> = { motorway: 3, trunk: 2, primary: 2, secondary: 2 };

export interface BBox {
  south: number;
  west: number;
  north: number;
  east: number;
}

export type RoadDetail = "all" | "collectors" | "major";

const DETAIL_FILTER: Record<RoadDetail, string> = {
  all: "^(motorway|trunk|primary|secondary|tertiary|unclassified|residential|living_street)(_link)?$",
  collectors: "^(motorway|trunk|primary|secondary|tertiary)(_link)?$",
  major: "^(motorway|trunk|primary|secondary)(_link)?$",
};

function parseSpeed(tag: string | undefined, hw: string): number {
  if (tag) {
    const m = tag.match(/(\d+(?:\.\d+)?)\s*(mph)?/i);
    if (m) return m[2] ? parseFloat(m[1]) * MPH : parseFloat(m[1]) / 3.6;
  }
  return (DEFAULT_SPEED_MPH[hw] ?? 30) * MPH;
}

interface OsmWay {
  id: number;
  nodes: number[];
  geometry: { lat: number; lon: number }[];
  tags: Record<string, string>;
}

/**
 * Load a drivable road network for a bounding box from OpenStreetMap and turn
 * it into an editable graph: one node per intersection / dead end, one edge per
 * road segment between them, with lanes, speed limits, one-way flags, existing
 * roundabouts and traffic signals carried over from the OSM tags.
 */
export async function importOsm(bbox: BBox, detail: RoadDetail): Promise<Network> {
  const b = `${bbox.south},${bbox.west},${bbox.north},${bbox.east}`;
  const query = `
    [out:json][timeout:60];
    way["highway"~"${DETAIL_FILTER[detail]}"](${b})->.w;
    .w out body geom;
    node(w.w)["highway"~"^(traffic_signals|stop)$"];
    out body;
  `;
  const res = await fetch(OVERPASS_ENDPOINT, {
    method: "POST",
    body: "data=" + encodeURIComponent(query),
    headers: { "Content-Type": "application/x-www-form-urlencoded" },
  });
  if (!res.ok) throw new Error(`Overpass query failed: ${res.status}`);
  const data = await res.json();

  const ways: OsmWay[] = data.elements.filter((el: any) => el.type === "way" && el.geometry);
  const controlNodes = new Map<number, { lat: number; lon: number; tags: Record<string, string> }>();
  for (const el of data.elements) if (el.type === "node") controlNodes.set(el.id, el);

  const origin: [number, number] = [(bbox.west + bbox.east) / 2, (bbox.south + bbox.north) / 2];
  const net = emptyNetwork(origin);
  const margin = 0.1;
  const latPad = (bbox.north - bbox.south) * margin, lonPad = (bbox.east - bbox.west) * margin;
  const inside = (g: { lat: number; lon: number }) =>
    g.lat >= bbox.south - latPad && g.lat <= bbox.north + latPad &&
    g.lon >= bbox.west - lonPad && g.lon <= bbox.east + lonPad;

  // Clip each way to the (padded) box, splitting it into runs where it leaves and re-enters.
  type Run = { way: OsmWay; ids: number[]; pts: Vec[] };
  const runs: Run[] = [];
  for (const w of ways) {
    let cur: Run | null = null;
    w.nodes.forEach((nid, i) => {
      const g = w.geometry[i];
      if (g && inside(g)) {
        if (!cur) { cur = { way: w, ids: [], pts: [] }; runs.push(cur); }
        cur.ids.push(nid);
        cur.pts.push(project(origin, [g.lon, g.lat]));
      } else {
        cur = null;
      }
    });
  }

  // A node becomes a graph node if it's a run endpoint or shared by multiple runs.
  const useCount = new Map<number, number>();
  for (const r of runs) for (const id of r.ids) useCount.set(id, (useCount.get(id) ?? 0) + 1);

  const graphNode = new Map<number, string>();
  const nodeFor = (osmId: number, p: Vec) => {
    let id = graphNode.get(osmId);
    if (!id) {
      id = newId(net, "n");
      net.nodes[id] = { id, p, control: "auto" };
      graphNode.set(osmId, id);
    }
    return id;
  };

  for (const r of runs) {
    if (r.ids.length < 2) continue;
    const t = r.way.tags;
    const hw = (t.highway ?? "").replace(/_link$/, "");
    const isLink = t.highway?.endsWith("_link");
    const roundabout = t.junction === "roundabout" || t.junction === "circular";
    const oneway = roundabout || t.oneway === "yes" || t.oneway === "1" || t.oneway === "-1" ||
      (hw === "motorway" && t.oneway !== "no");
    const totalLanes = parseInt(t.lanes ?? "", 10);
    const lanes = Number.isFinite(totalLanes) && totalLanes > 0
      ? (oneway ? totalLanes : Math.max(1, Math.ceil(totalLanes / 2)))
      : (isLink ? 1 : DEFAULT_LANES[hw] ?? 1);
    const template: Omit<RoadEdge, "id" | "a" | "b"> = {
      lanes: Math.min(lanes, 5),
      oneway,
      speed: parseSpeed(t.maxspeed, hw),
      cls: (CLASS_RANK[hw] ?? 2) - (isLink ? 0.5 : 0),
      name: t.name ?? t.ref,
      ...(roundabout ? { roundabout: true } : {}),
    };

    let startIdx = 0;
    for (let i = 1; i < r.ids.length; i++) {
      const isEnd = i === r.ids.length - 1;
      if (!isEnd && (useCount.get(r.ids[i]) ?? 0) < 2) continue;
      const a = nodeFor(r.ids[startIdx], r.pts[startIdx]);
      const bId = nodeFor(r.ids[i], r.pts[i]);
      if (a !== bId) {
        const id = newId(net, "e");
        const edge: RoadEdge = { id, a, b: bId, ...structuredClone(template), shape: r.pts.slice(startIdx + 1, i) };
        if (t.oneway === "-1") {
          edge.a = bId;
          edge.b = a;
          edge.shape = [...(edge.shape ?? [])].reverse();
        }
        net.edges[id] = edge;
      }
      startIdx = i;
    }
  }

  // Traffic signals / all-way stops: apply to the tagged node, or the nearest real intersection.
  const degree = new Map<string, number>();
  for (const e of Object.values(net.edges)) {
    degree.set(e.a, (degree.get(e.a) ?? 0) + 1);
    degree.set(e.b, (degree.get(e.b) ?? 0) + 1);
  }
  const intersections = Object.values(net.nodes).filter((n) => (degree.get(n.id) ?? 0) >= 3);
  for (const [osmId, cn] of controlNodes) {
    const kind: NodeControl | null = cn.tags.highway === "traffic_signals" ? "signal"
      : cn.tags.highway === "stop" && cn.tags.stop === "all" ? "stop" : null;
    if (!kind) continue;
    const p = project(origin, [cn.lon, cn.lat]);
    let target = graphNode.get(osmId);
    if (!target || (degree.get(target) ?? 0) < 3) {
      let best = 30;
      for (const n of intersections) {
        const d = Math.hypot(n.p[0] - p[0], n.p[1] - p[1]);
        if (d < best) { best = d; target = n.id; }
      }
      if (best === 30) target = undefined;
    }
    if (target && net.nodes[target]) net.nodes[target].control = kind;
  }

  return net;
}

/** Rough area of a bbox in km², to warn before huge downloads. */
export function bboxAreaKm2(b: BBox): number {
  const h = (b.north - b.south) * 110.54;
  const w = (b.east - b.west) * 111.32 * Math.cos((((b.north + b.south) / 2) * Math.PI) / 180);
  return Math.abs(h * w);
}
