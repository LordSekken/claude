import { type Vec, dist, sub, norm, segmentsCross, add, scale } from "./geom";
import { edgePoints, inRect, type FocusArea, type Network, type NodeControl } from "./network";

/*
 * Microscopic traffic simulation.
 *
 * Every road becomes one or two directed links, each with N lanes. Cars follow
 * the car ahead using the Intelligent Driver Model (IDM), pick routes by
 * current travel time (so they re-route around jams and around your fixes),
 * and negotiate intersections with simple, readable rules:
 *
 *  - Every movement through a node is a short line from the entry to the exit
 *    point. Two movements conflict if those lines cross or share an exit road.
 *  - A car may enter only if no conflicting movement is in progress, the exit
 *    road has room (don't block the box), and no higher-priority car is about
 *    to make a conflicting move.
 *  - Priority: circulating roundabout traffic > bigger road > smaller road;
 *    left turns yield to everything else at the same level.
 *  - Signals gate approaches by phase; all-way stops force a full stop and go
 *    first-stopped, first-served.
 */

const CAR_LEN = 4.5;
const LANE_W = 3.3;
const IDM = { a: 1.4, b: 2.0, s0: 2.0, T: 1.3 };
const BOX_R = 7; // meters — half the size of the conflict box drawn through each node
const DT = 0.25;

export interface Car {
  id: number;
  link: Link;
  lane: number;
  s: number;
  v: number;
  route: Link[];
  ri: number;
  spawnT: number;
  ffTime: number;
  linkEnterT: number;
  committed: boolean;
  stoppedAt: number; // sim time car came to rest at a stop sign (or -1)
  waitT: number; // time spent waiting at the current stop line
  stillT: number; // continuous time stopped, to detect gridlock
}

export interface Link {
  idx: number;
  edgeId: string;
  reverse: boolean;
  from: SNode;
  to: SNode;
  pts: Vec[];
  cum: number[];
  length: number;
  lanes: number;
  oneway: boolean;
  speed: number;
  cls: number;
  roundabout: boolean;
  outs: Link[];
  lane: Car[][]; // per lane, front (largest s) first
  speedEma: number;
  endDir: Vec;
  startDir: Vec;
}

interface Reservation {
  car: Car;
  inL: Link;
  outL: Link;
  until: number;
}

export interface SNode {
  id: string;
  p: Vec;
  control: NodeControl;
  cycle: number;
  ins: Link[];
  outs: Link[];
  group: Map<Link, 0 | 1>;
  twoPhase: boolean;
  active: Reservation[];
  delayEma: number;
  flow: number; // vehicles / second, exponentially decayed
  crossings: number;
}

export interface FocusStats {
  id: string;
  name: string;
  delay: number; // avg control delay per vehicle, s
  flow: number; // veh/h through intersections in the area
  cars: number;
  speed: number; // avg mph of cars inside
  los: string;
}

export interface SimStats {
  t: number;
  active: number;
  completed: number;
  gaveUp: number;
  pendingSpawns: number;
  avgDelay: number; // s per completed trip vs free-flow
  avgSpeedMph: number;
  throughputPerHour: number;
  /** Flow-weighted avg delay per vehicle per intersection, s. */
  nodeDelay: number;
  los: string;
  focus: FocusStats[];
}

export interface DemandSettings {
  vehPerHour: number;
  /** 0..1 — share of trips that start/end inside the area rather than at its edges. */
  localShare: number;
}

export const LOS_COLOR: Record<string, string> = {
  A: "#1b9e4b", B: "#6cc04a", C: "#e3c419", D: "#f08c1a", E: "#e0482a", F: "#a3172b",
};

export function losFor(delay: number, signalized = true): string {
  const th = signalized ? [10, 20, 35, 55, 80] : [10, 15, 25, 35, 50];
  const letters = ["A", "B", "C", "D", "E"];
  for (let i = 0; i < th.length; i++) if (delay <= th[i]) return letters[i];
  return "F";
}

const angleBetween = (a: Vec, b: Vec) => Math.atan2(a[0] * b[1] - a[1] * b[0], a[0] * b[0] + a[1] * b[1]);

export class Simulation {
  links: Link[] = [];
  nodes = new Map<string, SNode>();
  cars: Car[] = [];
  t = 0;
  completed = 0;
  gaveUp = 0;
  private nextCarId = 1;
  private spawnAcc = 0;
  private pending: { origin: SNode; dest: SNode }[] = [];
  private delayEma = 0;
  private recentDone: number[] = [];
  private gateways: { node: SNode; w: number }[] = [];
  private interior: SNode[] = [];
  private gwTotal = 0;
  private routeCache = new Map<string, { t: number; prev: Map<Link, Link | null>; dist: Map<Link, number> }>();
  private focus: FocusArea[];
  demand: DemandSettings;

  constructor(net: Network, demand: DemandSettings) {
    this.demand = demand;
    this.focus = net.focusAreas;
    for (const n of Object.values(net.nodes)) {
      this.nodes.set(n.id, {
        id: n.id, p: n.p, control: n.control, cycle: n.cycle ?? 70, ins: [], outs: [],
        group: new Map(), twoPhase: false, active: [], delayEma: 0, flow: 0, crossings: 0,
      });
    }
    const mk = (edgeId: string, reverse: boolean, pts: Vec[], e: Network["edges"][string]) => {
      const from = this.nodes.get(reverse ? e.b : e.a)!, to = this.nodes.get(reverse ? e.a : e.b)!;
      const cum = [0];
      for (let i = 1; i < pts.length; i++) cum.push(cum[i - 1] + dist(pts[i - 1], pts[i]));
      const length = Math.max(cum[cum.length - 1], 1);
      const link: Link = {
        idx: this.links.length, edgeId, reverse, from, to, pts, cum, length,
        lanes: Math.max(1, e.lanes), oneway: e.oneway, speed: e.speed, cls: e.cls,
        roundabout: !!e.roundabout, outs: [], lane: Array.from({ length: Math.max(1, e.lanes) }, () => []),
        speedEma: e.speed,
        endDir: norm(sub(pts[pts.length - 1], pts[Math.max(0, pts.length - 2)])),
        startDir: norm(sub(pts[Math.min(1, pts.length - 1)], pts[0])),
      };
      this.links.push(link);
      from.outs.push(link);
      to.ins.push(link);
    };
    for (const e of Object.values(net.edges)) {
      if (!this.nodes.has(e.a) || !this.nodes.has(e.b) || e.a === e.b) continue;
      const pts = edgePoints(net, e);
      mk(e.id, false, pts, e);
      if (!e.oneway) mk(e.id, true, [...pts].reverse(), e);
    }
    for (const l of this.links) {
      const outs = l.to.outs.filter((o) => o.edgeId !== l.edgeId);
      l.outs = outs.length > 0 ? outs : l.to.outs; // allow U-turns only at dead ends
    }
    for (const n of this.nodes.values()) this.setupSignalGroups(n);
    this.setupDemand();
  }

  private setupSignalGroups(n: SNode) {
    if (n.ins.length === 0) return;
    const axis = n.ins[0].endDir;
    for (const l of n.ins) {
      const a = Math.abs(angleBetween(axis, l.endDir));
      const g: 0 | 1 = a < Math.PI / 4 || a > (3 * Math.PI) / 4 ? 0 : 1;
      n.group.set(l, g);
      if (g === 1) n.twoPhase = true;
    }
  }

  private setupDemand() {
    this.gateways = [];
    this.interior = [];
    for (const n of this.nodes.values()) {
      const edges = new Set([...n.ins, ...n.outs].map((l) => l.edgeId));
      if (n.outs.length === 0 && n.ins.length === 0) continue;
      if (edges.size === 1) {
        const cls = Math.max(...[...n.ins, ...n.outs].map((l) => l.cls));
        this.gateways.push({ node: n, w: cls * cls * Math.max(...[...n.ins, ...n.outs].map((l) => l.lanes)) });
      } else if (n.outs.length > 0 && n.ins.length > 0) {
        this.interior.push(n);
      }
    }
    this.gwTotal = this.gateways.reduce((s, g) => s + g.w, 0);
  }

  get gatewayCount() {
    return this.gateways.length;
  }

  private pickNode(wantOut: boolean): SNode | null {
    const useLocal = this.gateways.length === 0 || Math.random() < this.demand.localShare;
    if (useLocal) {
      if (this.interior.length === 0) return null;
      return this.interior[Math.floor(Math.random() * this.interior.length)];
    }
    for (let tries = 0; tries < 10; tries++) {
      let r = Math.random() * this.gwTotal;
      for (const g of this.gateways) {
        r -= g.w;
        if (r <= 0) {
          if (wantOut ? g.node.outs.length > 0 : g.node.ins.length > 0) return g.node;
          break;
        }
      }
    }
    return null;
  }

  // ---------------- routing ----------------

  private linkCost(l: Link): number {
    return l.length / Math.max(1.5, l.speedEma) + Math.min(l.to.delayEma, 120);
  }

  private tree(origin: SNode) {
    const cached = this.routeCache.get(origin.id);
    if (cached && this.t - cached.t < 60) return cached;
    const distM = new Map<Link, number>();
    const prev = new Map<Link, Link | null>();
    // Binary heap keyed on cost.
    const heap: [number, Link][] = [];
    const push = (c: number, l: Link) => {
      heap.push([c, l]);
      let i = heap.length - 1;
      while (i > 0) {
        const p = (i - 1) >> 1;
        if (heap[p][0] <= heap[i][0]) break;
        [heap[p], heap[i]] = [heap[i], heap[p]];
        i = p;
      }
    };
    const pop = () => {
      const top = heap[0];
      const last = heap.pop()!;
      if (heap.length > 0) {
        heap[0] = last;
        let i = 0;
        for (;;) {
          const l = 2 * i + 1, r = l + 1;
          let m = i;
          if (l < heap.length && heap[l][0] < heap[m][0]) m = l;
          if (r < heap.length && heap[r][0] < heap[m][0]) m = r;
          if (m === i) break;
          [heap[m], heap[i]] = [heap[i], heap[m]];
          i = m;
        }
      }
      return top;
    };
    for (const l of origin.outs) {
      const c = this.linkCost(l);
      distM.set(l, c);
      prev.set(l, null);
      push(c, l);
    }
    while (heap.length) {
      const [c, l] = pop();
      if (c > (distM.get(l) ?? Infinity)) continue;
      for (const o of l.outs) {
        const turn = Math.abs(angleBetween(l.endDir, o.startDir)) > 0.5 ? 4 : 0;
        const nc = c + this.linkCost(o) + turn;
        if (nc < (distM.get(o) ?? Infinity)) {
          distM.set(o, nc);
          prev.set(o, l);
          push(nc, o);
        }
      }
    }
    const entry = { t: this.t, prev, dist: distM };
    this.routeCache.set(origin.id, entry);
    return entry;
  }

  private route(origin: SNode, dest: SNode): Link[] | null {
    const tr = this.tree(origin);
    let best: Link | null = null, bc = Infinity;
    for (const l of dest.ins) {
      const c = tr.dist.get(l);
      if (c !== undefined && c < bc) { bc = c; best = l; }
    }
    if (!best) return null;
    const path: Link[] = [];
    for (let l: Link | null = best; l; l = tr.prev.get(l) ?? null) path.push(l);
    return path.reverse();
  }

  // ---------------- spawning ----------------

  private trySpawn(origin: SNode, dest: SNode): "ok" | "blocked" | "noroute" {
    const route = this.route(origin, dest);
    if (!route || route.length === 0) return "noroute";
    const first = route[0];
    let lane = -1, room = -Infinity;
    for (let i = 0; i < first.lanes; i++) {
      const q = first.lane[i];
      const tail = q.length ? q[q.length - 1].s : Infinity;
      if (tail > room) { room = tail; lane = i; }
    }
    if (room < CAR_LEN + IDM.s0 + 1) return "blocked";
    const ffTime = route.reduce((s, l) => s + l.length / l.speed, 0);
    const car: Car = {
      id: this.nextCarId++, link: first, lane, s: 0, v: Math.min(first.speed, 8), route, ri: 0,
      spawnT: this.t, ffTime, linkEnterT: this.t, committed: false, stoppedAt: -1, waitT: 0, stillT: 0,
    };
    first.lane[lane].push(car);
    this.cars.push(car);
    return "ok";
  }

  private spawn(dt: number) {
    this.spawnAcc += (this.demand.vehPerHour / 3600) * dt;
    while (this.spawnAcc >= 1) {
      this.spawnAcc -= 1;
      const o = this.pickNode(true), d = this.pickNode(false);
      if (o && d && o !== d && this.pending.length < 2000) this.pending.push({ origin: o, dest: d });
    }
    const keep: typeof this.pending = [];
    for (const p of this.pending) {
      const r = this.trySpawn(p.origin, p.dest);
      if (r === "blocked") keep.push(p);
    }
    this.pending = keep;
  }

  // ---------------- intersections ----------------

  private turnLine(n: SNode, inL: Link, outL: Link): [Vec, Vec] {
    return [sub(n.p, scale(inL.endDir, BOX_R)), add(n.p, scale(outL.startDir, BOX_R))];
  }

  private conflicts(n: SNode, i1: Link, o1: Link, i2: Link, o2: Link): boolean {
    if (i1 === i2) return false;
    if (o1 === o2) return true;
    const [a, b] = this.turnLine(n, i1, o1);
    const [c, d] = this.turnLine(n, i2, o2);
    return segmentsCross(a, b, c, d);
  }

  private rank(inL: Link, outL: Link): number {
    if (inL.roundabout) return 1000;
    const turn = angleBetween(inL.endDir, outL.startDir);
    return inL.cls * 10 - (turn > 0.5 ? 5 : 0);
  }

  private signalGreen(n: SNode, inL: Link): boolean {
    if (n.control !== "signal" || !n.twoPhase) return true;
    const half = n.cycle / 2;
    const phaseT = this.t % n.cycle;
    const g = n.group.get(inL) ?? 0;
    const start = g === 0 ? 0 : half;
    const local = phaseT - start;
    return local >= 0 && local < half - 4; // 4 s yellow + all-red
  }

  private bestLane(l: Link): { lane: number; tail: number; car: Car | null } {
    let lane = 0, tail = -Infinity, car: Car | null = null;
    for (let i = 0; i < l.lanes; i++) {
      const q = l.lane[i];
      const last = q.length ? q[q.length - 1] : null;
      const t = last ? last.s : Infinity;
      if (t > tail) { tail = t; lane = i; car = last; }
    }
    return { lane, tail, car };
  }

  private hasRoom(l: Link) {
    return this.bestLane(l).tail > CAR_LEN + IDM.s0;
  }

  /** Can `car` (front of its lane on inL) proceed through the node onto outL right now? */
  private permit(car: Car, inL: Link, outL: Link): boolean {
    const n = inL.to;
    if (!this.hasRoom(outL)) return false;
    if (!this.signalGreen(n, inL)) return false;

    for (const r of n.active) {
      if (r.car !== car && this.conflicts(n, inL, outL, r.inL, r.outL)) return false;
    }

    if (n.control === "stop") {
      if (car.stoppedAt < 0) return false;
      for (const l of n.ins) {
        if (l === inL) continue;
        for (const q of l.lane) {
          const c = q[0];
          if (!c || c.stoppedAt < 0 || c.stoppedAt >= car.stoppedAt) continue;
          const o = c.route[c.ri + 1];
          if (o && this.conflicts(n, inL, outL, l, o)) return false;
        }
      }
      return true;
    }

    // Gridlock breaker: someone who has waited a long time goes once the box is clear.
    if (car.waitT > 40) return true;

    const myRank = n.control === "yield" ? 0 : this.rank(inL, outL);
    for (const l of n.ins) {
      if (l === inL) continue;
      for (const q of l.lane) {
        const c = q[0];
        if (!c || c.committed) continue; // committed cars are already in n.active
        const o = c.route[c.ri + 1];
        if (!o) continue;
        const theirRank = n.control === "yield" ? 0 : this.rank(l, o);
        if (theirRank <= myRank) continue;
        const rem = l.length - c.s;
        const horizon = Math.max(12, c.v * 4);
        if (rem > horizon) continue;
        if (!this.signalGreen(n, l) || !this.hasRoom(o)) continue;
        if (c.v < 0.5 && rem > 6) continue; // queued further back, not about to go
        if (this.conflicts(n, inL, outL, l, o)) return false;
      }
    }
    return true;
  }

  // ---------------- main loop ----------------

  step(steps = 1) {
    for (let i = 0; i < steps; i++) this.tick(DT);
  }

  private idm(v: number, v0: number, gap: number, dv: number): number {
    const sStar = IDM.s0 + Math.max(0, v * IDM.T + (v * dv) / (2 * Math.sqrt(IDM.a * IDM.b)));
    const g = Math.max(gap, 0.1);
    return IDM.a * (1 - Math.pow(v / Math.max(v0, 0.1), 4) - (sStar / g) ** 2);
  }

  private tick(dt: number) {
    this.t += dt;
    this.spawn(dt);

    for (const n of this.nodes.values()) {
      if (n.active.length) n.active = n.active.filter((r) => r.until > this.t);
    }

    // 1) Longitudinal update (IDM) + intersection decisions for lane leaders.
    for (const l of this.links) {
      let speedSum = 0, count = 0;
      for (const q of l.lane) {
        for (let k = 0; k < q.length; k++) {
          const car = q[k];
          let gap = Infinity, dv = 0;
          if (k > 0) {
            const lead = q[k - 1];
            gap = lead.s - car.s - CAR_LEN;
            dv = car.v - lead.v;
          } else {
            const rem = l.length - car.s;
            const next = car.route[car.ri + 1];
            if (next) {
              let clear = car.committed;
              if (!clear && rem < 200) {
                // Come to a stop at all-way stops.
                if (l.to.control === "stop" && car.stoppedAt < 0 && rem < 4 && car.v < 0.6) car.stoppedAt = this.t;
                clear = this.permit(car, l, next);
                const brake = (car.v * car.v) / (2 * IDM.b) + 3;
                if (clear && rem < Math.max(brake, 8)) {
                  car.committed = true;
                  l.to.active.push({
                    car, inL: l, outL: next,
                    until: this.t + (rem + BOX_R * 2 + CAR_LEN) / Math.max(car.v, 3) + 0.6,
                  });
                }
              }
              if (clear) {
                const { tail, car: nextCar } = this.bestLane(next);
                gap = tail === Infinity ? Infinity : rem + tail - CAR_LEN;
                dv = nextCar ? car.v - nextCar.v : 0;
                // Slow down ahead of a slower road (e.g. entering a roundabout).
                if (gap === Infinity && next.speed < car.v) { gap = rem + 25; dv = car.v - next.speed; }
              } else {
                gap = rem - 1;
                dv = car.v;
                if (rem < 15 && car.v < 1) car.waitT += dt;
              }
            }
          }
          const acc = this.idm(car.v, l.speed, gap, dv);
          car.v = Math.max(0, car.v + Math.max(acc, -9) * dt);
          car.s += car.v * dt;
          car.stillT = car.v < 0.1 ? car.stillT + dt : 0;
          speedSum += car.v;
          count++;
        }
      }
      const observed = count ? speedSum / count : l.speed;
      l.speedEma += (observed - l.speedEma) * Math.min(1, dt / 20);
    }

    // 2) Transfers to the next link / trip completion.
    const done = new Set<Car>();
    for (const l of this.links) {
      for (let li = 0; li < l.lanes; li++) {
        const q = l.lane[li];
        while (q.length && q[0].s >= l.length) {
          const car = q[0];
          const next = car.route[car.ri + 1];
          const n = l.to;
          if (!next) {
            q.shift();
            done.add(car);
            this.completed++;
            const delay = Math.max(0, this.t - car.spawnT - car.ffTime);
            this.delayEma += (delay - this.delayEma) * 0.03;
            this.recentDone.push(this.t);
            continue;
          }
          if (!car.committed) {
            car.s = l.length - 0.05;
            car.v = 0;
            break;
          }
          q.shift();
          // Control delay at this node = time on the link beyond free-flow.
          const d = Math.max(0, this.t - car.linkEnterT - l.length / l.speed);
          n.delayEma += (d - n.delayEma) * 0.05;
          n.flow += 1;
          n.crossings++;
          const { lane } = this.bestLane(next);
          car.s -= l.length;
          car.link = next;
          car.lane = lane;
          car.ri++;
          car.committed = false;
          car.stoppedAt = -1;
          car.waitT = 0;
          car.linkEnterT = this.t;
          next.lane[lane].push(car);
          next.lane[lane].sort((a, b) => b.s - a.s);
        }
      }
    }

    // 3) Remove cars hopelessly stuck (gridlock), like SUMO's "teleport".
    for (const car of this.cars) {
      if (!done.has(car) && car.stillT > 240) {
        const q = car.link.lane[car.lane];
        const i = q.indexOf(car);
        if (i >= 0) q.splice(i, 1);
        done.add(car);
        this.gaveUp++;
      }
    }
    if (done.size) this.cars = this.cars.filter((c) => !done.has(c));

    const decay = Math.exp(-dt / 300);
    for (const n of this.nodes.values()) n.flow *= decay;
  }

  /** World position + heading of a car (for drawing). */
  carPose(car: Car): { p: Vec; dir: Vec } {
    const l = car.link;
    const s = Math.min(Math.max(car.s, 0), l.length);
    let lo = 0, hi = l.cum.length - 1;
    while (hi - lo > 1) {
      const m = (lo + hi) >> 1;
      if (l.cum[m] <= s) lo = m; else hi = m;
    }
    const segLen = l.cum[hi] - l.cum[lo] || 1;
    const t = (s - l.cum[lo]) / segLen;
    const a = l.pts[lo], b = l.pts[hi];
    const dir = norm(sub(b, a));
    const right: Vec = [dir[1], -dir[0]];
    const off = l.oneway ? (car.lane - (l.lanes - 1) / 2) * LANE_W : (car.lane + 0.5) * LANE_W;
    return { p: [a[0] + (b[0] - a[0]) * t + right[0] * off, a[1] + (b[1] - a[1]) * t + right[1] * off], dir };
  }

  stats(): SimStats {
    this.recentDone = this.recentDone.filter((t) => this.t - t < 300);
    const window = Math.min(300, Math.max(this.t, 1));
    let vs = 0;
    for (const c of this.cars) vs += c.v;
    // n.flow decays with a 300 s time constant; correct for warm-up so early numbers aren't low.
    const flowToHourly = 12 / (1 - Math.exp(-Math.max(this.t, 1) / 300));
    let netDelay = 0, netFlow = 0;
    for (const n of this.nodes.values()) {
      if (n.ins.length < 2) continue;
      netDelay += n.delayEma * n.flow;
      netFlow += n.flow;
    }
    const focus: FocusStats[] = this.focus.map((f) => {
      let wDelay = 0, wFlow = 0;
      for (const n of this.nodes.values()) {
        if (n.ins.length < 2 || !inRect(n.p, f)) continue;
        wDelay += n.delayEma * n.flow;
        wFlow += n.flow;
      }
      let cars = 0, sp = 0;
      for (const c of this.cars) {
        if (inRect(this.carPose(c).p, f)) { cars++; sp += c.v; }
      }
      const delay = wFlow > 0 ? wDelay / wFlow : 0;
      return {
        id: f.id, name: f.name, delay, cars,
        flow: wFlow * flowToHourly,
        speed: cars ? sp / cars / 0.44704 : 0, los: losFor(delay),
      };
    });
    return {
      t: this.t,
      active: this.cars.length,
      completed: this.completed,
      gaveUp: this.gaveUp,
      pendingSpawns: this.pending.length,
      avgDelay: this.delayEma,
      avgSpeedMph: this.cars.length ? vs / this.cars.length / 0.44704 : 0,
      throughputPerHour: (this.recentDone.length / window) * 3600,
      nodeDelay: netFlow > 0 ? netDelay / netFlow : 0,
      los: losFor(netFlow > 0 ? netDelay / netFlow : 0),
      focus,
    };
  }
}

export const SIM_DT = DT;
