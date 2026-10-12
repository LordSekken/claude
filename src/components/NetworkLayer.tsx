import { useEffect, useRef } from "react";
import L from "leaflet";
import { type Vec, project, unproject } from "../lib/geom";
import {
  cloneNetwork,
  edgePoints,
  incidentEdges,
  mergeNodes,
  moveNode,
  nodeNear,
  type Network,
  type RoadEdge,
} from "../lib/network";
import { LOS_COLOR, losFor, type Simulation } from "../lib/sim";

export type Selection = { kind: "edge"; id: string } | { kind: "node"; id: string } | null;

interface Props {
  map: L.Map;
  net: Network;
  selection: Selection;
  onCommit: (net: Network) => void;
  simRef: React.RefObject<Simulation | null>;
  showTraffic: boolean;
  draftFrom: Vec | null;
  focusCorner: Vec | null;
}

const CONTROL_COLOR: Record<string, string> = {
  auto: "#ffffff", signal: "#2b8a3e", stop: "#c92a2a", yield: "#f59f00",
};

function edgeColor(e: RoadEdge): string {
  if (e.roundabout) return "#7048e8";
  if (e.cls >= 5) return "#4a4f57";
  if (e.cls >= 3) return "#6b717b";
  return "#8f959e";
}

function speedColor(ratio: number): string {
  if (ratio > 0.8) return "#2f9e44";
  if (ratio > 0.6) return "#94d82d";
  if (ratio > 0.4) return "#fcc419";
  if (ratio > 0.2) return "#f76707";
  return "#c92a2a";
}

export default function NetworkLayer(props: Props) {
  const { map, net, selection } = props;
  const propsRef = useRef(props);
  propsRef.current = props;

  const rendererRef = useRef<L.Canvas | null>(null);
  const edgeLayerRef = useRef<L.LayerGroup | null>(null);
  const handleLayerRef = useRef<L.LayerGroup | null>(null);
  const extraLayerRef = useRef<L.LayerGroup | null>(null);
  const polysRef = useRef(new Map<string, L.Polyline>());
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const draftRef = useRef<Network | null>(null);

  const toLL = (p: Vec): L.LatLngTuple => {
    const [lon, lat] = unproject(propsRef.current.net.origin, p);
    return [lat, lon];
  };

  const metersPerPixel = () => {
    const c = map.getCenter();
    return (40075016.686 * Math.cos((c.lat * Math.PI) / 180)) / Math.pow(2, map.getZoom() + 8);
  };

  const edgeStyle = (e: RoadEdge, selected: boolean): L.PathOptions => {
    const totalLanes = e.oneway ? e.lanes : e.lanes * 2;
    const w = Math.max(2, Math.min(40, (totalLanes * 3.3) / metersPerPixel()));
    return {
      color: selected ? "#1a4fd6" : edgeColor(e),
      weight: selected ? w + 3 : w,
      opacity: 0.9,
      lineCap: "round",
      lineJoin: "round",
      dashArray: e.oneway && !e.roundabout && w < 6 ? "6 4" : undefined,
    };
  };

  // ---------- one-time setup ----------
  useEffect(() => {
    rendererRef.current = L.canvas({ padding: 0.5 });
    edgeLayerRef.current = L.layerGroup().addTo(map);
    extraLayerRef.current = L.layerGroup().addTo(map);
    handleLayerRef.current = L.layerGroup().addTo(map);

    const canvas = document.createElement("canvas");
    canvas.className = "car-canvas";
    map.getContainer().appendChild(canvas);
    canvasRef.current = canvas;

    const restyle = () => {
      const { net: n, selection: sel } = propsRef.current;
      for (const [id, poly] of polysRef.current) {
        const e = n.edges[id];
        if (e) poly.setStyle(edgeStyle(e, sel?.kind === "edge" && sel.id === id));
      }
    };
    map.on("zoomend", restyle);

    return () => {
      map.off("zoomend", restyle);
      edgeLayerRef.current?.remove();
      extraLayerRef.current?.remove();
      handleLayerRef.current?.remove();
      canvas.remove();
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [map]);

  // ---------- draw roads ----------
  useEffect(() => {
    const layer = edgeLayerRef.current!;
    layer.clearLayers();
    polysRef.current.clear();
    for (const e of Object.values(net.edges)) {
      if (!net.nodes[e.a] || !net.nodes[e.b]) continue;
      const poly = L.polyline(edgePoints(net, e).map(toLL), {
        ...edgeStyle(e, selection?.kind === "edge" && selection.id === e.id),
        renderer: rendererRef.current!,
        interactive: false,
      });
      poly.addTo(layer);
      polysRef.current.set(e.id, poly);
    }
    // Focus areas
    const extra = extraLayerRef.current!;
    extra.clearLayers();
    for (const f of net.focusAreas) {
      L.rectangle([toLL(f.min), toLL(f.max)], {
        color: "#d6336c", weight: 2, dashArray: "8 6", fill: true, fillOpacity: 0.05, interactive: false,
      })
        .bindTooltip(f.name, { permanent: true, direction: "top", className: "focus-label" })
        .addTo(extra);
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [net, selection]);

  // ---------- drag handles ----------
  useEffect(() => {
    const layer = handleLayerRef.current!;
    layer.clearLayers();
    if (!selection) return;

    const refresh = (n: Network, edgeIds: string[]) => {
      for (const id of edgeIds) {
        const e = n.edges[id];
        const poly = polysRef.current.get(id);
        if (e && poly) poly.setLatLngs(edgePoints(n, e).map(toLL));
      }
    };

    const makeHandle = (p: Vec, cls: string, onDrag: (n: Network, to: Vec) => string[], onDrop?: (n: Network, to: Vec) => void) => {
      const m = L.marker(toLL(p), {
        draggable: true,
        icon: L.divIcon({ className: `handle ${cls}`, iconSize: [18, 18] }),
        zIndexOffset: 1000,
      });
      m.on("dragstart", () => {
        draftRef.current = cloneNetwork(propsRef.current.net);
      });
      m.on("drag", () => {
        const n = draftRef.current!;
        const ll = m.getLatLng();
        const to = project(n.origin, [ll.lng, ll.lat]);
        refresh(n, onDrag(n, to));
        guides.forEach((g) => g());
      });
      m.on("dragend", () => {
        const n = draftRef.current!;
        const ll = m.getLatLng();
        onDrop?.(n, project(n.origin, [ll.lng, ll.lat]));
        draftRef.current = null;
        propsRef.current.onCommit(n);
      });
      m.addTo(layer);
      return m;
    };

    const guides: (() => void)[] = [];

    const nodeHandle = (nodeId: string) => {
      const node = net.nodes[nodeId];
      if (!node) return;
      makeHandle(
        node.p,
        "handle-node",
        (n, to) => {
          moveNode(n, nodeId, to);
          return incidentEdges(n, nodeId).map((e) => e.id);
        },
        (n, to) => {
          // Dropping a road end onto another node connects them.
          const tol = metersPerPixel() * 14;
          const others = { ...n, nodes: { ...n.nodes } };
          delete others.nodes[nodeId];
          const target = nodeNear(others, to, tol);
          if (target) {
            moveNode(n, nodeId, target.p);
            mergeNodes(n, nodeId, target.id);
          }
        }
      );
    };

    if (selection.kind === "node") nodeHandle(selection.id);
    if (selection.kind === "edge") {
      const e = net.edges[selection.id];
      if (!e) return;
      nodeHandle(e.a);
      nodeHandle(e.b);
      if (e.c1 && e.c2) {
        for (const which of ["c1", "c2"] as const) {
          const anchor = which === "c1" ? e.a : e.b;
          const line = L.polyline([toLL(net.nodes[anchor].p), toLL(e[which]!)], {
            color: "#1a4fd6", weight: 1.5, dashArray: "4 4", interactive: false,
          }).addTo(layer);
          const h = makeHandle(e[which]!, "handle-ctrl", (n, to) => {
            n.edges[e.id][which] = to;
            return [e.id];
          });
          guides.push(() => {
            const n = draftRef.current;
            if (!n) return;
            line.setLatLngs([toLL(n.nodes[n.edges[e.id][anchor === e.a ? "a" : "b"]].p), h.getLatLng()]);
          });
        }
      }
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [net, selection]);

  // ---------- draft markers (road drawing / focus corner) ----------
  useEffect(() => {
    const layer = handleLayerRef.current!;
    const marks: L.Layer[] = [];
    if (props.draftFrom) {
      marks.push(L.circleMarker(toLL(props.draftFrom), { radius: 7, color: "#1a4fd6", fillColor: "#fff", fillOpacity: 1, weight: 3 }));
    }
    if (props.focusCorner) {
      marks.push(L.circleMarker(toLL(props.focusCorner), { radius: 7, color: "#d6336c", fillColor: "#fff", fillOpacity: 1, weight: 3 }));
    }
    marks.forEach((m) => m.addTo(layer));
    return () => marks.forEach((m) => m.remove());
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [props.draftFrom, props.focusCorner, net, selection]);

  // ---------- canvas: nodes, cars, intersection grades ----------
  useEffect(() => {
    let raf = 0;
    let lastColor = 0;
    const draw = (now: number) => {
      raf = requestAnimationFrame(draw);
      const canvas = canvasRef.current;
      if (!canvas) return;
      const size = map.getSize();
      const dpr = window.devicePixelRatio || 1;
      if (canvas.width !== size.x * dpr || canvas.height !== size.y * dpr) {
        canvas.width = size.x * dpr;
        canvas.height = size.y * dpr;
        canvas.style.width = `${size.x}px`;
        canvas.style.height = `${size.y}px`;
      }
      const ctx = canvas.getContext("2d")!;
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.clearRect(0, 0, size.x, size.y);

      const { net: n, simRef, showTraffic } = propsRef.current;
      // Affine local-meters → screen transform (exact enough within the view).
      const p0 = map.latLngToContainerPoint(toLL([0, 0]));
      const px = map.latLngToContainerPoint(toLL([1000, 0]));
      const py = map.latLngToContainerPoint(toLL([0, 1000]));
      const ax = [(px.x - p0.x) / 1000, (px.y - p0.y) / 1000];
      const ay = [(py.x - p0.x) / 1000, (py.y - p0.y) / 1000];
      const scr = (p: Vec): [number, number] => [p0.x + p[0] * ax[0] + p[1] * ay[0], p0.y + p[0] * ax[1] + p[1] * ay[1]];
      const pxPerM = Math.hypot(ax[0], ax[1]);
      const zoom = map.getZoom();
      const sim = simRef.current;

      // Intersection dots (control type) when zoomed in.
      if (zoom >= 16) {
        const deg = new Map<string, number>();
        for (const e of Object.values(n.edges)) {
          deg.set(e.a, (deg.get(e.a) ?? 0) + 1);
          deg.set(e.b, (deg.get(e.b) ?? 0) + 1);
        }
        for (const node of Object.values(n.nodes)) {
          const d = deg.get(node.id) ?? 0;
          if (d === 2 && node.control === "auto") continue;
          const [x, y] = scr(node.p);
          if (x < -10 || y < -10 || x > size.x + 10 || y > size.y + 10) continue;
          ctx.beginPath();
          ctx.arc(x, y, d >= 3 ? 5 : 3.5, 0, Math.PI * 2);
          ctx.fillStyle = CONTROL_COLOR[node.control];
          ctx.fill();
          ctx.lineWidth = 1.5;
          ctx.strokeStyle = "#222";
          ctx.stroke();
        }
      }

      if (sim) {
        // Re-color roads by congestion once a second.
        if (showTraffic && now - lastColor > 1000) {
          lastColor = now;
          const worst = new Map<string, number>();
          for (const l of sim.links) {
            const r = l.speedEma / l.speed;
            worst.set(l.edgeId, Math.min(worst.get(l.edgeId) ?? 1, r));
          }
          const sel = propsRef.current.selection;
          for (const [id, poly] of polysRef.current) {
            const e = n.edges[id];
            if (!e) continue;
            const base = edgeStyle(e, sel?.kind === "edge" && sel.id === id);
            poly.setStyle({ ...base, color: sel?.kind === "edge" && sel.id === id ? base.color : speedColor(worst.get(id) ?? 1) });
          }
        }

        // Cars
        const carL = Math.max(3, 4.5 * pxPerM), carW = Math.max(2, 2 * pxPerM);
        for (const car of sim.cars) {
          const { p, dir } = sim.carPose(car);
          const [x, y] = scr(p);
          if (x < -10 || y < -10 || x > size.x + 10 || y > size.y + 10) continue;
          const sx = dir[0] * ax[0] + dir[1] * ay[0], sy = dir[0] * ax[1] + dir[1] * ay[1];
          const ang = Math.atan2(sy, sx);
          const ratio = car.v / car.link.speed;
          ctx.save();
          ctx.translate(x, y);
          ctx.rotate(ang);
          ctx.fillStyle = ratio > 0.6 ? "#1864ab" : ratio > 0.25 ? "#e67700" : "#c92a2a";
          ctx.fillRect(-carL / 2, -carW / 2, carL, carW);
          ctx.restore();
        }

        // Intersection report-card bubbles
        if (zoom >= 15 && showTraffic) {
          ctx.font = "bold 11px system-ui, sans-serif";
          ctx.textAlign = "center";
          ctx.textBaseline = "middle";
          for (const node of sim.nodes.values()) {
            if (node.ins.length < 2 || node.crossings < 5) continue;
            const [x, y] = scr(node.p);
            if (x < -10 || y < -10 || x > size.x + 10 || y > size.y + 10) continue;
            const g = losFor(node.delayEma, node.control === "signal");
            if (g === "A" && zoom < 17) continue;
            ctx.beginPath();
            ctx.arc(x + 12, y - 12, 9, 0, Math.PI * 2);
            ctx.fillStyle = LOS_COLOR[g];
            ctx.fill();
            ctx.fillStyle = "#fff";
            ctx.fillText(g, x + 12, y - 11.5);
          }
        }
      }
    };
    raf = requestAnimationFrame(draw);
    return () => cancelAnimationFrame(raf);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [map]);

  // Reset congestion colors when traffic view turns off / sim resets.
  useEffect(() => {
    if (props.showTraffic) return;
    for (const [id, poly] of polysRef.current) {
      const e = net.edges[id];
      if (e) poly.setStyle(edgeStyle(e, selection?.kind === "edge" && selection.id === id));
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [props.showTraffic]);

  return null;
}
