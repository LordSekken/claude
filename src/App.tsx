import { useState, useCallback, useEffect, useRef } from "react";
import L from "leaflet";
import MapView from "./components/MapView";
import SearchBar from "./components/SearchBar";
import SketchPanel from "./components/SketchPanel";
import BuildPanel, { type RoadTemplate, type Tool } from "./components/BuildPanel";
import SimPanel from "./components/SimPanel";
import NetworkLayer, { type Selection } from "./components/NetworkLayer";
import { geocode, type GeocodeResult } from "./lib/geocode";
import { fetchRoadsNear, type RoadWay } from "./lib/overpass";
import {
  generateRoundabout,
  defaultRoundaboutParams,
  roundaboutToFeatureCollection,
  type RoundaboutParams,
  type RoundaboutDesign,
} from "./lib/roundabout";
import { detectLegs } from "./lib/detectLegs";
import { type Vec, pointAlong, project, unproject } from "./lib/geom";
import {
  MPH,
  addRoad,
  cloneNetwork,
  deleteEdge,
  deleteNode,
  edgeNear,
  edgePoints,
  emptyNetwork,
  joinAtNode,
  makeCurve,
  makeRoundabout,
  newId,
  nodeAt,
  nodeNear,
  reverseEdge,
  splitEdge,
  straighten,
  type Network,
  type RoadEdge,
} from "./lib/network";
import { bboxAreaKm2, importOsm, type RoadDetail } from "./lib/osmImport";
import { SIM_DT, Simulation, type DemandSettings, type SimStats } from "./lib/sim";
import "./App.css";

type Tab = "build" | "sim" | "sketch";

const STORAGE_KEY = "traffic-sandbox-network-v1";
const DEFAULT_ORIGIN: [number, number] = [-119.772, 36.746];

function loadSaved(): Network {
  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    if (raw) return JSON.parse(raw) as Network;
  } catch {
    /* storage unavailable — start fresh */
  }
  return emptyNetwork(DEFAULT_ORIGIN);
}

export default function App() {
  const [map, setMap] = useState<L.Map | null>(null);
  const [tab, setTab] = useState<Tab>("build");
  const [sheetOpen, setSheetOpen] = useState(false);

  const [searching, setSearching] = useState(false);
  const [searchResults, setSearchResults] = useState<GeocodeResult[]>([]);

  // ---------------- road network + undo history ----------------
  const [net, setNet] = useState<Network>(loadSaved);
  const undoRef = useRef<Network[]>([]);
  const redoRef = useRef<Network[]>([]);
  const [, setHistoryTick] = useState(0);
  const [selection, setSelection] = useState<Selection>(null);
  const [tool, setTool] = useState<Tool>("select");
  const [draft, setDraft] = useState<{ p: Vec; nodeId?: string } | null>(null);
  const [focusCorner, setFocusCorner] = useState<Vec | null>(null);
  const [template, setTemplate] = useState<RoadTemplate>({ lanes: 1, speedMph: 30, oneway: false });
  const [detail, setDetail] = useState<RoadDetail>("all");
  const [loading, setLoading] = useState(false);
  const [viewArea, setViewArea] = useState(0);
  const [toast, setToast] = useState<string | null>(null);

  // ---------------- simulation ----------------
  const simRef = useRef<Simulation | null>(null);
  const [running, setRunning] = useState(false);
  const [speed, setSpeed] = useState(5);
  const [demand, setDemand] = useState<DemandSettings>({ vehPerHour: 3000, localShare: 0.25 });
  const [showTraffic, setShowTraffic] = useState(true);
  const [stats, setStats] = useState<SimStats | null>(null);
  const [baseline, setBaseline] = useState<SimStats | null>(null);

  // ---------------- roundabout sketcher (original tool) ----------------
  const [center, setCenter] = useState<[number, number] | null>(null);
  const [roads, setRoads] = useState<RoadWay[]>([]);
  const [fetchingRoads, setFetchingRoads] = useState(false);
  const [params, setParams] = useState<RoundaboutParams | null>(null);
  const [design, setDesign] = useState<RoundaboutDesign | null>(null);

  const flash = useCallback((msg: string) => {
    setToast(msg);
    window.setTimeout(() => setToast((t) => (t === msg ? null : t)), 3500);
  }, []);

  const commit = useCallback((next: Network) => {
    setNet((prev) => {
      undoRef.current.push(prev);
      if (undoRef.current.length > 40) undoRef.current.shift();
      redoRef.current = [];
      return next;
    });
    setHistoryTick((t) => t + 1);
  }, []);

  /** Run an edit on a copy of the network; a returned string is shown as an error and discards the edit. */
  const edit = useCallback(
    (fn: (n: Network) => string | void, after?: (n: Network) => Selection) => {
      const n = cloneNetwork(net);
      const err = fn(n);
      if (typeof err === "string") {
        flash(err);
        return;
      }
      commit(n);
      if (after) setSelection(after(n));
    },
    [net, commit, flash]
  );

  const undo = useCallback(() => {
    const prev = undoRef.current.pop();
    if (!prev) return;
    redoRef.current.push(net);
    setNet(prev);
    setHistoryTick((t) => t + 1);
  }, [net]);

  const redo = useCallback(() => {
    const next = redoRef.current.pop();
    if (!next) return;
    undoRef.current.push(net);
    setNet(next);
    setHistoryTick((t) => t + 1);
  }, [net]);

  // Drop a selection that no longer exists (after undo, delete, …).
  useEffect(() => {
    if (selection?.kind === "edge" && !net.edges[selection.id]) setSelection(null);
    if (selection?.kind === "node" && !net.nodes[selection.id]) setSelection(null);
  }, [net, selection]);

  // Auto-save.
  useEffect(() => {
    const h = window.setTimeout(() => {
      try {
        localStorage.setItem(STORAGE_KEY, JSON.stringify(net));
      } catch {
        /* too big or storage blocked — export still works */
      }
    }, 600);
    return () => window.clearTimeout(h);
  }, [net]);

  // Any edit rebuilds the simulation so it always reflects the current design.
  useEffect(() => {
    if (Object.keys(net.edges).length === 0) {
      simRef.current = null;
      setStats(null);
      return;
    }
    if (simRef.current || running) {
      simRef.current = new Simulation(net, demand);
      setStats(simRef.current.stats());
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [net]);

  useEffect(() => {
    if (simRef.current) simRef.current.demand = demand;
  }, [demand]);

  // Simulation clock.
  useEffect(() => {
    if (!running) return;
    if (!simRef.current && Object.keys(net.edges).length > 0) simRef.current = new Simulation(net, demand);
    let raf = 0, last = performance.now(), acc = 0, lastStats = 0;
    const loop = (now: number) => {
      raf = requestAnimationFrame(loop);
      const sim = simRef.current;
      if (!sim) return;
      const realDt = Math.min(0.1, (now - last) / 1000);
      last = now;
      acc += realDt * speed;
      const steps = Math.min(Math.floor(acc / SIM_DT), 240);
      acc -= steps * SIM_DT;
      if (acc > 1) acc = 0; // can't keep up at this speed — drop the backlog instead of spiraling
      const t0 = performance.now();
      for (let i = 0; i < steps; i++) {
        sim.step(1);
        if (performance.now() - t0 > 30) { acc = 0; break; }
      }
      if (now - lastStats > 300) {
        lastStats = now;
        setStats(sim.stats());
      }
    };
    raf = requestAnimationFrame(loop);
    return () => cancelAnimationFrame(raf);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [running, speed]);

  const resetSim = useCallback(() => {
    simRef.current = Object.keys(net.edges).length ? new Simulation(net, demand) : null;
    setStats(simRef.current?.stats() ?? null);
  }, [net, demand]);

  // Keyboard shortcuts.
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if ((e.target as HTMLElement).closest("input, select, textarea")) return;
      if (e.key === "Escape") { setDraft(null); setFocusCorner(null); setSelection(null); }
      if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === "z") {
        e.preventDefault();
        if (e.shiftKey) redo(); else undo();
      }
      if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === "y") { e.preventDefault(); redo(); }
      if ((e.key === "Delete" || e.key === "Backspace") && selection) {
        edit((n) => (selection.kind === "edge" ? deleteEdge(n, selection.id) : deleteNode(n, selection.id)));
      }
      if (e.key === " " && tab === "sim") { e.preventDefault(); setRunning((r) => !r); }
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [undo, redo, selection, edit, tab]);

  // Track view size for the "load roads" warning.
  useEffect(() => {
    if (!map) return;
    const update = () => {
      const b = map.getBounds();
      setViewArea(bboxAreaKm2({ south: b.getSouth(), west: b.getWest(), north: b.getNorth(), east: b.getEast() }));
    };
    update();
    map.on("moveend", update);
    return () => { map.off("moveend", update); };
  }, [map]);

  // Frame a saved network on first load.
  const framedRef = useRef(false);
  useEffect(() => {
    if (!map || framedRef.current) return;
    framedRef.current = true;
    const pts = Object.values(net.nodes).map((n) => unproject(net.origin, n.p));
    if (pts.length) map.fitBounds(L.latLngBounds(pts.map(([lon, lat]) => [lat, lon])));
  }, [map, net]);

  // ---------------- search ----------------
  const pickResult = useCallback(
    (r: GeocodeResult) => {
      setCenter([r.lon, r.lat]);
      setSearchResults([]);
      map?.setView([r.lat, r.lon], tab === "sketch" ? 19 : 16);
    },
    [map, tab]
  );

  const handleSearch = useCallback(
    async (q: string) => {
      setSearching(true);
      try {
        const results = await geocode(q);
        setSearchResults(results);
        if (results.length === 1) pickResult(results[0]);
      } catch (err) {
        console.error(err);
        flash("Search failed — check your connection.");
      } finally {
        setSearching(false);
      }
    },
    [pickResult, flash]
  );

  // ---------------- map taps ----------------
  const metersPerPixel = useCallback(() => {
    if (!map) return 1;
    const c = map.getCenter();
    return (40075016.686 * Math.cos((c.lat * Math.PI) / 180)) / Math.pow(2, map.getZoom() + 8);
  }, [map]);

  const handleMapClick = useCallback(
    (lngLat: [number, number]) => {
      if (tab === "sketch") {
        setCenter(lngLat);
        setSheetOpen(true);
        return;
      }
      const p = project(net.origin, lngLat);
      const tol = metersPerPixel() * 14;

      if (tool === "select") {
        const n = nodeNear(net, p, tol);
        if (n) { setSelection({ kind: "node", id: n.id }); setSheetOpen(true); return; }
        const e = edgeNear(net, p, tol * 0.8);
        setSelection(e ? { kind: "edge", id: e.id } : null);
        if (e) setSheetOpen(true);
        return;
      }

      if (tool === "road") {
        if (!draft) {
          const snapped = nodeNear(net, p, tol);
          setDraft(snapped ? { p: snapped.p, nodeId: snapped.id } : { p });
          return;
        }
        const n = cloneNetwork(net);
        const a = draft.nodeId && n.nodes[draft.nodeId] ? draft.nodeId : nodeAt(n, draft.p, tol);
        const b = nodeAt(n, p, tol);
        if (a === b) { setDraft(null); return; }
        const id = addRoad(n, a, b, {
          lanes: template.lanes,
          oneway: template.oneway,
          speed: template.speedMph * MPH,
          cls: template.speedMph >= 40 ? 5 : template.speedMph >= 35 ? 3 : 2,
        });
        commit(n);
        setSelection({ kind: "edge", id });
        setDraft({ p: n.nodes[b].p, nodeId: b });
        return;
      }

      if (tool === "focus") {
        if (!focusCorner) { setFocusCorner(p); return; }
        const n = cloneNetwork(net);
        const id = newId(n, "f");
        n.focusAreas.push({
          id,
          name: `Zone ${n.focusAreas.length + 1}`,
          min: [Math.min(p[0], focusCorner[0]), Math.min(p[1], focusCorner[1])],
          max: [Math.max(p[0], focusCorner[0]), Math.max(p[1], focusCorner[1])],
        });
        commit(n);
        setFocusCorner(null);
        setTool("select");
        flash("Focus zone added — its report card is in the Simulate tab.");
      }
    },
    [tab, net, tool, draft, focusCorner, template, metersPerPixel, commit, flash]
  );

  useEffect(() => {
    setDraft(null);
    setFocusCorner(null);
  }, [tool, tab]);

  // ---------------- loading a city ----------------
  const handleLoadView = useCallback(async () => {
    if (!map) return;
    const b = map.getBounds();
    const bbox = { south: b.getSouth(), west: b.getWest(), north: b.getNorth(), east: b.getEast() };
    const area = bboxAreaKm2(bbox);
    const limit = detail === "all" ? 12 : detail === "collectors" ? 80 : 400;
    if (area > limit) {
      flash(`That view is ${area.toFixed(0)} km² — too big for "${detail}" detail. Zoom in or choose fewer streets.`);
      return;
    }
    if (Object.keys(net.edges).length > 0 && !confirm("Replace your current roads with a fresh copy from OpenStreetMap?")) return;
    setLoading(true);
    try {
      const n = await importOsm(bbox, detail);
      if (Object.keys(n.edges).length === 0) {
        flash("No drivable roads found here.");
        return;
      }
      setRunning(false);
      simRef.current = null;
      setStats(null);
      setBaseline(null);
      setSelection(null);
      commit(n);
      flash(`Loaded ${Object.keys(n.edges).length} road segments.`);
    } catch (err) {
      console.error(err);
      flash("Couldn't load roads — OpenStreetMap may be busy. Try again in a moment.");
    } finally {
      setLoading(false);
    }
  }, [map, detail, net, commit, flash]);

  // ---------------- edit operations for the inspector ----------------
  const onEdgeEdit = useCallback(
    (id: string, patch: Partial<RoadEdge>) => edit((n) => { Object.assign(n.edges[id], patch); }),
    [edit]
  );

  const zoomFocus = useCallback(
    (id: string) => {
      const f = net.focusAreas.find((x) => x.id === id);
      if (!f || !map) return;
      const [w, s] = unproject(net.origin, f.min), [e, nn] = unproject(net.origin, f.max);
      map.fitBounds([[s, w], [nn, e]], { padding: [30, 30] });
    },
    [net, map]
  );

  const ops = {
    makeCurve: (id: string) => edit((n) => makeCurve(n, n.edges[id])),
    straighten: (id: string) => edit((n) => straighten(n.edges[id])),
    split: (id: string) => {
      let mid = "";
      edit(
        (n) => { mid = splitEdge(n, id, pointAlong(edgePoints(n, n.edges[id]), 0.5)); },
        () => ({ kind: "node", id: mid })
      );
    },
    reverse: (id: string) => edit((n) => reverseEdge(n.edges[id])),
    deleteEdge: (id: string) => edit((n) => deleteEdge(n, id), () => null),
    deleteNode: (id: string) => edit((n) => deleteNode(n, id), () => null),
    joinNode: (id: string) => edit((n) => (joinAtNode(n, id) ? undefined : "Can't join these roads (one-way directions don't line up)."), () => null),
    setControl: (id: string, c: Network["nodes"][string]["control"], cycle?: number) =>
      edit((n) => { n.nodes[id].control = c; if (cycle) n.nodes[id].cycle = cycle; }),
    roundabout: (id: string, radius: number, lanes: 1 | 2) => {
      edit(
        (n) => {
          const r = makeRoundabout(n, id, { radius, lanes, speed: (radius > 20 ? 25 : 20) * MPH });
          if (typeof r === "string") return r;
        },
        () => null
      );
    },
    deleteFocus: (id: string) => edit((n) => { n.focusAreas = n.focusAreas.filter((f) => f.id !== id); }),
    zoomFocus,
  };

  const handleExport = useCallback(() => {
    const blob = new Blob([JSON.stringify({ kind: "traffic-sandbox", version: 1, network: net })], { type: "application/json" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = "traffic-project.json";
    a.click();
    URL.revokeObjectURL(url);
  }, [net]);

  const handleImport = useCallback(
    async (file: File) => {
      try {
        const data = JSON.parse(await file.text());
        const n: Network = data.network ?? data;
        if (!n.nodes || !n.edges || !n.origin) throw new Error("not a project file");
        n.focusAreas ??= [];
        setRunning(false);
        simRef.current = null;
        setBaseline(null);
        commit(n);
        const pts = Object.values(n.nodes).map((x) => unproject(n.origin, x.p));
        if (pts.length && map) map.fitBounds(L.latLngBounds(pts.map(([lon, lat]) => [lat, lon])));
      } catch {
        flash("That file isn't a project export.");
      }
    },
    [commit, map, flash]
  );

  // ---------------- roundabout sketcher handlers ----------------
  const handleFetchRoads = useCallback(async () => {
    if (!center) return;
    setFetchingRoads(true);
    try {
      setRoads(await fetchRoadsNear(center[1], center[0]));
    } catch (err) {
      console.error(err);
      flash("Fetching roads failed.");
    } finally {
      setFetchingRoads(false);
    }
  }, [center, flash]);

  const handleParamsChange = useCallback(
    (patch: Partial<RoundaboutParams>) => {
      setParams((prev) => {
        const next = { ...(prev ?? defaultRoundaboutParams(center ?? [0, 0])), ...patch };
        setDesign(generateRoundabout(next));
        return next;
      });
    },
    [center]
  );

  const handleDetectLegs = useCallback(() => {
    if (!center) return;
    const legs = detectLegs(center, roads);
    handleParamsChange({ legs: legs.length > 0 ? legs : undefined });
  }, [center, roads, handleParamsChange]);

  const handlePlaceRoundabout = useCallback(() => {
    if (!center) return;
    const legs = roads.length > 0 ? detectLegs(center, roads) : undefined;
    const next = defaultRoundaboutParams(center, params?.laneCount ?? 2, legs?.length ? legs : undefined);
    setParams(next);
    setDesign(generateRoundabout(next));
  }, [center, roads, params]);

  const handleSketchExport = useCallback(() => {
    if (!design) return;
    const fc = roundaboutToFeatureCollection(design);
    const blob = new Blob([JSON.stringify(fc, null, 2)], { type: "application/geo+json" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = "roundabout-design.geojson";
    a.click();
    URL.revokeObjectURL(url);
  }, [design]);

  const hasNetwork = Object.keys(net.edges).length > 0;
  const toolHint =
    tab !== "build" ? null
      : tool === "road" ? (draft ? "Tap the next point (tap the same point to finish)" : "Tap where the road starts")
      : tool === "focus" ? (focusCorner ? "Tap the opposite corner" : "Tap one corner of the zone")
      : null;

  return (
    <div className="app">
      <div className="map-container">
        <MapView
          onMapReady={setMap}
          onMapClick={handleMapClick}
          roads={tab === "sketch" ? roads : []}
          design={tab === "sketch" ? design : null}
          showClickMarker={tab === "sketch"}
        />
      </div>
      {map && tab !== "sketch" && (
        <NetworkLayer
          map={map}
          net={net}
          selection={selection}
          onCommit={commit}
          simRef={simRef}
          showTraffic={showTraffic && !!stats}
          draftFrom={draft?.p ?? null}
          focusCorner={focusCorner}
        />
      )}
      <SearchBar
        onSearch={handleSearch}
        searching={searching}
        searchResults={searchResults}
        onPickResult={pickResult}
      />
      {(toolHint || toast) && <div className="toast">{toast ?? toolHint}</div>}

      <div className={`sheet ${sheetOpen ? "open" : ""}`}>
        <button className="sheet-handle" onClick={() => setSheetOpen((o) => !o)} aria-expanded={sheetOpen}>
          <span className="sheet-handle-bar" />
          <span className="sheet-handle-label">
            {tab === "sim" && stats ? `Simulating · grade ${stats.los}` : "Traffic Sandbox"}
          </span>
        </button>
        <div className="tabs">
          {(["build", "sim", "sketch"] as Tab[]).map((t) => (
            <button
              key={t}
              className={tab === t ? "on" : ""}
              onClick={() => { setTab(t); setSheetOpen(true); }}
            >
              {t === "build" ? "Build" : t === "sim" ? "Simulate" : "Roundabout sketch"}
            </button>
          ))}
        </div>
        <div className="sheet-body">
          {tab === "build" && (
            <BuildPanel
              net={net}
              tool={tool}
              onTool={setTool}
              selection={selection}
              detail={detail}
              onDetail={setDetail}
              viewAreaKm2={viewArea}
              loading={loading}
              onLoadView={handleLoadView}
              canUndo={undoRef.current.length > 0}
              canRedo={redoRef.current.length > 0}
              onUndo={undo}
              onRedo={redo}
              template={template}
              onTemplate={setTemplate}
              drawing={!!draft}
              onFinishDrawing={() => setDraft(null)}
              onEdgeEdit={onEdgeEdit}
              ops={ops}
              onExport={handleExport}
              onImport={handleImport}
              onClear={() => {
                if (!confirm("Delete all roads and focus zones?")) return;
                setRunning(false);
                commit(emptyNetwork(net.origin));
              }}
            />
          )}
          {tab === "sim" && (
            <SimPanel
              hasNetwork={hasNetwork}
              running={running}
              onRun={(r) => {
                if (r && !simRef.current) resetSim();
                setRunning(r);
              }}
              speed={speed}
              onSpeed={setSpeed}
              onReset={resetSim}
              demand={demand}
              onDemand={setDemand}
              showTraffic={showTraffic}
              onShowTraffic={setShowTraffic}
              stats={stats}
              baseline={baseline}
              onSaveBaseline={() => setBaseline(stats)}
              onClearBaseline={() => setBaseline(null)}
              onZoomFocus={zoomFocus}
            />
          )}
          {tab === "sketch" && (
            <SketchPanel
              onFetchRoads={handleFetchRoads}
              fetchingRoads={fetchingRoads}
              roadCount={roads.length}
              hasCenter={!!center}
              params={params}
              onParamsChange={handleParamsChange}
              onDetectLegs={handleDetectLegs}
              onPlaceRoundabout={handlePlaceRoundabout}
              onExport={handleSketchExport}
            />
          )}
        </div>
      </div>
    </div>
  );
}
