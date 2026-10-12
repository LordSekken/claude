import { useEffect, useState } from "react";
import {
  MPH,
  edgeLength,
  incidentEdges,
  suggestRoundaboutRadius,
  type Network,
  type NodeControl,
  type RoadEdge,
} from "../lib/network";
import type { RoadDetail } from "../lib/osmImport";
import type { Selection } from "./NetworkLayer";

export type Tool = "select" | "road" | "focus";

export interface RoadTemplate {
  lanes: number;
  speedMph: number;
  oneway: boolean;
}

interface Props {
  net: Network;
  tool: Tool;
  onTool: (t: Tool) => void;
  selection: Selection;

  detail: RoadDetail;
  onDetail: (d: RoadDetail) => void;
  viewAreaKm2: number;
  loading: boolean;
  onLoadView: () => void;

  canUndo: boolean;
  canRedo: boolean;
  onUndo: () => void;
  onRedo: () => void;

  template: RoadTemplate;
  onTemplate: (t: RoadTemplate) => void;
  drawing: boolean;
  onFinishDrawing: () => void;

  onEdgeEdit: (id: string, patch: Partial<RoadEdge>) => void;
  ops: {
    makeCurve: (id: string) => void;
    straighten: (id: string) => void;
    split: (id: string) => void;
    reverse: (id: string) => void;
    deleteEdge: (id: string) => void;
    deleteNode: (id: string) => void;
    joinNode: (id: string) => void;
    setControl: (id: string, c: NodeControl, cycle?: number) => void;
    roundabout: (id: string, radius: number, lanes: 1 | 2) => void;
    deleteFocus: (id: string) => void;
    zoomFocus: (id: string) => void;
  };

  onExport: () => void;
  onImport: (file: File) => void;
  onClear: () => void;
}

const CONTROL_LABEL: Record<NodeControl, string> = {
  auto: "Priority (side streets yield)",
  signal: "Traffic signal",
  stop: "All-way stop",
  yield: "Uncontrolled (first come)",
};

export default function BuildPanel(p: Props) {
  const edgeCount = Object.keys(p.net.edges).length;
  return (
    <>
      <section>
        <h2>Your city</h2>
        <label>
          Road detail
          <select value={p.detail} onChange={(e) => p.onDetail(e.target.value as RoadDetail)}>
            <option value="all">All streets</option>
            <option value="collectors">Collectors &amp; up</option>
            <option value="major">Major roads only</option>
          </select>
        </label>
        <button className="primary" onClick={p.onLoadView} disabled={p.loading}>
          {p.loading ? "Loading roads…" : "Load roads in this view"}
        </button>
        <p className="hint">
          {edgeCount > 0 ? `${edgeCount} road segments, ${Object.keys(p.net.nodes).length} nodes. ` : ""}
          View is {p.viewAreaKm2.toFixed(1)} km². Load a neighborhood with all streets, or a whole city with
          major roads only, then zoom in on the spots you care about.
        </p>
      </section>

      <section>
        <h2>Tools</h2>
        <div className="seg">
          {(["select", "road", "focus"] as Tool[]).map((t) => (
            <button key={t} className={p.tool === t ? "on" : ""} onClick={() => p.onTool(t)}>
              {t === "select" ? "Select / drag" : t === "road" ? "Draw road" : "Focus zone"}
            </button>
          ))}
        </div>
        <div className="row">
          <button onClick={p.onUndo} disabled={!p.canUndo}>↶ Undo</button>
          <button onClick={p.onRedo} disabled={!p.canRedo}>↷ Redo</button>
        </div>
        {p.tool === "select" && (
          <p className="hint">
            Tap a road or intersection to select it. Drag the white dots to move road ends (drop one on another
            intersection to connect them); drag blue dots to bend curves.
          </p>
        )}
        {p.tool === "road" && (
          <>
            <p className="hint">
              Tap to place points — each tap adds a road from the last one. Taps snap to intersections and split
              roads they land on. Tap the last point again (or Done) to finish.
            </p>
            <TemplateEditor t={p.template} onChange={p.onTemplate} />
            {p.drawing && <button onClick={p.onFinishDrawing}>Done drawing</button>}
          </>
        )}
        {p.tool === "focus" && (
          <p className="hint">
            Tap two opposite corners to draw a focus zone. Each zone gets its own report card in Simulate.
          </p>
        )}
      </section>

      {p.selection?.kind === "edge" && p.net.edges[p.selection.id] && (
        <EdgeInspector e={p.net.edges[p.selection.id]} {...p} />
      )}
      {p.selection?.kind === "node" && p.net.nodes[p.selection.id] && (
        <NodeInspector id={p.selection.id} {...p} />
      )}

      {p.net.focusAreas.length > 0 && (
        <section>
          <h2>Focus zones</h2>
          {p.net.focusAreas.map((f) => (
            <div className="row" key={f.id}>
              <span className="grow">{f.name}</span>
              <button onClick={() => p.ops.zoomFocus(f.id)}>View</button>
              <button onClick={() => p.ops.deleteFocus(f.id)}>Delete</button>
            </div>
          ))}
        </section>
      )}

      <section>
        <h2>Save</h2>
        <p className="hint">Your work auto-saves in this browser. Export to back it up or move devices.</p>
        <button onClick={p.onExport} disabled={edgeCount === 0}>Export project</button>
        <label className="file-btn">
          Import project
          <input
            type="file"
            accept=".json,application/json"
            onChange={(e) => {
              const f = e.target.files?.[0];
              if (f) p.onImport(f);
              e.target.value = "";
            }}
          />
        </label>
        <button onClick={p.onClear} disabled={edgeCount === 0}>Clear all</button>
      </section>
    </>
  );
}

function TemplateEditor({ t, onChange }: { t: RoadTemplate; onChange: (t: RoadTemplate) => void }) {
  return (
    <>
      <label>
        Lanes each way
        <select value={t.lanes} onChange={(e) => onChange({ ...t, lanes: Number(e.target.value) })}>
          {[1, 2, 3, 4].map((n) => <option key={n} value={n}>{n}</option>)}
        </select>
      </label>
      <label>
        Speed limit (mph)
        <input type="number" min={5} max={80} step={5} value={t.speedMph}
          onChange={(e) => onChange({ ...t, speedMph: Number(e.target.value) })} />
      </label>
      <label>
        One-way
        <input type="checkbox" checked={t.oneway} onChange={(e) => onChange({ ...t, oneway: e.target.checked })} />
      </label>
    </>
  );
}

function EdgeInspector({ e, ...p }: { e: RoadEdge } & Props) {
  const curved = !!(e.c1 && e.c2);
  return (
    <section className="inspector">
      <h2>Road {e.name ? `· ${e.name}` : ""}</h2>
      <p className="hint">{Math.round(edgeLength(p.net, e))} m long{e.roundabout ? " · roundabout ring" : ""}</p>
      <label>
        Name
        <input type="text" value={e.name ?? ""} onChange={(ev) => p.onEdgeEdit(e.id, { name: ev.target.value })} />
      </label>
      <label>
        {e.oneway ? "Lanes" : "Lanes each way"}
        <select value={e.lanes} onChange={(ev) => p.onEdgeEdit(e.id, { lanes: Number(ev.target.value) })}>
          {[1, 2, 3, 4, 5].map((n) => <option key={n} value={n}>{n}</option>)}
        </select>
      </label>
      <label>
        Speed limit (mph)
        <input type="number" min={5} max={80} step={5} value={Math.round(e.speed / MPH)}
          onChange={(ev) => p.onEdgeEdit(e.id, { speed: Number(ev.target.value) * MPH })} />
      </label>
      <label>
        Road class
        <select value={Math.ceil(e.cls)} onChange={(ev) => p.onEdgeEdit(e.id, { cls: Number(ev.target.value) })}>
          <option value={7}>Freeway</option>
          <option value={6}>Expressway</option>
          <option value={5}>Arterial</option>
          <option value={4}>Major collector</option>
          <option value={3}>Collector</option>
          <option value={2}>Local street</option>
          <option value={1}>Shared street</option>
        </select>
      </label>
      <label>
        One-way
        <input type="checkbox" checked={e.oneway} onChange={(ev) => p.onEdgeEdit(e.id, { oneway: ev.target.checked })} />
      </label>
      <div className="row wrap">
        {curved ? (
          <button onClick={() => p.ops.straighten(e.id)}>Straighten</button>
        ) : (
          <button onClick={() => p.ops.makeCurve(e.id)}>Make curve</button>
        )}
        <button onClick={() => p.ops.split(e.id)}>Split in middle</button>
        {e.oneway && <button onClick={() => p.ops.reverse(e.id)}>Flip direction</button>}
        <button className="danger" onClick={() => p.ops.deleteEdge(e.id)}>Delete road</button>
      </div>
    </section>
  );
}

function NodeInspector({ id, ...p }: { id: string } & Props) {
  const node = p.net.nodes[id];
  const roads = incidentEdges(p.net, id);
  const [radius, setRadius] = useState(() => suggestRoundaboutRadius(p.net, id));
  const [lanes, setLanes] = useState<1 | 2>(() => (Math.max(...roads.map((r) => r.lanes)) >= 2 ? 2 : 1));
  useEffect(() => {
    setRadius(suggestRoundaboutRadius(p.net, id));
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [id]);
  const names = [...new Set(roads.map((r) => r.name).filter(Boolean))];

  return (
    <section className="inspector">
      <h2>Intersection</h2>
      <p className="hint">{names.length ? names.join(" & ") : `${roads.length} roads meet here`}</p>
      <label>
        Control
        <select value={node.control} onChange={(e) => p.ops.setControl(id, e.target.value as NodeControl, node.cycle)}>
          {(Object.keys(CONTROL_LABEL) as NodeControl[]).map((c) => (
            <option key={c} value={c}>{CONTROL_LABEL[c]}</option>
          ))}
        </select>
      </label>
      {node.control === "signal" && (
        <label>
          Signal cycle (s)
          <input type="number" min={30} max={180} step={5} value={node.cycle ?? 70}
            onChange={(e) => p.ops.setControl(id, "signal", Number(e.target.value))} />
        </label>
      )}
      {roads.length >= 3 && (
        <div className="subcard">
          <strong>Make roundabout</strong>
          <label>
            Radius (m)
            <input type="number" min={6} max={60} value={radius} onChange={(e) => setRadius(Number(e.target.value))} />
          </label>
          <label>
            Ring lanes
            <select value={lanes} onChange={(e) => setLanes(Number(e.target.value) as 1 | 2)}>
              <option value={1}>1 lane</option>
              <option value={2}>2 lanes</option>
            </select>
          </label>
          <button className="primary" onClick={() => p.ops.roundabout(id, radius, lanes)}>Build roundabout</button>
        </div>
      )}
      <div className="row wrap">
        {roads.length === 2 && <button onClick={() => p.ops.joinNode(id)}>Join roads</button>}
        <button className="danger" onClick={() => p.ops.deleteNode(id)}>Delete intersection</button>
      </div>
    </section>
  );
}
