import type { RoundaboutParams } from "../lib/roundabout";

interface ToolsSheetProps {
  open: boolean;
  onToggle: () => void;

  onFetchRoads: () => void;
  fetchingRoads: boolean;
  roadCount: number;
  hasCenter: boolean;

  params: RoundaboutParams | null;
  onParamsChange: (p: Partial<RoundaboutParams>) => void;
  onDetectLegs: () => void;
  onPlaceRoundabout: () => void;
  onExport: () => void;
}

export default function ToolsSheet(props: ToolsSheetProps) {
  return (
    <div className={`sheet ${props.open ? "open" : ""}`}>
      <button className="sheet-handle" onClick={props.onToggle} aria-expanded={props.open}>
        <span className="sheet-handle-bar" />
        <span className="sheet-handle-label">
          {props.params ? "Roundabout design" : "Design tools"}
        </span>
      </button>

      <div className="sheet-body">
        <section>
          <h2>1. Pull the real geometry</h2>
          <button onClick={props.onFetchRoads} disabled={!props.hasCenter || props.fetchingRoads}>
            {props.fetchingRoads ? "Fetching…" : "Fetch roads here"}
          </button>
          <p className="hint">{props.roadCount} road segments loaded</p>
        </section>

        <section>
          <h2>2. Design a roundabout</h2>
          <label>
            Lanes
            <select
              value={props.params?.laneCount ?? 2}
              onChange={(e) =>
                props.onParamsChange({ laneCount: Number(e.target.value) as 1 | 2 })
              }
            >
              <option value={1}>1 lane</option>
              <option value={2}>2 lane (spiral)</option>
            </select>
          </label>
          <label>
            Island radius (m)
            <input
              type="number"
              min={3}
              max={30}
              value={props.params?.islandRadius ?? 9}
              onChange={(e) => props.onParamsChange({ islandRadius: Number(e.target.value) })}
            />
          </label>
          <label>
            Lane width (m)
            <input
              type="number"
              min={2.5}
              max={5}
              step={0.1}
              value={props.params?.laneWidth ?? 3.5}
              onChange={(e) => props.onParamsChange({ laneWidth: Number(e.target.value) })}
            />
          </label>
          <label>
            Spiral bulge (m)
            <input
              type="number"
              min={0}
              max={8}
              step={0.5}
              value={props.params?.spiralBulge ?? 3}
              onChange={(e) => props.onParamsChange({ spiralBulge: Number(e.target.value) })}
            />
          </label>

          <button onClick={props.onDetectLegs} disabled={props.roadCount === 0}>
            Auto-detect legs from roads
          </button>
          <button className="primary" onClick={props.onPlaceRoundabout} disabled={!props.hasCenter}>
            Place roundabout here
          </button>
          <p className="hint">
            Drag any line or vertex on the map to fine-tune it — the roundabout is a live editable
            layer, not a fixed image.
          </p>
        </section>

        <section>
          <h2>3. Export</h2>
          <button onClick={props.onExport} disabled={!props.params}>
            Download GeoJSON
          </button>
        </section>
      </div>
    </div>
  );
}
