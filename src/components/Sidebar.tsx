import { useState } from "react";
import type { GeocodeResult } from "../lib/geocode";
import type { RoundaboutParams } from "../lib/roundabout";

interface SidebarProps {
  onSearch: (query: string) => Promise<void>;
  searching: boolean;
  searchResults: GeocodeResult[];
  onPickResult: (r: GeocodeResult) => void;
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

export default function Sidebar(props: SidebarProps) {
  const [query, setQuery] = useState("");

  return (
    <div className="sidebar">
      <h1>Intersection Editor</h1>

      <section>
        <h2>1. Find the intersection</h2>
        <form
          onSubmit={(e) => {
            e.preventDefault();
            props.onSearch(query);
          }}
        >
          <input
            type="text"
            placeholder="Shaw Ave & Pioneer Ave, Fresno CA"
            value={query}
            onChange={(e) => setQuery(e.target.value)}
          />
          <button type="submit" disabled={props.searching}>
            {props.searching ? "Searching…" : "Search"}
          </button>
        </form>
        {props.searchResults.length > 0 && (
          <ul className="results">
            {props.searchResults.map((r, i) => (
              <li key={i}>
                <button onClick={() => props.onPickResult(r)}>{r.displayName}</button>
              </li>
            ))}
          </ul>
        )}
        <p className="hint">Or just click the intersection on the map to set the center.</p>
      </section>

      <section>
        <h2>2. Pull the real geometry</h2>
        <button onClick={props.onFetchRoads} disabled={!props.hasCenter || props.fetchingRoads}>
          {props.fetchingRoads ? "Fetching…" : "Fetch roads here"}
        </button>
        <p className="hint">{props.roadCount} road segments loaded</p>
      </section>

      <section>
        <h2>3. Design a roundabout</h2>
        <label>
          Lanes
          <select
            value={props.params?.laneCount ?? 2}
            onChange={(e) => props.onParamsChange({ laneCount: Number(e.target.value) as 1 | 2 })}
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
        <button onClick={props.onPlaceRoundabout} disabled={!props.hasCenter}>
          Place roundabout at map center
        </button>
        <p className="hint">
          Drag any line/vertex on the map to fine-tune it — the roundabout is a live editable
          layer, not a fixed image.
        </p>
      </section>

      <section>
        <h2>4. Export</h2>
        <button onClick={props.onExport} disabled={!props.params}>
          Download GeoJSON
        </button>
      </section>
    </div>
  );
}
