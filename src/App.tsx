import { useState, useCallback } from "react";
import L from "leaflet";
import MapView from "./components/MapView";
import SearchBar from "./components/SearchBar";
import ToolsSheet from "./components/ToolsSheet";
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
import "./App.css";

export default function App() {
  const [map, setMap] = useState<L.Map | null>(null);
  const [center, setCenter] = useState<[number, number] | null>(null); // [lon, lat]

  const [searching, setSearching] = useState(false);
  const [searchResults, setSearchResults] = useState<GeocodeResult[]>([]);

  const [roads, setRoads] = useState<RoadWay[]>([]);
  const [fetchingRoads, setFetchingRoads] = useState(false);

  const [params, setParams] = useState<RoundaboutParams | null>(null);
  const [design, setDesign] = useState<RoundaboutDesign | null>(null);
  const [sheetOpen, setSheetOpen] = useState(false);

  const pickResult = useCallback(
    (r: GeocodeResult) => {
      setCenter([r.lon, r.lat]);
      setSearchResults([]);
      map?.setView([r.lat, r.lon], 19);
    },
    [map]
  );

  const handleSearch = useCallback(
    async (q: string) => {
      setSearching(true);
      try {
        const results = await geocode(q);
        setSearchResults(results);
        if (results.length === 1) {
          pickResult(results[0]);
        }
      } catch (err) {
        console.error(err);
        alert("Search failed — see console for details.");
      } finally {
        setSearching(false);
      }
    },
    [pickResult]
  );

  const handleMapClick = useCallback((lngLat: [number, number]) => {
    setCenter(lngLat);
    setSheetOpen(true);
  }, []);

  const handleFetchRoads = useCallback(async () => {
    if (!center) return;
    setFetchingRoads(true);
    try {
      const result = await fetchRoadsNear(center[1], center[0]);
      setRoads(result);
    } catch (err) {
      console.error(err);
      alert("Fetching roads failed — see console for details.");
    } finally {
      setFetchingRoads(false);
    }
  }, [center]);

  const handleParamsChange = useCallback(
    (patch: Partial<RoundaboutParams>) => {
      setParams((prev) => {
        const base = prev ?? defaultRoundaboutParams(center ?? [0, 0]);
        const next = { ...base, ...patch };
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

  const handleExport = useCallback(() => {
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

  return (
    <div className="app">
      <div className="map-container">
        <MapView onMapReady={setMap} onMapClick={handleMapClick} roads={roads} design={design} />
      </div>
      <SearchBar
        onSearch={handleSearch}
        searching={searching}
        searchResults={searchResults}
        onPickResult={(r) => {
          pickResult(r);
          setSheetOpen(true);
        }}
      />
      <ToolsSheet
        open={sheetOpen}
        onToggle={() => setSheetOpen((o) => !o)}
        onFetchRoads={handleFetchRoads}
        fetchingRoads={fetchingRoads}
        roadCount={roads.length}
        hasCenter={!!center}
        params={params}
        onParamsChange={handleParamsChange}
        onDetectLegs={handleDetectLegs}
        onPlaceRoundabout={() => {
          handlePlaceRoundabout();
          setSheetOpen(true);
        }}
        onExport={handleExport}
      />
    </div>
  );
}
