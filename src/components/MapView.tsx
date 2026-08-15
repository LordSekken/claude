import { useEffect, useRef } from "react";
import L from "leaflet";
import "leaflet/dist/leaflet.css";
import "@geoman-io/leaflet-geoman-free";
import "@geoman-io/leaflet-geoman-free/dist/leaflet-geoman.css";
import type { RoadWay } from "../lib/overpass";
import type { RoundaboutDesign } from "../lib/roundabout";

export interface MapViewHandle {
  map: L.Map;
}

interface MapViewProps {
  onMapReady: (map: L.Map) => void;
  onMapClick: (lngLat: [number, number]) => void;
  roads: RoadWay[];
  design: RoundaboutDesign | null;
}

const roadStyle = (highway: string): L.PathOptions => {
  const major = ["primary", "secondary", "trunk"].includes(highway);
  return { color: major ? "#666" : "#999", weight: major ? 5 : 3, opacity: 0.85 };
};

const designLayerStyle: Record<string, L.PathOptions> = {
  island: { color: "#2f7d32", fillColor: "#8fce8f", fillOpacity: 0.6, weight: 2 },
  "circulatory-outer": { color: "#1a4fd6", weight: 3 },
  "lane-divider": { color: "#e0a800", weight: 2, dashArray: "6 6" },
  "entry-lane": { color: "#d6281a", weight: 3 },
  "splitter-island": { color: "#2f7d32", fillColor: "#a3d9a5", fillOpacity: 0.7, weight: 1 },
};

export default function MapView({ onMapReady, onMapClick, roads, design }: MapViewProps) {
  const containerRef = useRef<HTMLDivElement | null>(null);
  const mapRef = useRef<L.Map | null>(null);
  const roadLayerRef = useRef<L.LayerGroup | null>(null);
  const designLayerRef = useRef<L.LayerGroup | null>(null);
  const clickMarkerRef = useRef<L.Marker | null>(null);
  const onMapClickRef = useRef(onMapClick);
  onMapClickRef.current = onMapClick;

  useEffect(() => {
    if (!containerRef.current || mapRef.current) return;
    const map = L.map(containerRef.current, { zoomControl: false }).setView(
      [36.746, -119.772],
      18
    );
    L.control.zoom({ position: "bottomright" }).addTo(map);
    L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
      maxZoom: 20,
      attribution: "&copy; OpenStreetMap contributors",
    }).addTo(map);

    roadLayerRef.current = L.layerGroup().addTo(map);
    designLayerRef.current = L.layerGroup().addTo(map);

    map.on("click", (e: L.LeafletMouseEvent) => {
      if (clickMarkerRef.current) {
        clickMarkerRef.current.setLatLng(e.latlng);
      } else {
        clickMarkerRef.current = L.marker(e.latlng, { title: "Intersection center" }).addTo(map);
      }
      onMapClickRef.current([e.latlng.lng, e.latlng.lat]);
    });

    mapRef.current = map;
    onMapReady(map);

    return () => {
      map.remove();
      mapRef.current = null;
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  useEffect(() => {
    const layer = roadLayerRef.current;
    if (!layer) return;
    layer.clearLayers();
    for (const road of roads) {
      const latlngs = road.coords.map(([lon, lat]) => [lat, lon] as [number, number]);
      L.polyline(latlngs, roadStyle(road.tags.highway ?? "")).bindTooltip(
        road.tags.name ?? road.tags.highway ?? "road",
        { sticky: true }
      ).addTo(layer);
    }
  }, [roads]);

  useEffect(() => {
    const layer = designLayerRef.current;
    if (!layer) return;
    layer.clearLayers();
    if (!design) return;

    const addFeature = (feature: any) => {
      const kind = feature.properties?.kind ?? "";
      const style = designLayerStyle[kind] ?? { color: "#333", weight: 2 };
      const geoLayer = L.geoJSON(feature, { style, pmIgnore: false });
      geoLayer.addTo(layer);
      geoLayer.eachLayer((l: any) => {
        if (l.pm) l.pm.enable({ allowSelfIntersection: true, draggable: true });
      });
    };

    addFeature(design.island);
    addFeature(design.circulatoryOuter);
    design.laneDividers.forEach(addFeature);
    design.approachLanes.forEach(addFeature);
    design.splitterIslands.forEach(addFeature);
  }, [design]);

  return <div ref={containerRef} style={{ width: "100%", height: "100%" }} />;
}
