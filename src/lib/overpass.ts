export interface RoadWay {
  id: number;
  tags: Record<string, string>;
  coords: [number, number][]; // [lon, lat]
}

const OVERPASS_ENDPOINT = "https://overpass-api.de/api/interpreter";

// Fetches drivable ways (highway=*) within `radiusMeters` of (lat, lon),
// giving us the real centerlines of the intersection to redesign over.
export async function fetchRoadsNear(
  lat: number,
  lon: number,
  radiusMeters = 120
): Promise<RoadWay[]> {
  const query = `
    [out:json][timeout:25];
    way(around:${radiusMeters},${lat},${lon})
      ["highway"]
      ["highway"!~"^(footway|path|cycleway|steps|pedestrian|track|service)$"];
    out body geom;
  `;

  const res = await fetch(OVERPASS_ENDPOINT, {
    method: "POST",
    body: "data=" + encodeURIComponent(query),
    headers: { "Content-Type": "application/x-www-form-urlencoded" },
  });
  if (!res.ok) throw new Error(`Overpass query failed: ${res.status}`);
  const data = await res.json();

  return (data.elements as any[])
    .filter((el) => el.type === "way" && el.geometry)
    .map((el) => ({
      id: el.id,
      tags: el.tags ?? {},
      coords: el.geometry.map((g: any) => [g.lon, g.lat] as [number, number]),
    }));
}
