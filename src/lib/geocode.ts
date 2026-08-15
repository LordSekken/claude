export interface GeocodeResult {
  lat: number;
  lon: number;
  displayName: string;
}

// Nominatim (OpenStreetMap) geocoding — free, no API key required.
// Usage is capped by their fair-use policy (≤1 req/sec, identify your app).
export async function geocode(query: string): Promise<GeocodeResult[]> {
  const url = new URL("https://nominatim.openstreetmap.org/search");
  url.searchParams.set("q", query);
  url.searchParams.set("format", "jsonv2");
  url.searchParams.set("limit", "5");

  const res = await fetch(url.toString(), {
    headers: { Accept: "application/json" },
  });
  if (!res.ok) throw new Error(`Geocoding failed: ${res.status}`);
  const data = await res.json();
  return data.map((d: any) => ({
    lat: parseFloat(d.lat),
    lon: parseFloat(d.lon),
    displayName: d.display_name as string,
  }));
}
