#!/usr/bin/env python3
"""Downloads CC0 photo-scanned PBR textures from Poly Haven and converts them
to DDS (DXT5, full mip chain) for Drift Lab's Assetto Corsa track export.

Usage: build_hd_textures.py <out_dir>

Writes <out_dir>/<kind>_d.dds (diffuse, alpha = gloss/specular mask) and
<kind>_n.dds (DirectX normal map) plus CREDITS.txt. Poly Haven assets are CC0
(public domain), so the files can be redistributed with the app.
"""
import io
import json
import os
import struct
import sys
import urllib.request

from PIL import Image, ImageChops

# kind: (poly haven id, diffuse res, normal res, real-world tile size in metres)
MATERIALS = {
    "asphalt": ("asphalt_track", "4k", "2k"),
    "grass": ("leafy_grass", "2k", "2k"),
    "dirt": ("forest_ground_04", "2k", "2k"),
    "gravel": ("gravel_floor_02", "2k", "2k"),
    "concrete": ("brushed_concrete_03", "2k", "2k"),
}
# Colour gain per channel applied to the diffuse map (kind -> (r, g, b)).
TINT = {"grass": (0.72, 1.0, 0.55)}
API = "https://api.polyhaven.com"
UA = {"User-Agent": "DriftLab-texture-builder/1.0"}


def fetch(url: str) -> bytes:
    req = urllib.request.Request(url, headers=UA)
    with urllib.request.urlopen(req, timeout=120) as r:
        return r.read()


def map_url(files: dict, key: str, res: str) -> str:
    return files[key][res]["jpg"]["url"]


def dds_dxt5(img: Image.Image) -> bytes:
    """RGBA image -> DXT5 DDS with a full mip chain (header written here,
    each level compressed by Pillow)."""
    img = img.convert("RGBA")
    w, h = img.size
    levels = []
    lw, lh = w, h
    cur = img
    while True:
        b = io.BytesIO()
        cur.save(b, "DDS", pixel_format="DXT5")
        raw = b.getvalue()
        assert raw[:4] == b"DDS " and raw[84:88] == b"DXT5", "unexpected DDS from Pillow"
        levels.append(raw[128:])
        if lw <= 4 and lh <= 4:
            break
        lw, lh = max(lw // 2, 4), max(lh // 2, 4)
        cur = img.resize((lw, lh), Image.LANCZOS)
    flags = 0x1 | 0x2 | 0x4 | 0x1000 | 0x20000 | 0x80000  # caps|height|width|pixelformat|mipmapcount|linearsize
    header = struct.pack(
        "<4s7I44x2I4s5I4I4x",
        b"DDS ", 124, flags, h, w, len(levels[0]), 0, len(levels),
        32, 0x4, b"DXT5", 0, 0, 0, 0, 0,          # pixel format: FOURCC DXT5
        0x1000 | 0x400000 | 0x8, 0, 0, 0,          # caps: texture|mipmap|complex
    )
    assert len(header) == 128
    return header + b"".join(levels)


def build(kind: str, asset: str, dres: str, nres: str, out: str) -> dict:
    files = json.loads(fetch(f"{API}/files/{asset}"))
    info = json.loads(fetch(f"{API}/info/{asset}"))
    diff = Image.open(io.BytesIO(fetch(map_url(files, "Diffuse", dres)))).convert("RGB")
    rough = Image.open(io.BytesIO(fetch(map_url(files, "Rough", dres)))).convert("L").resize(diff.size)
    if "AO" in files:
        ao = Image.open(io.BytesIO(fetch(map_url(files, "AO", dres)))).convert("L").resize(diff.size)
        # Bake ambient occlusion into the colour (softened) for depth in AC.
        ao = ao.point(lambda v: int(150 + v * 105 / 255))
        diff = ImageChops.multiply(diff, Image.merge("RGB", (ao, ao, ao)))
    if kind in TINT:
        # Pull dry, yellowish scans toward a fresher colour (per-channel gain).
        gr, gg, gb = TINT[kind]
        r, g, b = diff.split()
        diff = Image.merge("RGB", (r.point(lambda v: min(255, int(v * gr))), g.point(lambda v: min(255, int(v * gg))), b.point(lambda v: min(255, int(v * gb)))))
    gloss = rough.point(lambda v: 255 - v)
    d = diff.copy()
    d.putalpha(gloss)
    with open(os.path.join(out, f"{kind}_d.dds"), "wb") as f:
        f.write(dds_dxt5(d))
    nrm = Image.open(io.BytesIO(fetch(map_url(files, "nor_dx", nres)))).convert("RGB")
    n = nrm.copy()
    n.putalpha(255)
    with open(os.path.join(out, f"{kind}_n.dds"), "wb") as f:
        f.write(dds_dxt5(n))
    dims = info.get("dimensions") or [2000, 2000]
    return {"asset": asset, "tile_m": round(float(dims[0]) / 1000.0, 2), "authors": list((info.get("authors") or {}).keys())}


def main() -> None:
    out = sys.argv[1] if len(sys.argv) > 1 else "hd_textures"
    os.makedirs(out, exist_ok=True)
    open(os.path.join(out, ".gdignore"), "w").close()  # keep Godot's importer out of it
    manifest = {}
    credits = ["Textures from Poly Haven (https://polyhaven.com), CC0 public domain.\n"]
    for kind, (asset, dres, nres) in MATERIALS.items():
        print(f"{kind}: {asset} ({dres} diffuse, {nres} normal)...", flush=True)
        m = build(kind, asset, dres, nres, out)
        manifest[kind] = m
        credits.append(f"{kind}: {asset} by {', '.join(m['authors'])} - https://polyhaven.com/a/{asset}")
    with open(os.path.join(out, "manifest.json"), "w") as f:
        json.dump(manifest, f, indent=2)
    with open(os.path.join(out, "CREDITS.txt"), "w") as f:
        f.write("\n".join(credits) + "\n")
    print("done:", out)


if __name__ == "__main__":
    main()
