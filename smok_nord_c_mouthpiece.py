#!/usr/bin/env python3
"""
Parametric SMOK Nord C Mouthpiece / Drip Tip Generator

Generates a printable STL file. All dimensions in millimeters.
>>> MEASURE YOUR BROKEN PIECE WITH CALIPERS AND ADJUST THE PARAMETERS BELOW <<<

The Nord C mouthpiece is a press-fit oval/rounded-rectangle that tapers
from a wider base (which slots into the pod) to a narrower, flatter
lip opening at the top.
"""

import numpy as np
from stl import mesh
import math

# =============================================================================
# PARAMETERS — Adjust these to match YOUR mouthpiece measurements
# =============================================================================

# --- Base (the part that press-fits INTO the pod) ---
BASE_WIDTH = 19.0        # mm — wider dimension of the base oval
BASE_DEPTH = 12.5        # mm — narrower dimension of the base oval
BASE_CORNER_RADIUS = 3.0 # mm — how rounded the base corners are
BASE_INSERT_HEIGHT = 5.0 # mm — height of the straight press-fit section

# --- Lip (the part your mouth touches at the top) ---
LIP_WIDTH = 16.0         # mm — wider dimension of the lip opening
LIP_DEPTH = 8.0          # mm — narrower dimension (front-to-back)
LIP_CORNER_RADIUS = 3.5  # mm — rounded corners at the lip
LIP_FLARE_WIDTH = 17.5   # mm — slight flare/rim at very top for comfort
LIP_FLARE_DEPTH = 9.5    # mm — slight flare depth at very top
LIP_FLARE_HEIGHT = 1.5   # mm — height of the comfort flare at top

# --- Overall shape ---
TOTAL_HEIGHT = 18.0       # mm — total mouthpiece height
WALL_THICKNESS = 1.5      # mm — wall thickness throughout
TAPER_START = 5.0         # mm — height where taper from base to lip begins

# --- Bore (internal airflow channel) ---
BORE_WIDTH = None         # auto-calculated from wall thickness if None
BORE_DEPTH = None         # auto-calculated from wall thickness if None

# --- Press-fit tolerance ---
PRESS_FIT_CLEARANCE = 0.1 # mm — subtracted from base dims for snug fit
                           # increase to 0.15-0.2 if too tight

# --- Mesh resolution ---
CORNER_SEGMENTS = 12      # segments per rounded corner (higher = smoother)
HEIGHT_SLICES = 40        # vertical slices (higher = smoother taper)

# =============================================================================
# MODEL GENERATION — You probably don't need to edit below this line
# =============================================================================

def rounded_rect_points(width, depth, corner_radius, n_segments):
    """Generate points for a rounded rectangle centered at origin."""
    hw = width / 2.0
    hd = depth / 2.0
    r = min(corner_radius, hw, hd)

    points = []
    corners = [
        (hw - r,  hd - r, 0,           math.pi / 2),
        (-hw + r, hd - r, math.pi / 2, math.pi),
        (-hw + r, -hd + r, math.pi,    3 * math.pi / 2),
        (hw - r,  -hd + r, 3 * math.pi / 2, 2 * math.pi),
    ]

    for cx, cy, a_start, a_end in corners:
        for i in range(n_segments):
            t = i / n_segments
            angle = a_start + t * (a_end - a_start)
            x = cx + r * math.cos(angle)
            y = cy + r * math.sin(angle)
            points.append((x, y))

    return points


def interpolate(v0, v1, t):
    """Linear interpolation between two values."""
    return v0 + (v1 - v0) * t


def generate_mouthpiece():
    """Generate the full mouthpiece mesh."""
    n_pts = 4 * CORNER_SEGMENTS  # points per cross-section ring

    # Apply press-fit clearance to base dimensions
    base_w = BASE_WIDTH - PRESS_FIT_CLEARANCE
    base_d = BASE_DEPTH - PRESS_FIT_CLEARANCE

    # Calculate bore dimensions
    bore_base_w = base_w - 2 * WALL_THICKNESS
    bore_base_d = base_d - 2 * WALL_THICKNESS
    bore_lip_w = LIP_WIDTH - 2 * WALL_THICKNESS
    bore_lip_d = LIP_DEPTH - 2 * WALL_THICKNESS

    # Height zones
    taper_end = TOTAL_HEIGHT - LIP_FLARE_HEIGHT

    # Build outer and inner cross-section rings at each height slice
    outer_rings = []
    inner_rings = []
    heights = []

    for i in range(HEIGHT_SLICES + 1):
        t = i / HEIGHT_SLICES
        z = t * TOTAL_HEIGHT
        heights.append(z)

        if z <= TAPER_START:
            # Straight press-fit section
            ow, od, ocr = base_w, base_d, BASE_CORNER_RADIUS
            iw, id_, icr = bore_base_w, bore_base_d, max(BASE_CORNER_RADIUS - WALL_THICKNESS, 0.5)
        elif z <= taper_end:
            # Taper zone from base to lip
            frac = (z - TAPER_START) / (taper_end - TAPER_START)
            frac = frac * frac * (3 - 2 * frac)  # smoothstep
            ow = interpolate(base_w, LIP_WIDTH, frac)
            od = interpolate(base_d, LIP_DEPTH, frac)
            ocr = interpolate(BASE_CORNER_RADIUS, LIP_CORNER_RADIUS, frac)
            iw = interpolate(bore_base_w, bore_lip_w, frac)
            id_ = interpolate(bore_base_d, bore_lip_d, frac)
            icr = interpolate(max(BASE_CORNER_RADIUS - WALL_THICKNESS, 0.5),
                              max(LIP_CORNER_RADIUS - WALL_THICKNESS, 0.5), frac)
        else:
            # Lip flare at top
            frac = (z - taper_end) / LIP_FLARE_HEIGHT
            ow = interpolate(LIP_WIDTH, LIP_FLARE_WIDTH, frac)
            od = interpolate(LIP_DEPTH, LIP_FLARE_DEPTH, frac)
            ocr = LIP_CORNER_RADIUS
            iw = bore_lip_w
            id_ = bore_lip_d
            icr = max(LIP_CORNER_RADIUS - WALL_THICKNESS, 0.5)

        outer_pts = rounded_rect_points(ow, od, ocr, CORNER_SEGMENTS)
        inner_pts = rounded_rect_points(iw, id_, icr, CORNER_SEGMENTS)

        outer_ring = [(x, y, z) for x, y in outer_pts]
        inner_ring = [(x, y, z) for x, y in inner_pts]

        outer_rings.append(outer_ring)
        inner_rings.append(inner_ring)

    # Build triangle mesh
    triangles = []

    def add_quad(p0, p1, p2, p3):
        """Add two triangles forming a quad (p0-p1-p2-p3)."""
        triangles.append((p0, p1, p2))
        triangles.append((p0, p2, p3))

    # Outer wall
    for i in range(HEIGHT_SLICES):
        for j in range(n_pts):
            j_next = (j + 1) % n_pts
            add_quad(
                outer_rings[i][j],
                outer_rings[i][j_next],
                outer_rings[i + 1][j_next],
                outer_rings[i + 1][j],
            )

    # Inner wall (reversed winding for inward-facing normals)
    for i in range(HEIGHT_SLICES):
        for j in range(n_pts):
            j_next = (j + 1) % n_pts
            add_quad(
                inner_rings[i][j_next],
                inner_rings[i][j],
                inner_rings[i + 1][j],
                inner_rings[i + 1][j_next],
            )

    # Bottom cap (ring between outer and inner at z=0)
    for j in range(n_pts):
        j_next = (j + 1) % n_pts
        add_quad(
            outer_rings[0][j_next],
            outer_rings[0][j],
            inner_rings[0][j],
            inner_rings[0][j_next],
        )

    # Top cap (ring between outer and inner at z=TOTAL_HEIGHT)
    for j in range(n_pts):
        j_next = (j + 1) % n_pts
        add_quad(
            outer_rings[-1][j],
            outer_rings[-1][j_next],
            inner_rings[-1][j_next],
            inner_rings[-1][j],
        )

    # Convert to numpy-stl mesh
    stl_mesh = mesh.Mesh(np.zeros(len(triangles), dtype=mesh.Mesh.dtype))
    for i, tri in enumerate(triangles):
        for j in range(3):
            stl_mesh.vectors[i][j] = np.array(tri[j])

    return stl_mesh


def main():
    print("SMOK Nord C Mouthpiece Generator")
    print("=" * 40)
    print(f"Base:   {BASE_WIDTH:.1f} x {BASE_DEPTH:.1f} mm (corner r={BASE_CORNER_RADIUS:.1f})")
    print(f"Lip:    {LIP_WIDTH:.1f} x {LIP_DEPTH:.1f} mm (corner r={LIP_CORNER_RADIUS:.1f})")
    print(f"Height: {TOTAL_HEIGHT:.1f} mm")
    print(f"Wall:   {WALL_THICKNESS:.1f} mm")
    print(f"Press-fit clearance: {PRESS_FIT_CLEARANCE:.2f} mm")
    print()

    mouthpiece = generate_mouthpiece()

    out_path = "smok_nord_c_mouthpiece.stl"
    mouthpiece.save(out_path)
    print(f"Saved: {out_path}")

    vol = mouthpiece.get_mass_properties()[0]
    dims = mouthpiece.max_ - mouthpiece.min_
    print(f"Bounding box: {dims[0]:.1f} x {dims[1]:.1f} x {dims[2]:.1f} mm")
    print(f"Volume: {abs(vol):.1f} mm³")
    print()
    print("PRINT SETTINGS:")
    print("  Material: PETG or food-safe resin (NOT PLA — it's porous)")
    print("  Layer height: 0.1 mm for smooth lip contact")
    print("  Infill: 100%")
    print("  Supports: none needed (printed upright)")
    print("  Orientation: base down, lip up")
    print()
    print(">>> Measure your broken piece and adjust parameters at top of script <<<")


if __name__ == "__main__":
    main()
