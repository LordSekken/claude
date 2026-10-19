class_name DriftCircuitGenerator
extends RefCounted
## Generates drift circuits (Ebisu / Meihan style) as TrackSpec dictionaries.
##
## Layout grammar: a main straight into a fast entry corner, then chains of
## linked corners (transitions left<->right with short links and similar
## radii so they flow, double apexes, the odd hairpin after a slow section),
## separated by short resets. Each chain is a drift zone, each apex a clipping
## point.
##
## Scale: corners are sized for drifting a road car - sweeper 45-75 m radius,
## medium 25-40, tight 15-22, hairpin 11-15 (never tighter than the road can
## fold around). A drift's speed follows from the radius, v = sqrt(0.75 g R),
## so the report lists each corner's drift speed; "scale" multiplies every
## radius and straight (bigger, faster track) while the width stays a road.
##
## Closing the loop: after the corner chains, a turn - straight - turn (Dubins
## path) leads exactly back onto the start line, with the lap turning exactly
## once round. Layouts that still overlap fail TrackSpec.validate and the next
## attempt is tried.
##
## params: seed, length (m, ~1400), width (m, 13), scale (1.0), flow (0..1:
## more sweepers vs more technical), elevation (m, 6), clockwise (bool or omit).

const DRIFT_G := 0.75 * 9.81
const MAX_ATTEMPTS := 60

const KINDS := {
	# name: [min radius, max radius, min angle, max angle] (m, degrees)
	"sweeper": [45.0, 75.0, 45.0, 110.0],
	"medium": [25.0, 40.0, 60.0, 130.0],
	"tight": [15.0, 22.0, 80.0, 150.0],
	"hairpin": [11.0, 15.0, 150.0, 190.0],
}


static func generate(params: Dictionary = {}) -> Dictionary:
	var seed := int(params.get("seed", 1))
	var last_err := ""
	for attempt in MAX_ATTEMPTS:
		var r := _attempt(params, seed * 7919 + attempt)
		if r.ok:
			r.spec["generator"] = {"seed": seed, "attempt": attempt, "params": params.duplicate()}
			return r
		last_err = r.message
	return {"ok": false, "message": "no valid layout in %d attempts (last: %s)" % [MAX_ATTEMPTS, last_err]}


## Drift speed (km/h) for a corner radius.
static func drift_speed_kmh(radius: float) -> float:
	return sqrt(DRIFT_G * radius) * 3.6


static func _attempt(params: Dictionary, rng_seed: int) -> Dictionary:
	var rng := RandomNumberGenerator.new()
	rng.seed = rng_seed
	var target := float(params.get("length", 1400.0))
	var width := float(params.get("width", 13.0))
	var scale := clampf(float(params.get("scale", 1.0)), 0.5, 3.0)
	var flow := clampf(float(params.get("flow", 0.5)), 0.0, 1.0)
	var elevation := float(params.get("elevation", 6.0))
	var cw: bool = bool(params.clockwise) if params.has("clockwise") else rng.randf() < 0.5
	var dir := 1.0 if cw else -1.0   # net turning direction (+ = right)
	var min_r := width * 0.5 + 3.0    # tighter and the inside edge folds over itself

	# --- 1. Element list: straights (adjustable) and arcs. -------------------
	var els: Array = []   # {type:"s", len, min} | {type:"a", r, ang(deg, signed), kind, chain}
	var length := 0.0
	var chain := 0
	els.append({"type": "s", "len": rng.randf_range(110, 170) * scale, "min": 70.0 * scale})
	length += els[-1].len
	var first := true
	var net_so_far := 0.0
	while length < target * 0.7:   # the closing turn-straight-turn adds the rest
		chain += 1
		var n_corners := rng.randi_range(2, 4) if not first else rng.randi_range(2, 3)
		var prev_kind := ""
		var prev_sign := dir
		var prev_r := 0.0
		for c in n_corners:
			var kind := _pick_kind(rng, flow, first and c == 0, prev_kind)
			var k: Array = KINDS[kind]
			var r := rng.randf_range(k[0], k[1]) * scale
			if prev_r > 0.0 and kind != "hairpin" and prev_kind != "hairpin":
				r = clampf(r, prev_r * 0.65, prev_r * 1.6)   # linked corners: similar speeds
			r = maxf(r, min_r)
			var ang := rng.randf_range(k[2], k[3])
			# Transitions: mostly alternate direction inside a chain. Otherwise lean towards
			# the turning the lap needs by now (one full turn spread over the lap), so the
			# layout curls round instead of wandering off and closing with a long straight.
			var want := 360.0 * dir * clampf(length / (target * 0.7), 0.0, 1.0) * 0.8
			var lean := signf(want - net_so_far) if absf(want - net_so_far) > 30.0 else dir
			var sgn := -prev_sign if (c > 0 and rng.randf() < 0.6) else (lean if rng.randf() < 0.8 else -lean)
			if c > 0 and sgn == prev_sign and rng.randf() < 0.5:
				# Double apex: same direction, a short opening between.
				els.append({"type": "a", "r": r * 2.5, "ang": 0.0, "kind": "opening", "chain": chain})
			els.append({"type": "a", "r": r, "ang": ang * sgn, "kind": kind, "chain": chain})
			length += deg_to_rad(ang) * r
			net_so_far += ang * sgn
			if c < n_corners - 1:
				var link := rng.randf_range(12, 30) * scale
				els.append({"type": "s", "len": link, "min": 8.0 * scale})
				length += link
			prev_kind = kind
			prev_sign = sgn
			prev_r = r
		first = false
		var reset := rng.randf_range(35, 80) * scale
		els.append({"type": "s", "len": reset, "min": 20.0 * scale})
		length += reset

	# Opening arcs had angle 0 - give them a gentle real bend in the corner's direction
	# and take it back off the corner after.
	for i in els.size():
		var e: Dictionary = els[i]
		if e.type == "a" and e.kind == "opening" and i + 1 < els.size():
			var nxt: Dictionary = els[i + 1]
			var give := clampf(absf(float(nxt.ang)) * 0.15, 5.0, 20.0)
			e.ang = give * signf(float(nxt.ang))
			nxt.ang = float(nxt.ang) - float(e.ang)

	# --- 2. Close the loop exactly: turn - straight - turn back onto the start line
	# (a Dubins path), trying a few corner sizes; the whole lap must turn once round.
	var walk := _walk(els)
	var net := 0.0
	for e: Dictionary in els:
		if e.type == "a":
			net += float(e.ang)
	chain += 1
	var best: Array = []
	var best_len := INF
	for kind: String in ["medium", "sweeper", "tight"]:
		var k: Array = KINDS[kind]
		for frac: float in [0.2, 0.5, 0.8]:
			var r := maxf(lerpf(k[0], k[1], frac) * scale, min_r)
			for path: Array in _dubins(walk.end, walk.heading, Vector2.ZERO, 0.0, r):
				var a1: float = path[0]
				var straight: float = path[1]
				var a2: float = path[2]
				if straight < 20.0 * scale or straight > maxf(200.0 * scale, target * 0.15) or absf(a1) > 200.0 or absf(a2) > 200.0:
					continue
				if absf(net + a1 + a2 - 360.0 * dir) > 1.0:
					continue   # must make exactly one full turn (no figure-8 / extra loop)
				var plen := (absf(a1) + absf(a2)) * PI / 180.0 * r + straight
				if plen < best_len:
					best_len = plen
					best = [r, a1, straight, a2, kind]
	if best.is_empty():
		return {"ok": false, "message": "no closing path"}
	for spec_el: Dictionary in [{"type": "a", "r": best[0], "ang": best[1], "kind": best[4], "chain": chain},
			{"type": "s", "len": best[2], "min": 20.0 * scale},
			{"type": "a", "r": best[0], "ang": best[3], "kind": best[4], "chain": chain}]:
		if spec_el.type == "a" and absf(float(spec_el.ang)) < 2.0:
			continue
		els.append(spec_el)
	walk = _walk(els)
	if (walk.end as Vector2).length() > 0.5:
		return {"ok": false, "message": "closure gap %.1f m" % (walk.end as Vector2).length()}

	# --- 4. Control points, widths, banks, elevation, zones, clips. ----------
	var pts: Array = walk.points       # [Vector2 pos, dist, element index]
	var total := float(walk.length)
	if pts.size() > 1 and (pts[-1][0] as Vector2).distance_to(pts[0][0]) < 2.0:
		pts.pop_back()
	var phase := [rng.randf() * TAU, rng.randf() * TAU]
	var points: Array = []
	var widths: Array = []
	var banks: Array = []
	for p: Array in pts:
		var pos: Vector2 = p[0]
		var u := float(p[1]) / total
		var y := elevation * 0.5 * (0.6 * sin(TAU * u + phase[0]) + 0.4 * sin(TAU * 2.0 * u + phase[1]))   # ~elevation m top to bottom
		points.append([snappedf(pos.x, 0.01), snappedf(pos.y, 0.01), snappedf(y, 0.01)])
		var e: Dictionary = els[int(p[2])]
		var w := width
		var bank := 0.0
		if e.type == "a" and absf(float(e.ang)) > 1.0:
			if e.kind == "hairpin":
				w = width + 2.0   # room to swing the car round
			bank = -signf(float(e.ang)) * 0.02   # gentle crossfall into the corner
		widths.append(w)
		banks.append(bank)

	var zones: Array = []
	var clips: Array = []
	var corners: Array = []
	var chain_span := {}
	for i in els.size():
		var e: Dictionary = els[i]
		if e.type != "a" or absf(float(e.ang)) < 1.0:
			continue
		var t0 := float(walk.starts[i]) / total
		var t1 := float(walk.starts[i] + walk.lengths[i]) / total
		var apex := (t0 + t1) * 0.5
		if e.kind != "opening":
			var side := "right" if float(e.ang) > 0.0 else "left"
			clips.append({"t": snappedf(apex, 0.0001), "side": side, "radius": 2.0})
			corners.append({"kind": e.kind, "radius_m": snappedf(float(e.r), 0.1), "angle_deg": snappedf(float(e.ang), 0.1),
				"t": snappedf(apex, 0.0001), "drift_speed_kmh": snappedf(drift_speed_kmh(float(e.r)), 0.1)})
		var ch := int(e.chain)
		if not chain_span.has(ch):
			chain_span[ch] = [t0, t1]
		chain_span[ch][1] = t1
	var zi := 0
	for ch: int in chain_span:
		zi += 1
		var span: Array = chain_span[ch]
		zones.append({"name": "Zone %d" % zi, "start": snappedf(maxf(0.0, span[0] - 0.01), 0.0001),
			"end": snappedf(minf(1.0, span[1] + 0.005), 0.0001), "target": 8000 + 2000 * zi})

	var spec := {
		"name": str(params.get("name", "Drift Circuit %d" % int(params.get("seed", 1)))),
		"description": "Generated drift circuit: %d corners, %.0f m." % [corners.size(), total],
		"closed": true,
		"points": points,
		"width": width,
		"widths": widths,
		"banks": banks,
		"runoff": 5.0,
		"runoff_surface": "grass",
		"walls": {"left": true, "right": true, "height": 1.2},
		"kerbs": true,
		"zones": zones,
		"clips": clips,
		"start": 0.02,
		"scenery": {"seed": int(params.get("seed", 1)), "density": 1.0, "time_of_day": "day"},
		"corners": corners,
	}
	var v := TrackSpec.validate(spec)
	if not (v.errors as Array).is_empty():
		return {"ok": false, "message": "; ".join(v.errors)}
	return {"ok": true, "spec": spec, "stats": v.stats, "warnings": v.warnings, "corners": corners}


static func _pick_kind(rng: RandomNumberGenerator, flow: float, entry: bool, prev: String) -> String:
	if entry:
		return "sweeper" if rng.randf() < 0.5 + flow * 0.4 else "medium"
	var w := {"sweeper": 0.15 + 0.35 * flow, "medium": 0.4, "tight": 0.35 - 0.2 * flow, "hairpin": 0.12 - 0.08 * flow}
	if prev == "hairpin" or prev == "sweeper":
		w.hairpin = 0.0   # no hairpin straight out of a fast corner or another hairpin
	var total := 0.0
	for k: String in w:
		total += float(w[k])
	var x := rng.randf() * total
	for k: String in w:
		x -= float(w[k])
		if x <= 0.0:
			return k
	return "medium"


## Integrates the element list from the origin heading +X. Returns the end
## point, each element's start direction (straights), start distance and length,
## and control points (spacing ~R/2.5 on arcs, ~25 m on straights).
static func _walk(els: Array) -> Dictionary:
	var pos := Vector2.ZERO
	var h := 0.0   # heading (rad); + turns right in Godot's XZ (x right, z back)
	var dist := 0.0
	var dirs := {}
	var starts := {}
	var lengths := {}
	var points: Array = []
	for i in els.size():
		var e: Dictionary = els[i]
		starts[i] = dist
		if e.type == "s":
			var d := Vector2(cos(h), sin(h))
			dirs[i] = d
			var L := float(e.len)
			var n := maxi(1, int(ceil(L / 25.0)))
			for k in n:
				points.append([pos + d * (L * k / n), dist + L * k / n, i])
			pos += d * L
			dist += L
			lengths[i] = L
		else:
			var r := float(e.r)
			var ang := deg_to_rad(float(e.ang))
			var arc := absf(ang) * r
			var n := maxi(2, int(ceil(arc / clampf(r / 2.5, 3.0, 12.0))))
			var ds := arc / n
			var k_curv := signf(ang) / r
			for k in n:
				points.append([pos, dist, i])
				# Exact arc step: chord of the sub-arc.
				var dh := k_curv * ds
				var chord := 2.0 * r * sin(absf(dh) * 0.5) if absf(dh) > 1e-9 else ds
				var mid_h := h + dh * 0.5
				pos += Vector2(cos(mid_h), sin(mid_h)) * chord
				h += dh
				dist += ds
			lengths[i] = arc
	return {"end": pos, "heading": h, "dirs": dirs, "starts": starts, "lengths": lengths, "points": points, "length": dist}


## Dubins CSC paths from pose (p0, h0) to (p1, h1) with turn radius r. Headings in
## radians, + turning right (CCW in the x/z plane as maths axes). Returns
## [[arc1 deg (signed), straight m, arc2 deg (signed)], ...] for LSL/RSR/LSR/RSL.
static func _dubins(p0: Vector2, h0: float, p1: Vector2, h1: float, r: float) -> Array:
	var out: Array = []
	for a: float in [1.0, -1.0]:
		for b: float in [1.0, -1.0]:
			var c0 := p0 + a * r * Vector2(-sin(h0), cos(h0))
			var c1 := p1 + b * r * Vector2(-sin(h1), cos(h1))
			var dv := c1 - c0
			var d := dv.length()
			var phi := atan2(dv.y, dv.x)
			var theta: float
			var L: float
			if a == b:
				theta = phi
				L = d
			else:
				if d < 2.0 * r:
					continue
				L = sqrt(d * d - 4.0 * r * r)
				theta = phi + a * atan2(2.0 * r, L)
			var arc1 := fposmod(a * (theta - h0), TAU)
			var arc2 := fposmod(b * (h1 - theta), TAU)
			out.append([rad_to_deg(arc1) * a, L, rad_to_deg(arc2) * b])
	return out
