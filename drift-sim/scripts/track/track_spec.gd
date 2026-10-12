class_name TrackSpec
extends RefCounted
## Track description format (JSON) + spline sampling + validation.
##
## {
##   "name": "My Track", "author": "", "description": "",
##   "closed": true,
##   "points": [[x, z], [x, z, elevation], ...],   // centreline control points, metres
##   "width": 12.0, "widths": [optional per control point],
##   "banks": [optional per control point: crossfall, right edge height minus
##             left edge height divided by width (0.05 = 5%, right side higher)],
##   "kerb_ranges": [optional {"side": "left"|"right", "start": 0.1, "end": 0.12}]
##             explicit kerbs (track fractions); without it kerbs go on corners
##   "runoff": 4.0, "runoff_surface": "grass" | "asphalt" | "gravel" | "dirt",
##   "walls": {"left": true, "right": true, "height": 1.2},
##   "kerbs": true,
##   "pads": [{"points": [[x, z], ...], "surface": "asphalt", "y": 0.0}],
##   "zones": [{"name": "Zone 1", "start": 0.10, "end": 0.32, "target": 12000}],
##   "clips": [{"t": 0.2, "side": "left" | "right", "radius": 2.0}],
##   "start": 0.0,
##   "scenery": {"seed": 1, "density": 1.0, "time_of_day": "day" | "dusk" | "night"}
## }

const DEFAULTS := {
	"name": "Untitled Track",
	"closed": true,
	"width": 12.0,
	"runoff": 4.0,
	"runoff_surface": "grass",
	"walls": {"left": true, "right": true, "height": 1.2},
	"kerbs": true,
	"pads": [],
	"zones": [],
	"clips": [],
	"start": 0.0,
	"scenery": {"seed": 1, "density": 1.0, "time_of_day": "day"},
}


static func normalized(spec: Dictionary) -> Dictionary:
	var s := DEFAULTS.duplicate(true)
	for k: String in spec:
		if s.get(k) is Dictionary and spec[k] is Dictionary:
			(s[k] as Dictionary).merge(spec[k], true)
		else:
			s[k] = spec[k]
	return s


static func control_points(spec: Dictionary) -> PackedVector3Array:
	var out := PackedVector3Array()
	for p: Variant in spec.get("points", []):
		if p is Array and (p as Array).size() >= 2:
			var y := float(p[2]) if (p as Array).size() >= 3 else 0.0
			out.append(Vector3(float(p[0]), y, float(p[1])))
	return out


static func _catmull(p0: Vector3, p1: Vector3, p2: Vector3, p3: Vector3, t: float) -> Vector3:
	# Centripetal Catmull-Rom (alpha = 0.5): no cusps or self-loops on tight points.
	var t0 := 0.0
	var t1 := t0 + maxf(pow(p0.distance_to(p1), 0.5), 1e-4)
	var t2 := t1 + maxf(pow(p1.distance_to(p2), 0.5), 1e-4)
	var t3 := t2 + maxf(pow(p2.distance_to(p3), 0.5), 1e-4)
	var tt := lerpf(t1, t2, t)
	var a1 := p0 * (t1 - tt) / (t1 - t0) + p1 * (tt - t0) / (t1 - t0)
	var a2 := p1 * (t2 - tt) / (t2 - t1) + p2 * (tt - t1) / (t2 - t1)
	var a3 := p2 * (t3 - tt) / (t3 - t2) + p3 * (tt - t2) / (t3 - t2)
	var b1 := a1 * (t2 - tt) / (t2 - t0) + a2 * (tt - t0) / (t2 - t0)
	var b2 := a2 * (t3 - tt) / (t3 - t1) + a3 * (tt - t1) / (t3 - t1)
	return b1 * (t2 - tt) / (t2 - t1) + b2 * (tt - t1) / (t2 - t1)


## Samples the centreline at ~`spacing` metres. Returns a Dictionary with
## pos, tangent, right (PackedVector3Array), width, dist, curvature
## (PackedFloat32Array), length, closed.
static func sample(spec: Dictionary, spacing := 2.0) -> Dictionary:
	var cp := control_points(spec)
	var closed := bool(spec.get("closed", true))
	var n := cp.size()
	var widths: Array = spec.get("widths", [])
	var base_w := float(spec.get("width", 12.0))
	var banks: Array = spec.get("banks", [])
	var dense := PackedVector3Array()
	var dense_w := PackedFloat32Array()
	var dense_b := PackedFloat32Array()
	if n < 2:
		return {"pos": dense, "length": 0.0, "closed": closed}
	var seg_count := n if closed else n - 1
	for i in seg_count:
		var p0 := cp[(i - 1 + n) % n] if (closed or i > 0) else cp[0] * 2.0 - cp[1]
		var p1 := cp[i]
		var p2 := cp[(i + 1) % n]
		var p3: Vector3
		if closed or i + 2 < n:
			p3 = cp[(i + 2) % n]
		else:
			p3 = p2 * 2.0 - p1
		var w1 := float(widths[i]) if i < widths.size() else base_w
		var w2 := float(widths[(i + 1) % n]) if (i + 1) % n < widths.size() else base_w
		var b1 := float(banks[i]) if i < banks.size() else 0.0
		var b2 := float(banks[(i + 1) % n]) if (i + 1) % n < banks.size() else 0.0
		var steps := maxi(4, int(p1.distance_to(p2) / (spacing * 0.5)))
		for k in steps:
			var t := float(k) / steps
			dense.append(_catmull(p0, p1, p2, p3, t))
			dense_w.append(lerpf(w1, w2, t))
			dense_b.append(lerpf(b1, b2, t))
	if not closed:
		dense.append(cp[n - 1])
		dense_w.append(float(widths[n - 1]) if n - 1 < widths.size() else base_w)
		dense_b.append(float(banks[n - 1]) if n - 1 < banks.size() else 0.0)

	# Resample uniformly by arc length.
	var cum := PackedFloat32Array([0.0])
	var m := dense.size()
	var limit := m if closed else m - 1
	for i in limit:
		cum.append(cum[cum.size() - 1] + dense[i].distance_to(dense[(i + 1) % m]))
	var total := cum[cum.size() - 1]
	var count := maxi(int(total / spacing), 8)
	var pos := PackedVector3Array()
	var width := PackedFloat32Array()
	var bank := PackedFloat32Array()
	var dist := PackedFloat32Array()
	var j := 0
	var last := count if closed else count + 1
	for k in last:
		var d := total * k / count
		while j < cum.size() - 2 and cum[j + 1] < d:
			j += 1
		var seg := maxf(cum[j + 1] - cum[j], 1e-6)
		var t := clampf((d - cum[j]) / seg, 0.0, 1.0)
		pos.append(dense[j].lerp(dense[(j + 1) % m], t))
		width.append(lerpf(dense_w[j], dense_w[(j + 1) % m], t))
		bank.append(clampf(lerpf(dense_b[j], dense_b[(j + 1) % m], t), -0.15, 0.15))
		dist.append(d)
	var cnt := pos.size()
	var tangent := PackedVector3Array()
	var right := PackedVector3Array()
	tangent.resize(cnt)
	right.resize(cnt)
	for i in cnt:
		var a := pos[(i - 1 + cnt) % cnt] if (closed or i > 0) else pos[i]
		var b := pos[(i + 1) % cnt] if (closed or i < cnt - 1) else pos[i]
		var tng := (b - a)
		tng = tng.normalized() if tng.length() > 1e-5 else Vector3.FORWARD
		tangent[i] = tng
		var flat := Vector3(tng.x, 0, tng.z).normalized()
		right[i] = Vector3(-flat.z, 0, flat.x)  # +90deg in XZ (to the right when facing tangent)
	var curvature := PackedFloat32Array()
	curvature.resize(cnt)
	for i in cnt:
		if not closed and (i == 0 or i == cnt - 1):
			curvature[i] = 0.0
			continue
		var t0 := tangent[(i - 1 + cnt) % cnt]
		var t1 := tangent[(i + 1) % cnt]
		var cross := t0.x * t1.z - t0.z * t1.x
		curvature[i] = asin(clampf(cross, -1.0, 1.0)) / (2.0 * spacing)  # +: turning right
	return {"pos": pos, "tangent": tangent, "right": right, "width": width, "bank": bank, "dist": dist,
		"curvature": curvature, "length": total, "closed": closed, "spacing": total / count}


## Transform at track fraction t (0..1) facing along the track.
static func transform_at(samples: Dictionary, t: float, lift := 0.6) -> Transform3D:
	var pos: PackedVector3Array = samples.pos
	var n := pos.size()
	var i := clampi(int(fposmod(t, 1.0) * n), 0, n - 1)
	var tng: Vector3 = samples.tangent[i]
	var basis := Basis.looking_at(tng, Vector3.UP)
	return Transform3D(basis, pos[i] + Vector3.UP * lift)


## Returns {errors: [], warnings: [], stats: {}}.
static func validate(spec_in: Dictionary) -> Dictionary:
	var spec := normalized(spec_in)
	var errors := PackedStringArray()
	var warnings := PackedStringArray()
	var cp := control_points(spec)
	var closed := bool(spec.closed)
	var pads: Array = spec.get("pads", [])
	if cp.size() < (4 if closed else 2) and pads.is_empty():
		errors.append("Need at least %d centreline points (have %d)." % [4 if closed else 2, cp.size()])
		return {"errors": errors, "warnings": warnings, "stats": {}}
	var w := float(spec.width)
	if w < 5.0 or w > 40.0:
		errors.append("Track width %.1f m out of range (5-40 m)." % w)
	var stats := {}
	if cp.size() >= 2:
		var s := sample(spec, 2.0)
		var length := float(s.length)
		stats.length_m = snappedf(length, 1.0)
		if length < 150.0:
			errors.append("Track is only %d m long (min 150 m)." % length)
		if length > 25000.0:
			errors.append("Track is %d m long (max 25 km)." % length)
		var curv: PackedFloat32Array = s.curvature
		var min_r := INF
		var tight_at := -1.0
		for i in curv.size():
			var k := absf(curv[i])
			if k > 1e-5 and 1.0 / k < min_r:
				min_r = 1.0 / k
				tight_at = float(s.dist[i]) / length
		stats.min_radius_m = snappedf(min_r, 0.1) if min_r < INF else -1
		if min_r < w * 0.5 + 1.0:
			warnings.append("Very tight corner (radius %.1f m) at %.0f%% - the inside edge will fold over itself. Widen the corner or reduce width." % [min_r, tight_at * 100.0])
		var overlap := _find_overlap(s, -0.5)
		if overlap >= 0.0:
			errors.append("Track sections overlap near %.0f%% of the lap (roads cross or run into each other)." % (overlap * 100.0))
		elif _find_overlap(s, float(spec.runoff)) >= 0.0:
			warnings.append("Some sections run close together (hairpins); walls and run-off between them are trimmed so they don't cut into the other road.")
		if closed and cp[0].distance_to(cp[cp.size() - 1]) < 1.0:
			warnings.append("First and last points are on top of each other; for closed tracks don't repeat the first point.")
		var ys := []
		for p in cp:
			ys.append(p.y)
		stats.elevation_change_m = snappedf(ys.max() - ys.min(), 0.1)
		var max_grade := 0.0
		var pos: PackedVector3Array = s.pos
		for i in range(1, pos.size()):
			var dxz := Vector2(pos[i].x - pos[i - 1].x, pos[i].z - pos[i - 1].z).length()
			if dxz > 0.1:
				max_grade = maxf(max_grade, absf(pos[i].y - pos[i - 1].y) / dxz)
		stats.max_grade_pct = snappedf(max_grade * 100.0, 0.1)
		if max_grade > 0.25:
			warnings.append("Grade of %.0f%% is steeper than any real road (keep under ~20%%)." % (max_grade * 100.0))
	for z: Variant in spec.zones:
		if not (z is Dictionary) or not z.has("start") or not z.has("end"):
			errors.append("Each zone needs start and end (0-1).")
			continue
		if float(z.start) < 0.0 or float(z.start) > 1.0 or float(z.end) < 0.0 or float(z.end) > 1.0:
			errors.append("Zone '%s' start/end must be fractions 0-1." % z.get("name", "?"))
	for c: Variant in spec.clips:
		if not (c is Dictionary) or not c.has("t") or not ["left", "right"].has(str(c.get("side", ""))):
			errors.append("Each clip needs t (0-1) and side 'left'/'right'.")
	for p: Variant in pads:
		if not (p is Dictionary) or not (p.get("points") is Array) or (p.points as Array).size() < 3:
			errors.append("Each pad needs a 'points' polygon with >= 3 points.")
	return {"errors": errors, "warnings": warnings, "stats": stats}


## Returns the track fraction where two non-adjacent sections come closer than
## their combined half-widths, or -1.
static func _find_overlap(s: Dictionary, runoff: float) -> float:
	var pos: PackedVector3Array = s.pos
	var width: PackedFloat32Array = s.width
	var dist: PackedFloat32Array = s.dist
	var length := float(s.length)
	var cell := 16.0
	var grid := {}
	for i in pos.size():
		var key := Vector2i(floori(pos[i].x / cell), floori(pos[i].z / cell))
		if not grid.has(key):
			grid[key] = []
		grid[key].append(i)
	for i in pos.size():
		var key := Vector2i(floori(pos[i].x / cell), floori(pos[i].z / cell))
		for dx in range(-2, 3):
			for dz in range(-2, 3):
				var k2 := key + Vector2i(dx, dz)
				if not grid.has(k2):
					continue
				for j: int in grid[k2]:
					if j <= i:
						continue
					var along := absf(dist[j] - dist[i])
					if s.closed:
						along = minf(along, length - along)
					var need := (width[i] + width[j]) * 0.5 + minf(runoff, 2.0)
					if along < need * 3.0 + 10.0:
						continue
					# Bridges/overpasses: allow if vertically separated.
					if absf(pos[i].y - pos[j].y) > 5.0:
						continue
					var d := Vector2(pos[i].x - pos[j].x, pos[i].z - pos[j].z).length()
					if d < need:
						return dist[i] / length
	return -1.0
