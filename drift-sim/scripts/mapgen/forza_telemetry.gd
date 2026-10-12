class_name ForzaTelemetry
extends RefCounted
## Forza "Data Out" telemetry (Forza Horizon 4/5/6, Forza Motorsport 7/2023)
## -> recorded passes -> a TrackSpec.
##
## Packet layout (little-endian): a 232-byte "sled" block (engine, local
## accelerations/velocities, per-wheel data...) then the "dash" block, which
## starts with world PositionX/Y/Z and Speed. Horizon titles insert extra bytes
## before the dash block, and newer games may move it again, so the parser
## finds the block by looking for a Speed equal to the length of the velocity
## vector (known offsets first, then a scan) instead of trusting packet size.
## Mirroring is detected from physics too: in a corner the car's lateral
## acceleration (local +X = right) points into the turn.
##
## Per-wheel sled data used for the road surface:
##   WheelOnRumbleStrip (s32 x4 @116) -> kerbs
##   SurfaceRumble      (f32 x4 @148) -> rough ground (grass/dirt) vs tarmac

const SLED := 232
const KNOWN_OFFSETS := [SLED + 12, SLED]
const EDGE_OFFSET := 0.9     # car centre -> outside of the tyres when hugging an edge
const FL := 1
const FR := 2
const RL := 4
const RR := 8

var dash_offset := -1        # resolved start of PositionX
var format_name := "?"


## Parses one packet. Returns {} if it isn't usable.
func parse(p: PackedByteArray) -> Dictionary:
	if p.size() < 311:
		return {}
	var vel := Vector3(p.decode_float(32), p.decode_float(36), p.decode_float(40))
	if dash_offset < 0:
		_detect(p, vel.length())
		if dash_offset < 0:
			return {}
	var o := dash_offset
	if p.size() < o + 16:
		return {}
	var kerb := 0
	for w in 4:
		if p.decode_s32(116 + w * 4) != 0:
			kerb |= 1 << w
	var rum := PackedFloat32Array()
	for w in 4:
		var v := p.decode_float(148 + w * 4)
		rum.append(v if is_finite(v) else 0.0)
	return {
		"race_on": p.decode_s32(0) != 0,
		"t_ms": p.decode_u32(4),
		"rpm": p.decode_float(16),
		"accel_x": p.decode_float(20),
		"vel_local": vel,
		"yaw": p.decode_float(56),
		"kerb": kerb,
		"rumble": rum,   # FL, FR, RL, RR
		"pos": Vector3(p.decode_float(o), p.decode_float(o + 4), p.decode_float(o + 8)),
		"speed": p.decode_float(o + 12),
		"steer": p.decode_s8(o + 76) if p.size() > o + 76 else 0,
	}


func _detect(p: PackedByteArray, vel_len: float) -> void:
	if vel_len < 2.0:
		return  # can't tell while stationary; wait for a moving packet
	var tol := maxf(0.5, vel_len * 0.05)
	var best := -1
	for cand: int in KNOWN_OFFSETS:
		if p.size() >= cand + 16 and absf(p.decode_float(cand + 12) - vel_len) < tol and _plausible_pos(p, cand):
			best = cand
			break
	if best < 0:
		# Unknown layout (a newer game): scan the dash area for Speed.
		var best_err := INF
		for cand in range(SLED, mini(p.size() - 16, SLED + 200) + 1, 4):
			var spd := p.decode_float(cand + 12)
			var err := absf(spd - vel_len)
			if is_finite(spd) and err < tol and err < best_err and _plausible_pos(p, cand):
				best_err = err
				best = cand
	if best < 0:
		return
	dash_offset = best
	if best == SLED + 12:
		format_name = "Forza Horizon 4/5" if p.size() == 324 else "Forza Horizon (%d-byte packets)" % p.size()
	elif best == SLED:
		format_name = "Forza Motorsport 2023" if p.size() >= 331 else "Forza Motorsport 7"
	else:
		format_name = "Forza (%d-byte packets, position at byte %d)" % [p.size(), best]


static func _plausible_pos(p: PackedByteArray, o: int) -> bool:
	for k in 3:
		var v := p.decode_float(o + k * 4)
		if not is_finite(v) or absf(v) > 200000.0:
			return false
	return true


## One recorded pass: positions with lateral g and per-wheel surface data.
class Pass:
	var role := "line"            # "line" | "left" | "right"
	var points := PackedVector3Array()
	var accel_x := PackedFloat32Array()
	var kerb := PackedByteArray()          # FL/FR/RL/RR bits seen since the last point
	var rumble_l := PackedFloat32Array()   # max left-wheel surface rumble since the last point
	var rumble_r := PackedFloat32Array()
	var _pk := 0
	var _prl := 0.0
	var _prr := 0.0

	func add(s: Dictionary) -> void:
		var rum: PackedFloat32Array = s.get("rumble", PackedFloat32Array([0, 0, 0, 0]))
		_pk |= int(s.get("kerb", 0))
		_prl = maxf(_prl, (rum[0] + rum[2]) * 0.5)
		_prr = maxf(_prr, (rum[1] + rum[3]) * 0.5)
		var pos: Vector3 = s.pos
		if points.size() > 0 and points[points.size() - 1].distance_to(pos) < 0.5:
			return  # ~2 samples per metre is plenty
		points.append(pos)
		accel_x.append(float(s.accel_x))
		kerb.append(_pk)
		rumble_l.append(_prl)
		rumble_r.append(_prr)
		_pk = 0
		_prl = 0.0
		_prr = 0.0

	func length() -> float:
		var d := 0.0
		for i in range(1, points.size()):
			d += points[i].distance_to(points[i - 1])
		return d

	func kerb_count() -> int:
		var c := 0
		for k in kerb:
			if k != 0:
				c += 1
		return c

	func to_dict() -> Dictionary:
		var pts := []
		for i in points.size():
			pts.append([snappedf(points[i].x, 0.01), snappedf(points[i].y, 0.01), snappedf(points[i].z, 0.01), snappedf(accel_x[i], 0.01),
				int(kerb[i]), snappedf(rumble_l[i], 0.001), snappedf(rumble_r[i], 0.001)])
		return {"role": role, "points": pts}

	static func from_dict(d: Dictionary) -> Pass:
		var p := Pass.new()
		p.role = str(d.get("role", "line"))
		for q: Array in d.get("points", []):
			p.points.append(Vector3(float(q[0]), float(q[1]), float(q[2])))
			p.accel_x.append(float(q[3]) if q.size() > 3 else 0.0)
			p.kerb.append(int(q[4]) if q.size() > 4 else 0)
			p.rumble_l.append(float(q[5]) if q.size() > 5 else 0.0)
			p.rumble_r.append(float(q[6]) if q.size() > 6 else 0.0)
		return p


## Resamples a polyline to even spacing (metres).
static func resample(pts: PackedVector3Array, step: float) -> PackedVector3Array:
	var out := PackedVector3Array()
	if pts.size() < 2:
		return pts
	out.append(pts[0])
	var carry := 0.0
	for i in range(1, pts.size()):
		var a := pts[i - 1]
		var b := pts[i]
		var seg := a.distance_to(b)
		var d := step - carry
		while d <= seg:
			out.append(a.lerp(b, d / seg))
			d += step
		carry = seg - (d - step)
	return out


static func smooth(pts: PackedVector3Array, radius: int, closed: bool) -> PackedVector3Array:
	var n := pts.size()
	var out := PackedVector3Array()
	out.resize(n)
	for i in n:
		var acc := Vector3.ZERO
		var cnt := 0
		for k in range(-radius, radius + 1):
			var j := i + k
			if closed:
				j = posmod(j, n)
			elif j < 0 or j >= n:
				continue
			acc += pts[j]
			cnt += 1
		out[i] = acc / cnt
	return out


## Signed turn (+ = right in Drift Lab's frame) at each point of a path
## expressed in our (x, z) plane.
static func _turn_signs(pts: PackedVector3Array) -> PackedFloat32Array:
	var out := PackedFloat32Array()
	out.resize(pts.size())
	for i in range(1, pts.size() - 1):
		var t0 := pts[i] - pts[i - 1]
		var t1 := pts[i + 1] - pts[i]
		out[i] = t0.x * t1.z - t0.z * t1.x
	return out


## Flat right-hand normal of a path at each point (same convention as TrackSpec).
static func _rights(pts: PackedVector3Array, closed: bool) -> PackedVector3Array:
	var n := pts.size()
	var out := PackedVector3Array()
	out.resize(n)
	for i in n:
		var a := pts[posmod(i - 1, n)] if (closed or i > 0) else pts[i]
		var b := pts[(i + 1) % n] if (closed or i < n - 1) else pts[i]
		var t := Vector3(b.x - a.x, 0.0, b.z - a.z)
		t = t.normalized() if t.length() > 1e-5 else Vector3.FORWARD
		out[i] = Vector3(-t.z, 0.0, t.x)
	return out


## True if Forza's world needs X mirrored to match Drift Lab's handedness.
static func needs_mirror(passes: Array) -> bool:
	var score := 0.0
	for p: Pass in passes:
		var pts := p.points
		var turns := _turn_signs(pts)
		for i in range(1, pts.size() - 1):
			# Only clear corners with real lateral g.
			if absf(p.accel_x[i]) > 2.0 and absf(turns[i]) > 1e-4:
				score += signf(turns[i]) * signf(p.accel_x[i])
	return score < 0.0


## Car-centre points of an edge-hugging pass, minus points where the outer
## wheels were off the tarmac (outer wheels rumbling much more than the inner
## ones). The push out to the real edge happens later, relative to the road.
## Returns {points, dropped}.
static func _edge_points(p: Pass, side: float, conv: Callable) -> Dictionary:
	var outer := p.rumble_l if side < 0.0 else p.rumble_r
	var inner := p.rumble_r if side < 0.0 else p.rumble_l
	var peak := 0.0
	for v in outer:
		peak = maxf(peak, v)
	var off := PackedByteArray()
	off.resize(p.points.size())
	var n_off := 0
	if peak > 1e-4:
		for i in p.points.size():
			if outer[i] > inner[i] * 2.5 + peak * 0.08:
				off[i] = 1
				n_off += 1
	# If most of the pass "looks" off-road the road itself is just rough - keep it all.
	var filter := n_off < p.points.size() * 0.4
	var out := PackedVector3Array()
	for i in p.points.size():
		if not (filter and off[i] == 1):
			out.append(conv.call(p.points[i]))
	return {"points": out, "dropped": n_off if filter else 0}


## Splits a path (~2 m spacing) where the car reversed (a cusp) or drove back
## over road it had already covered, and returns the pieces (>= 10 points).
static func _split_clean(pts: PackedVector3Array) -> Array:
	var n := pts.size()
	var cut := PackedByteArray()
	cut.resize(n)
	for i in range(1, n - 1):
		var a := Vector2(pts[i].x - pts[i - 1].x, pts[i].z - pts[i - 1].z)
		var b := Vector2(pts[i + 1].x - pts[i].x, pts[i + 1].z - pts[i].z)
		if a.length() > 0.01 and b.length() > 0.01 and a.normalized().dot(b.normalized()) < -0.3:
			cut[i] = 1
	var cell := 8.0
	var grid := {}
	for i in n:
		var key := Vector2i(floori(pts[i].x / cell), floori(pts[i].z / cell))
		for dx in range(-1, 2):
			for dz in range(-1, 2):
				for j: int in grid.get(key + Vector2i(dx, dz), []):
					if i - j > 40 and not (j < 5 and i > n - 6) and absf(pts[i].y - pts[j].y) < 3.0 \
							and Vector2(pts[i].x - pts[j].x, pts[i].z - pts[j].z).length() < 4.0:
						cut[i] = 1
		if not grid.has(key):
			grid[key] = []
		(grid[key] as Array).append(i)
	var pieces := []
	var cur := PackedVector3Array()
	for i in n:
		if cut[i] == 1:
			if cur.size() >= 10:
				pieces.append(cur)
			cur = PackedVector3Array()
			continue
		cur.append(pts[i])
	if cur.size() >= 10:
		pieces.append(cur)
	return pieces


static func _grid(pts: PackedVector3Array, cell: float) -> Dictionary:
	var g := {}
	for i in pts.size():
		var key := Vector2i(floori(pts[i].x / cell), floori(pts[i].z / cell))
		if not g.has(key):
			g[key] = []
		(g[key] as Array).append(i)
	return g


## Nearest edge point beside reference point p (in the slice across the road
## at p, on the given side, at about the same height). -1 if none.
static func _edge_at(pts: PackedVector3Array, grid: Dictionary, p: Vector3, r: Vector3, side: float) -> int:
	var t := Vector3(r.z, 0.0, -r.x)
	var key := Vector2i(floori(p.x / 8.0), floori(p.z / 8.0))
	var best := -1
	var best_l := INF
	for dx in range(-3, 4):
		for dz in range(-3, 4):
			for j: int in grid.get(key + Vector2i(dx, dz), []):
				var d := pts[j] - p
				if absf(d.dot(t)) > 2.5 or absf(d.y) > 4.0:
					continue
				var l := d.dot(r) * side   # distance out to this side
				if l < -1.5 or l > 20.0:
					continue
				if l < best_l:
					best_l = l
					best = j
	return best


## Nearest point index on `pts` to `p`, searching a window after `hint`
## (or everything when hint < 0). Window search stops a hairpin from matching
## the other side of the hill.
static func _match(pts: PackedVector3Array, p: Vector3, hint: int, closed: bool) -> int:
	var n := pts.size()
	var lo := 0
	var hi := n - 1
	if hint >= 0:
		lo = hint - 20
		hi = hint + 70
	var best := -1
	var bd := INF
	for k in range(lo, hi + 1):
		var j := k
		if closed:
			j = posmod(k, n)
		elif k < 0 or k >= n:
			continue
		var q := pts[j]
		var d := Vector2(q.x - p.x, q.z - p.z).length_squared()
		if d < bd:
			bd = d
			best = j
	return best


## Builds a TrackSpec from recorded passes.
## Returns {ok, spec, info, error}.
static func build_track(passes: Array, name: String, default_width := 12.0) -> Dictionary:
	var usable: Array = passes.filter(func(p: Pass) -> bool: return p.points.size() >= 20)
	if usable.is_empty():
		return {"ok": false, "error": "Record at least one pass (drive the zone with Data Out on)."}
	var mirror := needs_mirror(usable)
	var conv := func(v: Vector3) -> Vector3: return Vector3(-v.x if mirror else v.x, v.y, v.z)
	var lines: Array = usable.filter(func(p: Pass) -> bool: return p.role == "line")
	var lefts: Array = usable.filter(func(p: Pass) -> bool: return p.role == "left")
	var rights_p: Array = usable.filter(func(p: Pass) -> bool: return p.role == "right")
	# Reference path: the longest clean stretch of any pass (passes are split
	# where the car reversed or drove back over itself), preferring a line pass.
	var best_any := PackedVector3Array()
	var best_line := PackedVector3Array()
	for p: Pass in usable:
		var raw := PackedVector3Array()
		for v in p.points:
			raw.append(conv.call(v))
		for piece: PackedVector3Array in _split_clean(resample(raw, 2.0)):
			if piece.size() > best_any.size():
				best_any = piece
			if p.role == "line" and piece.size() > best_line.size():
				best_line = piece
	var path := best_line if best_line.size() >= best_any.size() * 0.8 else best_any
	if path.size() < 10:
		return {"ok": false, "error": "No clean stretch of road in the recording (every pass turns back on itself?)."}
	var closed := path.size() > 50 and path[0].distance_to(path[path.size() - 1]) < 25.0
	if closed:
		# Trim the overlap so the loop closes cleanly.
		while path.size() > 10 and path[path.size() - 1].distance_to(path[0]) < 4.0:
			path.remove_at(path.size() - 1)
	path = smooth(path, 3, closed)
	var n := path.size()
	var widths := PackedFloat32Array()
	widths.resize(n)
	widths.fill(default_width)
	var banks := PackedFloat32Array()
	banks.resize(n)
	var has_edges := lefts.size() > 0 and rights_p.size() > 0
	var dropped := 0
	var edge_cover := 0.0
	var trimmed_m := 0.0
	if has_edges:
		# All passes for a side are pooled, so partial and overlapping passes
		# are fine: each slice across the road takes the nearest edge point.
		var lp := PackedVector3Array()
		var rp := PackedVector3Array()
		for p: Pass in lefts:
			var e := _edge_points(p, -1.0, conv)
			lp.append_array(e.points)
			dropped += int(e.dropped)
		for p: Pass in rights_p:
			var e := _edge_points(p, 1.0, conv)
			rp.append_array(e.points)
			dropped += int(e.dropped)
		if lp.size() < 2 or rp.size() < 2:
			return {"ok": false, "error": "An edge pass has no usable points (was the whole pass off the road?). Delete it and record it again."}
		var lg := _grid(lp, 8.0)
		var rg := _grid(rp, 8.0)
		var refr := _rights(path, closed)
		var delta := PackedVector3Array()
		delta.resize(n)
		var valid := PackedByteArray()
		valid.resize(n)
		for i in n:
			var il := _edge_at(lp, lg, path[i], refr[i], -1.0)
			var ir := _edge_at(rp, rg, path[i], refr[i], 1.0)
			if il < 0 or ir < 0:
				continue
			var ll := (lp[il] - path[i]).dot(refr[i]) - EDGE_OFFSET   # left road edge (negative)
			var lr := (rp[ir] - path[i]).dot(refr[i]) + EDGE_OFFSET   # right road edge
			var w := lr - ll
			if w < 4.0 or w > 40.0:
				continue
			valid[i] = 1
			delta[i] = refr[i] * ((ll + lr) * 0.5) + Vector3.UP * ((lp[il].y + rp[ir].y) * 0.5 - path[i].y)
			widths[i] = w
			banks[i] = clampf((rp[ir].y - lp[il].y) / w, -0.15, 0.15)
		var first := valid.find(1)
		var last := valid.rfind(1)
		if first < 0 or last - first < 3:
			has_edges = false
			widths.fill(default_width)
			banks.fill(0.0)
		else:
			if not closed and (first > 3 or last < n - 4):
				# Keep only the part both edges cover (plus a few metres).
				var a := maxi(first - 2, 0)
				var b := mini(last + 2, n - 1)
				for i in range(1, a + 1):
					trimmed_m += path[i].distance_to(path[i - 1])
				for i in range(b + 1, n):
					trimmed_m += path[i].distance_to(path[i - 1])
				path = path.slice(a, b + 1)
				widths = widths.slice(a, b + 1)
				banks = banks.slice(a, b + 1)
				delta = delta.slice(a, b + 1)
				valid = valid.slice(a, b + 1)
				n = path.size()
			var nv := 0
			for v in valid:
				nv += int(v)
			edge_cover = float(nv) / n
			_fill_gaps(valid, delta, widths, banks, closed)
			for i in n:
				path[i] += delta[i]
			path = smooth(path, 2, closed)
			widths = _smooth_f(widths, 4, closed)
			banks = _smooth_f(banks, 4, closed)
	# Kerbs: wherever Forza reported a wheel on a rumble strip.
	var refr2 := _rights(path, closed)
	var cum := PackedFloat32Array([0.0])
	for i in range(1, n):
		cum.append(cum[i - 1] + path[i].distance_to(path[i - 1]))
	var total := cum[n - 1] + (path[n - 1].distance_to(path[0]) if closed else 0.0)
	var kerb_d := {"left": [], "right": []}
	for p: Pass in usable:
		for k in p.points.size():
			var bits := int(p.kerb[k])
			if bits == 0:
				continue
			var q: Vector3 = conv.call(p.points[k])
			var i := _match(path, q, -1, closed)
			var lat := (q - path[i]).dot(refr2[i])
			var side := ""
			if absf(lat) > widths[i] * 0.25:
				side = "right" if lat > 0.0 else "left"
			elif bits & (FL | RL) and not bits & (FR | RR):
				side = "left"
			elif bits & (FR | RR) and not bits & (FL | RL):
				side = "right"
			else:
				side = "right" if lat > 0.0 else "left"
			(kerb_d[side] as Array).append(cum[i])
	var kerb_ranges := []
	for side: String in ["left", "right"]:
		for r: Vector2 in _merge_ranges(kerb_d[side], 8.0, 2.0, total):
			kerb_ranges.append({"side": side, "start": snappedf(r.x / total, 0.0001), "end": snappedf(r.y / total, 0.0001)})
	# Centre the track near the origin, keep real elevation (relative).
	var c := Vector3.ZERO
	var min_y := INF
	for p in path:
		c += p
		min_y = minf(min_y, p.y)
	c /= n
	# Control points every ~8 m keep the spline faithful without bloat.
	var points := []
	var wlist := []
	var blist := []
	var step := 4
	for i in range(0, n, step):
		var p := path[i]
		points.append([snappedf(p.x - c.x, 0.1), snappedf(p.z - c.z, 0.1), snappedf(p.y - min_y, 0.05)])
		wlist.append(snappedf(widths[i], 0.1))
		blist.append(snappedf(banks[i], 0.001))
	var avg_w := 0.0
	for w in widths:
		avg_w += w
	avg_w /= n
	var spec := {
		"name": name,
		"closed": closed,
		"points": points,
		"width": snappedf(avg_w, 0.1),
		"runoff": 0.5,   # walls just off the recorded edge, like a touge guardrail
		"runoff_surface": "grass",
		"walls": {"left": true, "right": true, "height": 1.0},
		"kerbs": not kerb_ranges.is_empty(),
		"zones": [{"name": name, "start": 0.02, "end": 0.98}] if not closed else [],
		"clips": [],
		"start": 0.0,
		"source": {"kind": "forza_telemetry", "mirrored_x": mirror, "passes": usable.size()},
	}
	if not kerb_ranges.is_empty():
		spec["kerb_ranges"] = kerb_ranges
	if has_edges:
		spec["widths"] = wlist
		spec["banks"] = blist
	var length := 0.0
	for i in range(1, n):
		length += path[i].distance_to(path[i - 1])
	var max_bank := 0.0
	for b in banks:
		max_bank = maxf(max_bank, absf(b))
	return {"ok": true, "spec": spec, "error": "", "info": {"length_m": length, "closed": closed, "mirrored": mirror,
		"width_m": avg_w, "edges": has_edges, "edge_cover": edge_cover, "elevation_m": _range_y(path),
		"kerbs": kerb_ranges.size(), "offroad_dropped": dropped, "max_bank_pct": max_bank * 100.0, "trimmed_m": trimmed_m,
		"one_edge_only": (lefts.size() > 0) != (rights_p.size() > 0)}}


## Fills invalid entries by linear interpolation between valid neighbours
## (holding the nearest value at the ends of an open track).
static func _fill_gaps(valid: PackedByteArray, delta: PackedVector3Array, widths: PackedFloat32Array, banks: PackedFloat32Array, closed: bool) -> void:
	var n := valid.size()
	var idx := []
	for i in n:
		if valid[i] == 1:
			idx.append(i)
	var m := idx.size()
	for g in m:
		var a: int = idx[g]
		var b: int
		var span: int
		if g + 1 < m:
			b = idx[g + 1]
			span = b - a
		elif closed:
			b = idx[0]
			span = b + n - a
		else:
			break
		for k in range(1, span):
			var i := (a + k) % n
			var t := float(k) / span
			delta[i] = delta[a].lerp(delta[b], t)
			widths[i] = lerpf(widths[a], widths[b], t)
			banks[i] = lerpf(banks[a], banks[b], t)
	if not closed:
		var first: int = idx[0]
		var last: int = idx[m - 1]
		for i in first:
			delta[i] = delta[first]
			widths[i] = widths[first]
			banks[i] = banks[first]
		for i in range(last + 1, n):
			delta[i] = delta[last]
			widths[i] = widths[last]
			banks[i] = banks[last]


## Distances along the track -> merged [start, end] ranges (metres).
static func _merge_ranges(ds: Array, gap: float, pad: float, total: float) -> Array:
	var out := []
	if ds.is_empty():
		return out
	ds.sort()
	var s: float = ds[0]
	var e: float = ds[0]
	for d: float in ds:
		if d - e > gap:
			out.append(Vector2(maxf(s - pad, 0.0), minf(e + pad, total)))
			s = d
		e = d
	out.append(Vector2(maxf(s - pad, 0.0), minf(e + pad, total)))
	return out


static func _smooth_f(a: PackedFloat32Array, r: int, closed: bool) -> PackedFloat32Array:
	var n := a.size()
	var out := PackedFloat32Array()
	out.resize(n)
	for i in n:
		var acc := 0.0
		var cnt := 0
		for k in range(-r, r + 1):
			var j := i + k
			if closed:
				j = posmod(j, n)
			elif j < 0 or j >= n:
				continue
			acc += a[j]
			cnt += 1
		out[i] = acc / cnt
	return out


static func _range_y(pts: PackedVector3Array) -> float:
	var lo := INF
	var hi := -INF
	for p in pts:
		lo = minf(lo, p.y)
		hi = maxf(hi, p.y)
	return hi - lo
