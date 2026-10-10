class_name ForzaTelemetry
extends RefCounted
## Forza "Data Out" telemetry (Forza Horizon 4/5, Forza Motorsport 7/2023)
## -> recorded passes -> a TrackSpec.
##
## Packet layout (little-endian): a 232-byte "sled" block (engine, local
## accelerations/velocities, wheels...) then the "dash" block, which starts
## with world PositionX/Y/Z and Speed. Horizon titles insert 12 extra bytes
## before the dash block. Rather than trusting the packet size alone, the
## parser checks which candidate offset gives a Speed equal to the length of
## the velocity vector, so a layout change can't silently scramble positions.
## Mirroring is detected from physics too: in a corner the car's lateral
## acceleration (local +X = right) points into the turn.

const SLED := 232

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
	return {
		"race_on": p.decode_s32(0) != 0,
		"t_ms": p.decode_u32(4),
		"rpm": p.decode_float(16),
		"accel_x": p.decode_float(20),
		"vel_local": vel,
		"yaw": p.decode_float(56),
		"pos": Vector3(p.decode_float(o), p.decode_float(o + 4), p.decode_float(o + 8)),
		"speed": p.decode_float(o + 12),
		"steer": p.decode_s8(o + 76) if p.size() > o + 76 else 0,
	}


func _detect(p: PackedByteArray, vel_len: float) -> void:
	if vel_len < 2.0:
		return  # can't tell while stationary; wait for a moving packet
	var best := -1
	var best_err := INF
	for cand in [SLED, SLED + 12]:
		if p.size() < cand + 16:
			continue
		var spd := p.decode_float(cand + 12)
		var err := absf(spd - vel_len)
		if err < best_err:
			best_err = err
			best = cand
	if best >= 0 and best_err < maxf(0.5, vel_len * 0.05):
		dash_offset = best
		format_name = "Forza Horizon 4/5" if best == SLED + 12 else ("Forza Motorsport 2023" if p.size() >= 331 else "Forza Motorsport 7")


## One recorded pass: list of positions with the lateral acceleration sign.
class Pass:
	var role := "line"            # "line" | "left" | "right"
	var points := PackedVector3Array()
	var accel_x := PackedFloat32Array()

	func add(s: Dictionary) -> void:
		var pos: Vector3 = s.pos
		if points.size() > 0 and points[points.size() - 1].distance_to(pos) < 0.5:
			return  # ~2 samples per metre is plenty
		points.append(pos)
		accel_x.append(float(s.accel_x))

	func length() -> float:
		var d := 0.0
		for i in range(1, points.size()):
			d += points[i].distance_to(points[i - 1])
		return d

	func to_dict() -> Dictionary:
		var pts := []
		for i in points.size():
			pts.append([snappedf(points[i].x, 0.01), snappedf(points[i].y, 0.01), snappedf(points[i].z, 0.01), snappedf(accel_x[i], 0.01)])
		return {"role": role, "points": pts}

	static func from_dict(d: Dictionary) -> Pass:
		var p := Pass.new()
		p.role = str(d.get("role", "line"))
		for q: Array in d.get("points", []):
			p.points.append(Vector3(float(q[0]), float(q[1]), float(q[2])))
			p.accel_x.append(float(q[3]) if q.size() > 3 else 0.0)
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
	var rights: Array = usable.filter(func(p: Pass) -> bool: return p.role == "right")
	# Reference path: the first line pass, or the first edge pass.
	var ref: Pass = (lines[0] if lines.size() > 0 else usable[0])
	var raw := PackedVector3Array()
	for v in ref.points:
		raw.append(conv.call(v))
	var path := resample(raw, 2.0)
	var closed := path.size() > 50 and path[0].distance_to(path[path.size() - 1]) < 25.0
	if closed:
		# Trim the overlap so the loop closes cleanly.
		while path.size() > 10 and path[path.size() - 1].distance_to(path[0]) < 4.0:
			path.remove_at(path.size() - 1)
	path = smooth(path, 3, closed)
	var n := path.size()
	# Edges -> centre + width: for each centre point, nearest point on each edge.
	var widths := PackedFloat32Array()
	widths.resize(n)
	widths.fill(default_width)
	var has_edges := lefts.size() > 0 and rights.size() > 0
	if has_edges:
		var lp := PackedVector3Array()
		var rp := PackedVector3Array()
		for v in (lefts[0] as Pass).points:
			lp.append(conv.call(v))
		for v in (rights[0] as Pass).points:
			rp.append(conv.call(v))
		lp = resample(lp, 1.0)
		rp = resample(rp, 1.0)
		for i in n:
			var a := _nearest(lp, path[i])
			var b := _nearest(rp, path[i])
			var w := Vector2(a.x - b.x, a.z - b.z).length() + 1.8  # car width: edges were driven by the car's centre
			if w > 4.0 and w < 60.0:
				widths[i] = w
				if lines.is_empty():
					path[i] = (a + b) * 0.5  # no line pass: use the middle of the road
		widths = _smooth_f(widths, 6, closed)
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
	var step := 4
	for i in range(0, n, step):
		var p := path[i]
		points.append([snappedf(p.x - c.x, 0.1), snappedf(p.z - c.z, 0.1), snappedf(p.y - min_y, 0.1)])
		wlist.append(snappedf(widths[i], 0.1))
	var avg_w := 0.0
	for w in widths:
		avg_w += w
	avg_w /= n
	var spec := {
		"name": name,
		"closed": closed,
		"points": points,
		"width": snappedf(avg_w, 0.1),
		"runoff": 3.0,
		"runoff_surface": "grass",
		"walls": {"left": true, "right": true, "height": 1.0},
		"kerbs": false,
		"zones": [{"name": name, "start": 0.02, "end": 0.98}] if not closed else [],
		"clips": [],
		"start": 0.0,
		"source": {"kind": "forza_telemetry", "mirrored_x": mirror, "passes": usable.size()},
	}
	if has_edges:
		spec["widths"] = wlist
	var length := 0.0
	for i in range(1, n):
		length += path[i].distance_to(path[i - 1])
	return {"ok": true, "spec": spec, "error": "", "info": {"length_m": length, "closed": closed, "mirrored": mirror,
		"width_m": avg_w, "edges": has_edges, "elevation_m": _range_y(path)}}


static func _nearest(pts: PackedVector3Array, p: Vector3) -> Vector3:
	var best := pts[0]
	var bd := INF
	for q in pts:
		var d := Vector2(q.x - p.x, q.z - p.z).length_squared()
		if d < bd:
			bd = d
			best = q
	return best


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
