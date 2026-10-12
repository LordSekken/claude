class_name ForzaSession
extends RefCounted
## Builds a track from a whole raw Forza session (ForzaRawLog): laps, wipeouts,
## rewinds and edge laps all count. Instead of trusting one careful pass, it
## learns the road from where every wheel touched and what it touched:
##
##   - wheel contact points come from position + heading (yaw, calibrated
##     against the direction of travel) and the car's wheel layout;
##   - each wheel's SurfaceRumble says what it was on. Forza reports a small
##     set of values: 0 on tarmac, a middle value on rough ground (grass, dirt,
##     gravel) and the highest on rumble strips. Classes are taken relative to
##     the largest value seen, so they don't depend on the exact numbers;
##   - for every metre of road (a slice across a reference lap) the tarmac
##     contacts give the road edges, rumble-strip contacts give kerbs, rough
##     contacts beyond the edge give the shoulder width, and the car's height
##     against its position across the road gives elevation and camber.

const HALF_TRACK := 0.8      # wheel offset from the car's centre line (m)
const HALF_BASE := 1.3       # axle offset from the car's centre (m)
const BIN := 0.25            # lateral histogram resolution (m)
const LMAX := 25.0           # lateral search range either side of the reference (m)
const TELEPORT := 15.0       # jump between samples that means a rewind/reset (m)
const TARMAC := 0
const ROUGH := 1
const KERB := 2


## packets: [{t_ms, data}] from ForzaRawLog.read. Returns {ok, spec, info, error}.
static func build(packets: Array, name: String) -> Dictionary:
	var tele := ForzaTelemetry.new()
	# --- Decode -----------------------------------------------------------------
	var pos := PackedVector3Array()
	var yaw := PackedFloat32Array()
	var vel_x := PackedFloat32Array()
	var vel_z := PackedFloat32Array()
	var surf := PackedFloat32Array()   # 4 per sample
	var seg := PackedInt32Array()      # segment id (split at rewinds/teleports)
	var laps := PackedInt32Array()
	var passes := []
	var cur := ForzaTelemetry.Pass.new()
	var seg_id := 0
	var surf_max := 0.0
	for pk: Dictionary in packets:
		var b: PackedByteArray = pk.data
		var s := tele.parse(b)
		if s.is_empty() or not bool(s.race_on) or float(s.speed) < 1.0:
			continue
		var p: Vector3 = s.pos
		if p.length() < 0.5:
			continue
		if pos.size() > 0 and p.distance_to(pos[pos.size() - 1]) > TELEPORT:
			seg_id += 1
			passes.append(cur)
			cur = ForzaTelemetry.Pass.new()
		cur.add(s)
		pos.append(p)
		yaw.append(b.decode_float(56))
		vel_x.append(b.decode_float(32))
		vel_z.append(b.decode_float(40))
		for w in 4:
			var v := b.decode_float(148 + w * 4)
			v = v if is_finite(v) else 0.0
			surf.append(v)
			surf_max = maxf(surf_max, v)
		seg.append(seg_id)
		laps.append(b.decode_u16(tele.dash_offset + 68) if b.size() > tele.dash_offset + 70 else 0)
	passes.append(cur)
	var n_s := pos.size()
	if n_s < 300:
		return {"ok": false, "error": "Not enough driving in the session (%d usable samples)." % n_s}
	var mirror := ForzaTelemetry.needs_mirror(passes.filter(func(q: ForzaTelemetry.Pass) -> bool: return q.points.size() >= 20))
	var conv := func(v: Vector3) -> Vector3: return Vector3(-v.x if mirror else v.x, v.y, v.z)

	# --- Heading calibration: world forward = (sin(s*yaw + c), 0, cos(s*yaw + c)) ---
	var cal := _calibrate_yaw(pos, yaw, vel_x, vel_z, seg)
	if cal.is_empty():
		return {"ok": false, "error": "Couldn't match the car's heading to its movement - is this a Forza session?"}
	var ys: float = cal.sign
	var yc: float = cal.offset
	var rs: float = cal.right_sign

	# --- Reference path: the longest clean stretch of car positions ----------------
	var best := PackedVector3Array()
	var start_i := 0
	while start_i < n_s:
		var end_i := start_i
		while end_i + 1 < n_s and seg[end_i + 1] == seg[start_i]:
			end_i += 1
		var pts := PackedVector3Array()
		for i in range(start_i, end_i + 1):
			pts.append(conv.call(pos[i]))
		for piece: PackedVector3Array in ForzaTelemetry._split_clean(ForzaTelemetry.resample(pts, 2.0)):
			if piece.size() > best.size():
				best = piece
		start_i = end_i + 1
	if best.size() < 30:
		return {"ok": false, "error": "No clean stretch of road in the session."}
	var closed := best.size() > 50 and best[0].distance_to(best[best.size() - 1]) < 25.0
	if closed:
		while best.size() > 10 and best[best.size() - 1].distance_to(best[0]) < 3.0:
			best.remove_at(best.size() - 1)
	var ref := ForzaTelemetry.resample(ForzaTelemetry.smooth(best, 3, closed), 1.0)
	if closed:
		while ref.size() > 10 and ref[ref.size() - 1].distance_to(ref[0]) < 0.5:
			ref.remove_at(ref.size() - 1)
	var n := ref.size()
	var right := ForzaTelemetry._rights(ref, closed)
	var grid := ForzaTelemetry._grid(ref, 8.0)

	# --- Accumulate wheel contacts per slice -------------------------------------
	var nb := int(2.0 * LMAX / BIN)
	var hist := PackedInt32Array()
	hist.resize(3 * n * nb)
	var cy_sum := PackedFloat64Array()   # per slice: n, sum l, sum y, sum l*l, sum l*y (car centre)
	cy_sum.resize(n * 5)
	var lap_marks := PackedInt32Array()  # slices where the lap counter ticked over
	var hint := -1
	var k_rough := surf_max * 0.25
	var k_kerb := surf_max * 0.75
	for i in n_s:
		var pc: Vector3 = conv.call(pos[i])
		var j := _nearest(ref, grid, pc, hint, closed)
		if j < 0:
			hint = -1
			continue
		hint = j
		var tng := Vector3(right[j].z, 0.0, -right[j].x)
		if absf((pc - ref[j]).dot(tng)) > 2.0 or absf(pc.y - ref[j].y) > 6.0:
			continue
		if i > 0 and seg[i] == seg[i - 1] and laps[i] != laps[i - 1]:
			lap_marks.append(j)
		var lc := (pc - ref[j]).dot(right[j])
		var o := j * 5
		cy_sum[o] += 1.0
		cy_sum[o + 1] += lc
		cy_sum[o + 2] += pc.y
		cy_sum[o + 3] += lc * lc
		cy_sum[o + 4] += lc * pc.y
		var a := ys * yaw[i] + yc
		var f := Vector3(sin(a), 0.0, cos(a))
		var r := Vector3(cos(a), 0.0, -sin(a)) * rs
		for w in 4:
			var lx := -HALF_TRACK if w % 2 == 0 else HALF_TRACK     # FL, FR, RL, RR
			var lz := HALF_BASE if w < 2 else -HALF_BASE
			var wp: Vector3 = conv.call(pos[i] + r * lx + f * lz)
			var jj := clampi(j + roundi((wp - ref[j]).dot(tng)), 0, n - 1) if not closed else posmod(j + roundi((wp - ref[j]).dot(tng)), n)
			var lw := (wp - ref[jj]).dot(right[jj])
			var bi := int((lw + LMAX) / BIN)
			if bi < 0 or bi >= nb:
				continue
			var v := surf[i * 4 + w]
			var c := KERB if v >= k_kerb and surf_max > 1e-4 else (ROUGH if v >= k_rough and surf_max > 1e-4 else TARMAC)
			hist[(c * n + jj) * nb + bi] += 1

	# --- Edges, kerbs, shoulders per slice ---------------------------------------
	var le := PackedFloat32Array()
	var re := PackedFloat32Array()
	le.resize(n)
	re.resize(n)
	var have := PackedByteArray()
	have.resize(n)
	var kerb_l := PackedByteArray()
	var kerb_r := PackedByteArray()
	kerb_l.resize(n)
	kerb_r.resize(n)
	var kerb_in := PackedFloat32Array()  # how far kerbs reach in from the edge
	var beyond := PackedFloat32Array()   # rough contacts beyond the edges (m)
	var agg := PackedInt32Array()
	agg.resize(3 * nb)
	for i in n:
		agg.fill(0)
		for d in range(-2, 3):
			var jj := i + d
			if closed:
				jj = posmod(jj, n)
			elif jj < 0 or jj >= n:
				continue
			for c in 3:
				var base := (c * n + jj) * nb
				for bi in nb:
					agg[c * nb + bi] += hist[base + bi]
		var lo := -1
		var hi := -1
		for bi in nb:
			if agg[bi] >= 3:
				if lo < 0:
					lo = bi
				hi = bi
		if lo < 0:
			continue
		have[i] = 1
		le[i] = (lo + 0.5) * BIN - LMAX - 0.15
		re[i] = (hi + 0.5) * BIN - LMAX + 0.15
		# Rough-ground contacts just beyond the furthest tarmac contact mean the
		# edge is right inside them (the tarmac in between just wasn't driven on
		# in this stretch, e.g. an edge lap that slid off).
		var reach := int(2.5 / BIN)
		for bi in range(lo - 1, maxi(lo - reach, 0) - 1, -1):
			if agg[nb + bi] >= 2:
				le[i] = minf(le[i], (bi + 0.5) * BIN - LMAX + 0.2)
				break
		for bi in range(hi + 1, mini(hi + reach, nb - 1) + 1):
			if agg[nb + bi] >= 2:
				re[i] = maxf(re[i], (bi + 0.5) * BIN - LMAX - 0.2)
				break
		var mid := (le[i] + re[i]) * 0.5
		for bi in nb:
			var l := (bi + 0.5) * BIN - LMAX
			if agg[2 * nb + bi] >= 3:
				if l < mid:
					kerb_l[i] = 1
					kerb_in.append(l - le[i])
				else:
					kerb_r[i] = 1
					kerb_in.append(re[i] - l)
			if agg[nb + bi] >= 2:
				if l < le[i]:
					beyond.append(le[i] - l)
				elif l > re[i]:
					beyond.append(l - re[i])
	var nv := have.count(1)
	if nv < n * 0.3:
		return {"ok": false, "error": "Only found the road edges along %d%% of the track." % int(100.0 * nv / n)}
	_fill(le, have, closed)
	_fill(re, have, closed)
	# Smooth the centre offset much harder than the width: a local change in
	# width (a paved run-off, a kerb) must not bend the road.
	var coff := PackedFloat32Array()
	var half := PackedFloat32Array()
	for i in n:
		coff.append((le[i] + re[i]) * 0.5)
		half.append((re[i] - le[i]) * 0.5)
	coff = ForzaTelemetry._smooth_f(_smooth_med(coff, 21, closed), 10, closed)
	half = _smooth_med(half, 9, closed)
	for i in n:
		le[i] = coff[i] - half[i]
		re[i] = coff[i] + half[i]

	# --- Elevation + camber from the car's height across the road ----------------
	var elev := PackedFloat32Array()
	var bank := PackedFloat32Array()
	elev.resize(n)
	bank.resize(n)
	var ok_e := PackedByteArray()
	ok_e.resize(n)
	for i in n:
		var sn := 0.0
		var sl := 0.0
		var sy := 0.0
		var sll := 0.0
		var sly := 0.0
		for d in range(-3, 4):
			var jj := i + d
			if closed:
				jj = posmod(jj, n)
			elif jj < 0 or jj >= n:
				continue
			var o := jj * 5
			sn += cy_sum[o]
			sl += cy_sum[o + 1]
			sy += cy_sum[o + 2]
			sll += cy_sum[o + 3]
			sly += cy_sum[o + 4]
		if sn < 10.0:
			continue
		var var_l := sll / sn - pow(sl / sn, 2.0)
		var mid := (le[i] + re[i]) * 0.5
		if var_l > 1.0:   # enough spread across the road to fit a slope
			var slope := (sly / sn - (sl / sn) * (sy / sn)) / var_l
			bank[i] = clampf(slope, -0.15, 0.15)
			elev[i] = sy / sn + bank[i] * (mid - sl / sn)
		else:
			elev[i] = sy / sn
		ok_e[i] = 1
	_fill(elev, ok_e, closed)
	_fill(bank, ok_e, closed)
	elev = ForzaTelemetry._smooth_f(elev, 4, closed)
	bank = ForzaTelemetry._smooth_f(_smooth_med(bank, 9, closed), 7, closed)

	# --- Kerb ranges -----------------------------------------------------------------
	var kerb_w := 1.0
	if kerb_in.size() > 10:
		var ks := Array(kerb_in)
		ks.sort()
		kerb_w = clampf(float(ks[ks.size() / 2]) + 0.3, 0.6, 1.6)
	var total := float(n) if closed else float(n - 1)
	var kerb_ranges := []
	for side: String in ["left", "right"]:
		var arr := kerb_l if side == "left" else kerb_r
		var ds := []
		for i in n:
			if arr[i] == 1:
				ds.append(float(i))
		for rg: Vector2 in ForzaTelemetry._merge_ranges(ds, 6.0, 1.0, total):
			if rg.y - rg.x >= 3.0:
				kerb_ranges.append({"side": side, "start": snappedf(rg.x / total, 0.0001), "end": snappedf(rg.y / total, 0.0001), "width": snappedf(kerb_w, 0.05)})
	var runoff := 1.0
	if beyond.size() > 10:
		var bs := Array(beyond)
		bs.sort()
		runoff = clampf(float(bs[int(bs.size() * 0.9)]) + 0.3, 0.5, 4.0)

	# --- Centreline + spec ---------------------------------------------------------
	var centre := PackedVector3Array()
	var widths := PackedFloat32Array()
	var min_y := INF
	for i in n:
		var c := ref[i] + right[i] * ((le[i] + re[i]) * 0.5)
		c.y = elev[i]
		centre.append(c)
		widths.append(clampf(re[i] - le[i], 4.0, 40.0))
		min_y = minf(min_y, elev[i])
	centre = ForzaTelemetry.smooth(centre, 3, closed)
	var cg := Vector3.ZERO
	for c in centre:
		cg += c
	cg /= n
	var points := []
	var wlist := []
	var blist := []
	for i in range(0, n, 4):
		points.append([snappedf(centre[i].x - cg.x, 0.05), snappedf(centre[i].z - cg.z, 0.05), snappedf(centre[i].y - min_y, 0.05)])
		wlist.append(snappedf(widths[i], 0.05))
		blist.append(snappedf(bank[i], 0.001))
	var start := 0.0
	if lap_marks.size() > 0:
		var counts := {}
		for j in lap_marks:
			var key := j / 5
			counts[key] = int(counts.get(key, 0)) + 1
		var best_k: int = counts.keys()[0]
		for k: int in counts:
			if counts[k] > counts[best_k]:
				best_k = k
		start = snappedf(float(best_k * 5) / total, 0.0001)
	var avg_w := 0.0
	for w in widths:
		avg_w += w
	avg_w /= n
	var spec := {
		"name": name, "closed": closed, "points": points, "width": snappedf(avg_w, 0.1),
		"widths": wlist, "banks": blist,
		"runoff": snappedf(runoff, 0.1), "runoff_surface": "grass",
		"walls": {"left": true, "right": true, "height": 1.0},
		"kerbs": not kerb_ranges.is_empty(),
		"zones": [] if closed else [{"name": name, "start": 0.02, "end": 0.98}],
		"clips": [], "start": start,
		"source": {"kind": "forza_session", "mirrored_x": mirror, "samples": n_s},
	}
	if not kerb_ranges.is_empty():
		spec["kerb_ranges"] = kerb_ranges
	var length := 0.0
	for i in range(1, n):
		length += centre[i].distance_to(centre[i - 1])
	var wsorted := Array(widths)
	wsorted.sort()
	var max_bank := 0.0
	for b in bank:
		max_bank = maxf(max_bank, absf(b))
	return {"ok": true, "spec": spec, "error": "", "info": {
		"length_m": length, "closed": closed, "mirrored": mirror, "width_m": avg_w,
		"width_min": float(wsorted[int(n * 0.05)]), "width_max": float(wsorted[int(n * 0.95)]),
		"edge_cover": float(nv) / n, "kerbs": kerb_ranges.size(), "kerb_width": kerb_w, "runoff": runoff,
		"max_bank_pct": max_bank * 100.0, "elevation_m": _range(elev), "minutes": packets.size() / 3600.0,
		"rewinds": seg_id, "surface_levels": surf_max, "origin": [cg.x, cg.z, min_y]}}


## Fits heading = s * yaw + c (s = +-1) against the direction of travel at low
## slip, and which way the car's local +X points. {} if it doesn't fit.
static func _calibrate_yaw(pos: PackedVector3Array, yaw: PackedFloat32Array, vx: PackedFloat32Array, vz: PackedFloat32Array, seg: PackedInt32Array) -> Dictionary:
	var out := {}
	var best_res := INF
	for s: float in [1.0, -1.0]:
		var cs := 0.0
		var sn := 0.0
		var diffs := PackedFloat32Array()
		for i in range(1, pos.size() - 1, 3):
			if seg[i - 1] != seg[i + 1] or vz[i] < 5.0 or absf(vx[i]) > 0.05 * vz[i]:
				continue
			var d := pos[i + 1] - pos[i - 1]
			if Vector2(d.x, d.z).length() < 0.2:
				continue
			var diff := wrapf(atan2(d.x, d.z) - s * yaw[i], -PI, PI)
			diffs.append(diff)
			cs += cos(diff)
			sn += sin(diff)
		if diffs.size() < 50:
			continue
		var c := atan2(sn, cs)
		var res := 0.0
		for df in diffs:
			res += absf(wrapf(df - c, -PI, PI))
		res /= diffs.size()
		if res < best_res:
			best_res = res
			out = {"sign": s, "offset": c}
	if out.is_empty() or best_res > 0.2:
		return {}
	# Local +X: predicted world velocity must match the real one while sliding.
	var errs := [0.0, 0.0]
	for k in 2:
		var rsg := 1.0 if k == 0 else -1.0
		for i in range(1, pos.size() - 1, 3):
			if seg[i - 1] != seg[i + 1] or absf(vx[i]) < 2.0:
				continue
			var a := float(out.sign) * yaw[i] + float(out.offset)
			var pred := Vector3(sin(a), 0, cos(a)) * vz[i] + Vector3(cos(a), 0, -sin(a)) * rsg * vx[i]
			var d := (pos[i + 1] - pos[i - 1]) * 30.0   # ~60 Hz
			errs[k] += Vector2(pred.x - d.x, pred.z - d.z).length()
	out["right_sign"] = 1.0 if errs[0] <= errs[1] else -1.0
	return out


static func _nearest(ref: PackedVector3Array, grid: Dictionary, p: Vector3, hint: int, closed: bool) -> int:
	var n := ref.size()
	if hint >= 0:
		var best := -1
		var bd := INF
		for d in range(-12, 13):
			var j := hint + d
			if closed:
				j = posmod(j, n)
			elif j < 0 or j >= n:
				continue
			var dd := Vector2(ref[j].x - p.x, ref[j].z - p.z).length_squared()
			if dd < bd:
				bd = dd
				best = j
		if bd < 30.0 * 30.0 and best != posmod(hint - 12, n) and best != posmod(hint + 12, n):
			return best
	var key := Vector2i(floori(p.x / 8.0), floori(p.z / 8.0))
	var best2 := -1
	var bd2 := 30.0 * 30.0
	for dx in range(-4, 5):
		for dz in range(-4, 5):
			for j: int in grid.get(key + Vector2i(dx, dz), []):
				var dd := Vector2(ref[j].x - p.x, ref[j].z - p.z).length_squared()
				if dd < bd2 and absf(ref[j].y - p.y) < 8.0:
					bd2 = dd
					best2 = j
	return best2


## Linear interpolation over slices without data.
static func _fill(a: PackedFloat32Array, have: PackedByteArray, closed: bool) -> void:
	var n := a.size()
	var idx := []
	for i in n:
		if have[i] == 1:
			idx.append(i)
	if idx.is_empty():
		return
	for g in idx.size():
		var i0: int = idx[g]
		var i1: int
		var span: int
		if g + 1 < idx.size():
			i1 = idx[g + 1]
			span = i1 - i0
		elif closed:
			i1 = idx[0]
			span = i1 + n - i0
		else:
			break
		for k in range(1, span):
			a[(i0 + k) % n] = lerpf(a[i0], a[i1], float(k) / span)
	if not closed:
		for i in idx[0]:
			a[i] = a[idx[0]]
		for i in range(idx[idx.size() - 1] + 1, n):
			a[i] = a[idx[idx.size() - 1]]


## Median then mean filter (kills single-slice spikes, then smooths).
static func _smooth_med(a: PackedFloat32Array, r: int, closed: bool) -> PackedFloat32Array:
	var n := a.size()
	var out := PackedFloat32Array()
	out.resize(n)
	var win := []
	for i in n:
		win.clear()
		for k in range(-r / 2, r / 2 + 1):
			var j := i + k
			if closed:
				j = posmod(j, n)
			elif j < 0 or j >= n:
				continue
			win.append(a[j])
		win.sort()
		out[i] = win[win.size() / 2]
	return ForzaTelemetry._smooth_f(out, r / 2, closed)


static func _range(a: PackedFloat32Array) -> float:
	var lo := INF
	var hi := -INF
	for v in a:
		lo = minf(lo, v)
		hi = maxf(hi, v)
	return hi - lo
