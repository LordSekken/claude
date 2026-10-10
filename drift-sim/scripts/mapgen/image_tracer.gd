class_name ImageTracer
extends RefCounted
## Offline "picture -> track": works on top-down images (satellite/map
## screenshots, drone shots of a lot) and on hand-drawn sketches.
##
## 1. Segment road pixels (grey asphalt, dark sketch lines, or a picked colour)
## 2. Clean up (morphological close/open), keep the largest connected blob
## 3. Thin to a 1-px skeleton (Zhang-Suen) and prune side spurs
## 4. Walk the loop (or the longest path for point-to-point roads)
## 5. Simplify, scale to metres (from the road's pixel width) and centre
##
## For ground-level photos, use the Claude agent instead (it designs a layout
## inspired by the photo).

const WORK_SIZE := 420

## options: {mode: auto|road|lines|color, color: Color, tolerance: float,
##           road_width_m: float, target_length_m: float, area_as_pad: bool}
## Returns {ok, spec, debug: Image, error, info}
static func trace(src: Image, options: Dictionary = {}) -> Dictionary:
	var img := src.duplicate() as Image
	if img.is_compressed():
		img.decompress()
	img.convert(Image.FORMAT_RGB8)
	var longest := maxi(img.get_width(), img.get_height())
	var scale_down := 1.0
	if longest > WORK_SIZE:
		scale_down = float(WORK_SIZE) / longest
		img.resize(maxi(int(img.get_width() * scale_down), 8), maxi(int(img.get_height() * scale_down), 8), Image.INTERPOLATE_BILINEAR)
	var w := img.get_width()
	var h := img.get_height()
	var mode := str(options.get("mode", "auto"))
	if mode == "auto":
		mode = _guess_mode(img)
	var mask := _segment(img, mode, options)
	if mode == "lines":
		mask = _dilate(mask, w, h, 1)
	mask = _erode(_dilate(mask, w, h, 2), w, h, 2)   # close gaps
	mask = _dilate(_erode(mask, w, h, 1), w, h, 1)   # remove speckle
	mask = _largest_component(mask, w, h)
	var area := 0
	for v in mask:
		area += v
	if area < 60:
		return {"ok": false, "error": "Couldn't find a road/line in the image. Try another mode or pick the road colour.", "debug": _debug(img, mask, PackedByteArray(), [])}

	var dist := _distance_transform(mask, w, h)
	var info := {"mode": mode, "pixels": area}

	if bool(options.get("area_as_pad", false)):
		return _as_pad(img, mask, w, h, options, info)

	var skel := _thin(mask.duplicate(), w, h)
	var pruned := _prune_all(skel.duplicate(), w, h)
	var path: Array[Vector2i] = []
	var closed := false
	var remaining := 0
	for v in pruned:
		remaining += v
	if remaining > 20:
		closed = true
		path = _walk_loop(pruned, w, h)
	if path.size() < 10:
		closed = false
		path = _longest_path(skel, w, h)
	if path.size() < 6:
		return {"ok": false, "error": "The traced shape is too small or fragmented.", "debug": _debug(img, mask, skel, [])}

	var pts := PackedVector2Array()
	for p in path:
		pts.append(Vector2(p))
	pts = _rdp(pts, 1.2, closed)
	pts = _resample(pts, 7.0, closed)

	# Scale: real road width vs. pixel width (median skeleton distance x2).
	var widths := []
	for p in path:
		widths.append(dist[p.y * w + p.x])
	widths.sort()
	var road_px := maxf(2.0 * float(widths[widths.size() / 2]), 2.0)
	var mpp: float
	var road_w := float(options.get("road_width_m", 12.0))
	if options.has("meters_per_pixel"):
		mpp = float(options.meters_per_pixel) / scale_down
	elif mode == "lines":
		var len_px := 0.0
		for i in pts.size() - (0 if closed else 1):
			len_px += pts[i].distance_to(pts[(i + 1) % pts.size()])
		mpp = float(options.get("target_length_m", 1100.0)) / maxf(len_px, 1.0)
	else:
		mpp = road_w / road_px
	info.road_px = road_px
	info.meters_per_pixel_original = mpp * scale_down

	var center := Vector2.ZERO
	for p in pts:
		center += p
	center /= pts.size()
	var points := []
	for p in pts:
		var m := (p - center) * mpp
		points.append([snappedf(m.x, 0.1), snappedf(m.y, 0.1)])
	var spec := {
		"name": str(options.get("name", "Traced Track")),
		"closed": closed,
		"points": points,
		"width": road_w,
		"runoff": 4.0,
		"walls": {"left": true, "right": true, "height": 1.2},
		"zones": [],
		"clips": [],
		"source": {"kind": "image_trace", "mode": mode, "meters_per_pixel": info.meters_per_pixel_original},
	}
	info.length_m = TrackSpec.sample(spec, 2.0).length
	return {"ok": true, "spec": spec, "debug": _debug(img, mask, skel, path), "info": info, "error": ""}


static func _lum(c: Color) -> float:
	return 0.299 * c.r + 0.587 * c.g + 0.114 * c.b


static func _guess_mode(img: Image) -> String:
	var total := 0.0
	var dark := 0
	var n := 0
	for y in range(0, img.get_height(), 3):
		for x in range(0, img.get_width(), 3):
			var l := _lum(img.get_pixel(x, y))
			total += l
			if l < 0.35:
				dark += 1
			n += 1
	var mean := total / n
	if mean > 0.62 and float(dark) / n < 0.2:
		return "lines"
	return "road"


static func _segment(img: Image, mode: String, opt: Dictionary) -> PackedByteArray:
	var w := img.get_width()
	var h := img.get_height()
	var mask := PackedByteArray()
	mask.resize(w * h)
	var target: Color = opt.get("color", Color(0.3, 0.3, 0.3))
	var tol := float(opt.get("tolerance", 0.12))
	# Otsu threshold on luminance for sketches.
	var thresh := 0.5
	if mode == "lines":
		thresh = _otsu(img)
	for y in h:
		for x in w:
			var c := img.get_pixel(x, y)
			var on := false
			match mode:
				"lines":
					on = _lum(c) < thresh
				"color":
					on = Vector3(c.r - target.r, c.g - target.g, c.b - target.b).length() < tol * 1.7
				_:
					var mx := maxf(c.r, maxf(c.g, c.b))
					var mn := minf(c.r, minf(c.g, c.b))
					var sat := 0.0 if mx < 0.001 else (mx - mn) / mx
					on = sat < 0.18 and mx > 0.12 and mx < 0.62
			mask[y * w + x] = 1 if on else 0
	return mask


static func _otsu(img: Image) -> float:
	var hist := PackedInt32Array()
	hist.resize(64)
	var n := 0
	for y in img.get_height():
		for x in img.get_width():
			hist[clampi(int(_lum(img.get_pixel(x, y)) * 63.0), 0, 63)] += 1
			n += 1
	var sum := 0.0
	for i in 64:
		sum += i * hist[i]
	var sum_b := 0.0
	var wb := 0
	var best := 0.0
	var thr := 32
	for i in 64:
		wb += hist[i]
		if wb == 0:
			continue
		var wf := n - wb
		if wf == 0:
			break
		sum_b += i * hist[i]
		var mb := sum_b / wb
		var mf := (sum - sum_b) / wf
		var between := float(wb) * wf * (mb - mf) * (mb - mf)
		if between > best:
			best = between
			thr = i
	return (thr + 0.5) / 63.0


static func _dilate(m: PackedByteArray, w: int, h: int, r: int) -> PackedByteArray:
	var out := m.duplicate()
	for y in h:
		for x in w:
			if m[y * w + x] == 1:
				continue
			var hit := false
			for dy in range(-r, r + 1):
				var yy := y + dy
				if yy < 0 or yy >= h:
					continue
				for dx in range(-r, r + 1):
					var xx := x + dx
					if xx >= 0 and xx < w and m[yy * w + xx] == 1:
						hit = true
						break
				if hit:
					break
			if hit:
				out[y * w + x] = 1
	return out


static func _erode(m: PackedByteArray, w: int, h: int, r: int) -> PackedByteArray:
	var inv := PackedByteArray()
	inv.resize(m.size())
	for i in m.size():
		inv[i] = 1 - m[i]
	var d := _dilate(inv, w, h, r)
	for i in d.size():
		d[i] = 1 - d[i]
	return d


static func _largest_component(m: PackedByteArray, w: int, h: int) -> PackedByteArray:
	var label := PackedInt32Array()
	label.resize(m.size())
	var best_label := 0
	var best_size := 0
	var next := 1
	for start in m.size():
		if m[start] == 0 or label[start] != 0:
			continue
		var stack := [start]
		label[start] = next
		var size := 0
		while not stack.is_empty():
			var i: int = stack.pop_back()
			size += 1
			var x := i % w
			var y := i / w
			for d: Vector2i in [Vector2i(1, 0), Vector2i(-1, 0), Vector2i(0, 1), Vector2i(0, -1)]:
				var xx := x + d.x
				var yy := y + d.y
				if xx < 0 or yy < 0 or xx >= w or yy >= h:
					continue
				var j := yy * w + xx
				if m[j] == 1 and label[j] == 0:
					label[j] = next
					stack.append(j)
		if size > best_size:
			best_size = size
			best_label = next
		next += 1
	var out := PackedByteArray()
	out.resize(m.size())
	for i in m.size():
		out[i] = 1 if label[i] == best_label and best_label > 0 else 0
	return out


## Chamfer distance to the nearest background pixel.
static func _distance_transform(m: PackedByteArray, w: int, h: int) -> PackedFloat32Array:
	var d := PackedFloat32Array()
	d.resize(m.size())
	var big := 1e6
	for i in m.size():
		d[i] = big if m[i] == 1 else 0.0
	var diag := 1.4142
	for y in h:
		for x in w:
			var i := y * w + x
			if d[i] == 0.0:
				continue
			var v := d[i]
			if x > 0: v = minf(v, d[i - 1] + 1.0)
			if y > 0:
				v = minf(v, d[i - w] + 1.0)
				if x > 0: v = minf(v, d[i - w - 1] + diag)
				if x < w - 1: v = minf(v, d[i - w + 1] + diag)
			if x == 0 or y == 0 or x == w - 1 or y == h - 1:
				v = minf(v, 1.0)
			d[i] = v
	for y in range(h - 1, -1, -1):
		for x in range(w - 1, -1, -1):
			var i := y * w + x
			if d[i] == 0.0:
				continue
			var v := d[i]
			if x < w - 1: v = minf(v, d[i + 1] + 1.0)
			if y < h - 1:
				v = minf(v, d[i + w] + 1.0)
				if x < w - 1: v = minf(v, d[i + w + 1] + diag)
				if x > 0: v = minf(v, d[i + w - 1] + diag)
			d[i] = v
	return d


static func _nb(m: PackedByteArray, w: int, h: int, x: int, y: int) -> Array[int]:
	# P2..P9 clockwise starting north.
	var out: Array[int] = []
	for d: Vector2i in [Vector2i(0, -1), Vector2i(1, -1), Vector2i(1, 0), Vector2i(1, 1), Vector2i(0, 1), Vector2i(-1, 1), Vector2i(-1, 0), Vector2i(-1, -1)]:
		var xx := x + d.x
		var yy := y + d.y
		out.append(m[yy * w + xx] if xx >= 0 and yy >= 0 and xx < w and yy < h else 0)
	return out


## Zhang-Suen thinning.
static func _thin(m: PackedByteArray, w: int, h: int) -> PackedByteArray:
	var changed := true
	var guard := 0
	while changed and guard < 200:
		changed = false
		guard += 1
		for step in 2:
			var kill := []
			for y in range(1, h - 1):
				for x in range(1, w - 1):
					var i := y * w + x
					if m[i] == 0:
						continue
					var p := _nb(m, w, h, x, y)
					var b := 0
					for v in p:
						b += v
					if b < 2 or b > 6:
						continue
					var a := 0
					for k in 8:
						if p[k] == 0 and p[(k + 1) % 8] == 1:
							a += 1
					if a != 1:
						continue
					if step == 0:
						if p[0] * p[2] * p[4] != 0 or p[2] * p[4] * p[6] != 0:
							continue
					else:
						if p[0] * p[2] * p[6] != 0 or p[0] * p[4] * p[6] != 0:
							continue
					kill.append(i)
			for i: int in kill:
				m[i] = 0
			if kill.size() > 0:
				changed = true
	return m


static func _degree(m: PackedByteArray, w: int, h: int, x: int, y: int) -> int:
	var c := 0
	for v in _nb(m, w, h, x, y):
		c += v
	return c


## Repeatedly removes endpoints: what survives is the loop(s).
static func _prune_all(m: PackedByteArray, w: int, h: int) -> PackedByteArray:
	var changed := true
	var guard := 0
	while changed and guard < 2000:
		changed = false
		guard += 1
		var kill := []
		for i in m.size():
			if m[i] == 1 and _degree(m, w, h, i % w, i / w) <= 1:
				kill.append(i)
		for i: int in kill:
			m[i] = 0
		changed = kill.size() > 0
	return m


static func _neighbors(m: PackedByteArray, w: int, h: int, p: Vector2i) -> Array[Vector2i]:
	var out: Array[Vector2i] = []
	for d: Vector2i in [Vector2i(0, -1), Vector2i(1, -1), Vector2i(1, 0), Vector2i(1, 1), Vector2i(0, 1), Vector2i(-1, 1), Vector2i(-1, 0), Vector2i(-1, -1)]:
		var q := p + d
		if q.x >= 0 and q.y >= 0 and q.x < w and q.y < h and m[q.y * w + q.x] == 1:
			out.append(q)
	return out


## Walks the outermost loop, preferring the straightest continuation at
## junctions so figure-8s and inner branches don't derail it.
static func _walk_loop(m: PackedByteArray, w: int, h: int) -> Array[Vector2i]:
	# Start at the left-most pixel (it's on the outer loop).
	var start := Vector2i(-1, -1)
	for x in w:
		for y in h:
			if m[y * w + x] == 1:
				start = Vector2i(x, y)
				break
		if start.x >= 0:
			break
	var path: Array[Vector2i] = [start]
	var visited := {start: true}
	var dir := Vector2(0, -1)
	var cur := start
	for _i in m.size():
		var cands := _neighbors(m, w, h, cur)
		var best := Vector2i(-1, -1)
		var best_score := -INF
		for q in cands:
			if visited.has(q):
				if q == start and path.size() > 20:
					return path
				continue
			var d := Vector2(q - cur).normalized()
			var score := d.dot(dir)
			if score > best_score:
				best_score = score
				best = q
		if best.x < 0:
			break
		dir = dir.lerp(Vector2(best - cur).normalized(), 0.5).normalized()
		cur = best
		visited[cur] = true
		path.append(cur)
	return path


static func _bfs_far(m: PackedByteArray, w: int, h: int, s: Vector2i) -> Dictionary:
	var prev := {s: s}
	var queue := [s]
	var last := s
	var head := 0
	while head < queue.size():
		var p: Vector2i = queue[head]
		head += 1
		last = p
		for q in _neighbors(m, w, h, p):
			if not prev.has(q):
				prev[q] = p
				queue.append(q)
	return {"far": last, "prev": prev}


static func _longest_path(m: PackedByteArray, w: int, h: int) -> Array[Vector2i]:
	var any := -1
	for i in m.size():
		if m[i] == 1:
			any = i
			break
	if any < 0:
		return []
	var a := _bfs_far(m, w, h, Vector2i(any % w, any / w))
	var b := _bfs_far(m, w, h, a.far)
	var path: Array[Vector2i] = []
	var cur: Vector2i = b.far
	var prev: Dictionary = b.prev
	while true:
		path.append(cur)
		var p: Vector2i = prev[cur]
		if p == cur:
			break
		cur = p
	return path


static func _rdp(pts: PackedVector2Array, eps: float, closed: bool) -> PackedVector2Array:
	if pts.size() < 3:
		return pts
	if closed:
		# Split the loop at the farthest point pair to run RDP on two halves.
		var far := 0
		var best := 0.0
		for i in pts.size():
			var d := pts[0].distance_to(pts[i])
			if d > best:
				best = d
				far = i
		var a := pts.slice(0, far + 1)
		var b := pts.slice(far)
		b.append(pts[0])
		var ra := _rdp(a, eps, false)
		var rb := _rdp(b, eps, false)
		ra.remove_at(ra.size() - 1)
		rb.remove_at(rb.size() - 1)
		ra.append_array(rb)
		return ra
	var dmax := 0.0
	var idx := 0
	var a0 := pts[0]
	var a1 := pts[pts.size() - 1]
	for i in range(1, pts.size() - 1):
		var d := _seg_dist(pts[i], a0, a1)
		if d > dmax:
			dmax = d
			idx = i
	if dmax > eps:
		var left := _rdp(pts.slice(0, idx + 1), eps, false)
		var right := _rdp(pts.slice(idx), eps, false)
		left.remove_at(left.size() - 1)
		left.append_array(right)
		return left
	return PackedVector2Array([a0, a1])


static func _seg_dist(p: Vector2, a: Vector2, b: Vector2) -> float:
	var ab := b - a
	var t := clampf((p - a).dot(ab) / maxf(ab.length_squared(), 1e-9), 0.0, 1.0)
	return p.distance_to(a + ab * t)


## Even spacing (in pixels) so the spline gets well-distributed control points.
static func _resample(pts: PackedVector2Array, step: float, closed: bool) -> PackedVector2Array:
	var out := PackedVector2Array()
	if pts.size() < 2:
		return pts
	var segs := pts.size() if closed else pts.size() - 1
	var carry := 0.0
	out.append(pts[0])
	for i in segs:
		var a := pts[i]
		var b := pts[(i + 1) % pts.size()]
		var L := a.distance_to(b)
		var d := step - carry
		while d <= L:
			out.append(a.lerp(b, d / L))
			d += step
		carry = L - (d - step)
	if closed and out.size() > 2 and out[out.size() - 1].distance_to(out[0]) < step * 0.5:
		out.remove_at(out.size() - 1)
	if not closed:
		out.append(pts[pts.size() - 1])
	return out


static func _as_pad(img: Image, mask: PackedByteArray, w: int, h: int, options: Dictionary, info: Dictionary) -> Dictionary:
	var bm := BitMap.new()
	bm.create(Vector2i(w, h))
	for y in h:
		for x in w:
			bm.set_bit(x, y, mask[y * w + x] == 1)
	var polys := bm.opaque_to_polygons(Rect2i(0, 0, w, h), 1.5)
	if polys.is_empty():
		return {"ok": false, "error": "Couldn't outline the area.", "debug": _debug(img, mask, PackedByteArray(), [])}
	var biggest: PackedVector2Array = polys[0]
	for p: PackedVector2Array in polys:
		if p.size() > biggest.size():
			biggest = p
	var mpp := float(options.get("meters_per_pixel", 0.5))
	if not options.has("meters_per_pixel"):
		# Assume the lot spans ~ target size (default 140 m across).
		var r := Rect2(biggest[0], Vector2.ZERO)
		for p in biggest:
			r = r.expand(p)
		mpp = float(options.get("target_size_m", 140.0)) / maxf(r.size.x, r.size.y)
	var c := Vector2.ZERO
	for p in biggest:
		c += p
	c /= biggest.size()
	var pts := []
	for p in biggest:
		pts.append([snappedf((p.x - c.x) * mpp, 0.1), snappedf((p.y - c.y) * mpp, 0.1)])
	var spec := {"name": str(options.get("name", "Traced Lot")), "closed": true, "points": [], "pads": [{"points": pts, "surface": "asphalt"}], "zones": [], "clips": []}
	return {"ok": true, "spec": spec, "debug": _debug(img, mask, PackedByteArray(), []), "info": info, "error": ""}


static func _debug(img: Image, mask: PackedByteArray, skel: PackedByteArray, path: Array) -> Image:
	var out := img.duplicate() as Image
	var w := out.get_width()
	for i in mask.size():
		if mask[i] == 1:
			var c := out.get_pixel(i % w, i / w)
			out.set_pixel(i % w, i / w, c.lerp(Color(0.2, 0.5, 1.0), 0.45))
	for i in skel.size():
		if skel[i] == 1:
			out.set_pixel(i % w, i / w, Color(1, 0.9, 0))
	for p: Vector2i in path:
		out.set_pixel(p.x, p.y, Color(1, 0.1, 0.1))
	return out
