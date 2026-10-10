class_name TrackRaster
extends RefCounted
## Renders a top-down image of a track (minimap, previews for the AI agent,
## Assetto Corsa map.png).

## Returns {image: Image, origin: Vector2, scale: float} where
## pixel = (world_xz - origin) * scale.
static func render(spec: Dictionary, size := 512, bg := Color(0, 0, 0, 0), road := Color(1, 1, 1, 0.9), draw_zones := false) -> Dictionary:
	var s := TrackSpec.normalized(spec)
	var samples := TrackSpec.sample(s, 2.0)
	var pos: PackedVector3Array = samples.get("pos", PackedVector3Array())
	var minv := Vector2(INF, INF)
	var maxv := Vector2(-INF, -INF)
	for p in pos:
		minv = minv.min(Vector2(p.x, p.z))
		maxv = maxv.max(Vector2(p.x, p.z))
	for pad: Dictionary in s.pads:
		for q: Array in pad.get("points", []):
			minv = minv.min(Vector2(float(q[0]), float(q[1])))
			maxv = maxv.max(Vector2(float(q[0]), float(q[1])))
	if minv.x == INF:
		minv = Vector2(-50, -50)
		maxv = Vector2(50, 50)
	var margin := float(s.width) + 4.0
	minv -= Vector2(margin, margin)
	maxv += Vector2(margin, margin)
	var extent := maxf(maxv.x - minv.x, maxv.y - minv.y)
	var scale := float(size) / extent
	var origin := minv - Vector2((extent - (maxv.x - minv.x)) * 0.5, (extent - (maxv.y - minv.y)) * 0.5)
	var img := Image.create(size, size, false, Image.FORMAT_RGBA8)
	img.fill(bg)
	for pad: Dictionary in s.pads:
		var poly := PackedVector2Array()
		for q: Array in pad.get("points", []):
			poly.append((Vector2(float(q[0]), float(q[1])) - origin) * scale)
		_fill_polygon(img, poly, road)
	var n := pos.size()
	if n >= 2:
		var width: PackedFloat32Array = samples.width
		var segs := n if samples.closed else n - 1
		for i in segs:
			var a := (Vector2(pos[i].x, pos[i].z) - origin) * scale
			var b := (Vector2(pos[(i + 1) % n].x, pos[(i + 1) % n].z) - origin) * scale
			_thick_line(img, a, b, maxf(width[i] * scale * 0.5, 1.0), road)
		if draw_zones:
			for z: Dictionary in s.zones:
				var i0 := int(float(z.start) * n)
				var i1 := int(float(z.end) * n)
				if i1 < i0:
					i1 += n
				for i in range(i0, i1):
					var p := pos[i % n]
					var q := pos[(i + 1) % n]
					_thick_line(img, (Vector2(p.x, p.z) - origin) * scale, (Vector2(q.x, q.z) - origin) * scale, maxf(1.5, width[i % n] * scale * 0.18), Color(1.0, 0.55, 0.1))
			var st := (Vector2(pos[int(float(s.start) * n) % n].x, pos[int(float(s.start) * n) % n].z) - origin) * scale
			_disc(img, st, 4.0, Color(0.1, 0.9, 0.2))
	return {"image": img, "origin": origin, "scale": scale}


static func _disc(img: Image, c: Vector2, r: float, col: Color) -> void:
	var w := img.get_width()
	var h := img.get_height()
	for y in range(maxi(int(c.y - r), 0), mini(int(c.y + r) + 1, h)):
		for x in range(maxi(int(c.x - r), 0), mini(int(c.x + r) + 1, w)):
			if Vector2(x, y).distance_to(c) <= r:
				img.set_pixel(x, y, col)


static func _thick_line(img: Image, a: Vector2, b: Vector2, r: float, col: Color) -> void:
	var w := img.get_width()
	var h := img.get_height()
	var lo := a.min(b) - Vector2(r, r)
	var hi := a.max(b) + Vector2(r, r)
	var ab := b - a
	var len2 := maxf(ab.length_squared(), 1e-6)
	for y in range(maxi(int(lo.y), 0), mini(int(hi.y) + 1, h)):
		for x in range(maxi(int(lo.x), 0), mini(int(hi.x) + 1, w)):
			var p := Vector2(x, y)
			var t := clampf((p - a).dot(ab) / len2, 0.0, 1.0)
			if p.distance_to(a + ab * t) <= r:
				img.set_pixel(x, y, col)


static func _fill_polygon(img: Image, poly: PackedVector2Array, col: Color) -> void:
	if poly.size() < 3:
		return
	var lo := Vector2(INF, INF)
	var hi := Vector2(-INF, -INF)
	for p in poly:
		lo = lo.min(p)
		hi = hi.max(p)
	for y in range(maxi(int(lo.y), 0), mini(int(hi.y) + 1, img.get_height())):
		for x in range(maxi(int(lo.x), 0), mini(int(hi.x) + 1, img.get_width())):
			if Geometry2D.is_point_in_polygon(Vector2(x, y), poly):
				img.set_pixel(x, y, col)
