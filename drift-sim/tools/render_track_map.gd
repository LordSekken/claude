extends SceneTree
## Top-down preview of a Drift Lab track: road in grey (true width), kerbs on
## corners, clipping points in red, start line in green, drift zones tinted,
## 50 m grid for scale.
##   godot --headless --path drift-sim -s tools/render_track_map.gd -- <track.json> <out.png> [size_px]

func _init() -> void:
	await process_frame
	var args := OS.get_cmdline_user_args()
	if args.size() < 2:
		printerr("usage: -- <track.json> <out.png> [size_px]")
		quit(2)
		return
	var spec: Variant = JsonUtil.load_file(args[0])
	if not (spec is Dictionary):
		printerr("FAIL can't read " + args[0])
		quit(1)
		return
	var size := int(args[2]) if args.size() > 2 else 900
	var s := TrackSpec.sample(TrackSpec.normalized(spec), 1.0)
	var pos: PackedVector3Array = s.pos
	var width: PackedFloat32Array = s.width
	var lo := Vector2(INF, INF)
	var hi := Vector2(-INF, -INF)
	for p in pos:
		lo = Vector2(minf(lo.x, p.x), minf(lo.y, p.z))
		hi = Vector2(maxf(hi.x, p.x), maxf(hi.y, p.z))
	var margin := 30.0
	lo -= Vector2(margin, margin)
	hi += Vector2(margin, margin)
	var k := (size - 1) / maxf(hi.x - lo.x, hi.y - lo.y)
	var img := Image.create(size, size, false, Image.FORMAT_RGB8)
	img.fill(Color(0.16, 0.22, 0.14))
	# 50 m grid
	var g := ceilf(lo.x / 50.0) * 50.0
	while g < hi.x:
		var x := int((g - lo.x) * k)
		for y in size:
			img.set_pixel(x, y, Color(0.2, 0.27, 0.18))
		g += 50.0
	g = ceilf(lo.y / 50.0) * 50.0
	while g < hi.y:
		var y := int((g - lo.y) * k)
		for x in size:
			img.set_pixel(x, y, Color(0.2, 0.27, 0.18))
		g += 50.0
	var zones: Array = spec.get("zones", [])
	var length := float(s.length)
	var dist: PackedFloat32Array = s.dist
	for i in pos.size():
		var u := dist[i] / length
		var in_zone := false
		for z: Dictionary in zones:
			if u >= float(z.start) and u <= float(z.end):
				in_zone = true
		var c := Color(0.45, 0.45, 0.48) if in_zone else Color(0.36, 0.36, 0.38)
		_disc(img, Vector2(pos[i].x - lo.x, pos[i].z - lo.y) * k, width[i] * 0.5 * k, c)
	for i in pos.size():
		_disc(img, Vector2(pos[i].x - lo.x, pos[i].z - lo.y) * k, maxf(1.0, 0.6 * k), Color(0.85, 0.85, 0.8))
	for c: Dictionary in spec.get("clips", []):
		var i := clampi(int(float(c.t) * pos.size()), 0, pos.size() - 1)
		var side := 1.0 if str(c.side) == "right" else -1.0
		var r: Vector3 = s.right[i]
		var p := pos[i] + r * side * width[i] * 0.5
		_disc(img, Vector2(p.x - lo.x, p.z - lo.y) * k, maxf(3.0, 2.0 * k), Color(0.9, 0.15, 0.15))
	var st := clampi(int(float(spec.get("start", 0.0)) * pos.size()), 0, pos.size() - 1)
	_disc(img, Vector2(pos[st].x - lo.x, pos[st].z - lo.y) * k, maxf(4.0, 3.0 * k), Color(0.2, 0.9, 0.3))
	img.save_png(args[1])
	print("OK   %s (%.0f x %.0f m, grid 50 m)" % [args[1], hi.x - lo.x, hi.y - lo.y])
	quit(0)


func _disc(img: Image, c: Vector2, r: float, col: Color) -> void:
	var ri := int(ceil(r))
	for dy in range(-ri, ri + 1):
		for dx in range(-ri, ri + 1):
			if dx * dx + dy * dy <= r * r:
				var x := int(c.x) + dx
				var y := int(c.y) + dy
				if x >= 0 and y >= 0 and x < img.get_width() and y < img.get_height():
					img.set_pixel(x, y, col)
