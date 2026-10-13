class_name Landscape
extends RefCounted
## Surroundings for "forest" style tracks (e.g. ones recorded from Forza):
##   - a grass terrain that follows the road: right under the road and its
##     shoulders it sits just below them (continuing the camber), further out
##     it rolls gently, so the track never floats above a flat plane;
##   - cut-out trees (pines and leafy trees) on crossed quads, in natural
##     clusters outside the walls and clear of every section of road. All trees
##     of a kind are merged into one mesh so a few hundred cost almost nothing.

const CELL := 4.0          # terrain grid (m)
const MARGIN := 160.0      # terrain beyond the track bounds (m)
const HASH := 16.0         # centreline spatial hash cell (m)


## Index of the centreline samples for nearest-sample queries.
static func _index(s: Dictionary) -> Dictionary:
	var g := {}
	var pos: PackedVector3Array = s.pos
	for i in pos.size():
		var k := Vector2i(floori(pos[i].x / HASH), floori(pos[i].z / HASH))
		if not g.has(k):
			g[k] = []
		(g[k] as Array).append(i)
	return g


## Nearest centreline sample to (x, z) within ~MARGIN, or -1.
static func _nearest(s: Dictionary, g: Dictionary, x: float, z: float) -> int:
	var pos: PackedVector3Array = s.pos
	var key := Vector2i(floori(x / HASH), floori(z / HASH))
	var best := -1
	var bd := INF
	var max_r := int(ceil(MARGIN / HASH)) + 1
	for r in max_r + 1:
		for dx in range(-r, r + 1):
			for dz in range(-r, r + 1):
				if maxi(absi(dx), absi(dz)) != r:
					continue
				for i: int in g.get(key + Vector2i(dx, dz), []):
					var d := Vector2(pos[i].x - x, pos[i].z - z).length_squared()
					if d < bd:
						bd = d
						best = i
		if best >= 0 and sqrt(bd) < (r - 1) * HASH:
			break
	return best


## Ground height at (x, z): just under the road/shoulders near the track
## (following its camber), gently rolling further away.
static func height_at(s: Dictionary, spec: Dictionary, g: Dictionary, noise: FastNoiseLite, x: float, z: float) -> float:
	var pos: PackedVector3Array = s.pos
	var right: PackedVector3Array = s.right
	var width: PackedFloat32Array = s.width
	var bank: PackedFloat32Array = s.get("bank", PackedFloat32Array())
	var runoff := float(spec.runoff) + 1.0
	var i := _nearest(s, g, x, z)
	if i < 0:
		return 0.0
	var p := Vector3(x, 0, z)
	var lat := (p - Vector3(pos[i].x, 0, pos[i].z)).dot(right[i])
	var half := width[i] * 0.5 + runoff
	var b := bank[i] if i < bank.size() else 0.0
	var base := pos[i].y + b * clampf(lat, -half, half)
	var h: float
	var e := absf(lat) - half
	if e <= 0.6:
		h = base - 0.35
	else:
		h = base - 0.06 + noise.get_noise_2d(x, z) * 5.0 * smoothstep(0.0, 45.0, e)
	# Never poke through another section of road (hairpins, switchbacks).
	var key := Vector2i(floori(x / HASH), floori(z / HASH))
	for dx in range(-2, 3):
		for dz in range(-2, 3):
			for j: int in g.get(key + Vector2i(dx, dz), []):
				var lj := (p - Vector3(pos[j].x, 0, pos[j].z)).dot(right[j])
				var hj := width[j] * 0.5 + runoff + 0.6
				if absf(lj) <= hj and Vector2(pos[j].x - x, pos[j].z - z).length() < hj + 2.0:
					var bj := bank[j] if j < bank.size() else 0.0
					h = minf(h, pos[j].y + bj * clampf(lj, -hj, hj) - 0.35)
	return h


static func _noise(seed: int) -> FastNoiseLite:
	var n := FastNoiseLite.new()
	n.seed = seed
	n.frequency = 0.012
	n.fractal_octaves = 3
	return n


## Terrain body named 1GRASS_ground (AC grass surface) with collision.
static func build_terrain(root: Node3D, spec: Dictionary, s: Dictionary, mat: Material) -> void:
	var pos: PackedVector3Array = s.pos
	var lo := Vector2(INF, INF)
	var hi := Vector2(-INF, -INF)
	for p in pos:
		lo = Vector2(minf(lo.x, p.x), minf(lo.y, p.z))
		hi = Vector2(maxf(hi.x, p.x), maxf(hi.y, p.z))
	lo -= Vector2.ONE * MARGIN
	hi += Vector2.ONE * MARGIN
	var nx := int(ceil((hi.x - lo.x) / CELL)) + 1
	var nz := int(ceil((hi.y - lo.y) / CELL)) + 1
	var g := _index(s)
	var noise := _noise(int((spec.scenery as Dictionary).get("seed", 1)))
	var hs := PackedFloat32Array()
	hs.resize(nx * nz)
	for iz in nz:
		for ix in nx:
			hs[iz * nx + ix] = height_at(s, spec, g, noise, lo.x + ix * CELL, lo.y + iz * CELL)
	var st := SurfaceTool.new()
	st.begin(Mesh.PRIMITIVE_TRIANGLES)
	for iz in nz:
		for ix in nx:
			var x := lo.x + ix * CELL
			var z := lo.y + iz * CELL
			st.set_uv(Vector2(x, z) / 4.0)
			st.add_vertex(Vector3(x, hs[iz * nx + ix], z))
	for iz in nz - 1:
		for ix in nx - 1:
			var a := iz * nx + ix
			var b := a + 1
			var c := a + nx
			var d := c + 1
			# Wound so the faces point up (Godot front = clockwise from above).
			st.add_index(a); st.add_index(b); st.add_index(c)
			st.add_index(b); st.add_index(d); st.add_index(c)
	st.generate_normals()
	var mesh := st.commit()
	var body := StaticBody3D.new()
	body.name = "1GRASS_ground"
	body.set_meta("surface", "grass")
	body.collision_layer = 1
	body.collision_mask = 0
	var mi := MeshInstance3D.new()
	mi.name = body.name
	mi.mesh = mesh
	mi.material_override = mat
	body.add_child(mi)
	var cs := CollisionShape3D.new()
	var shape := mesh.create_trimesh_shape()
	shape.backface_collision = true
	cs.shape = shape
	body.add_child(cs)
	root.add_child(body)


## Trees in clusters outside the walls, clear of every section of road.
static func build_trees(root: Node3D, spec: Dictionary, s: Dictionary) -> int:
	var sc: Dictionary = spec.scenery
	var rng := RandomNumberGenerator.new()
	rng.seed = int(sc.get("seed", 1)) * 7919 + 13
	var density := float(sc.get("density", 1.0))
	var pos: PackedVector3Array = s.pos
	var right: PackedVector3Array = s.right
	var width: PackedFloat32Array = s.width
	var n := pos.size()
	var g := _index(s)
	var noise := _noise(int(sc.get("seed", 1)))
	var clear := float(spec.runoff) + 5.0
	var placed := []   # Vector3 positions (spacing check)
	var kinds := [[], []]   # pine, leafy: [pos, height, yaw]
	var length := float(s.get("length", n * 2.0))
	var clusters := int(length / 35.0 * density)
	for c in clusters:
		var i := rng.randi_range(0, n - 1)
		var side := -1.0 if rng.randf() < 0.5 else 1.0
		var centre := pos[i] + right[i] * side * (width[i] * 0.5 + clear + rng.randf_range(4.0, 55.0))
		var count := rng.randi_range(3, 11)
		var pine_bias := rng.randf()
		for k in count:
			var p := centre + Vector3(rng.randf_range(-14, 14), 0, rng.randf_range(-14, 14))
			var j := _nearest(s, g, p.x, p.z)
			if j < 0:
				continue
			var lat := absf((Vector3(p.x, 0, p.z) - Vector3(pos[j].x, 0, pos[j].z)).dot(right[j]))
			if lat < width[j] * 0.5 + clear or Vector2(p.x - pos[j].x, p.z - pos[j].z).length() < width[j] * 0.5 + clear:
				continue
			var crowded := false
			for q: Vector3 in placed:
				if Vector2(q.x - p.x, q.z - p.z).length() < 3.5:
					crowded = true
					break
			if crowded:
				continue
			p.y = height_at(s, spec, g, noise, p.x, p.z)
			placed.append(p)
			var kind := 0 if rng.randf() < pine_bias else 1
			(kinds[kind] as Array).append([p, rng.randf_range(8.0, 15.0) if kind == 0 else rng.randf_range(7.0, 11.0), rng.randf() * TAU])
	for kind in 2:
		var list: Array = kinds[kind]
		if list.is_empty():
			continue
		var mi := MeshInstance3D.new()
		mi.name = "trees_pine" if kind == 0 else "trees_leafy"
		mi.mesh = _tree_mesh(list, 0.42 if kind == 0 else 0.7)
		var m := StandardMaterial3D.new()
		m.albedo_texture = ImageTexture.create_from_image(tree_image(kind))
		m.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA_SCISSOR
		m.alpha_scissor_threshold = 0.5
		m.cull_mode = BaseMaterial3D.CULL_DISABLED
		m.roughness = 0.9
		m.set_meta("export_name", "tree_pine" if kind == 0 else "tree_leafy")
		mi.material_override = m
		root.add_child(mi)
	return placed.size()


## Two crossed quads per tree, both faces (each with its own normal so the
## AC export keeps both sides visible).
static func _tree_mesh(list: Array, aspect: float) -> ArrayMesh:
	var st := SurfaceTool.new()
	st.begin(Mesh.PRIMITIVE_TRIANGLES)
	for t: Array in list:
		var p: Vector3 = t[0]
		var h: float = t[1]
		var yaw: float = t[2]
		var w := h * aspect
		for q in 2:
			var a := yaw + q * PI * 0.5
			var dir := Vector3(cos(a), 0, sin(a)) * (w * 0.5)
			var nrm := Vector3(-sin(a), 0.35, cos(a)).normalized()
			var v0 := p - dir + Vector3.DOWN * 0.3
			var v1 := p + dir + Vector3.DOWN * 0.3
			var v2 := p + dir + Vector3.UP * h
			var v3 := p - dir + Vector3.UP * h
			for side in [1.0, -1.0]:
				var nn := Vector3(nrm.x * side, nrm.y, nrm.z * side)
				var quad := [[v0, Vector2(0, 1)], [v1, Vector2(1, 1)], [v2, Vector2(1, 0)], [v0, Vector2(0, 1)], [v2, Vector2(1, 0)], [v3, Vector2(0, 0)]]
				if side < 0.0:
					quad = [quad[0], quad[2], quad[1], quad[3], quad[5], quad[4]]
				for vtx: Array in quad:
					st.set_normal(nn)
					st.set_uv(vtx[1])
					st.add_vertex(vtx[0])
	return st.commit()


static var _tree_cache := {}


## Procedural cut-out tree texture (RGBA, alpha = leaves/trunk).
## kind 0 = pine, 1 = leafy.
static func tree_image(kind: int) -> Image:
	if _tree_cache.has(kind):
		return _tree_cache[kind]
	var w := 256
	var h := 512
	var img := Image.create(w, h, false, Image.FORMAT_RGBA8)
	img.fill(Color(0.18, 0.3, 0.12, 0.0))
	var nz := FastNoiseLite.new()
	nz.seed = 5 + kind
	nz.frequency = 0.08
	nz.fractal_octaves = 3
	var cx := w * 0.5
	# Trunk.
	var trunk_top := h * (0.55 if kind == 1 else 0.75)
	for y in range(int(trunk_top), h):
		var tw := 7.0 + (y - trunk_top) / (h - trunk_top) * 5.0
		for x in range(int(cx - tw), int(cx + tw) + 1):
			var shade := 0.75 + 0.25 * nz.get_noise_2d(x * 3.0, y * 0.5)
			img.set_pixel(x, y, Color(0.30 * shade, 0.21 * shade, 0.13 * shade, 1.0))
	# Foliage.
	for y in h:
		for x in w:
			var inside := false
			var depth := 0.0
			if kind == 0:
				# Stacked tiers of a pine.
				var t := float(y) / (h * 0.9)
				if t < 1.0:
					var tier := fposmod(t * 6.0, 1.0)
					var half := (0.08 + 0.42 * t) * w * (0.55 + 0.45 * tier)
					var dxp := absf(x - cx)
					var edge := half * (0.85 + 0.25 * nz.get_noise_2d(x, y))
					inside = dxp < edge and y > 6
					depth = dxp / maxf(edge, 1.0)
			else:
				# Lumpy round crown.
				var cy := h * 0.36
				var d := Vector2((x - cx) / (w * 0.47), (y - cy) / (h * 0.34)).length()
				var edge := 0.92 + 0.22 * nz.get_noise_2d(x * 1.5, y * 1.5)
				inside = d < edge
				depth = d / edge
			if inside:
				var n := nz.get_noise_2d(x * 2.0, y * 2.0)
				var light := clampf(0.7 + 0.35 * n - 0.25 * depth + (0.15 if x < cx else 0.0), 0.3, 1.1)
				var base := Color(0.11, 0.24, 0.10) if kind == 0 else Color(0.20, 0.33, 0.11)
				img.set_pixel(x, y, Color(base.r * light, base.g * light, base.b * light, 1.0))
	_tree_cache[kind] = img
	return img
