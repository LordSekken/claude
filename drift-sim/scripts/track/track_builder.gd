class_name TrackBuilder
extends RefCounted
## Builds a playable 3D track (meshes + collision + scenery) from a TrackSpec.
## Mesh node names follow Assetto Corsa's physical-surface convention
## (1ROAD, 1KERB, 1GRASS, 1WALL...) so the scene can be exported for AC.

const SURFACE_COLORS := {
	"asphalt": Color(0.20, 0.20, 0.22),
	"concrete": Color(0.55, 0.55, 0.53),
	"grass": Color(0.15, 0.26, 0.11),
	"gravel": Color(0.55, 0.50, 0.42),
	"dirt": Color(0.42, 0.32, 0.22),
	"sand": Color(0.76, 0.68, 0.50),
}
const AC_NAMES := {
	"asphalt": "ROAD", "concrete": "ROAD", "kerb": "KERB", "grass": "GRASS",
	"gravel": "GRAVEL", "dirt": "DIRT", "sand": "SAND",
}


## Returns {root: Node3D, samples: Dictionary, spec: Dictionary}.
static func build(spec_in: Dictionary) -> Dictionary:
	var spec := TrackSpec.normalized(spec_in)
	var root := Node3D.new()
	root.name = "Track"
	var samples := TrackSpec.sample(spec, 2.0)
	var counter := {"n": 0}
	var has_line := (samples.pos as PackedVector3Array).size() >= 2
	var min_y := 0.0
	if has_line:
		for p: Vector3 in samples.pos:
			min_y = minf(min_y, p.y)
		_build_road(root, spec, samples, counter)
	for pad: Dictionary in spec.pads:
		_build_pad(root, pad, counter)
	_build_ground(root, min_y - 0.03)
	if has_line:
		_build_markers(root, spec, samples)
		_build_scenery(root, spec, samples)
	_build_environment(root, str(spec.scenery.get("time_of_day", "day")))
	if has_line:
		var start := TrackSpec.transform_at(samples, float(spec.start), 0.0)
		var marker := Node3D.new()
		marker.name = "AC_START_0"
		marker.transform = start
		root.add_child(marker)
	return {"root": root, "samples": samples, "spec": spec}


static func _mat(color: Color, rough := 0.85, vertex_color := false) -> StandardMaterial3D:
	var m := StandardMaterial3D.new()
	m.albedo_color = color
	m.roughness = rough
	m.cull_mode = BaseMaterial3D.CULL_DISABLED
	if vertex_color:
		m.vertex_color_use_as_albedo = true
	return m


static func _asphalt_mat() -> StandardMaterial3D:
	var m := _mat(Color(1, 1, 1), 0.9)
	var noise := FastNoiseLite.new()
	noise.frequency = 0.35
	var tex := NoiseTexture2D.new()
	tex.noise = noise
	tex.width = 256
	tex.height = 256
	tex.seamless = true
	tex.color_ramp = Gradient.new()
	tex.color_ramp.set_color(0, Color(0.16, 0.16, 0.17))
	tex.color_ramp.set_color(1, Color(0.27, 0.27, 0.28))
	m.albedo_texture = tex
	m.uv1_scale = Vector3(0.25, 0.25, 1)
	return m


static func _add_surface(root: Node3D, mesh: ArrayMesh, mat: Material, surface: String, counter: Dictionary, is_wall := false) -> void:
	if mesh.get_surface_count() == 0:
		return
	var body := StaticBody3D.new()
	var prefix := "WALL" if is_wall else str(AC_NAMES.get(surface, "ROAD"))
	counter.n = int(counter.n) + 1
	body.name = "1%s_%d" % [prefix, counter.n]
	body.set_meta("surface", surface)
	if is_wall:
		body.set_meta("wall", true)
		body.collision_layer = 2
	else:
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


## Generic strip between two offset curves (offsets in metres from centreline).
static func _strip(samples: Dictionary, off_a: PackedFloat32Array, off_b: PackedFloat32Array, y_a := 0.0, y_b := 0.0, mask := PackedByteArray(), colors := PackedColorArray()) -> ArrayMesh:
	var pos: PackedVector3Array = samples.pos
	var dist: PackedFloat32Array = samples.dist
	var n := pos.size()
	var closed := bool(samples.closed)
	var verts := PackedVector3Array()
	var normals := PackedVector3Array()
	var uvs := PackedVector2Array()
	var cols := PackedColorArray()
	var segs := n if closed else n - 1
	for i in segs:
		if mask.size() > 0 and (mask[i] == 0 or mask[(i + 1) % n] == 0):
			continue
		var j := (i + 1) % n
		var a0 := _pt(samples, i, off_a[i]) + Vector3.UP * y_a
		var b0 := _pt(samples, i, off_b[i]) + Vector3.UP * y_b
		var a1 := _pt(samples, j, off_a[j]) + Vector3.UP * y_a
		var b1 := _pt(samples, j, off_b[j]) + Vector3.UP * y_b
		var v0 := dist[i]
		var v1 := dist[i] + pos[i].distance_to(pos[j])
		var quad := [a0, b0, b1, a0, b1, a1]
		var quv := [Vector2(off_a[i], v0), Vector2(off_b[i], v0), Vector2(off_b[j], v1), Vector2(off_a[i], v0), Vector2(off_b[j], v1), Vector2(off_a[j], v1)]
		# Wind so the face points up regardless of which side is "a".
		var nrm := (b0 - a0).cross(a1 - a0)
		if nrm.y < 0.0:
			quad = [a0, b1, b0, a0, a1, b1]
			quv = [quv[0], quv[2], quv[1], quv[3], quv[5], quv[4]]
		nrm = nrm.normalized() * signf(nrm.y) if nrm.length() > 0 else Vector3.UP
		for k in 6:
			verts.append(quad[k])
			normals.append(nrm)
			uvs.append(quv[k])
			if colors.size() > 0:
				cols.append(colors[i])
	var mesh := ArrayMesh.new()
	if verts.is_empty():
		return mesh
	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = verts
	arrays[Mesh.ARRAY_NORMAL] = normals
	arrays[Mesh.ARRAY_TEX_UV] = uvs
	if cols.size() > 0:
		arrays[Mesh.ARRAY_COLOR] = cols
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	return mesh


## Point `off` metres to the right of centreline sample i, following the
## road's crossfall (kerbs, run-off and walls continue the same plane).
static func _pt(s: Dictionary, i: int, off: float) -> Vector3:
	var bank: PackedFloat32Array = s.get("bank", PackedFloat32Array())
	var b := bank[i] if i < bank.size() else 0.0
	return (s.pos as PackedVector3Array)[i] + (s.right as PackedVector3Array)[i] * off + Vector3.UP * (off * b)


static func _const(n: int, v: float) -> PackedFloat32Array:
	var a := PackedFloat32Array()
	a.resize(n)
	a.fill(v)
	return a


static func _build_road(root: Node3D, spec: Dictionary, s: Dictionary, counter: Dictionary) -> void:
	var n: int = (s.pos as PackedVector3Array).size()
	var width: PackedFloat32Array = s.width
	var half_l := PackedFloat32Array()
	var half_r := PackedFloat32Array()
	for i in n:
		half_l.append(-width[i] * 0.5)
		half_r.append(width[i] * 0.5)
	_add_surface(root, _strip(s, half_l, half_r), _asphalt_mat(), "asphalt", counter)

	# Painted edge lines (visual only).
	var line_mat := _mat(Color(0.9, 0.9, 0.88), 0.6)
	for side in [-1.0, 1.0]:
		var a := PackedFloat32Array()
		var b := PackedFloat32Array()
		for i in n:
			a.append(side * (width[i] * 0.5 - 0.45))
			b.append(side * (width[i] * 0.5 - 0.30))
		var mi := MeshInstance3D.new()
		mi.mesh = _strip(s, a, b, 0.012, 0.012)
		mi.material_override = line_mat
		root.add_child(mi)

	# Kerbs on the inside (and outside) of corners.
	var curv: PackedFloat32Array = s.curvature
	var kerb_w := 1.0
	if bool(spec.kerbs):
		var kerb_mat := _mat(Color.WHITE, 0.7, true)
		var ranges: Array = spec.get("kerb_ranges", [])
		var total_len := float(s.length)
		for side in [-1.0, 1.0]:
			var mask := PackedByteArray()
			var cols := PackedColorArray()
			var a := PackedFloat32Array()
			var b := PackedFloat32Array()
			for i in n:
				var on := false
				if ranges.is_empty():
					on = absf(curv[i]) > 1.0 / 70.0
				else:
					var f := float(s.dist[i]) / maxf(total_len, 1.0)
					for r: Dictionary in ranges:
						if (str(r.get("side", "")) == "right") == (side > 0.0) and f >= float(r.get("start", 0.0)) and f <= float(r.get("end", 0.0)):
							on = true
							break
				mask.append(1 if on else 0)
				a.append(side * width[i] * 0.5)
				b.append(side * (width[i] * 0.5 + kerb_w))
				cols.append(Color(0.85, 0.1, 0.1) if int(float(s.dist[i]) / 2.0) % 2 == 0 else Color(0.95, 0.95, 0.95))
			# Grow auto masks a little so kerbs start before the corner
			# (recorded ranges are already padded).
			var grown := mask.duplicate()
			for i in n:
				if mask[i] == 1 and ranges.is_empty():
					for d in range(-3, 4):
						var j := i + d
						if s.closed:
							j = (j + n) % n
						if j >= 0 and j < n:
							grown[j] = 1
			_add_surface(root, _strip(s, a, b, 0.0, 0.04, grown, cols), kerb_mat, "kerb", counter)

	# Run-off both sides.
	var runoff := float(spec.runoff)
	var surf := str(spec.runoff_surface)
	if runoff > 0.05:
		for side in [-1.0, 1.0]:
			var a := PackedFloat32Array()
			var b := PackedFloat32Array()
			for i in n:
				a.append(side * width[i] * 0.5)
				b.append(side * (width[i] * 0.5 + runoff + (kerb_w if bool(spec.kerbs) else 0.0)))
			var mat := _asphalt_mat() if surf == "asphalt" else _mat(SURFACE_COLORS.get(surf, SURFACE_COLORS.grass))
			_add_surface(root, _strip(s, a, b, -0.01, -0.01), mat, surf, counter)

	# Walls.
	var walls: Dictionary = spec.walls
	var wall_h := float(walls.get("height", 1.2))
	var wall_mat := _mat(Color(0.62, 0.62, 0.6), 0.9)
	var edge := runoff + (kerb_w if bool(spec.kerbs) else 0.0)
	for side_name in ["left", "right"]:
		if not bool(walls.get(side_name, false)):
			continue
		var side := -1.0 if side_name == "left" else 1.0
		var inner := PackedFloat32Array()
		var outer := PackedFloat32Array()
		for i in n:
			inner.append(side * (width[i] * 0.5 + edge))
			outer.append(side * (width[i] * 0.5 + edge + 0.4))
		_add_surface(root, _wall_mesh(s, inner, outer, wall_h), wall_mat, "concrete", counter, true)


static func _wall_mesh(s: Dictionary, inner: PackedFloat32Array, outer: PackedFloat32Array, h: float) -> ArrayMesh:
	# One-sided faces with proper normals (inner face toward the road, outer
	# face away, top up); collision uses backface_collision so it's still solid.
	var st := SurfaceTool.new()
	st.begin(Mesh.PRIMITIVE_TRIANGLES)
	var pos: PackedVector3Array = s.pos
	var right: PackedVector3Array = s.right
	var n := pos.size()
	var segs := n if s.closed else n - 1
	for i in segs:
		var j := (i + 1) % n
		var side := signf(inner[i]) if inner[i] != 0.0 else 1.0
		var i0 := _pt(s, i, inner[i])
		var i1 := _pt(s, j, inner[j])
		var o0 := _pt(s, i, outer[i])
		var o1 := _pt(s, j, outer[j])
		var up := Vector3.UP * h
		var low := Vector3.DOWN * 0.3
		var out_dir := ((right[i] + right[j]) * 0.5 * side).normalized()
		for face: Array in [[[i0 + low, i1 + low, i1 + up, i0 + up], -out_dir], [[o0 + low, o1 + low, o1 + up, o0 + up], out_dir], [[i0 + up, i1 + up, o1 + up, o0 + up], Vector3.UP]]:
			var q: Array = face[0]
			var want: Vector3 = face[1]
			# Godot front faces wind clockwise seen from the normal side.
			var tri := [q[0], q[1], q[2], q[0], q[2], q[3]]
			if ((q[1] as Vector3) - q[0]).cross((q[2] as Vector3) - q[0]).dot(want) > 0.0:
				tri = [q[0], q[2], q[1], q[0], q[3], q[2]]
			for v: Vector3 in tri:
				st.set_normal(want)
				st.set_uv(Vector2(0, 0))
				st.add_vertex(v)
	return st.commit()


static func _build_pad(root: Node3D, pad: Dictionary, counter: Dictionary) -> void:
	var poly := PackedVector2Array()
	for p: Variant in pad.get("points", []):
		poly.append(Vector2(float(p[0]), float(p[1])))
	if poly.size() < 3:
		return
	var idx := Geometry2D.triangulate_polygon(poly)
	if idx.is_empty():
		return
	var y := float(pad.get("y", 0.0)) - 0.005
	var verts := PackedVector3Array()
	var uvs := PackedVector2Array()
	var normals := PackedVector3Array()
	for k in range(0, idx.size(), 3):
		var tri := [poly[idx[k]], poly[idx[k + 1]], poly[idx[k + 2]]]
		var cross: float = (tri[1] - tri[0]).cross(tri[2] - tri[0])
		if cross > 0.0:  # make faces point up (+Y) in XZ space
			tri = [tri[0], tri[2], tri[1]]
		for v: Vector2 in tri:
			verts.append(Vector3(v.x, y, v.y))
			uvs.append(v)
			normals.append(Vector3.UP)
	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = verts
	arrays[Mesh.ARRAY_TEX_UV] = uvs
	arrays[Mesh.ARRAY_NORMAL] = normals
	var mesh := ArrayMesh.new()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	var surf := str(pad.get("surface", "asphalt"))
	var mat := _asphalt_mat() if surf == "asphalt" else _mat(SURFACE_COLORS.get(surf, SURFACE_COLORS.concrete))
	_add_surface(root, mesh, mat, surf, counter)


static func _build_ground(root: Node3D, y: float) -> void:
	var body := StaticBody3D.new()
	body.name = "1GRASS_ground"
	body.set_meta("surface", "grass")
	body.collision_layer = 1
	body.collision_mask = 0
	var cs := CollisionShape3D.new()
	var box := BoxShape3D.new()
	box.size = Vector3(6000, 2, 6000)
	cs.shape = box
	cs.position = Vector3(0, y - 1.0, 0)
	body.add_child(cs)
	var mi := MeshInstance3D.new()
	var pm := PlaneMesh.new()
	pm.size = Vector2(6000, 6000)
	mi.mesh = pm
	mi.position = Vector3(0, y, 0)
	mi.material_override = _mat(SURFACE_COLORS.grass, 0.95)
	body.add_child(mi)
	root.add_child(body)


static func _build_markers(root: Node3D, spec: Dictionary, s: Dictionary) -> void:
	var pos: PackedVector3Array = s.pos
	var n := pos.size()
	# Start/finish line.
	var i0 := clampi(int(float(spec.start) * n), 0, n - 1)
	var line := MeshInstance3D.new()
	var bm := BoxMesh.new()
	bm.size = Vector3(float(s.width[i0]), 0.02, 1.2)
	line.mesh = bm
	line.material_override = _mat(Color(0.95, 0.95, 0.95), 0.5)
	line.transform = TrackSpec.transform_at(s, float(spec.start), 0.01)
	root.add_child(line)
	# Clipping points: tall poles plus a ground ring.
	var clip_root := Node3D.new()
	clip_root.name = "Clips"
	root.add_child(clip_root)
	for c: Dictionary in spec.clips:
		var i := clampi(int(fposmod(float(c.t), 1.0) * n), 0, n - 1)
		var side := -1.0 if str(c.side) == "left" else 1.0
		var p: Vector3 = _pt(s, i, side * (float(s.width[i]) * 0.5 - 0.5))
		var marker := Node3D.new()
		marker.position = p
		marker.set_meta("clip", c)
		clip_root.add_child(marker)
		var pole := MeshInstance3D.new()
		var cm := CylinderMesh.new()
		cm.top_radius = 0.08
		cm.bottom_radius = 0.08
		cm.height = 2.2
		pole.mesh = cm
		pole.position.y = 1.1
		var pm := _mat(Color(1.0, 0.85, 0.0), 0.4)
		pm.emission_enabled = true
		pm.emission = Color(1.0, 0.7, 0.0)
		pole.material_override = pm
		marker.add_child(pole)
		var ring := MeshInstance3D.new()
		var tm := TorusMesh.new()
		var r := float(c.get("radius", 2.0))
		tm.inner_radius = r - 0.12
		tm.outer_radius = r
		ring.mesh = tm
		ring.scale = Vector3(1, 0.05, 1)
		ring.position.y = 0.03
		ring.material_override = pm
		marker.add_child(ring)


static func _build_scenery(root: Node3D, spec: Dictionary, s: Dictionary) -> void:
	var sc: Dictionary = spec.scenery
	var rng := RandomNumberGenerator.new()
	rng.seed = int(sc.get("seed", 1))
	var density := float(sc.get("density", 1.0))
	var pos: PackedVector3Array = s.pos
	var right: PackedVector3Array = s.right
	var n := pos.size()
	var clear := float(spec.runoff) + 6.0
	# Spatial hash of the centreline to keep scenery off the road.
	var grid := {}
	for i in n:
		var k := Vector2i(floori(pos[i].x / 20.0), floori(pos[i].z / 20.0))
		grid[k] = true

	var trunk_mat := _mat(Color(0.3, 0.2, 0.12))
	var leaf_mat := _mat(Color(0.12, 0.3, 0.12))
	var bld_mat := _mat(Color(0.5, 0.5, 0.52))
	var lamp_mat := _mat(Color(0.3, 0.3, 0.32), 0.5)
	var leaves := _cone_mesh(3.0, 7.0)
	var trunk := CylinderMesh.new()
	trunk.top_radius = 0.25
	trunk.bottom_radius = 0.3
	trunk.height = 2.0
	var count := int(n * 0.25 * density)
	for k in count:
		var i := rng.randi_range(0, n - 1)
		var side := -1.0 if rng.randf() < 0.5 else 1.0
		var off := float(s.width[i]) * 0.5 + clear + rng.randf_range(4.0, 60.0)
		var p := pos[i] + right[i] * side * off
		if _near_track(p, pos, float(s.width[i]) * 0.5 + clear, grid):
			continue
		if rng.randf() < 0.8:
			var t := MeshInstance3D.new()
			t.mesh = trunk
			t.material_override = trunk_mat
			t.position = p + Vector3.UP * 1.0
			root.add_child(t)
			var l := MeshInstance3D.new()
			l.mesh = leaves
			l.material_override = leaf_mat
			l.position = p + Vector3.UP * 5.0
			l.scale = Vector3.ONE * rng.randf_range(0.7, 1.4)
			root.add_child(l)
		else:
			var b := MeshInstance3D.new()
			var bm := BoxMesh.new()
			bm.size = Vector3(rng.randf_range(8, 25), rng.randf_range(5, 18), rng.randf_range(8, 25))
			b.mesh = bm
			b.material_override = bld_mat
			b.position = p + Vector3.UP * bm.size.y * 0.5
			b.rotation.y = rng.randf() * TAU
			root.add_child(b)
	# Light poles along the edge every ~40 m.
	var night := str(sc.get("time_of_day", "day")) != "day"
	var pole_mesh := CylinderMesh.new()
	pole_mesh.top_radius = 0.1
	pole_mesh.bottom_radius = 0.14
	pole_mesh.height = 9.0
	var step := maxi(int(40.0 / float(s.spacing)), 1)
	for i in range(0, n, step):
		var side := 1.0 if (i / step) % 2 == 0 else -1.0
		var p := _pt(s, i, side * (float(s.width[i]) * 0.5 + clear - 3.0))
		if _near_track(p, pos, float(s.width[i]) * 0.5 + 1.5, grid):
			continue
		var pole := MeshInstance3D.new()
		pole.mesh = pole_mesh
		pole.material_override = lamp_mat
		pole.position = p + Vector3.UP * 4.5
		root.add_child(pole)
		if night:
			var light := OmniLight3D.new()
			light.position = p + Vector3.UP * 8.5 - right[i] * side * 2.0
			light.omni_range = 30.0
			light.light_energy = 2.0
			light.light_color = Color(1.0, 0.85, 0.65)
			root.add_child(light)


static func _cone_mesh(radius: float, height: float) -> CylinderMesh:
	var c := CylinderMesh.new()
	c.top_radius = 0.0
	c.bottom_radius = radius
	c.height = height
	return c


static func _near_track(p: Vector3, pos: PackedVector3Array, min_d: float, grid: Dictionary) -> bool:
	var key := Vector2i(floori(p.x / 20.0), floori(p.z / 20.0))
	var r := int(ceil(min_d / 20.0)) + 1
	var near_cell := false
	for dx in range(-r, r + 1):
		for dz in range(-r, r + 1):
			if grid.has(key + Vector2i(dx, dz)):
				near_cell = true
	if not near_cell:
		return false
	for q in pos:
		if Vector2(q.x - p.x, q.z - p.z).length() < min_d:
			return true
	return false


static func _build_environment(root: Node3D, tod: String) -> void:
	var env := Environment.new()
	env.background_mode = Environment.BG_SKY
	var sky := Sky.new()
	var sm := ProceduralSkyMaterial.new()
	var sun := DirectionalLight3D.new()
	sun.shadow_enabled = true
	sun.directional_shadow_max_distance = 250.0
	match tod:
		"night":
			sm.sky_top_color = Color(0.01, 0.01, 0.04)
			sm.sky_horizon_color = Color(0.05, 0.05, 0.1)
			sm.ground_horizon_color = Color(0.03, 0.03, 0.05)
			sun.light_energy = 0.08
			sun.rotation_degrees = Vector3(-50, 30, 0)
			env.ambient_light_energy = 0.3
		"dusk":
			sm.sky_top_color = Color(0.2, 0.25, 0.5)
			sm.sky_horizon_color = Color(0.95, 0.55, 0.3)
			sun.light_color = Color(1.0, 0.7, 0.45)
			sun.light_energy = 0.8
			sun.rotation_degrees = Vector3(-12, 60, 0)
		_:
			sun.light_energy = 1.2
			sun.rotation_degrees = Vector3(-55, 35, 0)
	sky.sky_material = sm
	env.sky = sky
	env.ambient_light_source = Environment.AMBIENT_SOURCE_SKY
	env.tonemap_mode = Environment.TONE_MAPPER_FILMIC
	env.fog_enabled = true
	env.fog_density = 0.0015
	var we := WorldEnvironment.new()
	we.environment = env
	root.add_child(we)
	root.add_child(sun)
