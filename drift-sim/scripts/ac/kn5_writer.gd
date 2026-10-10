class_name Kn5Writer
extends RefCounted
## Writes a Godot scene as an Assetto Corsa .kn5 (version 5) so tracks can be
## dropped straight into content/tracks - no Blender / ksEditor step.
##
## Inverse of Kn5Loader: Godot (x, y, z) -> AC (-x, y, -z) (a 180 degree turn
## about Y; winding unchanged). Meshes are baked to world space and split to
## stay under the 16-bit index limit. Materials are plain ksPerPixel with
## generated textures chosen from the AC physics name (1ROAD, 1KERB, ...).

const MAX_VERTS := 65000

var _textures := {}        # name -> PackedByteArray (PNG)
var _materials: Array = [] # [{name, tex, spec}]
var _mat_index := {}       # key -> index
var _meshes: Array = []    # [{name, mat, verts, normals, uvs, indices}]
var _dummies: Array = []   # [{name, xform}]
var _obj_count := 0


static func write(root: Node3D, dummies: Array, path: String) -> Dictionary:
	var w := Kn5Writer.new()
	w._collect(root, root)
	for d: Dictionary in dummies:
		w._dummies.append(d)
	if w._meshes.is_empty():
		return {"ok": false, "error": "nothing to export"}
	var err := w._save(path)
	return {"ok": err == OK, "error": "" if err == OK else "write failed (%d)" % err, "meshes": w._meshes.size()}


static func _rel(n: Node3D, root: Node) -> Transform3D:
	var t := n.transform
	var p := n.get_parent()
	while p != null and p != root and p != root.get_parent():
		if p is Node3D:
			t = (p as Node3D).transform * t
		p = p.get_parent()
	if n == root:
		return Transform3D()
	return t


static func _surface_key(name: String) -> String:
	var up := name.to_upper()
	var m := RegEx.create_from_string("^[0-9]+([A-Z]+)").search(up)
	return m.get_string(1) if m else ""


func _collect(n: Node, root: Node3D) -> void:
	if n is MeshInstance3D and (n as MeshInstance3D).visible and (n as MeshInstance3D).mesh != null:
		var mi := n as MeshInstance3D
		var name := str(mi.name)
		var parent := mi.get_parent()
		if parent is StaticBody3D and _surface_key(str(parent.name)) != "":
			name = str(parent.name)
		elif name.begins_with("@") or name == "":
			_obj_count += 1
			name = "obj_%d" % _obj_count  # visual-only scenery
		_add_mesh(mi, name, _rel(mi, root))
	for c in n.get_children():
		_collect(c, root)


func _material_for(name: String, mi: MeshInstance3D) -> int:
	var key := _surface_key(name)
	var kind := ""
	match key:
		"ROAD": kind = "asphalt"
		"KERB": kind = "kerb"
		"GRASS": kind = "grass"
		"GRAVEL", "SAND": kind = "gravel"
		"DIRT": kind = "dirt"
		"WALL": kind = "concrete"
	var color := Color(0.6, 0.6, 0.6)
	if kind == "":
		var mat := mi.material_override
		if mat is BaseMaterial3D:
			color = (mat as BaseMaterial3D).albedo_color
		kind = "solid_" + color.to_html(false)
	if _mat_index.has(kind):
		return _mat_index[kind]
	var tex_name := kind + ".png"
	_textures[tex_name] = _make_texture(kind, color)
	_materials.append({"name": "dl_" + kind, "tex": tex_name, "spec": 0.12 if kind == "asphalt" else 0.04})
	_mat_index[kind] = _materials.size() - 1
	return _mat_index[kind]


func _make_texture(kind: String, color: Color) -> PackedByteArray:
	var img: Image
	if kind == "kerb":
		img = Image.create(8, 64, false, Image.FORMAT_RGB8)
		for y in 64:
			for x in 8:
				img.set_pixel(x, y, Color(0.8, 0.08, 0.08) if y < 32 else Color(0.95, 0.95, 0.95))
		return img.save_png_to_buffer()
	if kind.begins_with("solid_"):
		img = Image.create(4, 4, false, Image.FORMAT_RGB8)
		img.fill(color)
		return img.save_png_to_buffer()
	var base := {"asphalt": [Color(0.13, 0.13, 0.14), Color(0.26, 0.26, 0.27)], "grass": [Color(0.12, 0.22, 0.08), Color(0.22, 0.36, 0.14)],
		"gravel": [Color(0.45, 0.41, 0.34), Color(0.66, 0.6, 0.5)], "dirt": [Color(0.32, 0.24, 0.16), Color(0.46, 0.35, 0.24)],
		"concrete": [Color(0.5, 0.5, 0.48), Color(0.66, 0.66, 0.63)]}
	var cols: Array = base.get(kind, [Color(0.4, 0.4, 0.4), Color(0.6, 0.6, 0.6)])
	var noise := FastNoiseLite.new()
	noise.frequency = 0.08
	noise.fractal_octaves = 4
	var n_img := noise.get_seamless_image(256, 256)
	img = Image.create(256, 256, false, Image.FORMAT_RGB8)
	for y in 256:
		for x in 256:
			img.set_pixel(x, y, (cols[0] as Color).lerp(cols[1], n_img.get_pixel(x, y).r))
	return img.save_png_to_buffer()


func _uv_for(key: String, world: Vector3, normal: Vector3, uv: Vector2) -> Vector2:
	match key:
		"KERB":
			return Vector2(uv.x, uv.y / 4.0)  # one red/white pair every 4 m
		"WALL":
			var along := world.x + world.z
			return Vector2(along / 3.0, world.y / 3.0)
	if absf(normal.y) > 0.5:
		var s := 6.0 if key == "ROAD" else 10.0
		return Vector2(world.x / s, world.z / s)
	return Vector2((world.x + world.z) / 4.0, world.y / 4.0)


func _add_mesh(mi: MeshInstance3D, name: String, xf: Transform3D) -> void:
	var mesh := mi.mesh
	var key := _surface_key(name)
	var mat := _material_for(name, mi)
	for s in mesh.get_surface_count():
		var arr := mesh.surface_get_arrays(s)
		var v: PackedVector3Array = arr[Mesh.ARRAY_VERTEX]
		if v.is_empty():
			continue
		var nrm: PackedVector3Array = arr[Mesh.ARRAY_NORMAL] if arr[Mesh.ARRAY_NORMAL] != null else PackedVector3Array()
		var uv: PackedVector2Array = arr[Mesh.ARRAY_TEX_UV] if arr[Mesh.ARRAY_TEX_UV] != null else PackedVector2Array()
		var idx: PackedInt32Array = arr[Mesh.ARRAY_INDEX] if arr[Mesh.ARRAY_INDEX] != null else PackedInt32Array()
		if idx.is_empty():
			idx.resize(v.size())
			for i in v.size():
				idx[i] = i
		var basis_n := xf.basis.inverse().transposed()
		# Emit in chunks of whole triangles, remapping indices per chunk.
		var cv := PackedVector3Array()
		var cn := PackedVector3Array()
		var cu := PackedVector2Array()
		var ci := PackedInt32Array()
		var remap := {}
		var part := 0
		for t in range(0, idx.size() - 2, 3):
			if cv.size() + 3 > MAX_VERTS:
				_meshes.append({"name": "%s_%d" % [name, part] if part > 0 else name, "mat": mat, "verts": cv, "normals": cn, "uvs": cu, "indices": ci})
				part += 1
				cv = PackedVector3Array()
				cn = PackedVector3Array()
				cu = PackedVector2Array()
				ci = PackedInt32Array()
				remap.clear()
			for k in 3:
				var src := idx[t + k]
				if not remap.has(src):
					var wp := xf * v[src]
					var wn := (basis_n * (nrm[src] if src < nrm.size() else Vector3.UP)).normalized()
					remap[src] = cv.size()
					cv.append(wp)
					cn.append(wn)
					cu.append(_uv_for(key, wp, wn, uv[src] if src < uv.size() else Vector2.ZERO))
				ci.append(remap[src])
		if not ci.is_empty():
			_meshes.append({"name": "%s_%d" % [name, part] if part > 0 else name, "mat": mat, "verts": cv, "normals": cn, "uvs": cu, "indices": ci})


# --- Binary output -----------------------------------------------------------

func _str(f: FileAccess, s: String) -> void:
	var b := s.to_utf8_buffer()
	f.store_32(b.size())
	f.store_buffer(b)


func _matrix(f: FileAccess, t: Transform3D) -> void:
	var x := t.basis.x
	var y := t.basis.y
	var z := t.basis.z
	var o := t.origin
	for v in [x.x, -x.y, x.z, 0.0, -y.x, y.y, -y.z, 0.0, z.x, -z.y, z.z, 0.0, -o.x, o.y, -o.z, 1.0]:
		f.store_float(v)


func _save(path: String) -> int:
	var f := FileAccess.open(path, FileAccess.WRITE)
	if f == null:
		return FileAccess.get_open_error()
	f.store_buffer("sc6969".to_ascii_buffer())
	f.store_32(5)
	# Textures.
	f.store_32(_textures.size())
	for name: String in _textures:
		f.store_32(1)
		_str(f, name)
		var data: PackedByteArray = _textures[name]
		f.store_32(data.size())
		f.store_buffer(data)
	# Materials.
	f.store_32(_materials.size())
	for m: Dictionary in _materials:
		_str(f, m.name)
		_str(f, "ksPerPixel")
		f.store_8(0)   # blend: opaque
		f.store_8(0)   # alpha tested
		f.store_32(0)  # depth mode
		var props := [["ksAmbient", 0.45], ["ksDiffuse", 0.55], ["ksSpecular", float(m.spec)], ["ksSpecularEXP", 25.0], ["ksEmissive", 0.0], ["ksAlphaRef", 0.0]]
		f.store_32(props.size())
		for p: Array in props:
			_str(f, p[0])
			for i in 10:
				f.store_float(float(p[1]) if i == 0 else 0.0)
		f.store_32(1)
		_str(f, "txDiffuse")
		f.store_32(0)
		_str(f, m.tex)
	# Node tree: one root dummy with every mesh and marker as children.
	f.store_32(1)
	_str(f, "DriftLabTrack")
	f.store_32(_meshes.size() + _dummies.size())
	f.store_8(1)
	_matrix(f, Transform3D())
	for m: Dictionary in _meshes:
		f.store_32(2)
		_str(f, m.name)
		f.store_32(0)
		f.store_8(1)
		f.store_8(1)  # cast shadows
		f.store_8(1)  # visible
		f.store_8(0)  # transparent
		var verts: PackedVector3Array = m.verts
		var normals: PackedVector3Array = m.normals
		var uvs: PackedVector2Array = m.uvs
		f.store_32(verts.size())
		var center := Vector3.ZERO
		for i in verts.size():
			var p := verts[i]
			var nn := normals[i]
			center += p
			for val in [-p.x, p.y, -p.z, -nn.x, nn.y, -nn.z, uvs[i].x, uvs[i].y, 1.0, 0.0, 0.0]:
				f.store_float(val)
		center /= maxf(verts.size(), 1)
		var radius := 0.0
		for p in verts:
			radius = maxf(radius, p.distance_to(center))
		var ind: PackedInt32Array = m.indices
		f.store_32(ind.size())
		for i in ind:
			f.store_16(i)
		f.store_32(int(m.mat))
		f.store_32(0)       # layer
		f.store_float(0.0)  # lod in
		f.store_float(0.0)  # lod out (0 = always)
		for val in [-center.x, center.y, -center.z, radius]:
			f.store_float(val)
		f.store_8(1)        # renderable
	for d: Dictionary in _dummies:
		f.store_32(1)
		_str(f, str(d.name))
		f.store_32(0)
		f.store_8(1)
		_matrix(f, d.xform)
	f.close()
	return OK
