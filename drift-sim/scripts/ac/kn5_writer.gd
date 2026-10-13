class_name Kn5Writer
extends RefCounted
## Writes a Godot scene as an Assetto Corsa .kn5 (version 5) so tracks can be
## dropped straight into content/tracks - no Blender / ksEditor step.
##
## Inverse of Kn5Loader: Godot (x, y, z) -> AC (-x, y, -z) (a 180 degree turn
## about Y; winding unchanged). Meshes are baked to world space and split to
## stay under the 16-bit index limit. Materials are chosen from the AC
## physics name (1ROAD, 1KERB, ...): with the HD texture pack (hd_textures/,
## CC0 photo-scanned DDS from Poly Haven) road/grass/dirt/gravel/walls get
## ksPerPixelNM with 4K/2K diffuse + normal maps; otherwise ksPerPixel with
## small generated textures.

const MAX_VERTS := 65000

var _textures := {}        # name -> PackedByteArray (PNG or DDS)
var _materials: Array = [] # [{name, shader, tex, normal, spec, exp}]
var _hd := {}              # kind -> {tile_m} from hd_textures/manifest.json
var _hd_dir := ""
var _mat_index := {}       # key -> index
var _meshes: Array = []    # [{name, mat, verts, normals, uvs, tangents, indices}]
var _dummies: Array = []   # [{name, xform}]
var _obj_count := 0


static func write(root: Node3D, dummies: Array, path: String) -> Dictionary:
	var w := Kn5Writer.new()
	w._hd_dir = hd_texture_dir()
	if w._hd_dir != "":
		var m: Variant = JsonUtil.load_file(w._hd_dir.path_join("manifest.json"))
		if m is Dictionary:
			w._hd = m
	w._collect(root, root)
	for d: Dictionary in dummies:
		w._dummies.append(d)
	if w._meshes.is_empty():
		return {"ok": false, "error": "nothing to export"}
	var err := w._save(path)
	return {"ok": err == OK, "error": "" if err == OK else "write failed (%d)" % err, "meshes": w._meshes.size(),
		"materials": w._materials.map(func(m: Dictionary) -> String: return str(m.name)), "hd": not w._hd.is_empty()}


## Folder with the HD texture pack (next to the exe, or in the project when
## running from source), or "" if it isn't installed.
static func hd_texture_dir() -> String:
	for d: String in [OS.get_executable_path().get_base_dir().path_join("hd_textures"), ProjectSettings.globalize_path("res://hd_textures")]:
		if FileAccess.file_exists(d.path_join("manifest.json")):
			return d
	return ""


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
	if _hd.has(kind):
		var d := kind + "_d.dds"
		var nm := kind + "_n.dds"
		_textures[d] = FileAccess.get_file_as_bytes(_hd_dir.path_join(d))
		_textures[nm] = FileAccess.get_file_as_bytes(_hd_dir.path_join(nm))
		var spec: Array = {"asphalt": [0.18, 30.0], "concrete": [0.1, 20.0], "gravel": [0.08, 12.0]}.get(kind, [0.04, 6.0])
		_materials.append({"name": "dl_" + kind, "shader": "ksPerPixelNM", "tex": d, "normal": nm, "spec": spec[0], "exp": spec[1]})
	else:
		var tex_name := kind + ".dds"
		_textures[tex_name] = _make_texture(kind, color)
		_materials.append({"name": "dl_" + kind, "shader": "ksPerPixel", "tex": tex_name, "normal": "", "spec": 0.12 if kind == "asphalt" else 0.04, "exp": 25.0})
	_mat_index[kind] = _materials.size() - 1
	return _mat_index[kind]


static func _kind_for_key(key: String) -> String:
	match key:
		"ROAD": return "asphalt"
		"GRASS": return "grass"
		"GRAVEL", "SAND": return "gravel"
		"DIRT": return "dirt"
		"WALL": return "concrete"
	return ""


## Uncompressed 32-bit DDS (BGRA, full mip chain) - the same layout AC's own
## files use for small textures, so no PNG decoding is needed in the game.
static func _dds_rgba(src: Image) -> PackedByteArray:
	var img := src.duplicate() as Image
	img.convert(Image.FORMAT_RGBA8)
	img.generate_mipmaps()
	var data := img.get_data()
	for i in range(0, data.size(), 4):
		var r := data[i]
		data[i] = data[i + 2]
		data[i + 2] = r
	var h := StreamPeerBuffer.new()
	h.big_endian = false
	h.put_data("DDS ".to_ascii_buffer())
	for v in [124, 0x1 | 0x2 | 0x4 | 0x8 | 0x1000 | 0x20000, img.get_height(), img.get_width(), img.get_width() * 4, 0, img.get_mipmap_count() + 1]:
		h.put_32(v)
	for i in 11:
		h.put_32(0)
	for v in [32, 0x41, 0, 32, 0x00ff0000, 0x0000ff00, 0x000000ff, 0xff000000, 0x1000 | 0x400000 | 0x8, 0, 0, 0, 0]:
		h.put_32(v)
	var out := h.data_array
	out.append_array(data)
	return out


func _make_texture(kind: String, color: Color) -> PackedByteArray:
	var img: Image
	if kind == "kerb":
		img = Image.create(8, 64, false, Image.FORMAT_RGB8)
		for y in 64:
			for x in 8:
				img.set_pixel(x, y, Color(0.8, 0.08, 0.08) if y < 32 else Color(0.95, 0.95, 0.95))
		return _dds_rgba(img)
	if kind.begins_with("solid_"):
		img = Image.create(4, 4, false, Image.FORMAT_RGB8)
		img.fill(color)
		return _dds_rgba(img)
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
	return _dds_rgba(img)


func _uv_for(key: String, world: Vector3, normal: Vector3, uv: Vector2) -> Vector2:
	var kind := _kind_for_key(key)
	if _hd.has(kind):
		var t := maxf(float((_hd[kind] as Dictionary).get("tile_m", 2.0)), 0.5)
		if key == "ROAD":
			t *= 1.5  # slightly larger grain hides the repeat along a long road
			return uv / t  # road strips carry (metres across, metres along)
		if key == "WALL":
			return Vector2((world.x + world.z) / t, world.y / t)
		if absf(normal.y) > 0.5:
			return Vector2(world.x / t, world.z / t)
		return Vector2((world.x + world.z) / t, world.y / t)
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
				_meshes.append({"name": "%s_%d" % [name, part] if part > 0 else name, "mat": mat, "verts": cv, "normals": cn, "uvs": cu, "tangents": _tangents(cv, cn, cu, ci), "indices": ci})
				part += 1
				cv = PackedVector3Array()
				cn = PackedVector3Array()
				cu = PackedVector2Array()
				ci = PackedInt32Array()
				remap.clear()
			# Front faces must wind clockwise as seen from the normal side (Godot's
			# convention, which maps to AC front faces). Track strips are built the
			# other way round and drawn double-sided in Drift Lab, so fix the order
			# here.
			var order := [0, 1, 2]
			var q0 := xf * v[idx[t]]
			var q1 := xf * v[idx[t + 1]]
			var q2 := xf * v[idx[t + 2]]
			var nsum := Vector3.ZERO
			for k in 3:
				var si := idx[t + k]
				nsum += basis_n * (nrm[si] if si < nrm.size() else Vector3.UP)
			if (q1 - q0).cross(q2 - q0).dot(nsum) > 0.0:
				order = [0, 2, 1]
			for k: int in order:
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
			_meshes.append({"name": "%s_%d" % [name, part] if part > 0 else name, "mat": mat, "verts": cv, "normals": cn, "uvs": cu, "tangents": _tangents(cv, cn, cu, ci), "indices": ci})


## Per-vertex tangents (+U direction) from positions and UVs, for normal maps.
static func _tangents(v: PackedVector3Array, n: PackedVector3Array, uv: PackedVector2Array, idx: PackedInt32Array) -> PackedVector3Array:
	var acc := PackedVector3Array()
	acc.resize(v.size())
	for t in range(0, idx.size() - 2, 3):
		var i0 := idx[t]
		var i1 := idx[t + 1]
		var i2 := idx[t + 2]
		var e1 := v[i1] - v[i0]
		var e2 := v[i2] - v[i0]
		var d1 := uv[i1] - uv[i0]
		var d2 := uv[i2] - uv[i0]
		var det := d1.x * d2.y - d2.x * d1.y
		if absf(det) < 1e-12:
			continue
		var tg := (e1 * d2.y - e2 * d1.y) / det
		acc[i0] += tg
		acc[i1] += tg
		acc[i2] += tg
	for i in v.size():
		var nn := n[i]
		var tg := acc[i] - nn * nn.dot(acc[i])
		if tg.length_squared() < 1e-10:
			tg = Vector3.RIGHT - nn * nn.x
			if tg.length_squared() < 1e-6:
				tg = Vector3.FORWARD
		acc[i] = tg.normalized()
	return acc


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
		_str(f, str(m.shader))
		f.store_8(0)   # blend: opaque
		f.store_8(0)   # alpha tested
		f.store_32(0)  # depth mode
		var props := [["ksAmbient", 0.45], ["ksDiffuse", 0.55], ["ksSpecular", float(m.spec)], ["ksSpecularEXP", float(m.exp)], ["ksEmissive", 0.0], ["ksAlphaRef", 0.0]]
		f.store_32(props.size())
		for p: Array in props:
			_str(f, p[0])
			for i in 10:
				f.store_float(float(p[1]) if i == 0 else 0.0)
		var slots := [["txDiffuse", m.tex]]
		if str(m.normal) != "":
			slots.append(["txNormal", m.normal])
		f.store_32(slots.size())
		for k in slots.size():
			_str(f, slots[k][0])
			f.store_32(k)
			_str(f, slots[k][1])
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
		var tans: PackedVector3Array = m.tangents
		f.store_32(verts.size())
		var center := Vector3.ZERO
		for i in verts.size():
			var p := verts[i]
			var nn := normals[i]
			center += p
			var tg := tans[i]
			for val in [-p.x, p.y, -p.z, -nn.x, nn.y, -nn.z, uvs[i].x, uvs[i].y, -tg.x, tg.y, -tg.z]:
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
