class_name Kn5Loader
extends RefCounted
## Reads Assetto Corsa .kn5 models (unencrypted, versions 5/6) into Godot
## nodes at runtime, so cars can be shown straight from the player's own AC
## install. Nothing is converted or copied into Drift Lab.
##
## File layout: "sc6969", version, [extra int if v>5], textures, materials,
## then a node tree (1 = dummy with 4x4 matrix, 2 = mesh, 3 = skinned mesh).
## AC cars face +Z with +X to the car's left (right-handed, Y up); ours face
## -Z with +X right, so everything is turned 180 degrees about Y:
## (x, y, z) -> (-x, y, -z). AC front faces wind opposite to Godot's, so each
## triangle's order is reversed on load.

var f: FileAccess
var version := 0
var textures := {}          # name -> PackedByteArray (lazy-decoded)
var texture_cache := {}     # name -> Texture2D
var overrides := {}         # name -> absolute path (skin folder files)
var materials: Array = []   # [{name, shader, blend, alpha_tested, props, samplers}]
var mat_cache := {}
var named := {}             # node name -> Node3D (first occurrence)
var mesh_count := 0
var error := ""


func _str() -> String:
	var n := f.get_32()
	if n > 4096:
		error = "bad string length %d" % n
		return ""
	return f.get_buffer(n).get_string_from_utf8()


func _mat4() -> Transform3D:
	var m := f.get_buffer(64).to_float32_array()
	# Row-major with row vectors: rows 0-2 are the basis axes, row 3 the
	# origin. Conjugate by R = diag(-1, 1, -1) for the 180 degree turn.
	var x := Vector3(m[0], -m[1], m[2])
	var y := Vector3(-m[4], m[5], -m[6])
	var z := Vector3(m[8], -m[9], m[10])
	var o := Vector3(-m[12], m[13], -m[14])
	return Transform3D(Basis(x, y, z), o)


## Loads a kn5. skin_dir: optional folder whose texture files replace the
## kn5's textures of the same name (AC skins work this way).
static func load_model(path: String, skin_dir := "", hide_lods := true) -> Dictionary:
	var l := Kn5Loader.new()
	var root := l._load(path, skin_dir)
	if root == null:
		return {"ok": false, "error": l.error}
	if hide_lods:
		l._hide_extras(root)
	return {"ok": true, "root": root, "named": l.named, "meshes": l.mesh_count}


func _load(path: String, skin_dir: String) -> Node3D:
	f = FileAccess.open(path, FileAccess.READ)
	if f == null:
		error = "Can't open %s" % path
		return null
	if f.get_buffer(6).get_string_from_ascii() != "sc6969":
		error = "Not a kn5 file (or it's encrypted)"
		return null
	version = f.get_32()
	if version > 5:
		f.get_32()
	if skin_dir != "" and DirAccess.dir_exists_absolute(skin_dir):
		for fn in DirAccess.get_files_at(skin_dir):
			overrides[fn.to_lower()] = skin_dir.path_join(fn)
	var tex_count := f.get_32()
	for i in tex_count:
		f.get_32()  # active / type
		var name := _str()
		var size := f.get_32()
		textures[name.to_lower()] = f.get_buffer(size)
	var mat_count := f.get_32()
	for i in mat_count:
		var m := {"name": _str(), "shader": _str()}
		m.blend = f.get_8()
		m.alpha_tested = f.get_8() != 0
		if version > 4:
			m.depth = f.get_32()
		var props := {}
		for p in f.get_32():
			var pname := _str()
			var vals := f.get_buffer(40).to_float32_array()  # A, B[2], C[3], D[4]
			props[pname] = vals
		m.props = props
		var samplers := {}
		for s in f.get_32():
			var sname := _str()
			f.get_32()  # slot
			samplers[sname] = _str().to_lower()
		m.samplers = samplers
		materials.append(m)
		if error != "":
			return null
	var root := _node()
	if root == null and error == "":
		error = "No nodes"
	return root


func _node() -> Node3D:
	if f.eof_reached():
		error = "Unexpected end of file"
		return null
	var cls := f.get_32()
	var name := _str()
	var child_count := f.get_32()
	var active := f.get_8() != 0
	var node: Node3D
	match cls:
		1:
			node = Node3D.new()
			node.transform = _mat4()
		2:
			node = _mesh()
		3:
			node = _skinned()
		_:
			error = "Unknown node class %d at '%s'" % [cls, name]
			return null
	if node == null:
		return null
	node.name = name.validate_node_name() if name != "" else "node"
	node.visible = active
	if name != "" and not named.has(name):
		named[name] = node
	for i in child_count:
		var c := _node()
		if c == null:
			return node if error == "" else null
		node.add_child(c)
	return node


func _mesh() -> MeshInstance3D:
	var cast_shadows := f.get_8() != 0
	var visible := f.get_8() != 0
	var transparent := f.get_8() != 0
	var vcount := f.get_32()
	var raw := f.get_buffer(vcount * 44).to_float32_array()
	var icount := f.get_32()
	var idx := f.get_buffer(icount * 2)
	var mat_id := f.get_32()
	f.get_32()  # layer
	var lod_in := f.get_float()   # AC draws this mesh only between these distances (m)
	var lod_out := f.get_float()
	f.get_buffer(16)  # bounding sphere
	var renderable := f.get_8() != 0   # false = physics-only mesh (tracks)
	var mi := _build_mesh(raw, 11, vcount, idx, icount, mat_id)
	mi.visible = visible
	mi.set_meta("ac_renderable", renderable)
	mi.set_meta("ac_lod_in", lod_in)
	mi.set_meta("ac_lod_out", lod_out)
	mi.set_meta("ac_cast_shadows", cast_shadows)
	mi.set_meta("ac_transparent", transparent)
	return mi


func _skinned() -> MeshInstance3D:
	f.get_8()
	var visible := f.get_8() != 0
	f.get_8()
	var bones := f.get_32()
	for b in bones:
		_str()
		f.get_buffer(64)
	var vcount := f.get_32()
	var raw := f.get_buffer(vcount * 76).to_float32_array()  # pos, nrm, uv, tangent, 4 weights, 4 indices
	var icount := f.get_32()
	var idx := f.get_buffer(icount * 2)
	var mat_id := f.get_32()
	f.get_32()
	f.get_float()
	f.get_float()
	var mi := _build_mesh(raw, 19, vcount, idx, icount, mat_id)
	mi.visible = visible
	return mi


func _build_mesh(raw: PackedFloat32Array, stride: int, vcount: int, idx: PackedByteArray, icount: int, mat_id: int) -> MeshInstance3D:
	var verts := PackedVector3Array()
	var normals := PackedVector3Array()
	var uvs := PackedVector2Array()
	verts.resize(vcount)
	normals.resize(vcount)
	uvs.resize(vcount)
	for v in vcount:
		var o := v * stride
		verts[v] = Vector3(-raw[o], raw[o + 1], -raw[o + 2])
		normals[v] = Vector3(-raw[o + 3], raw[o + 4], -raw[o + 5])
		uvs[v] = Vector2(raw[o + 6], raw[o + 7])
	var indices := PackedInt32Array()
	indices.resize(icount)
	for i in range(0, icount - 2, 3):
		indices[i] = idx.decode_u16(i * 2)
		# AC (DirectX) front faces wind the other way from Godot's.
		indices[i + 1] = idx.decode_u16(i * 2 + 4)
		indices[i + 2] = idx.decode_u16(i * 2 + 2)
	var mi := MeshInstance3D.new()
	if vcount == 0 or icount < 3:
		return mi
	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = verts
	arrays[Mesh.ARRAY_NORMAL] = normals
	arrays[Mesh.ARRAY_TEX_UV] = uvs
	arrays[Mesh.ARRAY_INDEX] = indices
	var mesh := ArrayMesh.new()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	mi.mesh = mesh
	mi.material_override = _material(mat_id)
	mesh_count += 1
	return mi


func texture(name: String) -> Texture2D:
	name = name.to_lower()
	if texture_cache.has(name):
		return texture_cache[name]
	var bytes := PackedByteArray()
	if overrides.has(name):
		bytes = FileAccess.get_file_as_bytes(overrides[name])
	elif textures.has(name):
		bytes = textures[name]
	var tex: Texture2D = null
	var img := DdsLoader.image_from_bytes(bytes)
	if img:
		tex = ImageTexture.create_from_image(img)
	texture_cache[name] = tex
	return tex


const MULTIMAP_SHADER := preload("res://shaders/ac_multimap.gdshader")


func _material(id: int) -> Material:
	if mat_cache.has(id):
		return mat_cache[id]
	if id >= 0 and id < materials.size():
		var md0: Dictionary = materials[id]
		var p0: Dictionary = md0.props
		var s0: Dictionary = md0.samplers
		if p0.has("useDetail") and float(p0.useDetail[0]) > 0.5 and s0.has("txDetail") and s0.has("txDiffuse") and int(md0.blend) == 0:
			var sm := ShaderMaterial.new()
			sm.shader = MULTIMAP_SHADER
			sm.set_shader_parameter("tx_diffuse", texture(s0.txDiffuse))
			sm.set_shader_parameter("tx_detail", texture(s0.txDetail))
			sm.set_shader_parameter("detail_mult", float(p0.detailUVMultiplier[0]) if p0.has("detailUVMultiplier") else 1.0)
			sm.set_shader_parameter("diffuse_k", clampf(float(p0.ksDiffuse[0]) if p0.has("ksDiffuse") else 1.0, 0.3, 1.5))
			var se := float(p0.ksSpecularEXP[0]) if p0.has("ksSpecularEXP") else 30.0
			sm.set_shader_parameter("rough", clampf(1.0 - log(maxf(se, 1.0)) / 6.0, 0.08, 0.9))
			mat_cache[id] = sm
			return sm
	var m := StandardMaterial3D.new()
	if id >= 0 and id < materials.size():
		var md: Dictionary = materials[id]
		var props: Dictionary = md.props
		var samplers: Dictionary = md.samplers
		var diffuse := 1.0
		if props.has("ksDiffuse"):
			diffuse = clampf(props.ksDiffuse[0], 0.2, 1.5)
		m.albedo_color = Color(diffuse, diffuse, diffuse)
		if samplers.has("txDiffuse"):
			m.albedo_texture = texture(samplers.txDiffuse)
		# AC normal maps are often DXT5nm-swizzled; Godot reads those as tilted
		# normals that darken the paint, so they're skipped.
		var spec := float(props.ksSpecular[0]) if props.has("ksSpecular") else 0.2
		var spec_exp := float(props.ksSpecularEXP[0]) if props.has("ksSpecularEXP") else 20.0
		m.roughness = clampf(1.0 - log(maxf(spec_exp, 1.0)) / 6.0, 0.08, 0.95)
		m.metallic_specular = clampf(spec, 0.0, 1.0)
		if props.has("ksEmissive"):
			var e: PackedFloat32Array = props.ksEmissive
			var ec := Color(e[3], e[4], e[5])
			if ec.r + ec.g + ec.b > 0.01:
				m.emission_enabled = true
				m.emission = ec
				m.emission_energy_multiplier = 0.15
		var shader := str(md.shader).to_lower()
		if int(md.blend) == 1 or shader.contains("glass") or shader.contains("windscreen"):
			m.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
			m.cull_mode = BaseMaterial3D.CULL_DISABLED
			if shader.contains("windscreen") or shader.contains("glass") or str(md.name).to_lower().contains("glass"):
				m.albedo_color.a = 0.3  # tinted, not frosted
		elif bool(md.alpha_tested):
			m.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA_SCISSOR
			m.alpha_scissor_threshold = 0.5
			m.cull_mode = BaseMaterial3D.CULL_DISABLED
		if shader.contains("multimap") or shader.contains("car_paint"):
			m.clearcoat_enabled = true
			m.clearcoat = 0.6
	mat_cache[id] = m
	return m


## Hides lower-LOD duplicates and helper meshes so only the main car shows.
func _hide_extras(n: Node) -> void:
	for c in n.get_children():
		var nm := str(c.name).to_upper()
		if nm.contains("_LR") and (nm.begins_with("STEER") or nm.contains("RIM_BLUR") or nm.contains("BLUR")):
			(c as Node3D).visible = false
		elif nm.contains("SHADOW") or nm.begins_with("COCKPIT_LR") or nm.contains("_BLUR"):
			(c as Node3D).visible = false
		_hide_extras(c)
