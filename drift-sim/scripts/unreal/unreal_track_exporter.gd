class_name UnrealTrackExporter
extends RefCounted
## Exports a Drift Lab track for the DriftGame Unreal project:
##   <id>/<id>.glb     every mesh baked into world space and named by surface
##                     (ROAD_3, KERB_7, GRASS_ground, WALL_12, trees_pine ...),
##                     plus START / START_AHEAD marker meshes for the spawn
##   <id>/track.json   name, length, surfaces
## DriftGame's Scripts/build_track_level.py imports the folder: collision,
## photo-scanned materials per surface, lighting, a PlayerStart. The car
## reads the surface from the mesh name, so the names matter.

const SURFACE_KEYS := ["ROAD", "KERB", "GRASS", "GRAVEL", "SAND", "DIRT", "WALL"]


static func export_track(spec_in: Dictionary, out_dir: String) -> Dictionary:
	var spec := TrackSpec.normalized(spec_in)
	var id := JsonUtil.slugify(str(spec.get("name", "drift_lab_track")))
	var root_dir := out_dir.path_join(id)
	DirAccess.make_dir_recursive_absolute(root_dir)
	var built := TrackBuilder.build(spec)
	var src: Node3D = built.root
	var samples: Dictionary = built.samples
	var out := Node3D.new()
	out.name = id
	var counts := {}
	_collect(src, src, out, counts)
	var start := TrackSpec.transform_at(samples, float(spec.start), 0.3)
	out.add_child(_marker("START", start.origin))
	out.add_child(_marker("START_AHEAD", start.origin - start.basis.z * 5.0))
	src.free()

	var doc := GLTFDocument.new()
	var state := GLTFState.new()
	var err := doc.append_from_scene(out, state)
	var glb := root_dir.path_join(id + ".glb")
	if err == OK:
		err = doc.write_to_filesystem(state, glb)
	out.free()
	if err == OK:
		err = name_meshes(glb)
	if err != OK:
		return {"ok": false, "message": "glTF export failed (error %d)" % err}
	JsonUtil.save_file(root_dir.path_join("track.json"), {
		"format": "drift_lab_unreal_track_1",
		"id": id,
		"name": str(spec.get("name", id)),
		"length_m": float(samples.get("length", 0.0)),
		"closed": bool(spec.get("closed", false)),
		"meshes": counts,
	})
	return {"ok": true, "message": "Exported to %s - in Unreal run Scripts/build_track_level.py on that folder." % root_dir}


## Godot leaves glTF meshes unnamed; importers (Unreal's included) name the
## static mesh assets from them, so copy each node's name onto its mesh.
static func name_meshes(glb_path: String) -> Error:
	var b := FileAccess.get_file_as_bytes(glb_path)
	if b.size() < 20 or b.slice(0, 4).get_string_from_ascii() != "glTF":
		return ERR_FILE_CORRUPT
	var json_len := b.decode_u32(12)
	var j: Variant = JSON.parse_string(b.slice(20, 20 + json_len).get_string_from_utf8())
	if not j is Dictionary:
		return ERR_PARSE_ERROR
	var meshes: Array = j.get("meshes", [])
	for nd: Dictionary in j.get("nodes", []):
		if nd.has("mesh") and nd.has("name") and int(nd.mesh) < meshes.size():
			meshes[int(nd.mesh)]["name"] = nd.name
	var jb := JSON.stringify(_ints(j)).to_utf8_buffer()
	while jb.size() % 4 != 0:
		jb.append(0x20)
	var rest := b.slice(20 + json_len)
	var outb := PackedByteArray()
	outb.resize(20)
	outb.encode_u32(0, 0x46546C67)  # "glTF"
	outb.encode_u32(4, 2)
	outb.encode_u32(8, 20 + jb.size() + rest.size())
	outb.encode_u32(12, jb.size())
	outb.encode_u32(16, 0x4E4F534A)  # "JSON"
	outb.append_array(jb)
	outb.append_array(rest)
	var f := FileAccess.open(glb_path, FileAccess.WRITE)
	if f == null:
		return FileAccess.get_open_error()
	f.store_buffer(outb)
	f.close()
	return OK


## JSON.parse_string makes every number a float; glTF indices must be ints.
static func _ints(v: Variant) -> Variant:
	if v is Dictionary:
		var d := {}
		for k: String in v:
			d[k] = _ints(v[k])
		return d
	if v is Array:
		return (v as Array).map(func(e: Variant) -> Variant: return _ints(e))
	if v is float and is_finite(v) and v == floorf(v) and absf(v) < 9.0e15:
		return int(v)
	return v


## Surface name for a mesh: the StaticBody's AC-style name (1ROAD_3 -> ROAD_3),
## else the mesh node's own name (trees_pine), else obj_<n>.
static func _name_for(mi: MeshInstance3D) -> String:
	var parent := mi.get_parent()
	if parent is StaticBody3D:
		var m := RegEx.create_from_string("^[0-9]+([A-Z]+)(.*)$").search(str(parent.name).to_upper())
		if m and SURFACE_KEYS.has(m.get_string(1)):
			return m.get_string(1) + m.get_string(2)
	var n := str(mi.name)
	return "" if n.begins_with("@") else n


static func _collect(n: Node, src: Node3D, out: Node3D, counts: Dictionary) -> void:
	if n is MeshInstance3D and (n as MeshInstance3D).visible and (n as MeshInstance3D).mesh != null:
		var mi := n as MeshInstance3D
		var nm := _name_for(mi)
		if nm == "":
			nm = "obj"
		var key := nm.get_slice("_", 0)
		counts[key] = int(counts.get(key, 0)) + 1
		var baked := _bake(mi, src)
		if baked != null:
			var o := MeshInstance3D.new()
			o.name = "%s_%d" % [nm, counts[key]] if not nm.contains("_") else nm
			baked.resource_name = o.name
			o.mesh = baked
			out.add_child(o)
	for c in n.get_children():
		_collect(c, src, out, counts)


## Copies the mesh with its transform applied (identity node in the glTF, so
## the importer can't disagree about hierarchy). Materials: textured ones are
## kept (trees); surfaces get a flat colour - Unreal assigns real materials.
static func _bake(mi: MeshInstance3D, src: Node3D) -> ArrayMesh:
	var xf := src.global_transform.affine_inverse() * mi.global_transform if mi.is_inside_tree() else _rel(mi, src)
	var nb := xf.basis.inverse().transposed()
	var mesh := mi.mesh
	var am := ArrayMesh.new()
	for si in mesh.get_surface_count():
		var arrays := mesh.surface_get_arrays(si)
		var verts: PackedVector3Array = arrays[Mesh.ARRAY_VERTEX]
		if verts.is_empty():
			continue
		for i in verts.size():
			verts[i] = xf * verts[i]
		arrays[Mesh.ARRAY_VERTEX] = verts
		if arrays[Mesh.ARRAY_NORMAL] is PackedVector3Array and not (arrays[Mesh.ARRAY_NORMAL] as PackedVector3Array).is_empty():
			var nrm: PackedVector3Array = arrays[Mesh.ARRAY_NORMAL]
			for i in nrm.size():
				nrm[i] = (nb * nrm[i]).normalized()
			arrays[Mesh.ARRAY_NORMAL] = nrm
		arrays[Mesh.ARRAY_TANGENT] = null
		_face_normals(arrays)
		var prim := (mesh as ArrayMesh).surface_get_primitive_type(si) if mesh is ArrayMesh else Mesh.PRIMITIVE_TRIANGLES
		am.add_surface_from_arrays(prim, arrays)
		am.surface_set_material(am.get_surface_count() - 1, _export_material(mi, mesh, si))
	return am if am.get_surface_count() > 0 else null


## Track meshes are drawn double-sided in Drift Lab, so their winding was
## never checked - but other engines cull back faces. Order every triangle so
## its front (Godot: clockwise) faces the way its vertex normals point.
static func _face_normals(arrays: Array) -> void:
	var verts: PackedVector3Array = arrays[Mesh.ARRAY_VERTEX]
	var nrm: Variant = arrays[Mesh.ARRAY_NORMAL]
	if not nrm is PackedVector3Array or (nrm as PackedVector3Array).size() != verts.size():
		return
	var normals: PackedVector3Array = nrm
	var idx: PackedInt32Array
	if arrays[Mesh.ARRAY_INDEX] is PackedInt32Array and not (arrays[Mesh.ARRAY_INDEX] as PackedInt32Array).is_empty():
		idx = arrays[Mesh.ARRAY_INDEX]
	else:
		idx = PackedInt32Array(range(verts.size()))
	for i in range(0, idx.size() - 2, 3):
		var a := verts[idx[i]]
		var n := normals[idx[i]] + normals[idx[i + 1]] + normals[idx[i + 2]]
		if (verts[idx[i + 1]] - a).cross(verts[idx[i + 2]] - a).dot(n) > 0.0:
			var t := idx[i + 1]
			idx[i + 1] = idx[i + 2]
			idx[i + 2] = t
	arrays[Mesh.ARRAY_INDEX] = idx


static func _rel(n: Node3D, root: Node3D) -> Transform3D:
	var t := n.transform
	var p := n.get_parent()
	while p != null and p != root:
		if p is Node3D:
			t = (p as Node3D).transform * t
		p = p.get_parent()
	return t


static func _export_material(mi: MeshInstance3D, mesh: Mesh, si: int) -> Material:
	var mat: Material = mi.material_override if mi.material_override != null else mesh.surface_get_material(si)
	if mat is BaseMaterial3D:
		var bm := mat as BaseMaterial3D
		if bm.albedo_texture is ImageTexture and bm.albedo_texture.get_image() != null:
			return bm
		var m := StandardMaterial3D.new()
		m.albedo_color = bm.albedo_color
		m.roughness = bm.roughness
		m.vertex_color_use_as_albedo = bm.vertex_color_use_as_albedo
		return m
	return StandardMaterial3D.new()


static func _marker(nm: String, at: Vector3) -> MeshInstance3D:
	var m := MeshInstance3D.new()
	m.name = nm
	var box := BoxMesh.new()
	box.size = Vector3(0.2, 0.2, 0.2)
	var am := ArrayMesh.new()
	var arrays := box.get_mesh_arrays()
	var verts: PackedVector3Array = arrays[Mesh.ARRAY_VERTEX]
	for i in verts.size():
		verts[i] += at
	arrays[Mesh.ARRAY_VERTEX] = verts
	am.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	am.resource_name = nm
	m.mesh = am
	return m
