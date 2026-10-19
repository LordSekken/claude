class_name UnrealAcTrackExporter
extends RefCounted
## Exports an Assetto Corsa track (the player's own install) for DriftGame:
##   <out>/<id>/<id>.glb   driving surfaces merged per surface (ROAD_1, KERB_2,
##                         GRASS_3 ...; physics-only ones as ROAD_HIDDEN_4),
##                         everything else as visual-only VIS_<cell> chunks
##                         (merged by material in 150 m cells), plus START /
##                         START_AHEAD markers at the pit spawn (AC_PIT_0)
##   <out>/<id>/track.json  keep_materials: Unreal keeps AC's own textures
## AC naming: "<digit><SURFACE>..." meshes are physics surfaces (see
## ExternalTrackLoader.classify). Nothing here may be committed: it is the
## track author's content.

const CELL := 150.0
const UE_WORD := {"asphalt": "ROAD", "kerb": "KERB", "grass": "GRASS", "sand": "SAND",
	"gravel": "GRAVEL", "dirt": "DIRT", "concrete": "CONCRETE", "wall": "WALL"}


static func export_track(kn5_path: String, out_dir: String, track_name := "") -> Dictionary:
	var kr := Kn5Loader.load_model(kn5_path, "", false)
	if not kr.ok:
		return {"ok": false, "message": "Couldn't read %s: %s" % [kn5_path, kr.error]}
	var root: Node3D = kr.root
	if track_name == "":
		track_name = kn5_path.get_file().get_basename()
	var id := JsonUtil.slugify(track_name)

	var surfaces := {}   # "ROAD" / "ROAD_HIDDEN" -> [MeshInstance3D]
	var cells := {}      # Vector2i -> [MeshInstance3D]
	var meshes: Array[MeshInstance3D] = []
	_collect(root, meshes)
	for mi in meshes:
		var surf := ExternalTrackLoader.classify(mi.name)
		if surf == "":
			surf = ExternalTrackLoader.classify(str(mi.get_parent().name))
		var renderable: bool = mi.get_meta("ac_renderable", true) and _visible(mi, root)
		if surf != "":
			var key: String = UE_WORD.get(surf, "ROAD") + ("" if renderable else "_HIDDEN")
			if not surfaces.has(key):
				surfaces[key] = []
			(surfaces[key] as Array).append(mi)
		elif renderable:
			var p := AcCarModel._rel(mi, root).origin + mi.get_aabb().get_center()
			var c := Vector2i(floori(p.x / CELL), floori(p.z / CELL))
			if not cells.has(c):
				cells[c] = []
			(cells[c] as Array).append(mi)

	UnrealCarExporter.max_tex = 2048
	var tex_cache := {}
	var out := Node3D.new()
	out.name = id
	var n := 0
	var counts := {}
	for key: String in surfaces:
		var mesh := UnrealCarExporter._merge(surfaces[key], root, Vector3.ZERO, Vector3.ZERO, tex_cache)
		if mesh:
			n += 1
			out.add_child(_mi("%s_%d" % [key, n], mesh))
			counts[key] = (surfaces[key] as Array).size()
	var vis := 0
	for c: Vector2i in cells:
		var mesh := UnrealCarExporter._merge(cells[c], root, Vector3.ZERO, Vector3.ZERO, tex_cache)
		if mesh:
			vis += 1
			out.add_child(_mi("VIS_%d" % vis, mesh))
	UnrealCarExporter.max_tex = 4096
	counts["VIS"] = vis

	# Spawn at the pit (the start grid if there is no pit box).
	var start := Transform3D(Basis(), Vector3(0, 2, 0))
	for nm in ["AC_PIT_0", "AC_START_0", "AC_HOTLAP_START_0"]:
		if kr.named.get(nm) is Node3D:
			start = AcCarModel._rel(kr.named[nm], root)
			break
	root.free()
	out.add_child(UnrealTrackExporter._marker("START", start.origin + Vector3.UP * 0.3))
	out.add_child(UnrealTrackExporter._marker("START_AHEAD", start.origin + Vector3.UP * 0.3 - start.basis.z.normalized() * 5.0))

	var dir := out_dir.path_join(id)
	DirAccess.make_dir_recursive_absolute(dir)
	var glb := dir.path_join(id + ".glb")
	var doc := GLTFDocument.new()
	var state := GLTFState.new()
	var err := doc.append_from_scene(out, state)
	if err == OK:
		err = doc.write_to_filesystem(state, glb)
	out.free()
	if err == OK:
		err = UnrealTrackExporter.name_meshes(glb)
	if err != OK:
		return {"ok": false, "message": "glTF export failed (error %d)" % err}
	JsonUtil.save_file(dir.path_join("track.json"), {
		"format": "drift_lab_unreal_track_1", "id": id, "name": track_name,
		"source": "assetto_corsa", "keep_materials": true, "meshes": counts,
	})
	return {"ok": true, "id": id, "counts": counts, "message": "Exported %s to %s (%s)" % [track_name, dir, counts]}


static func _mi(nm: String, mesh: ArrayMesh) -> MeshInstance3D:
	var mi := MeshInstance3D.new()
	mi.name = nm
	mesh.resource_name = nm
	mi.mesh = mesh
	return mi


static func _collect(n: Node, out: Array[MeshInstance3D]) -> void:
	# AC_* objects are AC's helpers (pit boxes, start grid, timing gates...): AC never draws
	# them, but some tracks give them a little mesh - Brooklyn Park's 32 pit boxes became
	# rows of white pyramids in the pit lane.
	if str(n.name).to_upper().begins_with("AC_"):
		return
	if n is MeshInstance3D and (n as MeshInstance3D).mesh != null and (n as MeshInstance3D).skin == null:
		out.append(n)
	for c in n.get_children():
		_collect(c, out)


static func _visible(n: Node, root: Node) -> bool:
	while n != null and n != root:
		if n is Node3D and not (n as Node3D).visible:
			return false
		n = n.get_parent()
	return true
