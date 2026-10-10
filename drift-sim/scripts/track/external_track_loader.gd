class_name ExternalTrackLoader
extends RefCounted
## Loads a track model (.glb / .gltf / .fbx) at runtime - e.g. an Assetto Corsa
## track converted with Content Manager or Blender - and turns it into a
## drivable track. Assetto Corsa naming conventions are understood:
##   meshes named "<digit><SURFACE>..." (1ROAD, 2KERB_04, 1GRASS...) become
##   physics surfaces; *WALL*/BARRIER/FENCE meshes become walls;
##   AC_START_0 / AC_PIT_0 / AC_HOTLAP_START_0 set the spawn point.
## If the model has no AC-style physics names, every mesh is treated as road.

const SURFACE_KEYS := [
	["WALL", "wall"], ["BARRIER", "wall"], ["FENCE", "wall"], ["GUARD", "wall"],
	["KERB", "kerb"], ["CURB", "kerb"], ["GRASS", "grass"], ["SAND", "sand"],
	["GRAVEL", "gravel"], ["DIRT", "dirt"], ["CONCRETE", "concrete"],
	["ROAD", "asphalt"], ["ASPHALT", "asphalt"], ["TARMAC", "asphalt"], ["PIT", "asphalt"],
]


static func classify(node_name: String) -> String:
	var up := node_name.to_upper()
	var re := RegEx.create_from_string("^[0-9]+([A-Z_]+)")
	var m := re.search(up)
	if m == null:
		return ""
	var key := m.get_string(1)
	for pair: Array in SURFACE_KEYS:
		if key.begins_with(pair[0]) or key.contains(pair[0]):
			return pair[1]
	return "asphalt"


## Returns {ok, root, start: Transform3D, error}.
static func load_model(path: String) -> Dictionary:
	var scene: Node = null
	var ext := path.get_extension().to_lower()
	var err := OK
	if ext == "glb" or ext == "gltf":
		var doc := GLTFDocument.new()
		var state := GLTFState.new()
		err = doc.append_from_file(path, state)
		if err == OK:
			scene = doc.generate_scene(state)
	elif ext == "kn5":
		var kr := Kn5Loader.load_model(path, "", false)
		if not kr.ok:
			return {"ok": false, "error": "Could not load %s: %s" % [path, kr.error]}
		scene = kr.root
	elif ext == "fbx":
		var fdoc := FBXDocument.new()
		var fstate := FBXState.new()
		err = fdoc.append_from_file(path, fstate)
		if err == OK:
			scene = fdoc.generate_scene(fstate)
	else:
		return {"ok": false, "error": "Unsupported format .%s (use .kn5, .glb, .gltf or .fbx)" % ext}
	if scene == null:
		return {"ok": false, "error": "Could not load %s (error %d)" % [path, err]}

	var root := Node3D.new()
	root.name = "ExternalTrack"
	root.add_child(scene)
	var meshes: Array[MeshInstance3D] = []
	_collect(scene, meshes)
	var any_ac := false
	for mi in meshes:
		if classify(mi.name) != "" or classify(str(mi.get_parent().name)) != "":
			any_ac = true
			break
	var count := 0
	for mi in meshes:
		if mi.mesh == null:
			continue
		var surf := classify(mi.name)
		if surf == "":
			surf = classify(str(mi.get_parent().name))
		if surf == "" and any_ac:
			continue  # visual-only mesh
		if surf == "":
			surf = "asphalt"
		var body := StaticBody3D.new()
		body.collision_mask = 0
		if surf == "wall":
			body.set_meta("wall", true)
			body.set_meta("surface", "concrete")
			body.collision_layer = 2
		else:
			body.set_meta("surface", surf)
			body.collision_layer = 1
		var cs := CollisionShape3D.new()
		var shape := mi.mesh.create_trimesh_shape()
		if shape == null:
			continue
		shape.backface_collision = true
		cs.shape = shape
		body.add_child(cs)
		mi.add_child(body)
		count += 1
	var start := Transform3D(Basis(), Vector3(0, 2, 0))
	for nm in ["AC_START_0", "AC_HOTLAP_START_0", "AC_PIT_0"]:
		var n := scene.find_child(nm, true, false)
		if n is Node3D:
			var gt := _global_xform(n as Node3D, scene)
			var b := gt.basis.orthonormalized()
			if ext != "kn5":
				b = b.rotated(Vector3.UP, PI)  # glTF/FBX keep AC's +Z-forward dummies
			start = Transform3D(b, gt.origin + Vector3.UP * 0.8)
			break
	if count == 0:
		return {"ok": false, "error": "No meshes found in the model."}
	# Lighting if the file has none.
	var env := WorldEnvironment.new()
	var e := Environment.new()
	e.background_mode = Environment.BG_SKY
	e.sky = Sky.new()
	e.sky.sky_material = ProceduralSkyMaterial.new()
	e.ambient_light_source = Environment.AMBIENT_SOURCE_SKY
	e.tonemap_mode = Environment.TONE_MAPPER_FILMIC
	env.environment = e
	root.add_child(env)
	var sun := DirectionalLight3D.new()
	sun.rotation_degrees = Vector3(-55, 35, 0)
	sun.shadow_enabled = true
	root.add_child(sun)
	return {"ok": true, "root": root, "start": start, "collision_meshes": count}


static func _collect(n: Node, out: Array[MeshInstance3D]) -> void:
	if n is MeshInstance3D:
		out.append(n)
	for c in n.get_children():
		_collect(c, out)


static func _global_xform(n: Node3D, scene_root: Node) -> Transform3D:
	var t := n.transform
	var p := n.get_parent()
	while p != null and p != scene_root.get_parent():
		if p is Node3D:
			t = (p as Node3D).transform * t
		p = p.get_parent()
	return t
