class_name UnrealCarExporter
extends RefCounted
## Exports an Assetto Corsa car (the player's own install/download) for DriftGame:
##   <project>/Cars/<id>/<id>.glb   BODY, WHEEL_LF/RF/LR/RR, SUSP_* (steer, don't
##                                  spin), STEER (steering wheel), in DriftCore's
##                                  body frame: origin at axle height midway
##                                  between the axles, -Z forward, metres
##   <project>/Content/DriftData/Cars/ac_<id>.json
##                                  DriftCore spec: a Drift Lab base build (or the
##                                  car's own physics once unpacked) with the
##                                  wheelbase/track/wheel sizes/eye measured from
##                                  the model, plus a "model" block for the pawn.
## Unreal's Scripts/build_car.py imports the glb. Nothing here is meant for git:
## the meshes and textures are the car author's.

const WHEELS := AcCarModel.WHEELS  # LF, RF, LR, RR = DriftCore wheel order
const MAX_TEX := 4096


static func export_car(car_dir: String, project_dir: String, skin := "", base_build := "miata_na_bp_turbo") -> Dictionary:
	var model := AcCarModel.load_for({"path": car_dir, "skin": skin})
	if model.is_empty():
		return {"ok": false, "message": "Couldn't load the car model in %s (needs a .kn5 with WHEEL_LF/RF/LR/RR)." % car_dir}
	var id := JsonUtil.slugify(car_dir.trim_suffix("/").trim_suffix("\\").get_file())
	var root: Node3D = model.root
	var named: Dictionary = model.named
	var offset: Vector3 = model.geo.offset

	# Which group each visible mesh belongs to.
	var groups := {"BODY": []}
	var owners := {}  # node instance id -> group
	for w: String in WHEELS:
		groups["WHEEL_" + w] = []
		groups["SUSP_" + w] = []
		for key in ["WHEEL_" + w, "SUSP_" + w]:
			if named.get(key) is Node3D:
				owners[(named[key] as Node).get_instance_id()] = key
	groups["STEER"] = []
	if named.get("STEER_HR") is Node3D:
		owners[(named.STEER_HR as Node).get_instance_id()] = "STEER"
	_gather(root, root, "BODY", owners, groups)

	# Pivots, in the body frame.
	var pivots := {"BODY": Vector3.ZERO}
	for w: String in WHEELS:
		var c: Vector3 = (model.centres[w] as Vector3) + offset
		pivots["WHEEL_" + w] = c
		pivots["SUSP_" + w] = c
	var steer_axis := Vector3(0, 0, 1)
	if named.get("STEER_HR") is Node3D:
		var st := AcCarModel._rel(named.STEER_HR, root)
		pivots["STEER"] = st.origin + offset
		steer_axis = st.basis.z.normalized()

	var out := Node3D.new()
	out.name = id
	var tex_cache := {}
	var counts := {}
	var body_mesh: ArrayMesh = null
	for g: String in groups:
		var mesh := _merge(groups[g], root, offset, pivots.get(g, Vector3.ZERO), tex_cache)
		if mesh == null:
			continue
		var mi := MeshInstance3D.new()
		mi.name = g
		mesh.resource_name = g
		mi.mesh = mesh
		out.add_child(mi)
		counts[g] = (groups[g] as Array).size()
		if g == "BODY":
			body_mesh = mesh
	var hood_cam := _hood_cam(body_mesh, pivots.get("STEER", Vector3(0, 0.3, 0)))
	root.free()
	if not counts.has("BODY") or not counts.has("WHEEL_LF"):
		out.free()
		return {"ok": false, "message": "The model has no visible body or wheel meshes."}

	var dir := project_dir.path_join("Cars").path_join(id)
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

	# Physics: the base build, resized to the model.
	var spec_r := _spec(car_dir, base_build, model)
	if not spec_r.ok:
		return spec_r
	var spec: Dictionary = spec_r.spec
	var steer_ratio := 8.0
	if FileAccess.file_exists(car_dir.path_join("data/car.ini")):
		var ratio := absf(AcIni.f(AcIni.load_ini(car_dir.path_join("data/car.ini")), "CONTROLS", "STEER_RATIO", 0.0))
		if ratio > 1.0:
			steer_ratio = ratio
	spec["model"] = {
		"steer_ratio": steer_ratio,
		"id": id,
		"skin": str(model.skin),
		"steer_pivot": _arr(pivots.get("STEER", Vector3.ZERO)),
		"steer_axis": _arr(steer_axis),
		"has_steer": pivots.has("STEER"),
		"hood_cam": _arr(hood_cam),
		"parts": counts.keys(),
	}
	var spec_id := "ac_" + id
	var cars_dir := project_dir.path_join("Content/DriftData/Cars")
	DirAccess.make_dir_recursive_absolute(cars_dir)
	var f := FileAccess.open(cars_dir.path_join(spec_id + ".json"), FileAccess.WRITE)
	f.store_string(CarBuilder.spec_json(spec, spec_id))
	f.close()
	return {"ok": true, "id": id, "spec_id": spec_id, "glb": glb, "parts": counts, "physics": spec_r.physics,
		"message": "Exported %s (%s) - in Unreal run Scripts/build_car.py." % [id, ", ".join(counts.keys())]}


## Hood camera: on the centreline just ahead of the windshield (~0.7 m in
## front of the steering wheel), 10 cm above the highest body point there.
static func _hood_cam(body: ArrayMesh, steer: Vector3) -> Vector3:
	var z := steer.z - 0.7
	var top := -INF
	if body:
		for si in body.get_surface_count():
			for v: Vector3 in body.surface_get_arrays(si)[Mesh.ARRAY_VERTEX]:
				if absf(v.x) < 0.3 and absf(v.z - z) < 0.1 and v.y > top:
					top = v.y
	if top == -INF:
		top = steer.y + 0.15
	return Vector3(0, top + 0.1, z)


static func _gather(n: Node, root: Node3D, group: String, owners: Dictionary, groups: Dictionary) -> void:
	if n is Node3D and not (n as Node3D).visible:
		return
	var g := str(owners.get(n.get_instance_id(), group))
	if n is MeshInstance3D and (n as MeshInstance3D).mesh != null and (n as MeshInstance3D).skin == null:
		(groups[g] as Array).append(n)
	for c in n.get_children():
		_gather(c, root, g, owners, groups)


## Merges a group's meshes into one mesh, one surface per material, with
## vertices in the body frame minus the group's pivot.
static func _merge(nodes: Array, root: Node3D, offset: Vector3, pivot: Vector3, tex_cache: Dictionary) -> ArrayMesh:
	if nodes.is_empty():
		return null
	var by_mat := {}  # material -> SurfaceTool
	var order: Array = []
	for mi: MeshInstance3D in nodes:
		var xf := Transform3D(Basis(), offset - pivot) * AcCarModel._rel(mi, root)
		for s in mi.mesh.get_surface_count():
			if mi.mesh.surface_get_primitive_type(s) != Mesh.PRIMITIVE_TRIANGLES:
				continue
			var mat: Material = mi.material_override if mi.material_override else mi.mesh.surface_get_material(s)
			if not by_mat.has(mat):
				var st := SurfaceTool.new()
				st.begin(Mesh.PRIMITIVE_TRIANGLES)
				by_mat[mat] = st
				order.append(mat)
			(by_mat[mat] as SurfaceTool).append_from(mi.mesh, s, xf)
	var mesh := ArrayMesh.new()
	for mat: Material in order:
		var st: SurfaceTool = by_mat[mat]
		st.set_material(_gltf_material(mat, tex_cache))
		st.commit(mesh)
	return mesh if mesh.get_surface_count() > 0 else null


## glTF-friendly copy: plain StandardMaterial3D, uncompressed textures.
static func _gltf_material(mat: Material, tex_cache: Dictionary) -> Material:
	var m := StandardMaterial3D.new()
	if mat is ShaderMaterial:  # AC multimap (paint/decals over a detail map)
		m.albedo_texture = _plain(mat.get_shader_parameter("tx_diffuse"), tex_cache)
		m.roughness = float(mat.get_shader_parameter("rough"))
	elif mat is StandardMaterial3D:
		var s := mat as StandardMaterial3D
		m.albedo_color = s.albedo_color
		m.albedo_texture = _plain(s.albedo_texture, tex_cache)
		m.roughness = s.roughness
		m.metallic = 0.0
		m.transparency = s.transparency
		m.alpha_scissor_threshold = s.alpha_scissor_threshold
		m.cull_mode = s.cull_mode
		m.emission_enabled = s.emission_enabled
		m.emission = s.emission
	m.resource_name = mat.resource_name if mat else ""
	return m


static func _plain(tex: Variant, cache: Dictionary) -> Texture2D:
	if not (tex is Texture2D):
		return null
	var key := (tex as Texture2D).get_instance_id()
	if cache.has(key):
		return cache[key]
	var img := (tex as Texture2D).get_image()
	var out: Texture2D = null
	if img:
		img = img.duplicate()
		if img.is_compressed():
			img.decompress()
		img.clear_mipmaps()
		if img.get_width() > MAX_TEX or img.get_height() > MAX_TEX:
			var k := float(MAX_TEX) / maxf(img.get_width(), img.get_height())
			img.resize(int(img.get_width() * k), int(img.get_height() * k), Image.INTERPOLATE_LANCZOS)
		out = ImageTexture.create_from_image(img)
	cache[key] = out
	return out


## The car's physics: its own unpacked data/*.ini when present, else a Drift
## Lab build. Either way resized to the model's wheels.
static func _spec(car_dir: String, base_build: String, model: Dictionary) -> Dictionary:
	var lib := PartsLibrary.new()
	lib.reload()
	var build: Dictionary = {}
	var physics := ""
	if FileAccess.file_exists(car_dir.path_join("data/car.ini")):
		var imp := AcCarImporter.import_folder(car_dir)
		if imp.ok:
			# The importer registers its parts in the user library.
			lib.reload()
			build = imp.build
			physics = "AC data (unpacked)"
	if build.is_empty():
		var path := "res://data/cars/%s.json" % base_build
		var b: Variant = JsonUtil.load_file(path)
		if not (b is Dictionary):
			return {"ok": false, "message": "Base build %s not found." % base_build}
		build = b
		physics = "stand-in: " + str(build.get("name", base_build))
	var r := CarBuilder.assemble(build, lib)
	if not r.ok:
		return {"ok": false, "message": "Couldn't assemble %s: %s" % [build.get("id", "?"), ", ".join(r.errors)]}
	var spec: Dictionary = r.spec
	var radii := [spec.get("radius_f"), spec.get("radius_r")]
	AcCarModel.apply_geometry(spec, model)
	if physics.begins_with("AC data"):
		# The model gives wheel *positions*; tyres.ini has the real radius
		# (the model's wheel-centre height includes AC's GRAPHICS_OFFSET).
		spec["radius_f"] = radii[0]
		spec["radius_r"] = radii[1]
	if spec.get("eye") is Vector3:
		spec["eye"] = _arr(spec.eye)
	var ui: Variant = JsonUtil.load_file(car_dir.path_join("ui/ui_car.json"))
	if ui is Dictionary and str(ui.get("name", "")) != "":
		spec["name"] = str(ui.name).strip_edges().trim_prefix("-").strip_edges()
	return {"ok": true, "spec": spec, "physics": physics}


static func _arr(v: Vector3) -> Array:
	return [snappedf(v.x, 0.0001), snappedf(v.y, 0.0001), snappedf(v.z, 0.0001)]
