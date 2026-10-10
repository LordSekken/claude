class_name AcTrackExporter
extends RefCounted
## Exports a Drift Lab track as an Assetto Corsa track mod skeleton:
##   <id>/<id>.glb            geometry with AC physics names (1ROAD_*, 1KERB_*,
##                             1GRASS_*, 1WALL_*) + AC_START_0, AC_PIT_0,
##                             AC_HOTLAP_START_0, AC_TIME_0_L/R dummies
##   <id>/ui/ui_track.json, ui/outline.png, ui/preview.png
##   <id>/map.png, data/map.ini, data/surfaces.ini
##   <id>/HOW_TO_FINISH.txt   glb -> FBX -> kn5 steps (ksEditor)
## AC only loads .kn5 models, which are made with Kunos' ksEditor from FBX.

static func export_track(spec_in: Dictionary, out_dir: String) -> Dictionary:
	var spec := TrackSpec.normalized(spec_in)
	var id := JsonUtil.slugify(str(spec.get("name", "drift_lab_track")))
	var root_dir := out_dir.path_join(id)
	for d in [root_dir, root_dir.path_join("ui"), root_dir.path_join("data")]:
		DirAccess.make_dir_recursive_absolute(d)
	var built := TrackBuilder.build(spec)
	var root: Node3D = built.root
	var samples: Dictionary = built.samples
	# Strip things AC doesn't want in the model.
	for c in root.get_children():
		if c is WorldEnvironment or c is DirectionalLight3D or c is OmniLight3D:
			root.remove_child(c)
			c.free()
	# Dummies. AC cars face the dummy's +Z, ours face -Z: rotate 180.
	var flip := Basis(Vector3.UP, PI)
	var start := TrackSpec.transform_at(samples, float(spec.start), 0.3)
	var existing := root.get_node_or_null("AC_START_0")
	if existing:
		root.remove_child(existing)
		existing.free()
	for nm in ["AC_START_0", "AC_HOTLAP_START_0", "AC_PIT_0"]:
		var n := Node3D.new()
		n.name = nm
		n.transform = Transform3D(start.basis * flip, start.origin + (start.basis.z * 8.0 if nm == "AC_PIT_0" else Vector3.ZERO))
		root.add_child(n)
	var i0 := clampi(int(float(spec.start) * (samples.pos as PackedVector3Array).size()), 0, (samples.pos as PackedVector3Array).size() - 1)
	var half := float(samples.width[i0]) * 0.5 + 1.0
	for side in [["L", -1.0], ["R", 1.0]]:
		var n := Node3D.new()
		n.name = "AC_TIME_0_" + side[0]
		n.transform = Transform3D(start.basis * flip, start.origin + (samples.right[i0] as Vector3) * side[1] * half)
		root.add_child(n)
	var gltf := GLTFDocument.new()
	var state := GLTFState.new()
	var err := gltf.append_from_scene(root, state)
	if err == OK:
		err = gltf.write_to_filesystem(state, root_dir.path_join(id + ".glb"))
	root.free()
	if err != OK:
		return {"ok": false, "message": "glTF export failed (%d)" % err}

	# Map + UI images.
	var outline := TrackRaster.render(spec, 512, Color(0, 0, 0, 0), Color(1, 1, 1, 1))
	(outline.image as Image).save_png(root_dir.path_join("ui/outline.png"))
	(outline.image as Image).save_png(root_dir.path_join("map.png"))
	var prev := TrackRaster.render(spec, 512, Color(0.12, 0.13, 0.15), Color(0.9, 0.9, 0.9), true)
	(prev.image as Image).save_png(root_dir.path_join("ui/preview.png"))
	# AC map.ini: world = pixel * SCALE_FACTOR - offset.
	var scale := 1.0 / float(outline.scale)
	var origin: Vector2 = outline.origin
	_write(root_dir.path_join("data/map.ini"), "[PARAMETERS]\nWIDTH=512\nHEIGHT=512\nMARGIN=0\nSCALE_FACTOR=%f\nX_OFFSET=%f\nZ_OFFSET=%f\nDRAWING_SIZE=10\n" % [scale, -origin.x, -origin.y])
	var surf := ""
	var rows := [["ROAD", 0.99, 1, 0.0], ["KERB", 0.93, 1, 0.0], ["GRASS", 0.6, 0, 0.0], ["GRAVEL", 0.6, 0, 0.12], ["DIRT", 0.72, 0, 0.05], ["SAND", 0.5, 0, 0.2], ["WALL", 0.2, 0, 0.0]]
	for i in rows.size():
		var r: Array = rows[i]
		surf += "[SURFACE_%d]\nKEY=%s\nFRICTION=%.2f\nDAMPING=%.2f\nWAV=\nWAV_PITCH=0\nFF_EFFECT=NULL\nDIRT_ADDITIVE=0\nIS_VALID_TRACK=%d\nBLACK_FLAG_TIME=0\nSIN_HEIGHT=0\nSIN_LENGTH=0\nIS_PITLANE=0\nVIBRATION_GAIN=0\nVIBRATION_LENGTH=0\n\n" % [i, r[0], r[1], r[3], r[2]]
	_write(root_dir.path_join("data/surfaces.ini"), surf)
	var length := float(samples.get("length", 0.0))
	var ui := {
		"name": str(spec.get("name", id)), "description": str(spec.get("description", "Made in Drift Lab")),
		"tags": ["drift", "drift lab", "original"], "geotags": [], "country": "", "city": "",
		"length": "%d" % int(length), "width": "%d" % int(float(spec.width)), "pitboxes": "1", "run": "clockwise",
	}
	JsonUtil.save_file(root_dir.path_join("ui/ui_track.json"), ui)
	_write(root_dir.path_join("HOW_TO_FINISH.txt"), """Drift Lab -> Assetto Corsa
=========================
Assetto Corsa loads tracks as .kn5 files, which only Kunos' free ksEditor can create (from FBX).

1. Open %s.glb in Blender (File > Import > glTF). Keep object names - AC uses them:
   1ROAD_* = driveable road, 1KERB_* = kerbs, 1GRASS_*/1GRAVEL_*/1SAND_*/1DIRT_* = run-off,
   1WALL_* = walls, AC_START_0 / AC_PIT_0 / AC_HOTLAP_START_0 / AC_TIME_0_L / AC_TIME_0_R = spawn & timing.
2. File > Export > FBX (Apply Scalings: FBX Units Scale, Forward: -Z, Up: Y, include Empties).
3. Open the FBX in ksEditor (in the AC SDK), assign shaders (ksPerPixel for road/grass, ksTree for trees),
   then File > Save kn5 as %s.kn5 into this folder.
4. Copy this whole folder to assettocorsa/content/tracks/ and it will show up in Content Manager.

map.png, data/map.ini, data/surfaces.ini and ui/ are already set up.
""" % [id, id])
	return {"ok": true, "message": "Exported to %s (see HOW_TO_FINISH.txt for the kn5 step)." % root_dir}


static func _write(path: String, text: String) -> void:
	var f := FileAccess.open(path, FileAccess.WRITE)
	if f:
		f.store_string(text)
