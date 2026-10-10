class_name AcTrackExporter
extends RefCounted
## Exports a Drift Lab track as an Assetto Corsa track mod skeleton:
##   <id>/<id>.kn5            geometry with AC physics names (1ROAD_*, 1KERB_*,
##                             1GRASS_*, 1WALL_*) + AC_START_0, AC_PIT_0,
##                             AC_HOTLAP_START_0, AC_TIME_0_L/R dummies
##   <id>/ui/ui_track.json, ui/outline.png, ui/preview.png
##   <id>/map.png, data/map.ini, data/surfaces.ini
## The folder drops straight into assettocorsa/content/tracks.

static func export_track(spec_in: Dictionary, out_dir: String) -> Dictionary:
	var spec := TrackSpec.normalized(spec_in)
	var id := JsonUtil.slugify(str(spec.get("name", "drift_lab_track")))
	var root_dir := out_dir.path_join(id)
	for d in [root_dir, root_dir.path_join("ui"), root_dir.path_join("data")]:
		DirAccess.make_dir_recursive_absolute(d)
	for stale in [id + ".glb", "HOW_TO_FINISH.txt"]:  # older Drift Lab exports
		if FileAccess.file_exists(root_dir.path_join(stale)):
			DirAccess.remove_absolute(root_dir.path_join(stale))
	var built := TrackBuilder.build(spec)
	var root: Node3D = built.root
	var samples: Dictionary = built.samples
	# Strip things AC doesn't want in the model.
	for c in root.get_children():
		if c is WorldEnvironment or c is DirectionalLight3D or c is OmniLight3D:
			root.remove_child(c)
			c.free()
	# Spawn/timing dummies. Kn5Writer converts axes, so a Godot transform whose
	# -Z points along the track becomes an AC dummy whose +Z does.
	var start := TrackSpec.transform_at(samples, float(spec.start), 0.3)
	var existing := root.get_node_or_null("AC_START_0")
	if existing:
		root.remove_child(existing)
		existing.free()
	var dummies := []
	for nm in ["AC_START_0", "AC_HOTLAP_START_0", "AC_PIT_0"]:
		dummies.append({"name": nm, "xform": Transform3D(start.basis, start.origin + (start.basis.z * 8.0 if nm == "AC_PIT_0" else Vector3.ZERO))})
	var i0 := clampi(int(float(spec.start) * (samples.pos as PackedVector3Array).size()), 0, (samples.pos as PackedVector3Array).size() - 1)
	var half := float(samples.width[i0]) * 0.5 + 1.0
	for side in [["L", -1.0], ["R", 1.0]]:
		dummies.append({"name": "AC_TIME_0_" + side[0], "xform": Transform3D(start.basis, start.origin + (samples.right[i0] as Vector3) * side[1] * half)})
	var wr := Kn5Writer.write(root, dummies, root_dir.path_join(id + ".kn5"))
	root.free()
	if not wr.ok:
		return {"ok": false, "message": "kn5 export failed: %s" % wr.error}

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
	_write(root_dir.path_join("README.txt"), """%s - made with Drift Lab
Copy this folder into assettocorsa/content/tracks/ and pick it in Content Manager.
%s.kn5 holds the road (1ROAD), kerbs (1KERB), run-off (1GRASS/1GRAVEL/1DIRT), walls (1WALL)
and the AC_START / AC_PIT / AC_TIME markers. No AI line is included, so use Practice or
free roam / a server without AI cars.
""" % [str(spec.get("name", id)), id])
	return {"ok": true, "message": "Exported to %s - copy that folder into assettocorsa/content/tracks." % root_dir}


static func _write(path: String, text: String) -> void:
	var f := FileAccess.open(path, FileAccess.WRITE)
	if f:
		f.store_string(text)
