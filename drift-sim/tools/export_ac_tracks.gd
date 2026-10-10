extends SceneTree
## Exports every built-in track as an Assetto Corsa track folder:
##   godot --headless --path drift-sim -s tools/export_ac_tracks.gd -- <out_dir>

func _init() -> void:
	await process_frame
	var args := OS.get_cmdline_user_args()
	var out := args[0] if args.size() > 0 else ProjectSettings.globalize_path("user://ac_tracks")
	DirAccess.make_dir_recursive_absolute(out)
	var failed := 0
	for path in JsonUtil.list_json("res://tracks"):
		var spec: Dictionary = JsonUtil.load_file(path)
		if (spec.get("points", []) as Array).size() < 2:
			continue  # pads-only lots have no start line / timing
		var r := AcTrackExporter.export_track(spec, out)
		print(("OK   " if r.ok else "FAIL ") + str(r.message))
		if not r.ok:
			failed += 1
	quit(1 if failed > 0 else 0)
