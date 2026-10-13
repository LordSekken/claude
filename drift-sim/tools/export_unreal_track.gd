extends SceneTree
## Exports a saved Drift Lab track (user://tracks/<name>.json) for DriftGame/Unreal:
##   godot --headless --path drift-sim -s tools/export_unreal_track.gd -- <track.json> <out_dir> [name]
## [name] renames the track (and so the export folder / Unreal map id).

func _init() -> void:
	await process_frame
	var args := OS.get_cmdline_user_args()
	if args.size() < 2:
		printerr("usage: -- <track.json> <out_dir> [name]")
		quit(2)
		return
	var spec: Variant = JsonUtil.load_file(args[0])
	if not (spec is Dictionary):
		printerr("FAIL can't read track " + args[0])
		quit(1)
		return
	if args.size() > 2:
		spec["name"] = args[2]
	DirAccess.make_dir_recursive_absolute(args[1])
	var r := UnrealTrackExporter.export_track(spec, args[1])
	print(("OK   " if r.ok else "FAIL ") + str(r.message))
	quit(0 if r.ok else 1)
