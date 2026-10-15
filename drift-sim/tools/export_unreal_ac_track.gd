extends SceneTree
## Exports an Assetto Corsa track (.kn5) for DriftGame/Unreal:
##   godot --headless --path drift-sim -s tools/export_unreal_ac_track.gd -- <track.kn5> <unreal project>/Tracks [name]
## Then in Unreal: py "<project>/Scripts/build_track_level.py" (or the headless build).

func _init() -> void:
	await process_frame
	var args := OS.get_cmdline_user_args()
	if args.size() < 2:
		printerr("usage: -- <track.kn5> <out_dir> [name]")
		quit(2)
		return
	var r := UnrealAcTrackExporter.export_track(args[0].replace("\\", "/"), args[1].replace("\\", "/"), args[2] if args.size() > 2 else "")
	print(("OK   " if r.ok else "FAIL ") + str(r.message))
	quit(0 if r.ok else 1)
