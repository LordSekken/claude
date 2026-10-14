extends SceneTree
## Exports an Assetto Corsa car folder for DriftGame/Unreal:
##   godot --headless --path drift-sim -s tools/export_unreal_car.gd -- <ac_car_dir> <unreal_project_dir> [skin] [base_build]
## See UnrealCarExporter. Then in Unreal: py "<project>/Scripts/build_car.py"

func _init() -> void:
	await process_frame
	var args := OS.get_cmdline_user_args()
	if args.size() < 2:
		printerr("usage: -- <ac_car_dir> <unreal_project_dir> [skin] [base_build]")
		quit(2)
		return
	var skin := args[2] if args.size() > 2 else ""
	var base := args[3] if args.size() > 3 else "miata_na_bp_turbo"
	var r := UnrealCarExporter.export_car(args[0].replace("\\", "/"), args[1].replace("\\", "/"), skin, base)
	print(("OK   " if r.ok else "FAIL ") + str(r.message))
	if r.ok:
		print("     physics: %s, spec: %s" % [r.physics, r.spec_id])
	quit(0 if r.ok else 1)
