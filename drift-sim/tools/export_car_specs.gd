extends SceneTree
## Exports every built-in car build, assembled, as DriftCore spec JSON
## (what the Unreal project loads):
##   godot --headless --path drift-sim -s tools/export_car_specs.gd -- <out_dir>

func _init() -> void:
	await process_frame
	var args := OS.get_cmdline_user_args()
	var out := args[0] if args.size() > 0 else ProjectSettings.globalize_path("user://car_specs")
	DirAccess.make_dir_recursive_absolute(out)
	var lib := PartsLibrary.new()
	lib.reload()
	var failed := 0
	for path in JsonUtil.list_json("res://data/cars"):
		var build: Dictionary = JsonUtil.load_file(path)
		var r := CarBuilder.assemble(build, lib)
		var id := str(build.get("id", path.get_file().get_basename()))
		if not r.ok:
			print("FAIL %s: %s" % [id, ", ".join(r.errors)])
			failed += 1
			continue
		var f := FileAccess.open(out.path_join(id + ".json"), FileAccess.WRITE)
		f.store_string(CarBuilder.spec_json(r.spec, id))
		f.close()
		print("OK   %s" % id)
	quit(1 if failed > 0 else 0)
