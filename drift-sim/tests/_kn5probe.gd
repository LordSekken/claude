extends SceneTree
func _init() -> void:
	var t0 := Time.get_ticks_msec()
	var args := OS.get_cmdline_user_args()
	var r := Kn5Loader.load_model(args[0], args[1] if args.size() > 1 else "")
	print("ok=", r.ok, " err=", r.get("error", ""), " meshes=", r.get("meshes", 0), " ms=", Time.get_ticks_msec() - t0)
	if r.ok:
		var keys: Array = r.named.keys()
		var interesting := keys.filter(func(k: String) -> bool: var u := k.to_upper(); return u.begins_with("WHEEL") or u.begins_with("STEER") or u.begins_with("SUSP") or u.begins_with("COCKPIT") or u.begins_with("DRIVER") or u.begins_with("HUB") or u.begins_with("DISC") or u.begins_with("TYRE") or u.begins_with("RIM"))
		print("named nodes: ", keys.size(), " interesting: ", interesting)
		for k in ["WHEEL_LF", "WHEEL_RF", "WHEEL_LR", "WHEEL_RR", "STEER_HR"]:
			if r.named.has(k):
				var n: Node3D = r.named[k]
				var gt := n.transform
				var p := n.get_parent()
				while p:
					if p is Node3D: gt = (p as Node3D).transform * gt
					p = p.get_parent()
				print(k, " at ", gt.origin)
		r.root.free()
	quit()
