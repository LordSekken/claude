extends SceneTree
func _init() -> void:
	var l := Kn5Loader.new()
	var root := l._load(OS.get_cmdline_user_args()[0], "")
	var shaders := {}
	for m: Dictionary in l.materials:
		var key: String = m.shader
		shaders[key] = shaders.get(key, 0) + 1
	print(shaders)
	for m: Dictionary in l.materials:
		var smp: Dictionary = m.samplers
		if str(smp.values()).contains("skin") or str(m.name).to_lower().contains("paint") or str(m.name).to_lower().contains("body"):
			print(m.name, " | ", m.shader, " | ", smp, " | ", (m.props as Dictionary).keys())
	root.free()
	quit()
