class_name MainMenu
extends Control
## Title screen: pick track + car, then drive. Fully navigable with a gamepad.

signal navigate(target: String)

var _track_opt: OptionButton
var _car_opt: OptionButton
var _car_info: Label
var _error: Label
var _tracks: PackedStringArray
var _cars: PackedStringArray


func _ready() -> void:
	var app: Node = get_node("/root/App")
	app.parts.reload()
	var vb := UiUtil.screen(self, 720)
	var t := UiUtil.label("DRIFT LAB", 64, Color(1, 0.8, 0.25))
	t.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	vb.add_child(t)
	var sub := UiUtil.label("build it · tune it · send it", 18, Color(0.7, 0.7, 0.75))
	sub.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	vb.add_child(sub)
	vb.add_child(HSeparator.new())

	_tracks = app.list_tracks()
	var names := []
	var sel := 0
	for i in _tracks.size():
		var d: Variant = JsonUtil.load_file(_tracks[i])
		names.append((str(d.get("name", _tracks[i].get_file())) if d is Dictionary else _tracks[i].get_file()) + ("  (mine)" if _tracks[i].begins_with("user://") else ""))
		if _tracks[i] == app.selected_track_path:
			sel = i
	if app.external_track_model != "":
		names.append("Model: " + app.external_track_model.get_file())
		sel = names.size() - 1
	_track_opt = UiUtil.option(names, sel, _on_track)
	vb.add_child(UiUtil.row("Track", _track_opt, 120))
	var ext_row := HBoxContainer.new()
	ext_row.add_child(UiUtil.button("Load track model (.glb / .fbx / converted AC track)...", _load_external))
	vb.add_child(ext_row)

	_cars = app.list_cars()
	var cnames := []
	var csel := 0
	for i in _cars.size():
		var d: Variant = JsonUtil.load_file(_cars[i])
		cnames.append((str(d.get("name", _cars[i].get_file())) if d is Dictionary else _cars[i].get_file()) + ("  (mine)" if _cars[i].begins_with("user://") else ""))
		if _cars[i] == app.selected_car_path:
			csel = i
	_car_opt = UiUtil.option(cnames, csel, _on_car)
	vb.add_child(UiUtil.row("Car", _car_opt, 120))
	_car_info = UiUtil.label("", 15, Color(0.75, 0.8, 0.85), true)
	vb.add_child(_car_info)
	_on_car(csel)
	if app.external_track_model == "":
		_on_track(sel)

	var drive := UiUtil.button("DRIVE", func() -> void: navigate.emit("drive"), 64)
	drive.add_theme_font_size_override("font_size", 28)
	vb.add_child(drive)
	var grid := GridContainer.new()
	grid.columns = 2
	grid.add_theme_constant_override("h_separation", 10)
	grid.add_theme_constant_override("v_separation", 10)
	for pair in [["Garage / Car Builder", "garage"], ["Track Editor", "editor"], ["Photo -> Track", "photo"], ["Settings & Controls", "settings"]]:
		var b := UiUtil.button(pair[0], func() -> void: navigate.emit(pair[1]))
		b.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		grid.add_child(b)
	vb.add_child(grid)
	vb.add_child(UiUtil.button("Quit", func() -> void: navigate.emit("quit")))
	_error = UiUtil.label("", 16, Color(1, 0.4, 0.4), true)
	vb.add_child(_error)
	drive.grab_focus.call_deferred()


func show_error(msg: String) -> void:
	_error.text = msg


func _on_track(i: int) -> void:
	var app: Node = get_node("/root/App")
	if i < _tracks.size():
		app.selected_track_path = _tracks[i]
		app.external_track_model = ""


func _on_car(i: int) -> void:
	var app: Node = get_node("/root/App")
	if i >= _cars.size():
		return
	app.selected_car_path = _cars[i]
	var build: Variant = JsonUtil.load_file(_cars[i])
	if not (build is Dictionary):
		_car_info.text = "Could not read car file."
		return
	var r := CarBuilder.assemble(build, app.parts)
	if not r.ok:
		_car_info.text = "Build error: " + ", ".join(r.errors)
		return
	var s: Dictionary = r.stats
	_car_info.text = "%s · %s · %s\n%d hp @ %d · %d Nm · %d kg · %d hp/t · %.0f%% front · 0-100 %.1fs · %d° lock" % [
		r.spec.chassis_name, r.spec.engine_name, s.drivetrain, s.peak_hp, s.peak_hp_rpm, s.peak_torque_nm,
		s.mass_kg, s.hp_per_tonne, s.weight_front_pct, s.zero_to_100_s, s.steer_lock_deg]


func _load_external() -> void:
	UiUtil.file_dialog(self, PackedStringArray(["*.glb, *.gltf, *.fbx ; Track models"]), func(path: String) -> void:
		var app: Node = get_node("/root/App")
		app.external_track_model = path
		navigate.emit("menu"))
