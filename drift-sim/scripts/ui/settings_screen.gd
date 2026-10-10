class_name SettingsScreen
extends Control
## Settings: Claude API key, assists, and controller / steering-wheel setup.
## Wheels (e.g. Logitech G920) are bound by moving the control: no guessing
## which axis is which, and any pedal direction/range works.

signal navigate(target: String)

var settings: Node
var _capture := ""          # binding being captured
var _baseline := {}         # "dev:axis" -> value
var _capture_best := {}     # {device, axis, released, pressed, dev}
var _capture_timer := 0.0
var _monitor: Label
var _status: Label
var _bind_labels := {}
var _ffb_status: Label


func _ready() -> void:
	settings = get_node("/root/Settings")
	var vb := UiUtil.screen(self, 900)
	var head := HBoxContainer.new()
	head.add_child(UiUtil.button("< Menu", func() -> void:
		settings.save_settings()
		navigate.emit("menu")))
	head.add_child(UiUtil.title("  Settings & Controls"))
	vb.add_child(head)

	vb.add_child(UiUtil.label("Claude (AI track + car building)", 22, Color(1, 0.8, 0.25)))
	var key := LineEdit.new()
	key.secret = true
	key.text = str(settings.api_key)
	key.placeholder_text = "sk-ant-... (from console.anthropic.com)"
	key.text_changed.connect(func(t: String) -> void: settings.api_key = t.strip_edges())
	vb.add_child(UiUtil.row("Anthropic API key", key))
	var model := LineEdit.new()
	model.text = str(settings.model)
	model.text_changed.connect(func(t: String) -> void: settings.model = t.strip_edges())
	vb.add_child(UiUtil.row("Model", model))
	vb.add_child(UiUtil.label("The key is stored only on this PC (%s)." % ProjectSettings.globalize_path("user://settings.cfg"), 13, Color(0.6, 0.6, 0.65)))

	vb.add_child(HSeparator.new())
	vb.add_child(UiUtil.label("Assetto Corsa", 22, Color(1, 0.8, 0.25)))
	var ac_edit := LineEdit.new()
	ac_edit.text = str(settings.ac_root)
	ac_edit.placeholder_text = "auto-detect (Steam)  e.g. D:/SteamLibrary/steamapps/common/assettocorsa"
	ac_edit.text_changed.connect(func(t: String) -> void: settings.ac_root = t.strip_edges())
	vb.add_child(UiUtil.row("AC folder", ac_edit))
	var found := AcPaths.cars_dir()
	vb.add_child(UiUtil.label(("Found %d cars in %s" % [AcPaths.list_cars().size(), found]) if found != "" else "Assetto Corsa not found yet - paste its folder above.", 13, Color(0.6, 0.9, 0.6) if found != "" else Color(1, 0.8, 0.4), true))
	vb.add_child(HSeparator.new())
	vb.add_child(UiUtil.label("Assists", 22, Color(1, 0.8, 0.25)))
	vb.add_child(UiUtil.row("Volume", UiUtil.slider(0, 1, 0.05, float(settings.volume), func(v: float) -> void: settings.volume = v), 280))
	vb.add_child(UiUtil.row("Steering assist (pad/keyboard)", UiUtil.slider(0, 1, 0.05, float(settings.steering_assist), func(v: float) -> void: settings.steering_assist = v), 280))
	for pair in [["auto_clutch", "Auto clutch"], ["auto_shift", "Automatic gearbox"], ["units_kmh", "km/h (off = mph)"]]:
		var cb := CheckBox.new()
		cb.text = pair[1]
		cb.button_pressed = bool(settings.get(pair[0]))
		cb.toggled.connect(func(on: bool) -> void: settings.set(pair[0], on))
		vb.add_child(cb)

	vb.add_child(HSeparator.new())
	vb.add_child(UiUtil.label("Controllers", 22, Color(1, 0.8, 0.25)))
	vb.add_child(UiUtil.label("Xbox / XInput pads work out of the box: left stick steer, RT gas, LT brake, A handbrake, B/X shift up/down, Y camera, right-stick look / recentre VR, Back reset, Start pause.", 14, Color(0.75, 0.75, 0.8)))
	var pads := Input.get_connected_joypads()
	var names := []
	for p in pads:
		names.append("#%d  %s" % [p, Input.get_joy_name(p)])
	vb.add_child(UiUtil.label("Connected: " + (", ".join(names) if names.size() > 0 else "none detected - plug in and reopen this screen"), 15))

	var wheel := CheckBox.new()
	wheel.text = "Use a steering wheel + pedals (e.g. Logitech G920)"
	wheel.button_pressed = bool(settings.use_wheel)
	wheel.toggled.connect(func(on: bool) -> void: settings.use_wheel = on)
	vb.add_child(wheel)
	var rot := SpinBox.new()
	rot.min_value = 180
	rot.max_value = 1080
	rot.step = 10
	rot.value = float(settings.wheel_rotation_deg)
	rot.suffix = "°"
	rot.value_changed.connect(func(v: float) -> void: settings.wheel_rotation_deg = v)
	vb.add_child(UiUtil.row("Wheel rotation (set the same in G HUB)", rot, 300))
	vb.add_child(UiUtil.label("Wheel setup: click a Bind button, then move that control all the way (turn the wheel to the RIGHT for steering) and release. In G HUB, turn OFF the 'centering spring' so Drift Lab's force feedback can do the work.", 14, Color(0.75, 0.75, 0.8)))
	var grid := GridContainer.new()
	grid.columns = 3
	for b in [["steer", "Steering (turn right)"], ["throttle", "Throttle pedal"], ["brake", "Brake pedal"], ["clutch", "Clutch pedal"], ["handbrake", "Handbrake (axis)"]]:
		grid.add_child(UiUtil.label(b[1], 15))
		grid.add_child(UiUtil.button("Bind", func() -> void: _start_capture(b[0]), 34))
		var l := UiUtil.label(_desc(b[0]), 13, Color(0.6, 0.9, 0.6))
		_bind_labels[b[0]] = l
		grid.add_child(l)
	for b in [["shift_up", "Shift up (paddle)"], ["shift_down", "Shift down (paddle)"], ["handbrake_btn", "Handbrake (button)"], ["gear_1", "H-shifter 1st"], ["gear_2", "H-shifter 2nd"], ["gear_3", "H-shifter 3rd"], ["gear_4", "H-shifter 4th"], ["gear_5", "H-shifter 5th"], ["gear_6", "H-shifter 6th"], ["gear_r", "H-shifter reverse"]]:
		grid.add_child(UiUtil.label(b[1], 15))
		grid.add_child(UiUtil.button("Bind", func() -> void: _start_capture("btn:" + b[0]), 34))
		var l := UiUtil.label(_desc("btn:" + b[0]), 13, Color(0.6, 0.9, 0.6))
		_bind_labels["btn:" + b[0]] = l
		grid.add_child(l)
	vb.add_child(grid)
	vb.add_child(HSeparator.new())
	vb.add_child(UiUtil.label("Force feedback (wheels)", 22, Color(1, 0.8, 0.25)))
	vb.add_child(UiUtil.label("Self-aligning torque from the front tyres: the wheel goes light when the fronts let go, snaps back on a transition, and kicks when you hit a wall. Runs through ffb_helper.exe (shipped next to DriftLab.exe). Close other force-feedback tools while playing - only one app can drive the wheel's motor at a time.", 14, Color(0.75, 0.75, 0.8)))
	var ffb: Node = get_node("/root/ForceFeedback")
	var ffb_on := CheckBox.new()
	ffb_on.text = "Enable force feedback"
	ffb_on.button_pressed = bool(settings.ffb_enabled)
	ffb_on.toggled.connect(func(on: bool) -> void:
		settings.ffb_enabled = on
		if on:
			ffb.start()
		else:
			ffb.stop())
	vb.add_child(ffb_on)
	vb.add_child(UiUtil.row("Strength", UiUtil.slider(0, 1.5, 0.05, float(settings.ffb_gain), func(v: float) -> void: settings.ffb_gain = v), 280))
	vb.add_child(UiUtil.row("Damping (weight)", UiUtil.slider(0, 2, 0.05, float(settings.ffb_damper), func(v: float) -> void: settings.ffb_damper = v), 280))
	var ffb_row := HBoxContainer.new()
	ffb_row.add_child(UiUtil.button("Test (pulls left, then right)", func() -> void: ffb.test(), 36))
	ffb_row.add_child(UiUtil.button("Open log", func() -> void: OS.shell_open(OS.get_environment("TEMP").path_join("DriftLab_ffb.log")), 36))
	_ffb_status = UiUtil.label("", 14, Color(0.6, 0.9, 0.6), true)
	ffb_row.add_child(_ffb_status)
	vb.add_child(ffb_row)

	vb.add_child(HSeparator.new())
	vb.add_child(UiUtil.label("View & VR", 22, Color(1, 0.8, 0.25)))
	vb.add_child(UiUtil.row("Default camera (monitor)", UiUtil.option(["Chase", "Close chase", "Hood", "Cockpit (first person)"], int(settings.camera_mode), func(i: int) -> void: settings.camera_mode = i), 280))
	var apex := CheckBox.new()
	apex.text = "Cockpit view glances into the drift (desktop only)"
	apex.button_pressed = bool(settings.look_to_apex)
	apex.toggled.connect(func(on: bool) -> void: settings.look_to_apex = on)
	vb.add_child(apex)
	vb.add_child(UiUtil.button("Reset seat position", func() -> void:
		settings.seat_offset = Vector3.ZERO
		_status.text = "Seat reset.", 36))
	vb.add_child(UiUtil.label("VR (Quest 2 etc.): connect with Quest Link, Air Link or Virtual Desktop (set it as the OpenXR runtime), then start 'Drift Lab VR.bat'. In the car: V or right-stick click recentres your view; numpad 8/2 moves the seat forward/back and 9/3 up/down. Menus show on the PC monitor.", 14, Color(0.75, 0.75, 0.8)))
	_status = UiUtil.label("", 18, Color(1, 0.85, 0.3), true)
	vb.add_child(_status)
	vb.add_child(UiUtil.label("Live axes", 16, Color(1, 0.8, 0.25)))
	_monitor = UiUtil.label("", 13, Color(0.8, 0.8, 0.85))
	vb.add_child(_monitor)
	vb.add_child(UiUtil.button("Save", func() -> void:
		settings.save_settings()
		_status.text = "Saved."))


func _desc(what: String) -> String:
	if what.begins_with("btn:"):
		var name := what.substr(4)
		if name == "handbrake_btn":
			name = "handbrake"
		var b: Dictionary = settings.button_bindings.get(name, {})
		return "button %d on #%d" % [b.button, b.device] if not b.is_empty() else "-"
	var bind: Dictionary = settings.get(what + "_binding")
	if bind.is_empty():
		return "-"
	if what == "steer" and int(bind.get("device", -1)) < 0:
		return "not bound yet"
	if what == "steer":
		return "axis %d on #%d%s" % [bind.axis, bind.device, " (inverted)" if bind.get("invert", false) else ""]
	return "axis %d on #%d  (%.2f -> %.2f)" % [bind.axis, bind.device, float(bind.released), float(bind.pressed)]


func _start_capture(what: String) -> void:
	_capture = what
	_capture_best = {}
	_capture_timer = 0.0
	_baseline.clear()
	for dev in Input.get_connected_joypads():
		for ax in JOY_AXIS_SDL_MAX:
			_baseline["%d:%d" % [dev, ax]] = Input.get_joy_axis(dev, ax as JoyAxis)
	_status.text = ("Press the button / paddle now..." if what.begins_with("btn:") else "Move it all the way now, then let go...")


func _input(ev: InputEvent) -> void:
	if _capture.begins_with("btn:") and ev is InputEventJoypadButton and ev.pressed:
		var name := _capture.substr(4)
		if name == "handbrake_btn":
			name = "handbrake"
			var e := InputEventJoypadButton.new()
			e.button_index = ev.button_index
			e.device = ev.device
			InputMap.action_add_event("handbrake", e)
		settings.button_bindings[name] = {"device": ev.device, "button": ev.button_index}
		_bind_labels[_capture].text = _desc(_capture)
		_status.text = "Bound."
		_capture = ""
		get_viewport().set_input_as_handled()


func _process(dt: float) -> void:
	if _ffb_status:
		_ffb_status.text = str(get_node("/root/ForceFeedback").status)
	var txt := ""
	for dev in Input.get_connected_joypads():
		var parts := []
		for ax in JOY_AXIS_SDL_MAX:
			parts.append("%d:%+.2f" % [ax, Input.get_joy_axis(dev, ax as JoyAxis)])
		txt += "#%d %s\n   %s\n" % [dev, Input.get_joy_name(dev), "  ".join(parts)]
	_monitor.text = txt if txt != "" else "(no controllers)"
	if _capture == "" or _capture.begins_with("btn:"):
		return
	# Track the axis with the largest deviation from where it started.
	for key: String in _baseline:
		var dev := int(key.get_slice(":", 0))
		var ax := int(key.get_slice(":", 1))
		var v := Input.get_joy_axis(dev, ax as JoyAxis)
		var dev_amt := absf(v - float(_baseline[key]))
		if dev_amt > 0.4 and dev_amt > float(_capture_best.get("amount", 0.0)):
			_capture_best = {"device": dev, "axis": ax, "released": float(_baseline[key]), "pressed": v, "amount": dev_amt}
	if not _capture_best.is_empty():
		_capture_timer += dt
		if _capture_timer > 1.5:
			var b := _capture_best.duplicate()
			b.erase("amount")
			if _capture == "steer":
				settings.steer_binding = {"device": b.device, "axis": b.axis, "invert": float(b.pressed) < float(b.released)}
				settings.use_wheel = true
			else:
				settings.set(_capture + "_binding", b)
			_bind_labels[_capture].text = _desc(_capture)
			_status.text = "Bound %s." % _capture
			_capture = ""
