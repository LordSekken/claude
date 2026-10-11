class_name ForzaRecordScreen
extends Control
## Record Forza runs via "Data Out" telemetry and turn them into a track.

signal navigate(target: String)

const ROLES := ["line", "left", "right"]

var tele := ForzaTelemetry.new()
var udp := PacketPeerUDP.new()
var listening := false
var recording := false
var passes: Array = []
var current: ForzaTelemetry.Pass
var _role := 0
var _pkt_count := 0
var _pkt_rate := 0.0
var _rate_t := 0.0
var _last: Dictionary = {}
var _port: SpinBox
var _live: Label
var _list: Label
var _status: RichTextLabel
var _rec_btn: Button
var _name: LineEdit
var _width: SpinBox
var _preview: TextureRect
var result_spec: Dictionary = {}


func _ready() -> void:
	var vb := UiUtil.screen(self, 1000)
	var head := HBoxContainer.new()
	head.add_child(UiUtil.button("< Menu", func() -> void: navigate.emit("menu")))
	head.add_child(UiUtil.title("  Record from Forza"))
	vb.add_child(head)
	vb.add_child(UiUtil.label("In Forza: Settings > HUD and Gameplay > Data Out = ON, Data Out IP = 127.0.0.1 (or this PC's IP if Forza runs on an Xbox), port = the number below. Press Listen and drive a few metres until the line below shows your position.", 15, Color(0.75, 0.75, 0.8)))
	vb.add_child(UiUtil.label("For an exact copy record 3 passes, each from the same start point to the same end point:\n  1. Hugging the LEFT edge - slowly, left tyres right on the edge of the tarmac (kerbs are fine, grass is ignored).\n  2. Hugging the RIGHT edge - the same on the right.\n  3. Your driving line - drift it like normal.\nThe edges set the turns, road width and camber; elevation comes from every pass; kerbs come from where Forza says your tyres hit a rumble strip.", 15, Color(0.75, 0.75, 0.8)))
	_port = SpinBox.new()
	_port.min_value = 1024
	_port.max_value = 65535
	_port.value = 5300
	var row := UiUtil.row("Port", _port, 120)
	row.add_child(UiUtil.button("Listen", _listen, 36))
	vb.add_child(row)
	_live = UiUtil.label("Not listening.", 16, Color(0.6, 0.9, 0.6), true)
	vb.add_child(_live)
	vb.add_child(UiUtil.row("This pass is", UiUtil.option(["My driving line", "Hugging the LEFT edge", "Hugging the RIGHT edge"], 0, func(i: int) -> void: _role = i), 120))
	_rec_btn = UiUtil.button("Record", _toggle_record, 56)
	_rec_btn.add_theme_font_size_override("font_size", 24)
	vb.add_child(_rec_btn)
	_list = UiUtil.label("No passes yet.", 15)
	vb.add_child(_list)
	var row2 := HBoxContainer.new()
	row2.add_child(UiUtil.button("Delete last pass", func() -> void:
		if passes.size() > 0:
			passes.pop_back()
		_update_list(), 36))
	row2.add_child(UiUtil.button("Save recording...", _save_recording, 36))
	row2.add_child(UiUtil.button("Load recording...", _load_recording, 36))
	vb.add_child(row2)
	vb.add_child(HSeparator.new())
	_name = LineEdit.new()
	_name.text = "Forza Zone"
	vb.add_child(UiUtil.row("Track name", _name, 160))
	_width = SpinBox.new()
	_width.min_value = 5
	_width.max_value = 40
	_width.value = 12
	_width.suffix = "m"
	vb.add_child(UiUtil.row("Road width (if no edge passes)", _width, 260))
	vb.add_child(UiUtil.button("Make track", _make, 52))
	_status = RichTextLabel.new()
	_status.bbcode_enabled = true
	_status.fit_content = true
	vb.add_child(_status)
	_preview = TextureRect.new()
	_preview.custom_minimum_size = Vector2(420, 420)
	_preview.stretch_mode = TextureRect.STRETCH_KEEP_ASPECT_CENTERED
	_preview.expand_mode = TextureRect.EXPAND_IGNORE_SIZE
	vb.add_child(_preview)
	var row3 := HBoxContainer.new()
	row3.add_child(UiUtil.button("Open in Track Editor", _open_editor, 40))
	row3.add_child(UiUtil.button("Save + Drive in Drift Lab", _drive, 40))
	row3.add_child(UiUtil.button("Export for Assetto Corsa...", _export_ac, 40))
	vb.add_child(row3)


func _listen() -> void:
	udp.close()
	tele = ForzaTelemetry.new()
	var err := udp.bind(int(_port.value))
	listening = err == OK
	_live.text = ("Listening on port %d - waiting for Forza..." % int(_port.value)) if listening else ("Couldn't open port %d (in use?)" % int(_port.value))


func _toggle_record() -> void:
	if not recording:
		current = ForzaTelemetry.Pass.new()
		current.role = ROLES[_role]
		recording = true
		_rec_btn.text = "Stop"
		if not listening:
			_listen()
	else:
		recording = false
		_rec_btn.text = "Record"
		if current.points.size() >= 20:
			passes.append(current)
		else:
			_status.text = "[color=#ffcc66]That pass was too short (no telemetry received?).[/color]"
		current = null
		_update_list()


func _update_list() -> void:
	if passes.is_empty():
		_list.text = "No passes yet."
		return
	var lines := []
	for i in passes.size():
		var p: ForzaTelemetry.Pass = passes[i]
		lines.append("%d. %s - %d m, kerb contact at %d points" % [i + 1, {"line": "driving line", "left": "left edge", "right": "right edge"}[p.role], int(p.length()), p.kerb_count()])
	_list.text = "\n".join(lines)


func _process(dt: float) -> void:
	if not listening:
		return
	while udp.get_available_packet_count() > 0:
		var pkt := udp.get_packet()
		_pkt_count += 1
		var s := tele.parse(pkt)
		if s.is_empty():
			continue
		_last = s
		if recording and current != null and bool(s.race_on) and float(s.speed) > 1.0:
			current.add(s)
	_rate_t += dt
	if _rate_t >= 0.5:
		_pkt_rate = _pkt_count / _rate_t
		_pkt_count = 0
		_rate_t = 0.0
		if _last.is_empty():
			_live.text = "Listening on port %d - %d packets/s%s" % [int(_port.value), int(_pkt_rate), "" if _pkt_rate > 0 else " (nothing yet: check Data Out is ON and the IP/port match)"] + (" (drive a little so the format can be detected)" if _pkt_rate > 0 else "")
		else:
			var p: Vector3 = _last.pos
			_live.text = "%s · %d packets/s · %d km/h · position %.1f, %.1f, %.1f · %s%s" % [tele.format_name, int(_pkt_rate), int(float(_last.speed) * 3.6), p.x, p.y, p.z,
				_surface_text(_last), ("   ● RECORDING %d m" % int(current.length())) if recording and current else ""]


## Live per-wheel surface readout so you can see kerbs/grass being picked up.
func _surface_text(s: Dictionary) -> String:
	var k := int(s.get("kerb", 0))
	var rum: PackedFloat32Array = s.get("rumble", PackedFloat32Array([0, 0, 0, 0]))
	var parts := []
	if k & (ForzaTelemetry.FL | ForzaTelemetry.RL):
		parts.append("KERB left")
	if k & (ForzaTelemetry.FR | ForzaTelemetry.RR):
		parts.append("KERB right")
	var l := (rum[0] + rum[2]) * 0.5
	var r := (rum[1] + rum[3]) * 0.5
	if parts.is_empty():
		parts.append("surface L %.2f / R %.2f" % [l, r])
	return ", ".join(parts)


func _make() -> void:
	var r := ForzaTelemetry.build_track(passes, _name.text, _width.value)
	if not r.ok:
		_status.text = "[color=#ff6666]%s[/color]" % r.error
		return
	result_spec = r.spec
	var info: Dictionary = r.info
	var v := TrackSpec.validate(result_spec)
	var txt := "[color=#66ff88]Track built: %d m, %s, avg width %.1f m%s, elevation change %.1f m.[/color]\n" % [int(info.length_m), "loop" if info.closed else "point-to-point", float(info.width_m), " (from edge passes)" if info.edges else " (set by hand)", float(info.elevation_m)]
	if info.edges:
		txt += "Edges found along %d%% of the road, max camber %.1f%%%s.\n" % [int(float(info.edge_cover) * 100.0), float(info.max_bank_pct),
			(", ignored %d off-road edge points" % int(info.offroad_dropped)) if int(info.offroad_dropped) > 0 else ""]
	elif bool(info.one_edge_only):
		txt += "[color=#ffcc66]Only one edge pass - record the other edge too for exact width and position.[/color]\n"
	txt += ("Kerbs: %d sections from rumble-strip contact.\n" % int(info.kerbs)) if int(info.kerbs) > 0 else "No kerb contact recorded (no kerbs added).\n"
	for e: String in v.errors:
		txt += "[color=#ff6666]✖ %s[/color]\n" % e
	for w: String in v.warnings:
		txt += "[color=#ffcc66]⚠ %s[/color]\n" % w
	_status.text = txt
	var prev := TrackRaster.render(result_spec, 420, Color(0.1, 0.1, 0.12), Color(0.95, 0.95, 0.95), true)
	_preview.texture = ImageTexture.create_from_image(prev.image)


func _recording_dict() -> Dictionary:
	return {"format": tele.format_name, "passes": passes.map(func(p: ForzaTelemetry.Pass) -> Dictionary: return p.to_dict())}


func _save_recording() -> void:
	if passes.is_empty():
		_status.text = "Nothing recorded yet."
		return
	var path := "user://recordings/%s.json" % JsonUtil.slugify(_name.text)
	JsonUtil.save_file(path, _recording_dict())
	_status.text = "Saved %s" % ProjectSettings.globalize_path(path)
	OS.shell_open(ProjectSettings.globalize_path("user://recordings"))


func _load_recording() -> void:
	UiUtil.file_dialog(self, PackedStringArray(["*.json ; Forza recordings"]), func(path: String) -> void:
		var d: Variant = JsonUtil.load_file(path)
		if not (d is Dictionary):
			return
		passes = (d.get("passes", []) as Array).map(func(x: Variant) -> ForzaTelemetry.Pass: return ForzaTelemetry.Pass.from_dict(x))
		_name.text = path.get_file().get_basename().capitalize()
		_update_list())


func _save_track() -> String:
	if result_spec.is_empty():
		_make()
	if result_spec.is_empty():
		return ""
	var path := "user://tracks/%s.json" % JsonUtil.slugify(str(result_spec.name))
	JsonUtil.save_file(path, result_spec)
	JsonUtil.save_file("user://recordings/%s.json" % JsonUtil.slugify(str(result_spec.name)), _recording_dict())
	var app: Node = get_node("/root/App")
	app.selected_track_path = path
	app.external_track_model = ""
	app.editor_track = result_spec
	return path


func _open_editor() -> void:
	if _save_track() != "":
		get_node("/root/App").editor_background_path = ""
		navigate.emit("editor")


func _drive() -> void:
	if _save_track() != "":
		navigate.emit("drive")


func _export_ac() -> void:
	if _save_track() == "":
		return
	UiUtil.dir_dialog(self, func(dir: String) -> void:
		var r := AcTrackExporter.export_track(result_spec, dir)
		_status.text = ("[color=#66ff88]%s[/color]" if r.ok else "[color=#ff6666]%s[/color]") % r.message)


func _exit_tree() -> void:
	udp.close()
