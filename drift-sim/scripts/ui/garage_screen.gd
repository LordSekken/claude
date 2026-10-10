class_name GarageScreen
extends Control
## Modular car builder: chassis + engine + transmission + drivetrain + tune,
## with live stats, an AI "Garage agent" and Assetto Corsa car import.

signal navigate(target: String)

const TUNE_FIELDS := [
	# [path, label, min, max, step]
	["boost_bar", "Boost (bar)", 0.0, 2.0, 0.05],
	["power_mult", "ECU / bolt-ons (x power)", 0.7, 1.6, 0.01],
	["rev_limit_rpm", "Rev limit (rpm)", 4000, 10000, 50],
	["final_drive", "Final drive", 2.5, 6.0, 0.01],
	["diff.power", "LSD power lock", 0.0, 1.0, 0.05],
	["diff.coast", "LSD coast lock", 0.0, 1.0, 0.05],
	["diff.preload_nm", "LSD preload (Nm)", 0, 400, 5],
	["awd_front_split", "AWD front torque split", 0.1, 0.9, 0.05],
	["springs_front_n_mm", "Front springs (N/mm)", 10, 200, 1],
	["springs_rear_n_mm", "Rear springs (N/mm)", 10, 200, 1],
	["arb_front_n_mm", "Front anti-roll bar (N/mm)", 0, 80, 1],
	["arb_rear_n_mm", "Rear anti-roll bar (N/mm)", 0, 80, 1],
	["damp_bump_front", "Front bump damping (Ns/m)", 500, 9000, 50],
	["damp_rebound_front", "Front rebound damping (Ns/m)", 500, 12000, 50],
	["damp_bump_rear", "Rear bump damping (Ns/m)", 500, 9000, 50],
	["damp_rebound_rear", "Rear rebound damping (Ns/m)", 500, 12000, 50],
	["ride_height_mm", "Ride height change (mm)", -150, 80, 5],
	["steer_lock_deg", "Steering lock (deg)", 25, 70, 1],
	["ackermann", "Ackermann (0 parallel - 1 full)", 0.0, 1.0, 0.05],
	["brake_bias_front", "Brake bias front", 0.4, 0.85, 0.01],
	["weight_reduction_kg", "Weight reduction (kg)", 0, 300, 5],
	["tire_front.grip_mult", "Front tyre grip x", 0.7, 1.3, 0.01],
	["tire_rear.grip_mult", "Rear tyre grip x", 0.7, 1.3, 0.01],
]
const DIFFS := ["open", "lsd", "welded"]
const LAYOUTS := ["stock", "RWD", "AWD", "FWD"]
const COMPOUNDS := ["economy", "street", "sport", "drift", "semi_slick"]

var build: Dictionary = {}
var app: Node
var _stats: RichTextLabel
var _tune_box: VBoxContainer
var _parts_box: VBoxContainer
var _name_edit: LineEdit
var _ai_prompt: TextEdit
var _ai_log: RichTextLabel
var _ai_busy := false
var _agent: GarageAgent
var _status: Label


func _ready() -> void:
	app = get_node("/root/App")
	app.parts.reload()
	var b: Variant = JsonUtil.load_file(app.selected_car_path)
	build = b if b is Dictionary else {"chassis": "subaru_legacy_outback_1998", "tune": {}}
	var root := UiUtil.screen(self, 1300)
	var head := HBoxContainer.new()
	head.add_child(UiUtil.button("< Menu", func() -> void: navigate.emit("menu")))
	var t := UiUtil.title("  Garage")
	t.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	head.add_child(t)
	head.add_child(UiUtil.button("Import AC car (unpacked data folder)...", _import_ac))
	head.add_child(UiUtil.button("Save build", _save))
	head.add_child(UiUtil.button("Save + Drive", func() -> void:
		if _save():
			navigate.emit("drive")))
	root.add_child(head)
	_status = UiUtil.label("", 15, Color(0.6, 1, 0.6), true)
	root.add_child(_status)

	var cols := HBoxContainer.new()
	cols.add_theme_constant_override("separation", 24)
	root.add_child(cols)
	var left := VBoxContainer.new()
	left.custom_minimum_size = Vector2(640, 0)
	cols.add_child(left)
	var right := VBoxContainer.new()
	right.custom_minimum_size = Vector2(620, 0)
	cols.add_child(right)

	_parts_box = VBoxContainer.new()
	left.add_child(_parts_box)
	left.add_child(HSeparator.new())
	left.add_child(UiUtil.label("Tune", 22, Color(1, 0.8, 0.25)))
	_tune_box = VBoxContainer.new()
	left.add_child(_tune_box)

	right.add_child(UiUtil.label("Ask the Garage agent (Claude)", 22, Color(1, 0.8, 0.25)))
	right.add_child(UiUtil.label("Describe a car in plain words - real models, swaps, conversions, tunes. It researches specs, creates missing parts, assembles and checks the build, then saves it. Tip: paste your VIN for an exact match.", 14, Color(0.75, 0.75, 0.8)))
	_ai_prompt = TextEdit.new()
	_ai_prompt.custom_minimum_size = Vector2(0, 90)
	_ai_prompt.wrap_mode = TextEdit.LINE_WRAPPING_BOUNDARY
	_ai_prompt.placeholder_text = "e.g. My '98 Legacy Outback with a WRX engine, converted to RWD, with a realistic street/drift tune"
	right.add_child(_ai_prompt)
	var ai_row := HBoxContainer.new()
	ai_row.add_child(UiUtil.button("Build it with Claude", _run_agent))
	ai_row.add_child(UiUtil.button("Tweak current build with Claude", func() -> void: _run_agent(true)))
	right.add_child(ai_row)
	_ai_log = RichTextLabel.new()
	_ai_log.custom_minimum_size = Vector2(0, 220)
	_ai_log.scroll_following = true
	_ai_log.bbcode_enabled = true
	right.add_child(_ai_log)
	right.add_child(HSeparator.new())
	right.add_child(UiUtil.label("Build stats", 22, Color(1, 0.8, 0.25)))
	_stats = RichTextLabel.new()
	_stats.custom_minimum_size = Vector2(0, 420)
	_stats.bbcode_enabled = true
	_stats.fit_content = true
	right.add_child(_stats)
	_rebuild_ui()


func _tune() -> Dictionary:
	if not (build.get("tune") is Dictionary):
		build["tune"] = {}
	return build.tune


static func _get_path(d: Dictionary, path: String) -> Variant:
	var cur: Variant = d
	for k in path.split("."):
		if not (cur is Dictionary) or not (cur as Dictionary).has(k):
			return null
		cur = cur[k]
	return cur


static func _set_path(d: Dictionary, path: String, v: Variant) -> void:
	var keys := path.split(".")
	var cur := d
	for i in keys.size() - 1:
		if not (cur.get(keys[i]) is Dictionary):
			cur[keys[i]] = {}
		cur = cur[keys[i]]
	cur[keys[keys.size() - 1]] = v


func _rebuild_ui() -> void:
	for c in _parts_box.get_children():
		c.queue_free()
	for c in _tune_box.get_children():
		c.queue_free()
	var lib: PartsLibrary = app.parts
	_name_edit = LineEdit.new()
	_name_edit.text = str(build.get("name", "My build"))
	_name_edit.text_changed.connect(func(tx: String) -> void: build["name"] = tx)
	_parts_box.add_child(UiUtil.row("Build name", _name_edit))
	for kind_key in [["chassis", "chassis", "Chassis"], ["engines", "engine", "Engine"], ["transmissions", "transmission", "Transmission"]]:
		var kind: String = kind_key[0]
		var key: String = kind_key[1]
		var ids: Array = lib.ids(kind)
		var labels := []
		if key != "chassis":
			labels.append("(stock)")
		for id: String in ids:
			labels.append(str(lib.get_part(kind, id).get("name", id)))
		var cur: Variant = build.get(key)
		var sel := 0
		if cur != null and ids.has(cur):
			sel = ids.find(cur) + (1 if key != "chassis" else 0)
		var opt := UiUtil.option(labels, sel, func(i: int) -> void:
			if key != "chassis" and i == 0:
				build[key] = null
			else:
				build[key] = ids[i - (1 if key != "chassis" else 0)]
			_refresh_stats())
		_parts_box.add_child(UiUtil.row(kind_key[2], opt))
	var lay: Variant = build.get("drivetrain")
	var lsel := 0 if lay == null else maxi(LAYOUTS.find(str(lay)), 0)
	_parts_box.add_child(UiUtil.row("Drivetrain", UiUtil.option(["Stock layout", "RWD", "AWD", "FWD"], lsel, func(i: int) -> void:
		build["drivetrain"] = null if i == 0 else LAYOUTS[i]
		_refresh_stats())))
	var t := _tune()
	var diff_t := str(_get_path(t, "diff.type")) if _get_path(t, "diff.type") != null else "lsd"
	_tune_box.add_child(UiUtil.row("Differential", UiUtil.option(["Open", "Clutch LSD", "Welded / spool"], DIFFS.find(diff_t), func(i: int) -> void:
		_set_path(_tune(), "diff.type", DIFFS[i])
		_refresh_stats())))
	for end in ["front", "rear"]:
		var path := "tire_%s.compound" % end
		var comp: Variant = _get_path(t, path)
		_tune_box.add_child(UiUtil.row("Tyres %s compound" % end, UiUtil.option(COMPOUNDS, maxi(COMPOUNDS.find(str(comp if comp != null else "sport")), 0), func(i: int) -> void:
			_set_path(_tune(), path, COMPOUNDS[i])
			_refresh_stats())))
		var size_edit := LineEdit.new()
		var sz: Variant = _get_path(t, "tire_%s.size" % end)
		size_edit.text = str(sz) if sz != null else ""
		size_edit.placeholder_text = "e.g. 235/40R17 (blank = stock)"
		size_edit.text_submitted.connect(func(tx: String) -> void:
			_set_path(_tune(), "tire_%s.size" % end, tx if tx != "" else null)
			_refresh_stats())
		size_edit.focus_exited.connect(func() -> void:
			_set_path(_tune(), "tire_%s.size" % end, size_edit.text if size_edit.text != "" else null)
			_refresh_stats())
		_tune_box.add_child(UiUtil.row("Tyres %s size" % end, size_edit))
	var hb := CheckButton.new()
	hb.text = "Hydraulic handbrake"
	hb.button_pressed = bool(t.get("hydro_handbrake", false))
	hb.toggled.connect(func(on: bool) -> void:
		_tune()["hydro_handbrake"] = on
		_refresh_stats())
	_tune_box.add_child(hb)
	var color := ColorPickerButton.new()
	color.color = Color.from_string(str(build.get("color", "#c8c8c8")), Color.GRAY)
	color.custom_minimum_size = Vector2(80, 32)
	color.color_changed.connect(func(c: Color) -> void: build["color"] = "#" + c.to_html(false))
	_tune_box.add_child(UiUtil.row("Paint", color))

	var res := CarBuilder.assemble(build, lib)
	for f: Array in TUNE_FIELDS:
		var path: String = f[0]
		var v: Variant = _get_path(t, path)
		var current := _effective(path, v, res)
		var val_label := UiUtil.label(_fmt(current, float(f[4])), 14)
		val_label.custom_minimum_size = Vector2(70, 0)
		var sl := UiUtil.slider(float(f[2]), float(f[3]), float(f[4]), current, func(x: float) -> void:
			_set_path(_tune(), path, x)
			val_label.text = _fmt(x, float(f[4]))
			_refresh_stats())
		var h := UiUtil.row(f[1], sl, 260)
		h.add_child(val_label)
		var reset := Button.new()
		reset.text = "stock"
		reset.pressed.connect(func() -> void:
			_set_path(_tune(), path, null)
			_rebuild_ui())
		h.add_child(reset)
		_tune_box.add_child(h)
	_refresh_stats()


func _fmt(v: float, step: float) -> String:
	return str(int(v)) if step >= 1.0 else ("%.2f" % v)


## Value the slider should show: the tune value if set, else the stock value.
func _effective(path: String, v: Variant, res: Dictionary) -> float:
	if v != null:
		return float(v)
	if not res.get("ok", false):
		return 0.0
	var s: Dictionary = res.spec
	match path:
		"boost_bar": return float(s.boost_target)
		"power_mult": return 1.0
		"rev_limit_rpm": return float(s.limiter_rpm)
		"final_drive": return float(s.final)
		"diff.power": return float(s.diff_power)
		"diff.coast": return float(s.diff_coast)
		"diff.preload_nm": return float(s.diff_preload)
		"awd_front_split": return float(s.awd_front_split)
		"springs_front_n_mm": return float(s.spring_f) / 1000.0
		"springs_rear_n_mm": return float(s.spring_r) / 1000.0
		"arb_front_n_mm": return float(s.arb_f) / 1000.0
		"arb_rear_n_mm": return float(s.arb_r) / 1000.0
		"damp_bump_front": return float(s.bump_f)
		"damp_rebound_front": return float(s.rebound_f)
		"damp_bump_rear": return float(s.bump_r)
		"damp_rebound_rear": return float(s.rebound_r)
		"ride_height_mm": return float(s.ride_offset) * 1000.0
		"steer_lock_deg": return rad_to_deg(float(s.steer_lock))
		"ackermann": return float(s.ackermann)
		"brake_bias_front": return float(s.brake_bias)
		"weight_reduction_kg": return 0.0
		"tire_front.grip_mult", "tire_rear.grip_mult": return 1.0
	return 0.0


func _refresh_stats() -> void:
	var res := CarBuilder.assemble(build, app.parts)
	if not res.ok:
		_stats.text = "[color=#ff6666]%s[/color]" % "\n".join(res.errors)
		return
	var s: Dictionary = res.stats
	var sp: Dictionary = res.spec
	var txt := "[b]%s[/b]\n%s\n%s\n%s · %s\n\n" % [sp.name, sp.chassis_name, sp.engine_name, sp.transmission_name, s.drivetrain]
	txt += "[b]%d hp[/b] @ %d rpm   [b]%d Nm[/b] @ %d rpm\n" % [s.peak_hp, s.peak_hp_rpm, s.peak_torque_nm, s.peak_torque_rpm]
	txt += "%d kg · %.1f%% front · [b]%d hp/tonne[/b]\n" % [s.mass_kg, s.weight_front_pct, s.hp_per_tonne]
	txt += "0-100 km/h ~%.1f s · top speed ~%d km/h\n" % [s.zero_to_100_s, s.top_speed_kmh]
	txt += "Gear top speeds (km/h): %s\n" % ", ".join((s.gear_top_speeds_kmh as Array).map(func(x: Variant) -> String: return str(int(x))))
	txt += "Steering lock %d° · diff %s\n" % [s.steer_lock_deg, sp.diff_type]
	if (res.warnings as Array).size() > 0:
		txt += "\n[color=#ffcc66]" + "\n".join((res.warnings as Array).map(func(w: Variant) -> String: return "• " + str(w))) + "[/color]"
	if build.has("notes"):
		txt += "\n\n[color=#aaaaaa]%s[/color]" % build.notes
	_stats.text = txt


func _save() -> bool:
	var res := CarBuilder.assemble(build, app.parts)
	if not res.ok:
		_status.text = "Can't save: " + ", ".join(res.errors)
		return false
	var id := JsonUtil.slugify(str(build.get("name", "my_build")))
	build["id"] = id
	var path := "user://cars/%s.json" % id
	JsonUtil.save_file(path, build)
	app.selected_car_path = path
	_status.text = "Saved to %s" % ProjectSettings.globalize_path(path)
	return true


func _log(text: String) -> void:
	_ai_log.append_text(text + "\n")


func _run_agent(tweak := false) -> void:
	if _ai_busy:
		return
	var prompt := _ai_prompt.text.strip_edges()
	if prompt == "":
		_log("[color=#ffcc66]Type what you want first.[/color]")
		return
	var settings: Node = get_node("/root/Settings")
	if str(settings.api_key) == "":
		_log("[color=#ff6666]Add your Anthropic API key in Settings first (or set ANTHROPIC_API_KEY).[/color]")
		return
	_ai_busy = true
	_ai_log.clear()
	_log("[b]You:[/b] " + prompt)
	_agent = GarageAgent.new()
	add_child(_agent)
	_agent.progress.connect(_log)
	var result: Dictionary = await _agent.run(prompt, build if tweak else {})
	_ai_busy = false
	_agent.queue_free()
	if result.get("build") is Dictionary:
		build = result.build
		app.selected_car_path = str(result.get("path", app.selected_car_path))
		_rebuild_ui()
		_log("[color=#66ff88]Build loaded into the garage.[/color]")
	if str(result.get("error", "")) != "":
		_log("[color=#ff6666]%s[/color]" % result.error)


func _import_ac() -> void:
	UiUtil.dir_dialog(self, func(dir: String) -> void:
		var r := AcCarImporter.import_folder(dir)
		if not r.ok:
			_status.text = "AC import failed: " + str(r.error)
			return
		app.parts.reload()
		build = r.build
		_rebuild_ui()
		_status.text = "Imported %s from Assetto Corsa data (chassis, engine and gearbox saved as parts)." % build.name)
