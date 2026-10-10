class_name PhotoImportScreen
extends Control
## Picture -> track. Offline tracer for top-down shots / sketches, or the
## Claude track agent for anything (including ground-level photos).

signal navigate(target: String)

const MODES := ["auto", "road", "lines", "color"]

var image: Image
var image_path := ""
var result_spec: Dictionary = {}
var _preview: TextureRect
var _result_view: TextureRect
var _mode := 0
var _pick_color := Color(0.3, 0.3, 0.3)
var _width: SpinBox
var _length: SpinBox
var _pad: CheckBox
var _name: LineEdit
var _notes: TextEdit
var _log: RichTextLabel
var _busy := false
var _result_actions: HBoxContainer


func _ready() -> void:
	var vb := UiUtil.screen(self, 1300)
	var head := HBoxContainer.new()
	head.add_child(UiUtil.button("< Menu", func() -> void: navigate.emit("menu")))
	var t := UiUtil.title("  Photo -> Track")
	head.add_child(t)
	vb.add_child(head)
	vb.add_child(UiUtil.label("Top-down images (satellite/map screenshots, drone shots) and sketches can be traced offline. Any picture - including a photo taken standing in the place - can be turned into a track by Claude.", 15, Color(0.75, 0.75, 0.8)))
	vb.add_child(UiUtil.button("Choose image...", _choose))

	var cols := HBoxContainer.new()
	cols.add_theme_constant_override("separation", 20)
	vb.add_child(cols)
	_preview = TextureRect.new()
	_preview.custom_minimum_size = Vector2(620, 460)
	_preview.expand_mode = TextureRect.EXPAND_IGNORE_SIZE
	_preview.stretch_mode = TextureRect.STRETCH_KEEP_ASPECT_CENTERED
	_preview.gui_input.connect(_on_preview_input)
	cols.add_child(_preview)
	_result_view = TextureRect.new()
	_result_view.custom_minimum_size = Vector2(620, 460)
	_result_view.expand_mode = TextureRect.EXPAND_IGNORE_SIZE
	_result_view.stretch_mode = TextureRect.STRETCH_KEEP_ASPECT_CENTERED
	cols.add_child(_result_view)

	_name = LineEdit.new()
	_name.text = "My Spot"
	vb.add_child(UiUtil.row("Track name", _name))
	vb.add_child(UiUtil.row("Trace mode", UiUtil.option(["Auto", "Asphalt (aerial)", "Sketch lines", "Picked colour (click the road)"], 0, func(i: int) -> void: _mode = i)))
	_width = SpinBox.new()
	_width.min_value = 5
	_width.max_value = 40
	_width.value = 12
	_width.suffix = "m"
	vb.add_child(UiUtil.row("Real road width", _width))
	_length = SpinBox.new()
	_length.min_value = 200
	_length.max_value = 10000
	_length.step = 50
	_length.value = 1100
	_length.suffix = "m"
	vb.add_child(UiUtil.row("Sketch track length", _length))
	_pad = CheckBox.new()
	_pad.text = "It's an open lot / skidpad (make a pad, not a road)"
	vb.add_child(_pad)
	var btns := HBoxContainer.new()
	btns.add_child(UiUtil.button("Trace offline", _trace))
	vb.add_child(btns)
	vb.add_child(HSeparator.new())
	_notes = TextEdit.new()
	_notes.custom_minimum_size = Vector2(0, 70)
	_notes.placeholder_text = "Notes for Claude (optional): e.g. 'this is the industrial park behind my work, make a tight technical layout with a wall on the outside of the long sweeper, night time'"
	_notes.wrap_mode = TextEdit.LINE_WRAPPING_BOUNDARY
	vb.add_child(_notes)
	vb.add_child(UiUtil.button("Generate with Claude (agentic)", _agent))
	_log = RichTextLabel.new()
	_log.bbcode_enabled = true
	_log.custom_minimum_size = Vector2(0, 200)
	_log.scroll_following = true
	vb.add_child(_log)
	_result_actions = HBoxContainer.new()
	_result_actions.visible = false
	_result_actions.add_child(UiUtil.button("Open in Track Editor", _open_editor))
	_result_actions.add_child(UiUtil.button("Save + Drive", _save_drive))
	vb.add_child(_result_actions)


func _choose() -> void:
	UiUtil.file_dialog(self, PackedStringArray(["*.png, *.jpg, *.jpeg, *.webp, *.bmp ; Images"]), func(path: String) -> void:
		var img := Image.load_from_file(path)
		if img == null or img.is_empty():
			_log.append_text("[color=#ff6666]Couldn't load that image.[/color]\n")
			return
		image = img
		image_path = path
		_name.text = path.get_file().get_basename().capitalize()
		_preview.texture = ImageTexture.create_from_image(img))


func _on_preview_input(ev: InputEvent) -> void:
	if image == null or not (ev is InputEventMouseButton) or not ev.pressed:
		return
	# Map click to image pixel (keep-aspect-centred).
	var r := _preview.size
	var s := minf(r.x / image.get_width(), r.y / image.get_height())
	var off := (r - Vector2(image.get_width(), image.get_height()) * s) * 0.5
	var p: Vector2 = (ev.position - off) / s
	if p.x >= 0 and p.y >= 0 and p.x < image.get_width() and p.y < image.get_height():
		_pick_color = image.get_pixel(int(p.x), int(p.y))
		_log.append_text("Picked road colour %s - choose 'Picked colour' mode and trace.\n" % _pick_color.to_html(false))


func _options() -> Dictionary:
	return {"mode": MODES[_mode], "color": _pick_color, "road_width_m": _width.value,
		"target_length_m": _length.value, "area_as_pad": _pad.button_pressed, "name": _name.text}


func _trace() -> void:
	if image == null:
		_log.append_text("Choose an image first.\n")
		return
	var r := ImageTracer.trace(image, _options())
	if r.has("debug") and r.debug is Image:
		_result_view.texture = ImageTexture.create_from_image(r.debug)
	if not r.ok:
		_log.append_text("[color=#ff6666]%s[/color]\n" % r.error)
		return
	_set_result(r.spec)
	var v := TrackSpec.validate(r.spec)
	_log.append_text("Traced (%s mode): %d m, %s. %s\n" % [r.info.mode, int(r.info.get("length_m", 0)), "loop" if r.spec.closed else "point-to-point",
		("Issues: " + "; ".join(v.errors + v.warnings)) if (v.errors.size() + v.warnings.size()) > 0 else "Looks good."])


func _agent() -> void:
	if _busy:
		return
	if image == null:
		_log.append_text("Choose an image first.\n")
		return
	if str(get_node("/root/Settings").api_key) == "":
		_log.append_text("[color=#ff6666]Add your Anthropic API key in Settings first.[/color]\n")
		return
	_busy = true
	var agent := TrackAgent.new()
	add_child(agent)
	agent.trace_options = _options()
	agent.progress.connect(func(tx: String) -> void: _log.append_text(tx + "\n"))
	_log.append_text("[b]Claude is looking at your picture...[/b]\n")
	var r: Dictionary = await agent.run(image, _notes.text)
	agent.queue_free()
	_busy = false
	if str(r.error) != "":
		_log.append_text("[color=#ff6666]%s[/color]\n" % r.error)
	if not (r.spec as Dictionary).is_empty():
		var spec: Dictionary = r.spec
		if str(spec.get("name", "")) == "":
			spec["name"] = _name.text
		_set_result(spec)


func _set_result(spec: Dictionary) -> void:
	result_spec = spec
	result_spec["source_image"] = image_path
	var prev := TrackRaster.render(spec, 512, Color(0.1, 0.1, 0.12), Color(0.95, 0.95, 0.95), true)
	_result_view.texture = ImageTexture.create_from_image(prev.image)
	_result_actions.visible = true


func _open_editor() -> void:
	var app: Node = get_node("/root/App")
	app.editor_track = result_spec
	app.editor_background_path = image_path
	navigate.emit("editor")


func _save_drive() -> void:
	var app: Node = get_node("/root/App")
	var path := "user://tracks/%s.json" % JsonUtil.slugify(str(result_spec.get("name", "track")))
	JsonUtil.save_file(path, result_spec)
	app.selected_track_path = path
	app.external_track_model = ""
	navigate.emit("drive")
