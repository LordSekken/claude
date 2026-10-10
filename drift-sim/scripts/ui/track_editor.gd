class_name TrackEditor
extends Control
## Top-down track editor.
##  Left click: add point (on empty space) / select + drag point
##  Shift + left click on the road: insert a point there
##  Right click a point: delete      Middle/right drag: pan      Wheel: zoom

signal navigate(target: String)

var spec: Dictionary = {}
var selected := -1
var cam := Vector2.ZERO
var zoom := 1.0
var bg_tex: Texture2D
var bg_mpp := 0.5   # metres per pixel of the background image
var bg_alpha := 0.6
var _dragging := false
var _panning := false
var _canvas: Control
var _panel: VBoxContainer
var _status: RichTextLabel
var _point_box: VBoxContainer
var _zone_start := -1
var _samples: Dictionary = {}


func _ready() -> void:
	var app: Node = get_node("/root/App")
	spec = TrackSpec.normalized(app.editor_track if not app.editor_track.is_empty() else _load_selected())
	if app.editor_background_path != "":
		var img := Image.load_from_file(app.editor_background_path)
		if img and not img.is_empty():
			bg_tex = ImageTexture.create_from_image(img)
			var src: Dictionary = spec.get("source", {})
			bg_mpp = float(src.get("meters_per_pixel", 0.5))
	var bg := ColorRect.new()
	bg.color = Color(0.07, 0.08, 0.1)
	bg.set_anchors_preset(Control.PRESET_FULL_RECT)
	add_child(bg)
	var h := HBoxContainer.new()
	h.set_anchors_preset(Control.PRESET_FULL_RECT)
	add_child(h)
	_canvas = Control.new()
	_canvas.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_canvas.clip_contents = true
	_canvas.draw.connect(_draw_canvas)
	_canvas.gui_input.connect(_canvas_input)
	h.add_child(_canvas)
	var scroll := ScrollContainer.new()
	scroll.custom_minimum_size = Vector2(380, 0)
	scroll.horizontal_scroll_mode = ScrollContainer.SCROLL_MODE_DISABLED
	h.add_child(scroll)
	_panel = VBoxContainer.new()
	_panel.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_panel.add_theme_constant_override("separation", 6)
	scroll.add_child(_panel)
	_build_panel()
	_fit_view.call_deferred()


func _load_selected() -> Dictionary:
	var app: Node = get_node("/root/App")
	var d: Variant = JsonUtil.load_file(app.selected_track_path)
	return d if d is Dictionary else {"name": "New Track", "points": []}


func _points() -> Array:
	if not (spec.get("points") is Array):
		spec["points"] = []
	return spec.points


func _build_panel() -> void:
	for c in _panel.get_children():
		c.queue_free()
	var top := HBoxContainer.new()
	top.add_child(UiUtil.button("< Menu", func() -> void: navigate.emit("menu"), 36))
	top.add_child(UiUtil.button("New", func() -> void:
		spec = TrackSpec.normalized({"name": "New Track", "points": []})
		bg_tex = null
		_changed(true), 36))
	top.add_child(UiUtil.button("Open...", _open, 36))
	_panel.add_child(top)
	var name_edit := LineEdit.new()
	name_edit.text = str(spec.get("name", ""))
	name_edit.text_changed.connect(func(tx: String) -> void: spec["name"] = tx)
	_panel.add_child(UiUtil.row("Name", name_edit, 90))
	var closed := CheckBox.new()
	closed.text = "Closed loop"
	closed.button_pressed = bool(spec.closed)
	closed.toggled.connect(func(on: bool) -> void:
		spec["closed"] = on
		_changed())
	_panel.add_child(closed)
	_panel.add_child(_num_row("Road width (m)", "width", 5, 40, 0.5))
	_panel.add_child(_num_row("Run-off (m)", "runoff", 0, 20, 0.5))
	var surfs := ["grass", "asphalt", "gravel", "dirt", "sand"]
	_panel.add_child(UiUtil.row("Run-off", UiUtil.option(surfs, maxi(surfs.find(str(spec.runoff_surface)), 0), func(i: int) -> void:
		spec["runoff_surface"] = surfs[i]
		_changed()), 90))
	for side in ["left", "right"]:
		var cb := CheckBox.new()
		cb.text = "Wall on the %s" % side
		cb.button_pressed = bool(spec.walls.get(side, true))
		cb.toggled.connect(func(on: bool) -> void:
			spec.walls[side] = on
			_changed())
		_panel.add_child(cb)
	var kerbs := CheckBox.new()
	kerbs.text = "Kerbs in corners"
	kerbs.button_pressed = bool(spec.kerbs)
	kerbs.toggled.connect(func(on: bool) -> void: spec["kerbs"] = on)
	_panel.add_child(kerbs)
	var tods := ["day", "dusk", "night"]
	_panel.add_child(UiUtil.row("Time", UiUtil.option(tods, maxi(tods.find(str(spec.scenery.get("time_of_day", "day"))), 0), func(i: int) -> void: spec.scenery["time_of_day"] = tods[i]), 90))

	_panel.add_child(HSeparator.new())
	_panel.add_child(UiUtil.label("Selected point", 18, Color(1, 0.8, 0.25)))
	_point_box = VBoxContainer.new()
	_panel.add_child(_point_box)
	_refresh_point_box()

	_panel.add_child(HSeparator.new())
	_panel.add_child(UiUtil.label("Drift zones & clips", 18, Color(1, 0.8, 0.25)))
	_panel.add_child(UiUtil.button("Zone: start at selected point", func() -> void:
		_zone_start = selected
		_set_status("Zone start set. Select the end point and press 'Zone: end'."), 36))
	_panel.add_child(UiUtil.button("Zone: end at selected point", _add_zone, 36))
	var cl := HBoxContainer.new()
	cl.add_child(UiUtil.button("Clip L", func() -> void: _add_clip("left"), 36))
	cl.add_child(UiUtil.button("Clip R", func() -> void: _add_clip("right"), 36))
	cl.add_child(UiUtil.button("Start here", _set_start, 36))
	cl.add_child(UiUtil.button("Clear zones/clips", func() -> void:
		spec["zones"] = []
		spec["clips"] = []
		_changed(), 36))
	_panel.add_child(cl)

	_panel.add_child(HSeparator.new())
	if bg_tex:
		_panel.add_child(UiUtil.label("Reference image", 18, Color(1, 0.8, 0.25)))
		var scale_box := SpinBox.new()
		scale_box.min_value = 0.01
		scale_box.max_value = 20
		scale_box.step = 0.01
		scale_box.value = bg_mpp
		scale_box.value_changed.connect(func(v: float) -> void:
			bg_mpp = v
			_canvas.queue_redraw())
		_panel.add_child(UiUtil.row("m / pixel", scale_box, 90))
		_panel.add_child(UiUtil.row("Opacity", UiUtil.slider(0, 1, 0.05, bg_alpha, func(v: float) -> void:
			bg_alpha = v
			_canvas.queue_redraw()), 90))
	_panel.add_child(UiUtil.button("Save", _save))
	_panel.add_child(UiUtil.button("Save + Test drive", func() -> void:
		if _save():
			navigate.emit("drive")))
	_panel.add_child(UiUtil.button("Export for Assetto Corsa...", _export_ac))
	_status = RichTextLabel.new()
	_status.bbcode_enabled = true
	_status.fit_content = true
	_status.custom_minimum_size = Vector2(0, 120)
	_panel.add_child(_status)
	_panel.add_child(UiUtil.label("Click: add/select · drag: move · Shift+click road: insert · right-click point: delete · wheel: zoom · right/middle-drag: pan", 13, Color(0.6, 0.6, 0.65)))
	_changed()


func _num_row(label: String, key: String, lo: float, hi: float, step: float) -> HBoxContainer:
	var sb := SpinBox.new()
	sb.min_value = lo
	sb.max_value = hi
	sb.step = step
	sb.value = float(spec.get(key, lo))
	sb.value_changed.connect(func(v: float) -> void:
		spec[key] = v
		_changed())
	return UiUtil.row(label, sb, 140)


func _refresh_point_box() -> void:
	if _point_box == null:
		return
	for c in _point_box.get_children():
		c.queue_free()
	var pts := _points()
	if selected < 0 or selected >= pts.size():
		_point_box.add_child(UiUtil.label("(none - click a point)", 14, Color(0.6, 0.6, 0.6)))
		return
	var p: Array = pts[selected]
	var elev := SpinBox.new()
	elev.min_value = -200
	elev.max_value = 500
	elev.step = 0.5
	elev.value = float(p[2]) if p.size() > 2 else 0.0
	elev.suffix = "m"
	elev.value_changed.connect(func(v: float) -> void:
		var q: Array = _points()[selected]
		if q.size() < 3:
			q.append(v)
		else:
			q[2] = v
		_changed())
	_point_box.add_child(UiUtil.row("Elevation", elev, 140))
	var widths: Array = spec.get("widths", [])
	var wsb := SpinBox.new()
	wsb.min_value = 5
	wsb.max_value = 40
	wsb.step = 0.5
	wsb.value = float(widths[selected]) if selected < widths.size() else float(spec.width)
	wsb.value_changed.connect(func(v: float) -> void:
		var ws: Array = spec.get("widths", [])
		while ws.size() < _points().size():
			ws.append(float(spec.width))
		ws[selected] = v
		spec["widths"] = ws
		_changed())
	_point_box.add_child(UiUtil.row("Width here", wsb, 140))


func _changed(refit := false) -> void:
	_samples = TrackSpec.sample(spec, 2.0) if _points().size() >= 2 else {}
	var v := TrackSpec.validate(spec)
	var txt := ""
	var st: Dictionary = v.stats
	if st.has("length_m"):
		txt += "Length %d m · tightest radius %s m · elevation %s m\n" % [st.length_m, st.get("min_radius_m", "-"), st.get("elevation_change_m", 0)]
	for e: String in v.errors:
		txt += "[color=#ff6666]✖ %s[/color]\n" % e
	for w: String in v.warnings:
		txt += "[color=#ffcc66]⚠ %s[/color]\n" % w
	if _status:
		_status.text = txt
	if refit:
		_fit_view()
	_canvas.queue_redraw()


func _set_status(t: String) -> void:
	_status.text = t


func _fit_view() -> void:
	var pts := _points()
	if pts.is_empty():
		cam = Vector2.ZERO
		zoom = 2.0
		_canvas.queue_redraw()
		return
	var lo := Vector2(INF, INF)
	var hi := Vector2(-INF, -INF)
	for p: Array in pts:
		lo = lo.min(Vector2(float(p[0]), float(p[1])))
		hi = hi.max(Vector2(float(p[0]), float(p[1])))
	cam = (lo + hi) * 0.5
	var ext := (hi - lo) + Vector2(60, 60)
	var sz := _canvas.size if _canvas.size.x > 10 else Vector2(1000, 800)
	zoom = minf(sz.x / ext.x, sz.y / ext.y)
	_canvas.queue_redraw()


func to_screen(w: Vector2) -> Vector2:
	return (w - cam) * zoom + _canvas.size * 0.5


func to_world(s: Vector2) -> Vector2:
	return (s - _canvas.size * 0.5) / zoom + cam


func _draw_canvas() -> void:
	var c := _canvas
	# Background image is centred on the world origin (as the tracer does).
	if bg_tex:
		var sz := Vector2(bg_tex.get_width(), bg_tex.get_height()) * bg_mpp
		var r := Rect2(to_screen(-sz * 0.5), sz * zoom)
		c.draw_texture_rect(bg_tex, r, false, Color(1, 1, 1, bg_alpha))
	# Grid every 10 m (50 m bold).
	var step := 10.0
	var w0 := to_world(Vector2.ZERO)
	var w1 := to_world(c.size)
	if zoom > 0.6:
		var x := floorf(w0.x / step) * step
		while x < w1.x:
			c.draw_line(to_screen(Vector2(x, w0.y)), to_screen(Vector2(x, w1.y)), Color(1, 1, 1, 0.1 if int(x) % 50 == 0 else 0.04))
			x += step
		var y := floorf(w0.y / step) * step
		while y < w1.y:
			c.draw_line(to_screen(Vector2(w0.x, y)), to_screen(Vector2(w1.x, y)), Color(1, 1, 1, 0.1 if int(y) % 50 == 0 else 0.04))
			y += step
	for pad: Dictionary in spec.get("pads", []):
		var poly := PackedVector2Array()
		for q: Array in pad.get("points", []):
			poly.append(to_screen(Vector2(float(q[0]), float(q[1]))))
		if poly.size() >= 3 and not Geometry2D.triangulate_polygon(poly).is_empty():
			c.draw_colored_polygon(poly, Color(0.5, 0.5, 0.55, 0.6))
	if not _samples.is_empty():
		var pos: PackedVector3Array = _samples.pos
		var line := PackedVector2Array()
		for p in pos:
			line.append(to_screen(Vector2(p.x, p.z)))
		if _samples.closed and line.size() > 0:
			line.append(line[0])
		var wpx := maxf(float(spec.width) * zoom, 2.0)
		c.draw_polyline(line, Color(0.32, 0.32, 0.36, 0.85), wpx + float(spec.runoff) * 2.0 * zoom, true)
		c.draw_polyline(line, Color(0.75, 0.75, 0.78, 0.9), wpx, true)
		c.draw_polyline(line, Color(1, 1, 1, 0.5), 1.0, true)
		var n := pos.size()
		for z: Dictionary in spec.get("zones", []):
			var zl := PackedVector2Array()
			var i0 := int(float(z.start) * n)
			var i1 := int(float(z.end) * n)
			if i1 < i0:
				i1 += n
			for i in range(i0, i1 + 1):
				var p := pos[i % n]
				zl.append(to_screen(Vector2(p.x, p.z)))
			if zl.size() > 1:
				c.draw_polyline(zl, Color(1, 0.55, 0.1, 0.8), maxf(wpx * 0.3, 3.0), true)
		for cl: Dictionary in spec.get("clips", []):
			var i := clampi(int(float(cl.t) * n), 0, n - 1)
			var side := -1.0 if str(cl.side) == "left" else 1.0
			var p := pos[i] + (_samples.right[i] as Vector3) * side * (float(_samples.width[i]) * 0.5)
			c.draw_circle(to_screen(Vector2(p.x, p.z)), maxf(float(cl.get("radius", 2.0)) * zoom, 4.0), Color(1, 0.85, 0, 0.9))
		var si := clampi(int(float(spec.start) * n), 0, n - 1)
		var sp := to_screen(Vector2(pos[si].x, pos[si].z))
		c.draw_circle(sp, 7.0, Color(0.2, 1, 0.3))
		var tg: Vector3 = _samples.tangent[si]
		c.draw_line(sp, sp + Vector2(tg.x, tg.z) * 22.0, Color(0.2, 1, 0.3), 3.0)
	var pts := _points()
	for i in pts.size():
		var p := to_screen(Vector2(float(pts[i][0]), float(pts[i][1])))
		var col := Color(1, 0.3, 0.3) if i == selected else (Color(0.3, 0.8, 1) if i != _zone_start else Color(1, 0.55, 0.1))
		c.draw_circle(p, 6.0 if i == selected else 4.5, col)
	if pts.size() > 0:
		c.draw_string(get_theme_default_font(), to_screen(Vector2(float(pts[0][0]), float(pts[0][1]))) + Vector2(8, -8), "1", HORIZONTAL_ALIGNMENT_LEFT, -1, 14)


func _hit_point(sp: Vector2) -> int:
	var pts := _points()
	var best := -1
	var best_d := 12.0
	for i in pts.size():
		var d := to_screen(Vector2(float(pts[i][0]), float(pts[i][1]))).distance_to(sp)
		if d < best_d:
			best_d = d
			best = i
	return best


func _canvas_input(ev: InputEvent) -> void:
	if ev is InputEventMouseButton:
		var mb := ev as InputEventMouseButton
		if mb.button_index == MOUSE_BUTTON_WHEEL_UP and mb.pressed:
			_zoom_at(mb.position, 1.15)
		elif mb.button_index == MOUSE_BUTTON_WHEEL_DOWN and mb.pressed:
			_zoom_at(mb.position, 1.0 / 1.15)
		elif mb.button_index == MOUSE_BUTTON_LEFT:
			if mb.pressed:
				var hit := _hit_point(mb.position)
				if hit >= 0:
					selected = hit
					_dragging = true
				elif mb.shift_pressed:
					_insert_point(to_world(mb.position))
				else:
					_points().append([snappedf(to_world(mb.position).x, 0.1), snappedf(to_world(mb.position).y, 0.1)])
					selected = _points().size() - 1
					_changed()
				_refresh_point_box()
				_canvas.queue_redraw()
			else:
				if _dragging:
					_changed()
				_dragging = false
		elif mb.button_index == MOUSE_BUTTON_RIGHT:
			if mb.pressed:
				var hit := _hit_point(mb.position)
				if hit >= 0:
					_points().remove_at(hit)
					var ws: Array = spec.get("widths", [])
					if hit < ws.size():
						ws.remove_at(hit)
					selected = -1
					_refresh_point_box()
					_changed()
				else:
					_panning = true
			else:
				_panning = false
		elif mb.button_index == MOUSE_BUTTON_MIDDLE:
			_panning = mb.pressed
	elif ev is InputEventMouseMotion:
		var mm := ev as InputEventMouseMotion
		if _dragging and selected >= 0:
			var w := to_world(mm.position)
			var p: Array = _points()[selected]
			p[0] = snappedf(w.x, 0.1)
			p[1] = snappedf(w.y, 0.1)
			_samples = TrackSpec.sample(spec, 3.0) if _points().size() >= 2 else {}
			_canvas.queue_redraw()
		elif _panning:
			cam -= mm.relative / zoom
			_canvas.queue_redraw()


func _zoom_at(sp: Vector2, f: float) -> void:
	var before := to_world(sp)
	zoom = clampf(zoom * f, 0.05, 40.0)
	var after := to_world(sp)
	cam += before - after
	_canvas.queue_redraw()


func _insert_point(w: Vector2) -> void:
	var pts := _points()
	if pts.size() < 2:
		pts.append([w.x, w.y])
		_changed()
		return
	var best := 0
	var best_d := INF
	var segs := pts.size() if bool(spec.closed) else pts.size() - 1
	for i in segs:
		var a := Vector2(float(pts[i][0]), float(pts[i][1]))
		var b := Vector2(float(pts[(i + 1) % pts.size()][0]), float(pts[(i + 1) % pts.size()][1]))
		var d := ImageTracer._seg_dist(w, a, b)
		if d < best_d:
			best_d = d
			best = i
	pts.insert(best + 1, [snappedf(w.x, 0.1), snappedf(w.y, 0.1)])
	var ws: Array = spec.get("widths", [])
	if best + 1 <= ws.size() and ws.size() > 0:
		ws.insert(best + 1, ws[best])
	selected = best + 1
	_changed()


## Track fraction of a control point (via the nearest sample).
func _frac_of_point(i: int) -> float:
	if _samples.is_empty() or i < 0:
		return 0.0
	var p: Array = _points()[i]
	var target := Vector3(float(p[0]), float(p[2]) if p.size() > 2 else 0.0, float(p[1]))
	var pos: PackedVector3Array = _samples.pos
	var best := 0
	var best_d := INF
	for k in pos.size():
		var d := Vector2(pos[k].x - target.x, pos[k].z - target.z).length_squared()
		if d < best_d:
			best_d = d
			best = k
	return snappedf(float(best) / pos.size(), 0.001)


func _add_zone() -> void:
	if _zone_start < 0 or selected < 0:
		_set_status("Select a start point, press 'Zone: start', then select the end point.")
		return
	var zones: Array = spec.get("zones", [])
	zones.append({"name": "Zone %d" % (zones.size() + 1), "start": _frac_of_point(_zone_start), "end": _frac_of_point(selected)})
	spec["zones"] = zones
	_zone_start = -1
	_changed()


func _add_clip(side: String) -> void:
	if selected < 0:
		_set_status("Select a point first.")
		return
	var clips: Array = spec.get("clips", [])
	clips.append({"t": _frac_of_point(selected), "side": side, "radius": 2.0})
	spec["clips"] = clips
	_changed()


func _set_start() -> void:
	if selected >= 0:
		spec["start"] = _frac_of_point(selected)
		_changed()


func _save() -> bool:
	var v := TrackSpec.validate(spec)
	if (v.errors as Array).size() > 0:
		_set_status("[color=#ff6666]Fix errors before saving:\n%s[/color]" % "\n".join(v.errors))
		return false
	var app: Node = get_node("/root/App")
	var path := "user://tracks/%s.json" % JsonUtil.slugify(str(spec.get("name", "track")))
	JsonUtil.save_file(path, spec)
	app.selected_track_path = path
	app.external_track_model = ""
	app.editor_track = spec
	_set_status("[color=#66ff88]Saved %s[/color]" % ProjectSettings.globalize_path(path))
	return true


func _open() -> void:
	UiUtil.file_dialog(self, PackedStringArray(["*.json ; Track files"]), func(path: String) -> void:
		var d: Variant = JsonUtil.load_file(path)
		if d is Dictionary:
			spec = TrackSpec.normalized(d)
			selected = -1
			_build_panel()
			_changed(true))


func _export_ac() -> void:
	var v := TrackSpec.validate(spec)
	if (v.errors as Array).size() > 0:
		_set_status("[color=#ff6666]Fix errors first.[/color]")
		return
	UiUtil.dir_dialog(self, func(dir: String) -> void:
		var r := AcTrackExporter.export_track(spec, dir)
		_set_status(("[color=#66ff88]%s[/color]" if r.ok else "[color=#ff6666]%s[/color]") % r.message))
