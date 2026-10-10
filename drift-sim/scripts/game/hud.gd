class_name Hud
extends CanvasLayer
## Speed/gear/rpm cluster, drift chain + multiplier, popups, zone info, minimap.

var vehicle: Vehicle
var scorer: DriftScorer
var tracker: TrackTracker
var minimap_info: Dictionary = {}
var kmh := true

var _chain_label: Label
var _mult_label: Label
var _total_label: Label
var _speed_label: Label
var _gear_label: Label
var _rpm_bar: ProgressBar
var _boost_label: Label
var _angle_label: Label
var _info_label: Label
var _popups: VBoxContainer
var _minimap: TextureRect
var _dot: Control
var _help: Label


func _label(size: int, color := Color.WHITE, align := HORIZONTAL_ALIGNMENT_LEFT) -> Label:
	var l := Label.new()
	l.add_theme_font_size_override("font_size", size)
	l.add_theme_color_override("font_color", color)
	l.add_theme_color_override("font_outline_color", Color(0, 0, 0, 0.85))
	l.add_theme_constant_override("outline_size", maxi(size / 8, 3))
	l.horizontal_alignment = align
	return l


## Pins a control to a screen corner/edge: a = anchor (0..1), o = offset px.
static func _pin(c: Control, a: Vector2, o: Vector2, sz: Vector2) -> void:
	c.anchor_left = a.x
	c.anchor_right = a.x
	c.anchor_top = a.y
	c.anchor_bottom = a.y
	c.offset_left = o.x
	c.offset_top = o.y
	c.offset_right = o.x + sz.x
	c.offset_bottom = o.y + sz.y


func setup(v: Vehicle, s: DriftScorer, t: TrackTracker, track_spec: Dictionary) -> void:
	vehicle = v
	scorer = s
	tracker = t
	var root := Control.new()
	root.set_anchors_preset(Control.PRESET_FULL_RECT)
	root.mouse_filter = Control.MOUSE_FILTER_IGNORE
	add_child(root)

	# Top centre: chain score + multiplier + popups.
	var top := VBoxContainer.new()
	_pin(top, Vector2(0.5, 0), Vector2(-300, 20), Vector2(600, 0))
	top.alignment = BoxContainer.ALIGNMENT_BEGIN
	root.add_child(top)
	_chain_label = _label(54, Color(1, 0.85, 0.2), HORIZONTAL_ALIGNMENT_CENTER)
	top.add_child(_chain_label)
	_mult_label = _label(28, Color(1, 0.6, 0.1), HORIZONTAL_ALIGNMENT_CENTER)
	top.add_child(_mult_label)
	_popups = VBoxContainer.new()
	top.add_child(_popups)

	# Top left: total + zone/lap info.
	var tl := VBoxContainer.new()
	tl.position = Vector2(24, 20)
	root.add_child(tl)
	_total_label = _label(30)
	tl.add_child(_total_label)
	_info_label = _label(20, Color(0.85, 0.9, 1.0))
	tl.add_child(_info_label)

	# Bottom right: cluster.
	var br := VBoxContainer.new()
	_pin(br, Vector2(1, 1), Vector2(-330, -175), Vector2(310, 150))
	root.add_child(br)
	var row := HBoxContainer.new()
	br.add_child(row)
	_speed_label = _label(56, Color.WHITE, HORIZONTAL_ALIGNMENT_RIGHT)
	_speed_label.custom_minimum_size = Vector2(220, 0)
	row.add_child(_speed_label)
	_gear_label = _label(56, Color(0.4, 0.9, 1.0), HORIZONTAL_ALIGNMENT_RIGHT)
	_gear_label.custom_minimum_size = Vector2(90, 0)
	row.add_child(_gear_label)
	_rpm_bar = ProgressBar.new()
	_rpm_bar.custom_minimum_size = Vector2(300, 18)
	_rpm_bar.show_percentage = false
	br.add_child(_rpm_bar)
	_boost_label = _label(18, Color(0.8, 0.8, 0.8), HORIZONTAL_ALIGNMENT_RIGHT)
	br.add_child(_boost_label)

	# Bottom left: drift angle.
	_angle_label = _label(40, Color(0.9, 0.9, 0.9))
	_pin(_angle_label, Vector2(0, 1), Vector2(24, -110), Vector2(200, 60))
	root.add_child(_angle_label)

	_help = _label(15, Color(1, 1, 1, 0.6))
	_pin(_help, Vector2(0, 1), Vector2(24, -34), Vector2(1100, 24))
	_help.text = "WASD/Arrows drive · Space handbrake · E/Q shift · Shift clutch · C camera · Z/X look · R reset · Esc menu   |   Pad: RT/LT, A handbrake, B/X shift, right stick look"
	root.add_child(_help)

	# Minimap.
	if not track_spec.is_empty() and (track_spec.get("points", []) as Array).size() >= 2:
		minimap_info = TrackRaster.render(track_spec, 220, Color(0, 0, 0, 0.35), Color(1, 1, 1, 0.75), true)
		_minimap = TextureRect.new()
		_minimap.texture = ImageTexture.create_from_image(minimap_info.image)
		_pin(_minimap, Vector2(1, 0), Vector2(-240, 20), Vector2(220, 220))
		root.add_child(_minimap)
		_dot = Control.new()
		_dot.custom_minimum_size = Vector2(220, 220)
		_dot.draw.connect(_draw_dot)
		_minimap.add_child(_dot)

	scorer.popup.connect(_on_popup)


func _draw_dot() -> void:
	if vehicle == null or minimap_info.is_empty():
		return
	var p := vehicle.global_position
	var px := (Vector2(p.x, p.z) - (minimap_info.origin as Vector2)) * float(minimap_info.scale)
	var f := vehicle.forward()
	var dir := Vector2(f.x, f.z).normalized()
	var tri := PackedVector2Array([px + dir * 7, px + dir.rotated(2.5) * 5, px + dir.rotated(-2.5) * 5])
	_dot.draw_colored_polygon(tri, Color(1, 0.2, 0.2))


func _on_popup(text: String, kind: String) -> void:
	var colors := {"bonus": Color(0.5, 1, 0.6), "clip": Color(1, 0.9, 0.2), "bank": Color(0.3, 1, 0.4), "fail": Color(1, 0.3, 0.3), "zone": Color(0.6, 0.8, 1)}
	var l := _label(30 if kind != "bank" else 40, colors.get(kind, Color.WHITE), HORIZONTAL_ALIGNMENT_CENTER)
	l.text = text
	_popups.add_child(l)
	var tw := create_tween()
	tw.tween_interval(1.4)
	tw.tween_property(l, "modulate:a", 0.0, 0.5)
	tw.tween_callback(l.queue_free)
	while _popups.get_child_count() > 4:
		_popups.get_child(0).free()


func _process(_dt: float) -> void:
	if vehicle == null:
		return
	var spd := vehicle.speed * (3.6 if kmh else 2.237)
	_speed_label.text = "%d %s" % [int(spd), "km/h" if kmh else "mph"]
	_gear_label.text = vehicle.gear_label()
	_rpm_bar.max_value = float(vehicle.spec.limiter_rpm)
	_rpm_bar.value = vehicle.rpm()
	_rpm_bar.modulate = Color(1, 0.3, 0.3) if vehicle.rpm() > float(vehicle.spec.limiter_rpm) * 0.92 else Color.WHITE
	_boost_label.text = ("%.2f bar  ·  " % vehicle.boost if vehicle.spec.turbo else "") + "%d rpm" % int(vehicle.rpm())
	_angle_label.text = "%d°" % int(absf(vehicle.drift_angle)) if scorer.drifting else ""
	if scorer.chain_active:
		_chain_label.text = DriftScorer.format_points(int(scorer.chain_points))
		_mult_label.text = "x%.1f   =  %s" % [scorer.multiplier, DriftScorer.format_points(scorer.chain_total())]
		_chain_label.modulate.a = 1.0 if scorer.drifting else 0.5 + 0.5 * absf(sin(Time.get_ticks_msec() * 0.01))
	else:
		_chain_label.text = ""
		_mult_label.text = ""
	_total_label.text = "SCORE  " + DriftScorer.format_points(scorer.banked)
	var info := ""
	if tracker and tracker.valid() and tracker.samples.closed:
		info += "Lap %d   %s" % [tracker.lap, _fmt_time(tracker.lap_time)]
		if tracker.best_lap > 0.0:
			info += "   best %s" % _fmt_time(tracker.best_lap)
		info += "\n"
	if scorer.zone_index >= 0:
		var z: Dictionary = scorer.zones[scorer.zone_index]
		info += "%s: %s / %s" % [z.get("name", "Zone"), DriftScorer.format_points(int(scorer.zone_points)), DriftScorer.format_points(int(z.get("target", 0)))]
	info += "\nbest chain %s" % DriftScorer.format_points(scorer.best_chain)
	_info_label.text = info
	if _dot:
		_dot.queue_redraw()


static func _fmt_time(t: float) -> String:
	return "%d:%05.2f" % [int(t / 60.0), fmod(t, 60.0)]
