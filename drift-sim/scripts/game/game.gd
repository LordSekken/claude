class_name Game
extends Node3D
## Gameplay scene: track + car + camera + HUD + scoring.

signal exit_requested

var vehicle: Vehicle
var camera: ChaseCamera
var hud: Hud
var scorer := DriftScorer.new()
var tracker: TrackTracker
var input: DriverInput
var track_spec: Dictionary = {}
var samples: Dictionary = {}
var start_xform := Transform3D(Basis(), Vector3(0, 1, 0))
var clips: Array = []  # [{node, data, cooldown}]
var pause_menu: Control
var build_result: Dictionary = {}


## track: either a TrackSpec dictionary or {"external": path}
func start(track: Dictionary, car_build: Dictionary) -> String:
	var app: Node = get_node("/root/App")
	build_result = CarBuilder.assemble(car_build, app.parts)
	if not build_result.ok:
		return "Car build error: " + ", ".join(build_result.errors)
	if track.has("external"):
		var r := ExternalTrackLoader.load_model(str(track.external))
		if not r.ok:
			return str(r.error)
		add_child(r.root)
		start_xform = r.start
		tracker = TrackTracker.new({})
	else:
		var built := TrackBuilder.build(track)
		add_child(built.root)
		track_spec = built.spec
		samples = built.samples
		tracker = TrackTracker.new(samples)
		if tracker.valid():
			start_xform = TrackSpec.transform_at(samples, float(track_spec.start), 0.9)
			for c: Node in built.root.get_node("Clips").get_children():
				clips.append({"node": c, "data": c.get_meta("clip"), "cooldown": 0.0})
		else:
			var pads: Array = track_spec.get("pads", [])
			if pads.size() > 0:
				var c := Vector2.ZERO
				for q: Array in pads[0].points:
					c += Vector2(float(q[0]), float(q[1]))
				c /= (pads[0].points as Array).size()
				start_xform = Transform3D(Basis(), Vector3(c.x, 0.9, c.y))
		# Zones: default target scales with zone length.
		var length := float(samples.get("length", 0.0))
		for z: Dictionary in track_spec.get("zones", []):
			var zz := z.duplicate()
			var frac := fposmod(float(z.end) - float(z.start), 1.0)
			if not zz.has("target"):
				zz["target"] = int(maxf(frac * length * 28.0, 3000.0))
			scorer.zones.append(zz)

	vehicle = Vehicle.new()
	vehicle.name = "Car"
	vehicle.setup(build_result.spec)
	add_child(vehicle)
	vehicle.add_child(CarVisual.create(vehicle))
	vehicle.reset_to(start_xform)
	vehicle.hard_impact.connect(scorer.on_wall_impact)

	camera = ChaseCamera.new()
	camera.target = vehicle
	camera.far = 3000.0
	add_child(camera)
	camera.snap()
	camera.make_current()

	input = DriverInput.new(vehicle)
	hud = Hud.new()
	hud.kmh = bool(get_node("/root/Settings").units_kmh)
	add_child(hud)
	hud.setup(vehicle, scorer, tracker, track_spec)
	_build_pause_menu()
	return ""


func _physics_process(dt: float) -> void:
	if vehicle == null or get_tree().paused or input == null:
		return
	input.update(dt)
	tracker.update(vehicle.global_position, dt, float(track_spec.get("start", 0.0)))
	var on_track := vehicle.on_road_wheels >= 2
	scorer.update(dt, vehicle.speed, vehicle.drift_angle, on_track, vehicle.wall_proximity, tracker.progress)
	input.rumble(1.0)
	# Clipping points: rear bumper passes through the ring while drifting.
	var rear := vehicle.global_transform * Vector3(0, 0, float(vehicle.spec.length) * 0.5)
	var front := vehicle.global_transform * Vector3(0, 0, -float(vehicle.spec.length) * 0.5)
	for c: Dictionary in clips:
		c.cooldown = maxf(float(c.cooldown) - dt, 0.0)
		var p: Vector3 = (c.node as Node3D).global_position
		var r := float(c.data.get("radius", 2.0))
		var d := minf(Vector2(rear.x - p.x, rear.z - p.z).length(), Vector2(front.x - p.x, front.z - p.z).length())
		if d < r and c.cooldown <= 0.0 and scorer.drifting:
			scorer.add_clip()
			c.cooldown = 3.0
	# Fell off the world.
	if vehicle.global_position.y < start_xform.origin.y - 80.0:
		reset_car()


func _unhandled_input(ev: InputEvent) -> void:
	if vehicle == null:
		return
	if ev.is_action_pressed("pause"):
		_toggle_pause()
		return
	if get_tree().paused:
		return
	input.handle_event(ev)
	if ev.is_action_pressed("camera"):
		camera.cycle()
	elif ev.is_action_pressed("reset_car"):
		reset_car()


func reset_car() -> void:
	scorer.lose_chain("RESET")
	var xf := start_xform
	if tracker.valid():
		var i := tracker.nearest(vehicle.global_position, true)
		xf = TrackSpec.transform_at(samples, float(i) / (samples.pos as PackedVector3Array).size(), 0.9)
	vehicle.reset_to(xf)
	camera.snap()


func _build_pause_menu() -> void:
	var layer := CanvasLayer.new()
	layer.layer = 10
	layer.process_mode = Node.PROCESS_MODE_ALWAYS
	add_child(layer)
	pause_menu = PanelContainer.new()
	pause_menu.set_anchors_preset(Control.PRESET_CENTER)
	pause_menu.position = Vector2(-180, -160)
	pause_menu.custom_minimum_size = Vector2(360, 0)
	pause_menu.visible = false
	layer.add_child(pause_menu)
	var vb := VBoxContainer.new()
	vb.add_theme_constant_override("separation", 10)
	pause_menu.add_child(vb)
	var title := Label.new()
	title.text = "PAUSED"
	title.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	title.add_theme_font_size_override("font_size", 32)
	vb.add_child(title)
	var stats := Label.new()
	stats.name = "Stats"
	vb.add_child(stats)
	for pair in [["Resume", _toggle_pause], ["Reset car", func() -> void: _toggle_pause(); reset_car()], ["Main menu", func() -> void: get_tree().paused = false; exit_requested.emit()]]:
		var b := Button.new()
		b.text = pair[0]
		b.custom_minimum_size = Vector2(0, 44)
		b.pressed.connect(pair[1])
		if pair[0] == "Resume":
			var sc := Shortcut.new()
			var ev := InputEventAction.new()
			ev.action = "pause"
			ev.pressed = true
			sc.events = [ev]
			b.shortcut = sc
		vb.add_child(b)


func _toggle_pause() -> void:
	get_tree().paused = not get_tree().paused
	pause_menu.visible = get_tree().paused
	if pause_menu.visible:
		var st: Dictionary = build_result.stats
		var lines := "%s\n%d hp · %d Nm · %d kg · %s\n\nScore %s · best chain %s\nlongest drift %.1fs · max angle %d°" % [
			build_result.spec.name, st.peak_hp, st.peak_torque_nm, st.mass_kg, st.drivetrain,
			DriftScorer.format_points(scorer.banked), DriftScorer.format_points(scorer.best_chain),
			scorer.longest_drift, int(scorer.max_angle)]
		for zn: String in scorer.zone_results:
			var zr: Dictionary = scorer.zone_results[zn]
			lines += "\n%s: %s %s" % [zn, "★".repeat(int(zr.stars)), DriftScorer.format_points(int(zr.points))]
		(pause_menu.get_node("VBoxContainer/Stats") as Label).text = lines
		(pause_menu.get_child(0).get_child(2) as Button).grab_focus()
