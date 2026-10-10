extends SceneTree
## Renders screenshots of the main screens (needs a display / xvfb):
##   xvfb-run godot --path drift-sim --rendering-method gl_compatibility -s tests/screenshots.gd -- <out_dir>

var out := "/tmp/shots"


func _shot(name: String) -> void:
	await process_frame
	await process_frame
	await RenderingServer.frame_post_draw
	var img := root.get_viewport().get_texture().get_image()
	img.save_png(out.path_join(name + ".png"))
	print("saved ", name)


func _init() -> void:
	var args := OS.get_cmdline_user_args()
	if args.size() > 0:
		out = args[0]
	DirAccess.make_dir_recursive_absolute(out)
	root.size = Vector2i(1600, 900)
	await process_frame
	var main: Node = load("res://scripts/main.gd").new()
	root.add_child(main)
	for i in 5:
		await process_frame
	await _shot("01_menu")
	main._on_nav("garage")
	for i in 5:
		await process_frame
	await _shot("02_garage")
	main._on_nav("editor")
	for i in 10:
		await process_frame
	await _shot("03_editor")
	main._on_nav("settings")
	for i in 5:
		await process_frame
	await _shot("04_settings")
	main._on_nav("photo")
	for i in 5:
		await process_frame
	await _shot("05_photo")
	# Drive: full throttle, then a handbrake-initiated drift.
	var app: Node = root.get_node("App")
	if args.size() > 1:
		app.selected_track_path = args[1]
	main.show_game()
	var g: Game = main.current
	var t := 0.0
	var shot_times := {2.5: "06_drive_start", 7.6: "07_drift", 8.6: "08_drift_b", 10.5: "09_drift_c"}
	g.input = null
	while t < 11.0:
		var v := g.vehicle
		v.auto_shift = t < 6.5
		if t < 6.5:
			v.in_throttle = 1.0
			v.in_steer = 0.0
		elif t < 7.1:
			if v.gear > 2:
				v.gear = 2
			v.in_throttle = 0.4
			v.in_steer = -0.7
			v.in_handbrake = 1.0
		else:
			v.in_handbrake = 0.0
			v.steering_assist = 0.85
			v.in_steer = 0.0
			v.in_throttle = clampf(0.75 + 0.03 * (35.0 - absf(v.drift_angle)), 0.0, 1.0)
		g.tracker.update(v.global_position, 1.0 / 60.0, 0.0)
		g.scorer.update(1.0 / 60.0, v.speed, v.drift_angle, v.on_road_wheels >= 2, v.wall_proximity, g.tracker.progress)
		await process_frame
		t += 1.0 / 60.0
		for k: float in shot_times.keys():
			if absf(t - k) < 0.5 / 60.0:
				await _shot(shot_times[k])
	quit()
