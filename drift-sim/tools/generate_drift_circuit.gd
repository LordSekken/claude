extends SceneTree
## Generates a drift circuit and saves it as a Drift Lab track (open it in the
## track editor, export it to AC or Unreal like any other track):
##   godot --headless --path drift-sim -s tools/generate_drift_circuit.gd -- <out.json> [seed] [length_m] [width_m] [scale] [flow]
## Prints the corner sheet (radius, angle, drift speed) so the scale can be checked.

func _init() -> void:
	await process_frame
	var args := OS.get_cmdline_user_args()
	if args.is_empty():
		printerr("usage: -- <out.json> [seed] [length_m] [width_m] [scale] [flow]")
		quit(2)
		return
	var params := {"seed": int(args[1]) if args.size() > 1 else 1}
	if args.size() > 2:
		params.length = float(args[2])
	if args.size() > 3:
		params.width = float(args[3])
	if args.size() > 4:
		params.scale = float(args[4])
	if args.size() > 5:
		params.flow = float(args[5])
	var r := DriftCircuitGenerator.generate(params)
	if not r.ok:
		printerr("FAIL " + str(r.message))
		quit(1)
		return
	var spec: Dictionary = r.spec
	JsonUtil.save_file(args[0], spec)
	print("OK   %s: %s m, %d corners, min radius %s m, elevation %s m (attempt %d)" % [spec.name, r.stats.length_m,
		(r.corners as Array).size(), r.stats.min_radius_m, r.stats.elevation_change_m, spec.generator.attempt])
	for c: Dictionary in r.corners:
		print("     %-8s R %5.1f m  %+6.1f deg  drift ~%3.0f km/h  at %4.1f%%" % [c.kind, c.radius_m, c.angle_deg,
			c.drift_speed_kmh, float(c.t) * 100.0])
	for w: String in r.warnings:
		print("WARN " + w)
	quit(0)
