extends SceneTree
## Headless test suite:
##   godot --headless --path drift-sim -s tests/run_tests.gd

var failures := 0
var passes := 0
var app: Node


func check(cond: bool, what: String) -> void:
	if cond:
		passes += 1
	else:
		failures += 1
		printerr("FAIL: ", what)


func _init() -> void:
	await process_frame
	app = root.get_node("App")
	test_tire()
	test_parts_and_builds()
	test_tracks()
	test_tracer()
	test_scorer()
	test_ac_import()
	test_ac_export()
	await test_agent_tools()
	await test_physics()
	await test_game_smoke()
	print("\n%d passed, %d failed" % [passes, failures])
	quit(1 if failures > 0 else 0)


func test_tire() -> void:
	var t := TireModel.new({"tail": 0.8})
	check(absf(t.curve(1.0) - 1.0) < 1e-3, "tyre curve peaks at s=1")
	check(t.curve(0.5) < 1.0 and t.curve(0.5) > 0.5, "tyre curve rising before peak")
	check(absf(t.curve(1000.0) - 0.8) < 0.02, "tyre curve tends to tail")
	var f := t.compute(0.0, deg_to_rad(4.0), 4000.0)
	check(f.y < 0.0 and absf(f.x) < 1e-3, "pure slip angle -> restoring lateral force only")
	var spin := t.compute(1.0, deg_to_rad(4.0), 4000.0)
	check(absf(spin.y) < absf(f.y), "wheelspin reduces lateral grip (combined slip)")


func test_parts_and_builds() -> void:
	var lib: PartsLibrary = app.parts
	check(lib.ids("chassis").has("subaru_legacy_outback_1998"), "Outback chassis in library")
	for kind in PartsLibrary.KINDS:
		for id: String in lib.ids(kind):
			var errs := PartsLibrary.validate_part(kind, lib.get_part(kind, id))
			check(errs.is_empty(), "%s/%s valid: %s" % [kind, id, errs])
	for path in app.list_cars():
		var b: Dictionary = JsonUtil.load_file(path)
		var r := CarBuilder.assemble(b, lib)
		check(r.ok, "build %s assembles: %s" % [path, r.get("errors", [])])
	var stock := CarBuilder.assemble(JsonUtil.load_file("res://data/cars/outback98_stock.json"), lib)
	check(absf(float(stock.stats.peak_hp) - 165.0) < 10.0, "stock Outback ~165 hp (got %s)" % stock.stats.peak_hp)
	check(absf(float(stock.stats.mass_kg) - 1505.0) < 5.0, "stock Outback curb weight")
	check(stock.spec.drivetrain == "AWD", "stock Outback is AWD")
	var drift := CarBuilder.assemble(JsonUtil.load_file("res://data/cars/outback98_wrx_rwd_drift.json"), lib)
	check(drift.spec.drivetrain == "RWD", "drift Outback converted to RWD")
	check(float(drift.stats.peak_hp) > 240.0 and float(drift.stats.peak_hp) < 300.0, "tuned EJ205 power plausible (%s)" % drift.stats.peak_hp)
	check(float(drift.stats.mass_kg) < float(stock.stats.mass_kg), "drift build lighter than stock")
	check((drift.warnings as Array).any(func(w: Variant) -> bool: return str(w).contains("AWD->RWD")), "RWD conversion note present")
	var bad := PartsLibrary.validate_part("engines", {"id": "x", "name": "x", "mass_kg": 100, "idle_rpm": 800, "limiter_rpm": 7000, "torque_curve": [[1000, 100], [900, 120], [3000, 50]]})
	check(bad.size() > 0, "engine with non-increasing rpm rejected")
	check(absf(CarBuilder.tire_radius("235/40R17", 0.3) - 0.3099) < 0.001, "tyre size parsing")


func test_tracks() -> void:
	for path in app.list_tracks():
		var spec: Dictionary = JsonUtil.load_file(path)
		var v := TrackSpec.validate(spec)
		check((v.errors as Array).is_empty(), "track %s valid: %s" % [path, v.errors])
		var built := TrackBuilder.build(spec)
		check(built.root.get_child_count() > 2, "track %s builds" % path)
		built.root.free()
	# A figure-8 that crosses itself on the flat must be rejected.
	var pts := []
	for i in 24:
		var t := TAU * i / 24.0
		pts.append([200.0 * sin(t), 100.0 * sin(2.0 * t)])
	var v8 := TrackSpec.validate({"points": pts, "width": 12})
	check((v8.errors as Array).size() > 0, "self-crossing track rejected")
	var s := TrackSpec.sample({"points": [[0, 0], [100, 0], [100, 100], [0, 100]], "closed": true}, 2.0)
	check(absf(float(s.length) - 360.0) < 80.0, "square-ish loop length plausible (%s)" % s.length)


func test_tracer() -> void:
	# Aerial-style: grey ring road (radius 150 px, 14 px wide) on green.
	var img := Image.create(400, 400, false, Image.FORMAT_RGB8)
	img.fill(Color(0.25, 0.45, 0.2))
	for y in 400:
		for x in 400:
			var d := Vector2(x - 200, y - 200).length()
			if absf(d - 150.0) < 7.0:
				img.set_pixel(x, y, Color(0.35, 0.35, 0.36))
	var r := ImageTracer.trace(img, {"mode": "road", "road_width_m": 12.0})
	check(r.ok, "tracer finds ring road: %s" % r.get("error", ""))
	if r.ok:
		check(bool(r.spec.closed), "ring traced as closed loop")
		# 150 px radius with 14 px road = 12 m -> radius ~128 m -> length ~805 m.
		var length := float(r.info.length_m)
		check(length > 650.0 and length < 950.0, "ring length scaled from road width (%d m)" % length)
		check((TrackSpec.validate(r.spec).errors as Array).is_empty(), "traced ring validates")
	# Sketch: dark ellipse outline on white paper.
	var sk := Image.create(300, 200, false, Image.FORMAT_RGB8)
	sk.fill(Color(0.97, 0.97, 0.95))
	for k in 2000:
		var t := TAU * k / 2000.0
		var p := Vector2(150 + 120 * cos(t), 100 + 70 * sin(t))
		for dx in range(-1, 2):
			for dy in range(-1, 2):
				sk.set_pixel(int(p.x) + dx, int(p.y) + dy, Color(0.05, 0.05, 0.08))
	var r2 := ImageTracer.trace(sk, {"mode": "auto", "target_length_m": 1000.0})
	check(r2.ok and r2.info.mode == "lines", "sketch auto-detected as lines")
	if r2.ok:
		check(bool(r2.spec.closed) and absf(float(r2.info.length_m) - 1000.0) < 120.0, "sketch scaled to target length (%s)" % r2.info.length_m)


func test_scorer() -> void:
	var s := DriftScorer.new()
	s.zones = [{"name": "Z", "start": 0.1, "end": 0.3, "target": 500}]
	var banked := [0]
	s.chain_banked.connect(func(p: int) -> void: banked[0] = p)
	for i in 240:  # 4 s drifting at 70 km/h, 35 deg, inside the zone
		s.update(1.0 / 60.0, 70.0 / 3.6, 35.0, true, INF, 0.15)
	check(s.chain_active and s.chain_points > 500.0, "drifting accrues chain points")
	check(s.multiplier > 1.0, "sustained drift grows multiplier")
	for i in 180:  # straighten up for 3 s -> bank
		s.update(1.0 / 60.0, 70.0 / 3.6, 0.0, true, INF, 0.2)
	check(banked[0] > 0 and s.banked == banked[0], "chain banks after delay")
	s.update(1.0 / 60.0, 70.0 / 3.6, 0.0, true, INF, 0.5)  # leave zone
	check(s.zone_results.has("Z") and int(s.zone_results.Z.stars) == 3, "zone scored with stars")
	for i in 60:
		s.update(1.0 / 60.0, 70.0 / 3.6, -30.0, true, INF, 0.6)
	var before := s.banked
	s.on_wall_impact(5.0)
	check(not s.chain_active and s.banked == before, "wall hit loses unbanked chain")
	# Transition bonus.
	var t := DriftScorer.new()
	for i in 60:
		t.update(1.0 / 60.0, 20.0, 30.0, true, INF, -1)
	var m := t.multiplier
	for i in 5:
		t.update(1.0 / 60.0, 20.0, -30.0, true, INF, -1)
	check(t.multiplier > m, "transition bumps multiplier")
	check(DriftScorer.format_points(1234567) == "1,234,567", "points formatting")


func test_ac_import() -> void:
	var dir := "user://test_ac_car/data"
	DirAccess.make_dir_recursive_absolute(dir)
	var files := {
		"car.ini": "[INFO]\nSCREEN_NAME=Test S13\n[BASIC]\nTOTALMASS=1275\nINERTIA=1.69,1.29,4.52\n[CONTROLS]\nSTEER_LOCK=540\nSTEER_RATIO=9 ; drift ratio\n",
		"engine.ini": "[HEADER]\nPOWER_CURVE=power.lut\n[ENGINE_DATA]\nINERTIA=0.15\nLIMITER=7800\nMINIMUM=900\n[TURBO_0]\nMAX_BOOST=1.0\nWASTEGATE=0.9\nREFERENCE_RPM=4500\n",
		"power.lut": "0|0\n1000|90\n3000|140\n5000|165\n6500|160\n7800|130\n",
		"drivetrain.ini": "[TRACTION]\nTYPE=RWD\n[GEARS]\nCOUNT=5\nGEAR_R=-3.38\nGEAR_1=3.32\nGEAR_2=1.90\nGEAR_3=1.31\nGEAR_4=1.00\nGEAR_5=0.76\nFINAL=4.11\n[DIFFERENTIAL]\nPOWER=1.0\nCOAST=1.0\nPRELOAD=0\n[GEARBOX]\nCHANGE_UP_TIME=120\n",
		"suspensions.ini": "[BASIC]\nWHEELBASE=2.475\nCG_LOCATION=0.54\n[FRONT]\nTRACK=1.48\nSPRING_RATE=90000\nDAMP_BUMP=3500\nDAMP_REBOUND=6000\n[REAR]\nTRACK=1.47\nSPRING_RATE=70000\nDAMP_BUMP=3000\nDAMP_REBOUND=5000\n[ARB]\nFRONT=30000\nREAR=8000\n",
		"tyres.ini": "[FRONT]\nWIDTH=0.225\nRADIUS=0.31\nDY_REF=1.15\nDX_REF=1.2\nFRICTION_LIMIT_ANGLE=8.5\n[REAR]\nWIDTH=0.235\nRADIUS=0.31\nDY_REF=1.05\nDX_REF=1.1\n",
		"brakes.ini": "[DATA]\nMAX_TORQUE=2200\nFRONT_SHARE=0.68\nHANDBRAKE_TORQUE=2600\n",
	}
	for f: String in files:
		var fa := FileAccess.open(dir.path_join(f), FileAccess.WRITE)
		fa.store_string(files[f])
		fa.close()
	var r := AcCarImporter.import_folder(ProjectSettings.globalize_path("user://test_ac_car"))
	check(r.ok, "AC car import: %s" % r.get("error", ""))
	if r.ok:
		var res := CarBuilder.assemble(r.build, app.parts)
		check(res.ok, "imported AC car assembles")
		check(res.spec.diff_type == "welded", "AC POWER=COAST=1 -> welded diff")
		check(absf(rad_to_deg(float(res.spec.steer_lock)) - 60.0) < 0.5, "AC steer lock 540/9 = 60 deg")
		check(int(res.spec.gears.size()) == 5, "AC gears imported")
		check(float(res.stats.peak_hp) > 150.0 and float(res.stats.peak_hp) < 360.0, "AC turbo engine power plausible (%s)" % res.stats.peak_hp)


func test_ac_export() -> void:
	var spec: Dictionary = JsonUtil.load_file("res://tracks/kanto_lot.json")
	var out := ProjectSettings.globalize_path("user://test_ac_export")
	var r := AcTrackExporter.export_track(spec, out)
	check(r.ok, "AC export: %s" % r.message)
	for f in ["kanto_lot/kanto_lot.glb", "kanto_lot/ui/ui_track.json", "kanto_lot/map.png", "kanto_lot/data/map.ini", "kanto_lot/data/surfaces.ini"]:
		check(FileAccess.file_exists(out.path_join(f)), "AC export wrote " + f)
	# Re-import the glb with the external loader (AC naming -> physics).
	var ext := ExternalTrackLoader.load_model(out.path_join("kanto_lot/kanto_lot.glb"))
	check(ext.ok and int(ext.collision_meshes) >= 3, "exported glb re-imports with AC surfaces (%s)" % ext.get("collision_meshes", ext.get("error")))
	if ext.ok:
		check((ext.start as Transform3D).origin.length() > 1.0, "AC_START_0 found in re-imported model")
		ext.root.free()
	check(ExternalTrackLoader.classify("1ROAD_12") == "asphalt" and ExternalTrackLoader.classify("2KERB") == "kerb" and ExternalTrackLoader.classify("1WALL") == "wall" and ExternalTrackLoader.classify("tree01") == "", "AC surface name classification")


func test_agent_tools() -> void:
	# Exercise the tools Claude calls, without the network.
	var ga := GarageAgent.new()
	root.add_child(ga)
	var listed: Dictionary = await ga._handle("list_parts", {"kind": "engines"})
	check(str(listed.content).contains("subaru_ej205_wrx"), "garage tool: list_parts")
	var bad: Dictionary = await ga._handle("save_part", {"kind": "engines", "part": {"id": "x"}})
	check(bool(bad.get("is_error", false)), "garage tool: invalid part rejected")
	var eng := {"id": "test_ej20g", "name": "Subaru EJ20G (test)", "mass_kg": 140, "idle_rpm": 800, "limiter_rpm": 7000,
		"aspiration": "turbo", "boost_bar": 0.8, "torque_curve": [[1000, 120], [3000, 250], [4500, 300], [6000, 270], [7000, 230]]}
	var saved: Dictionary = await ga._handle("save_part", {"kind": "engines", "part": eng})
	check(not bool(saved.get("is_error", false)), "garage tool: save_part")
	var build := {"name": "Test Outback EJ20G", "chassis": "subaru_legacy_outback_1998", "engine": "test_ej20g", "drivetrain": "RWD", "tune": {"diff": {"type": "welded"}}}
	var ev: Dictionary = await ga._handle("evaluate_build", {"build": build})
	var evd: Variant = JSON.parse_string(str(ev.content))
	check(evd is Dictionary and float(evd.stats.peak_hp) > 150.0, "garage tool: evaluate_build returns stats")
	var sv: Dictionary = await ga._handle("save_build", {"build": build})
	check(not bool(sv.get("is_error", false)) and FileAccess.file_exists(ga.saved_path), "garage tool: save_build writes file")
	DirAccess.remove_absolute(ga.saved_path)
	DirAccess.remove_absolute("user://parts/engines/test_ej20g.json")
	app.parts.reload()
	ga.queue_free()
	var ta := TrackAgent.new()
	root.add_child(ta)
	var img := Image.create(300, 300, false, Image.FORMAT_RGB8)
	img.fill(Color(0.25, 0.45, 0.2))
	for y in 300:
		for x in 300:
			if absf(Vector2(x - 150, y - 150).length() - 110.0) < 6.0:
				img.set_pixel(x, y, Color(0.35, 0.35, 0.36))
	ta.image = img
	var tr: Dictionary = await ta._handle("trace_image", {"mode": "road", "road_width_m": 12})
	check(tr.content is Array and (tr.content as Array).size() == 2 and str(tr.content[1].type) == "image", "track tool: trace_image returns points + debug image")
	var sub: Dictionary = await ta._handle("submit_track", {"track": JsonUtil.load_file("res://tracks/minami_tech.json")})
	check(not ta.best_spec.is_empty() and str(sub.content[0].text).contains("errors"), "track tool: submit_track validates + previews")
	var body := JSON.stringify({"tools": ga._tools()})
	check(body.contains("web_search_20260209") and body.contains("decode_vin"), "garage agent tool list serialises")
	ta.queue_free()


func test_physics() -> void:
	var track := TrackBuilder.build({"points": [], "pads": [{"points": [[-500, -500], [500, -500], [500, 500], [-500, 500]]}], "scenery": {"density": 0}})
	root.add_child(track.root)
	for car in ["outback98_stock", "outback98_wrx_rwd_drift", "s14_sr20_drift", "miata_na_drift"]:
		var res := CarBuilder.assemble(JsonUtil.load_file("res://data/cars/%s.json" % car), app.parts)
		var v := Vehicle.new()
		v.setup(res.spec)
		root.add_child(v)
		v.reset_to(Transform3D(Basis(), Vector3(0, 0.6, 400)))
		v.auto_shift = true
		var max_angle := 0.0
		var t := 0.0
		var dt := 1.0 / 240.0
		var speed_at_6 := 0.0
		var ok := true
		while t < 9.0:
			if t < 6.0:
				v.in_throttle = 1.0
			else:
				v.auto_shift = false
				v.in_throttle = 0.9
				v.in_steer = 0.7
				v.in_handbrake = 1.0 if t < 6.6 else 0.0
			await physics_frame
			t += dt
			if absf(t - 6.0) < dt:
				speed_at_6 = v.speed * 3.6
			if is_nan(v.global_position.x) or v.global_position.y < -5.0 or v.global_position.y > 5.0:
				ok = false
				break
			if t > 6.0:
				max_angle = maxf(max_angle, absf(v.drift_angle))
		check(ok, "%s physics stays stable" % car)
		check(speed_at_6 > 45.0, "%s accelerates (%.0f km/h after 6 s)" % [car, speed_at_6])
		if car == "outback98_stock":
			# Stock AWD + viscous centre: the handbrake can't lock the rears, it
			# just understeers - like the real car.
			check(max_angle < 15.0, "stock AWD Outback resists handbrake slides (%.0f deg)" % max_angle)
		else:
			check(max_angle > 15.0, "%s can be provoked into a slide (max %.0f deg)" % [car, max_angle])
		check(v.global_basis.y.y > 0.5, "%s stays on its wheels" % car)
		v.queue_free()
		await physics_frame
	track.root.queue_free()


func test_game_smoke() -> void:
	for path in app.list_tracks():
		var g := Game.new()
		root.add_child(g)
		var err := g.start(JsonUtil.load_file(path), JsonUtil.load_file("res://data/cars/outback98_wrx_rwd_drift.json"))
		check(err == "", "game starts on %s: %s" % [path, err])
		for i in 120:
			g.vehicle.in_throttle = 1.0
			await physics_frame
		check(g.vehicle.global_position.y > -10.0, "car on track %s" % path)
		g.queue_free()
		await process_frame
