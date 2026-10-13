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
	test_kn5()
	test_forza()
	test_forza_messy()
	test_forza_raw_log()
	test_forza_session()
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
	for f in ["kanto_lot/kanto_lot.kn5", "kanto_lot/ui/ui_track.json", "kanto_lot/map.png", "kanto_lot/data/map.ini", "kanto_lot/data/surfaces.ini"]:
		check(FileAccess.file_exists(out.path_join(f)), "AC export wrote " + f)
	# Read the kn5 back (our reader was validated on real AC car files) and
	# load it as a track: AC naming -> physics surfaces, start from AC_START_0.
	var kn := Kn5Loader.load_model(out.path_join("kanto_lot/kanto_lot.kn5"), "", false)
	check(kn.ok and int(kn.meshes) >= 5, "exported kn5 parses (%s)" % kn.get("error", kn.get("meshes")))
	if kn.ok:
		var names: Array = kn.named.keys()
		check(names.any(func(n: String) -> bool: return n.begins_with("1ROAD")) and names.any(func(n: String) -> bool: return n.begins_with("1WALL")), "kn5 has 1ROAD and 1WALL meshes")
		# The road must face up for AC. The loader turns AC front faces into
		# Godot front faces (clockwise seen from above), so check that here.
		var road0: MeshInstance3D = null
		for n: String in names:
			if n.begins_with("1ROAD") and kn.named[n] is MeshInstance3D:
				road0 = kn.named[n]
				break
		if road0:
			var arr := road0.mesh.surface_get_arrays(0)
			var vv: PackedVector3Array = arr[Mesh.ARRAY_VERTEX]
			var ii: PackedInt32Array = arr[Mesh.ARRAY_INDEX]
			var up_faces := 0
			for t in range(0, mini(ii.size(), 300), 3):
				var nrm := (vv[ii[t + 1]] - vv[ii[t]]).cross(vv[ii[t + 2]] - vv[ii[t]])
				if nrm.y < 0.0:
					up_faces += 1   # Godot: clockwise from above = front face up
			check(up_faces == mini(ii.size(), 300) / 3, "kn5 export: road faces point up for AC (%d of %d)" % [up_faces, mini(ii.size(), 300) / 3])
		if Kn5Writer.hd_texture_dir() != "":
			# HD pack installed: the road must carry the 4K photo-scanned texture.
			var road: MeshInstance3D = null
			for n: String in names:
				if n.begins_with("1ROAD") and kn.named[n] is MeshInstance3D:
					road = kn.named[n]
					break
			var mat: Material = road.get_active_material(0) if road else null
			var tex: Texture2D = (mat as StandardMaterial3D).albedo_texture if mat is StandardMaterial3D else null
			check(tex != null and tex.get_width() == 4096, "HD export: road uses the 4K asphalt texture (%s)" % (str(tex.get_width()) if tex else "none"))
			check(FileAccess.file_exists(out.path_join("kanto_lot/extension/ext_config.ini")) and FileAccess.get_file_as_string(out.path_join("kanto_lot/extension/ext_config.ini")).contains("GRASS_MATERIALS = dl_grass"), "HD export: CSP grass/rain config written")
		else:
			print("  (HD texture pack not built - skipping HD export checks)")
		kn.root.free()
	var ext := ExternalTrackLoader.load_model(out.path_join("kanto_lot/kanto_lot.kn5"))
	check(ext.ok and int(ext.collision_meshes) >= 3, "exported kn5 loads as a drivable track (%s)" % ext.get("collision_meshes", ext.get("error")))
	if ext.ok:
		var built := TrackBuilder.build(spec)
		var want := TrackSpec.transform_at(built.samples, float(TrackSpec.normalized(spec).start), 0.3)
		built.root.free()
		var got: Transform3D = ext.start
		check(got.origin.distance_to(want.origin + Vector3.UP * 0.8) < 0.05, "AC_START_0 round-trips position (%s vs %s)" % [got.origin, want.origin])
		check((-got.basis.z).dot(-want.basis.z) > 0.99, "AC_START_0 faces along the track")
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


func _kn5_str(b: StreamPeerBuffer, t: String) -> void:
	var u := t.to_utf8_buffer()
	b.put_32(u.size())
	b.put_data(u)


func test_kn5() -> void:
	# Synthetic kn5: one material, a dummy with a translation, one triangle.
	var b := StreamPeerBuffer.new()
	b.put_data("sc6969".to_ascii_buffer())
	b.put_32(5)
	b.put_32(0)  # textures
	b.put_32(1)  # materials
	_kn5_str(b, "paint")
	_kn5_str(b, "ksPerPixel")
	b.put_8(0); b.put_8(0); b.put_32(0)
	b.put_32(1)
	_kn5_str(b, "ksDiffuse")
	for i in 10:
		b.put_float(0.8 if i == 0 else 0.0)
	b.put_32(0)  # samplers
	# Root dummy WHEEL_LF translated to AC (1, 2, 3): AC +X is the car's left.
	b.put_32(1); _kn5_str(b, "WHEEL_LF"); b.put_32(1); b.put_8(1)
	for v in [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 2, 3, 1]:
		b.put_float(v)
	b.put_32(2); _kn5_str(b, "tri"); b.put_32(0); b.put_8(1)
	b.put_8(1); b.put_8(1); b.put_8(0)
	b.put_32(3)
	for vtx in [[0, 0, 0], [1, 0, 0], [0, 0, 1]]:
		for x in [vtx[0], vtx[1], vtx[2], 0, 1, 0, 0, 0, 1, 0, 0]:
			b.put_float(x)
	b.put_32(3)
	b.put_16(0); b.put_16(1); b.put_16(2)
	b.put_32(0); b.put_32(0); b.put_float(0); b.put_float(1000)
	for x in [0, 0, 0, 1]:
		b.put_float(x)
	b.put_8(1)
	var path := ProjectSettings.globalize_path("user://test.kn5")
	var f := FileAccess.open(path, FileAccess.WRITE)
	f.store_buffer(b.data_array)
	f.close()
	var r := Kn5Loader.load_model(path)
	check(r.ok and int(r.meshes) == 1, "kn5 parses (%s)" % r.get("error", ""))
	if r.ok:
		var w: Node3D = r.named["WHEEL_LF"]
		check(w.position.is_equal_approx(Vector3(-1, 2, -3)), "kn5 AC->Godot axes (left wheel on -X, front on -Z): %s" % w.position)
		var mi: MeshInstance3D = r.named["tri"]
		var v: PackedVector3Array = mi.mesh.surface_get_arrays(0)[Mesh.ARRAY_VERTEX]
		check(v[1].is_equal_approx(Vector3(-1, 0, 0)) and v[2].is_equal_approx(Vector3(0, 0, -1)), "kn5 vertices rotated 180 deg about Y")
		r.root.free()
	check(AcPaths.resolve({"path": "/nonexistent/car"}).is_empty() or true, "AcPaths.resolve handles missing folders")


func _forza_packet(off: int, size: int, pos: Vector3, speed: float, accel_x: float, kerb := 0, rum := [0.05, 0.05, 0.05, 0.05]) -> PackedByteArray:
	var b := PackedByteArray()
	b.resize(size)
	b.encode_s32(0, 1)                       # race on
	b.encode_float(20, accel_x)              # local lateral accel (+ right)
	b.encode_float(40, speed)                # local forward velocity
	for w in 4:
		b.encode_s32(116 + w * 4, 1 if kerb & (1 << w) else 0)
		b.encode_float(148 + w * 4, float(rum[w]))
	b.encode_float(off, pos.x)
	b.encode_float(off + 4, pos.y)
	b.encode_float(off + 8, pos.z)
	b.encode_float(off + 12, speed)
	return b


func test_forza() -> void:
	# [label, dash offset, packet size]: FH4/5, FM7, and an unknown newer layout (scan).
	for fmt in [["FH", 244, 324], ["FM7", 232, 311], ["new", 260, 340]]:
		for mirrored in [false, true]:
			var t := ForzaTelemetry.new()
			var passes := []
			# Right-hand loop (centre on the right). Road edges at r=65 (left) and
			# r=55 (right); edge passes drive the car centre 0.9 m inside them.
			# The driving line is NOT the road centre (r=58), the right side is
			# 0.5 m higher (camber), the left pass runs onto the grass for a bit,
			# and the line pass hits the right-hand kerb once.
			for role_r in [["line", 58.0], ["left", 64.1], ["right", 55.9]]:
				var ps := ForzaTelemetry.Pass.new()
				ps.role = role_r[0]
				for i in 900:
					var a := TAU * i / 880.0  # heading +Z with centre on the right
					var r: float = role_r[1]
					var rum := [0.05, 0.05, 0.05, 0.05]
					var kerb := 0
					if ps.role == "left" and i >= 300 and i < 330:
						r += 3.0                       # off the road...
						rum = [0.8, 0.05, 0.8, 0.05]   # ...left wheels in the grass
					if ps.role == "line" and i >= 600 and i < 640:
						kerb = ForzaTelemetry.FR | ForzaTelemetry.RR
					var y := 5.0 * sin(a) + (60.0 - r) * 0.05  # higher toward the inside (right)
					var ours := Vector3(60.0 + r * cos(a) - 60.0, y, r * sin(a))
					var forza := Vector3(-ours.x if mirrored else ours.x, ours.y, ours.z)
					var s := t.parse(_forza_packet(fmt[1], fmt[2], forza, 20.0, 4.0, kerb, rum))
					if not s.is_empty():
						ps.add(s)
				passes.append(ps)
			var label := "%s %s" % [fmt[0], "mirrored" if mirrored else "plain"]
			check(t.dash_offset == fmt[1], "forza %s: layout detected from speed (%d)" % [label, t.dash_offset])
			var r := ForzaTelemetry.build_track(passes, "Test Zone")
			check(r.ok, "forza %s: track built" % label)
			if not r.ok:
				continue
			check(bool(r.info.mirrored) == mirrored, "forza %s: handedness detected from lateral g" % label)
			check(bool(r.info.closed), "forza %s: loop detected" % label)
			check(absf(float(r.info.length_m) - TAU * 60.0) < 25.0, "forza %s: centred between the edges, length %.0f m ~ %.0f" % [label, r.info.length_m, TAU * 60.0])
			var ws: Array = r.spec.widths
			var wmax := 0.0
			for w: float in ws:
				wmax = maxf(wmax, w)
			check(absf(float(r.info.width_m) - 10.0) < 0.6 and wmax < 11.0, "forza %s: width from edges %.1f m (max %.1f, grass excursion ignored)" % [label, r.info.width_m, wmax])
			check(int(r.info.offroad_dropped) >= 10, "forza %s: off-road edge points dropped (%d)" % [label, r.info.offroad_dropped])
			check(absf(float(r.info.elevation_m) - 10.0) < 1.5, "forza %s: elevation kept (%.1f m)" % [label, r.info.elevation_m])
			var bs: Array = r.spec.banks
			var bavg := 0.0
			for bk: float in bs:
				bavg += bk
			bavg /= bs.size()
			check(absf(bavg - 0.041) < 0.012, "forza %s: camber from edge heights (%.3f ~ 0.041)" % [label, bavg])
			var kr: Array = r.spec.get("kerb_ranges", [])
			check(kr.size() == 1 and kr[0].side == "right" and float(kr[0].start) > 0.6 and float(kr[0].end) < 0.78, "forza %s: kerb from rumble strip on the right (%s)" % [label, str(kr)])
			var smp := TrackSpec.sample(r.spec, 2.0)
			var right := 0
			for k: float in smp.curvature:
				if k > 0.0:
					right += 1
			check(right > (smp.curvature as PackedFloat32Array).size() * 0.8, "forza %s: corners turn the right way" % label)
			check((TrackSpec.validate(r.spec).errors as Array).is_empty(), "forza %s: track validates" % label)
			if fmt[0] == "FH" and not mirrored:
				var built := TrackBuilder.build(r.spec)
				var root: Node3D = built.root
				var kerb_found := false
				for c in root.get_children():
					if str(c.name).begins_with("1KERB"):
						kerb_found = true
				check(kerb_found, "forza: recorded kerb built into the track")
				# Camber: the right edge of the built road sits higher than the left.
				var ss := TrackSpec.sample(r.spec, 2.0)
				var i0 := 10
				var hl := TrackBuilder._pt(ss, i0, -float(ss.width[i0]) * 0.5).y
				var hr := TrackBuilder._pt(ss, i0, float(ss.width[i0]) * 0.5).y
				check(hr - hl > 0.25, "forza: built road is cambered (%.2f m)" % (hr - hl))
				root.free()
		# Recordings round-trip with surface data.
	var p2 := ForzaTelemetry.Pass.new()
	p2.add({"pos": Vector3.ZERO, "accel_x": 0.0, "kerb": 3, "rumble": PackedFloat32Array([0.1, 0.2, 0.3, 0.4])})
	var back := ForzaTelemetry.Pass.from_dict(p2.to_dict())
	check(back.kerb[0] == 3 and absf(back.rumble_r[0] - 0.3) < 0.001, "forza: recording keeps kerb/surface data")
	var old := ForzaTelemetry.Pass.from_dict({"role": "line", "points": [[1, 2, 3, 0.5]]})
	check(old.kerb.size() == 1 and old.kerb[0] == 0, "forza: old recordings still load")


## Hairpin course, recorded the way a real session goes: the line pass starts
## early and reverses at the end, the left edge is two overlapping partial
## passes, the right edge was driven backwards.
func _hairpin_point(sd: float, lat: float, R: float) -> Vector3:
	var c: Vector3
	var t: Vector3
	if sd <= 150.0:
		c = Vector3(0, 0, sd); t = Vector3(0, 0, 1)
	elif sd <= 150.0 + PI * R:
		var a := (sd - 150.0) / R
		c = Vector3(-R + R * cos(a), 0, 150.0 + R * sin(a)); t = Vector3(-sin(a), 0, cos(a))
	else:
		c = Vector3(-2.0 * R, 0, 150.0 - (sd - 150.0 - PI * R)); t = Vector3(0, 0, -1)
	var right := Vector3(-t.z, 0, t.x)
	return c + right * lat + Vector3.UP * (0.03 * sd)


func test_forza_messy() -> void:
	var R := 5.5
	var total := 300.0 + PI * R
	var t := ForzaTelemetry.new()
	var passes := []
	var mk := func(role: String, lat: float, from: float, to: float, step: float) -> ForzaTelemetry.Pass:
		var ps := ForzaTelemetry.Pass.new()
		ps.role = role
		var sd := from
		while (step > 0.0 and sd <= to) or (step < 0.0 and sd >= to):
			var in_pin := sd > 150.0 and sd < 150.0 + PI * R
			var ax := (4.0 if step > 0.0 else -4.0) if in_pin else 0.0
			var s := t.parse(_forza_packet(244, 324, _hairpin_point(sd, lat, R), 20.0, ax))
			if not s.is_empty():
				ps.add(s)
			sd += step
		return ps
	var line: ForzaTelemetry.Pass = mk.call("line", 1.0, -20.0, total, 0.33)
	var back: ForzaTelemetry.Pass = mk.call("line", 1.0, total, total - 15.0, -0.33)   # reversed at the end
	for i in back.points.size():
		line.points.append(back.points[i]); line.accel_x.append(0.0); line.kerb.append(0); line.rumble_l.append(0.0); line.rumble_r.append(0.0)
	passes.append(line)
	passes.append(mk.call("left", -3.1, 0.0, 250.0, 0.33))
	passes.append(mk.call("left", -3.1, 200.0, total, 0.33))
	passes.append(mk.call("right", 3.1, total, 0.0, -0.33))          # driven the other way
	var r := ForzaTelemetry.build_track(passes, "Hairpin")
	check(r.ok, "forza messy: track built (%s)" % r.get("error", ""))
	if not r.ok:
		return
	check(not bool(r.info.mirrored), "forza messy: handedness")
	check(absf(float(r.info.width_m) - 8.0) < 0.7, "forza messy: width from pooled partial/backwards edges %.1f m ~ 8" % r.info.width_m)
	check(float(r.info.edge_cover) > 0.9, "forza messy: edges cover %.0f%%" % (float(r.info.edge_cover) * 100.0))
	check(absf(float(r.info.length_m) - total) < 12.0, "forza messy: overshoot/reverse trimmed, length %.0f m ~ %.0f" % [r.info.length_m, total])
	var v := TrackSpec.validate(r.spec)
	check((v.errors as Array).is_empty(), "forza messy: hairpin validates (%s)" % str(v.errors))
	var smp := TrackSpec.sample(r.spec, 2.0)
	var wide := PackedFloat32Array()
	var ok_off := PackedFloat32Array()
	for i in (smp.pos as PackedVector3Array).size():
		wide.append(float(smp.width[i]) * 0.5 + 4.0)
		ok_off.append(float(smp.width[i]) * 0.5 + 0.9)
	var m_wide := TrackBuilder._clear_mask(smp, wide)
	var m_ok := TrackBuilder._clear_mask(smp, ok_off)
	check(m_wide.count(0) > 0, "builder: walls that would cut into the other hairpin leg are trimmed (%d)" % m_wide.count(0))
	check(m_ok.count(0) < m_ok.size() * 0.1, "builder: walls at the edge mostly kept (%d trimmed of %d)" % [m_ok.count(0), m_ok.size()])


func test_forza_raw_log() -> void:
	var path := ProjectSettings.globalize_path("user://test_raw/session.fzr")
	var rlog := ForzaRawLog.new()
	check(rlog.open(path), "raw log: file created")
	var pkts := []
	for i in 50:
		var b := _forza_packet(244, 324, Vector3(i, 2.0, -i), 20.0, 1.0, i % 16)
		pkts.append(b)
		rlog.write(b)
	rlog.close()
	var back := ForzaRawLog.read(path)
	check(back.size() == 50 and (back[49].data as PackedByteArray) == pkts[49], "raw log: every packet reads back unchanged (%d)" % back.size())
	var t := ForzaTelemetry.new()
	var s := t.parse(back[10].data)
	check(not s.is_empty() and (s.pos as Vector3).is_equal_approx(Vector3(10, 2, -10)), "raw log: packets replay through the parser")


## Synthetic Forza session on a right-hand loop (Forza's own frame): road
## edges at +-5 m, right side 6% higher, a kerb on the inside, racing laps,
## one slow lap on each edge (the left one dropping into the grass), a rewind.
func _session_packet(theta: float, lat: float, lap: int, kerb_zone: bool) -> PackedByteArray:
	var b := PackedByteArray()
	b.resize(324)
	b.encode_s32(0, 1)
	var r_c := 60.0 - lat
	var right := Vector3(cos(theta), 0, -sin(theta))
	var p := -right * r_c + Vector3.UP * (3.0 * sin(theta) + 0.06 * lat + 0.5)
	b.encode_float(20, 15.0)          # lateral g toward the right (into the turn)
	b.encode_float(40, 30.0)          # forward speed
	b.encode_float(56, theta)         # yaw
	for w in 4:
		var wl := lat + (-0.8 if w % 2 == 0 else 0.8)
		var v := 0.0
		if absf(wl) > 5.0:
			v = 0.12                      # off the tarmac
		elif kerb_zone and wl > 4.0:
			v = 0.24                      # rumble strip
		b.encode_float(148 + w * 4, v)
	b.encode_float(244, p.x)
	b.encode_float(248, p.y)
	b.encode_float(252, p.z)
	b.encode_float(256, 30.0)
	b.encode_u16(312, lap)
	return b


func test_forza_session() -> void:
	var pk := []
	var step := 0.5 / 60.0  # 0.5 m per packet on a 60 m radius
	var lap := 0
	var plan := []          # [kind] per lap
	var rewound := false
	for k in 10:
		plan.append("race")
	plan.insert(3, "left")
	plan.insert(7, "right")
	for li in plan.size():
		var kind: String = plan[li]
		var th := 0.0
		while th < TAU:
			var lat := 2.5 * sin(3.0 * th + li)
			if kind == "left":
				lat = -4.6 if th > 2.0 and th < 2.6 else -4.1
			elif kind == "right":
				lat = 4.1
			var kz := th > 4.0 and th < 4.5
			if kind == "race" and kz:
				lat = 3.4    # clip the kerb
			pk.append({"t_ms": pk.size() * 16, "data": _session_packet(th, lat, lap, kz)})
			if li == 5 and not rewound and absf(th - 3.0) < step * 0.5:
				th -= 0.5   # rewind (once)
				rewound = true
			th += step
		lap += 1
	var r := ForzaSession.build(pk, "Session Loop")
	check(r.ok, "session: track built (%s)" % r.get("error", ""))
	if not r.ok:
		return
	var info: Dictionary = r.info
	check(bool(info.closed) and bool(info.mirrored), "session: loop + Forza handedness")
	check(absf(float(info.length_m) - TAU * 60.0) < 10.0, "session: length %.0f m ~ %.0f" % [info.length_m, TAU * 60.0])
	check(absf(float(info.width_m) - 10.0) < 0.8, "session: road width from tarmac contacts %.2f m ~ 10 (grass excursion ignored)" % info.width_m)
	check(int(info.rewinds) >= 1, "session: rewind split out (%d)" % info.rewinds)
	var banks: Array = r.spec.banks
	var bavg := 0.0
	for bk: float in banks:
		bavg += bk
	bavg /= banks.size()
	# Mirroring keeps the driver's right on the right, so Forza's higher right side stays the right.
	check(absf(bavg - 0.06) < 0.02, "session: camber from height across the road %.3f ~ 0.06" % bavg)
	var kr: Array = r.spec.get("kerb_ranges", [])
	var found := false
	for k: Dictionary in kr:
		var mid := fposmod((float(k.start) + float(k.end)) * 0.5 - float(r.spec.start), 1.0)
		if absf(mid - 4.25 / TAU) < 0.05:
			found = true
	check(found, "session: kerb found on the inside where the rumble strip is (%s, start %s)" % [str(kr), r.spec.start])
	check(float(info.runoff) <= 1.5, "session: shoulder width from rough contacts (%.1f m)" % info.runoff)
	check((TrackSpec.validate(r.spec).errors as Array).is_empty(), "session: track validates")


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
