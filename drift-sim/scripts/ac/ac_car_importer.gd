class_name AcCarImporter
extends RefCounted
## Imports an Assetto Corsa car's physics from its *unpacked* data folder
## (content/cars/<car>/data/*.ini) into Drift Lab parts + a build.
## Packed cars (data.acd only) must be unpacked first - Content Manager can
## do this ("Unpack data" for the car).

static func import_folder(dir: String) -> Dictionary:
	var data := dir
	if not FileAccess.file_exists(data.path_join("car.ini")) and FileAccess.file_exists(dir.path_join("data/car.ini")):
		data = dir.path_join("data")
	if not FileAccess.file_exists(data.path_join("car.ini")):
		if FileAccess.file_exists(dir.path_join("data.acd")) or FileAccess.file_exists(dir.get_base_dir().path_join("data.acd")):
			return {"ok": false, "error": "This car's physics are packed in data.acd. Unpack them first (Content Manager: select the car > 'Unpack data'), then pick the car folder again."}
		return {"ok": false, "error": "No car.ini found. Pick the car folder (content/cars/<car>) or its data folder."}
	var car_dir := data.get_base_dir() if data.ends_with("data") else data
	var car_id := JsonUtil.slugify(car_dir.get_file())
	var car := AcIni.load_ini(data.path_join("car.ini"))
	var eng := AcIni.load_ini(data.path_join("engine.ini"))
	var drv := AcIni.load_ini(data.path_join("drivetrain.ini"))
	var sus := AcIni.load_ini(data.path_join("suspensions.ini"))
	var tyr := AcIni.load_ini(data.path_join("tyres.ini"))
	var brk := AcIni.load_ini(data.path_join("brakes.ini"))
	var ui: Variant = JsonUtil.load_file(car_dir.path_join("ui/ui_car.json"))
	var name := AcIni.s(car, "INFO", "SCREEN_NAME", car_dir.get_file())
	var year := 0
	var brand := ""
	if ui is Dictionary:
		name = str(ui.get("name", name))
		year = int(ui.get("year", 0))
		brand = str(ui.get("brand", ""))

	# --- Engine ------------------------------------------------------------
	var lut_name := AcIni.s(eng, "HEADER", "POWER_CURVE", "power.lut")
	var lut := AcIni.parse_lut(FileAccess.get_file_as_string(data.path_join(lut_name))) if FileAccess.file_exists(data.path_join(lut_name)) else []
	if lut.size() < 3:
		return {"ok": false, "error": "Couldn't read the engine torque curve (%s)." % lut_name}
	var boost := 0.0
	if eng.has("TURBO_0"):
		boost = AcIni.f(eng, "TURBO_0", "MAX_BOOST", 0.0)
		var wg := AcIni.f(eng, "TURBO_0", "WASTEGATE", 0.0)
		if wg > 0.0:
			boost = minf(boost, wg)
	var limiter := AcIni.f(eng, "ENGINE_DATA", "LIMITER", 7000.0)
	var idle := AcIni.f(eng, "ENGINE_DATA", "MINIMUM", 900.0)
	# AC's power.lut is the naturally-aspirated curve, roughly at the wheels;
	# boost multiplies it by (1 + boost). Convert to crank torque at stock boost.
	var curve := []
	for p: Array in lut:
		if float(p[0]) < idle * 0.8 or float(p[0]) > limiter + 600.0:
			continue
		curve.append([float(p[0]), snappedf(float(p[1]) * (1.0 + boost) / 0.87, 0.1)])
	if curve.size() < 3:
		curve = lut
	var engine := {
		"id": car_id + "_engine", "name": name + " engine (AC import)",
		"mass_kg": 160.0, "idle_rpm": idle, "limiter_rpm": limiter, "max_safe_rpm": limiter + 300.0,
		"aspiration": "turbo" if boost > 0.0 else "na", "boost_bar": boost, "max_boost_bar": maxf(boost * 1.4, boost + 0.2),
		"spool_rpm": AcIni.f(eng, "TURBO_0", "REFERENCE_RPM", 3500.0) * 0.8, "turbo_lag_s": 0.5,
		"inertia": AcIni.f(eng, "ENGINE_DATA", "INERTIA", 0.18),
		"torque_curve": curve,
		"notes": "Imported from Assetto Corsa %s/%s." % [car_dir.get_file(), lut_name],
	}

	# --- Gearbox ------------------------------------------------------------
	var count := int(AcIni.f(drv, "GEARS", "COUNT", 5))
	var gears := []
	for i in range(1, count + 1):
		gears.append(AcIni.f(drv, "GEARS", "GEAR_%d" % i, 1.0))
	var trans := {
		"id": car_id + "_gearbox", "name": name + " gearbox (AC import)",
		"gear_ratios": gears, "final_drive": AcIni.f(drv, "GEARS", "FINAL", 4.0),
		"reverse_ratio": absf(AcIni.f(drv, "GEARS", "GEAR_R", -3.3)),
		"max_torque_nm": 0, "mass_kg": 50.0,
		"shift_time_s": AcIni.f(drv, "GEARBOX", "CHANGE_UP_TIME", 180.0) / 1000.0,
	}

	# --- Chassis ------------------------------------------------------------
	var inertia := AcIni.s(car, "BASIC", "INERTIA", "1.8,1.3,4.4").split(",")
	var w := float(inertia[0]) if inertia.size() > 0 else 1.8
	var h := float(inertia[1]) if inertia.size() > 1 else 1.3
	var l := float(inertia[2]) if inertia.size() > 2 else 4.4
	var lock_wheel := AcIni.f(car, "CONTROLS", "STEER_LOCK", 450.0)
	var ratio := maxf(AcIni.f(car, "CONTROLS", "STEER_RATIO", 13.0), 1.0)
	var road_lock := clampf(lock_wheel / ratio, 20.0, 75.0)
	var traction := AcIni.s(drv, "TRACTION", "TYPE", "RWD").to_upper()
	if traction.begins_with("AWD"):
		traction = "AWD"
	elif traction != "FWD":
		traction = "RWD"
	var radius := AcIni.f(tyr, "FRONT", "RADIUS", 0.31)
	var chassis := {
		"id": car_id, "name": name, "make": brand, "year": year, "body_style": "coupe", "color": "#c8c8c8",
		"mass_kg": AcIni.f(car, "BASIC", "TOTALMASS", 1300.0) - 75.0,
		"stock_drivetrain": traction, "stock_engine": engine.id, "stock_transmission": trans.id,
		"wheelbase_m": AcIni.f(sus, "BASIC", "WHEELBASE", 2.5),
		"track_front_m": AcIni.f(sus, "FRONT", "TRACK", 1.5), "track_rear_m": AcIni.f(sus, "REAR", "TRACK", 1.5),
		"length_m": clampf(l, 2.5, 6.5), "width_m": clampf(w, 1.4, 2.3), "height_m": clampf(h, 1.0, 2.2),
		"cg_height_m": clampf(h * 0.36, 0.35, 0.7), "weight_front": AcIni.f(sus, "BASIC", "CG_LOCATION", 0.52),
		"engine_cg_from_front_axle_m": 0.1, "wheel_radius_m": radius,
		"tire_width_mm": AcIni.f(tyr, "FRONT", "WIDTH", 0.225) * 1000.0,
		"steer_lock_deg": road_lock, "max_steer_lock_deg": maxf(road_lock, 68.0),
		"brake_torque_nm": AcIni.f(brk, "DATA", "MAX_TORQUE", 3000.0) * 2.0,
		"brake_bias_front": AcIni.f(brk, "DATA", "FRONT_SHARE", 0.65),
		"handbrake_torque_nm": AcIni.f(brk, "DATA", "HANDBRAKE_TORQUE", 1000.0),
		"cd": 0.34, "frontal_area_m2": w * h * 0.82,
		"suspension": {
			"spring_front_n_mm": AcIni.f(sus, "FRONT", "SPRING_RATE", 60000.0) / 1000.0,
			"spring_rear_n_mm": AcIni.f(sus, "REAR", "SPRING_RATE", 50000.0) / 1000.0,
			"damp_bump_front": AcIni.f(sus, "FRONT", "DAMP_BUMP", 3000.0),
			"damp_rebound_front": AcIni.f(sus, "FRONT", "DAMP_REBOUND", 5000.0),
			"damp_bump_rear": AcIni.f(sus, "REAR", "DAMP_BUMP", 3000.0),
			"damp_rebound_rear": AcIni.f(sus, "REAR", "DAMP_REBOUND", 5000.0),
			"arb_front_n_mm": AcIni.f(sus, "ARB", "FRONT", 20000.0) / 1000.0,
			"arb_rear_n_mm": AcIni.f(sus, "ARB", "REAR", 10000.0) / 1000.0,
			"travel_m": 0.18,
		},
		"notes": "Imported from Assetto Corsa (%s). CG height and engine mass are estimates." % car_dir.get_file(),
	}

	var lib: PartsLibrary = Engine.get_main_loop().root.get_node("App").parts
	for pair in [["chassis", chassis], ["engines", engine], ["transmissions", trans]]:
		var errs := lib.save_user_part(pair[0], pair[1])
		if errs.size() > 0:
			return {"ok": false, "error": "Imported %s looks wrong: %s" % [pair[0], ", ".join(errs)]}

	var tire := func(sec: String) -> Dictionary:
		var d := {"compound": "sport", "size": null}
		var dy := AcIni.f(tyr, sec, "DY_REF", 0.0)
		var dx := AcIni.f(tyr, sec, "DX_REF", 0.0)
		if dy > 0.0:
			d["mu_lat"] = dy
		if dx > 0.0:
			d["mu_long"] = dx
		var fla := AcIni.f(tyr, sec, "FRICTION_LIMIT_ANGLE", 0.0)
		if fla > 0.0:
			d["peak_slip_angle_deg"] = fla
		return d
	var dtype := "lsd"
	var dp := AcIni.f(drv, "DIFFERENTIAL", "POWER", 0.0)
	var dc := AcIni.f(drv, "DIFFERENTIAL", "COAST", 0.0)
	if dp >= 0.99 and dc >= 0.99:
		dtype = "welded"
	elif dp <= 0.01 and dc <= 0.01 and AcIni.f(drv, "DIFFERENTIAL", "PRELOAD", 0.0) < 5.0:
		dtype = "open"
	var build := {
		"id": car_id, "name": name, "chassis": car_id, "engine": null, "transmission": null, "drivetrain": null,
		"color": "#c8c8c8",
		"tune": {
			"diff": {"type": dtype, "power": dp, "coast": dc, "preload_nm": AcIni.f(drv, "DIFFERENTIAL", "PRELOAD", 0.0)},
			"awd_front_split": AcIni.f(drv, "AWD", "FRONT_SHARE", 0.4),
			"tire_front": tire.call("FRONT"), "tire_rear": tire.call("REAR"),
		},
		"notes": "Imported from Assetto Corsa: %s" % car_dir,
	}
	JsonUtil.save_file("user://cars/%s.json" % car_id, build)
	return {"ok": true, "build": build}
