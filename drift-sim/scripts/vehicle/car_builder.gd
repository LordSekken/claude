class_name CarBuilder
extends RefCounted
## Assembles a modular build (chassis + engine + transmission + drivetrain +
## tune) into a physics spec, and computes realistic-ish stats/warnings.
##
## Build JSON:
## {
##   "id", "name", "chassis": <id>, "engine": <id|null = stock>,
##   "transmission": <id|null = stock>, "drivetrain": "RWD"|"FWD"|"AWD"|null,
##   "tune": { ... see TUNE_DEFAULTS ... }, "color": "#rrggbb", "notes": ""
## }

const TIRE_COMPOUNDS := {
	"economy": {"mu_long": 0.95, "mu_lat": 0.90, "tail": 0.84, "peak_slip_angle_deg": 9.0, "peak_slip_ratio": 0.12},
	"street": {"mu_long": 1.05, "mu_lat": 1.00, "tail": 0.82, "peak_slip_angle_deg": 8.5, "peak_slip_ratio": 0.11},
	"sport": {"mu_long": 1.15, "mu_lat": 1.10, "tail": 0.80, "peak_slip_angle_deg": 8.0, "peak_slip_ratio": 0.10},
	"semi_slick": {"mu_long": 1.30, "mu_lat": 1.25, "tail": 0.77, "peak_slip_angle_deg": 7.0, "peak_slip_ratio": 0.09},
	"drift": {"mu_long": 1.18, "mu_lat": 1.12, "tail": 0.84, "peak_slip_angle_deg": 8.0, "peak_slip_ratio": 0.10},
}

## Approximate driveline component masses (kg) and their position as a
## fraction of wheelbase behind the front axle.
const DRIVELINE_PARTS := {
	"front_diff": [30.0, 0.0],
	"rear_diff": [45.0, 1.0],
	"prop_shaft": [14.0, 0.6],
	"transfer": [12.0, 0.15],
}
const DRIVELINE_LAYOUT := {
	"FWD": ["front_diff"],
	"RWD": ["rear_diff", "prop_shaft"],
	"AWD": ["front_diff", "rear_diff", "prop_shaft", "transfer"],
}

const TUNE_DEFAULTS := {
	"boost_bar": null,            # null = engine stock boost
	"power_mult": 1.0,            # ECU tune / bolt-ons
	"rev_limit_rpm": null,
	"final_drive": null,
	"gear_ratios": null,
	"diff": {"type": "lsd", "power": 0.5, "coast": 0.3, "preload_nm": 60.0},
	"awd_front_split": 0.4,       # AWD only
	"center_lock_nm": 250.0,      # AWD only
	"springs_front_n_mm": null, "springs_rear_n_mm": null,
	"damp_bump_front": null, "damp_rebound_front": null,
	"damp_bump_rear": null, "damp_rebound_rear": null,
	"arb_front_n_mm": null, "arb_rear_n_mm": null,
	"ride_height_mm": 0.0,        # negative = lower
	"steer_lock_deg": null,       # angle kit
	"ackermann": 0.6,             # 1 = full Ackermann, 0 = parallel
	"brake_bias_front": null,
	"hydro_handbrake": false,
	"tire_front": {"compound": "sport", "size": null},
	"tire_rear": {"compound": "sport", "size": null},
	"weight_reduction_kg": 0.0,
}


static func merged_tune(build: Dictionary) -> Dictionary:
	var t := TUNE_DEFAULTS.duplicate(true)
	var bt: Variant = build.get("tune", {})
	if bt is Dictionary:
		for k: String in bt:
			if t.get(k) is Dictionary and bt[k] is Dictionary:
				(t[k] as Dictionary).merge(bt[k], true)
			else:
				t[k] = bt[k]
	return t


static func _opt(v: Variant, fallback: float) -> float:
	if v == null:
		return fallback
	return float(v)


## "225/45R17" -> radius in metres. Returns fallback if unparsable.
static func tire_radius(size: Variant, fallback: float) -> float:
	if not (size is String):
		return fallback
	var re := RegEx.create_from_string("(\\d{3})/(\\d{2})\\s*Z?R\\s*(\\d{2})")
	var m := re.search(size)
	if m == null:
		return fallback
	var w := m.get_string(1).to_float()
	var ar := m.get_string(2).to_float() / 100.0
	var rim := m.get_string(3).to_float() * 25.4
	return (rim / 2.0 + w * ar) / 1000.0


static func tire_width_mm(size: Variant, fallback: float) -> float:
	if size is String:
		var m := RegEx.create_from_string("(\\d{3})/").search(size)
		if m:
			return m.get_string(1).to_float()
	return fallback


static func _driveline_mass_moment(layout: String, wheelbase: float) -> Vector2:
	var mass := 0.0
	var moment := 0.0
	for part: String in DRIVELINE_LAYOUT.get(layout, []):
		var mp: Array = DRIVELINE_PARTS[part]
		mass += mp[0]
		moment += mp[0] * mp[1] * wheelbase
	return Vector2(mass, moment)


static func torque_at(curve_rpm: PackedFloat32Array, curve_nm: PackedFloat32Array, rpm: float) -> float:
	var n := curve_rpm.size()
	if n == 0:
		return 0.0
	if rpm <= curve_rpm[0]:
		return curve_nm[0] * clampf(rpm / maxf(curve_rpm[0], 1.0), 0.0, 1.0)
	for i in range(1, n):
		if rpm <= curve_rpm[i]:
			var t := (rpm - curve_rpm[i - 1]) / (curve_rpm[i] - curve_rpm[i - 1])
			return lerpf(curve_nm[i - 1], curve_nm[i], t)
	return curve_nm[n - 1]


## Returns {ok, spec, stats, warnings, errors}.
static func assemble(build: Dictionary, lib: PartsLibrary) -> Dictionary:
	var errors := PackedStringArray()
	var warnings := PackedStringArray()
	var ch := lib.get_part("chassis", str(build.get("chassis", "")))
	if ch.is_empty():
		errors.append("unknown chassis '%s'" % build.get("chassis", ""))
		return {"ok": false, "errors": errors, "warnings": warnings}
	var stock_engine := lib.get_part("engines", str(ch.get("stock_engine", "")))
	var eng_id: Variant = build.get("engine")
	var eng := stock_engine if (eng_id == null or str(eng_id) == "") else lib.get_part("engines", str(eng_id))
	if eng.is_empty():
		errors.append("unknown engine '%s'" % str(eng_id))
	var tr_id: Variant = build.get("transmission")
	var trans := lib.get_part("transmissions", str(ch.get("stock_transmission", ""))) if (tr_id == null or str(tr_id) == "") else lib.get_part("transmissions", str(tr_id))
	if trans.is_empty():
		errors.append("unknown transmission '%s'" % str(tr_id))
	if errors.size() > 0:
		return {"ok": false, "errors": errors, "warnings": warnings}

	var t := merged_tune(build)
	var stock_layout := str(ch.stock_drivetrain)
	var layout := str(build.get("drivetrain", stock_layout)) if build.get("drivetrain") != null else stock_layout
	if not DRIVELINE_LAYOUT.has(layout):
		errors.append("drivetrain must be RWD, FWD or AWD")
		return {"ok": false, "errors": errors, "warnings": warnings}

	var L := float(ch.wheelbase_m)
	# --- Mass & balance -----------------------------------------------------
	var mass := float(ch.mass_kg)
	var a := (1.0 - float(ch.weight_front)) * L # CoM distance behind front axle
	var moment := mass * a
	var eng_x := JsonUtil.num(ch, "engine_cg_from_front_axle_m", -0.1)
	var d_eng := float(eng.mass_kg) - JsonUtil.num(stock_engine, "mass_kg", float(eng.mass_kg))
	mass += d_eng
	moment += d_eng * eng_x
	var stock_dl := _driveline_mass_moment(stock_layout, L)
	var new_dl := _driveline_mass_moment(layout, L)
	mass += new_dl.x - stock_dl.x
	moment += new_dl.y - stock_dl.y
	var d_trans := JsonUtil.num(trans, "mass_kg", 50.0) - JsonUtil.num(lib.get_part("transmissions", str(ch.get("stock_transmission", ""))), "mass_kg", JsonUtil.num(trans, "mass_kg", 50.0))
	mass += d_trans
	moment += d_trans * (eng_x + 0.55)
	var wr := float(t.weight_reduction_kg)
	mass -= wr
	moment -= wr * (moment / maxf(mass + wr, 1.0))
	var com_from_front := moment / mass
	var weight_front := 1.0 - com_from_front / L
	var ride := float(t.ride_height_mm) / 1000.0
	var cg_h := float(ch.cg_height_m) + ride * 0.9

	# --- Tyres ---------------------------------------------------------------
	var base_r := float(ch.wheel_radius_m)
	var base_w := JsonUtil.num(ch, "tire_width_mm", 205.0)
	var tf: Dictionary = t.tire_front
	var trr: Dictionary = t.tire_rear
	var r_f := tire_radius(tf.get("size"), base_r)
	var r_r := tire_radius(trr.get("size"), base_r)
	var tire_f := _tire_params(tf, tire_width_mm(tf.get("size"), base_w))
	var tire_r := _tire_params(trr, tire_width_mm(trr.get("size"), base_w))

	# --- Engine --------------------------------------------------------------
	var rpm_arr := PackedFloat32Array()
	var nm_arr := PackedFloat32Array()
	for p: Array in eng.torque_curve:
		rpm_arr.append(float(p[0]))
		nm_arr.append(float(p[1]))
	var turbo := str(eng.get("aspiration", "na")) != "na"
	var boost_stock := JsonUtil.num(eng, "boost_bar", 0.0)
	var boost_target := _opt(t.boost_bar, boost_stock) if turbo else 0.0
	var boost_max := JsonUtil.num(eng, "max_boost_bar", boost_stock * 1.6)
	if turbo and boost_target > boost_max:
		warnings.append("Boost %.2f bar exceeds this turbo's practical max (%.2f) - clamped." % [boost_target, boost_max])
		boost_target = boost_max
	var limiter := _opt(t.rev_limit_rpm, float(eng.limiter_rpm))
	var safe_rpm := JsonUtil.num(eng, "max_safe_rpm", float(eng.limiter_rpm) + 500.0)
	if limiter > safe_rpm:
		warnings.append("Rev limit %d rpm is past the safe limit for this engine (%d) - clamped." % [limiter, safe_rpm])
		limiter = safe_rpm
	var power_mult := clampf(float(t.power_mult), 0.5, 2.0)

	# --- Transmission --------------------------------------------------------
	var gears: Array = t.gear_ratios if t.gear_ratios is Array else trans.gear_ratios
	var final := _opt(t.final_drive, float(trans.final_drive))
	var peak_nm := 0.0
	for i in nm_arr.size():
		peak_nm = maxf(peak_nm, nm_arr[i])
	var boost_factor := (1.0 + boost_target) / (1.0 + boost_stock) if turbo else 1.0
	var tuned_peak_nm := peak_nm * boost_factor * power_mult
	var trans_max := JsonUtil.num(trans, "max_torque_nm", 0.0)
	if trans_max > 0.0 and tuned_peak_nm > trans_max * 1.1:
		warnings.append("%s is rated ~%d Nm; this engine makes ~%d Nm. Expect it to break in real life." % [trans.name, trans_max, tuned_peak_nm])
	if layout != stock_layout:
		warnings.append(_conversion_note(stock_layout, layout, str(ch.get("make", ""))))

	# --- Suspension / steering -----------------------------------------------
	var susp: Dictionary = ch.get("suspension", {})
	var steer_lock := _opt(t.steer_lock_deg, JsonUtil.num(ch, "steer_lock_deg", 35.0))
	var steer_max := JsonUtil.num(ch, "max_steer_lock_deg", 68.0)
	if steer_lock > JsonUtil.num(ch, "steer_lock_deg", 35.0) + 3.0:
		warnings.append("%d° steering lock needs an angle kit (modified knuckles / extended lower arms)." % steer_lock)
	if steer_lock > steer_max:
		steer_lock = steer_max
	var diff: Dictionary = t.diff
	var diff_type := str(diff.get("type", "lsd"))

	var spec := {
		"name": str(build.get("name", ch.name)),
		"color": str(build.get("color", ch.get("color", "#c8c8c8"))),
		"body_style": str(build.get("body_style", ch.get("body_style", "sedan"))),
		"wing": bool(build.get("wing", false)),
		"mass": mass,
		"length": float(ch.length_m), "width": float(ch.width_m), "height": float(ch.height_m),
		"wheelbase": L,
		"track_f": float(ch.track_front_m), "track_r": float(ch.track_rear_m),
		"com_from_front": com_from_front,
		"cg_height": cg_h,
		"ride_offset": ride,
		"radius_f": r_f, "radius_r": r_r,
		"wheel_inertia_f": 1.4 * pow(r_f / 0.31, 2), "wheel_inertia_r": 1.4 * pow(r_r / 0.31, 2),
		"spring_f": _opt(t.springs_front_n_mm, JsonUtil.num(susp, "spring_front_n_mm", 35.0)) * 1000.0,
		"spring_r": _opt(t.springs_rear_n_mm, JsonUtil.num(susp, "spring_rear_n_mm", 30.0)) * 1000.0,
		"bump_f": _opt(t.damp_bump_front, JsonUtil.num(susp, "damp_bump_front", 2800.0)),
		"rebound_f": _opt(t.damp_rebound_front, JsonUtil.num(susp, "damp_rebound_front", 4500.0)),
		"bump_r": _opt(t.damp_bump_rear, JsonUtil.num(susp, "damp_bump_rear", 2500.0)),
		"rebound_r": _opt(t.damp_rebound_rear, JsonUtil.num(susp, "damp_rebound_rear", 4000.0)),
		"arb_f": _opt(t.arb_front_n_mm, JsonUtil.num(susp, "arb_front_n_mm", 20.0)) * 1000.0,
		"arb_r": _opt(t.arb_rear_n_mm, JsonUtil.num(susp, "arb_rear_n_mm", 10.0)) * 1000.0,
		"travel": JsonUtil.num(susp, "travel_m", 0.20) + (ride if ride > 0.0 else ride * 0.5),
		"tire_f": tire_f, "tire_r": tire_r,
		"steer_lock": deg_to_rad(steer_lock),
		"ackermann": float(t.ackermann),
		"brake_torque": JsonUtil.num(ch, "brake_torque_nm", 3200.0),
		"brake_bias": _opt(t.brake_bias_front, JsonUtil.num(ch, "brake_bias_front", 0.65)),
		"handbrake_torque": 3000.0 if bool(t.hydro_handbrake) else JsonUtil.num(ch, "handbrake_torque_nm", 1100.0),
		"drivetrain": layout,
		"awd_front_split": clampf(float(t.awd_front_split), 0.05, 0.95),
		"center_lock_nm": float(t.center_lock_nm),
		"diff_type": diff_type,
		"diff_power": clampf(JsonUtil.num(diff, "power", 0.5), 0.0, 1.0),
		"diff_coast": clampf(JsonUtil.num(diff, "coast", 0.3), 0.0, 1.0),
		"diff_preload": JsonUtil.num(diff, "preload_nm", 60.0),
		"curve_rpm": rpm_arr, "curve_nm": nm_arr,
		"idle_rpm": float(eng.idle_rpm),
		"limiter_rpm": limiter,
		"engine_inertia": JsonUtil.num(eng, "inertia", 0.18),
		"coast_nm": JsonUtil.num(eng, "coast_torque_nm", 0.0) if JsonUtil.num(eng, "coast_torque_nm", 0.0) > 0.0 else 0.12 * peak_nm,
		"limiter_hz": JsonUtil.num(eng, "limiter_hz", 0.0),
		"turbo": turbo,
		"boost_stock": boost_stock, "boost_target": boost_target,
		"spool_rpm": JsonUtil.num(eng, "spool_rpm", 3200.0),
		"turbo_lag_s": JsonUtil.num(eng, "turbo_lag_s", 0.45),
		"power_mult": power_mult,
		"gears": gears.map(func(g: Variant) -> float: return float(g)),
		"reverse": JsonUtil.num(trans, "reverse_ratio", 3.4),
		"final": final,
		"shift_time": JsonUtil.num(trans, "shift_time_s", 0.18),
		"clutch_nm": maxf(JsonUtil.num(trans, "clutch_max_torque_nm", 0.0), tuned_peak_nm * 1.4),
		"cda": JsonUtil.num(ch, "cd", 0.34) * JsonUtil.num(ch, "frontal_area_m2", 2.1),
		"engine_name": str(eng.name),
		"cylinders": _cylinders(eng),
		"boxer": str(eng.get("layout", "")).to_lower().begins_with("flat") or str(eng.name).to_lower().contains("subaru ej"),
		"transmission_name": str(trans.name),
		"chassis_name": str(ch.name),
		"driver_side": str(ch.get("driver_side", "left")),
	}
	var stats := compute_stats(spec)
	if stats.hp_per_tonne > 330.0:
		warnings.append("%.0f hp/tonne is a lot - expect it to be a handful." % stats.hp_per_tonne)
	return {"ok": true, "spec": spec, "stats": stats, "warnings": warnings, "errors": errors}


## Assembled spec as JSON for DriftCore (the Unreal physics): {"id", "spec"}.
static func spec_json(spec: Dictionary, id: String) -> String:
	var clean := {}
	for k: String in spec:
		var v: Variant = spec[k]
		if v is PackedFloat32Array or v is PackedFloat64Array:
			v = Array(v)
		clean[k] = v
	return JSON.stringify({"id": id, "format": "drift_core_car_1", "spec": clean}, "\t")


static func _cylinders(eng: Dictionary) -> int:
	if eng.has("cylinders"):
		return int(eng.cylinders)
	var n := str(eng.get("name", "")).to_upper() + " " + str(eng.get("layout", "")).to_upper()
	for pair in [["V12", 12], ["V10", 10], ["V8", 8], ["LS3", 8], ["V6", 6], ["I6", 6], ["2JZ", 6], ["1JZ", 6], ["RB2", 6], ["M52", 6], ["FLAT6", 6], ["ROTARY", 2], ["13B", 2], ["I3", 3]]:
		if n.contains(pair[0]):
			return pair[1]
	return 4


static func _tire_params(tdef: Dictionary, width_mm: float) -> Dictionary:
	var compound := str(tdef.get("compound", "sport"))
	var p: Dictionary = TIRE_COMPOUNDS.get(compound, TIRE_COMPOUNDS.sport).duplicate()
	# Wider tyres: a little more grip (weak effect, like real life).
	var wf := pow(width_mm / 215.0, 0.18)
	p.mu_long *= wf
	p.mu_lat *= wf
	for k in ["mu_long", "mu_lat", "peak_slip_angle_deg", "peak_slip_ratio", "tail", "load_sens"]:
		if tdef.has(k) and tdef[k] != null:
			p[k] = float(tdef[k]) * (wf if k.begins_with("mu") else 1.0)
	var gm := JsonUtil.num(tdef, "grip_mult", 1.0)
	p.mu_long *= gm
	p.mu_lat *= gm
	p["width_mm"] = width_mm
	return p


static func _conversion_note(from: String, to: String, make: String) -> String:
	if from == "AWD" and to == "RWD":
		if make.to_lower().contains("subaru"):
			return "AWD->RWD on a Subaru: in real life this means locking/welding the centre diff or a spool coupler and pulling the front axles (otherwise the open front diff takes all the torque). Modelled as front axles removed."
		return "AWD->RWD conversion: front axles removed, transfer case locked to the rear."
	if to == "AWD":
		return "Converting to AWD needs a transfer case, front diff and axles - a major fabrication job."
	if from == "FWD" and to == "RWD":
		return "FWD->RWD needs a longitudinal engine swap or a custom rear subframe - a full fabrication build."
	return "Drivetrain changed from %s to %s." % [from, to]


## Power figures and a quick 1-D acceleration run (0-100 km/h, top speed).
static func compute_stats(spec: Dictionary) -> Dictionary:
	var rpm: PackedFloat32Array = spec.curve_rpm
	var nm: PackedFloat32Array = spec.curve_nm
	var bf := (1.0 + float(spec.boost_target)) / (1.0 + float(spec.boost_stock)) if spec.turbo else 1.0
	var pm := float(spec.power_mult)
	var peak_hp := 0.0
	var peak_hp_rpm := 0.0
	var peak_nm := 0.0
	var peak_nm_rpm := 0.0
	var r := float(spec.idle_rpm)
	while r <= float(spec.limiter_rpm):
		var tq := torque_at(rpm, nm, r) * bf * pm
		var hp := tq * r * TAU / 60.0 / 745.7
		if hp > peak_hp:
			peak_hp = hp
			peak_hp_rpm = r
		if tq > peak_nm:
			peak_nm = tq
			peak_nm_rpm = r
		r += 50.0
	var mass := float(spec.mass)
	var gears: Array = spec.gears
	var final := float(spec.final)
	var rw := float(spec.radius_r) if spec.drivetrain != "FWD" else float(spec.radius_f)
	var gear_top_speeds := []
	for g: float in gears:
		gear_top_speeds.append(snappedf(float(spec.limiter_rpm) / 60.0 * TAU / (g * final) * rw * 3.6, 0.1))
	# 1-D launch sim.
	var driven_frac: float = {"RWD": float(spec.com_from_front) / float(spec.wheelbase), "FWD": 1.0 - float(spec.com_from_front) / float(spec.wheelbase), "AWD": 1.0}[spec.drivetrain]
	var mu := float((spec.tire_r if spec.drivetrain != "FWD" else spec.tire_f).mu_long)
	var v := 0.0
	var tsim := 0.0
	var gi := 0
	var t100 := -1.0
	var shift := 0.0
	var dt := 0.01
	var top := 0.0
	while tsim < 90.0:
		var g := float(gears[gi])
		var eng_rpm := maxf(v / rw * g * final * 60.0 / TAU, float(spec.idle_rpm) * 2.0)
		if eng_rpm >= float(spec.limiter_rpm) * 0.98 and gi < gears.size() - 1:
			gi += 1
			shift = float(spec.shift_time)
			continue
		var f_drive := 0.0
		if shift <= 0.0 and eng_rpm < float(spec.limiter_rpm):  # fuel cut at the limiter
			f_drive = torque_at(rpm, nm, eng_rpm) * bf * pm * g * final * 0.88 / rw
			f_drive = minf(f_drive, mu * mass * 9.81 * driven_frac * 1.05)
		shift -= dt
		var f_drag := 0.5 * 1.2 * float(spec.cda) * v * v + 0.012 * mass * 9.81
		var acc := (f_drive - f_drag) / mass
		v += acc * dt
		tsim += dt
		if t100 < 0.0 and v >= 100.0 / 3.6:
			t100 = tsim
		top = maxf(top, v)
		if acc < 0.02 and shift <= 0.0 and tsim > 20.0:
			break
	return {
		"mass_kg": snappedf(mass, 1.0),
		"weight_front_pct": snappedf((1.0 - float(spec.com_from_front) / float(spec.wheelbase)) * 100.0, 0.1),
		"peak_hp": snappedf(peak_hp, 1.0), "peak_hp_rpm": peak_hp_rpm,
		"peak_torque_nm": snappedf(peak_nm, 1.0), "peak_torque_rpm": peak_nm_rpm,
		"hp_per_tonne": snappedf(peak_hp / mass * 1000.0, 1.0),
		"zero_to_100_s": snappedf(t100, 0.01),
		"top_speed_kmh": snappedf(top * 3.6, 1.0),
		"gear_top_speeds_kmh": gear_top_speeds,
		"steer_lock_deg": snappedf(rad_to_deg(float(spec.steer_lock)), 1.0),
		"drivetrain": spec.drivetrain,
	}
