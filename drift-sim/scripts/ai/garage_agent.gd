class_name GarageAgent
extends ClaudeClient
## "Build me a '98 Outback with a WRX swap, RWD, drift tune" -> a saved build.
## Claude researches specs (web search, NHTSA VIN decoder), creates missing
## parts in the parts library, assembles + evaluates the build with the same
## code the game uses, iterates, then saves it.

const SYSTEM := """You are the build engineer in Drift Lab, a drift driving simulator with modular cars.
A car = chassis + engine + transmission + drivetrain layout + tune. Parts live in a library (JSON). You can read parts, create new ones, assemble/evaluate builds with the game's own physics code, and save builds.

How to work:
1. Check the library (list_parts) for the requested chassis/engine/gearbox before creating anything.
2. If a part is missing, research it (web_search, and decode_vin when the user gives a VIN) and create it with save_part using real published specs: curb weight, wheelbase, track widths, dimensions, weight distribution, stock tyre size, engine torque curve, redline, gearbox ratios, final drive. Estimate what isn't published (CG height, spring/ARB rates, damping, inertia) from similar cars and say so in the part's "notes".
3. Assemble with evaluate_build, read the stats and warnings, and adjust until the build is realistic and drivable for what the user asked (e.g. a drift build wants RWD or a locked centre, a locking diff, extra steering lock, a hydraulic handbrake, sensible springs, appropriate gearing). Mirror how the build would really be done; mention real-world caveats (e.g. gearbox torque limits, conversion methods).
4. save_build once you're happy. Then reply with a short summary: what the build is, key numbers, and the real-world notes. Keep it brief.

Units & formats:
- chassis: {id, name, make, year, body_style (sedan|wagon|coupe|hatch|roadster), color, mass_kg (curb, with stock engine), stock_drivetrain (RWD|FWD|AWD), stock_engine (engine id), stock_transmission (transmission id), wheelbase_m, track_front_m, track_rear_m, length_m, width_m, height_m, cg_height_m, weight_front (0-1), engine_cg_from_front_axle_m (negative = engine ahead of the front axle), wheel_radius_m, tire_width_mm, stock_tires, steer_lock_deg (stock road-wheel lock), max_steer_lock_deg (with an angle kit), brake_torque_nm (total), brake_bias_front, handbrake_torque_nm, cd, frontal_area_m2, suspension: {spring_front_n_mm, spring_rear_n_mm (wheel rates), damp_bump_front, damp_rebound_front, damp_bump_rear, damp_rebound_rear (N*s/m), arb_front_n_mm, arb_rear_n_mm, travel_m}, notes}
- engine: {id, name, mass_kg (dressed), idle_rpm, limiter_rpm, max_safe_rpm, aspiration (na|turbo|supercharged), boost_bar (stock), max_boost_bar (practical for the stock turbo), spool_rpm, turbo_lag_s, inertia (kg*m^2, ~0.1-0.3), torque_curve: [[rpm, Nm], ...] at the crank at stock boost (>= 6 points, idle to limiter), notes}
- transmission: {id, name, gear_ratios: [...], final_drive, reverse_ratio, max_torque_nm (rough real-world limit), mass_kg, shift_time_s, notes}
- build: {id, name, chassis, engine (null = stock), transmission (null = stock), drivetrain (null = stock, or RWD|FWD|AWD), color "#rrggbb", notes, tune: {boost_bar, power_mult (ECU/bolt-ons, 1.0 = stock), rev_limit_rpm, final_drive, gear_ratios, diff: {type: open|lsd|welded, power 0-1, coast 0-1, preload_nm}, awd_front_split, center_lock_nm, springs_front_n_mm, springs_rear_n_mm, damp_bump_front, damp_rebound_front, damp_bump_rear, damp_rebound_rear, arb_front_n_mm, arb_rear_n_mm, ride_height_mm (negative = lower), steer_lock_deg, ackermann (0-1), brake_bias_front, hydro_handbrake (bool), weight_reduction_kg, tire_front: {compound: economy|street|sport|drift|semi_slick, size: "235/40R17", grip_mult}, tire_rear: {...}}}
Omit tune keys you don't want to change (stock values are used). Use lowercase snake_case ids like "subaru_ej205_wrx"."""

var _vin_cache := {}
var saved_build: Dictionary = {}
var saved_path := ""


func _tools() -> Array:
	return [
		{"name": "list_parts", "description": "List part ids and names in the library for a kind.",
			"input_schema": {"type": "object", "properties": {"kind": {"type": "string", "enum": ["chassis", "engines", "transmissions"]}}, "required": ["kind"]}},
		{"name": "get_part", "description": "Get the full JSON of one part.",
			"input_schema": {"type": "object", "properties": {"kind": {"type": "string", "enum": ["chassis", "engines", "transmissions"]}, "id": {"type": "string"}}, "required": ["kind", "id"]}},
		{"name": "save_part", "description": "Validate and save a new or updated part to the user's library. Returns validation errors if any.",
			"input_schema": {"type": "object", "properties": {"kind": {"type": "string", "enum": ["chassis", "engines", "transmissions"]}, "part": {"type": "object"}}, "required": ["kind", "part"]}},
		{"name": "evaluate_build", "description": "Assemble a build with the game's physics code and return computed stats (hp, torque, mass, weight distribution, 0-100, gear speeds) and warnings. Does not save.",
			"input_schema": {"type": "object", "properties": {"build": {"type": "object"}}, "required": ["build"]}},
		{"name": "save_build", "description": "Save the final build so the player can drive it. Call once at the end.",
			"input_schema": {"type": "object", "properties": {"build": {"type": "object"}}, "required": ["build"]}},
		{"name": "decode_vin", "description": "Decode a 17-character VIN with the US NHTSA vPIC database (make, model, year, trim, engine, drivetrain, body).",
			"input_schema": {"type": "object", "properties": {"vin": {"type": "string"}}, "required": ["vin"]}},
		{"type": "web_search_20260209", "name": "web_search", "max_uses": 8},
	]


## existing: optional current build to modify. Returns {build, path, text, error}.
func run(request: String, existing: Dictionary = {}) -> Dictionary:
	var lib: PartsLibrary = get_node("/root/App").parts
	var user := request
	if not existing.is_empty():
		var clean := existing.duplicate(true)
		user = "Here is my current build:\n```json\n%s\n```\n\nChange it like this: %s" % [JSON.stringify(clean, " "), request]
	user += "\n\n(Library summary - chassis: %s; engines: %s; transmissions: %s)" % [
		", ".join(lib.ids("chassis")), ", ".join(lib.ids("engines")), ", ".join(lib.ids("transmissions"))]
	var messages := [{"role": "user", "content": user}]
	var r := await run_agent(SYSTEM, messages, _tools(), _handle)
	if str(r.text).strip_edges() != "":
		progress.emit("[b]Claude:[/b] " + str(r.text).strip_edges())
	return {"build": saved_build if not saved_build.is_empty() else null, "path": saved_path, "text": r.text, "error": r.error}


func _handle(name: String, input: Dictionary) -> Dictionary:
	var lib: PartsLibrary = get_node("/root/App").parts
	match name:
		"list_parts":
			return {"content": JSON.stringify(lib.summary(str(input.get("kind", ""))))}
		"get_part":
			var p := lib.get_part(str(input.get("kind", "")), str(input.get("id", "")))
			if p.is_empty():
				return {"content": "No such part.", "is_error": true}
			var c := p.duplicate()
			c.erase("_path")
			return {"content": JSON.stringify(c)}
		"save_part":
			if not (input.get("part") is Dictionary):
				return {"content": "'part' must be an object", "is_error": true}
			var errs := lib.save_user_part(str(input.get("kind", "")), input.part)
			if errs.size() > 0:
				return {"content": "Validation failed:\n- " + "\n- ".join(errs), "is_error": true}
			progress.emit("[color=#66ff88]+ saved %s: %s[/color]" % [input.kind, input.part.get("name", "")])
			return {"content": "Saved %s '%s'." % [input.kind, JsonUtil.slugify(str(input.part.id))]}
		"evaluate_build", "save_build":
			if not (input.get("build") is Dictionary):
				return {"content": "'build' must be an object", "is_error": true}
			var b: Dictionary = input.build
			var res := CarBuilder.assemble(b, lib)
			if not res.ok:
				return {"content": "Build errors:\n- " + "\n- ".join(res.errors), "is_error": true}
			if name == "save_build":
				var id := JsonUtil.slugify(str(b.get("id", b.get("name", "ai_build"))))
				b["id"] = id
				saved_path = "user://cars/%s.json" % id
				JsonUtil.save_file(saved_path, b)
				saved_build = b
				progress.emit("[color=#66ff88]✔ build saved: %s[/color]" % b.get("name", id))
				return {"content": "Saved. Stats: " + JSON.stringify(res.stats)}
			var s: Dictionary = res.stats
			progress.emit("   %d hp · %d Nm · %d kg · %.0f%% front · %s" % [s.peak_hp, s.peak_torque_nm, s.mass_kg, s.weight_front_pct, s.drivetrain])
			return {"content": JSON.stringify({"stats": res.stats, "warnings": res.warnings})}
		"decode_vin":
			return await _decode_vin(str(input.get("vin", "")))
	return {"content": "Unknown tool " + name, "is_error": true}


func _decode_vin(vin: String) -> Dictionary:
	vin = vin.strip_edges().to_upper()
	if vin.length() != 17:
		return {"content": "VIN must be 17 characters.", "is_error": true}
	if _vin_cache.has(vin):
		return _vin_cache[vin]
	var http := HTTPRequest.new()
	http.timeout = 30.0
	add_child(http)
	http.request("https://vpic.nhtsa.dot.gov/api/vehicles/DecodeVinValues/%s?format=json" % vin.uri_encode())
	var res: Array = await http.request_completed
	http.queue_free()
	if res[0] != HTTPRequest.RESULT_SUCCESS or res[1] != 200:
		return {"content": "VIN lookup failed (network).", "is_error": true}
	var data: Variant = JSON.parse_string((res[3] as PackedByteArray).get_string_from_utf8())
	if not (data is Dictionary) or (data.get("Results", []) as Array).is_empty():
		return {"content": "VIN lookup returned nothing.", "is_error": true}
	var row: Dictionary = data.Results[0]
	var keep := {}
	for k: String in row:
		var v := str(row[k])
		if v != "" and v != "Not Applicable" and v != "0":
			keep[k] = v
	var out := {"content": JSON.stringify(keep)}
	_vin_cache[vin] = out
	progress.emit("[color=#88aaff]VIN: %s %s %s[/color]" % [keep.get("ModelYear", ""), keep.get("Make", ""), keep.get("Model", "")])
	return out
