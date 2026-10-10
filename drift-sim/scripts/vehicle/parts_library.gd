class_name PartsLibrary
extends RefCounted
## Loads modular car parts (chassis / engines / transmissions) from the
## built-in data folder and the user's folder (user parts override built-ins).

const KINDS := ["chassis", "engines", "transmissions"]

var parts := {"chassis": {}, "engines": {}, "transmissions": {}}


func reload() -> void:
	for kind: String in KINDS:
		parts[kind] = {}
		for base in ["res://data/parts", "user://parts"]:
			for path in JsonUtil.list_json(base.path_join(kind)):
				var d: Variant = JsonUtil.load_file(path)
				if d is Dictionary and d.has("id"):
					d["_path"] = path
					parts[kind][d.id] = d


func get_part(kind: String, id: String) -> Dictionary:
	return parts.get(kind, {}).get(id, {})


func ids(kind: String) -> Array:
	var a: Array = parts.get(kind, {}).keys()
	a.sort()
	return a


func summary(kind: String) -> Array:
	var out := []
	for id in ids(kind):
		var p: Dictionary = parts[kind][id]
		out.append({"id": id, "name": p.get("name", id)})
	return out


## Validates then saves a part to the user folder. Returns error strings.
func save_user_part(kind: String, part: Dictionary) -> PackedStringArray:
	var errs := validate_part(kind, part)
	if errs.size() > 0:
		return errs
	var clean := part.duplicate(true)
	clean.erase("_path")
	clean["id"] = JsonUtil.slugify(str(part.id))
	JsonUtil.save_file("user://parts".path_join(kind).path_join(clean.id + ".json"), clean)
	reload()
	return errs


static func _need(errs: PackedStringArray, d: Dictionary, key: String, lo: float, hi: float) -> void:
	if not d.has(key):
		errs.append("missing '%s'" % key)
		return
	var v := JsonUtil.num(d, key, NAN)
	if is_nan(v) or v < lo or v > hi:
		errs.append("'%s'=%s out of range [%s, %s]" % [key, str(d[key]), lo, hi])


static func validate_part(kind: String, p: Dictionary) -> PackedStringArray:
	var e := PackedStringArray()
	if str(p.get("id", "")) == "":
		e.append("missing 'id'")
	if str(p.get("name", "")) == "":
		e.append("missing 'name'")
	match kind:
		"chassis":
			_need(e, p, "mass_kg", 500, 3500)
			_need(e, p, "wheelbase_m", 1.8, 4.0)
			_need(e, p, "track_front_m", 1.1, 2.0)
			_need(e, p, "track_rear_m", 1.1, 2.0)
			_need(e, p, "length_m", 2.5, 6.5)
			_need(e, p, "width_m", 1.4, 2.3)
			_need(e, p, "height_m", 1.0, 2.2)
			_need(e, p, "cg_height_m", 0.3, 1.0)
			_need(e, p, "weight_front", 0.35, 0.70)
			_need(e, p, "wheel_radius_m", 0.25, 0.45)
			if not ["RWD", "FWD", "AWD"].has(str(p.get("stock_drivetrain", ""))):
				e.append("'stock_drivetrain' must be RWD, FWD or AWD")
		"engines":
			_need(e, p, "mass_kg", 40, 400)
			_need(e, p, "idle_rpm", 400, 2000)
			_need(e, p, "limiter_rpm", 3000, 12000)
			var curve: Variant = p.get("torque_curve", [])
			if not (curve is Array) or (curve as Array).size() < 3:
				e.append("'torque_curve' needs >= 3 [rpm, Nm] points")
			else:
				var last := -1.0
				for pt: Variant in curve:
					if not (pt is Array) or (pt as Array).size() < 2:
						e.append("torque_curve entries must be [rpm, Nm]")
						break
					if float(pt[0]) <= last:
						e.append("torque_curve rpm must increase")
						break
					last = float(pt[0])
					if float(pt[1]) < 0 or float(pt[1]) > 3000:
						e.append("torque value %s Nm out of range" % pt[1])
						break
		"transmissions":
			var g: Variant = p.get("gear_ratios", [])
			if not (g is Array) or (g as Array).size() < 3 or (g as Array).size() > 10:
				e.append("'gear_ratios' must have 3-10 ratios")
			_need(e, p, "final_drive", 2.0, 7.0)
		_:
			e.append("unknown part kind '%s'" % kind)
	return e
