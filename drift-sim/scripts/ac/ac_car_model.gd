class_name AcCarModel
extends RefCounted
## Uses an Assetto Corsa car model (from the player's own AC install) as the
## car's visuals: measures wheel positions so the physics matches the model,
## moves the real wheels/hubs onto the simulated wheels, turns the real
## steering wheel, and puts the first-person eye behind it.

const WHEELS := ["LF", "RF", "LR", "RR"]  # same order as Vehicle.wheels


static func _rel(n: Node3D, root: Node) -> Transform3D:
	var t := n.transform
	var p := n.get_parent()
	while p != null and p != root:
		if p is Node3D:
			t = (p as Node3D).transform * t
		p = p.get_parent()
	return t


## Loads and measures the model. Returns {} if it can't be used.
static func load_for(ac_model: Variant) -> Dictionary:
	var where := AcPaths.resolve(ac_model)
	if where.is_empty():
		return {}
	var r := Kn5Loader.load_model(where.kn5, where.skin_dir)
	if not r.ok:
		push_warning("AC model: " + str(r.error))
		return {}
	var root: Node3D = r.root
	var named: Dictionary = r.named
	var centres := {}
	for w in WHEELS:
		var n: Variant = named.get("WHEEL_" + w)
		if not (n is Node3D):
			root.free()
			return {}
		centres[w] = _rel(n, root).origin
	var front_z: float = (centres.LF.z + centres.RF.z) * 0.5
	var rear_z: float = (centres.LR.z + centres.RR.z) * 0.5
	var wheel_y: float = (centres.LF.y + centres.RF.y + centres.LR.y + centres.RR.y) * 0.25
	var geo := {
		"wheelbase": absf(rear_z - front_z),
		"track_f": absf(centres.RF.x - centres.LF.x),
		"track_r": absf(centres.RR.x - centres.LR.x),
		"radius_f": (centres.LF.y + centres.RF.y) * 0.5,
		"radius_r": (centres.LR.y + centres.RR.y) * 0.5,
		# Body frame: origin at axle height midway between the axles.
		"offset": Vector3(0, -wheel_y, -(front_z + rear_z) * 0.5),
	}
	var steer: Variant = named.get("STEER_HR")
	if steer is Node3D:
		geo["steer_pos"] = _rel(steer, root).origin + geo.offset
	return {"root": root, "named": named, "geo": geo, "centres": centres, "skin": where.skin, "car_dir": where.car_dir}


## Makes the physics match the model (wheel positions/sizes, seat).
static func apply_geometry(spec: Dictionary, model: Dictionary) -> void:
	var g: Dictionary = model.geo
	if g.wheelbase > 1.6 and g.wheelbase < 4.0:
		spec.wheelbase = g.wheelbase
	if g.track_f > 1.0 and g.track_f < 2.2:
		spec.track_f = g.track_f
	if g.track_r > 1.0 and g.track_r < 2.2:
		spec.track_r = g.track_r
	if g.radius_f > 0.2 and g.radius_f < 0.5:
		spec.radius_f = g.radius_f
		spec.radius_r = g.radius_r
	if g.has("steer_pos"):
		# Eyes ~27 cm above and ~50 cm behind the steering wheel hub.
		spec["eye"] = (g.steer_pos as Vector3) + Vector3(0, 0.27, 0.5)
		spec["driver_side"] = "left" if (g.steer_pos as Vector3).x < 0.0 else "right"


## Attaches the model to a CarVisual, moving wheels/hubs onto the sim wheels.
static func attach(cv: CarVisual, model: Dictionary) -> void:
	var root: Node3D = model.root
	var named: Dictionary = model.named
	var offset: Vector3 = model.geo.offset
	root.position = offset
	cv.add_child(root)
	for i in WHEELS.size():
		var w: String = WHEELS[i]
		var pivot: Node3D = cv.wheel_nodes[i]
		var spinner: Node3D = pivot.get_node("spin")
		for c in spinner.get_children():
			c.queue_free()  # procedural tyre/rim
		var wheel: Node3D = named["WHEEL_" + w]
		var wt := _rel(wheel, root)
		wheel.get_parent().remove_child(wheel)
		wheel.transform = Transform3D(wt.basis, Vector3.ZERO)
		spinner.add_child(wheel)
		var susp: Variant = named.get("SUSP_" + w)
		if susp is Node3D and (susp as Node3D).is_inside_tree() == false:
			var st := _rel(susp, root)
			(susp as Node3D).get_parent().remove_child(susp)
			(susp as Node3D).transform = Transform3D(st.basis, st.origin - wt.origin)
			pivot.add_child(susp)
	var steer: Variant = named.get("STEER_HR")
	if steer is Node3D:
		cv.ac_steer = steer
		cv.ac_steer_base = (steer as Node3D).transform
