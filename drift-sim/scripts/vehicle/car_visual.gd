class_name CarVisual
extends Node3D
## Procedural low-poly body built from the chassis dimensions and body style,
## plus wheel meshes that follow the simulated suspension and wheel spin.

var vehicle: Vehicle
var wheel_nodes: Array[Node3D] = []
var brake_lights: StandardMaterial3D
var steering_wheel: Node3D
var gauge: Label3D
## Steering wheel turns per road-wheel degree (set from the wheel rotation).
var steer_ratio := 8.0


static func create(v: Vehicle) -> CarVisual:
	var cv := CarVisual.new()
	cv.vehicle = v
	cv._build()
	return cv


func _mat(color: Color, metallic := 0.0, rough := 0.5) -> StandardMaterial3D:
	var m := StandardMaterial3D.new()
	m.albedo_color = color
	m.metallic = metallic
	m.roughness = rough
	return m


func _box(size: Vector3, pos: Vector3, mat: Material) -> MeshInstance3D:
	var mi := MeshInstance3D.new()
	var bm := BoxMesh.new()
	bm.size = size
	mi.mesh = bm
	mi.position = pos
	mi.material_override = mat
	add_child(mi)
	return mi


func _build() -> void:
	var s := vehicle.spec
	var w := float(s.width)
	var h := float(s.height)
	var l := float(s.length)
	var rmean := (float(s.radius_f) + float(s.radius_r)) * 0.5
	var ground := -rmean
	var clearance := maxf(0.14 + float(s.ride_offset) * 0.6, 0.07)
	var paint := _mat(Color.from_string(str(s.color), Color(0.7, 0.7, 0.7)), 0.4, 0.35)
	var glass := _mat(Color(0.1, 0.13, 0.17, 0.28), 0.2, 0.05)
	glass.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	var dark := _mat(Color(0.05, 0.05, 0.05), 0.0, 0.8)
	var style := str(s.body_style)
	var lower_h := h * 0.42
	var base_y := ground + clearance + lower_h * 0.5
	_box(Vector3(w, lower_h, l), Vector3(0, base_y, 0), paint)
	# Cabin: wagon roof runs to the rear, coupe is short, roadster has none.
	var cabin_h := h - clearance - lower_h
	var cab_len := l * 0.48
	var cab_z := l * 0.04
	match style:
		"wagon":
			cab_len = l * 0.66
			cab_z = l * 0.14
		"hatch":
			cab_len = l * 0.55
			cab_z = l * 0.12
		"coupe":
			cab_len = l * 0.42
			cab_z = l * 0.06
		"roadster":
			cabin_h *= 0.35
			cab_len = l * 0.22
	var cab_y := base_y + lower_h * 0.5 + cabin_h * 0.5
	if style == "roadster":
		_box(Vector3(w * 0.86, cabin_h, cab_len), Vector3(0, cab_y, cab_z), glass)
	else:
		# Painted roof + pillars with glass inset on every side.
		var cw := w * 0.86
		_box(Vector3(cw, cabin_h * 0.98, cab_len), Vector3(0, cab_y, cab_z), paint)
		var gh := cabin_h * 0.62
		var gy := cab_y - cabin_h * 0.02
		_box(Vector3(cw + 0.02, gh, cab_len * 0.82), Vector3(0, gy, cab_z), glass)
		_box(Vector3(cw * 0.88, gh, cab_len + 0.02), Vector3(0, gy, cab_z), glass)
		# Hood and rear deck/hatch slope hint.
		_box(Vector3(w * 0.9, 0.04, l * 0.02), Vector3(0, base_y + lower_h * 0.5, cab_z - cab_len * 0.5 - l * 0.01), dark)
	if style == "wagon":
		# Roof rails - every Outback has them.
		for side: float in [-1.0, 1.0]:
			_box(Vector3(0.04, 0.05, cab_len * 0.85), Vector3(side * w * 0.38, cab_y + cabin_h * 0.5 + 0.05, cab_z), dark)
	if style != "roadster":
		_build_interior(w, base_y + lower_h * 0.5, cab_y + cabin_h * 0.5, cab_z, cab_len)
	else:
		_build_interior(w, base_y + lower_h * 0.5, -1.0, cab_z, l * 0.4)
	# Bumpers, lights.
	_box(Vector3(w * 1.01, lower_h * 0.35, 0.12), Vector3(0, base_y - lower_h * 0.25, -l * 0.5), dark)
	_box(Vector3(w * 1.01, lower_h * 0.35, 0.12), Vector3(0, base_y - lower_h * 0.25, l * 0.5), dark)
	var head := _mat(Color(1, 1, 0.9))
	head.emission_enabled = true
	head.emission = Color(1, 1, 0.85)
	brake_lights = _mat(Color(0.5, 0, 0))
	brake_lights.emission_enabled = true
	brake_lights.emission = Color(1, 0, 0)
	brake_lights.emission_energy_multiplier = 0.3
	for side: float in [-1.0, 1.0]:
		_box(Vector3(w * 0.2, 0.08, 0.04), Vector3(side * w * 0.33, base_y + lower_h * 0.2, -l * 0.5 - 0.01), head)
		_box(Vector3(w * 0.2, 0.08, 0.04), Vector3(side * w * 0.33, base_y + lower_h * 0.2, l * 0.5 + 0.01), brake_lights)
	# Wheels.
	var tire_mat := _mat(Color(0.06, 0.06, 0.06), 0.0, 0.9)
	var rim_mat := _mat(Color(0.75, 0.75, 0.78), 0.8, 0.3)
	for wh in vehicle.wheels:
		var pivot := Node3D.new()
		add_child(pivot)
		var spinner := Node3D.new()
		spinner.name = "spin"
		pivot.add_child(spinner)
		var tire := MeshInstance3D.new()
		var cyl := CylinderMesh.new()
		cyl.top_radius = wh.radius
		cyl.bottom_radius = wh.radius
		var tw := float((s.tire_f if wh.front else s.tire_r).get("width_mm", 205.0)) / 1000.0
		cyl.height = tw
		cyl.radial_segments = 20
		tire.mesh = cyl
		tire.rotation = Vector3(0, 0, PI / 2)
		tire.material_override = tire_mat
		spinner.add_child(tire)
		var rim := MeshInstance3D.new()
		var rb := BoxMesh.new()
		rb.size = Vector3(tw + 0.01, wh.radius * 1.2, 0.08)
		rim.mesh = rb
		rim.material_override = rim_mat
		spinner.add_child(rim)
		wheel_nodes.append(pivot)


## A box spanning from a to b (for pillars, rails).
func _beam(a: Vector3, b: Vector3, t: float, mat: Material) -> void:
	var mi := MeshInstance3D.new()
	var bm := BoxMesh.new()
	bm.size = Vector3(t, t, a.distance_to(b))
	mi.mesh = bm
	mi.material_override = mat
	add_child(mi)
	var mid := (a + b) * 0.5
	var up := Vector3.UP if absf((b - a).normalized().dot(Vector3.UP)) < 0.95 else Vector3.RIGHT
	mi.transform = Transform3D(Basis.looking_at(b - a, up), mid)


func _build_interior(w: float, belt_y: float, roof_y: float, cab_z: float, cab_len: float) -> void:
	var trim := _mat(Color(0.09, 0.09, 0.1), 0.0, 0.85)
	trim.cull_mode = BaseMaterial3D.CULL_DISABLED
	var liner := _mat(Color(0.55, 0.55, 0.52), 0.0, 0.95)
	liner.cull_mode = BaseMaterial3D.CULL_DISABLED
	var eye := vehicle.seat_local()
	var cw := w * 0.84
	var front := cab_z - cab_len * 0.5
	var rear := cab_z + cab_len * 0.5
	var dash_top := eye.y - 0.33
	# Dashboard.
	_box(Vector3(cw, 0.24, 0.5), Vector3(0, dash_top - 0.12, front + 0.25), trim)
	_box(Vector3(cw * 0.9, 0.05, 0.12), Vector3(0, dash_top + 0.02, front + 0.47), trim)
	# Door cards / beltline rails.
	for side: float in [-1.0, 1.0]:
		_beam(Vector3(side * cw * 0.5, belt_y + 0.04, front + 0.1), Vector3(side * cw * 0.5, belt_y + 0.04, rear - 0.1), 0.08, trim)
	if roof_y > 0.0:
		_box(Vector3(cw * 0.96, 0.02, cab_len * 0.92), Vector3(0, roof_y - 0.05, cab_z), liner)
		for side: float in [-1.0, 1.0]:
			var x := side * cw * 0.5
			_beam(Vector3(x, dash_top, front + 0.12), Vector3(x * 0.96, roof_y - 0.04, front + 0.62), 0.075, trim)  # A
			_beam(Vector3(x, belt_y, eye.z + 0.28), Vector3(x * 0.96, roof_y - 0.04, eye.z + 0.28), 0.09, trim)   # B
			_beam(Vector3(x, belt_y, rear - 0.12), Vector3(x * 0.96, roof_y - 0.04, rear - 0.25), 0.11, trim)      # C/D
	# Steering wheel on a tilted column.
	var column := Node3D.new()
	column.position = Vector3(eye.x, eye.y - 0.32, eye.z - 0.46)
	column.rotation = Vector3(deg_to_rad(68.0), 0, 0)
	add_child(column)
	steering_wheel = Node3D.new()
	column.add_child(steering_wheel)
	var rim := MeshInstance3D.new()
	var tm := TorusMesh.new()
	tm.inner_radius = 0.165
	tm.outer_radius = 0.19
	rim.mesh = tm
	rim.material_override = trim
	steering_wheel.add_child(rim)
	var spoke_mat := _mat(Color(0.25, 0.25, 0.27), 0.5, 0.4)
	for ang: float in [0.0, 2.1, -2.1]:
		var sp := MeshInstance3D.new()
		var sb := BoxMesh.new()
		sb.size = Vector3(0.03, 0.015, 0.17)
		sp.mesh = sb
		sp.material_override = spoke_mat
		sp.transform = Transform3D(Basis(Vector3.UP, ang), Vector3(sin(ang), 0, cos(ang)) * 0.085)
		steering_wheel.add_child(sp)
	var marker := MeshInstance3D.new()
	var mb := BoxMesh.new()
	mb.size = Vector3(0.02, 0.02, 0.03)
	marker.mesh = mb
	marker.material_override = _mat(Color(1, 0.5, 0.1))
	marker.position = Vector3(0, 0.0, -0.178)
	steering_wheel.add_child(marker)
	# Gauge readout on the dash.
	gauge = Label3D.new()
	gauge.font_size = 48
	gauge.pixel_size = 0.0008
	gauge.outline_size = 8
	gauge.modulate = Color(0.75, 0.95, 1.0)
	gauge.position = Vector3(eye.x + 0.27, dash_top + 0.1, front + 0.42)
	gauge.rotation = Vector3(deg_to_rad(-8.0), 0, 0)
	add_child(gauge)


func _process(_dt: float) -> void:
	if vehicle == null:
		return
	for i in vehicle.wheels.size():
		var wh := vehicle.wheels[i]
		var pivot := wheel_nodes[i]
		var drop := wh.travel - wh.compression
		pivot.position = wh.mount - Vector3(0, drop, 0)
		pivot.rotation = Vector3(0, -wh.steer, 0)
		(pivot.get_node("spin") as Node3D).rotation = Vector3(-wh.spin, 0, 0)
	brake_lights.emission_energy_multiplier = 3.0 if vehicle.in_brake > 0.1 else 0.3
	if steering_wheel:
		steering_wheel.rotation.y = -vehicle.steer_angle * steer_ratio
	if gauge:
		gauge.text = "%d  %s  %d" % [int(vehicle.speed * 3.6), vehicle.gear_label(), int(vehicle.rpm())]
