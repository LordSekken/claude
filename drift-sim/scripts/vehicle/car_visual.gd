class_name CarVisual
extends Node3D
## Procedural low-poly body built from the chassis dimensions and body style,
## plus wheel meshes that follow the simulated suspension and wheel spin.

var vehicle: Vehicle
var wheel_nodes: Array[Node3D] = []
var brake_lights: StandardMaterial3D


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
	var glass := _mat(Color(0.08, 0.1, 0.13), 0.2, 0.1)
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
