class_name Vehicle
extends RigidBody3D
## Raycast-suspension car with a custom tyre model and a full drivetrain:
## engine (torque curve, turbo lag, limiter, engine braking) -> clutch ->
## gearbox -> centre/axle differentials (open / clutch LSD / welded) -> wheels.
##
## Wheel spin is integrated in sub-steps with an implicit tyre-force solve so
## the stiff tyre/wheel system stays stable at the 240 Hz physics rate.
## Local axes: -Z forward, +X right, +Y up.

signal hard_impact(strength: float)

const SUBSTEPS := 4
const SURFACES := {
	"asphalt": {"grip": 1.0, "drag": 0.0},
	"kerb": {"grip": 0.93, "drag": 0.0},
	"concrete": {"grip": 0.95, "drag": 0.0},
	"dirt": {"grip": 0.72, "drag": 0.05},
	"grass": {"grip": 0.58, "drag": 0.09},
	"gravel": {"grip": 0.55, "drag": 0.20},
	"sand": {"grip": 0.5, "drag": 0.30},
}
const ROAD_SURFACES := ["asphalt", "kerb", "concrete"]


class Wheel:
	var mount: Vector3  # suspension top, local space
	var front := false
	var left := false
	var radius := 0.31
	var inertia := 1.4
	var spring := 30000.0
	var bump := 2500.0
	var rebound := 4000.0
	var travel := 0.2
	var tire: TireModel
	var omega := 0.0
	var spin := 0.0
	var compression := 0.0
	var prev_compression := 0.0
	var contact := false
	var point := Vector3.ZERO
	var normal := Vector3.UP
	var surface := "asphalt"
	var fz := 0.0
	var steer := 0.0
	var v_long := 0.0
	var v_lat := 0.0
	var slip_ratio := 0.0
	var slip_angle := 0.0
	var fx := 0.0
	var fy := 0.0
	var drive_torque := 0.0
	var dir_f := Vector3.FORWARD
	var dir_s := Vector3.RIGHT
	var brake_torque := 0.0
	var visual: Node3D


var spec: Dictionary = {}
var wheels: Array[Wheel] = []

# Driver inputs (set by the controller each tick).
var in_steer := 0.0      # -1 left .. +1 right (fraction of lock)
var in_throttle := 0.0
var in_brake := 0.0
var in_handbrake := 0.0
var in_clutch := 0.0     # 1 = pedal fully pressed
var steering_assist := 0.0
var auto_clutch := true
var auto_shift := false
## When true, in_steer is a direct wheel angle fraction (steering wheel users).
var direct_steer := false

var gear := 1            # -1 = R, 0 = N, 1.. = forward gears
var engine_omega := 80.0
var boost := 0.0
var shift_timer := 0.0
var limiter_timer := 0.0
var auto_shift_cooldown := 0.0
var steer_angle := 0.0
var clutch_slip := 0.0
var speed := 0.0
var drift_angle := 0.0   # body slip angle in degrees (signed)
var wall_contact := false
var wall_proximity := INF
var on_road_wheels := 0

var _gears: Array = []
var _prev_velocity := Vector3.ZERO
var _rpm_curve: PackedFloat32Array
var _nm_curve: PackedFloat32Array
var _ray_params := PhysicsRayQueryParameters3D.new()


func setup(car_spec: Dictionary) -> void:
	spec = car_spec
	mass = float(spec.mass)
	_gears = spec.gears
	_rpm_curve = spec.curve_rpm
	_nm_curve = spec.curve_nm
	collision_layer = 4
	collision_mask = 1 | 2
	contact_monitor = true
	max_contacts_reported = 8
	continuous_cd = true
	can_sleep = false
	var pm := PhysicsMaterial.new()
	pm.friction = 0.25
	pm.bounce = 0.05
	physics_material_override = pm

	var L := float(spec.wheelbase)
	var rf := float(spec.radius_f)
	var rr := float(spec.radius_r)
	var rmean := (rf + rr) * 0.5
	# Body origin sits at axle height, midway between the axles.
	center_of_mass_mode = RigidBody3D.CENTER_OF_MASS_MODE_CUSTOM
	center_of_mass = Vector3(0.0, float(spec.cg_height) - rmean, -L * 0.5 + float(spec.com_from_front))
	var w := float(spec.width)
	var h := float(spec.height)
	var l := float(spec.length)
	# Same convention as Assetto Corsa's INERTIA box: inertia of a uniform box.
	inertia = Vector3(
		mass * (h * h + l * l) / 12.0,
		mass * (w * w + l * l) / 12.0,
		mass * (w * w + h * h) / 12.0) * 0.9

	var shape := CollisionShape3D.new()
	var box := BoxShape3D.new()
	var body_h := h * 0.62
	box.size = Vector3(w * 0.96, body_h, l * 0.98)
	shape.shape = box
	var clearance := 0.14 + float(spec.ride_offset) * 0.6
	shape.position = Vector3(0.0, -rmean + maxf(clearance, 0.07) + body_h * 0.5, 0.0)
	add_child(shape)

	wheels.clear()
	for i in 4:
		var wh := Wheel.new()
		wh.front = i < 2
		wh.left = i % 2 == 0
		var track := float(spec.track_f if wh.front else spec.track_r)
		var z := -L * 0.5 if wh.front else L * 0.5
		wh.radius = rf if wh.front else rr
		wh.travel = maxf(float(spec.travel), 0.08)
		wh.mount = Vector3((-0.5 if wh.left else 0.5) * track, wh.travel * 0.5, z)
		wh.inertia = float(spec.wheel_inertia_f if wh.front else spec.wheel_inertia_r)
		wh.spring = float(spec.spring_f if wh.front else spec.spring_r)
		wh.bump = float(spec.bump_f if wh.front else spec.bump_r)
		wh.rebound = float(spec.rebound_f if wh.front else spec.rebound_r)
		wh.tire = TireModel.new(spec.tire_f if wh.front else spec.tire_r)
		wheels.append(wh)
	engine_omega = float(spec.idle_rpm) * TAU / 60.0
	_ray_params.collision_mask = 1
	_ray_params.exclude = [get_rid()]


# --- Public helpers ---------------------------------------------------------

func rpm() -> float:
	return engine_omega * 60.0 / TAU


func forward() -> Vector3:
	return -global_basis.z


func shift_up() -> void:
	if gear < _gears.size():
		gear += 1
		shift_timer = float(spec.shift_time)


func shift_down() -> void:
	if gear > -1:
		gear -= 1
		shift_timer = float(spec.shift_time)


func gear_label() -> String:
	return "R" if gear < 0 else ("N" if gear == 0 else str(gear))


func reset_to(xform: Transform3D) -> void:
	global_transform = xform
	linear_velocity = Vector3.ZERO
	angular_velocity = Vector3.ZERO
	for wh in wheels:
		wh.omega = 0.0
		wh.compression = 0.0
		wh.prev_compression = 0.0
	gear = 1
	boost = 0.0
	engine_omega = float(spec.idle_rpm) * TAU / 60.0


func engine_torque(rpm_now: float, throttle: float) -> float:
	var full := CarBuilder.torque_at(_rpm_curve, _nm_curve, rpm_now) * float(spec.power_mult)
	if spec.turbo:
		full *= (1.0 + boost) / (1.0 + float(spec.boost_stock))
	var coast := float(spec.coast_nm) * clampf(rpm_now / float(spec.limiter_rpm), 0.15, 1.0)
	return throttle * full - (1.0 - throttle) * coast


# --- Simulation ---------------------------------------------------------------

func _physics_process(dt: float) -> void:
	if spec.is_empty():
		return
	var space := get_world_3d().direct_space_state
	var xf := global_transform
	var up := xf.basis.y
	var fwd := -xf.basis.z
	var right := xf.basis.x
	var lv := linear_velocity
	speed = lv.length()
	var com_world := xf * center_of_mass

	_update_steering(dt, fwd, right, lv)
	_update_suspension(space, xf, up, dt)

	# Contact-patch kinematics.
	on_road_wheels = 0
	for wh in wheels:
		if not wh.contact:
			continue
		if ROAD_SURFACES.has(wh.surface):
			on_road_wheels += 1
		var wf := fwd.rotated(up, -wh.steer)
		wf = (wf - wh.normal * wf.dot(wh.normal)).normalized()
		var ws := wh.normal.cross(wf).normalized() * -1.0  # points to the car's right
		var pv := lv + angular_velocity.cross(wh.point - com_world)
		wh.v_long = pv.dot(wf)
		wh.v_lat = pv.dot(ws)
		wh.dir_f = wf
		wh.dir_s = ws

	_update_drivetrain(dt)

	# Apply forces.
	for wh in wheels:
		if not wh.contact:
			continue
		var f: Vector3 = wh.dir_f * wh.fx + wh.dir_s * wh.fy + up * wh.fz
		var drag := float(SURFACES.get(wh.surface, SURFACES.asphalt).drag)
		if drag > 0.0:
			f -= wh.dir_f * signf(wh.v_long) * drag * wh.fz
		apply_force(f, wh.point - xf.origin)

	# Aero drag + rolling resistance.
	if speed > 0.1:
		apply_central_force(-lv * (0.5 * 1.2 * float(spec.cda) * speed + 0.012 * mass * 9.81 / maxf(speed, 1.0)))

	# Derived values for HUD / scoring.
	var v_flat := lv - up * lv.dot(up)
	if v_flat.length() > 2.0:
		drift_angle = rad_to_deg(atan2(v_flat.dot(right), v_flat.dot(fwd)))
	else:
		drift_angle = 0.0
	_update_contacts(space, xf, dt)
	_prev_velocity = lv


func _update_steering(dt: float, fwd: Vector3, right: Vector3, lv: Vector3) -> void:
	var lock := float(spec.steer_lock)
	var target := in_steer * lock
	if not direct_steer:
		# Gamepad/keyboard: front wheels self-align with the direction of travel
		# when you let go (like caster on a real car) - makes counter-steer easy.
		var v_long := lv.dot(fwd)
		if absf(v_long) > 3.0 and steering_assist > 0.0:
			var front_v := lv + angular_velocity.cross(fwd * float(spec.wheelbase) * 0.5)
			var align := atan2(front_v.dot(right), absf(front_v.dot(fwd)))
			align = clampf(align, -lock, lock)
			target = lerpf(target, align, steering_assist * (1.0 - absf(in_steer)))
		var rate := 3.2 * lock
		steer_angle = move_toward(steer_angle, target, rate * dt)
	else:
		steer_angle = clampf(target, -lock, lock)
	# Ackermann: inner wheel turns more. Blend with parallel steer.
	var L := float(spec.wheelbase)
	var t := float(spec.track_f)
	var ack := float(spec.ackermann)
	for wh in wheels:
		if not wh.front:
			wh.steer = 0.0
			continue
		if absf(steer_angle) < 1e-4:
			wh.steer = 0.0
			continue
		var r := L / tan(absf(steer_angle))
		var inner := (wh.left and steer_angle < 0.0) or (not wh.left and steer_angle > 0.0)
		var ack_angle := atan(L / (r - t * 0.5)) if inner else atan(L / (r + t * 0.5))
		wh.steer = signf(steer_angle) * lerpf(absf(steer_angle), ack_angle, ack)


func _update_suspension(space: PhysicsDirectSpaceState3D, xf: Transform3D, up: Vector3, dt: float) -> void:
	for wh in wheels:
		var from := xf * wh.mount
		var length := wh.travel + wh.radius
		_ray_params.from = from
		_ray_params.to = from - up * length
		var hit := space.intersect_ray(_ray_params)
		wh.prev_compression = wh.compression
		if hit.is_empty():
			wh.contact = false
			wh.compression = 0.0
			wh.fz = 0.0
			continue
		wh.contact = true
		wh.point = hit.position
		wh.normal = hit.normal
		var col: Object = hit.collider
		wh.surface = str(col.get_meta("surface", "asphalt")) if col else "asphalt"
		wh.compression = clampf(length - from.distance_to(hit.position), 0.0, wh.travel)
	# Spring + damper + anti-roll bars.
	for axle in 2:
		var l := wheels[axle * 2]
		var r := wheels[axle * 2 + 1]
		var arb := float(spec.arb_f if axle == 0 else spec.arb_r)
		var arb_f := arb * (l.compression - r.compression)
		for wh: Wheel in [l, r]:
			if not wh.contact:
				continue
			var vel := (wh.compression - wh.prev_compression) / dt
			var damp := wh.bump if vel > 0.0 else wh.rebound
			var f := wh.spring * wh.compression + damp * vel
			if wh.compression > wh.travel * 0.92:
				f += (wh.compression - wh.travel * 0.92) * wh.spring * 12.0  # bump stop
			f += arb_f if wh == l else -arb_f
			wh.fz = maxf(f, 0.0) * maxf(up.dot(wh.normal), 0.2)


func _update_drivetrain(dt: float) -> void:
	var layout := str(spec.drivetrain)
	var limiter := float(spec.limiter_rpm)
	var idle := float(spec.idle_rpm)
	var rpm_now := rpm()

	# Automatic gearbox helpers.
	auto_shift_cooldown -= dt
	if auto_shift and shift_timer <= 0.0 and gear > 0 and auto_shift_cooldown <= 0.0:
		if rpm_now > limiter * 0.97 and gear < _gears.size():
			shift_up()
			auto_shift_cooldown = 0.8
		elif rpm_now < limiter * 0.42 and gear > 1:
			shift_down()
			auto_shift_cooldown = 0.8
	var throttle := in_throttle
	var brake := in_brake
	if auto_shift:
		var v_fwd := linear_velocity.dot(forward())
		if gear >= 1 and in_brake > 0.3 and in_throttle < 0.05 and v_fwd < 0.5:
			gear = -1
		elif gear == -1 and in_throttle > 0.3 and v_fwd > -0.5:
			gear = 1
		if gear == -1:
			throttle = in_brake
			brake = in_throttle

	# Turbo.
	if spec.turbo:
		var spool := smoothstep(float(spec.spool_rpm) * 0.6, float(spec.spool_rpm) * 1.1, rpm_now)
		var target_boost := float(spec.boost_target) * spool * throttle
		boost += (target_boost - boost) * minf(dt / float(spec.turbo_lag_s), 1.0)

	# Rev limiter (fuel cut) + idle control.
	if rpm_now > limiter:
		limiter_timer = 0.06
	if limiter_timer > 0.0:
		limiter_timer -= dt
		throttle = 0.0
	if shift_timer > 0.0:
		shift_timer -= dt
		throttle *= 0.15
	if rpm_now < idle:
		throttle = maxf(throttle, clampf((idle - rpm_now) / 250.0, 0.0, 0.6))

	# Clutch engagement (1 = fully engaged).
	var engage := 1.0 - in_clutch
	if auto_clutch:
		# Slip the clutch like a driver launching: engage as the revs come up.
		var launch := clampf((rpm_now - idle * 1.1) / 1800.0, 0.0, 1.0)
		var wheel_rpm := absf(_driven_omega()) * absf(_ratio()) * 60.0 / TAU
		if wheel_rpm > idle * 1.1:
			launch = 1.0
		engage = minf(engage, launch)
	if gear == 0 or shift_timer > 0.0:
		engage = 0.0

	var ratio := _ratio()
	var ie := float(spec.engine_inertia)
	var split := float(spec.awd_front_split)
	var front_driven := layout != "RWD"
	var rear_driven := layout != "FWD"
	var front_share := 1.0 if layout == "FWD" else (split if layout == "AWD" else 0.0)
	var i_driven := 0.0
	for wh in wheels:
		if (wh.front and front_driven) or (not wh.front and rear_driven):
			i_driven += wh.inertia
	var hb := in_handbrake * float(spec.handbrake_torque)
	var bt := brake * float(spec.brake_torque)
	var bias := float(spec.brake_bias)

	var sub_dt := dt / SUBSTEPS
	var acc_fx: Array[float] = [0.0, 0.0, 0.0, 0.0]
	var acc_fy: Array[float] = [0.0, 0.0, 0.0, 0.0]
	clutch_slip = 0.0
	for _s in SUBSTEPS:
		# Engine -> clutch.
		var te := engine_torque(rpm(), throttle)
		var t_clutch := 0.0
		if engage > 0.0 and absf(ratio) > 0.0:
			var w_in := _driven_omega() * ratio
			var i_dl := i_driven / (ratio * ratio)
			var t_eq := (engine_omega - w_in) / (sub_dt * (1.0 / ie + 1.0 / i_dl))
			var cap := float(spec.clutch_nm) * engage
			t_clutch = clampf(t_eq, -cap, cap)
			clutch_slip = maxf(clutch_slip, absf(engine_omega - w_in))
		engine_omega += (te - t_clutch) / ie * sub_dt
		engine_omega = maxf(engine_omega, 0.0)

		# Gearbox output -> differentials.
		var t_out := t_clutch * ratio
		for wh in wheels:
			wh.drive_torque = 0.0
		if front_driven:
			_axle_diff(wheels[0], wheels[1], t_out * front_share, sub_dt)
		if rear_driven:
			_axle_diff(wheels[2], wheels[3], t_out * (1.0 - front_share), sub_dt)
		if layout == "AWD":
			_center_coupling(t_out, sub_dt)

		# Tyres (implicit wheel-spin solve).
		for i in 4:
			var wh := wheels[i]
			var brake_t := bt * (bias if wh.front else 1.0 - bias) * 0.5
			if not wh.front:
				brake_t += hb * 0.5
			_solve_wheel(wh, brake_t, sub_dt)
			acc_fx[i] += wh.fx
			acc_fy[i] += wh.fy
	for i in 4:
		wheels[i].fx = acc_fx[i] / SUBSTEPS
		wheels[i].fy = acc_fy[i] / SUBSTEPS
		wheels[i].spin = fmod(wheels[i].spin + wheels[i].omega * dt, TAU)


func _ratio() -> float:
	if gear == 0:
		return 0.0
	var g: float = -float(spec.reverse) if gear < 0 else float(_gears[gear - 1])
	return g * float(spec.final)


func _driven_omega() -> float:
	var f := (wheels[0].omega + wheels[1].omega) * 0.5
	var r := (wheels[2].omega + wheels[3].omega) * 0.5
	match str(spec.drivetrain):
		"FWD": return f
		"RWD": return r
	var s := float(spec.awd_front_split)
	return f * s + r * (1.0 - s)


## Splits torque 50/50 then applies a clutch-type LSD locking torque that
## resists speed difference: preload + ramp * |input torque| (power / coast).
## "welded" = effectively infinite locking, "open" = none.
func _axle_diff(l: Wheel, r: Wheel, t_in: float, dt: float) -> void:
	l.drive_torque += t_in * 0.5
	r.drive_torque += t_in * 0.5
	var dtype := str(spec.diff_type)
	if dtype == "open":
		return
	var cap := 1.0e7
	if dtype != "welded" and dtype != "locked":
		var ramp := float(spec.diff_power) if t_in >= 0.0 else float(spec.diff_coast)
		cap = float(spec.diff_preload) + ramp * absf(t_in) * 2.0
	var dw := l.omega - r.omega
	var t_eq := dw / (dt * (1.0 / l.inertia + 1.0 / r.inertia))
	var t_lock := clampf(t_eq, -cap, cap)
	l.drive_torque -= t_lock
	r.drive_torque += t_lock


func _center_coupling(t_out: float, dt: float) -> void:
	var f := (wheels[0].omega + wheels[1].omega) * 0.5
	var r := (wheels[2].omega + wheels[3].omega) * 0.5
	var i_f := wheels[0].inertia + wheels[1].inertia
	var i_r := wheels[2].inertia + wheels[3].inertia
	var cap := float(spec.center_lock_nm) + 0.25 * absf(t_out)
	var t_eq := (f - r) / (dt * (1.0 / i_f + 1.0 / i_r))
	var t_lock := clampf(t_eq, -cap, cap) * 0.5
	wheels[0].drive_torque -= t_lock
	wheels[1].drive_torque -= t_lock
	wheels[2].drive_torque += t_lock
	wheels[3].drive_torque += t_lock


func _tire_force(wh: Wheel, omega: float) -> Vector2:
	var grip := float(SURFACES.get(wh.surface, SURFACES.asphalt).grip)
	var vden := maxf(absf(wh.v_long), 3.0)
	var sr := (omega * wh.radius - wh.v_long) / vden
	var sa := atan2(wh.v_lat, vden)
	wh.slip_ratio = sr
	wh.slip_angle = sa
	return wh.tire.compute(sr, sa, wh.fz, grip)


func _solve_wheel(wh: Wheel, brake_t: float, dt: float) -> void:
	if not wh.contact:
		wh.omega += wh.drive_torque / wh.inertia * dt
		wh.omega = move_toward(wh.omega, 0.0, brake_t / wh.inertia * dt + 0.5 * dt)
		wh.fx = 0.0
		wh.fy = 0.0
		return
	var w0 := wh.omega
	var f0 := _tire_force(wh, w0)
	var eps := 0.05
	var f1 := _tire_force(wh, w0 + eps)
	var k := maxf((f1.x - f0.x) / eps, 0.0)  # dFx/domega
	var denom := wh.inertia + dt * wh.radius * k
	var w1 := w0 + dt * (wh.drive_torque - wh.radius * f0.x) / denom
	var dw_brake := brake_t * dt / denom
	if absf(w1) <= dw_brake:
		w1 = 0.0
	else:
		w1 -= signf(w1) * dw_brake
	wh.omega = w1
	var f := _tire_force(wh, w1)
	wh.fx = f.x
	wh.fy = f.y


func _update_contacts(space: PhysicsDirectSpaceState3D, xf: Transform3D, _dt: float) -> void:
	wall_contact = false
	for b in get_colliding_bodies():
		if b.has_meta("wall"):
			wall_contact = true
	var dv := (linear_velocity - _prev_velocity).length()
	if wall_contact and dv > 2.2:
		hard_impact.emit(dv)
	# Wall proximity from the rear bumper corners (tail-to-the-wall bonus).
	var q := PhysicsRayQueryParameters3D.new()
	q.collision_mask = 2
	q.exclude = [get_rid()]
	wall_proximity = INF
	var half_w := float(spec.width) * 0.5
	var half_l := float(spec.length) * 0.5
	for side: float in [-1.0, 1.0]:
		var corner := xf * Vector3(side * half_w, 0.1, half_l)
		for dir: Vector3 in [xf.basis.x * side, (xf.basis.x * side + xf.basis.z).normalized(), xf.basis.z]:
			q.from = corner
			q.to = corner + dir * 2.5
			var hit := space.intersect_ray(q)
			if not hit.is_empty():
				wall_proximity = minf(wall_proximity, corner.distance_to(hit.position))
