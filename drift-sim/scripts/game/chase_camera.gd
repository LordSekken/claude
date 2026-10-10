class_name ChaseCamera
extends Camera3D
## Cameras: chase (follows the direction of travel so you see the angle),
## close chase, bonnet, and first-person cockpit. In the cockpit the view
## can glance into the drift (desktop only - in VR your head does that).

enum Mode { CHASE, CHASE_CLOSE, HOOD, COCKPIT }
var target: Vehicle
var mode := Mode.COCKPIT
var look_to_apex := true
var seat_offset := Vector3.ZERO
var _pos := Vector3.ZERO
var _look := Vector3.ZERO
var _yaw_dir := Vector3.FORWARD
var _glance := 0.0


func cycle() -> void:
	mode = ((mode + 1) % Mode.size()) as Mode


func snap() -> void:
	if target == null:
		return
	_yaw_dir = target.forward()
	_pos = target.global_position - _yaw_dir * 7.0 + Vector3.UP * 2.5
	_look = target.global_position
	global_position = _pos


func _physics_process(dt: float) -> void:
	if target == null:
		return
	var car_pos := target.global_position
	var fwd := target.forward()
	var vel := target.linear_velocity
	vel.y = 0.0
	var desired := fwd
	if vel.length() > 4.0:
		desired = fwd.slerp(vel.normalized(), 0.65).normalized()
	_yaw_dir = _yaw_dir.slerp(desired, 1.0 - exp(-4.0 * dt)).normalized()
	match mode:
		Mode.CHASE, Mode.CHASE_CLOSE:
			var dist := 7.0 if mode == Mode.CHASE else 5.0
			var height := 2.4 if mode == Mode.CHASE else 1.7
			var want := car_pos - _yaw_dir * dist + Vector3.UP * height
			_pos = _pos.lerp(want, 1.0 - exp(-10.0 * dt))
			_look = _look.lerp(car_pos + Vector3.UP * 0.6 + _yaw_dir * 2.0, 1.0 - exp(-14.0 * dt))
			global_position = _pos
			look_at(_look, Vector3.UP)
			fov = 70.0 + clampf(target.speed * 0.15, 0.0, 12.0)
		Mode.HOOD:
			var h := float(target.spec.height) * 0.45
			global_transform = target.global_transform * Transform3D(Basis(), Vector3(0, h - 0.2, -float(target.spec.length) * 0.3))
			fov = 75.0
		Mode.COCKPIT:
			var glance_target := 0.0
			if look_to_apex and target.speed > 5.0:
				glance_target = clampf(-deg_to_rad(target.drift_angle) * 0.55, -0.8, 0.8)
			var manual := Input.get_action_strength("look_right") - Input.get_action_strength("look_left")
			var pads := Input.get_connected_joypads()
			if pads.size() > 0 and absf(Input.get_joy_axis(pads[0], JOY_AXIS_RIGHT_X)) > 0.2:
				manual = Input.get_joy_axis(pads[0], JOY_AXIS_RIGHT_X)
			if absf(manual) > 0.05:
				glance_target = -manual * 1.4
			_glance = lerpf(_glance, glance_target, 1.0 - exp(-6.0 * dt))
			var basis := Basis(Vector3.UP, _glance) * Basis(Vector3.RIGHT, deg_to_rad(-4.0))
			global_transform = target.global_transform * Transform3D(basis, target.seat_local(seat_offset))
			fov = 78.0
