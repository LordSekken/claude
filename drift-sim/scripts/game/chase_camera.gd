class_name ChaseCamera
extends Camera3D
## Drift-friendly chase camera: follows the direction of travel (so you see
## the car's angle), plus bumper and cockpit views.

enum Mode { CHASE, CHASE_CLOSE, LOCKED, HOOD }
var target: Vehicle
var mode := Mode.CHASE
var _pos := Vector3.ZERO
var _look := Vector3.ZERO
var _yaw_dir := Vector3.FORWARD


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
	# Blend heading toward velocity direction while moving.
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
		Mode.LOCKED:
			global_position = target.global_transform * Vector3(0, 1.8, 6.0)
			look_at(target.global_transform * Vector3(0, 0.6, -3.0), Vector3.UP)
			fov = 70.0
		Mode.HOOD:
			var h := float(target.spec.height) * 0.62
			global_transform = target.global_transform * Transform3D(Basis(), Vector3(-0.35, h - 0.25, 0.1))
			fov = 75.0
