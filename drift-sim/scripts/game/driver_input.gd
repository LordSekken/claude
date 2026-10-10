class_name DriverInput
extends RefCounted
## Reads keyboard, Xbox-style gamepads and steering wheels (e.g. Logitech
## G920) and writes the result into a Vehicle each physics tick.
##
## Gamepad: left stick steers, RT throttle, LT brake, A handbrake,
## B/X shift up/down, LB clutch, Y camera, Back reset.
## Wheel: axes are calibrated in Settings > Controls (move each pedal once);
## the paddles/shifter buttons are bound there too.

var vehicle: Vehicle
var _kb_steer := 0.0
var last_device := "keyboard"


func _init(v: Vehicle) -> void:
	vehicle = v


func _pad() -> int:
	var pads := Input.get_connected_joypads()
	return pads[0] if pads.size() > 0 else -1


func update(dt: float) -> void:
	var s: Node = Engine.get_main_loop().root.get_node("Settings")
	var steer := 0.0
	var throttle := 0.0
	var brake := 0.0
	var handbrake := 0.0
	var clutch := 0.0
	var direct := false

	# Keyboard (smoothed steering so taps aren't binary).
	var kb_target := Input.get_action_strength("steer_right") - Input.get_action_strength("steer_left")
	var rate := 2.5 if absf(kb_target) > absf(_kb_steer) else 4.0
	_kb_steer = move_toward(_kb_steer, kb_target, rate * dt)
	steer = _kb_steer
	throttle = Input.get_action_strength("throttle")
	brake = Input.get_action_strength("brake")
	handbrake = Input.get_action_strength("handbrake")
	clutch = Input.get_action_strength("clutch")
	if absf(kb_target) > 0.0 or throttle > 0.0 or brake > 0.0:
		last_device = "keyboard"

	if s.use_wheel and not (s.steer_binding as Dictionary).is_empty():
		# Steering wheel: map wheel rotation to road-wheel angle 1:1 up to the
		# car's lock, like a real steering ratio (900 deg wheel -> ~15:1).
		var sb: Dictionary = s.steer_binding
		var dev := maxi(int(sb.get("device", 0)), 0)
		if Input.get_connected_joypads().has(dev):
			var raw := Input.get_joy_axis(dev, int(sb.get("axis", 0)) as JoyAxis)
			if bool(sb.get("invert", false)):
				raw = -raw
			var wheel_deg := raw * float(s.wheel_rotation_deg) * 0.5
			var ratio := float(s.wheel_rotation_deg) * 0.5 / rad_to_deg(float(vehicle.spec.steer_lock))
			var road_deg := wheel_deg / maxf(ratio, 1.0)
			steer = clampf(road_deg / rad_to_deg(float(vehicle.spec.steer_lock)), -1.0, 1.0)
			direct = true
			last_device = "wheel"
		for pair in [["throttle", s.throttle_binding], ["brake", s.brake_binding], ["clutch", s.clutch_binding], ["handbrake", s.handbrake_binding]]:
			var val: float = s.read_axis_binding(pair[1])
			if val >= 0.0:
				match pair[0]:
					"throttle": throttle = maxf(throttle, val)
					"brake": brake = maxf(brake, val)
					"clutch": clutch = maxf(clutch, val)
					"handbrake": handbrake = maxf(handbrake, val)
	else:
		var pad := _pad()
		if pad >= 0:
			var lx := Input.get_joy_axis(pad, JOY_AXIS_LEFT_X)
			var rt := Input.get_joy_axis(pad, JOY_AXIS_TRIGGER_RIGHT)
			var lt := Input.get_joy_axis(pad, JOY_AXIS_TRIGGER_LEFT)
			if absf(lx) > 0.08:
				# Non-linear response: fine control near centre, full lock at the edge.
				steer = signf(lx) * pow((absf(lx) - 0.08) / 0.92, 1.6)
				last_device = "gamepad"
			if rt > 0.03 or lt > 0.03:
				last_device = "gamepad"
			throttle = maxf(throttle, rt)
			brake = maxf(brake, lt)

	vehicle.in_steer = clampf(steer, -1.0, 1.0)
	vehicle.in_throttle = throttle
	vehicle.in_brake = brake
	vehicle.in_handbrake = handbrake
	vehicle.in_clutch = clutch
	vehicle.direct_steer = direct
	vehicle.steering_assist = 0.0 if direct else float(s.steering_assist)
	vehicle.auto_clutch = bool(s.auto_clutch) and clutch < 0.05
	vehicle.auto_shift = bool(s.auto_shift)


## Gear changes from button events (keyboard, pad, wheel paddles/shifter).
func handle_event(ev: InputEvent) -> void:
	if ev.is_action_pressed("shift_up"):
		vehicle.shift_up()
	elif ev.is_action_pressed("shift_down"):
		vehicle.shift_down()
	var s: Node = Engine.get_main_loop().root.get_node("Settings")
	if ev is InputEventJoypadButton and ev.pressed:
		var b: Dictionary = s.button_bindings
		for action: String in b:
			var bind: Dictionary = b[action]
			if int(bind.get("button", -1)) == ev.button_index and (int(bind.get("device", -1)) < 0 or int(bind.device) == ev.device):
				if action == "shift_up":
					vehicle.shift_up()
				elif action == "shift_down":
					vehicle.shift_down()
				elif action.begins_with("gear_"):
					# H-pattern shifter: gear_r, gear_1 .. gear_7
					var g := action.substr(5)
					vehicle.gear = -1 if g == "r" else int(g)
					vehicle.shift_timer = float(vehicle.spec.shift_time)
	if ev is InputEventJoypadButton and not ev.pressed:
		# Releasing an H-shifter gear drops to neutral.
		for action: String in s.button_bindings:
			var bind: Dictionary = s.button_bindings[action]
			if action.begins_with("gear_") and int(bind.get("button", -1)) == ev.button_index:
				vehicle.gear = 0


## Simple rumble on gamepads: tyre slip + engine near the limiter.
func rumble(pad_strength: float) -> void:
	var pad := _pad()
	if pad < 0 or last_device != "gamepad":
		return
	var rear_slip := 0.0
	for wh in vehicle.wheels:
		if not wh.front and wh.contact:
			rear_slip = maxf(rear_slip, clampf(absf(wh.slip_ratio) * 1.5 + absf(wh.slip_angle), 0.0, 1.0))
	var weak := clampf(rear_slip * 0.6, 0.0, 1.0) * pad_strength
	var strong := 0.0
	if vehicle.wall_contact:
		strong = 0.8
	Input.start_joy_vibration(pad, weak, strong, 0.1)
