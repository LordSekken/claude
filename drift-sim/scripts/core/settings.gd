extends Node
## Persistent user settings: API key, control bindings, assists.

const PATH := "user://settings.cfg"

var api_key := ""
var model := "claude-opus-5-5"

# Assists (keyboard/gamepad players want these; wheel players usually not).
var steering_assist := 0.6
var auto_clutch := true
var auto_shift := true
var units_kmh := true

# Controller bindings. Each axis binding: {device, axis, released, pressed}
# maps raw axis value linearly so that released -> 0 and pressed -> 1.
# Steering uses {device, axis, invert, range_deg}.
var steer_binding := {"device": -1, "axis": 0, "invert": false, "deadzone": 0.02}
var throttle_binding := {}
var brake_binding := {}
var clutch_binding := {}
var handbrake_binding := {}
# Joypad buttons: {device, button}
var button_bindings := {}  # action name -> {device, button}
var wheel_rotation_deg := 900.0
var use_wheel := false


func _ready() -> void:
	load_settings()
	if api_key == "" and OS.has_environment("ANTHROPIC_API_KEY"):
		api_key = OS.get_environment("ANTHROPIC_API_KEY")
	_ensure_default_actions()


func load_settings() -> void:
	var cfg := ConfigFile.new()
	if cfg.load(PATH) != OK:
		return
	api_key = cfg.get_value("ai", "api_key", api_key)
	model = cfg.get_value("ai", "model", model)
	steering_assist = cfg.get_value("assists", "steering", steering_assist)
	auto_clutch = cfg.get_value("assists", "auto_clutch", auto_clutch)
	auto_shift = cfg.get_value("assists", "auto_shift", auto_shift)
	units_kmh = cfg.get_value("display", "kmh", units_kmh)
	use_wheel = cfg.get_value("controls", "use_wheel", use_wheel)
	wheel_rotation_deg = cfg.get_value("controls", "wheel_rotation_deg", wheel_rotation_deg)
	steer_binding = cfg.get_value("controls", "steer", steer_binding)
	throttle_binding = cfg.get_value("controls", "throttle", throttle_binding)
	brake_binding = cfg.get_value("controls", "brake", brake_binding)
	clutch_binding = cfg.get_value("controls", "clutch", clutch_binding)
	handbrake_binding = cfg.get_value("controls", "handbrake", handbrake_binding)
	button_bindings = cfg.get_value("controls", "buttons", button_bindings)


func save_settings() -> void:
	var cfg := ConfigFile.new()
	cfg.set_value("ai", "api_key", api_key)
	cfg.set_value("ai", "model", model)
	cfg.set_value("assists", "steering", steering_assist)
	cfg.set_value("assists", "auto_clutch", auto_clutch)
	cfg.set_value("assists", "auto_shift", auto_shift)
	cfg.set_value("display", "kmh", units_kmh)
	cfg.set_value("controls", "use_wheel", use_wheel)
	cfg.set_value("controls", "wheel_rotation_deg", wheel_rotation_deg)
	cfg.set_value("controls", "steer", steer_binding)
	cfg.set_value("controls", "throttle", throttle_binding)
	cfg.set_value("controls", "brake", brake_binding)
	cfg.set_value("controls", "clutch", clutch_binding)
	cfg.set_value("controls", "handbrake", handbrake_binding)
	cfg.set_value("controls", "buttons", button_bindings)
	cfg.save(PATH)


## Keyboard + standard gamepad defaults. Wheels/pedals are read as raw axes.
func _ensure_default_actions() -> void:
	var keys := {
		"steer_left": [KEY_LEFT, KEY_A],
		"steer_right": [KEY_RIGHT, KEY_D],
		"throttle": [KEY_UP, KEY_W],
		"brake": [KEY_DOWN, KEY_S],
		"handbrake": [KEY_SPACE],
		"clutch": [KEY_SHIFT],
		"shift_up": [KEY_E],
		"shift_down": [KEY_Q],
		"camera": [KEY_C],
		"reset_car": [KEY_R],
		"pause": [KEY_ESCAPE],
	}
	var pad_buttons := {
		"handbrake": JOY_BUTTON_A,
		"shift_up": JOY_BUTTON_B,
		"shift_down": JOY_BUTTON_X,
		"camera": JOY_BUTTON_Y,
		"reset_car": JOY_BUTTON_BACK,
		"pause": JOY_BUTTON_START,
	}
	for action: String in keys:
		if not InputMap.has_action(action):
			InputMap.add_action(action, 0.1)
		for k: int in keys[action]:
			var ev := InputEventKey.new()
			ev.physical_keycode = k as Key
			InputMap.action_add_event(action, ev)
		if pad_buttons.has(action):
			var jb := InputEventJoypadButton.new()
			jb.button_index = pad_buttons[action]
			jb.device = -1
			InputMap.action_add_event(action, jb)
	# Handbrake bound to a wheel/button box button in Settings > Controls.
	var hb: Dictionary = button_bindings.get("handbrake", {})
	if not hb.is_empty():
		var e := InputEventJoypadButton.new()
		e.button_index = int(hb.button) as JoyButton
		e.device = int(hb.device)
		InputMap.action_add_event("handbrake", e)


## Maps a raw axis value using a {released, pressed} calibration to 0..1.
static func map_pedal(raw: float, binding: Dictionary) -> float:
	var released: float = binding.get("released", -1.0)
	var pressed: float = binding.get("pressed", 1.0)
	if is_equal_approx(released, pressed):
		return 0.0
	return clampf((raw - released) / (pressed - released), 0.0, 1.0)


func read_axis_binding(binding: Dictionary) -> float:
	if binding.is_empty() or int(binding.get("axis", -1)) < 0:
		return -1.0
	var dev := int(binding.get("device", 0))
	if dev < 0:
		dev = 0
	if not Input.get_connected_joypads().has(dev):
		return -1.0
	return map_pedal(Input.get_joy_axis(dev, int(binding.axis) as JoyAxis), binding)
