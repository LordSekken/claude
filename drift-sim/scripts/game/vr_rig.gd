class_name VrRig
extends XROrigin3D
## OpenXR (Quest 2 via Quest Link / Air Link / Virtual Desktop / SteamVR):
## puts your head in the driver's seat. Launch with "Drift Lab VR.bat"
## (DriftLab.exe --xr-mode on). Recentre: V / right-stick click.

var vehicle: Vehicle
var camera: XRCamera3D
var seat_offset := Vector3.ZERO
var score_label: Label3D
var popup_label: Label3D
var pause_label: Label3D
var _popup_t := 0.0
var _recentred := false
var _frames := 0


static func available() -> bool:
	var xr := XRServer.find_interface("OpenXR")
	return xr != null and xr.is_initialized()


func setup(v: Vehicle, offset: Vector3) -> void:
	vehicle = v
	seat_offset = offset
	camera = XRCamera3D.new()
	camera.near = 0.03
	camera.far = 3000.0
	add_child(camera)
	# World-locked text in front of the car (2D HUDs don't show in a headset).
	score_label = _label(72, Color(1, 0.85, 0.2))
	popup_label = _label(56, Color(0.6, 1, 0.7))
	pause_label = _label(48, Color.WHITE)
	pause_label.visible = false
	pause_label.text = "PAUSED\\nStart / Esc: resume   ·   Back / R: reset car   ·   Y / C: quit to menu"
	v.add_child(score_label)
	v.add_child(popup_label)
	v.add_child(pause_label)
	var eye := v.seat_local(offset)
	score_label.position = Vector3(0, eye.y + 0.55, -float(v.spec.length) * 0.5 - 4.0)
	popup_label.position = score_label.position + Vector3(0, -0.45, 0)
	pause_label.position = eye + Vector3(0, 0.0, -1.2)
	pause_label.pixel_size = 0.0012
	get_viewport().use_xr = true
	# The headset paces frames; desktop vsync would halve the framerate.
	DisplayServer.window_set_vsync_mode(DisplayServer.VSYNC_DISABLED)


func _label(size: int, color: Color) -> Label3D:
	var l := Label3D.new()
	l.font_size = size
	l.pixel_size = 0.004
	l.outline_size = 12
	l.modulate = color
	l.no_depth_test = true
	l.render_priority = 10
	return l


func recentre() -> void:
	XRServer.center_on_hmd(XRServer.RESET_BUT_KEEP_TILT, true)


func show_popup(text: String) -> void:
	popup_label.text = text
	_popup_t = 1.6


func set_paused(p: bool) -> void:
	pause_label.visible = p


func _process(dt: float) -> void:
	if vehicle == null:
		return
	# Recentre once tracking has settled.
	_frames += 1
	if not _recentred and _frames > 30:
		recentre()
		_recentred = true
	global_transform = vehicle.global_transform * Transform3D(Basis(), vehicle.seat_local(seat_offset))
	_popup_t -= dt
	popup_label.visible = _popup_t > 0.0


func shutdown() -> void:
	get_viewport().use_xr = false
