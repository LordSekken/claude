extends Node
## Steering-wheel force feedback via the native helper (native/ffb), which
## drives DirectInput FFB through SDL3. Godot itself can't do wheel FFB.
## The helper ships next to DriftLab.exe as ffb_helper.exe + SDL3.dll.

const OUT_PORT := 47811
const IN_PORT := 47812

var status := "Force feedback off"
var running := false
var _pid := -1
var _tx := PacketPeerUDP.new()
var _rx := PacketPeerUDP.new()
var _keepalive := 0.0
var _last_status_ms := 0
var _started_ms := 0
var _pending_test := false


func _ready() -> void:
	_rx.bind(IN_PORT, "127.0.0.1")
	_tx.set_dest_address("127.0.0.1", OUT_PORT)
	var s: Node = get_node("/root/Settings")
	if bool(s.ffb_enabled):
		start()


func helper_path() -> String:
	var candidates := [OS.get_executable_path().get_base_dir().path_join("ffb_helper.exe")]
	if OS.has_feature("editor"):
		candidates.append(ProjectSettings.globalize_path("res://native/ffb/bin/ffb_helper.exe"))
	for c: String in candidates:
		if FileAccess.file_exists(c):
			return c
	return ""


func start() -> void:
	if running:
		return
	if OS.get_name() != "Windows":
		status = "Force feedback needs Windows"
		return
	var path := helper_path()
	if path == "":
		status = "ffb_helper.exe not found next to DriftLab.exe"
		return
	_pid = OS.create_process(path, ["--parent", str(OS.get_process_id())])
	running = _pid > 0
	_started_ms = Time.get_ticks_msec()
	_last_status_ms = _started_ms
	status = "Starting force feedback..." if running else "Couldn't start ffb_helper.exe (blocked by antivirus?)"


func stop() -> void:
	if running:
		_tx.put_packet("DLQT".to_ascii_buffer())
	if _pid > 0 and OS.is_process_running(_pid):
		OS.kill(_pid)
	_pid = -1
	running = false
	status = "Force feedback off"


func test() -> void:
	if not running:
		start()
	# The helper needs a moment to start and find the wheel; send when it's up.
	_pending_test = true


## force: -1..1 (+ = push wheel right). damper/spring: 0..1.
func send(force: float, damper: float, spring: float) -> void:
	if not running:
		return
	var s: Node = get_node("/root/Settings")
	var buf := StreamPeerBuffer.new()
	buf.put_data("DLFB".to_ascii_buffer())
	buf.put_float(clampf(force, -1.0, 1.0))
	buf.put_float(clampf(damper * float(s.ffb_damper), 0.0, 1.0))
	buf.put_float(clampf(spring, 0.0, 1.0))
	buf.put_float(float(s.ffb_gain))
	_tx.put_packet(buf.data_array)


func _process(dt: float) -> void:
	while _rx.get_available_packet_count() > 0:
		var pkt := _rx.get_packet()
		if pkt.size() > 4 and pkt.slice(0, 4).get_string_from_ascii() == "DLST":
			status = pkt.slice(4).get_string_from_utf8()
			_last_status_ms = Time.get_ticks_msec()
			if _pending_test and status.begins_with("FFB ready"):
				_pending_test = false
				_tx.put_packet("DLTS".to_ascii_buffer())
	if not running:
		return
	_keepalive -= dt
	if _keepalive <= 0.0:
		_keepalive = 0.5
		_tx.put_packet("DLHI".to_ascii_buffer())
	# Watchdog: the helper reports every second. If it went quiet, restart it.
	var now := Time.get_ticks_msec()
	if now - _last_status_ms > 6000 and now - _started_ms > 6000:
		status = "Force feedback helper stopped responding - restarting..."
		stop()
		start()


func _notification(what: int) -> void:
	if what == NOTIFICATION_WM_CLOSE_REQUEST or what == NOTIFICATION_PREDELETE:
		stop()
