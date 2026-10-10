extends Node
## Screen router: main menu, garage, track editor, photo import, settings, game.

var current: Node
var ui_layer: CanvasLayer


func _ready() -> void:
	get_window().title = "Drift Lab"
	ui_layer = CanvasLayer.new()
	add_child(ui_layer)
	var args := OS.get_cmdline_user_args()
	if args.has("--quick-drive"):
		show_game()
	else:
		show_menu()


func _set_screen(n: Node) -> void:
	if current:
		current.queue_free()
	current = n
	if n is Control:
		(n as Control).set_anchors_preset(Control.PRESET_FULL_RECT)
		ui_layer.add_child(n)
	else:
		add_child(n)


func show_menu() -> void:
	var m := MainMenu.new()
	m.navigate.connect(_on_nav)
	_set_screen(m)


func _on_nav(target: String) -> void:
	match target:
		"drive": show_game()
		"garage": _screen(GarageScreen.new())
		"editor": _screen(TrackEditor.new())
		"photo": _screen(PhotoImportScreen.new())
		"settings": _screen(SettingsScreen.new())
		"menu": show_menu()
		"quit": get_tree().quit()


func _screen(s: Control) -> void:
	s.connect("navigate", _on_nav)
	_set_screen(s)


func show_game(track_override: Dictionary = {}) -> void:
	var app: Node = get_node("/root/App")
	var track: Dictionary
	if not track_override.is_empty():
		track = track_override
	elif app.external_track_model != "":
		track = {"external": app.external_track_model}
	elif not app.editor_track.is_empty() and app.selected_track_path == "":
		track = app.editor_track
	else:
		var t: Variant = JsonUtil.load_file(app.selected_track_path)
		track = t if t is Dictionary else {}
	var car: Variant = JsonUtil.load_file(app.selected_car_path)
	if not (car is Dictionary):
		car = JsonUtil.load_file("res://data/cars/outback98_wrx_rwd_drift.json")
	var g := Game.new()
	_set_screen(g)
	var err := g.start(track, car)
	if err != "":
		push_warning(err)
		show_menu()
		(current as MainMenu).show_error(err)
		return
	g.exit_requested.connect(show_menu)
	if OS.get_cmdline_user_args().has("--quick-drive"):
		print("Drift Lab: %d tracks, %d cars available; driving '%s' on '%s'" % [app.list_tracks().size(), app.list_cars().size(), g.build_result.spec.name, g.track_spec.get("name", "?")])
