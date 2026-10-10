extends Node
## Global app state: selected track/car and data folder locations.

const USER_TRACKS := "user://tracks"
const USER_CARS := "user://cars"
const USER_PARTS := "user://parts"
const BUILTIN_TRACKS := "res://tracks"
const BUILTIN_CARS := "res://data/cars"
const BUILTIN_PARTS := "res://data/parts"

var selected_track_path := "res://tracks/kanto_lot.json"
var selected_car_path := "res://data/cars/outback98_wrx_rwd_drift.json"
## When set, the game loads this external model (.glb/.gltf/.fbx) as the track.
var external_track_model := ""
## Optional reference image shown under the track editor (from photo import).
var editor_background_path := ""
var editor_track: Dictionary = {}

var parts := PartsLibrary.new()


func _ready() -> void:
	for d in [USER_TRACKS, USER_CARS, USER_PARTS + "/chassis", USER_PARTS + "/engines", USER_PARTS + "/transmissions"]:
		DirAccess.make_dir_recursive_absolute(d)
	parts.reload()


func list_tracks() -> PackedStringArray:
	var out := JsonUtil.list_json(BUILTIN_TRACKS)
	out.append_array(JsonUtil.list_json(USER_TRACKS))
	return out


func list_cars() -> PackedStringArray:
	var out := JsonUtil.list_json(BUILTIN_CARS)
	out.append_array(JsonUtil.list_json(USER_CARS))
	return out
