class_name AcPaths
extends RefCounted
## Finds the player's Assetto Corsa install and the cars inside it.


static func _settings() -> Node:
	return Engine.get_main_loop().root.get_node("Settings")


## Folder that contains the car folders (…/assettocorsa/content/cars).
static func cars_dir() -> String:
	var custom := str(_settings().ac_root)
	var candidates: Array[String] = []
	if custom != "":
		candidates.append(custom)
		candidates.append(custom.path_join("content/cars"))
	if OS.get_name() == "Windows":
		var libs: Array[String] = ["C:/Program Files (x86)/Steam", "C:/Program Files/Steam"]
		# Extra Steam library folders (other drives).
		for steam: String in ["C:/Program Files (x86)/Steam", "C:/Program Files/Steam"]:
			var vdf: String = steam.path_join("steamapps/libraryfolders.vdf")
			if FileAccess.file_exists(vdf):
				var re := RegEx.create_from_string("\"path\"\\s+\"([^\"]+)\"")
				for m in re.search_all(FileAccess.get_file_as_string(vdf)):
					libs.append(m.get_string(1).replace("\\\\", "/"))
		for d in ["D", "E", "F", "G"]:
			libs.append("%s:/SteamLibrary" % d)
			libs.append("%s:/Steam" % d)
		for lib in libs:
			candidates.append(lib.path_join("steamapps/common/assettocorsa/content/cars"))
	for c in candidates:
		if DirAccess.dir_exists_absolute(c) and _looks_like_cars(c):
			return c
	return ""


static func _looks_like_cars(dir: String) -> bool:
	for sub in DirAccess.get_directories_at(dir):
		if not find_kn5(dir.path_join(sub)).is_empty():
			return true
	return false


static func list_cars() -> PackedStringArray:
	var d := cars_dir()
	var out := PackedStringArray()
	if d == "":
		return out
	for sub in DirAccess.get_directories_at(d):
		if find_kn5(d.path_join(sub)) != "":
			out.append(sub)
	out.sort()
	return out


## Main model of a car folder: the largest .kn5 that isn't a collider or LOD.
static func find_kn5(car_dir: String) -> String:
	var best := ""
	var best_size := -1
	if not DirAccess.dir_exists_absolute(car_dir):
		return ""
	for fn in DirAccess.get_files_at(car_dir):
		var low := fn.to_lower()
		if not low.ends_with(".kn5") or low.begins_with("collider") or low.contains("_lod"):
			continue
		var fa := FileAccess.open(car_dir.path_join(fn), FileAccess.READ)
		if fa and int(fa.get_length()) > best_size:
			best_size = int(fa.get_length())
			best = car_dir.path_join(fn)
	return best


static func list_skins(car_dir: String) -> PackedStringArray:
	var s := car_dir.path_join("skins")
	var out := PackedStringArray()
	if DirAccess.dir_exists_absolute(s):
		out = DirAccess.get_directories_at(s)
		out.sort()
	return out


## Resolves a build's "ac_model": {"car": id | "path": folder, "skin": name}
## to {car_dir, kn5, skin_dir} or {} if not installed.
static func resolve(ac_model: Variant) -> Dictionary:
	if not (ac_model is Dictionary):
		return {}
	var car_dir := str(ac_model.get("path", ""))
	if car_dir == "" or not DirAccess.dir_exists_absolute(car_dir):
		var root := cars_dir()
		if root == "":
			return {}
		car_dir = root.path_join(str(ac_model.get("car", "")))
	var kn5 := find_kn5(car_dir)
	if kn5 == "":
		return {}
	var skins := list_skins(car_dir)
	var skin := str(ac_model.get("skin", ""))
	if not skins.has(skin):
		skin = skins[0] if skins.size() > 0 else ""
	return {"car_dir": car_dir, "kn5": kn5, "skin_dir": car_dir.path_join("skins").path_join(skin) if skin != "" else "", "skin": skin}
