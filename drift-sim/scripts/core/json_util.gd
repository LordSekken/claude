class_name JsonUtil
extends RefCounted
## Small helpers for reading/writing JSON files and listing data folders.


static func load_file(path: String) -> Variant:
	if not FileAccess.file_exists(path):
		return null
	var text := FileAccess.get_file_as_string(path)
	var json := JSON.new()
	if json.parse(text) != OK:
		push_warning("JSON parse error in %s line %d: %s" % [path, json.get_error_line(), json.get_error_message()])
		return null
	return json.data


static func save_file(path: String, data: Variant) -> bool:
	DirAccess.make_dir_recursive_absolute(path.get_base_dir())
	var f := FileAccess.open(path, FileAccess.WRITE)
	if f == null:
		push_warning("Cannot write %s" % path)
		return false
	f.store_string(JSON.stringify(data, "  ", false))
	return true


## Lists *.json files (full paths) in a directory; missing dirs return [].
static func list_json(dir_path: String) -> PackedStringArray:
	var out := PackedStringArray()
	var dir := DirAccess.open(dir_path)
	if dir == null:
		return out
	for f in dir.get_files():
		# Exported builds may list "x.json.remap"/".import" entries; normalise.
		var name := f.trim_suffix(".remap")
		if name.ends_with(".json") and not out.has(dir_path.path_join(name)):
			out.append(dir_path.path_join(name))
	out.sort()
	return out


static func slugify(text: String) -> String:
	var s := text.to_lower().strip_edges()
	var out := ""
	for c in s:
		if (c >= "a" and c <= "z") or (c >= "0" and c <= "9"):
			out += c
		elif not out.ends_with("_"):
			out += "_"
	out = out.strip_edges().trim_prefix("_").trim_suffix("_")
	return out if out != "" else "untitled"


static func num(d: Dictionary, key: String, default: float) -> float:
	var v: Variant = d.get(key, default)
	if v is float or v is int:
		return float(v)
	if v is String and (v as String).is_valid_float():
		return (v as String).to_float()
	return default
