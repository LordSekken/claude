class_name AcIni
extends RefCounted
## Parser for Assetto Corsa data files (.ini with ; comments, and .lut tables).


static func parse(text: String) -> Dictionary:
	var out := {}
	var section := ""
	for raw in text.split("\n"):
		var line := raw.strip_edges()
		var sc := line.find(";")
		if sc >= 0:
			line = line.substr(0, sc).strip_edges()
		var sl := line.find("//")
		if sl >= 0:
			line = line.substr(0, sl).strip_edges()
		if line == "":
			continue
		if line.begins_with("[") and line.ends_with("]"):
			section = line.substr(1, line.length() - 2).strip_edges().to_upper()
			if not out.has(section):
				out[section] = {}
			continue
		var eq := line.find("=")
		if eq < 0 or section == "":
			continue
		out[section][line.substr(0, eq).strip_edges().to_upper()] = line.substr(eq + 1).strip_edges()
	return out


static func load_ini(path: String) -> Dictionary:
	if not FileAccess.file_exists(path):
		return {}
	return parse(FileAccess.get_file_as_string(path))


## "rpm|value" lines -> [[x, y], ...]
static func parse_lut(text: String) -> Array:
	var out := []
	for raw in text.split("\n"):
		var line := raw.strip_edges()
		var sc := line.find(";")
		if sc >= 0:
			line = line.substr(0, sc)
		var parts := line.split("|")
		if parts.size() == 2 and parts[0].strip_edges().is_valid_float() and parts[1].strip_edges().is_valid_float():
			out.append([parts[0].strip_edges().to_float(), parts[1].strip_edges().to_float()])
	return out


static func f(ini: Dictionary, section: String, key: String, default: float) -> float:
	var s: Variant = ini.get(section, {}).get(key, null)
	if s == null:
		return default
	var t := str(s).split(",")[0].strip_edges()
	return t.to_float() if t.is_valid_float() else default


static func s(ini: Dictionary, section: String, key: String, default := "") -> String:
	return str(ini.get(section, {}).get(key, default))
