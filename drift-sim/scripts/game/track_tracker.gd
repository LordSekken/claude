class_name TrackTracker
extends RefCounted
## Tracks the car's progress along the centreline (0..1), laps and lap times.

var samples: Dictionary
var index := 0
var progress := 0.0
var lap := 1
var lap_time := 0.0
var best_lap := -1.0
var last_lap := -1.0
var _armed := false


func _init(s: Dictionary) -> void:
	samples = s


func valid() -> bool:
	return samples.has("pos") and (samples.pos as PackedVector3Array).size() >= 2


func nearest(p: Vector3, full := false) -> int:
	var pos: PackedVector3Array = samples.pos
	var n := pos.size()
	var best := index
	var best_d := INF
	if full:
		for i in n:
			var d := pos[i].distance_squared_to(p)
			if d < best_d:
				best_d = d
				best = i
		return best
	for k in range(-40, 41):
		var i := index + k
		if samples.closed:
			i = posmod(i, n)
		elif i < 0 or i >= n:
			continue
		var d := pos[i].distance_squared_to(p)
		if d < best_d:
			best_d = d
			best = i
	if best_d > 900.0:
		return nearest(p, true)
	return best


func update(p: Vector3, dt: float, start_frac: float) -> void:
	if not valid():
		progress = -1.0
		return
	var n: int = (samples.pos as PackedVector3Array).size()
	var prev := progress
	index = nearest(p)
	progress = float(index) / n
	lap_time += dt
	if not samples.closed:
		return
	var rel_prev := fposmod(prev - start_frac, 1.0)
	var rel := fposmod(progress - start_frac, 1.0)
	if rel_prev < 0.5 and rel >= 0.5 and rel < 0.7:
		_armed = true  # passed half-way going forwards
	if rel_prev > 0.9 and rel < 0.1 and _armed:
		last_lap = lap_time
		if best_lap < 0.0 or lap_time < best_lap:
			best_lap = lap_time
		lap += 1
		lap_time = 0.0
		_armed = false
