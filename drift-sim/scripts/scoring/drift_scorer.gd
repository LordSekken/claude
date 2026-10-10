class_name DriftScorer
extends RefCounted
## Forza-style drift scoring.
##
## * Points accrue while sliding: rate grows with speed and angle.
## * A chain multiplier builds with sustained drifting, transitions
##   (direction changes), clipping points and wall proximity.
## * The chain banks (points x multiplier) once you've stopped drifting for
##   BANK_DELAY seconds. Hitting a wall hard, leaving the track or spinning
##   out loses the whole unbanked chain.
## * Drift zones (sections of the track) are scored separately and given a
##   1-3 star rating.

signal popup(text: String, kind: String)
signal chain_banked(points: int)
signal chain_lost(reason: String, points: int)
signal zone_finished(zone_name: String, points: int, stars: int)

const MIN_SPEED_KMH := 20.0
const MIN_ANGLE := 10.0
const SPIN_ANGLE := 105.0
const BANK_DELAY := 2.0
const OFF_TRACK_GRACE := 0.6
const MAX_MULT := 5.0
const PROXIMITY_DIST := 1.2

var banked := 0
var chain_points := 0.0
var multiplier := 1.0
var chain_active := false
var drifting := false
var drift_time := 0.0      # continuous time in the current drift
var idle_time := 0.0       # time since last drifting
var off_track_time := 0.0
var spin_time := 0.0
var last_side := 0
var side_time := 0.0
var proximity_active := false
var angle := 0.0
var rate := 0.0

# Stats.
var best_chain := 0
var longest_drift := 0.0
var max_angle := 0.0

# Zones: [{name, start, end, target}] with start/end as track fractions.
var zones: Array = []
var zone_index := -1
var zone_points := 0.0
var zone_results := {}  # name -> {points, stars}

var _mult_progress := 0.0


func reset() -> void:
	banked = 0
	chain_points = 0.0
	multiplier = 1.0
	chain_active = false
	drifting = false
	zone_index = -1
	zone_points = 0.0
	zone_results.clear()
	best_chain = 0
	longest_drift = 0.0
	max_angle = 0.0


func chain_total() -> int:
	return int(chain_points * multiplier)


## speed in m/s, angle_deg signed body slip angle, on_track = wheels on road.
func update(dt: float, speed: float, angle_deg: float, on_track: bool, wall_proximity: float, progress: float) -> void:
	var kmh := speed * 3.6
	angle = absf(angle_deg)
	var side := int(signf(angle_deg))
	drifting = kmh >= MIN_SPEED_KMH and angle >= MIN_ANGLE and angle < SPIN_ANGLE and on_track
	rate = 0.0

	# Spin-out detection.
	if angle > SPIN_ANGLE and kmh > 15.0:
		spin_time += dt
		if spin_time > 0.35:
			lose_chain("SPUN OUT")
	else:
		spin_time = 0.0

	# Off track.
	if not on_track and chain_active:
		off_track_time += dt
		if off_track_time > OFF_TRACK_GRACE:
			lose_chain("OFF TRACK")
	else:
		off_track_time = 0.0

	if drifting:
		if not chain_active:
			chain_active = true
			chain_points = 0.0
			multiplier = 1.0
			_mult_progress = 0.0
		# Transition: switched sides quickly while holding angle.
		if last_side != 0 and side != last_side and idle_time < 1.0:
			_bump_mult(0.25)
			popup.emit("TRANSITION", "bonus")
		last_side = side
		idle_time = 0.0
		drift_time += dt
		longest_drift = maxf(longest_drift, drift_time)
		max_angle = maxf(max_angle, angle)
		# Base rate: speed x angle, angle benefit flattens past ~50 deg.
		var angle_f := clampf(angle / 30.0, 0.0, 1.0) + clampf((angle - 30.0) / 40.0, 0.0, 0.6)
		rate = 2.2 * kmh * angle_f
		# Wall proximity: tail near the wall without touching it.
		var prox := wall_proximity < PROXIMITY_DIST
		if prox:
			rate *= 1.0 + 0.8 * (1.0 - wall_proximity / PROXIMITY_DIST)
			if not proximity_active:
				popup.emit("PROXIMITY", "bonus")
		proximity_active = prox
		chain_points += rate * dt
		# Sustained drift slowly grows the multiplier.
		_mult_progress += dt
		if _mult_progress >= 1.5:
			_mult_progress = 0.0
			_bump_mult(0.1)
	else:
		drift_time = 0.0
		proximity_active = false
		if chain_active:
			idle_time += dt
			if idle_time >= BANK_DELAY:
				bank_chain()
		else:
			last_side = 0

	_update_zone(progress, rate * dt * multiplier)


func _bump_mult(amount: float) -> void:
	multiplier = minf(multiplier + amount, MAX_MULT)


func add_clip(bonus := 400.0) -> void:
	if not drifting:
		return
	chain_points += bonus
	_bump_mult(0.3)
	popup.emit("CLIP!", "clip")


func bank_chain() -> void:
	var pts := chain_total()
	if pts > 0:
		banked += pts
		best_chain = maxi(best_chain, pts)
		chain_banked.emit(pts)
		popup.emit("+%s" % format_points(pts), "bank")
	chain_active = false
	chain_points = 0.0
	multiplier = 1.0
	idle_time = 0.0
	last_side = 0


func lose_chain(reason: String) -> void:
	if not chain_active:
		return
	var pts := chain_total()
	chain_active = false
	chain_points = 0.0
	multiplier = 1.0
	idle_time = 0.0
	last_side = 0
	if zone_index >= 0:
		zone_points = 0.0
	chain_lost.emit(reason, pts)
	popup.emit(reason, "fail")


func on_wall_impact(strength: float) -> void:
	if strength > 2.2:
		lose_chain("WRECKED")


static func in_range(p: float, a: float, b: float) -> bool:
	if a <= b:
		return p >= a and p <= b
	return p >= a or p <= b  # zone wraps past the start line


func _update_zone(progress: float, earned: float) -> void:
	if progress < 0.0 or zones.is_empty():
		return
	var current := -1
	for i in zones.size():
		var z: Dictionary = zones[i]
		if in_range(progress, float(z.start), float(z.end)):
			current = i
			break
	if current != zone_index:
		if zone_index >= 0:
			_finish_zone(zones[zone_index])
		zone_index = current
		zone_points = 0.0
		if current >= 0:
			popup.emit("ZONE: %s" % zones[current].get("name", "Drift Zone"), "zone")
	if zone_index >= 0:
		zone_points += earned


func _finish_zone(z: Dictionary) -> void:
	var target := float(z.get("target", 10000.0))
	var pts := int(zone_points)
	var stars := 0
	if pts >= target * 0.4:
		stars = 1
	if pts >= target * 0.7:
		stars = 2
	if pts >= target:
		stars = 3
	var name := str(z.get("name", "Drift Zone"))
	var prev: Dictionary = zone_results.get(name, {"points": 0})
	if pts >= int(prev.points):
		zone_results[name] = {"points": pts, "stars": stars}
	zone_finished.emit(name, pts, stars)
	popup.emit("%s  %s" % ["★".repeat(stars) + "☆".repeat(3 - stars), format_points(pts)], "zone")


static func format_points(p: int) -> String:
	var s := str(absi(p))
	var out := ""
	while s.length() > 3:
		out = "," + s.substr(s.length() - 3) + out
		s = s.substr(0, s.length() - 3)
	return ("-" if p < 0 else "") + s + out
