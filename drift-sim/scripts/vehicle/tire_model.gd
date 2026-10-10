class_name TireModel
extends RefCounted
## Normalised "Magic Formula" style tyre with combined slip.
##
## Slip ratio and slip angle are each normalised by their peak value, combined
## into a single slip magnitude s, run through f(s) = sin(C*atan(B*s)) (which
## peaks at s = 1 and decays toward `tail` when fully sliding), then split back
## into longitudinal / lateral components. This captures the things drifting
## depends on: a defined peak, grip falloff past the peak, and wheelspin eating
## lateral grip (and vice versa).

var mu_long := 1.15
var mu_lat := 1.1
var peak_slip_ratio := 0.10
var peak_slip_angle := deg_to_rad(8.0)
## Fraction of peak grip left when fully sliding (sideways / spinning).
var tail := 0.80
## Load sensitivity exponent (1 = none). Effective mu ~ (Fz/Fz_ref)^(ls-1).
var load_sens := 0.85
var fz_ref := 3500.0

var _b := 0.0
var _c := 0.0


func _init(params: Dictionary = {}) -> void:
	configure(params)


func configure(p: Dictionary) -> void:
	mu_long = JsonUtil.num(p, "mu_long", mu_long)
	mu_lat = JsonUtil.num(p, "mu_lat", mu_lat)
	peak_slip_ratio = JsonUtil.num(p, "peak_slip_ratio", peak_slip_ratio)
	peak_slip_angle = deg_to_rad(JsonUtil.num(p, "peak_slip_angle_deg", rad_to_deg(peak_slip_angle)))
	tail = clampf(JsonUtil.num(p, "tail", tail), 0.3, 0.99)
	load_sens = JsonUtil.num(p, "load_sens", load_sens)
	fz_ref = JsonUtil.num(p, "fz_ref", fz_ref)
	# sin(C*pi/2) = tail  ->  C in (1, 2); peak where C*atan(B) = pi/2.
	_c = 2.0 - 2.0 * asin(tail) / PI
	_b = tan(PI / (2.0 * _c))


## Normalised grip curve: 0 at s=0, 1 at s=1, -> tail as s -> inf.
func curve(s: float) -> float:
	return sin(_c * atan(_b * s))


func load_mu_scale(fz: float) -> float:
	return pow(maxf(fz, 1.0) / fz_ref, load_sens - 1.0)


## Returns Vector2(Fx, Fy) in Newtons.
## slip_ratio > 0 -> wheel faster than ground -> +Fx (drive).
## slip_angle > 0 -> contact patch moving to +side -> Fy is negative (restoring).
func compute(slip_ratio: float, slip_angle: float, fz: float, grip: float = 1.0) -> Vector2:
	if fz <= 0.0:
		return Vector2.ZERO
	var sx := slip_ratio / peak_slip_ratio
	var sy := slip_angle / peak_slip_angle
	var s := sqrt(sx * sx + sy * sy)
	if s < 1e-6:
		return Vector2.ZERO
	var c := curve(s)
	# Longitudinal grip falls off less past the peak than lateral grip does
	# (a spinning tyre still pushes quite well; a sliding one loses more).
	if s > 1.0:
		var wl := (sx * sx) / (s * s)
		c += (1.0 - c) * 0.55 * wl
	var f := c * fz * load_mu_scale(fz) * grip
	return Vector2(mu_long * f * sx / s, -mu_lat * f * sy / s)
