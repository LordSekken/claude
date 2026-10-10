class_name CarAudio
extends AudioStreamPlayer
## Procedural car audio, synthesised from the simulation every frame:
## engine (firing-frequency harmonics shaped by load; flat-4s get the
## unequal-length boxer burble), turbo whine + blow-off, tyre squeal from
## slip, surface rumble, wind and wall impacts.

const RATE := 22050.0

var vehicle: Vehicle
var playback: AudioStreamGeneratorPlayback
var cylinders := 4
var boxer := false
var volume := 0.8

var _ph := 0.0          # engine firing phase
var _ph_turbo := 0.0
var _ph_thud := 0.0
var _rpm := 900.0
var _load := 0.0
var _squeal := 0.0
var _rumble := 0.0
var _wind := 0.0
var _boost := 0.0
var _bov := 0.0
var _thud := 0.0
var _prev_throttle := 0.0
# Filter states.
var _bp_low := 0.0
var _bp_band := 0.0
var _lp_rumble := 0.0
var _lp_wind := 0.0
var _lp_bov := 0.0
var _lp_noise := 0.0


func setup(v: Vehicle, vol: float) -> void:
	vehicle = v
	volume = vol
	cylinders = int(v.spec.get("cylinders", 4))
	boxer = bool(v.spec.get("boxer", false))
	var gen := AudioStreamGenerator.new()
	gen.mix_rate = RATE
	gen.buffer_length = 0.12
	stream = gen
	bus = "Master"
	play()
	playback = get_stream_playback()
	v.hard_impact.connect(func(strength: float) -> void: _thud = clampf(strength / 6.0, 0.3, 1.0))


func _process(dt: float) -> void:
	if vehicle == null or playback == null:
		return
	# Smooth the control values once per frame.
	var k := 1.0 - exp(-dt * 25.0)
	_rpm = lerpf(_rpm, vehicle.rpm(), k)
	var thr := vehicle.in_throttle if vehicle.limiter_timer <= 0.0 else 0.0
	_load = lerpf(_load, clampf(thr, 0.0, 1.0), 1.0 - exp(-dt * 12.0))
	var slip := 0.0
	var rough := 0.0
	for wh in vehicle.wheels:
		if not wh.contact:
			continue
		if Vehicle.ROAD_SURFACES.has(wh.surface):
			var sa := absf(wh.slip_angle) / wh.tire.peak_slip_angle
			var sr := absf(wh.slip_ratio) / wh.tire.peak_slip_ratio
			slip = maxf(slip, maxf(sa - 0.7, (sr - 1.0) * 0.6))
		else:
			rough = maxf(rough, 1.0)
	var spd := vehicle.speed
	_squeal = lerpf(_squeal, clampf(slip, 0.0, 1.5) * clampf(spd / 6.0, 0.0, 1.0), 1.0 - exp(-dt * 10.0))
	_rumble = lerpf(_rumble, rough * clampf(spd / 15.0, 0.0, 1.0), 1.0 - exp(-dt * 8.0))
	_wind = clampf(spd / 50.0, 0.0, 1.0)
	if vehicle.spec.turbo:
		var bmax := maxf(float(vehicle.spec.boost_target), 0.1)
		_boost = clampf(vehicle.boost / bmax, 0.0, 1.0)
		if _prev_throttle > 0.6 and vehicle.in_throttle < 0.2 and _boost > 0.4:
			_bov = 1.0
	_prev_throttle = vehicle.in_throttle
	_fill()


func _fill() -> void:
	var frames := playback.get_frames_available()
	if frames <= 0:
		return
	var fire_hz := _rpm / 60.0 * float(cylinders) * 0.5
	var step := TAU * fire_hz / RATE
	var turbo_step := TAU * (900.0 + 5200.0 * _boost) / RATE
	var load := _load
	var eng_amp := (0.22 + 0.55 * load) * clampf(_rpm / 1500.0, 0.5, 1.0)
	var h2 := 0.45 + 0.35 * load
	var h3 := 0.3 + 0.35 * load
	var h4 := 0.15 + 0.3 * load
	var h6 := 0.05 + 0.25 * load
	var noise_amp := 0.04 + 0.12 * load
	var squeal_amp := 0.32 * _squeal
	var rumble_amp := 0.35 * _rumble
	var wind_amp := 0.12 * _wind * _wind
	var turbo_amp := 0.05 * _boost * (0.4 + 0.6 * load)
	# Band-pass for squeal around ~950 Hz (state-variable filter).
	var f := 2.0 * sin(PI * (850.0 + 300.0 * _squeal) / RATE)
	var q := 0.08
	var buf := PackedVector2Array()
	buf.resize(frames)
	for i in frames:
		_ph = fmod(_ph + step, TAU * 2.0)
		var p := _ph
		var e := sin(p) + h2 * sin(2.0 * p) + h3 * sin(3.0 * p) + h4 * sin(4.0 * p) + h6 * sin(6.0 * p)
		if boxer:
			# Unequal-length headers: pulses alternate in strength -> burble.
			e *= 1.0 + 0.45 * sin(p * 0.5)
		var n := randf() * 2.0 - 1.0
		_lp_noise += (n - _lp_noise) * 0.35
		e += _lp_noise * noise_amp * 4.0
		var s := e * eng_amp * 0.35
		# Tyre squeal.
		var hp := n - _bp_low - q * _bp_band
		_bp_band += f * hp
		_bp_low += f * _bp_band
		s += _bp_band * squeal_amp
		# Gravel/grass rumble + wind (low-passed noise).
		_lp_rumble += (n - _lp_rumble) * 0.04
		s += _lp_rumble * rumble_amp * 3.0
		_lp_wind += (n - _lp_wind) * 0.08
		s += _lp_wind * wind_amp * 2.0
		# Turbo.
		if turbo_amp > 0.001:
			_ph_turbo = fmod(_ph_turbo + turbo_step, TAU)
			s += sin(_ph_turbo) * turbo_amp
		if _bov > 0.001:
			_lp_bov += (n - _lp_bov) * 0.5
			s += _lp_bov * _bov * 0.25
			_bov *= 0.99985
		if _thud > 0.001:
			_ph_thud += TAU * 55.0 / RATE
			s += sin(_ph_thud) * _thud * 0.8 + n * _thud * 0.15
			_thud *= 0.9996
		var out := tanh(s * 1.4) * volume
		buf[i] = Vector2(out, out)
	playback.push_buffer(buf)
