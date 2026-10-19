#include "drift/CarSpec.h"

namespace drift {

double CarSpec::TorqueAt(double rpm) const {
	size_t n = curve_rpm.size();
	if (n == 0 || curve_nm.size() != n) return 0.0;
	if (rpm <= curve_rpm[0]) return curve_nm[0] * Clamp(rpm / (curve_rpm[0] > 1.0 ? curve_rpm[0] : 1.0), 0.0, 1.0);
	for (size_t i = 1; i < n; ++i) {
		if (rpm <= curve_rpm[i]) {
			double t = (rpm - curve_rpm[i - 1]) / (curve_rpm[i] - curve_rpm[i - 1]);
			return Lerp(curve_nm[i - 1], curve_nm[i], t);
		}
	}
	return curve_nm[n - 1];
}

static void ReadTire(const JsonValue& t, TireParams& p) {
	if (!t.IsObject()) return;
	p.mu_long = t.Num("mu_long", p.mu_long);
	p.mu_lat = t.Num("mu_lat", p.mu_lat);
	p.peak_slip_ratio = t.Num("peak_slip_ratio", p.peak_slip_ratio);
	p.peak_slip_angle = Deg2Rad(t.Num("peak_slip_angle_deg", Rad2Deg(p.peak_slip_angle)));
	p.tail = t.Num("tail", p.tail);
	p.load_sens = t.Num("load_sens", p.load_sens);
	p.fz_ref = t.Num("fz_ref", p.fz_ref);
}

static std::vector<double> ReadNums(const JsonValue& a) {
	std::vector<double> out;
	if (!a.IsArray()) return out;
	for (const JsonValue& e : a.arr) {
		if (e.IsNumber()) out.push_back(e.num);
	}
	return out;
}

void CarSpec::FromJsonValue(const JsonValue& root, CarSpec& c) {
	const JsonValue& v = root.Has("spec") ? root["spec"] : root;
	c.name = v.Str("name", c.name);
	c.mass = v.Num("mass", c.mass);
	c.length = v.Num("length", c.length);
	c.yaw_inertia_scale = v.Num("yaw_inertia_scale", c.yaw_inertia_scale);
	c.width = v.Num("width", c.width);
	c.height = v.Num("height", c.height);
	c.wheelbase = v.Num("wheelbase", c.wheelbase);
	c.track_f = v.Num("track_f", c.track_f);
	c.track_r = v.Num("track_r", c.track_r);
	c.com_from_front = v.Num("com_from_front", c.com_from_front);
	c.cg_height = v.Num("cg_height", c.cg_height);
	c.ride_offset = v.Num("ride_offset", c.ride_offset);
	c.radius_f = v.Num("radius_f", c.radius_f);
	c.radius_r = v.Num("radius_r", c.radius_r);
	c.wheel_inertia_f = v.Num("wheel_inertia_f", c.wheel_inertia_f);
	c.wheel_inertia_r = v.Num("wheel_inertia_r", c.wheel_inertia_r);
	c.spring_f = v.Num("spring_f", c.spring_f);
	c.spring_r = v.Num("spring_r", c.spring_r);
	c.bump_f = v.Num("bump_f", c.bump_f);
	c.rebound_f = v.Num("rebound_f", c.rebound_f);
	c.bump_r = v.Num("bump_r", c.bump_r);
	c.rebound_r = v.Num("rebound_r", c.rebound_r);
	c.arb_f = v.Num("arb_f", c.arb_f);
	c.arb_r = v.Num("arb_r", c.arb_r);
	c.travel = v.Num("travel", c.travel);
	ReadTire(v["tire_f"], c.tire_f);
	ReadTire(v["tire_r"], c.tire_r);
	c.steer_lock = v.Num("steer_lock", c.steer_lock);
	c.ackermann = v.Num("ackermann", c.ackermann);
	c.brake_torque = v.Num("brake_torque", c.brake_torque);
	c.brake_bias = v.Num("brake_bias", c.brake_bias);
	c.handbrake_torque = v.Num("handbrake_torque", c.handbrake_torque);
	std::string layout = v.Str("drivetrain", "RWD");
	c.layout = layout == "FWD" ? Layout::FWD : (layout == "AWD" ? Layout::AWD : Layout::RWD);
	c.awd_front_split = v.Num("awd_front_split", c.awd_front_split);
	c.center_lock_nm = v.Num("center_lock_nm", c.center_lock_nm);
	std::string diff = v.Str("diff_type", "lsd");
	c.diff_type = diff == "open" ? DiffType::Open : ((diff == "welded" || diff == "locked") ? DiffType::Welded : DiffType::Lsd);
	c.diff_power = v.Num("diff_power", c.diff_power);
	c.diff_coast = v.Num("diff_coast", c.diff_coast);
	c.diff_preload = v.Num("diff_preload", c.diff_preload);
	std::vector<double> rpm = ReadNums(v["curve_rpm"]);
	std::vector<double> nm = ReadNums(v["curve_nm"]);
	if (!rpm.empty() && rpm.size() == nm.size()) {
		c.curve_rpm = rpm;
		c.curve_nm = nm;
	}
	c.idle_rpm = v.Num("idle_rpm", c.idle_rpm);
	c.limiter_rpm = v.Num("limiter_rpm", c.limiter_rpm);
	c.limiter_hz = v.Num("limiter_hz", c.limiter_hz);
	c.engine_inertia = v.Num("engine_inertia", c.engine_inertia);
	c.coast_nm = v.Num("coast_nm", c.coast_nm);
	c.turbo = v.Bool("turbo", c.turbo);
	c.boost_stock = v.Num("boost_stock", c.boost_stock);
	c.boost_target = v.Num("boost_target", c.boost_target);
	c.spool_rpm = v.Num("spool_rpm", c.spool_rpm);
	c.turbo_lag_s = v.Num("turbo_lag_s", c.turbo_lag_s);
	c.power_mult = v.Num("power_mult", c.power_mult);
	std::vector<double> gears = ReadNums(v["gears"]);
	if (!gears.empty()) c.gears = gears;
	c.reverse = v.Num("reverse", c.reverse);
	c.final_drive = v.Num("final", c.final_drive);
	c.shift_time = v.Num("shift_time", c.shift_time);
	c.dog_box = v.Bool("dog_box", c.dog_box);
	c.synchro_nm = v.Num("synchro_nm", c.synchro_nm);
	c.gearbox_inertia = v.Num("gearbox_inertia", c.gearbox_inertia);
	c.shift_window_rpm = v.Num("shift_window_rpm", c.shift_window_rpm);
	c.dog_window_rpm = v.Num("dog_window_rpm", c.dog_window_rpm);
	c.clutch_nm = v.Num("clutch_nm", c.clutch_nm);
	c.cda = v.Num("cda", c.cda);
	const JsonValue& eye = v["eye"];
	if (eye.IsArray() && eye.arr.size() == 3) {
		c.has_eye = true;
		for (int k = 0; k < 3; ++k) c.eye[k] = eye.arr[static_cast<size_t>(k)].num;
	}
	if (c.curve_rpm.empty()) {
		// No curve: a gentle generic 150 kW four-cylinder.
		c.curve_rpm = {1000, 2500, 4000, 5500, 7000};
		c.curve_nm = {150, 210, 240, 225, 190};
	}
}

bool CarSpec::FromJson(const std::string& text, CarSpec& out, std::string* err) {
	JsonValue v;
	if (!JsonValue::Parse(text, v, err)) return false;
	FromJsonValue(v, out);
	return true;
}

}  // namespace drift
