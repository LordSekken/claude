// Assembled car description - the same fields Drift Lab's CarBuilder
// produces (Drift Lab exports it as JSON), so builds made in Drift Lab's
// garage (e.g. the RWD EJ205 Legacy) drive the same here.
#pragma once

#include <string>
#include <vector>

#include "drift/Json.h"
#include "drift/Tire.h"

namespace drift {

enum class Layout { RWD, FWD, AWD };
enum class DiffType { Open, Lsd, Welded };

struct CarSpec {
	std::string name = "Car";
	double mass = 1300.0;
	double length = 4.4, width = 1.75, height = 1.35;
	double yaw_inertia_scale = 1.0;   // tuning: multiplies the box-formula yaw inertia
	double wheelbase = 2.5;
	double track_f = 1.48, track_r = 1.48;
	double com_from_front = 1.2;   // m behind the front axle
	double cg_height = 0.48;       // above ground
	double ride_offset = 0.0;
	double radius_f = 0.31, radius_r = 0.31;
	double wheel_inertia_f = 1.4, wheel_inertia_r = 1.4;
	double spring_f = 35000.0, spring_r = 30000.0;   // N/m wheel rate
	double bump_f = 2800.0, rebound_f = 4500.0;
	double bump_r = 2500.0, rebound_r = 4000.0;
	double arb_f = 20000.0, arb_r = 10000.0;          // N/m
	double travel = 0.20;
	TireParams tire_f, tire_r;
	double steer_lock = Deg2Rad(35.0);
	double ackermann = 0.6;
	double brake_torque = 3200.0;
	double brake_bias = 0.65;
	double handbrake_torque = 1100.0;
	Layout layout = Layout::RWD;
	double awd_front_split = 0.4;
	double center_lock_nm = 250.0;
	DiffType diff_type = DiffType::Lsd;
	double diff_power = 0.5, diff_coast = 0.3, diff_preload = 60.0;
	std::vector<double> curve_rpm, curve_nm;
	double idle_rpm = 900.0;
	double limiter_rpm = 7000.0;
	double engine_inertia = 0.18;
	double coast_nm = 30.0;
	bool turbo = false;
	double boost_stock = 0.0, boost_target = 0.0;
	double spool_rpm = 3200.0;
	double turbo_lag_s = 0.45;
	double power_mult = 1.0;
	std::vector<double> gears{3.5, 2.1, 1.4, 1.0, 0.8};
	double reverse = 3.4;
	double final_drive = 4.1;
	double shift_time = 0.18;
	double clutch_nm = 600.0;
	double cda = 0.7;
	bool has_eye = false;
	double eye[3] = {0, 0, 0};

	// Torque (N m) on the full-throttle curve at `rpm` (linear interpolation).
	double TorqueAt(double rpm) const;

	// Loads Drift Lab's exported spec JSON ({"spec": {...}} or the spec
	// itself). Missing fields keep their defaults. False + err on bad JSON.
	static bool FromJson(const std::string& text, CarSpec& out, std::string* err = nullptr);
	static void FromJsonValue(const JsonValue& v, CarSpec& out);
};

}  // namespace drift
