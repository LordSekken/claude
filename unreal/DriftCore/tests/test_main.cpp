// DriftCore behaviour tests: a flat world (optionally with a wall) stands in
// for Unreal, and each test drives the car the way a player would.
#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <sstream>
#include <string>
#include <vector>

#include "drift/CarSpec.h"
#include "drift/Vehicle.h"

using namespace drift;

namespace {

int g_failed = 0;
int g_checks = 0;

#define CHECK(cond, ...)                                                   \
	do {                                                                   \
		++g_checks;                                                        \
		if (!(cond)) {                                                     \
			++g_failed;                                                    \
			std::printf("  FAIL %s:%d: %s -- ", __FILE__, __LINE__, #cond); \
			std::printf(__VA_ARGS__);                                      \
			std::printf("\n");                                             \
		}                                                                  \
	} while (0)

// Flat ground at y = 0. Optional wall: the plane z = wall_z, facing +Z
// (the car drives toward -Z and hits it). Ground beyond |x| > grass_x is grass.
class FlatWorld : public World {
public:
	bool has_wall = false;
	double wall_z = -60.0;
	double grass_x = 1e9;

	GroundHit Raycast(const Vec3& from, const Vec3& to) override {
		GroundHit h;
		if (from.y < 0.0 || to.y > 0.0) return h;
		const double t = from.y / (from.y - to.y);
		h.hit = true;
		h.point = from + (to - from) * t;
		h.normal = {0, 1, 0};
		h.surface = std::fabs(h.point.x) > grass_x ? Surface::Grass : Surface::Asphalt;
		return h;
	}

	void BodyContacts(const Vec3& c, const Mat3& r, const Vec3& he, std::vector<BodyContact>& out) override {
		if (!has_wall) return;
		// Deepest box corner past the wall plane.
		double best = 0.0;
		Vec3 best_p;
		for (int i = 0; i < 8; ++i) {
			Vec3 l((i & 1) ? he.x : -he.x, (i & 2) ? he.y : -he.y, (i & 4) ? he.z : -he.z);
			Vec3 p = c + r * l;
			const double d = wall_z - p.z;
			if (d > best) {
				best = d;
				best_p = p;
			}
		}
		if (best > 0.0) {
			BodyContact bc;
			bc.point = best_p;
			bc.normal = {0, 0, 1};
			bc.depth = best;
			bc.wall = true;
			out.push_back(bc);
		}
	}
};

CarSpec TestCar() {
	CarSpec c;
	c.name = "Test RWD";
	c.mass = 1250.0;
	c.curve_rpm = {1000, 3000, 5000, 6500, 7500};
	c.curve_nm = {180, 260, 300, 290, 250};
	c.limiter_rpm = 7400;
	c.diff_type = DiffType::Lsd;
	c.diff_power = 0.6;
	c.diff_coast = 0.4;
	return c;
}

// Car at rest on the ground at the origin facing -Z.
Vehicle Settled(const CarSpec& spec, World& w, double seconds = 2.0) {
	Vehicle v(spec);
	const double rmean = (spec.radius_f + spec.radius_r) * 0.5;
	v.Reset(Vec3(0, rmean + 0.05, 0), Mat3::Identity());
	v.Step(0.0, w);
	for (int i = 0; i < static_cast<int>(seconds * 60); ++i) v.Step(1.0 / 60.0, w);
	return v;
}

void Run(double seconds, Vehicle& v, World& w, const std::function<void(double)>& each = nullptr, double dt = 1.0 / 60.0) {
	const int n = static_cast<int>(std::lround(seconds / dt));
	for (int i = 0; i < n; ++i) {
		if (each) each(i * dt);
		v.Step(dt, w);
	}
}

void TestJsonParse() {
	JsonValue v;
	std::string err;
	CHECK(JsonValue::Parse(R"({"a": 1.5, "b": [1, 2, 3], "c": {"d": "xé"}, "e": true, "f": null})", v, &err), "%s", err.c_str());
	CHECK(v.Num("a", 0) == 1.5, "a=%g", v.Num("a", 0));
	CHECK(v["b"].arr.size() == 3, "b size");
	CHECK(v["c"].Str("d", "") == "x\xc3\xa9", "utf8 escape");
	CHECK(v.Bool("e", false), "bool");
	CHECK(v["missing"]["deeper"].Num("x", 7.0) == 7.0, "missing keys fall back");
	CHECK(!JsonValue::Parse("{\"a\": }", v, &err), "bad json rejected");
	CHECK(!JsonValue::Parse("[1, 2] x", v, &err), "trailing junk rejected");
}

void TestSpecFromJson() {
	const std::string text = R"({
		"spec": {
			"name": "Legacy RWD", "mass": 1380, "wheelbase": 2.63,
			"drivetrain": "RWD", "diff_type": "welded",
			"curve_rpm": [1000, 4000, 7000], "curve_nm": [150, 300, 260],
			"gears": [3.45, 2.06, 1.45, 1.09, 0.78], "final": 4.44,
			"turbo": true, "boost_target": 1.1,
			"tire_r": {"mu_lat": 1.0, "peak_slip_angle_deg": 7.0},
			"eye": [-0.37, 0.71, 0.15]
		}
	})";
	CarSpec c;
	std::string err;
	CHECK(CarSpec::FromJson(text, c, &err), "%s", err.c_str());
	CHECK(c.name == "Legacy RWD", "name %s", c.name.c_str());
	CHECK(c.mass == 1380.0 && c.wheelbase == 2.63, "mass/wb");
	CHECK(c.layout == Layout::RWD && c.diff_type == DiffType::Welded, "layout/diff");
	CHECK(c.gears.size() == 5 && c.final_drive == 4.44, "gears/final");
	CHECK(c.turbo && c.boost_target == 1.1, "turbo");
	CHECK(std::fabs(c.tire_r.peak_slip_angle - Deg2Rad(7.0)) < 1e-9, "tire angle");
	CHECK(c.has_eye && c.eye[0] == -0.37, "eye");
	CHECK(std::fabs(c.TorqueAt(5500) - 280.0) < 1e-9, "torque interp %g", c.TorqueAt(5500));
	CHECK(c.tire_f.mu_lat == TireParams().mu_lat, "untouched fields keep defaults");
}

void TestTireCurve() {
	TireModel t;
	CHECK(std::fabs(t.Curve(0.0)) < 1e-9, "zero slip, zero force");
	CHECK(std::fabs(t.Curve(1.0) - 1.0) < 1e-6, "peak at s=1: %g", t.Curve(1.0));
	CHECK(t.Curve(8.0) < 0.95 && t.Curve(8.0) > 0.6, "falls toward tail: %g", t.Curve(8.0));
	Vec2 drive = t.Compute(0.1, 0.0, 3500);
	CHECK(drive.x > 3000 && std::fabs(drive.y) < 1e-6, "pure drive %g %g", drive.x, drive.y);
	Vec2 corner = t.Compute(0.0, Deg2Rad(8.0), 3500);
	CHECK(corner.y < -3000, "slip angle + -> Fy - (%g)", corner.y);
	Vec2 both = t.Compute(0.5, Deg2Rad(8.0), 3500);
	CHECK(std::fabs(both.y) < std::fabs(corner.y) * 0.6, "wheelspin eats lateral grip %g vs %g", both.y, corner.y);
}

void TestSettles() {
	FlatWorld w;
	CarSpec spec = TestCar();
	Vehicle v = Settled(spec, w, 3.0);
	double fz = 0.0;
	int contacts = 0;
	for (int i = 0; i < 4; ++i) {
		fz += v.Wheel(i).fz;
		contacts += v.Wheel(i).contact ? 1 : 0;
	}
	CHECK(contacts == 4, "all wheels down (%d)", contacts);
	CHECK(std::fabs(fz - spec.mass * Vehicle::kGravity) < spec.mass * 0.03, "wheel loads carry the weight: %g", fz);
	CHECK(v.Speed() < 0.05, "at rest: %g", v.Speed());
	CHECK(std::fabs(v.Origin().x) < 0.02 && std::fabs(v.Origin().z) < 0.05, "doesn't creep: %g %g", v.Origin().x, v.Origin().z);
	CHECK(v.Up().y > 0.999, "level");
	// Front heavier than rear for a car with COM ahead of the middle.
	double front = v.Wheel(0).fz + v.Wheel(1).fz, rear = v.Wheel(2).fz + v.Wheel(3).fz;
	double expect_front = spec.mass * Vehicle::kGravity * (1.0 - spec.com_from_front / spec.wheelbase);
	CHECK(std::fabs(front - expect_front) < spec.mass * 0.4, "weight distribution f %g r %g (expect f %g)", front, rear, expect_front);
}

void TestAccelerates() {
	FlatWorld w;
	Vehicle v = Settled(TestCar(), w);
	v.in.throttle = 1.0;
	v.in.auto_shift = true;
	double t60 = -1.0;
	Run(12.0, v, w, [&](double t) {
		if (t60 < 0.0 && v.Speed() > 26.8) t60 = t;   // 60 mph
	});
	CHECK(t60 > 3.0 && t60 < 11.0, "0-60 mph in %.2f s", t60);
	CHECK(v.Origin().z < -100.0, "went forward (-Z): z=%g", v.Origin().z);
	CHECK(std::fabs(v.Origin().x) < 2.0, "straight: x=%g", v.Origin().x);
	CHECK(v.Gear() >= 3, "auto shifted up: gear %d", v.Gear());
	CHECK(v.Up().y > 0.99, "stays upright");
}

void TestBrakes() {
	FlatWorld w;
	Vehicle v = Settled(TestCar(), w);
	v.in.throttle = 1.0;
	v.in.auto_shift = true;
	Run(6.0, v, w);
	const double v0 = v.Speed();
	const double z0 = v.Origin().z;
	v.in.throttle = 0.0;
	v.in.brake = 1.0;
	double t = 0.0;
	while (v.Speed() > 0.5 && t < 10.0) {
		v.Step(1.0 / 60.0, w);
		t += 1.0 / 60.0;
	}
	const double dist = z0 - v.Origin().z;
	// Ideal on ~1.1 mu: v^2 / (2 mu g). Allow ABS-less inefficiency.
	const double ideal = v0 * v0 / (2.0 * 1.1 * 9.81);
	CHECK(v.Speed() < 0.5, "stopped (%g)", v.Speed());
	CHECK(dist > ideal * 0.9 && dist < ideal * 2.0, "braking distance %g from %g m/s (ideal %g)", dist, v0, ideal);
}

void TestTurnsRight() {
	FlatWorld w;
	Vehicle v = Settled(TestCar(), w);
	v.in.throttle = 0.6;
	v.in.auto_shift = true;
	Run(4.0, v, w);
	v.in.throttle = 0.25;
	v.in.steer = 0.4;
	const double x0 = v.Origin().x;
	Run(1.5, v, w);
	CHECK(v.Origin().x > x0 + 2.0, "steer + goes right: dx=%g", v.Origin().x - x0);
	CHECK(v.Forward().x > 0.2, "heading turned right: fwd.x=%g", v.Forward().x);
	CHECK(v.AngularVelocity().y < 0.0, "yaw rate clockwise from above: %g", v.AngularVelocity().y);
	CHECK(v.Wheel(1).steer > v.Wheel(0).steer, "ackermann: inner (right) wheel turns more");
}

void TestPowerOversteer() {
	FlatWorld w;
	Vehicle v = Settled(TestCar(), w);
	v.in.throttle = 0.5;
	v.in.auto_shift = false;
	Run(3.0, v, w);   // first gear ~ 12-15 m/s
	if (v.Gear() == 1) v.ShiftUp();
	Run(1.0, v, w);
	// Flick in, handbrake tap, then full throttle with counter-steer.
	double max_drift = 0.0;
	v.in.steer = 0.8;
	v.in.handbrake = 1.0;
	v.in.throttle = 1.0;
	Run(0.4, v, w, [&](double) { max_drift = std::fmax(max_drift, std::fabs(v.DriftAngleDeg())); });
	v.in.handbrake = 0.0;
	v.in.steer = -0.3;
	double sustained = 0.0;
	Run(2.0, v, w, [&](double) {
		max_drift = std::fmax(max_drift, std::fabs(v.DriftAngleDeg()));
		if (std::fabs(v.DriftAngleDeg()) > 15.0) sustained += 1.0 / 60.0;
	});
	CHECK(max_drift > 20.0, "gets sideways: max drift %g deg", max_drift);
	CHECK(sustained > 0.4, "holds a slide on power: %g s past 15 deg", sustained);
	CHECK(v.Up().y > 0.9, "doesn't roll over");
}

void TestWall() {
	FlatWorld w;
	w.has_wall = true;
	w.wall_z = -40.0;
	Vehicle v = Settled(TestCar(), w);
	v.in.throttle = 1.0;
	v.in.auto_shift = true;
	double max_impact = 0.0, deepest = 1e9;
	bool touched = false;
	Run(8.0, v, w, [&](double) {
		max_impact = std::fmax(max_impact, v.LastImpact());
		touched = touched || v.WallContact();
		// Furthest body corner toward the wall (the car may have spun).
		const Vec3 he = v.CollisionHalfExtents();
		for (int k = 0; k < 8; ++k) {
			Vec3 l((k & 1) ? he.x : -he.x, (k & 2) ? he.y : -he.y, (k & 4) ? he.z : -he.z);
			deepest = std::fmin(deepest, (v.Origin() + v.Rotation() * (v.CollisionCenterLocal() + l)).z);
		}
	});
	CHECK(touched, "hit the wall");
	CHECK(deepest > w.wall_z - 0.25, "doesn't tunnel through: deepest corner z %g", deepest);
	CHECK(max_impact > 5.0, "impact registered: %g m/s", max_impact);
	CHECK(v.Up().y > 0.9, "upright after the hit");
}

void TestGrassSlowsCar() {
	FlatWorld road, grass;
	grass.grass_x = -1.0;   // everything is grass
	CarSpec spec = TestCar();
	Vehicle a = Settled(spec, road), b = Settled(spec, grass);
	for (Vehicle* v : {&a, &b}) {
		v->in.throttle = 1.0;
		v->in.auto_shift = true;
	}
	Run(6.0, a, road);
	Run(6.0, b, grass);
	CHECK(b.Speed() < a.Speed() * 0.92, "grass slower: %g vs %g", b.Speed(), a.Speed());
	CHECK(b.OnRoadWheels() == 0 && a.OnRoadWheels() == 4, "surface tracking");
}

void TestFrameRateIndependent() {
	FlatWorld w;
	CarSpec spec = TestCar();
	Vehicle a = Settled(spec, w), b = Settled(spec, w);
	for (Vehicle* v : {&a, &b}) {
		v->in.throttle = 0.8;
		v->in.auto_shift = true;
	}
	Run(3.0, a, w, nullptr, 1.0 / 30.0);
	Run(3.0, b, w, nullptr, 1.0 / 144.0);
	const double dz = std::fabs(a.Origin().z - b.Origin().z);
	CHECK(dz < 1.5, "30 vs 144 fps differ by %g m", dz);
}

void TestFfbDirection() {
	FlatWorld w;
	Vehicle v = Settled(TestCar(), w);
	v.in.throttle = 0.5;
	v.in.auto_shift = true;
	Run(4.0, v, w);
	v.in.throttle = 0.3;
	v.in.direct_steer = true;
	v.in.steer = 0.15;
	double sum = 0.0;
	Run(0.8, v, w, [&](double) { sum += v.FfbForce(); });
	// Turning right, self-aligning torque pulls the wheel back left.
	CHECK(sum < 0.0, "aligning torque opposes steer: %g", sum);
	CHECK(v.FfbDamper() < 0.2, "light damper at speed");
}

void TestFfbParked() {
	// Stopped with the wheel turned: no tyre torque shoving the wheel, a centring spring instead.
	FlatWorld w;
	Vehicle v = Settled(TestCar(), w);
	v.in.direct_steer = true;
	v.in.steer = 0.4;
	double worst = 0.0;
	Run(1.0, v, w, [&](double) { worst = std::fmax(worst, std::fabs(v.FfbForce())); });
	CHECK(worst < 0.05, "parked steering force %g", worst);
	CHECK(v.FfbSpring() > 0.1, "parked spring %g", v.FfbSpring());
}

void TestFfbDriftSelfSteer() {
	// Sliding with the wheel held straight, the rim pulls firmly toward countersteer.
	FlatWorld w;
	Vehicle v = Settled(TestCar(), w);
	v.in.throttle = 0.5;
	v.in.direct_steer = true;
	Run(3.0, v, w);
	if (v.Gear() == 1) v.ShiftUp();
	Run(1.0, v, w);
	v.in.steer = 0.8;
	v.in.handbrake = 1.0;
	v.in.throttle = 1.0;
	Run(0.4, v, w);
	v.in.handbrake = 0.0;
	v.in.steer = -0.3;
	Run(0.6, v, w);
	v.in.steer = 0.0;
	double weakest = 1.0;
	int samples = 0;
	Run(1.2, v, w, [&](double) {
		if (std::fabs(v.DriftAngleDeg()) < 10.0) return;
		++samples;
		// +drift: velocity right of the nose, so countersteer is to the right (+).
		weakest = std::fmin(weakest, v.FfbForce() * (v.DriftAngleDeg() > 0 ? 1.0 : -1.0));
	});
	CHECK(samples > 30, "got sideways (%d samples past 10 deg)", samples);
	// AC (owner's runs): ~0.5 into countersteer while sliding, scaled by front load.
	CHECK(weakest > 0.3, "pulls into countersteer while sliding: weakest %g", weakest);
}

void TestFfbLightOnCentre() {
	// AC: ~0.1 at half a degree of front slip, ~0.4 at 2.5 deg - light around centre.
	FlatWorld w;
	Vehicle v = Settled(TestCar(), w);
	v.in.direct_steer = true;
	v.in.throttle = 0.5;
	v.in.auto_shift = true;
	Run(4.0, v, w);
	v.in.throttle = 0.3;
	v.in.steer = 0.01;
	Run(1.0, v, w);
	CHECK(std::fabs(v.FfbForce()) < 0.2, "near-straight steering is light: %g", v.FfbForce());
}

void TestStallRestart() {
	FlatWorld w;
	Vehicle v = Settled(TestCar(), w);
	v.in.auto_clutch = false;
	v.in.clutch = 0.0;   // in gear, clutch out, brakes on: stalls
	v.in.brake = 1.0;
	Run(2.0, v, w);
	CHECK(v.Rpm() < 200.0, "stalled: %g rpm", v.Rpm());
	v.in.clutch = 1.0;   // clutch in: the starter brings it back
	Run(1.0, v, w);
	CHECK(v.Rpm() > v.Spec().idle_rpm * 0.8, "restarted: %g rpm", v.Rpm());
}

// A world with one flat floor plus a fixed overlap against "another car".
class CarContactWorld : public FlatWorld {
public:
	Vec3 other_vel;
	void BodyContacts(const Vec3& center, const Mat3&, const Vec3&, std::vector<BodyContact>& out) override {
		BodyContact c;
		c.point = center + Vec3(0.8, 0, 0);
		c.normal = Vec3(-1, 0, 0);   // pushed to the left
		c.depth = 0.01;
		c.wall = false;
		c.other_vel = other_vel;
		c.other_mass = 1200.0;
		out.push_back(c);
	}
};

void TestCarContact() {
	// Side by side at the same speed: no bounce. Other car sliding into us: we get pushed.
	for (int moving = 0; moving < 2; ++moving) {
		CarContactWorld w;
		Vehicle v = Settled(TestCar(), w, 0.5);
		w.other_vel = moving ? Vec3(-4.0, 0, 0) : Vec3(0, 0, 0);
		Run(0.05, v, w);
		const double side = v.Velocity().x;
		if (moving) CHECK(side < -0.5, "car sliding into us pushes us sideways: %g m/s", side);
		else CHECK(std::fabs(side) < 0.5, "touching a stationary car barely moves us: %g m/s", side);
	}
}

void TestSetGear() {
	Vehicle v(TestCar());
	v.SetGear(3);
	CHECK(v.Gear() == 3, "gear %d", v.Gear());
	v.SetGear(-1);
	CHECK(v.Gear() == -1, "reverse");
	v.SetGear(99);
	CHECK(v.Gear() == v.GearCount(), "clamped to top gear %d", v.Gear());
	v.SetGear(0);
	CHECK(v.Gear() == 0, "neutral");
}

// Rolling at 20 m/s in `gear` with the engine on that gear's speed.
Vehicle Rolling(const CarSpec& spec, World& w, int gear) {
	Vehicle v = Settled(spec, w, 1.0);
	v.SetGear(gear, true);
	v.SetMotion(Vec3(0, 0, -20.0), Vec3());
	for (int i = 0; i < 4; ++i) v.SetWheelOmega(i, 20.0 / v.Wheel(i).radius);
	v.SetEngineRpm(20.0 / v.Wheel(2).radius * spec.gears[static_cast<size_t>(gear - 1)] * spec.final_drive * 60.0 / kTau);
	return v;
}

void TestSynchro() {
	FlatWorld w;
	const double dt = 1.0 / 240.0;
	// Clutch in, 3rd -> 2nd at 20 m/s (~1800 rpm to spin up): the synchro takes ~0.1 s.
	{
		Vehicle v = Rolling(TestCar(), w, 3);
		v.in.auto_clutch = false;
		v.in.clutch = 1.0;
		v.SelectGear(2);
		CHECK(v.Gear() == 0, "out of gear at once (%d)", v.Gear());
		double t_in = -1.0;
		Run(0.6, v, w, [&](double t) { if (t_in < 0.0 && v.Gear() == 2) t_in = t; }, dt);
		CHECK(t_in > 0.05 && t_in < 0.25, "synchro downshift took %.3f s", t_in);
		CHECK(!v.Grinding(), "no grind with the clutch in");
	}
	// Clutch out, 2nd -> 4th with ~2800 rpm mismatch: the box refuses (grind), stays neutral.
	{
		Vehicle v = Rolling(TestCar(), w, 2);
		v.in.auto_clutch = false;
		v.in.clutch = 0.0;
		v.SelectGear(4);
		Run(0.1, v, w, nullptr, dt);
		CHECK(v.Gear() == 0 && v.Grinding(), "clutchless unmatched shift refused (gear %d grind %d)", v.Gear(), v.Grinding() ? 1 : 0);
		v.in.clutch = 1.0;
		Run(0.3, v, w, nullptr, dt);
		CHECK(v.Gear() == 4 && !v.Grinding(), "goes in once the clutch is pressed (gear %d)", v.Gear());
	}
	// Clutch out but rev-matched (3rd -> 3rd's speed in 4th +- window): goes in.
	{
		CarSpec spec = TestCar();
		spec.shift_window_rpm = 1200.0;
		Vehicle v = Rolling(spec, w, 3);
		v.in.auto_clutch = false;
		v.SelectGear(4);
		Run(0.1, v, w, nullptr, dt);
		CHECK(v.Gear() == 4, "rev-matched clutchless upshift (gear %d)", v.Gear());
	}
	// Dog box: clutchless 3rd -> 4th in a few ms.
	{
		CarSpec spec = TestCar();
		spec.dog_box = true;
		Vehicle v = Rolling(spec, w, 3);
		v.in.auto_clutch = false;
		v.SelectGear(4);
		double t_in = -1.0;
		Run(0.2, v, w, [&](double t) { if (t_in < 0.0 && v.Gear() == 4) t_in = t; }, dt);
		CHECK(t_in >= 0.0 && t_in < 0.03, "dog box shift took %.3f s", t_in);
	}
}

void TestExampleSpecs() {
	const char* files[] = {"legacy_rwd.json", "outback98_stock.json", "s14_sr20_drift.json", "miata_na_drift.json"};
	for (const char* f : files) {
		std::ifstream in(std::string(DRIFT_TEST_DATA) + "/" + f);
		if (!in) {
			CHECK(false, "missing %s", f);
			continue;
		}
		std::stringstream ss;
		ss << in.rdbuf();
		CarSpec spec;
		std::string err;
		CHECK(CarSpec::FromJson(ss.str(), spec, &err), "%s: %s", f, err.c_str());
		FlatWorld w;
		Vehicle v = Settled(spec, w);
		v.in.throttle = 1.0;
		v.in.auto_shift = true;
		Run(8.0, v, w);
		CHECK(v.Speed() > 25.0 && v.Up().y > 0.99, "%s drives: %g m/s", f, v.Speed());
	}
}

}  // namespace

int main() {
	struct T {
		const char* name;
		void (*fn)();
	} tests[] = {
		{"json parse", TestJsonParse},
		{"spec from json", TestSpecFromJson},
		{"tire curve", TestTireCurve},
		{"settles", TestSettles},
		{"accelerates", TestAccelerates},
		{"brakes", TestBrakes},
		{"turns right", TestTurnsRight},
		{"power oversteer", TestPowerOversteer},
		{"wall", TestWall},
		{"grass", TestGrassSlowsCar},
		{"frame rate", TestFrameRateIndependent},
		{"ffb direction", TestFfbDirection},
		{"ffb parked", TestFfbParked},
		{"ffb drift self-steer", TestFfbDriftSelfSteer},
		{"ffb light on centre", TestFfbLightOnCentre},
		{"stall restart", TestStallRestart},
		{"car contact", TestCarContact},
		{"set gear", TestSetGear},
		{"synchro", TestSynchro},
		{"example specs", TestExampleSpecs},
	};
	for (const T& t : tests) {
		const int before = g_failed;
		t.fn();
		std::printf("%s %s\n", g_failed == before ? "ok  " : "FAIL", t.name);
	}
	std::printf("%d checks, %d failed\n", g_checks, g_failed);
	return g_failed == 0 ? 0 : 1;
}
