// Replays Assetto Corsa telemetry (drift-sim/native/actelemetry CSV) through
// DriftCore: for many short windows it starts the car in AC's recorded state
// (velocity, yaw rate, wheel spin, rpm, gear) and drives it with the recorded
// inputs, then compares where it ends up with where AC's car went. Scores how
// closely the physics matches AC - e.g. whether we over-rotate in drifts.
//   ac_replay <car spec json> <ac csv> [window_s=0.5] [steer_ratio=8.5]
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "drift/CarSpec.h"
#include "drift/Vehicle.h"

using namespace drift;

namespace {
class FlatWorld : public World {
public:
	GroundHit Raycast(const Vec3& from, const Vec3& to) override {
		GroundHit h;
		if (from.y > 0.0 && to.y <= 0.0) {
			const double t = from.y / (from.y - to.y);
			h.hit = true;
			h.point = from + (to - from) * t;
			h.normal = {0, 1, 0};
			h.surface = Surface::Asphalt;
		}
		return h;
	}
};

struct Row { std::map<std::string, double> v; double operator[](const std::string& k) const { auto it = v.find(k); return it == v.end() ? 0.0 : it->second; } };

std::vector<Row> LoadCsv(const char* path) {
	std::ifstream f(path);
	std::string line;
	std::vector<std::string> cols;
	std::vector<Row> rows;
	while (std::getline(f, line)) {
		if (line.empty() || line[0] == '#') continue;
		std::stringstream ss(line);
		std::string cell;
		if (cols.empty()) {
			while (std::getline(ss, cell, ',')) cols.push_back(cell);
			continue;
		}
		Row r;
		size_t i = 0;
		while (std::getline(ss, cell, ',') && i < cols.size()) r.v[cols[i++]] = std::atof(cell.c_str());
		rows.push_back(r);
	}
	return rows;
}

double AcDrift(const Row& r) { return r["speed_kmh"] > 3 ? std::atan2(r["vel_x"], r["vel_z"]) * 57.29578 : 0.0; }
}  // namespace

int main(int argc, char** argv) {
	if (argc < 3) {
		std::printf("usage: ac_replay <car spec json> <ac csv> [window_s] [steer_ratio]\n");
		return 2;
	}
	std::ifstream sf(argv[1]);
	std::stringstream ss;
	ss << sf.rdbuf();
	CarSpec spec;
	std::string err;
	if (!CarSpec::FromJson(ss.str(), spec, &err)) {
		std::printf("bad spec: %s\n", err.c_str());
		return 1;
	}
	const std::vector<Row> rows = LoadCsv(argv[2]);
	const double window = argc > 3 ? std::atof(argv[3]) : 0.5;
	const double ratio = argc > 4 ? std::atof(argv[4]) : 8.5;
	// Real time between rows: recorder versions before real timestamps wrote
	// t = row / 100, but AC only refreshes its shared memory at the frame rate
	// (~64 Hz on the owner's PC) - estimate it from distance travelled / speed.
	double row_dt = 0.01;
	{
		std::vector<double> est;
		for (size_t i = 10; i < rows.size(); i += 10) {
			const Row& p = rows[i - 10];
			const Row& q = rows[i];
			const double v = (p["speed_kmh"] + q["speed_kmh"]) / 7.2;
			if (v < 8.0) continue;
			est.push_back(std::hypot(q["pos_x"] - p["pos_x"], q["pos_z"] - p["pos_z"]) / v / 10.0);
		}
		if (!est.empty()) {
			std::sort(est.begin(), est.end());
			row_dt = est[est.size() / 2];
		}
		const double t_dt = rows.size() > 1 ? (rows.back()["t"] - rows.front()["t"]) / (rows.size() - 1) : 0.01;
		if (std::fabs(t_dt - row_dt) > 0.002) std::printf("rows are %.4f s apart (t column says %.4f) - using %.4f\n", row_dt, t_dt, row_dt);
		else row_dt = t_dt;
	}
	const int n = static_cast<int>(window / row_dt + 0.5);
	const int substeps = static_cast<int>(row_dt * 480.0 + 0.5);
	const double lock_deg = spec.steer_lock * 57.29578;

	FlatWorld w;
	Vehicle base(spec);
	base.Reset(Vec3(0, 0.6, 0), Mat3::Identity());
	for (int i = 0; i < 480 * 2; ++i) base.Step(1.0 / 480.0, w);   // settle on its springs

	struct Bin { double n = 0, err_drift = 0, err_drift2 = 0, err_yaw = 0, err_speed = 0; };
	std::map<int, Bin> bins;   // by |AC drift| band
	int windows = 0;
	double base2 = 0, sxy = 0, sxx = 0, syy = 0;
	for (size_t i = 0; i + n < rows.size(); i += n) {
		const Row& a = rows[i];
		const Row& b = rows[i + n];
		if (a["speed_kmh"] < 25 || a["gear"] < 1 || a["vel_z"] < 3) continue;
		bool same_gear = true;
		for (int k = 0; k < n; ++k) same_gear &= rows[i + k]["gear"] == a["gear"];
		if (!same_gear) continue;
		Vehicle v = base;
		v.in.direct_steer = true;
		v.in.steering_assist = 0;
		v.in.auto_shift = false;
		v.in.auto_clutch = false;
		v.SetGear(static_cast<int>(a["gear"]), true);
		// AC body frame: +x left, +z forward, yaw + = left. DriftCore: +x right, -z forward, yaw + = left.
		v.SetMotion(Vec3(-a["vel_x"], a["vel_y"], -a["vel_z"]), Vec3(0, a["yaw_rate"], 0));
		static const char* omegas[4] = {"omega_fl", "omega_fr", "omega_rl", "omega_rr"};
		for (int k = 0; k < 4; ++k) v.SetWheelOmega(k, std::fabs(a[omegas[k]]));
		v.SetEngineRpm(a["rpm"]);
		for (int k = 0; k < n; ++k) {
			const Row& r = rows[i + k];
			v.in.steer = std::fmax(-1.0, std::fmin(1.0, r["steer"] * 450.0 / ratio / lock_deg));
			v.in.throttle = r["gas"];
			v.in.brake = r["brake"];
			v.in.clutch = 1.0 - r["clutch"];   // AC: 1 = engaged
			v.in.handbrake = 0.0;
			for (int s = 0; s < substeps; ++s) v.Step(row_dt / substeps, w);
		}
		// Ours in AC's signs: drift + = velocity left of the nose.
		const double our_drift = -v.DriftAngleDeg();
		const double ac0 = AcDrift(a), ac1 = AcDrift(b);
		const double sgn = ac0 >= 0 ? 1.0 : -1.0;
		const Mat3& R = v.Rotation();
		const double our_yaw = R.c1.Dot(v.AngularVelocity());
		const int band = static_cast<int>(std::fabs(ac0)) / 10 * 10;
		Bin& bin = bins[band > 60 ? 60 : band];
		bin.n += 1;
		bin.err_drift += (our_drift - ac1) * sgn;   // + = we end more sideways (over-rotate) than AC
		bin.err_drift2 += (our_drift - ac1) * (our_drift - ac1);
		bin.err_yaw += (our_yaw - b["yaw_rate"]) * (b["yaw_rate"] >= 0 ? 1.0 : -1.0);
		bin.err_speed += v.Speed() * 3.6 - b["speed_kmh"];
		{ const double dp = our_drift - ac0, da = ac1 - ac0; base2 += da * da; sxy += dp * da; sxx += dp * dp; syy += da * da; }
		++windows;
	}
	std::printf("%d windows of %.2f s\n|AC drift|  n     drift error (+ = we rotate more)  RMS     yaw error (+ = faster)   speed error km/h\n", windows, window);
	double tn = 0, td = 0, td2 = 0;
	for (auto& [band, bin] : bins) {
		std::printf("%3d-%-3d  %5.0f   %+6.2f deg                        %5.2f   %+6.3f rad/s            %+5.2f\n", band, band + 10, bin.n,
			bin.err_drift / bin.n, std::sqrt(bin.err_drift2 / bin.n), bin.err_yaw / bin.n, bin.err_speed / bin.n);
		tn += bin.n; td += bin.err_drift; td2 += bin.err_drift2;
	}
	std::printf("ALL      %5.0f   %+6.2f deg                        %5.2f\n", tn, td / tn, std::sqrt(td2 / tn));
	std::printf("baseline (assume no change) RMS %.2f | corr(our change, AC change) %.2f | our change / AC change %.2f\n",
		std::sqrt(base2 / windows), sxy / std::sqrt(sxx * syy), sxy / syy);
	return 0;
}
