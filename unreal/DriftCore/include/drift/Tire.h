// Normalised "Magic Formula" style tyre with combined slip (port of Drift
// Lab's TireModel). Slip ratio and slip angle are normalised by their peaks,
// combined into one slip magnitude s, run through f(s) = sin(C*atan(B*s))
// (peak at s = 1, decaying toward `tail` when fully sliding) and split back
// into longitudinal / lateral parts - so wheelspin eats lateral grip.
#pragma once

#include "drift/Math.h"

namespace drift {

struct TireParams {
	double mu_long = 1.15;
	double mu_lat = 1.10;
	double peak_slip_ratio = 0.10;
	double peak_slip_angle = Deg2Rad(8.0);
	double tail = 0.80;        // fraction of peak grip left fully sliding
	double load_sens = 0.85;   // effective mu ~ (Fz/Fz_ref)^(ls-1)
	double fz_ref = 3500.0;
};

class TireModel {
public:
	TireModel() { Configure(TireParams()); }
	explicit TireModel(const TireParams& p) { Configure(p); }

	void Configure(const TireParams& p);
	const TireParams& Params() const { return p_; }

	// Normalised grip curve: 0 at s=0, 1 at s=1, -> tail as s -> inf.
	double Curve(double s) const;
	double LoadMuScale(double fz) const;

	// Returns (Fx, Fy) in newtons. slip_ratio > 0: wheel faster than ground ->
	// +Fx (drive). slip_angle > 0: patch moving to +side -> Fy negative.
	Vec2 Compute(double slip_ratio, double slip_angle, double fz, double grip = 1.0) const;

private:
	TireParams p_;
	double b_ = 0.0, c_ = 0.0;
};

}  // namespace drift
