#include "drift/Tire.h"

#include <cmath>

namespace drift {

void TireModel::Configure(const TireParams& p) {
	p_ = p;
	p_.tail = Clamp(p_.tail, 0.3, 0.99);
	// sin(C*pi/2) = tail -> C in (1, 2); peak where C*atan(B) = pi/2.
	c_ = 2.0 - 2.0 * std::asin(p_.tail) / kPi;
	b_ = std::tan(kPi / (2.0 * c_));
}

double TireModel::Curve(double s) const { return std::sin(c_ * std::atan(b_ * s)); }

double TireModel::LoadMuScale(double fz) const {
	return std::pow((fz > 1.0 ? fz : 1.0) / p_.fz_ref, p_.load_sens - 1.0);
}

Vec2 TireModel::Compute(double slip_ratio, double slip_angle, double fz, double grip) const {
	if (fz <= 0.0) return {};
	double sx = slip_ratio / p_.peak_slip_ratio;
	double sy = slip_angle / p_.peak_slip_angle;
	double s = std::sqrt(sx * sx + sy * sy);
	if (s < 1e-6) return {};
	double c = Curve(s);
	// A spinning tyre still pushes quite well; a sliding one loses more.
	if (s > 1.0) {
		double wl = (sx * sx) / (s * s);
		c += (1.0 - c) * 0.55 * wl;
	}
	double f = c * fz * LoadMuScale(fz) * grip;
	return {p_.mu_long * f * sx / s, -p_.mu_lat * f * sy / s};
}

}  // namespace drift
