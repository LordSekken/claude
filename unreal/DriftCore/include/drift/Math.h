// DriftCore - engine-independent drift car physics.
// Small vector/matrix helpers. Frame (same as Drift Lab): right-handed,
// +X right, +Y up, -Z forward. Units: metres, kilograms, seconds, radians.
#pragma once

#include <cmath>

namespace drift {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTau = 2.0 * kPi;

inline double Clamp(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline double Lerp(double a, double b, double t) { return a + (b - a) * t; }
inline double Sign(double v) { return v > 0.0 ? 1.0 : (v < 0.0 ? -1.0 : 0.0); }
inline double MoveToward(double from, double to, double delta) {
	if (std::fabs(to - from) <= delta) return to;
	return from + Sign(to - from) * delta;
}
inline double SmoothStep(double e0, double e1, double x) {
	if (e1 == e0) return x < e0 ? 0.0 : 1.0;
	double t = Clamp((x - e0) / (e1 - e0), 0.0, 1.0);
	return t * t * (3.0 - 2.0 * t);
}
inline double Deg2Rad(double d) { return d * kPi / 180.0; }
inline double Rad2Deg(double r) { return r * 180.0 / kPi; }

struct Vec2 {
	double x = 0.0, y = 0.0;
};

struct Vec3 {
	double x = 0.0, y = 0.0, z = 0.0;
	Vec3() = default;
	Vec3(double ax, double ay, double az) : x(ax), y(ay), z(az) {}
	Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
	Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
	Vec3 operator-() const { return {-x, -y, -z}; }
	Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
	Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }
	Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
	Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
	Vec3& operator*=(double s) { x *= s; y *= s; z *= s; return *this; }
	double Dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
	Vec3 Cross(const Vec3& o) const { return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x}; }
	double Length() const { return std::sqrt(x * x + y * y + z * z); }
	double LengthSq() const { return x * x + y * y + z * z; }
	Vec3 Normalized() const {
		double l = Length();
		return l > 1e-12 ? *this / l : Vec3(0, 0, 0);
	}
};
inline Vec3 operator*(double s, const Vec3& v) { return v * s; }

// 3x3 rotation/orientation matrix stored as columns: the body's local X, Y, Z
// axes expressed in world space.
struct Mat3 {
	Vec3 c0{1, 0, 0}, c1{0, 1, 0}, c2{0, 0, 1};
	Vec3 operator*(const Vec3& v) const { return c0 * v.x + c1 * v.y + c2 * v.z; }
	// Transpose-multiply: world -> local.
	Vec3 TMul(const Vec3& v) const { return {c0.Dot(v), c1.Dot(v), c2.Dot(v)}; }
	Mat3 operator*(const Mat3& o) const {
		Mat3 r;
		r.c0 = (*this) * o.c0;
		r.c1 = (*this) * o.c1;
		r.c2 = (*this) * o.c2;
		return r;
	}
	static Mat3 Identity() { return Mat3(); }
	// Rotation of `angle` radians about unit `axis`.
	static Mat3 AxisAngle(const Vec3& axis, double angle) {
		Vec3 a = axis.Normalized();
		double c = std::cos(angle), s = std::sin(angle), t = 1.0 - c;
		Mat3 m;
		m.c0 = {t * a.x * a.x + c, t * a.x * a.y + s * a.z, t * a.x * a.z - s * a.y};
		m.c1 = {t * a.x * a.y - s * a.z, t * a.y * a.y + c, t * a.y * a.z + s * a.x};
		m.c2 = {t * a.x * a.z + s * a.y, t * a.y * a.z - s * a.x, t * a.z * a.z + c};
		return m;
	}
	// Re-orthonormalise (Gram-Schmidt) to stop drift during integration.
	void Orthonormalize() {
		c0 = c0.Normalized();
		c1 = (c1 - c0 * c0.Dot(c1)).Normalized();
		c2 = c0.Cross(c1);
	}
};

// Rotates vector v about unit axis by angle (Rodrigues).
inline Vec3 Rotated(const Vec3& v, const Vec3& axis, double angle) {
	Vec3 a = axis.Normalized();
	double c = std::cos(angle), s = std::sin(angle);
	return v * c + a.Cross(v) * s + a * (a.Dot(v) * (1.0 - c));
}

}  // namespace drift
