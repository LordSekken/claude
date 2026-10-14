// Drift car simulation (port of Drift Lab's Vehicle): raycast suspension,
// combined-slip tyres, engine (curve, turbo lag, limiter, engine braking) ->
// clutch -> gearbox -> centre/axle diffs (open / clutch LSD / welded) ->
// wheels, plus the rigid body itself, integrated at a fixed rate so the
// result doesn't depend on the host's frame rate.
//
// The host engine only answers two questions through `World`:
//   - where is the ground under a wheel (ray cast),
//   - is the body touching anything (walls/obstacles) right now.
// Frame: right-handed, +X right, +Y up, -Z forward (body local).
#pragma once

#include <vector>

#include "drift/CarSpec.h"
#include "drift/Math.h"
#include "drift/Tire.h"

namespace drift {

enum class Surface { Asphalt, Kerb, Concrete, Dirt, Grass, Gravel, Sand };
struct SurfaceInfo {
	double grip;
	double drag;
	bool road;
};
SurfaceInfo GetSurfaceInfo(Surface s);

struct GroundHit {
	bool hit = false;
	Vec3 point;
	Vec3 normal{0, 1, 0};
	Surface surface = Surface::Asphalt;
};

// A penetration between the car body and the world. `normal` points out of
// the obstacle (the direction the car must move to separate).
struct BodyContact {
	Vec3 point;
	Vec3 normal;
	double depth = 0.0;
	bool wall = true;
};

class World {
public:
	virtual ~World() = default;
	virtual GroundHit Raycast(const Vec3& from, const Vec3& to) = 0;
	// Box in world space: centre, orientation, half extents.
	virtual void BodyContacts(const Vec3& center, const Mat3& rot, const Vec3& half_extents, std::vector<BodyContact>& out) {
		(void)center; (void)rot; (void)half_extents; (void)out;
	}
};

struct Inputs {
	double steer = 0.0;      // -1 left .. +1 right (fraction of lock)
	double throttle = 0.0;
	double brake = 0.0;
	double handbrake = 0.0;
	double clutch = 0.0;     // 1 = pedal fully pressed
	bool direct_steer = false;   // steering wheel: steer is a direct angle fraction
	double steering_assist = 0.0;
	bool auto_clutch = true;
	bool auto_shift = false;
};

struct WheelState {
	Vec3 mount;              // suspension top, body local
	bool front = false, left = false;
	double radius = 0.31, inertia = 1.4;
	double spring = 30000, bump = 2500, rebound = 4000, travel = 0.2;
	TireModel tire;
	double omega = 0.0, spin = 0.0;
	double compression = 0.0, prev_compression = 0.0;
	bool contact = false;
	Vec3 point, normal{0, 1, 0};
	Surface surface = Surface::Asphalt;
	double fz = 0.0, steer = 0.0;
	double v_long = 0.0, v_lat = 0.0;
	double slip_ratio = 0.0, slip_angle = 0.0;
	double fx = 0.0, fy = 0.0;
	double drive_torque = 0.0;
	Vec3 dir_f{0, 0, -1}, dir_s{1, 0, 0};
};

class Vehicle {
public:
	static constexpr double kBodyHz = 480.0;
	static constexpr int kWheelSubsteps = 2;
	static constexpr double kGravity = 9.81;

	explicit Vehicle(const CarSpec& spec);

	Inputs in;

	// Places the body origin (axle height, midway between the axles) at
	// `origin` with orientation `rot`, at rest.
	void Reset(const Vec3& origin, const Mat3& rot);
	// Advances by dt seconds in fixed internal steps. Returns steps taken.
	int Step(double dt, World& world);

	// --- State -----------------------------------------------------------------
	const CarSpec& Spec() const { return spec_; }
	Vec3 Origin() const { return com_ - rot_ * com_local_; }
	const Mat3& Rotation() const { return rot_; }
	Vec3 Com() const { return com_; }
	const Vec3& Velocity() const { return vel_; }
	const Vec3& AngularVelocity() const { return ang_vel_; }
	Vec3 Forward() const { return -rot_.c2; }
	Vec3 Up() const { return rot_.c1; }
	Vec3 Right() const { return rot_.c0; }
	const WheelState& Wheel(int i) const { return wheels_[static_cast<size_t>(i)]; }
	// Wheel centre in body-local space (for drawing the wheel).
	Vec3 WheelCenterLocal(int i) const;
	Vec3 CollisionHalfExtents() const { return box_half_; }
	Vec3 CollisionCenterLocal() const { return box_center_; }
	Vec3 SeatLocal() const;

	double Rpm() const { return engine_omega_ * 60.0 / kTau; }
	int Gear() const { return gear_; }   // -1 R, 0 N, 1.. forward
	int GearCount() const { return static_cast<int>(spec_.gears.size()); }
	double Speed() const { return vel_.Length(); }
	double DriftAngleDeg() const { return drift_angle_; }
	double SteerAngle() const { return steer_angle_; }
	double Boost() const { return boost_; }
	int OnRoadWheels() const { return on_road_wheels_; }
	bool WallContact() const { return wall_contact_; }
	double LastImpact() const { return last_impact_; }   // m/s velocity change of the latest hit

	// Force feedback (-1..1, + pushes the steering wheel right), damper 0..1.
	double FfbForce() const { return ffb_force_; }
	double FfbDamper() const { return ffb_damper_; }

	void ShiftUp();
	void ShiftDown();
	void SetGear(int gear);   // H-shifter: -1 R, 0 N, 1..GearCount()
	double EngineTorque(double rpm, double throttle) const;

private:
	void SubStep(double dt, World& world);
	void UpdateSteering(double dt);
	void UpdateSuspension(World& world, double dt);
	void UpdateDrivetrain(double dt);
	void Integrate(double dt, const Vec3& force, const Vec3& torque);
	void ResolveContacts(World& world);
	void ComputeFfb(double dt);
	double Ratio() const;
	double DrivenOmega() const;
	void AxleDiff(WheelState& l, WheelState& r, double t_in, double dt) const;
	void CenterCoupling(double t_out, double dt);
	Vec2 TireForce(WheelState& wh, double omega) const;
	void SolveWheel(WheelState& wh, double brake_t, double dt);
	Vec3 InvInertiaWorld(const Vec3& v) const;

	CarSpec spec_;
	std::vector<WheelState> wheels_;
	Vec3 com_local_;
	Vec3 inertia_;         // body-frame diagonal
	Vec3 box_half_, box_center_;

	Vec3 com_;
	Mat3 rot_;
	Vec3 vel_, ang_vel_;
	Vec3 force_acc_, torque_acc_;

	int gear_ = 1;
	double engine_omega_ = 80.0;
	double boost_ = 0.0;
	double shift_timer_ = 0.0;
	double limiter_timer_ = 0.0;
	double auto_shift_cooldown_ = 0.0;
	double steer_angle_ = 0.0;
	double drift_angle_ = 0.0;
	int on_road_wheels_ = 0;
	bool wall_contact_ = false;
	double last_impact_ = 0.0;
	double accumulator_ = 0.0;

	double ffb_force_ = 0.0, ffb_damper_ = 0.0;
	double ffb_jolt_ = 0.0, ffb_jolt_t_ = 0.0, ffb_time_ = 0.0;
	unsigned rng_ = 12345u;
	std::vector<BodyContact> contacts_;
};

}  // namespace drift
