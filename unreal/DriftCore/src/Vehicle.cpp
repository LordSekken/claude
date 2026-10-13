#include "drift/Vehicle.h"

#include <cmath>

namespace drift {

SurfaceInfo GetSurfaceInfo(Surface s) {
	switch (s) {
		case Surface::Asphalt: return {1.0, 0.0, true};
		case Surface::Kerb: return {0.93, 0.0, true};
		case Surface::Concrete: return {0.95, 0.0, true};
		case Surface::Dirt: return {0.72, 0.05, false};
		case Surface::Grass: return {0.58, 0.09, false};
		case Surface::Gravel: return {0.55, 0.20, false};
		case Surface::Sand: return {0.5, 0.30, false};
	}
	return {1.0, 0.0, true};
}

Vehicle::Vehicle(const CarSpec& spec) : spec_(spec) {
	const double L = spec_.wheelbase;
	const double rmean = (spec_.radius_f + spec_.radius_r) * 0.5;
	// Body origin sits at axle height, midway between the axles.
	com_local_ = {0.0, spec_.cg_height - rmean, -L * 0.5 + spec_.com_from_front};
	const double m = spec_.mass, w = spec_.width, h = spec_.height, l = spec_.length;
	// Inertia of a uniform box (same convention as Assetto Corsa's INERTIA).
	inertia_ = Vec3(m * (h * h + l * l) / 12.0, m * (w * w + l * l) / 12.0, m * (w * w + h * h) / 12.0) * 0.9;
	const double body_h = h * 0.62;
	const double clearance = 0.14 + spec_.ride_offset * 0.6;
	box_half_ = {w * 0.96 * 0.5, body_h * 0.5, l * 0.98 * 0.5};
	box_center_ = {0.0, -rmean + (clearance > 0.07 ? clearance : 0.07) + body_h * 0.5, 0.0};

	wheels_.resize(4);
	for (int i = 0; i < 4; ++i) {
		WheelState& wh = wheels_[static_cast<size_t>(i)];
		wh.front = i < 2;
		wh.left = i % 2 == 0;
		const double track = wh.front ? spec_.track_f : spec_.track_r;
		const double z = wh.front ? -L * 0.5 : L * 0.5;
		wh.radius = wh.front ? spec_.radius_f : spec_.radius_r;
		wh.travel = spec_.travel > 0.08 ? spec_.travel : 0.08;
		wh.mount = {(wh.left ? -0.5 : 0.5) * track, wh.travel * 0.5, z};
		wh.inertia = wh.front ? spec_.wheel_inertia_f : spec_.wheel_inertia_r;
		wh.spring = wh.front ? spec_.spring_f : spec_.spring_r;
		wh.bump = wh.front ? spec_.bump_f : spec_.bump_r;
		wh.rebound = wh.front ? spec_.rebound_f : spec_.rebound_r;
		wh.tire.Configure(wh.front ? spec_.tire_f : spec_.tire_r);
	}
	Reset(Vec3(0, 0, 0), Mat3::Identity());
}

void Vehicle::Reset(const Vec3& origin, const Mat3& rot) {
	rot_ = rot;
	rot_.Orthonormalize();
	com_ = origin + rot_ * com_local_;
	vel_ = ang_vel_ = Vec3();
	for (WheelState& wh : wheels_) {
		wh.omega = 0.0;
		wh.compression = wh.prev_compression = 0.0;
		wh.contact = false;
	}
	gear_ = 1;
	boost_ = 0.0;
	shift_timer_ = limiter_timer_ = 0.0;
	engine_omega_ = spec_.idle_rpm * kTau / 60.0;
	accumulator_ = 0.0;
}

Vec3 Vehicle::WheelCenterLocal(int i) const {
	const WheelState& wh = wheels_[static_cast<size_t>(i)];
	return wh.mount - Vec3(0, wh.travel - wh.compression, 0);
}

Vec3 Vehicle::SeatLocal() const {
	if (spec_.has_eye) return {spec_.eye[0], spec_.eye[1], spec_.eye[2]};
	const double rmean = (spec_.radius_f + spec_.radius_r) * 0.5;
	const double eye_h = spec_.height * 0.70 + spec_.ride_offset * 0.6;
	return {-spec_.width * 0.21, -rmean + eye_h, spec_.wheelbase * 0.06};
}

void Vehicle::ShiftUp() {
	if (gear_ < GearCount()) {
		gear_ += 1;
		shift_timer_ = spec_.shift_time;
	}
}

void Vehicle::ShiftDown() {
	if (gear_ > -1) {
		gear_ -= 1;
		shift_timer_ = spec_.shift_time;
	}
}

double Vehicle::EngineTorque(double rpm, double throttle) const {
	double full = spec_.TorqueAt(rpm) * spec_.power_mult;
	if (spec_.turbo) full *= (1.0 + boost_) / (1.0 + spec_.boost_stock);
	const double coast = spec_.coast_nm * Clamp(rpm / spec_.limiter_rpm, 0.15, 1.0);
	return throttle * full - (1.0 - throttle) * coast;
}

int Vehicle::Step(double dt, World& world) {
	const double h = 1.0 / kBodyHz;
	accumulator_ += dt;
	if (accumulator_ > 0.25) accumulator_ = 0.25;   // don't spiral after a hitch
	int steps = 0;
	while (accumulator_ >= h) {
		SubStep(h, world);
		accumulator_ -= h;
		++steps;
	}
	return steps;
}

void Vehicle::SubStep(double dt, World& world) {
	force_acc_ = torque_acc_ = Vec3();
	const Vec3 up = Up(), fwd = Forward(), right = Right();

	UpdateSteering(dt);
	UpdateSuspension(world, dt);

	// Contact-patch kinematics.
	on_road_wheels_ = 0;
	for (WheelState& wh : wheels_) {
		if (!wh.contact) continue;
		if (GetSurfaceInfo(wh.surface).road) ++on_road_wheels_;
		Vec3 wf = Rotated(fwd, up, -wh.steer);
		wf = (wf - wh.normal * wf.Dot(wh.normal)).Normalized();
		Vec3 ws = wh.normal.Cross(wf).Normalized() * -1.0;   // the car's right
		Vec3 pv = vel_ + ang_vel_.Cross(wh.point - com_);
		wh.v_long = pv.Dot(wf);
		wh.v_lat = pv.Dot(ws);
		wh.dir_f = wf;
		wh.dir_s = ws;
	}

	UpdateDrivetrain(dt);

	// Tyre + suspension forces.
	for (WheelState& wh : wheels_) {
		if (!wh.contact) continue;
		// Load acts along the ground normal: the suspension arms react the part
		// of the strut force that isn't (else a pitched car creeps at rest).
		Vec3 f = wh.dir_f * wh.fx + wh.dir_s * wh.fy + wh.normal * wh.fz;
		const double drag = GetSurfaceInfo(wh.surface).drag;
		if (drag > 0.0) f -= wh.dir_f * (Sign(wh.v_long) * drag * wh.fz);
		force_acc_ += f;
		torque_acc_ += (wh.point - com_).Cross(f);
	}
	// Aero drag + rolling resistance.
	const double speed = vel_.Length();
	if (speed > 0.1) {
		force_acc_ -= vel_ * (0.5 * 1.2 * spec_.cda * speed + 0.012 * spec_.mass * kGravity / (speed > 1.0 ? speed : 1.0));
	}
	force_acc_ += Vec3(0, -kGravity * spec_.mass, 0);

	Integrate(dt, force_acc_, torque_acc_);
	ResolveContacts(world);

	Vec3 v_flat = vel_ - up * vel_.Dot(up);
	drift_angle_ = v_flat.Length() > 2.0 ? Rad2Deg(std::atan2(v_flat.Dot(right), v_flat.Dot(fwd))) : 0.0;
	ComputeFfb(dt);
}

Vec3 Vehicle::InvInertiaWorld(const Vec3& v) const {
	Vec3 b = rot_.TMul(v);
	b = {b.x / inertia_.x, b.y / inertia_.y, b.z / inertia_.z};
	return rot_ * b;
}

void Vehicle::Integrate(double dt, const Vec3& force, const Vec3& torque) {
	vel_ += force * (dt / spec_.mass);
	// Angular: Euler's equations in the body frame (with gyroscopic term).
	Vec3 wb = rot_.TMul(ang_vel_);
	Vec3 tb = rot_.TMul(torque);
	Vec3 iw = {inertia_.x * wb.x, inertia_.y * wb.y, inertia_.z * wb.z};
	Vec3 rhs = tb - wb.Cross(iw);
	wb += Vec3(rhs.x / inertia_.x, rhs.y / inertia_.y, rhs.z / inertia_.z) * dt;
	ang_vel_ = rot_ * wb;
	com_ += vel_ * dt;
	const double w = ang_vel_.Length();
	if (w > 1e-9) {
		rot_ = Mat3::AxisAngle(ang_vel_ / w, w * dt) * rot_;
		rot_.Orthonormalize();
	}
}

void Vehicle::ResolveContacts(World& world) {
	contacts_.clear();
	world.BodyContacts(Origin() + rot_ * box_center_, rot_, box_half_, contacts_);
	wall_contact_ = false;
	for (const BodyContact& c : contacts_) {
		if (c.depth <= 0.0) continue;
		Vec3 n = c.normal.Normalized();
		if (c.wall) wall_contact_ = true;
		// Push out of the obstacle (most of the penetration per step).
		com_ += n * (c.depth * 0.8);
		Vec3 r = c.point - com_;
		Vec3 vp = vel_ + ang_vel_.Cross(r);
		double vn = vp.Dot(n);
		if (vn >= 0.0) continue;
		const double e = 0.15;
		double k = 1.0 / spec_.mass + n.Dot(InvInertiaWorld(r.Cross(n)).Cross(r));
		double j = -(1.0 + e) * vn / k;
		Vec3 impulse = n * j;
		// Friction (scraping along the wall).
		Vec3 vt = vp - n * vn;
		double vt_len = vt.Length();
		if (vt_len > 1e-6) {
			Vec3 t = vt / vt_len;
			double kt = 1.0 / spec_.mass + t.Dot(InvInertiaWorld(r.Cross(t)).Cross(r));
			double jt = vt_len / kt;
			const double mu = 0.35;
			if (jt > mu * j) jt = mu * j;
			impulse -= t * jt;
		}
		vel_ += impulse / spec_.mass;
		ang_vel_ += InvInertiaWorld(r.Cross(impulse));
		double dv = impulse.Length() / spec_.mass;
		last_impact_ = dv;
		if (dv > 0.8) {
			// Kick the steering wheel away from the side that hit.
			double lateral = (impulse / spec_.mass).Dot(Right());
			ffb_jolt_ = Clamp(lateral * 0.25, -0.9, 0.9);
			ffb_jolt_t_ = 0.09;
		}
	}
}

void Vehicle::UpdateSteering(double dt) {
	const double lock = spec_.steer_lock;
	double target = in.steer * lock;
	if (!in.direct_steer) {
		// Gamepad/keyboard: front wheels self-align with the direction of travel
		// when you let go (like caster) - makes counter-steer easy.
		const Vec3 fwd = Forward(), right = Right();
		const double v_long = vel_.Dot(fwd);
		if (std::fabs(v_long) > 3.0 && in.steering_assist > 0.0) {
			Vec3 front_v = vel_ + ang_vel_.Cross(fwd * (spec_.wheelbase * 0.5));
			double align = std::atan2(front_v.Dot(right), std::fabs(front_v.Dot(fwd)));
			align = Clamp(align, -lock, lock);
			target = Lerp(target, align, in.steering_assist * (1.0 - std::fabs(in.steer)));
		}
		steer_angle_ = MoveToward(steer_angle_, target, 3.2 * lock * dt);
	} else {
		steer_angle_ = Clamp(target, -lock, lock);
	}
	// Ackermann: inner wheel turns more, blended with parallel steer.
	const double L = spec_.wheelbase, t = spec_.track_f, ack = spec_.ackermann;
	for (WheelState& wh : wheels_) {
		if (!wh.front || std::fabs(steer_angle_) < 1e-4) {
			wh.steer = 0.0;
			continue;
		}
		const double r = L / std::tan(std::fabs(steer_angle_));
		const bool inner = (wh.left && steer_angle_ < 0.0) || (!wh.left && steer_angle_ > 0.0);
		const double ack_angle = inner ? std::atan(L / (r - t * 0.5)) : std::atan(L / (r + t * 0.5));
		wh.steer = Sign(steer_angle_) * Lerp(std::fabs(steer_angle_), ack_angle, ack);
	}
}

void Vehicle::UpdateSuspension(World& world, double dt) {
	const Vec3 up = Up();
	const Vec3 origin = Origin();
	for (WheelState& wh : wheels_) {
		const Vec3 from = origin + rot_ * wh.mount;
		const double length = wh.travel + wh.radius;
		GroundHit hit = world.Raycast(from, from - up * length);
		wh.prev_compression = wh.compression;
		if (!hit.hit) {
			wh.contact = false;
			wh.compression = 0.0;
			wh.fz = 0.0;
			continue;
		}
		wh.contact = true;
		wh.point = hit.point;
		wh.normal = hit.normal.Normalized();
		wh.surface = hit.surface;
		wh.compression = Clamp(length - (hit.point - from).Length(), 0.0, wh.travel);
	}
	// Spring + damper + anti-roll bars.
	for (int axle = 0; axle < 2; ++axle) {
		WheelState& l = wheels_[static_cast<size_t>(axle * 2)];
		WheelState& r = wheels_[static_cast<size_t>(axle * 2 + 1)];
		const double arb = axle == 0 ? spec_.arb_f : spec_.arb_r;
		const double arb_f = arb * (l.compression - r.compression);
		for (int k = 0; k < 2; ++k) {
			WheelState& wh = k == 0 ? l : r;
			if (!wh.contact) continue;
			const double vel = (wh.compression - wh.prev_compression) / dt;
			const double damp = vel > 0.0 ? wh.bump : wh.rebound;
			double f = wh.spring * wh.compression + damp * vel;
			if (wh.compression > wh.travel * 0.92) f += (wh.compression - wh.travel * 0.92) * wh.spring * 12.0;   // bump stop
			f += k == 0 ? arb_f : -arb_f;
			const double tilt = up.Dot(wh.normal);
			wh.fz = (f > 0.0 ? f : 0.0) * (tilt > 0.2 ? tilt : 0.2);
		}
	}
}

double Vehicle::Ratio() const {
	if (gear_ == 0) return 0.0;
	const double g = gear_ < 0 ? -spec_.reverse : spec_.gears[static_cast<size_t>(gear_ - 1)];
	return g * spec_.final_drive;
}

double Vehicle::DrivenOmega() const {
	const double f = (wheels_[0].omega + wheels_[1].omega) * 0.5;
	const double r = (wheels_[2].omega + wheels_[3].omega) * 0.5;
	switch (spec_.layout) {
		case Layout::FWD: return f;
		case Layout::RWD: return r;
		case Layout::AWD: break;
	}
	return f * spec_.awd_front_split + r * (1.0 - spec_.awd_front_split);
}

// Splits torque 50/50 then applies a clutch-type LSD locking torque that
// resists speed difference: preload + ramp * |input torque| (power / coast).
void Vehicle::AxleDiff(WheelState& l, WheelState& r, double t_in, double dt) const {
	l.drive_torque += t_in * 0.5;
	r.drive_torque += t_in * 0.5;
	if (spec_.diff_type == DiffType::Open) return;
	double cap = 1.0e7;
	if (spec_.diff_type == DiffType::Lsd) {
		const double ramp = t_in >= 0.0 ? spec_.diff_power : spec_.diff_coast;
		cap = spec_.diff_preload + ramp * std::fabs(t_in) * 2.0;
	}
	const double dw = l.omega - r.omega;
	const double t_eq = dw / (dt * (1.0 / l.inertia + 1.0 / r.inertia));
	const double t_lock = Clamp(t_eq, -cap, cap);
	l.drive_torque -= t_lock;
	r.drive_torque += t_lock;
}

void Vehicle::CenterCoupling(double t_out, double dt) {
	const double f = (wheels_[0].omega + wheels_[1].omega) * 0.5;
	const double r = (wheels_[2].omega + wheels_[3].omega) * 0.5;
	const double i_f = wheels_[0].inertia + wheels_[1].inertia;
	const double i_r = wheels_[2].inertia + wheels_[3].inertia;
	const double cap = spec_.center_lock_nm + 0.25 * std::fabs(t_out);
	const double t_eq = (f - r) / (dt * (1.0 / i_f + 1.0 / i_r));
	const double t_lock = Clamp(t_eq, -cap, cap) * 0.5;
	wheels_[0].drive_torque -= t_lock;
	wheels_[1].drive_torque -= t_lock;
	wheels_[2].drive_torque += t_lock;
	wheels_[3].drive_torque += t_lock;
}

Vec2 Vehicle::TireForce(WheelState& wh, double omega) const {
	const double grip = GetSurfaceInfo(wh.surface).grip;
	const double vden = std::fabs(wh.v_long) > 3.0 ? std::fabs(wh.v_long) : 3.0;
	const double sr = (omega * wh.radius - wh.v_long) / vden;
	const double sa = std::atan2(wh.v_lat, vden);
	wh.slip_ratio = sr;
	wh.slip_angle = sa;
	return wh.tire.Compute(sr, sa, wh.fz, grip);
}

// Implicit wheel-spin solve: stable with stiff tyres at a modest rate.
void Vehicle::SolveWheel(WheelState& wh, double brake_t, double dt) {
	if (!wh.contact) {
		wh.omega += wh.drive_torque / wh.inertia * dt;
		wh.omega = MoveToward(wh.omega, 0.0, brake_t / wh.inertia * dt + 0.5 * dt);
		wh.fx = wh.fy = 0.0;
		return;
	}
	const double w0 = wh.omega;
	const Vec2 f0 = TireForce(wh, w0);
	const double eps = 0.05;
	const Vec2 f1 = TireForce(wh, w0 + eps);
	double k = (f1.x - f0.x) / eps;   // dFx/domega
	if (k < 0.0) k = 0.0;
	const double denom = wh.inertia + dt * wh.radius * k;
	double w1 = w0 + dt * (wh.drive_torque - wh.radius * f0.x) / denom;
	const double dw_brake = brake_t * dt / denom;
	if (std::fabs(w1) <= dw_brake) w1 = 0.0;
	else w1 -= Sign(w1) * dw_brake;
	wh.omega = w1;
	const Vec2 f = TireForce(wh, w1);
	wh.fx = f.x;
	wh.fy = f.y;
}

void Vehicle::UpdateDrivetrain(double dt) {
	const double limiter = spec_.limiter_rpm, idle = spec_.idle_rpm;
	const double rpm_now = Rpm();

	auto_shift_cooldown_ -= dt;
	if (in.auto_shift && shift_timer_ <= 0.0 && gear_ > 0 && auto_shift_cooldown_ <= 0.0) {
		if (rpm_now > limiter * 0.97 && gear_ < GearCount()) {
			ShiftUp();
			auto_shift_cooldown_ = 0.8;
		} else if (rpm_now < limiter * 0.42 && gear_ > 1) {
			ShiftDown();
			auto_shift_cooldown_ = 0.8;
		}
	}
	double throttle = in.throttle, brake = in.brake;
	if (in.auto_shift) {
		const double v_fwd = vel_.Dot(Forward());
		if (gear_ >= 1 && in.brake > 0.3 && in.throttle < 0.05 && v_fwd < 0.5) gear_ = -1;
		else if (gear_ == -1 && in.throttle > 0.3 && v_fwd > -0.5) gear_ = 1;
		if (gear_ == -1) {
			throttle = in.brake;
			brake = in.throttle;
		}
	}

	if (spec_.turbo) {
		const double spool = SmoothStep(spec_.spool_rpm * 0.6, spec_.spool_rpm * 1.1, rpm_now);
		const double target = spec_.boost_target * spool * throttle;
		boost_ += (target - boost_) * (dt / spec_.turbo_lag_s < 1.0 ? dt / spec_.turbo_lag_s : 1.0);
	}

	if (rpm_now > limiter) limiter_timer_ = 0.06;   // fuel cut
	if (limiter_timer_ > 0.0) {
		limiter_timer_ -= dt;
		throttle = 0.0;
	}
	if (shift_timer_ > 0.0) {
		shift_timer_ -= dt;
		throttle *= 0.15;
	}
	if (rpm_now < idle) {
		const double idle_t = Clamp((idle - rpm_now) / 250.0, 0.0, 0.6);
		if (idle_t > throttle) throttle = idle_t;
	}

	double engage = 1.0 - in.clutch;
	const double ratio = Ratio();
	if (in.auto_clutch) {
		double launch = Clamp((rpm_now - idle * 1.1) / 1800.0, 0.0, 1.0);
		const double wheel_rpm = std::fabs(DrivenOmega()) * std::fabs(ratio) * 60.0 / kTau;
		if (wheel_rpm > idle * 1.1) launch = 1.0;
		if (launch < engage) engage = launch;
	}
	if (gear_ == 0 || shift_timer_ > 0.0) engage = 0.0;

	const double ie = spec_.engine_inertia;
	const bool front_driven = spec_.layout != Layout::RWD;
	const bool rear_driven = spec_.layout != Layout::FWD;
	const double front_share = spec_.layout == Layout::FWD ? 1.0 : (spec_.layout == Layout::AWD ? spec_.awd_front_split : 0.0);
	double i_driven = 0.0;
	for (const WheelState& wh : wheels_) {
		if ((wh.front && front_driven) || (!wh.front && rear_driven)) i_driven += wh.inertia;
	}
	const double hb = in.handbrake * spec_.handbrake_torque;
	const double bt = brake * spec_.brake_torque;
	const double bias = spec_.brake_bias;

	const double sub_dt = dt / kWheelSubsteps;
	double acc_fx[4] = {0, 0, 0, 0}, acc_fy[4] = {0, 0, 0, 0};
	for (int s = 0; s < kWheelSubsteps; ++s) {
		const double te = EngineTorque(Rpm(), throttle);
		double t_clutch = 0.0;
		if (engage > 0.0 && std::fabs(ratio) > 0.0) {
			const double w_in = DrivenOmega() * ratio;
			const double i_dl = i_driven / (ratio * ratio);
			const double t_eq = (engine_omega_ - w_in) / (sub_dt * (1.0 / ie + 1.0 / i_dl));
			const double cap = spec_.clutch_nm * engage;
			t_clutch = Clamp(t_eq, -cap, cap);
		}
		engine_omega_ += (te - t_clutch) / ie * sub_dt;
		if (engine_omega_ < 0.0) engine_omega_ = 0.0;

		const double t_out = t_clutch * ratio;
		for (WheelState& wh : wheels_) wh.drive_torque = 0.0;
		if (front_driven) AxleDiff(wheels_[0], wheels_[1], t_out * front_share, sub_dt);
		if (rear_driven) AxleDiff(wheels_[2], wheels_[3], t_out * (1.0 - front_share), sub_dt);
		if (spec_.layout == Layout::AWD) CenterCoupling(t_out, sub_dt);

		for (int i = 0; i < 4; ++i) {
			WheelState& wh = wheels_[static_cast<size_t>(i)];
			double brake_t = bt * (wh.front ? bias : 1.0 - bias) * 0.5;
			if (!wh.front) brake_t += hb * 0.5;
			SolveWheel(wh, brake_t, sub_dt);
			acc_fx[i] += wh.fx;
			acc_fy[i] += wh.fy;
		}
	}
	for (int i = 0; i < 4; ++i) {
		WheelState& wh = wheels_[static_cast<size_t>(i)];
		wh.fx = acc_fx[i] / kWheelSubsteps;
		wh.fy = acc_fy[i] / kWheelSubsteps;
		wh.spin = std::fmod(wh.spin + wh.omega * dt, kTau);
	}
}

// Steering-rack torque from the front tyres: self-aligning torque = lateral
// force x (pneumatic + caster trail); pneumatic trail collapses past peak slip
// so the wheel goes light as the fronts let go. Plus kerb/rough rumble,
// wall jolts and low-speed damping.
void Vehicle::ComputeFfb(double dt) {
	ffb_time_ += dt;
	double torque = 0.0, rumble = 0.0;
	const double speed = Speed();
	for (const WheelState& wh : wheels_) {
		if (!wh.front || !wh.contact) continue;
		const double peak = wh.tire.Params().peak_slip_angle;
		double pneumatic = 0.035 * (1.0 - std::fabs(wh.slip_angle) / (peak * 1.7));
		if (pneumatic < 0.0) pneumatic = 0.0;
		torque += -wh.fy * (pneumatic + 0.022);
		if (wh.surface == Surface::Kerb) {
			rumble += std::sin(ffb_time_ * kTau * 22.0) * 0.12 * Clamp(speed / 15.0, 0.0, 1.0);
		} else if (!GetSurfaceInfo(wh.surface).road) {
			rng_ = rng_ * 1664525u + 1013904223u;
			const double r = static_cast<double>(rng_ >> 8) / 16777216.0;
			rumble += (r - 0.5) * 0.12 * Clamp(speed / 10.0, 0.0, 1.0);
		}
	}
	const double static_front = spec_.mass * kGravity * (1.0 - spec_.com_from_front / spec_.wheelbase);
	const double norm = (static_front > 1000.0 ? static_front : 1000.0) * 1.1 * 0.057;
	double f = torque / norm;
	if (ffb_jolt_t_ > 0.0) {
		ffb_jolt_t_ -= dt;
		f += ffb_jolt_;
	}
	ffb_force_ = Clamp(f + rumble, -1.0, 1.0);
	ffb_damper_ = Lerp(0.45, 0.08, Clamp(speed / 8.0, 0.0, 1.0));
}

}  // namespace drift
