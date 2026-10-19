// DriftChrono standard manoeuvre harness: textbook vehicle-dynamics tests on flat
// dry ground through the same C interface the game uses, each with a metric table
// and pass/fail against real-car ranges.
//   dc_maneuvers <car root> <car id> [test numbers, e.g. 1 2 6]
//
// Frames and signs (DriftCore): +X right, +Y up, -Z forward. Here "right" quantities are
// positive for a right turn: yaw rate r = -ang_vel.y, sideslip beta = atan2(v_right, v_fwd)
// (+ = CoM velocity points right of the heading), steer + = right.
//
// The ranges are what a real lightweight RWD sports car (Mazda MX-5 class, ~1000-1100 kg,
// performance street tyres) shows in these tests; each check cites where its range comes from.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "drift_chrono.h"

namespace {
constexpr double kG = 9.81;
constexpr double kDeg = 57.29578;
constexpr double kDt = 0.01;   // host frame, like the game at 100 fps

int Flat(void*, const double from[3], const double to[3], double p[3], double n[3], double* grip) {
    if (from[1] < 0 || to[1] > 0) return 0;
    const double t = from[1] / (from[1] - to[1]);
    for (int i = 0; i < 3; ++i) p[i] = from[i] + (to[i] - from[i]) * t;
    n[0] = 0, n[1] = 1, n[2] = 0;
    *grip = 1.0;
    return 1;
}

int g_pass = 0, g_fail = 0;
// One metric row: value against [lo, hi]; `src` names where the range comes from.
void Metric(const char* name, double v, double lo, double hi, const char* unit, const char* src) {
    const bool ok = std::isfinite(v) && v >= lo && v <= hi;
    std::printf("  %-44s %9.3f %-7s [%7.2f, %7.2f]  %s  (%s)\n", name, v, unit, lo, hi, ok ? "PASS" : "FAIL", src);
    ok ? ++g_pass : ++g_fail;
}
void Info(const char* name, double v, const char* unit) { std::printf("  %-44s %9.3f %-7s\n", name, v, unit); }

double Clamp(double v, double lo, double hi) { return std::min(hi, std::max(lo, v)); }

// Least-squares y = c0 + c1 x.
struct Fit { double slope = NAN, icpt = NAN; int n = 0; };
Fit LinFit(const std::vector<double>& x, const std::vector<double>& y) {
    Fit f;
    const size_t n = x.size();
    f.n = static_cast<int>(n);
    if (n < 3) return f;
    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (size_t i = 0; i < n; ++i) sx += x[i], sy += y[i], sxx += x[i] * x[i], sxy += x[i] * y[i];
    const double d = n * sxx - sx * sx;
    if (std::fabs(d) < 1e-12) return f;
    f.slope = (n * sxy - sx * sy) / d;
    f.icpt = (sy - f.slope * sx) / n;
    return f;
}

// Least-squares y = a sin(wt) + b cos(wt) + c: amplitude and phase (rad).
void SineFit(const std::vector<double>& t, const std::vector<double>& y, double w, double& amp, double& phase) {
    double m[3][4] = {};
    for (size_t i = 0; i < t.size(); ++i) {
        const double f[3] = {std::sin(w * t[i]), std::cos(w * t[i]), 1.0};
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) m[r][c] += f[r] * f[c];
            m[r][3] += f[r] * y[i];
        }
    }
    for (int p = 0; p < 3; ++p) {   // Gauss-Jordan
        int best = p;
        for (int r = p + 1; r < 3; ++r)
            if (std::fabs(m[r][p]) > std::fabs(m[best][p])) best = r;
        for (int c = 0; c < 4; ++c) std::swap(m[p][c], m[best][c]);
        for (int r = 0; r < 3; ++r) {
            if (r == p) continue;
            const double k = m[r][p] / m[p][p];
            for (int c = 0; c < 4; ++c) m[r][c] -= k * m[p][c];
        }
    }
    const double a = m[0][3] / m[0][0], b = m[1][3] / m[1][1];
    amp = std::hypot(a, b);
    phase = std::atan2(b, a);
}

// The car plus derived kinematics and a few driver helpers.
struct Sim {
    dc_car* car = nullptr;
    dc_state s{};
    dc_controls c{0, 0, 1, 0, 1, 1};
    double t = 0;
    double L = 2.3, lock = 0.9, rw = 0.29;   // wheelbase, steer lock, rolling radius
    // Kinematics (updated every frame).
    double px = 0, pz = 0, fx = 0, fz = -1, rx = 1, rz = 0;
    double vf = 0, vr = 0, speed = 0, yaw = 0 /* + right */, beta = 0 /* rad */, steer = 0 /* mean road wheel */;
    double ax = 0, ay = 0;   // body-frame accel (forward, right), low-passed
    double pvx = 0, pvz = 0;
    // Auto shift.
    bool autoshift = false;
    int min_gear = 1, max_gear = 5;
    double shift_timer = 0;
    // Speed control.
    double th_int = 0.3;

    void Reset() {
        const double origin[3] = {0, 0.30, 0};
        const double rot[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        dc_reset(car, origin, rot);
        c = {0, 0, 1, 0, 1, 1};
        autoshift = false, shift_timer = 0, th_int = 0.3;
        for (int i = 0; i < 100; ++i) dc_step(car, kDt, &c, nullptr, 0);
        dc_get_state(car, &s);
        t = 0;
        Kin(true);
        lock = s.steer_lock;
        L = 0.5 * (s.wheel[2].center_local[2] + s.wheel[3].center_local[2]) -
            0.5 * (s.wheel[0].center_local[2] + s.wheel[1].center_local[2]);
    }
    void Kin(bool first = false) {
        px = s.origin[0], pz = s.origin[2];
        rx = s.rot[0], rz = s.rot[2];
        fx = -s.rot[6], fz = -s.rot[8];
        const double hn = std::hypot(fx, fz), rn = std::hypot(rx, rz);
        fx /= hn, fz /= hn, rx /= rn, rz /= rn;
        vf = s.vel[0] * fx + s.vel[2] * fz;
        vr = s.vel[0] * rx + s.vel[2] * rz;
        speed = std::hypot(s.vel[0], s.vel[2]);
        yaw = -s.ang_vel[1];
        beta = speed > 0.5 ? std::atan2(vr, vf) : 0.0;
        steer = 0.5 * (s.wheel[0].steer + s.wheel[1].steer);
        if (!first) {
            const double dax = (s.vel[0] - pvx) / kDt, daz = (s.vel[2] - pvz) / kDt;
            const double k = 0.2;   // ~50 ms low-pass
            ax += k * ((dax * fx + daz * fz) - ax);
            ay += k * ((dax * rx + daz * rz) - ay);
        } else {
            ax = ay = 0;
        }
        pvx = s.vel[0], pvz = s.vel[2];
        // Rolling radius from the free-rolling (undriven) front wheels when cruising straight.
        if (speed > 8 && std::fabs(steer) < 0.01 && c.brake == 0 && std::fabs(beta) < 0.01) {
            const double om = 0.5 * (s.wheel[0].omega + s.wheel[1].omega);
            if (om > 1) rw += 0.02 * (vf / om - rw);
        }
    }
    void Step() {
        if (autoshift) {
            shift_timer -= kDt;
            if (shift_timer <= 0) {
                int g = c.gear;
                if (s.rpm > 6900 && g < max_gear) ++g;
                else if (s.rpm < 2600 && g > min_gear) --g;
                if (g != c.gear) c.gear = g, shift_timer = 0.25;
            }
            c.clutch_engage = shift_timer > 0.1 ? 0.0 : 1.0;
        }
        dc_step(car, kDt, &c, nullptr, 0);
        dc_get_state(car, &s);
        t += kDt;
        Kin();
    }
    // Wheel slip ratio from omega and the car's forward speed (straight-line use).
    double Slip(int i) const { return (s.wheel[i].omega * rw - vf) / std::max(std::fabs(vf), 1.0); }
    // Throttle PI on speed (brake 0).
    void Cruise(double v_target) {
        const double e = v_target - speed;
        th_int = Clamp(th_int + 0.15 * e * kDt, 0.0, 1.0);
        c.throttle = Clamp(th_int + 0.3 * e, 0.0, 1.0);
        c.brake = 0;
    }
    // Straight from rest to v (m/s) with simple traction control, then hold it for `settle` s.
    bool Launch(double v, double settle, int max_gear_ = 5) {
        Reset();
        c = {0, 0, 0, 0, 1, 1};
        autoshift = true, max_gear = max_gear_;
        th_int = 0.5;
        double held = 0;
        for (int i = 0; i < 6000; ++i) {
            Cruise(v);
            const double slip = 0.5 * (Slip(2) + Slip(3));
            if (slip > 0.08) c.throttle *= 0.4, th_int = std::min(th_int, 0.6);
            c.steer_angle = Clamp(-0.5 * beta - 0.15 * yaw, -0.05, 0.05);   // keep it straight
            Step();
            held = std::fabs(speed - v) < 0.3 ? held + kDt : 0;
            if (held >= settle) return true;
        }
        return false;
    }
};

// Constant-radius circle follower: course error + cross-track preview + integral feedforward.
struct CircleCtl {
    double cx = 0, cz = 0, R = 40;
    int dir = 1;   // +1 right turn, -1 left
    double integ = 0;
    double e = 0;  // radial error, + = outside
    void Start(const Sim& m, double radius, int d) {
        R = radius, dir = d;
        cx = m.px + d * R * m.rx, cz = m.pz + d * R * m.rz;
        integ = m.L / R;
    }
    double Steer(const Sim& m) {
        const double dx = m.px - cx, dz = m.pz - cz, dist = std::hypot(dx, dz);
        const double ux = dx / dist, uz = dz / dist;
        e = dist - R;
        const double sp = std::max(m.speed, 1.0);
        const double out = std::asin(Clamp((m.s.vel[0] * ux + m.s.vel[2] * uz) / sp, -1, 1));   // course, + = outward
        const double Lp = std::max(8.0, 1.5 * sp);
        const double Kc = m.L / (std::max(sp, 3.0) * 0.8);
        integ += 0.003 * e * kDt;
        return dir * (integ + Kc * (out + e / Lp));
    }
};

const char* kSrcIso4138 = "ISO 4138 const-radius; MX-5 class: Gillespie/Milliken typical";

// ---------------------------------------------------------------------------------------------
double g_K = NAN;   // understeer gradient (rad per m/s^2) from test 1, used by tests 2/3

void TestCircle(Sim& m) {
    std::printf("\n== 1. Steady-state circular, R = 40 m, speed ramp 0.1 m/s^2 (ISO 4138 constant radius) ==\n");
    m.Launch(7.0, 1.0, 3);
    CircleCtl pc;
    pc.Start(m, 40.0, +1);
    const double v0 = m.speed;
    std::vector<double> ayv, stv, bev;   // g, deg, deg (beta toward the centre)
    double ay_win[100] = {}, max_ay = 0;
    int wi = 0, wn = 0;
    double lag = 0, t0 = m.t;
    const char* why = "ramp ended";
    double steer_at_lim = 0, beta_at_lim = 0, e_at_lim = 0;
    for (int i = 0; i < 25000; ++i) {
        const double tt = m.t - t0;
        const double vt = v0 + 0.1 * std::max(0.0, tt - 3.0);
        m.c.steer_angle = Clamp(pc.Steer(m), -m.lock, m.lock);
        m.Cruise(vt);
        m.Step();
        const double ay_ss = m.speed * m.yaw / kG;   // steady-state lateral accel, g
        // Max lateral g: 1 s mean of the measured (accelerometer) lateral accel, only while the
        // car is on the circle with a sane sideslip (the spin at the limit inflates V*r).
        const bool tracking = std::fabs(pc.e) < 1.0 && std::fabs(m.beta) < 0.1;
        ay_win[wi] = tracking ? m.ay / kG : 0.0, wi = (wi + 1) % 100, wn = tracking ? std::min(wn + 1, 100) : 0;
        if (wn == 100 && tt > 6) {
            double a = 0;
            for (double v : ay_win) a += v;
            max_ay = std::max(max_ay, a / 100);
        }
        if (tt > 6 && i % 10 == 0 && std::fabs(pc.e) < 1.0) {
            ayv.push_back(ay_ss), stv.push_back(m.steer * kDeg), bev.push_back(m.beta * kDeg);
        }
        lag = (vt - m.speed > 1.0) ? lag + kDt : 0;
        steer_at_lim = m.steer * kDeg, beta_at_lim = m.beta * kDeg, e_at_lim = pc.e;
        if (tt > 6) {
            if (std::fabs(pc.e) > 4) { why = pc.e > 0 ? "ran wide (radius +4 m): understeer limit" : "tightened (radius -4 m): oversteer limit"; break; }
            if (std::fabs(m.beta) > 0.25) { why = "sideslip > 14 deg: oversteer / spin limit"; break; }
            if (std::fabs(m.c.steer_angle) > 0.9 * m.lock) { why = "steering at lock"; break; }
            if (lag > 3) { why = "speed fell 1 m/s behind the ramp for 3 s (tyre drag / power limit)"; break; }
        }
    }
    std::printf("  limit reached at %.1f km/h: %s (steer %.1f deg, beta %.1f deg, radius error %+.2f m)\n", m.speed * 3.6, why,
                steer_at_lim, beta_at_lim, e_at_lim);
    std::printf("  Ackermann steer L/R = %.2f deg (L = %.3f m)\n", m.L / 40.0 * kDeg, m.L);
    std::printf("  %-10s %-12s %-12s %s\n", "ay [g]", "steer [deg]", "beta [deg]", "samples");
    for (int b = 0; b < 12; ++b) {
        const double lo = 0.05 + 0.1 * b, hi = lo + 0.1;
        double ss = 0, sb = 0;
        int n = 0;
        for (size_t i = 0; i < ayv.size(); ++i)
            if (ayv[i] >= lo && ayv[i] < hi) ss += stv[i], sb += bev[i], ++n;
        if (n) std::printf("  %-10.2f %-12.2f %-12.2f %d\n", lo + 0.05, ss / n, sb / n, n);
    }
    std::vector<double> x, ys, yb;
    for (size_t i = 0; i < ayv.size(); ++i)
        if (ayv[i] >= 0.1 && ayv[i] <= 0.4) x.push_back(ayv[i]), ys.push_back(stv[i]), yb.push_back(bev[i]);
    const Fit fs = LinFit(x, ys), fb = LinFit(x, yb);
    g_K = fs.slope / kDeg / kG;
    // Beyond the linear range: secant from 0.4 g to 0.7 g (progressive understeer is normal).
    std::vector<double> x2, y2;
    for (size_t i = 0; i < ayv.size(); ++i)
        if (ayv[i] >= 0.4 && ayv[i] <= 0.7) x2.push_back(ayv[i]), y2.push_back(stv[i]);
    const Fit f2 = LinFit(x2, y2);
    // Understeer gradient at the road wheels, 0.1-0.4 g. Sports cars ~0.5-2 deg/g (Gillespie,
    // "Fundamentals of Vehicle Dynamics" ch.6; Milliken RCVD); owner's AC measurement 0.55 deg/g.
    Metric("understeer gradient 0.1-0.4 g", fs.slope, 0.3, 2.5, "deg/g", kSrcIso4138);
    Info("understeer gradient 0.4-0.7 g (secant)", f2.slope, "deg/g");
    Info("steer intercept (should be ~L/R)", fs.icpt, "deg");
    // Sideslip gradient = -(rear cornering compliance); 1-4 deg/g for passenger/sports cars
    // (Milliken RCVD cornering-compliance data; Gillespie). Sign: + = velocity toward the centre.
    Metric("sideslip gradient 0.1-0.4 g", fb.slope, -4.5, -0.8, "deg/g", "ISO 4138; Milliken RCVD rear compliance 1-4 deg/g");
    // MX-5 (ND/NC) skidpad 0.88-0.98 g on stock tyres (Car and Driver / Motor Trend road tests);
    // streetspec on stickier 200TW tyres up to ~1.1 g.
    Metric("max steady lateral accel (1 s mean)", max_ay, 0.85, 1.15, "g", "MX-5 skidpad road tests 0.88-0.98 g");
    Info("speed at limit", m.speed * 3.6, "km/h");
}

// ---------------------------------------------------------------------------------------------
struct StepResult { double ay_ss, r_ss, d_ss, t_resp, t_rise, t_peak, overshoot, t_wheel50, t_ay90; };

StepResult RunStep(Sim& m, double v, double delta) {
    m.Launch(v, 2.0, 5);
    m.autoshift = false;
    // Hold the cruise throttle (ISO 7401: throttle fixed at the initial-speed setting).
    double th = 0;
    for (int i = 0; i < 100; ++i) {
        m.Cruise(v);
        m.c.steer_angle = 0;
        m.Step();
        th += m.c.throttle / 100;
    }
    m.c.throttle = th;
    const double ramp = 0.10;   // ISO 7401: steering ramp as fast as practical, <= 0.15 s
    const double t0 = m.t;
    std::vector<double> tv, rv, dv, av;
    for (int i = 0; i < 400; ++i) {
        const double tt = m.t - t0;
        m.c.steer_angle = delta * Clamp(tt / ramp, 0, 1);
        m.Step();
        tv.push_back(m.t - t0), rv.push_back(m.yaw), dv.push_back(m.steer), av.push_back(m.ay);
    }
    StepResult r{};
    const size_t n = tv.size();
    double rs = 0, ds = 0, as = 0;
    int k = 0;
    for (size_t i = 0; i < n; ++i)
        if (tv[i] > 2.5 && tv[i] <= 3.5) rs += rv[i], ds += dv[i], as += av[i], ++k;
    r.r_ss = rs / k, r.d_ss = ds / k, r.ay_ss = as / k;
    const double t50 = ramp * 0.5;   // ISO 7401 time reference: 50 % of the steering input
    double t10 = NAN, t90 = NAN, rmax = 0, tmax = 0;
    r.t_wheel50 = r.t_ay90 = NAN;
    for (size_t i = 0; i < n; ++i) {
        if (std::isnan(r.t_wheel50) && dv[i] >= 0.5 * r.d_ss) r.t_wheel50 = tv[i];
        if (std::isnan(r.t_ay90) && av[i] >= 0.9 * r.ay_ss) r.t_ay90 = tv[i] - t50;
        if (std::isnan(t10) && rv[i] >= 0.1 * r.r_ss) t10 = tv[i];
        if (std::isnan(t90) && rv[i] >= 0.9 * r.r_ss) t90 = tv[i];
        if (tv[i] < 2.5 && rv[i] > rmax) rmax = rv[i], tmax = tv[i];
    }
    r.t_resp = t90 - t50, r.t_rise = t90 - t10, r.t_peak = tmax - t50;
    r.overshoot = (rmax - r.r_ss) / r.r_ss * 100;
    return r;
}

void TestStep(Sim& m) {
    std::printf("\n== 2. Step steer at 80 km/h to 4 m/s^2 (ISO 7401 step input) ==\n");
    const double v = 80 / 3.6;
    const double K = std::isfinite(g_K) ? g_K : 1.0 / kDeg / kG;
    double delta = (m.L / (v * v) + K) * 4.0;
    StepResult r = RunStep(m, v, delta);
    std::printf("  trial: steer cmd %.2f deg -> ay %.2f m/s^2\n", delta * kDeg, r.ay_ss);
    delta *= 4.0 / std::max(0.5, r.ay_ss);
    r = RunStep(m, v, delta);
    std::printf("  final: steer cmd %.2f deg, road wheel %.2f deg -> ay %.2f m/s^2, yaw rate %.2f deg/s, %.1f km/h\n",
                delta * kDeg, r.d_ss * kDeg, r.ay_ss, r.r_ss * kDeg, m.speed * 3.6);
    const char* src = "ISO 7401 step; sports-car ranges (Heydinger/NHTSA, Mimuro)";
    // Yaw-rate response time (50 % steer -> 90 % yaw): ~0.1-0.2 s sports cars, 0.15-0.35 s sedans.
    Metric("yaw-rate response time (50% steer->90%)", r.t_resp, 0.06, 0.30, "s", src);
    Info("yaw-rate rise time (10->90 %)", r.t_rise, "s");
    Info("yaw-rate peak response time", r.t_peak, "s");
    Info("road wheel reaches 50 % at (cmd 50 % at 0.05)", r.t_wheel50, "s");
    Info("lateral-accel response time (50%->90%)", r.t_ay90, "s");
    // Overshoot: well-damped road cars 0-25 % at 80 km/h.
    Metric("yaw-rate overshoot", r.overshoot, 0.0, 25.0, "%", src);
    const double gain = r.r_ss / r.d_ss;
    Info("steady yaw gain r/delta (road wheel)", gain, "1/s");
    if (std::isfinite(g_K)) {
        // Bicycle-model consistency with test 1: r/delta = V / (L + K V^2).
        const double pred = v / (m.L + g_K * v * v);
        Metric("yaw gain / bicycle prediction from test 1", gain / pred, 0.8, 1.2, "ratio",
               "linear bicycle model (Gillespie eq. 6-18) self-consistency");
    }
    Info("neutral-steer gain V/L for reference", v / m.L, "1/s");
}

// ---------------------------------------------------------------------------------------------
void TestFreq(Sim& m) {
    std::printf("\n== 3. Sine steer 0.2-2 Hz at 80 km/h (ISO 7401 continuous sinusoidal input) ==\n");
    const double v = 80 / 3.6;
    const double K = std::isfinite(g_K) ? g_K : 1.0 / kDeg / kG;
    const double A = (m.L / (v * v) + K) * 4.0;   // ~4 m/s^2 at low frequency
    const double freqs[] = {0.2, 0.4, 0.6, 0.8, 1.0, 1.25, 1.5, 2.0};
    std::printf("  amplitude %.2f deg (road wheel cmd)\n", A * kDeg);
    std::printf("  %-8s %-12s %-12s %-12s %-12s %s\n", "f [Hz]", "gain [1/s]", "gain/0.2Hz", "lag [deg]", "ay amp[g]", "speed[km/h]");
    double g0 = NAN, lag1 = NAN, peak = 0, g2 = NAN;
    m.Launch(v, 2.0, 5);
    m.autoshift = false;
    for (double f : freqs) {
        const double w = 2 * 3.14159265358979 * f;
        const double dur = std::max(4.0 / f, 8.0);
        std::vector<double> tv, rv, dv, av;
        for (int i = 0; i < 150; ++i) {   // straighten up
            m.Cruise(v);
            m.c.steer_angle = Clamp(-0.5 * m.beta - 0.15 * m.yaw, -0.05, 0.05);
            m.Step();
        }
        const double t0 = m.t;
        double vs = 0;
        int vn = 0;
        while (m.t - t0 < dur) {
            const double tt = m.t - t0;
            m.Cruise(v);
            m.c.steer_angle = A * std::sin(w * (tt + kDt));
            m.Step();
            if (tt > 1.0 / f) tv.push_back(m.t - t0), rv.push_back(m.yaw), dv.push_back(m.steer), av.push_back(m.ay), vs += m.speed, ++vn;
        }
        double ad, pd, ar, pr, aa, pa;
        SineFit(tv, dv, w, ad, pd);
        SineFit(tv, rv, w, ar, pr);
        SineFit(tv, av, w, aa, pa);
        double lag = (pd - pr) * kDeg;
        while (lag > 180) lag -= 360;
        while (lag < -180) lag += 360;
        const double gain = ar / ad;
        if (std::isnan(g0)) g0 = gain;
        if (std::fabs(f - 1.0) < 1e-6) lag1 = lag;
        if (std::fabs(f - 2.0) < 1e-6) g2 = gain / g0;
        peak = std::max(peak, gain / g0);
        std::printf("  %-8.2f %-12.2f %-12.2f %-12.1f %-12.3f %.1f\n", f, gain, gain / g0, lag, aa / kG, vs / vn * 3.6);
    }
    const char* src = "ISO 7401 freq. response; sports cars (Mimuro 1990, Heydinger 1990s NHTSA)";
    // Yaw-rate phase lag at 1 Hz: ~10-25 deg sports cars, 20-45 deg sedans.
    Metric("yaw-rate phase lag at 1 Hz", lag1, 5.0, 40.0, "deg", src);
    // Yaw gain resonance: a damped road car peaks <= ~1.3x its steady gain.
    Metric("peak yaw gain / 0.2 Hz gain", peak, 0.95, 1.35, "ratio", src);
    Info("yaw gain at 2 Hz / 0.2 Hz", g2, "ratio");
}

// ---------------------------------------------------------------------------------------------
// Straight braking from v0; abs = slip-controlled pedal (select-low on the worst wheel).
struct BrakeResult { double dist, time, mfdd, mean, max_slip_f, max_slip_r, yaw_dev; };
BrakeResult RunBrake(Sim& m, double v0, bool abs) {
    m.Launch(v0, 1.0, 5);
    m.autoshift = false;
    BrakeResult r{};
    const double x0 = m.px, z0 = m.pz, hx = m.fx, hz = m.fz;
    const double t0 = m.t;
    double p = abs ? 0.6 : 1.0;
    double s08 = NAN, s01 = NAN;
    double v08 = 0.8 * v0, v01 = 0.1 * v0;
    while (m.speed > 0.3 && m.t - t0 < 15) {
        m.c.throttle = 0, m.c.clutch_engage = 0, m.c.steer_angle = 0;
        if (abs && m.speed > 2) {
            double worst = 0;
            for (int i = 0; i < 4; ++i) worst = std::min(worst, m.Slip(i));
            p = Clamp(p + kDt * 20.0 * (worst + 0.10), 0.05, 1.0);   // aim the worst wheel at -10 % slip
        }
        m.c.brake = p;
        const double vprev = m.speed;
        m.Step();
        const double d = (m.px - x0) * hx + (m.pz - z0) * hz;
        if (std::isnan(s08) && vprev >= v08 && m.speed < v08) s08 = d;
        if (std::isnan(s01) && vprev >= v01 && m.speed < v01) s01 = d;
        for (int i = 0; i < 2; ++i) r.max_slip_f = std::min(r.max_slip_f, m.Slip(i));
        for (int i = 2; i < 4; ++i) r.max_slip_r = std::min(r.max_slip_r, m.Slip(i));
        r.yaw_dev = std::max(r.yaw_dev, std::fabs(std::asin(Clamp(hx * m.fz - hz * m.fx, -1, 1))) * kDeg);
    }
    r.dist = (m.px - x0) * hx + (m.pz - z0) * hz;
    r.time = m.t - t0;
    r.mean = v0 * v0 / (2 * r.dist) / kG;
    r.mfdd = (v08 * v08 - v01 * v01) / (2 * (s01 - s08)) / kG;   // UN ECE R13-H mean fully developed decel
    return r;
}

void TestBrake(Sim& m) {
    std::printf("\n== 4. Braking from 100 km/h; braking in a turn (ISO 7975) ==\n");
    const double v0 = 100 / 3.6;
    std::printf("  rolling radius from front wheels: %.3f m\n", m.rw);
    std::printf("  %-22s %-10s %-8s %-10s %-10s %-12s %-12s %s\n", "mode", "dist [m]", "time[s]", "mean [g]", "MFDD [g]",
                "min slip F", "min slip R", "heading dev[deg]");
    const BrakeResult a = RunBrake(m, v0, true);
    std::printf("  %-22s %-10.1f %-8.2f %-10.3f %-10.3f %-12.2f %-12.2f %.2f\n", "slip-controlled (ABS)", a.dist, a.time, a.mean,
                a.mfdd, a.max_slip_f, a.max_slip_r, a.yaw_dev);
    const BrakeResult b = RunBrake(m, v0, false);
    std::printf("  %-22s %-10.1f %-8.2f %-10.3f %-10.3f %-12.2f %-12.2f %.2f\n", "full pedal (no ABS)", b.dist, b.time, b.mean,
                b.mfdd, b.max_slip_f, b.max_slip_r, b.yaw_dev);
    // MX-5 100-0 km/h ~34-39 m (ADAC / Auto Bild tests; C&D 70-0 mph 155-165 ft scaled).
    Metric("stopping distance 100-0 (ABS-like)", a.dist, 32.0, 40.0, "m", "MX-5 road tests 34-39 m");
    Metric("MFDD (ABS-like)", a.mfdd, 0.95, 1.20, "g", "UN ECE R13-H MFDD; street tyres ~1.0-1.1 g");
    // Locked or near-locked wheels: friction drops to ~0.7-0.9 g on dry asphalt; must stay straight.
    Metric("mean decel, full pedal", b.mean, 0.65, 1.15, "g", "locked-wheel mu 0.7-0.9 (Pacejka, Tyre & Veh. Dyn. ch.1)");
    Metric("heading change, full pedal straight", b.yaw_dev, 0.0, 5.0, "deg", "straight-line braking stability, ECE R13-H");

    // Calibrate pedal -> decel (straight, low-pedal linear range).
    m.Launch(80 / 3.6, 1.0, 5);
    m.autoshift = false;
    const double vs = m.speed;
    for (int i = 0; i < 100; ++i) m.c = {0, 0, 0.3, 0, 0, m.c.gear}, m.Step();
    const double g_per_pedal = (vs - m.speed) / 1.0 / kG / 0.3;
    std::printf("  pedal calibration: %.2f g per unit pedal\n", g_per_pedal);

    // Braking in a turn (ISO 7975): R = 100 m at 0.5 g (~80 km/h), steering held, clutch open, brake at t0.
    std::printf("  braking in a turn, R 100 m, ay0 ~0.5 g, steer fixed; values at t0 + 1 s:\n");
    std::printf("  %-10s %-10s %-14s %-14s %-12s %s\n", "target[g]", "decel[g]", "yaw/yaw0", "yaw/(V/R0)", "dbeta[deg]", "max |beta|");
    bool all_ok = true;
    for (double tg : {0.3, 0.5, 0.7}) {
        const double R0 = 100, v = std::sqrt(0.5 * kG * R0);
        m.Launch(v, 0.5, 5);
        CircleCtl pc;
        pc.Start(m, R0, +1);
        for (int i = 0; i < 1500; ++i) {
            m.c.steer_angle = Clamp(pc.Steer(m), -m.lock, m.lock);
            m.Cruise(v);
            m.Step();
        }
        m.autoshift = false;
        const double r0 = m.yaw, b0 = m.beta, sp0 = m.speed;
        const double pedal = Clamp(tg / std::max(0.1, g_per_pedal), 0, 1);
        const double t0 = m.t;
        double maxb = 0;
        while (m.t - t0 < 1.0 - 1e-9) {
            m.c.throttle = 0, m.c.clutch_engage = 0, m.c.brake = pedal;
            m.Step();
            maxb = std::max(maxb, std::fabs(m.beta) * kDeg);
        }
        const double dec = (sp0 - m.speed) / 1.0 / kG;
        const double ratio0 = m.yaw / r0, ratioP = m.yaw / (m.speed / R0), db = (m.beta - b0) * kDeg;
        std::printf("  %-10.2f %-10.3f %-14.2f %-14.2f %-12.2f %.1f\n", tg, dec, ratio0, ratioP, db, maxb);
        // Stable: no spin (sideslip change small) and the path curvature stays within 0.6-1.6x.
        if (!(std::fabs(db) < 5 && ratioP > 0.6 && ratioP < 1.6)) all_ok = false;
    }
    Metric("braking-in-turn stable (|dbeta|<5, curv 0.6-1.6)", all_ok ? 1 : 0, 1, 1, "bool",
           "ISO 7975 braking in a turn; road-car stability");
}

// ---------------------------------------------------------------------------------------------
// Drift angle (deg), + = rear out on the outside of a right turn.
double DriftAngle(const Sim& m) { return -m.beta * kDeg; }

void TestPowerOversteer(Sim& m, bool then_hold) {
    if (!then_hold) std::printf("\n== 5. Power oversteer: 2nd gear, 40 km/h on a 30 m circle, full throttle, steer held ==\n");
    else std::printf("\n== 6. Drift hold: PD counter-steer on drift angle, PI throttle to 35 deg, 10 s ==\n");
    const double v = 40 / 3.6;
    m.Launch(v, 0.5, 2);
    m.autoshift = false;
    m.c.gear = 2;
    m.c.clutch_engage = 1;
    CircleCtl pc;
    pc.Start(m, 30, +1);
    for (int i = 0; i < 600; ++i) {
        m.c.steer_angle = Clamp(pc.Steer(m), -m.lock, m.lock);
        m.Cruise(v);
        m.Step();
    }
    const double steer0 = m.c.steer_angle;
    std::printf("  on circle: %.1f km/h, %.0f rpm, gear %d, steer %.2f deg, ay %.2f g, drift %.1f deg\n", m.speed * 3.6, m.s.rpm,
                m.s.gear, m.steer * kDeg, m.speed * m.yaw / kG, DriftAngle(m));
    const double t0 = m.t;
    double t30 = NAN, t10 = NAN, max_slip = 0;
    while (m.t - t0 < 5.0) {
        m.c.throttle = 1, m.c.steer_angle = steer0;
        m.Step();
        max_slip = std::max(max_slip, 0.5 * (m.s.wheel[2].omega + m.s.wheel[3].omega) * m.rw / std::max(1.0, m.vf) - 1);
        if (std::isnan(t10) && DriftAngle(m) > 10) t10 = m.t - t0;
        if (std::isnan(t30) && DriftAngle(m) > 30) { t30 = m.t - t0; break; }
    }
    if (!then_hold) {
        Info("time to 10 deg drift angle", t10, "s");
        Info("max rear wheel slip (wheel speed / vf - 1)", max_slip, "");
        Info("speed / rpm at 30 deg", m.speed * 3.6, "km/h");
        Info("  rpm", m.s.rpm, "rpm");
        // Informal drift-initiation check: a ~1 t RWD car with ~250 Nm, locked/LSD diff, in 2nd at
        // 40 km/h mid-corner breaks the rears loose within ~0.5-2.5 s (drift practice / onboard data).
        Metric("breaks traction: time to 30 deg drift", t30, 0.3, 3.0, "s", "typical lightweight RWD, no standard");
        return;
    }
    if (std::isnan(t30)) {
        std::printf("  never reached 30 deg - cannot start the drift hold\n");
        Metric("drift sustained 10 s", 0, 1, 1, "bool", "drift practice");
        return;
    }
    // Drift controller. Counter-steer (+ = toward the velocity, i.e. left in a right drift):
    // feed-forward = drift angle (front wheels along the car's path) + PD on the angle error.
    const double target = 35;
    double th_i = 0.7, prev = DriftAngle(m), dd = 0;
    std::vector<double> dv, sv, fsv, thv, rpv, spv;
    double min_d = 1e9, max_d = -1e9;
    const double t1 = m.t;
    bool spun = false;
    while (m.t - t1 < 12.0) {
        const double d = DriftAngle(m);
        dd += 0.3 * ((d - prev) / kDt - dd);
        prev = d;
        const double err = d - target;
        const double cs = 1.0 * d + 0.6 * err + 0.08 * dd;   // deg of counter-steer
        m.c.steer_angle = Clamp(-cs / kDeg, -m.lock, m.lock);
        th_i = Clamp(th_i - 0.02 * err * kDt, 0.2, 1.0);
        m.c.throttle = Clamp(th_i - 0.03 * err - 0.004 * dd, 0, 1);
        m.c.gear = m.s.rpm > 7300 ? 3 : (m.s.rpm < 3500 && m.c.gear == 3 ? 2 : m.c.gear);
        m.Step();
        if (std::fabs(d) > 100 || m.speed < 2) { spun = true; break; }
        if (m.t - t1 > 2.0) {
            dv.push_back(d), sv.push_back(-m.steer * kDeg);
            fsv.push_back(0.5 * (m.s.wheel[0].slip_angle + m.s.wheel[1].slip_angle) * kDeg);
            thv.push_back(m.c.throttle), rpv.push_back(m.s.rpm), spv.push_back(m.speed * 3.6);
            min_d = std::min(min_d, d), max_d = std::max(max_d, d);
        }
        if (std::fmod(m.t - t1, 1.0) < kDt * 0.99)
            std::printf("    t %4.1f drift %6.1f  counter-steer %6.1f deg (road wheel)  thr %.2f  %5.0f rpm g%d  %5.1f km/h  front slip %+.1f  column %+.2f Nm\n",
                        m.t - t1, d, -m.steer * kDeg, m.c.throttle, m.s.rpm, m.s.gear, m.speed * 3.6,
                        0.5 * (m.s.wheel[0].slip_angle + m.s.wheel[1].slip_angle) * kDeg, m.s.steer_torque);
    }
    auto mean = [](const std::vector<double>& x) {
        double s = 0;
        for (double v : x) s += v;
        return x.empty() ? NAN : s / x.size();
    };
    const double md = mean(dv), ms = mean(sv);
    Info("mean drift angle (last 10 s)", md, "deg");
    Info("drift angle min / max", min_d, "deg");
    Info("", max_d, "deg");
    Info("mean counter-steer (road wheel)", ms, "deg");
    Info("counter-steer / steer lock", ms / (m.lock * kDeg), "");
    Info("mean front slip angle", mean(fsv), "deg");
    Info("mean throttle", mean(thv), "");
    Info("mean rpm", mean(rpv), "rpm");
    Info("mean speed", mean(spv), "km/h");
    const bool held = !spun && dv.size() > 900 && min_d > 20 && max_d < 50;
    Metric("drift sustained 10 s (20-50 deg, no spin)", held ? 1 : 0, 1, 1, "bool", "drift practice");
    // AC reference (owner's telemetry): counter-steer ~ drift angle at 40-50 deg (~0.67 of lock);
    // physically the front wheels point near the direction of travel, so steer/drift ~0.8-1.2.
    Metric("counter-steer / drift angle", ms / md, 0.7, 1.3, "ratio", "owner's AC telemetry; kinematics");
}

// ---------------------------------------------------------------------------------------------
void TestEngine(Sim& m) {
    std::printf("\n== 7. Engine free rev: gear 1, clutch out (engage 0), brakes on, throttle 0 -> 1 -> 0 ==\n");
    // Note: Chrono's EngineSimpleMap clamps the *reported* speed to [0, max rpm] and evaluates the
    // torque map at that clamped speed, so a pinned 7900 can hide a shaft spinning faster. The
    // time the reading stays pinned after a lift measures that hidden overspeed.
    m.Reset();
    m.c = {0, 0, 1, 0, 0, 1};
    double idle = 0;
    int ni = 0;
    for (int i = 0; i < 300; ++i) {   // idle 3 s
        m.Step();
        if (i >= 200) idle += m.s.rpm, ++ni;
    }
    idle /= ni;
    std::vector<double> tv, rv;
    const double t0 = m.t;
    double t_top = NAN;
    while (m.t - t0 < 5.0) {   // full throttle until the reading tops out, then 1 s more
        m.c.throttle = 1;
        m.Step();
        tv.push_back(m.t - t0), rv.push_back(m.s.rpm);
        if (std::isnan(t_top) && m.s.rpm >= 7890) t_top = m.t - t0;
        if (!std::isnan(t_top) && m.t - t0 > t_top + 1.0) break;
    }
    const double t_lift = m.t - t0;
    double t_unpin = NAN, t6000 = NAN, t2500 = NAN;
    while (m.t - t0 < t_lift + 25.0) {   // lift
        m.c.throttle = 0;
        m.Step();
        const double tt = m.t - t0;
        tv.push_back(tt), rv.push_back(m.s.rpm);
        if (std::isnan(t_unpin) && m.s.rpm < 7890) t_unpin = tt;
        if (std::isnan(t6000) && m.s.rpm <= 6000) t6000 = tt;
        if (std::isnan(t2500) && m.s.rpm <= 2500) { t2500 = tt; break; }
    }
    double up1 = NAN, up2 = NAN;
    const double lo = std::max(2000.0, idle + 300);
    for (size_t i = 1; i < rv.size(); ++i) {
        if (std::isnan(up1) && rv[i] >= lo) up1 = tv[i];
        if (std::isnan(up2) && rv[i] >= 6000) up2 = tv[i];
    }
    const double rise = (6000 - lo) / (up2 - up1);
    const double fall = (6000 - 2500) / (t2500 - t6000);
    const double fall_top = (7890 - 6000) / (t6000 - t_unpin);
    const double pinned = t_unpin - t_lift;
    const double est_peak = 7900 + fall_top * pinned;   // rough: shaft speed when the throttle closed
    // Short 0.2 s blip from rest (heel-toe style).
    for (int i = 0; i < 300; ++i) m.c.throttle = 0, m.Step();
    const double blip0 = m.s.rpm;
    double blip_peak = 0;
    for (int i = 0; i < 20; ++i) m.c.throttle = 1, m.Step(), blip_peak = std::max(blip_peak, m.s.rpm);
    for (int i = 0; i < 100; ++i) m.c.throttle = 0, m.Step(), blip_peak = std::max(blip_peak, m.s.rpm);

    // In gear: 2nd, full throttle with the clutch in; engine speed implied from the rear wheels.
    m.Launch(50 / 3.6, 1.0, 2);
    m.autoshift = false;
    m.c.gear = 2, m.c.clutch_engage = 1;
    for (int i = 0; i < 100; ++i) m.Cruise(50 / 3.6), m.Step();   // steady in 2nd: engine/wheel ratio
    const double k_rpm = m.s.rpm / (0.5 * (m.s.wheel[2].omega + m.s.wheel[3].omega));
    double implied_max = 0, v_end = 0;
    for (int i = 0; i < 800; ++i) {
        m.c.throttle = 1, m.c.steer_angle = Clamp(-0.15 * m.yaw, -0.05, 0.05);
        m.Step();
        implied_max = std::max(implied_max, k_rpm * 0.5 * (m.s.wheel[2].omega + m.s.wheel[3].omega));
        v_end = m.speed;
    }
    std::printf("  2nd gear: %.1f rpm per rad/s of rear wheel (redline 7900 = %.0f km/h at r %.3f m)\n", k_rpm,
                7900 / k_rpm * m.rw * 3.6, m.rw);

    std::printf("  idle reading %.0f rpm; full throttle reaches the 7900 reading after %.2f s\n", idle, t_top);
    std::printf("  after lift the reading stays pinned at 7900 for %.2f s, then falls at %.0f rpm/s\n", pinned, fall_top);
    std::printf("  2nd gear full throttle 8 s: implied engine speed from rear wheels %.0f rpm at %.1f km/h (reading %.0f)\n",
                implied_max, v_end * 3.6, m.s.rpm);
    const char* src = "1.8-2.0 L I4 free-rev (dyno/OBD logs)";
    Metric("idle rpm (throttle 0, clutch out)", idle, 650, 1100, "rpm", "MX-5 idle spec 800-900 rpm");
    // Free-rev of a light 4-cyl with flywheel (~0.1-0.15 kg m^2) and 150-280 Nm: T/I ~ 10-20k rpm/s
    // on paper, ~5-15k rpm/s measured (intake/turbo lag).
    Metric("free-rev rise rate (idle+300 or 2000 -> 6000)", rise, 4000, 16000, "rpm/s", src);
    // Closed-throttle fall (pumping + friction ~30-50 Nm on ~0.13 kg m^2): ~2000-7000 rpm/s.
    Metric("free-rev fall rate (6000 -> 2500)", fall, 1500, 8000, "rpm/s", src);
    // Limiter: fuel cut at 7900 holds the shaft within ~100-200 rpm of the cut.
    Metric("free-rev limiter: est. shaft speed at lift", est_peak, 7500, 8300, "rpm", "limiter 7900 (car data)");
    Metric("in-gear limiter: implied rpm from wheels", implied_max, 7500, 8300, "rpm", "limiter 7900 (car data)");
    Info("0.2 s blip from rest: start", blip0, "rpm");
    Info("0.2 s blip from rest: peak", blip_peak, "rpm");
}
// 8. Rim in the loop: start and settle a drift with the test driver, then hand the steering to a
// simulated G920 rim driven only by force feedback (column torque scaled to the wheel's motor);
// the driver keeps the throttle. A real car's wheel winds into countersteer by itself (front
// wheels follow the direction of travel); in Assetto Corsa the owner's countersteer tracks the
// drift angle ~1:1. Hands-off and light-hands (extra damping) variants.
void TestRimLoop(Sim& m, double hand_damping, const char* label, double t_ref) {
    std::printf("\n== 8. Rim in the loop (%s): simulated G920 driven by the column torque, %.0f Nm = full force ==\n", label, t_ref);
    const double v = 40 / 3.6;
    m.Launch(v, 0.5, 2);
    m.autoshift = false;
    m.c.gear = 2;
    m.c.clutch_engage = 1;
    CircleCtl pc;
    pc.Start(m, 30, +1);
    for (int i = 0; i < 600; ++i) {
        m.c.steer_angle = Clamp(pc.Steer(m), -m.lock, m.lock);
        m.Cruise(v);
        m.Step();
    }
    const double steer0 = m.c.steer_angle;
    const double t0 = m.t;
    while (m.t - t0 < 5.0 && DriftAngle(m) < 30) {
        m.c.throttle = 1, m.c.steer_angle = steer0;
        m.Step();
    }
    const double target = 30, ratio = 8.5;
    double th_i = 0.7, prev = DriftAngle(m), dd = 0;
    // Settle with the test driver for 3 s.
    for (int i = 0; i < 300; ++i) {
        const double d = DriftAngle(m);
        dd += 0.3 * ((d - prev) / kDt - dd);
        prev = d;
        const double err = d - target;
        m.c.steer_angle = Clamp(-(1.0 * d + 0.6 * err + 0.08 * dd) / kDeg, -m.lock, m.lock);
        th_i = Clamp(th_i - 0.02 * err * kDt, 0.2, 1.0);
        m.c.throttle = Clamp(th_i - 0.03 * err - 0.004 * dd, 0, 1);
        m.c.gear = m.s.rpm > 7300 ? 3 : (m.s.rpm < 3500 && m.c.gear == 3 ? 2 : m.c.gear);
        m.Step();
    }
    std::printf("  hand-over: drift %.1f deg, counter-steer %.1f deg, %.1f km/h, column %+.1f Nm\n", DriftAngle(m), -m.steer * kDeg,
                m.speed * 3.6, m.s.steer_torque);
    // G920: ~2.2 Nm motor, rim + motor inertia ~0.035 kg m^2, gear drag ~0.03 N m s/rad + ~0.04 N m Coulomb.
    const double J = 0.035, b = 0.03 + hand_damping, coul = 0.04, tmax = 2.2;
    double th = m.c.steer_angle * ratio, w = 0;   // rim angle (rad, + right) and speed
    std::vector<double> dv, sv, fsv;
    bool spun = false;
    const double t1 = m.t;
    while (m.t - t1 < 8.0) {
        const double d = DriftAngle(m);
        dd += 0.3 * ((d - prev) / kDt - dd);
        prev = d;
        const double err = d - target;
        th_i = Clamp(th_i - 0.02 * err * kDt, 0.2, 1.0);
        m.c.throttle = Clamp(th_i - 0.03 * err - 0.004 * dd, 0, 1);
        m.c.gear = m.s.rpm > 7300 ? 3 : (m.s.rpm < 3500 && m.c.gear == 3 ? 2 : m.c.gear);
        // Rim dynamics over the frame (10 substeps).
        const double motor = Clamp(m.s.steer_torque / t_ref, -1, 1) * tmax;
        for (int k = 0; k < 10; ++k) {
            const double h = kDt / 10;
            double tq = motor - b * w;
            if (std::fabs(w) > 1e-3) tq -= coul * (w > 0 ? 1 : -1);
            else if (std::fabs(tq) < coul) tq = 0;
            w += tq / J * h;
            th += w * h;
            const double lim = m.lock * ratio;
            if (std::fabs(th) > lim) th = Clamp(th, -lim, lim), w = 0;
        }
        m.c.steer_angle = th / ratio;
        m.Step();
        if (std::getenv("DC_RIM_TRACE") && m.t - t1 > 1.0)
            std::printf("      t %.3f v %.2f yaw %+.2f th %+.0f tq %+.1f om %.1f %.1f %.1f %.1f fz %.0f %.0f %.0f %.0f\n", m.t - t1,
                        m.speed, m.s.ang_vel[1], th * kDeg, m.s.steer_torque, m.s.wheel[0].omega, m.s.wheel[1].omega,
                        m.s.wheel[2].omega, m.s.wheel[3].omega, m.s.wheel[0].fz, m.s.wheel[1].fz, m.s.wheel[2].fz,
                        m.s.wheel[3].fz);
        if (!std::isfinite(m.speed) || !std::isfinite(m.s.steer_torque)) {
            std::printf("  NaN in the physics state at t %.3f\n", m.t - t1);
            spun = true;
            break;
        }
        if (std::fabs(d) > 100 || m.speed < 2) { spun = true; break; }
        if (m.t - t1 > 2.0) {
            dv.push_back(d), sv.push_back(-m.steer * kDeg);
            fsv.push_back(0.5 * (m.s.wheel[0].slip_angle + m.s.wheel[1].slip_angle) * kDeg);
        }
        if (std::fmod(m.t - t1, 1.0) < kDt * 0.99)
            std::printf("    t %4.1f drift %6.1f  counter-steer %6.1f deg (road wheel)  rim %+6.0f deg  ffb %+.2f  front slip %+.1f\n",
                        m.t - t1, d, -m.steer * kDeg, th * kDeg, motor / tmax,
                        0.5 * (m.s.wheel[0].slip_angle + m.s.wheel[1].slip_angle) * kDeg);
    }
    auto mean = [](const std::vector<double>& x) {
        double s = 0;
        for (double v : x) s += v;
        return x.empty() ? NAN : s / x.size();
    };
    Info("mean drift angle", mean(dv), "deg");
    Info("mean counter-steer (road wheel)", mean(sv), "deg");
    Info("mean front slip angle", mean(fsv), "deg");
    Metric("drift held with the rim on its own (no spin)", spun ? 0 : 1, 1, 1, "bool", "real cars / AC: the wheel self-steers");
    Metric("rim counter-steer / drift angle", mean(sv) / mean(dv), 0.7, 1.3, "ratio", "owner's AC telemetry; kinematics");
}
}  // namespace

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    if (argc < 3) {
        std::printf("usage: dc_maneuvers <car root> <car id> [tests 1..7]\n");
        return 2;
    }
    char err[512] = {};
    Sim m;
    m.car = dc_create(argv[1], argv[2], err, sizeof(err));
    if (!m.car) {
        std::printf("create failed: %s\n", err);
        return 1;
    }
    dc_set_ground(m.car, Flat, nullptr);
    bool run[9] = {};
    if (argc == 3) std::fill(run, run + 9, true);
    for (int i = 3; i < argc; ++i) {
        const int k = std::atoi(argv[i]);
        if (k >= 1 && k <= 8) run[k] = true;
    }
    const auto w0 = std::chrono::steady_clock::now();
    m.Reset();
    std::printf("car %s: wheelbase %.3f m, steer lock %.1f deg\n", argv[2], m.L, m.lock * kDeg);
    if (run[1]) TestCircle(m);
    if (run[2]) TestStep(m);
    if (run[3]) TestFreq(m);
    if (run[4]) TestBrake(m);
    if (run[5]) TestPowerOversteer(m, false);
    if (run[6]) TestPowerOversteer(m, true);
    if (run[7]) TestEngine(m);
    const double t_ref = std::getenv("DC_FFB_TREF") ? std::atof(std::getenv("DC_FFB_TREF")) : 80.0;
    if (run[8]) TestRimLoop(m, 0.0, "hands off", t_ref);
    if (run[8]) TestRimLoop(m, 0.15, "light hands", t_ref);
    const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - w0).count();
    std::printf("\n%d passed, %d failed (%.0f s wall)\n", g_pass, g_fail, wall);
    dc_destroy(m.car);
    return g_fail ? 1 : 0;
}
