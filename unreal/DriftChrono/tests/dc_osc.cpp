// Straight-line wheel-speed oscillation check (owner's centre "shake", 2026-10-19): drives the car
// straight on flat ground at low speed in gear and in neutral and reports the left/right wheel-speed
// split and steering-column torque swing. A locked/LSD axle must keep L/R together.
//   dc_osc <ChronoCars root> <car id>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include "drift_chrono.h"

static int Flat(void*, const double f[3], const double t[3], double p[3], double n[3], double* grip) {
    if (f[1] < 0.0 || t[1] > 0.0) return 0;
    const double a = f[1] / (f[1] - t[1]);
    for (int i = 0; i < 3; ++i) p[i] = f[i] + (t[i] - f[i]) * a;
    n[0] = 0; n[1] = 1; n[2] = 0;
    *grip = 1.0;
    return 1;
}

int main(int argc, char** argv) {
    if (argc < 3) return 2;
    char err[512];
    dc_car* car = dc_create(argv[1], argv[2], err, sizeof err);
    if (!car) { std::printf("create failed: %s\n", err); return 1; }
    dc_set_ground(car, Flat, nullptr);
    const double o[3] = {0, 0.5, 0}, r[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    dc_reset(car, o, r);
    dc_controls c{0, 0, 0, 0, 1, 1};
    dc_state s;
    const double dt = 1.0 / 120.0;
    int bad = 0;
    struct Phase { const char* name; double secs; int gear; double engage; double target_kmh; } phases[] = {
        {"launch 1st", 4, 1, 1, 15}, {"2nd 18 km/h", 6, 2, 1, 18}, {"neutral, clutch in, gas", 3, 0, 0, -1},
        {"2nd 25 km/h", 5, 2, 1, 25}, {"3rd 40 km/h", 5, 3, 1, 40}};
    for (const Phase& ph : phases) {
        double split_max = 0, split_f = 0, tq_min = 1e9, tq_max = -1e9;
        const int n = static_cast<int>(ph.secs / dt);
        for (int i = 0; i < n; ++i) {
            dc_get_state(car, &s);
            const double v = std::sqrt(s.vel[0] * s.vel[0] + s.vel[2] * s.vel[2]) * 3.6;
            c.gear = ph.gear;
            c.clutch_engage = ph.engage;
            c.throttle = ph.target_kmh < 0 ? 1.0 : std::fmin(1.0, std::fmax(0.0, 0.15 + 0.08 * (ph.target_kmh - v)));
            dc_step(car, dt, &c, nullptr, 0);
            dc_get_state(car, &s);
            if (std::getenv("DC_OSC_TRACE") && i % 3 == 0 && i > n / 2 && i < n / 2 + 90)
                std::printf("  t %.3f thr %.2f v %.2f yaw %+.3f omF %.2f %.2f fxF %+5.0f %+5.0f fyF %+5.0f %+5.0f tq %+6.2f\n", i * dt,
                            c.throttle, v, s.ang_vel[1], s.wheel[0].omega, s.wheel[1].omega, s.wheel[0].fx, s.wheel[1].fx,
                            s.wheel[0].fy, s.wheel[1].fy, s.steer_torque);
            if (i > n / 3) {
                split_max = std::fmax(split_max, std::fabs(s.wheel[2].omega - s.wheel[3].omega));
                split_f = std::fmax(split_f, std::fabs(s.wheel[0].omega - s.wheel[1].omega));
                tq_min = std::fmin(tq_min, s.steer_torque);
                tq_max = std::fmax(tq_max, s.steer_torque);
            }
        }
        const double v = std::sqrt(s.vel[0] * s.vel[0] + s.vel[2] * s.vel[2]) * 3.6;
        const bool ok = split_max < 1.0 && split_f < 1.0 && tq_max - tq_min < 3.0;
        bad += ok ? 0 : 1;
        std::printf("%-26s %5.1f km/h  rear L-R max %6.2f rad/s  front L-R max %5.2f  column torque p-p %6.2f N m  %s\n",
                    ph.name, v, split_max, split_f, tq_max - tq_min, ok ? "ok" : "FAIL");
    }
    dc_destroy(car);
    return bad ? 1 : 0;
}
