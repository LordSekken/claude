// DriftChrono smoke test: flat ground at y = 0 and a wall across the road,
// all through the C interface the game uses.
//   dc_smoke <car root> <car id>
#include <cmath>
#include <cstdio>
#include <initializer_list>

#include "drift_chrono.h"

namespace {
int Flat(void*, const double from[3], const double to[3], double p[3], double n[3], double* grip) {
    if (from[1] < 0 || to[1] > 0) return 0;
    const double t = from[1] / (from[1] - to[1]);
    for (int i = 0; i < 3; ++i) p[i] = from[i] + (to[i] - from[i]) * t;
    n[0] = 0, n[1] = 1, n[2] = 0;
    *grip = 1.0;
    return 1;
}

// Flat ground with a 15 cm kerb (vertical step) for z < kKerbZ.
constexpr double kKerbZ = -25.0;
int Kerb(void*, const double from[3], const double to[3], double p[3], double n[3], double* grip) {
    const double h = from[2] < kKerbZ ? 0.15 : 0.0;
    if (from[1] < h || to[1] > h) return 0;
    const double t = (from[1] - h) / (from[1] - to[1]);
    for (int i = 0; i < 3; ++i) p[i] = from[i] + (to[i] - from[i]) * t;
    n[0] = 0, n[1] = 1, n[2] = 0;
    *grip = 1.0;
    return 1;
}

int failures = 0;
void Check(bool ok, const char* what) {
    std::printf("%s  %s\n", ok ? "ok  " : "FAIL", what);
    if (!ok) ++failures;
}

double Speed(const dc_state& s) { return std::sqrt(s.vel[0] * s.vel[0] + s.vel[2] * s.vel[2]); }
}  // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::printf("usage: dc_smoke <car root> <car id>\n");
        return 2;
    }
    char err[512] = {};
    dc_car* car = dc_create(argv[1], argv[2], err, sizeof(err));
    if (!car) {
        std::printf("create failed: %s\n", err);
        return 1;
    }
    dc_set_ground(car, Flat, nullptr);
    const double origin[3] = {0, 0.30, 0};
    const double rot[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    dc_reset(car, origin, rot);
    dc_state s;
    dc_controls c = {0, 0, 1, 0, 1, 0};

    for (int i = 0; i < 100; ++i) {
        if (i < 3) {
            dc_get_state(car, &s);
            std::printf("  step %d: origin %.3f %.3f %.3f fz %.0f wheel0 local %.2f %.2f %.2f\n", i, s.origin[0], s.origin[1],
                        s.origin[2], s.wheel[0].fz, s.wheel[0].center_local[0], s.wheel[0].center_local[1],
                        s.wheel[0].center_local[2]);
        }
        dc_step(car, i < 3 ? 0.001 : 0.01, &c, nullptr, 0);
    }
    dc_get_state(car, &s);
    double fz = 0;
    for (auto& w : s.wheel) fz += w.fz;
    std::printf("settled: origin y %.3f, wheel loads %.0f %.0f %.0f %.0f (sum %.0f N), lock %.1f deg\n", s.origin[1],
                s.wheel[0].fz, s.wheel[1].fz, s.wheel[2].fz, s.wheel[3].fz, fz, s.steer_lock * 57.3);
    Check(std::fabs(s.origin[1] - 0.275) < 0.05, "rests at about wheel-centre height");
    Check(fz > 9000 && fz < 13000, "wheel loads carry the car's weight");
    Check(s.wheel[0].center_local[0] < 0 && s.wheel[0].center_local[2] < 0, "wheel 0 is front-left (x<0, z<0)");

    c = {0, 0.6, 0, 0, 1, 1};
    double us = 0;
    int n = 0;
    for (int i = 0; i < 400; ++i, ++n) {
        dc_step(car, 0.01, &c, nullptr, 0);
        us += dc_last_step_us(car);
    }
    dc_get_state(car, &s);
    std::printf("after 4 s in 1st: %.1f km/h, origin z %.1f, %.0f rpm, gear %d, %.0f us per 10 ms frame\n",
                Speed(s) * 3.6, s.origin[2], s.rpm, s.gear, us / n);
    Check(s.origin[2] < -5 && s.vel[2] < -3, "drives forward (-z)");
    Check(s.wheel[2].omega > 5, "rear wheels roll forward (+omega)");

    c.gear = 2;
    c.steer_angle = 0.15;
    for (int i = 0; i < 150; ++i) dc_step(car, 0.01, &c, nullptr, 0);
    dc_get_state(car, &s);
    std::printf("steer right 8.6 deg: yaw rate %+.3f, wheel steer FL %+.1f FR %+.1f deg, rack force %+.0f N, lat Fy FL %+.0f\n",
                s.ang_vel[1], s.wheel[0].steer * 57.3, s.wheel[1].steer * 57.3, s.rack_force, s.wheel[0].fy);
    Check(s.ang_vel[1] < -0.1, "turns right (negative yaw rate about +y)");
    Check(s.wheel[0].steer > 0.05 && s.wheel[1].steer > 0.05, "front wheels steer right (+)");
    Check(s.rack_force < -500, "road pushes the rack back towards centre (self-aligning)");
    std::printf("  front slip angle in the right turn: FL %+.2f FR %+.2f deg\n", s.wheel[0].slip_angle * 57.3, s.wheel[1].slip_angle * 57.3);
    Check(s.wheel[0].slip_angle < -0.005, "slip angle in DriftCore's sign (+ = travelling right of the wheel)");
    Check(s.wheel[0].fy > 500, "front tyre lateral force points right (into the turn)");

    // Smoothness: cruising straight at ~30 km/h, the rack force and the front wheel positions must
    // be steady sample to sample (the owner's wheel buzzed: the iterative solver's joint forces were noisy).
    {
        dc_reset(car, origin, rot);
        c = {0, 0.5, 0, 0, 1, 1};
        for (int i = 0; i < 800; ++i) {
            dc_step(car, 0.0167, &c, nullptr, 0);
            dc_get_state(car, &s);
            c.throttle = Speed(s) > 30 / 3.6 ? 0.15 : 0.5;
        }
        double prev_rack = 0, prev_y = 0, jr = 0, jy = 0, sum_rack = 0;
        for (int i = 0; i < 200; ++i) {
            // A wheel sends a slightly different angle every frame (60 fps), like the owner's G920.
            c.steer_angle = 0.01 * std::sin(i * 0.15);
            dc_step(car, 0.0167, &c, nullptr, 0);
            dc_get_state(car, &s);
            if (i) jr += std::fabs(s.rack_force - prev_rack), jy += std::fabs(s.wheel[0].center_local[1] - prev_y);
            prev_rack = s.rack_force, prev_y = s.wheel[0].center_local[1];
            sum_rack += s.rack_force;
        }
        std::printf("straight %.0f km/h: rack mean %+.0f N, mean step change %.0f N; wheel height jitter %.2f mm\n",
                    Speed(s) * 3.6, sum_rack / 200, jr / 199, jy / 199 * 1000);
        Check(jr / 199 < 100 && jy / 199 < 0.5, "rack force and wheels are steady when cruising");
    }

    // Clutch open: full throttle in gear with the clutch down must rev the engine, not move the car.
    {
        dc_reset(car, origin, rot);
        c = {0, 0, 1, 0, 1, 1};
        for (int i = 0; i < 100; ++i) dc_step(car, 0.01, &c, nullptr, 0);
        c = {0, 1, 0, 0, 0, 1};
        for (int i = 0; i < 150; ++i) dc_step(car, 0.01, &c, nullptr, 0);
        dc_get_state(car, &s);
        std::printf("clutch down, full throttle 1.5 s: %.1f km/h, %.0f rpm\n", Speed(s) * 3.6, s.rpm);
        Check(Speed(s) < 2 && s.rpm > 3000, "clutch disconnects the engine");
    }

    // Low-speed launch, wheel straight: the rack shouldn't be yanked (Unreal showed full FFB at ~10 km/h).
    dc_reset(car, origin, rot);
    c = {0, 0.35, 0, 0, 1, 1};
    double max_rack = 0;
    for (int i = 0; i < 600; ++i) {
        dc_step(car, 0.01, &c, nullptr, 0);
        dc_get_state(car, &s);
        if (i > 30 && std::fabs(s.rack_force) > max_rack) {   // skip the reset itself (rack re-centring in the air)
            max_rack = std::fabs(s.rack_force);
            if (max_rack > 1500)
                std::printf("  launch t %.2f %.1f km/h rack %+.0f N  Fy FL %+.0f FR %+.0f  Fx FL %+.0f  steer FL %+.3f FR %+.3f deg"
                            "  Fz FL %.0f FR %.0f  wheel y FL %.3f  rpm %.0f\n",
                            i * 0.01, Speed(s) * 3.6, s.rack_force, s.wheel[0].fy, s.wheel[1].fy, s.wheel[0].fx,
                            s.wheel[0].steer * 57.3, s.wheel[1].steer * 57.3, s.wheel[0].fz, s.wheel[1].fz,
                            s.wheel[0].center_local[1], s.rpm);
        }
    }
    std::printf("launch straight to %.0f km/h: max |rack force| %.0f N\n", Speed(s) * 3.6, max_rack);
    Check(max_rack < 2000, "straight launch keeps the rack force small");

    // Gentle turn (the Unreal test that spun): 1st gear, 35 % throttle to ~48 km/h, then 2.1 deg of steering.
    dc_reset(car, origin, rot);
    c = {0, 0.35, 0, 0, 1, 1};
    for (int i = 0; i < 1200; ++i) {
        dc_step(car, 0.01, &c, nullptr, 0);
        dc_get_state(car, &s);
        if (Speed(s) > 48 / 3.6) break;
    }
    c.steer_angle = 0.04 * s.steer_lock;
    double max_drift = 0;
    for (int i = 0; i < 300; ++i) {
        dc_step(car, 0.01, &c, nullptr, 0);
        dc_get_state(car, &s);
        const double fwd_x = -s.rot[6], fwd_z = -s.rot[8], right_x = s.rot[0], right_z = s.rot[2];
        const double drift = std::atan2(s.vel[0] * right_x + s.vel[2] * right_z, s.vel[0] * fwd_x + s.vel[2] * fwd_z) * 57.3;
        max_drift = std::fmax(max_drift, std::fabs(drift));
        if (i % 50 == 0)
            std::printf("  turn t %.1f: %.1f km/h drift %+.1f yaw %+.2f rpm %.0f fz %.0f %.0f %.0f %.0f\n", i * 0.01, Speed(s) * 3.6,
                        drift, s.ang_vel[1], s.rpm, s.wheel[0].fz, s.wheel[1].fz, s.wheel[2].fz, s.wheel[3].fz);
    }
    Check(max_drift < 10, "2 deg of steering at 48 km/h on part throttle doesn't spin it");

    // Wall: fresh straight run, then a wall 15 m ahead along the heading.
    dc_reset(car, origin, rot);
    c = {0, 0.6, 0, 0, 1, 1};
    for (int i = 0; i < 300; ++i) dc_step(car, 0.01, &c, nullptr, 0);
    dc_get_state(car, &s);
    const double fwd[3] = {-s.rot[6], -s.rot[7], -s.rot[8]};
    const double wall = (s.origin[0] * fwd[0] + s.origin[2] * fwd[2]) + 15.0;   // distance along heading
    const double v0 = Speed(s);
    c.throttle = 0.4;
    double min_gap = 1e9;
    for (int i = 0; i < 400; ++i) {
        dc_get_state(car, &s);
        const double nose = (s.origin[0] * fwd[0] + s.origin[2] * fwd[2]) + 2.0;
        dc_contact ct = {};
        int count = 0;
        if (nose > wall) {
            for (int k = 0; k < 3; ++k) ct.point[k] = s.origin[k] + fwd[k] * 2.0;
            ct.normal[0] = -fwd[0], ct.normal[1] = 0, ct.normal[2] = -fwd[2];
            ct.depth = nose - wall;
            count = 1;
        }
        min_gap = std::fmin(min_gap, wall - nose);
        dc_step(car, 0.01, &c, &ct, count);
    }
    dc_get_state(car, &s);
    std::printf("wall: hit at %.1f km/h, deepest %.2f m, now %.1f km/h\n", v0 * 3.6, -min_gap, Speed(s) * 3.6);
    Check(-min_gap < 0.5 && Speed(s) < 5, "wall stops the car without passing through");

    // Kerb: 15 cm step crossed at ~12 km/h, straight on and at 30 deg.
    for (double yaw_deg : {0.0, 30.0}) {
        dc_destroy(car);
        car = dc_create(argv[1], argv[2], err, sizeof(err));
        dc_set_ground(car, Kerb, nullptr);
        const double a = yaw_deg / 57.2958;
        // DriftCore columns (right, up, back) for a heading turned by `a` to the left.
        const double r9[9] = {std::cos(a), 0, -std::sin(a), 0, 1, 0, std::sin(a), 0, std::cos(a)};
        const double o[3] = {0, 0.30, -10.0};
        dc_reset(car, o, r9);
        c = {0, 0.25, 0, 0, 1, 1};
        double max_roll = 0, max_fz = 0, top_y = 0;
        for (int i = 0; i < 800; ++i) {
            dc_step(car, 0.01, &c, nullptr, 0);
            dc_get_state(car, &s);
            c.throttle = Speed(s) > 12 / 3.6 ? 0.15 : 0.4;   // hold ~12 km/h, keep pushing at the kerb
            max_roll = std::fmax(max_roll, std::fabs(std::asin(std::fmin(1.0, std::fabs(s.rot[1])))) * 57.3);   // right.y
            for (auto& w : s.wheel) max_fz = std::fmax(max_fz, w.fz);
            top_y = std::fmax(top_y, s.origin[1]);
        }
        std::printf("kerb at %.0f deg: max roll %.1f deg, max wheel load %.0f N, origin now y %.2f (top %.2f), z %.1f\n",
                    yaw_deg, max_roll, max_fz, s.origin[1], top_y, s.origin[2]);
        // A square 15 cm step at 12 km/h is a real impact (~20-25 kN on a tyre); the old single-point
        // contact gave a 23 kN kick sideways that rolled the car 84 deg.
        Check(max_roll < 10 && max_fz < 35000 && top_y - s.origin[1] < 0.3, "climbs a 15 cm kerb without a launch or a roll");
        Check(s.origin[2] < kKerbZ - 3 && s.origin[1] > 0.38, "ends up on top of the kerb");
    }
    // Reset restores the design layout: loads symmetric left/right afterwards.
    dc_set_ground(car, Flat, nullptr);
    dc_reset(car, origin, rot);
    c = {0, 0, 1, 0, 1, 0};
    for (int i = 0; i < 150; ++i) dc_step(car, 0.01, &c, nullptr, 0);
    dc_get_state(car, &s);
    std::printf("after reset: loads %.0f %.0f %.0f %.0f\n", s.wheel[0].fz, s.wheel[1].fz, s.wheel[2].fz, s.wheel[3].fz);
    Check(std::fabs(s.wheel[0].fz - s.wheel[1].fz) < 150 && std::fabs(s.wheel[2].fz - s.wheel[3].fz) < 150,
          "reset restores a level, symmetric car");

    dc_destroy(car);
    std::printf(failures ? "%d FAILED\n" : "all passed\n", failures);
    return failures ? 1 : 0;
}
