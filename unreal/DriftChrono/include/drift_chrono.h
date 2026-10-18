// DriftChrono: Project Chrono vehicle physics behind a plain C interface, so
// hosts (Unreal, test tools) never include Chrono's headers (Unreal builds
// without RTTI/exceptions; Chrono needs both).
//
// Everything is in DriftCore's frame: right-handed metres, +X right, +Y up,
// -Z forward; rotations are 3x3 matrices stored as columns (right, up, back).
// "Origin" is DriftCore's body origin: midway between the axles at wheel-centre
// height, so the same car models and cameras line up with either backend.
// Wheels: 0 FL, 1 FR, 2 RL, 3 RR.
#pragma once

#ifdef _WIN32
#ifdef DRIFT_CHRONO_EXPORTS
#define DC_API __declspec(dllexport)
#else
#define DC_API __declspec(dllimport)
#endif
#else
#define DC_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define DC_VERSION 1

typedef struct dc_car dc_car;

/* Ground under a point: cast from `from` to `to`; on a hit fill point/normal and the
   surface grip factor (1 = dry tarmac) and return 1, else return 0. */
typedef int (*dc_raycast_fn)(void* ctx, const double from[3], const double to[3], double point[3], double normal[3],
                             double* grip);

/* A penetration of the car body found by the host this frame (walls, other cars). */
typedef struct dc_contact {
    double point[3];
    double normal[3];   /* out of the obstacle */
    double depth;
    double other_vel[3];
    double other_mass;  /* 0 = immovable */
} dc_contact;

/* Driver controls after the host's driver aids (assist, auto clutch, auto shift). */
typedef struct dc_controls {
    double steer_angle;   /* mean front road-wheel angle, rad, + = right */
    double throttle;      /* 0..1 */
    double brake;         /* 0..1 */
    double handbrake;     /* 0..1 */
    double clutch_engage; /* 0 = pedal down .. 1 = engaged */
    int gear;             /* -1 R, 0 N, 1.. */
} dc_controls;

typedef struct dc_wheel {
    double center_local[3]; /* wheel centre in the body frame (relative to origin) */
    double steer;           /* rad, + = right */
    double omega;           /* rad/s, + = rolling forward */
    double spin;            /* accumulated spin angle, rad */
    int contact;
    double point[3];        /* contact point, world */
    double fz, fx, fy;      /* tyre forces, N (fx + = forward, fy + = right) */
    double slip_ratio;
    double slip_angle;      /* rad */
} dc_wheel;

typedef struct dc_state {
    double origin[3];
    double rot[9];
    double vel[3];         /* centre-of-mass velocity, world */
    double ang_vel[3];     /* world */
    dc_wheel wheel[4];
    double rpm;
    int gear;
    double rack_force;     /* steering rack force, N, + = pushes the wheels to the right */
    double steer_lock;     /* road-wheel angle at full steering input, rad */
    double sim_time;
} dc_state;

DC_API int dc_version(void);

/* Loads <car_root>/<car_id>/car.json (written by drift-sim/tools/ac_to_chrono.py).
   Returns NULL on failure with a message in err. */
DC_API dc_car* dc_create(const char* car_root, const char* car_id, char* err, int err_len);
DC_API void dc_destroy(dc_car* car);

DC_API void dc_set_ground(dc_car* car, dc_raycast_fn fn, void* ctx);

/* Places the car (origin + rotation), at rest. */
DC_API void dc_reset(dc_car* car, const double origin[3], const double rot[9]);

/* Advances by dt in fixed 1 ms steps (leftover time carries over). The contacts
   are this frame's body penetrations; they act as springs until the next call.
   Returns the number of physics steps taken. */
DC_API int dc_step(dc_car* car, double dt, const dc_controls* controls, const dc_contact* contacts, int contact_count);

DC_API void dc_get_state(const dc_car* car, dc_state* out);

/* Wall time of the last dc_step in microseconds (profiling). */
DC_API double dc_last_step_us(const dc_car* car);

#ifdef __cplusplus
}
#endif
