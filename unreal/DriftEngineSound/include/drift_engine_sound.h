/*
 * drift_engine_sound: headless engine sound from AngeTheGreat's engine-sim
 * (MIT, https://github.com/ange-yaghi/engine-sim) behind a plain C API.
 *
 * engine-sim simulates the engine (crank/rods/pistons, gas dynamics in cylinders, intake and
 * exhaust) and synthesises sound from the exhaust pressure pulses convolved with an impulse
 * response. This library runs that synchronously (no audio thread): the host advances the
 * simulation and pulls mono float PCM at 44.1 kHz.
 *
 * Typical game use: DES_MODE_HOLD with des_set_target_rpm() = the vehicle physics' engine rpm
 * each frame and des_set_throttle() = the pedal; call des_render() from the audio callback (or
 * des_advance() + des_read() from the game thread). One handle per engine; a handle must not be
 * used from two threads at once. Loading (des_create) is serialised internally.
 *
 * Coordinates/units: rpm, seconds, N m. All functions are exception-free.
 */
#ifndef DRIFT_ENGINE_SOUND_H
#define DRIFT_ENGINE_SOUND_H

#ifdef _WIN32
#  ifdef DES_EXPORTS
#    define DES_API __declspec(dllexport)
#  else
#    define DES_API __declspec(dllimport)
#  endif
#else
#  define DES_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define DES_SAMPLE_RATE 44100

typedef struct des_engine des_engine;

enum des_mode {
    DES_MODE_FREE = 0,   /* no dyno: the engine revs freely in neutral (auto starter if stalled) */
    DES_MODE_HOLD = 1,   /* two-sided dyno: crank held at the target rpm (drives it on overrun) */
    DES_MODE_ABSORB = 2  /* one-sided dyno: brakes the engine down to the target rpm only */
};

/* Optional induction-noise layer (engine-sim itself only voices the exhaust). The source is the
 * simulated plenum pressure pulsation + flow noise, shaped by a resonance (airbox/snorkel) and
 * a low-pass (filter element). gain 0 disables it (default). */
typedef struct des_intake_sound {
    float gain;          /* overall level relative to the exhaust (0 = off) */
    float resonance_hz;  /* main intake resonance (airbox Helmholtz ~100-200 Hz, cone ~300-600) */
    float resonance_q;   /* sharpness of that resonance */
    float lowpass_hz;    /* filter element / box muffling (stock ~800, open cone ~4000) */
    float hiss;          /* broadband flow noise share 0..1 (induction "whoosh") */
} des_intake_sound;

typedef struct des_info {
    int cylinders;
    int exhaust_systems;
    double displacement_l;
    double redline_rpm;
    double simulation_hz;   /* physics steps per second */
    char name[64];
} des_info;

/* Compile an engine-sim .mr script (a top-level script that calls main(), like assets/main.mr)
 * and build the simulation. library_dir = engine-sim checkout root (its es/ holds engine_sim.mr
 * and the impulse-response library) or the es/ folder itself. On failure returns NULL and writes
 * the compiler errors to err (if given). */
DES_API des_engine *des_create(const char *script_path, const char *library_dir,
                               char *err, int err_len);
DES_API void des_destroy(des_engine *e);

DES_API void des_get_info(const des_engine *e, des_info *out);

/* Controls (take effect smoothly over the next advance). */
DES_API void des_set_throttle(des_engine *e, double throttle01);
DES_API void des_set_mode(des_engine *e, int mode);
DES_API void des_set_target_rpm(des_engine *e, double rpm);
DES_API void des_set_ignition(des_engine *e, int on);
DES_API void des_set_starter(des_engine *e, int on);      /* manual starter (free mode) */
DES_API void des_set_auto_starter(des_engine *e, int on); /* default on */
DES_API void des_set_gain(des_engine *e, double gain);    /* output gain, default 1 */
DES_API void des_set_leveler(des_engine *e, int on);      /* engine-sim's AGC; default off */
DES_API void des_set_intake_sound(des_engine *e, const des_intake_sound *p);

/* Simulate dt seconds; the audio produced is queued (read it with des_read). Returns the number
 * of samples queued now, or -1 on error. */
DES_API int des_advance(des_engine *e, double dt);
DES_API int des_available(const des_engine *e);
/* Pop up to max_samples queued samples into out; returns the count. */
DES_API int des_read(des_engine *e, float *out, int max_samples);
/* Audio-callback style: simulate exactly as far as needed and write n samples. Returns n or -1. */
DES_API int des_render(des_engine *e, float *out, int n);

/* State */
DES_API double des_get_rpm(const des_engine *e);
DES_API double des_get_dyno_torque(const des_engine *e); /* N m absorbed by the dyno, averaged over
                                                            one engine cycle (+ = engine driving it,
                                                            - = dyno motoring it) */
DES_API double des_get_manifold_pressure(const des_engine *e); /* Pa absolute */
DES_API double des_get_sim_time(const des_engine *e);    /* simulated seconds */
DES_API double des_get_cpu_time(const des_engine *e);    /* wall seconds spent simulating+rendering */

#ifdef __cplusplus
}
#endif

#endif /* DRIFT_ENGINE_SOUND_H */
