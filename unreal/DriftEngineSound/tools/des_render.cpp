// des_render: renders the 1998 Subaru Outback EJ25 variants with drift_engine_sound to
// 16-bit 44.1 kHz mono WAVs and checks them (silence, clipping, firing-frequency tracking).
//
//   des_render <engines dir> <engine-sim dir> <out dir> [variant substring]
//
// Each WAV: 2 s idle (free-running) | 6 s full-throttle sweep idle->6000 rpm (dyno held,
// ramping) | 0.25 s lift 6000->5000 + 2 s overrun 5000->2000 at 0 throttle (dyno drives) |
// 0.5 s 2000->3500 + 3 s steady 3500 rpm cruise at 30 % throttle.
// All variants share one normalisation gain (loudest peak -> -1 dBFS) so relative loudness is kept.
#include "drift_engine_sound.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <cstdint>
#include <string>
#include <vector>

namespace {

const double kFs = DES_SAMPLE_RATE;
const double kBlock = 0.005;  // control update period (s), like a 200 Hz game tick

struct Variant {
    const char *name;
    const char *script;
    des_intake_sound intake;
    const char *desc;
};

struct Segment { const char *name; double t0, t1; double rpm0, rpm1; };

struct Render {
    std::vector<float> pcm;
    std::vector<double> rpmTrack;  // per block: commanded (hold) or measured (free) rpm
    std::vector<double> mapTrack, trqTrack;  // per block: manifold kPa, cycle-averaged dyno N m
    std::vector<Segment> segs;
    double idleRpm = 0, cpu = 0, sim = 0;
    double holdErrMax = 0, holdErrSum = 0; long holdErrN = 0;  // |measured - commanded| rpm while held
    std::string info;
};

void fft(std::vector<std::complex<double>> &a) {
    const size_t n = a.size();
    for (size_t i = 1, j = 0; i < n; ++i) {
        size_t bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) std::swap(a[i], a[j]);
    }
    for (size_t len = 2; len <= n; len <<= 1) {
        const double ang = -2 * M_PI / len;
        const std::complex<double> wl(std::cos(ang), std::sin(ang));
        for (size_t i = 0; i < n; i += len) {
            std::complex<double> w(1);
            for (size_t j = 0; j < len / 2; ++j) {
                const auto u = a[i + j], v = a[i + j + len / 2] * w;
                a[i + j] = u + v;
                a[i + j + len / 2] = u - v;
                w *= wl;
            }
        }
    }
}

bool render(const Variant &v, const std::string &enginesDir, const std::string &lib, Render &r) {
    char err[4096] = {0};
    const std::string script = enginesDir + "/" + v.script;
    des_engine *e = des_create(script.c_str(), lib.c_str(), err, sizeof(err));
    if (e == nullptr) {
        std::printf("  load failed: %s\n", err);
        return false;
    }
    des_info info;
    des_get_info(e, &info);
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s, %d cyl, %.2f L, redline %.0f, %.0f Hz physics", info.name,
                  info.cylinders, info.displacement_l, info.redline_rpm, info.simulation_hz);
    r.info = buf;
    des_set_intake_sound(e, &v.intake);

    std::vector<float> block(4096);
    auto run = [&](double seconds, bool record, auto &&control) {
        const int n = static_cast<int>(std::lround(seconds / kBlock));
        for (int i = 0; i < n; ++i) {
            const double a = (i + 0.5) / n;
            const double cmdRpm = control(a);
            const int samples = static_cast<int>(std::lround((r.pcm.size() + 0.0) + kBlock * kFs)) - static_cast<int>(r.pcm.size());
            des_render(e, block.data(), samples);
            if (record) {
                r.pcm.insert(r.pcm.end(), block.begin(), block.begin() + samples);
                r.rpmTrack.push_back(cmdRpm > 0 ? cmdRpm : des_get_rpm(e));
                if (cmdRpm > 0) {
                    const double err = std::abs(des_get_rpm(e) - cmdRpm);
                    r.holdErrMax = std::max(r.holdErrMax, err); r.holdErrSum += err; ++r.holdErrN;
                }
                r.mapTrack.push_back(des_get_manifold_pressure(e) / 1000.0);
                r.trqTrack.push_back(des_get_dyno_torque(e));
            }
        }
    };

    // warm-up: crank and settle at idle, free-running (not recorded)
    des_set_mode(e, DES_MODE_FREE);
    des_set_throttle(e, 0.0);
    run(2.5, false, [&](double) { return 0.0; });

    double t = 0;
    // 1) idle 2 s, free running
    double idleSum = 0; int idleN = 0;
    run(2.0, true, [&](double) { idleSum += des_get_rpm(e); ++idleN; return 0.0; });
    r.idleRpm = idleSum / std::max(1, idleN);
    r.segs.push_back({"idle", t, t + 2.0, r.idleRpm, r.idleRpm}); t += 2.0;

    // 2) full-throttle sweep idle -> 6000 under load (dyno holds a ramping speed)
    const double r0 = des_get_rpm(e);
    des_set_mode(e, DES_MODE_HOLD);
    des_set_target_rpm(e, r0);
    des_set_throttle(e, 1.0);
    run(6.0, true, [&](double a) {
        const double rpm = r0 + (6000.0 - r0) * a;
        des_set_target_rpm(e, rpm);
        return rpm;
    });
    r.segs.push_back({"wot_sweep", t, t + 6.0, r0, 6000}); t += 6.0;

    // 3) lift-off: 0 throttle, 6000 -> 5000 in 0.25 s, then overrun 5000 -> 2000 over 2 s
    des_set_throttle(e, 0.0);
    run(0.25, true, [&](double a) { const double rpm = 6000 - 1000 * a; des_set_target_rpm(e, rpm); return rpm; });
    r.segs.push_back({"lift", t, t + 0.25, 6000, 5000}); t += 0.25;
    run(2.0, true, [&](double a) { const double rpm = 5000 - 3000 * a; des_set_target_rpm(e, rpm); return rpm; });
    r.segs.push_back({"overrun", t, t + 2.0, 5000, 2000}); t += 2.0;

    // 4) 2000 -> 3500 at 30 %, then 3 s steady cruise
    des_set_throttle(e, 0.3);
    run(0.5, true, [&](double a) { const double rpm = 2000 + 1500 * a; des_set_target_rpm(e, rpm); return rpm; });
    r.segs.push_back({"to_cruise", t, t + 0.5, 2000, 3500}); t += 0.5;
    run(3.0, true, [&](double) { des_set_target_rpm(e, 3500); return 3500.0; });
    r.segs.push_back({"cruise", t, t + 3.0, 3500, 3500}); t += 3.0;

    r.cpu = des_get_cpu_time(e);
    r.sim = des_get_sim_time(e);
    des_destroy(e);
    return true;
}

bool writeWav(const std::string &path, const std::vector<float> &pcm, double gain) {
    FILE *f = std::fopen(path.c_str(), "wb");
    if (!f) return false;
    const uint32_t n = static_cast<uint32_t>(pcm.size()), dataBytes = n * 2;
    auto u32 = [&](uint32_t x) { std::fwrite(&x, 4, 1, f); };
    auto u16 = [&](uint16_t x) { std::fwrite(&x, 2, 1, f); };
    std::fwrite("RIFF", 1, 4, f); u32(36 + dataBytes); std::fwrite("WAVE", 1, 4, f);
    std::fwrite("fmt ", 1, 4, f); u32(16); u16(1); u16(1); u32(44100); u32(88200); u16(2); u16(16);
    std::fwrite("data", 1, 4, f); u32(dataBytes);
    for (float s : pcm) {
        const double x = std::clamp(s * gain, -1.0, 1.0);
        const int16_t q = static_cast<int16_t>(std::lround(x * 32767.0));
        std::fwrite(&q, 2, 1, f);
    }
    std::fclose(f);
    return true;
}

double db(double x) { return 20.0 * std::log10(std::max(x, 1e-12)); }

double rms(const std::vector<float> &p, size_t a, size_t b, double g) {
    double s = 0;
    for (size_t i = a; i < b && i < p.size(); ++i) s += double(p[i]) * p[i];
    return std::sqrt(s / std::max<size_t>(1, b - a)) * g;
}

// Firing-frequency check: in 186 ms windows, search f in [0.6, 1.6] x the expected firing
// frequency (rpm/60*2 for a 4-cylinder 4-stroke) for the best harmonic-sum, and count how often
// it lands within 8 % of the expected value.
void pitchCheck(const Render &r, const Segment &s, double &hitRate, double &medianRatio) {
    const int N = 8192, W = 8192;
    std::vector<double> ratios;
    int hits = 0, total = 0;
    for (double tc = s.t0 + 0.15; tc + 0.1 < s.t1; tc += 0.1) {
        const long c = static_cast<long>(tc * kFs);
        if (c - W / 2 < 0 || c + W / 2 > static_cast<long>(r.pcm.size())) continue;
        std::vector<std::complex<double>> x(N);
        for (int i = 0; i < W; ++i) {
            const double w = 0.5 - 0.5 * std::cos(2 * M_PI * i / (W - 1));
            x[i] = r.pcm[c - W / 2 + i] * w;
        }
        fft(x);
        std::vector<double> mag(N / 2);
        for (int i = 0; i < N / 2; ++i) mag[i] = std::abs(x[i]);
        const size_t blk = std::min(r.rpmTrack.size() - 1, static_cast<size_t>(tc / kBlock));
        const double rpm = r.rpmTrack[blk];
        const double fexp = rpm / 60.0 * 2.0;
        auto magAt = [&](double f) {
            const double bin = f * N / kFs;
            const int b = static_cast<int>(bin);
            if (b + 1 >= N / 2) return 0.0;
            return std::max(mag[b], mag[b + 1]);
        };
        double best = 0, bestF = 0;
        for (double f = 0.6 * fexp; f <= 1.6 * fexp; f += 0.25) {
            double sum = 0;
            for (int k = 1; k <= 6; ++k) sum += magAt(k * f) / std::sqrt(double(k));
            if (sum > best) { best = sum; bestF = f; }
        }
        const double ratio = bestF / fexp;
        ratios.push_back(ratio);
        ++total;
        if (std::abs(ratio - 1.0) < 0.08) ++hits;
    }
    hitRate = total ? double(hits) / total : 0;
    std::sort(ratios.begin(), ratios.end());
    medianRatio = ratios.empty() ? 0 : ratios[ratios.size() / 2];
}

} // namespace

int main(int argc, char **argv) {
    if (argc < 4) {
        std::printf("usage: des_render <engines dir> <engine-sim dir> <out dir> [variant substring]\n");
        return 2;
    }
    const std::string enginesDir = argv[1], lib = argv[2], outDir = argv[3];
    const std::string filter = argc > 4 ? argv[4] : "";

    // Induction layer presets (engine-sim voices only the exhaust; see des_intake_sound).
    const des_intake_sound airbox{0.2f, 140.0f, 3.0f, 700.0f, 0.15f};   // stock box + snorkel + paper filter
    const des_intake_sound cone{2.0f, 420.0f, 1.2f, 4000.0f, 0.6f};      // open cone filter
    const Variant variants[] = {
        {"outback_stock", "outback_stock.mr", airbox,
         "equal-length headers, stock airbox, muffled exhaust + cat"},
        {"outback_intake", "outback_intake.mr", cone,
         "equal-length headers, open cone intake, stock muffled exhaust"},
        {"outback_intake_nomuffler", "outback_intake_nomuffler.mr", cone,
         "equal-length headers, cone intake, muffler delete"},
        {"outback_uel_intake_nomuffler", "outback_uel_intake_nomuffler.mr", cone,
         "unequal-length headers (burble), cone intake, muffler delete"},
    };

    std::vector<Render> renders;
    std::vector<const Variant *> used;
    for (const Variant &v : variants) {
        if (!filter.empty() && std::string(v.name).find(filter) == std::string::npos) continue;
        std::printf("rendering %s (%s)\n", v.name, v.desc);
        Render r;
        if (!render(v, enginesDir, lib, r)) return 1;
        std::printf("  %s\n  cpu %.2f s for %.2f s simulated -> %.2fx real time (%.0f%% of one core)\n",
                    r.info.c_str(), r.cpu, r.sim, r.sim / r.cpu, 100.0 * r.cpu / r.sim);
        std::printf("  dyno hold tracking |rpm - target|: mean %.1f, max %.1f rpm\n", r.holdErrSum / std::max(1L, r.holdErrN), r.holdErrMax);
        std::printf("  free idle %.0f rpm; WOT sweep torque (dyno, cycle-averaged):", r.idleRpm);
        const Segment &sw = r.segs[1];
        for (double rpm = 2000; rpm <= 6000; rpm += 1000) {
            const double a = (rpm - sw.rpm0) / (sw.rpm1 - sw.rpm0);
            const size_t b = std::min(r.trqTrack.size() - 1, size_t((sw.t0 + a * (sw.t1 - sw.t0)) / kBlock));
            const double tq = r.trqTrack[b];
            std::printf(" %.0f:%.0fNm/%.0fhp", rpm, tq, tq * rpm * 2 * M_PI / 60 / 745.7);
        }
        std::printf("\n");
        renders.push_back(std::move(r));
        used.push_back(&v);
    }

    double peak = 0;
    for (const Render &r : renders)
        for (float s : r.pcm) peak = std::max(peak, double(std::abs(s)));
    const double gain = peak > 0 ? std::pow(10.0, -1.0 / 20.0) / peak : 1.0;
    std::printf("\ncommon normalisation gain %.3f (%.1f dB): loudest peak -> -1 dBFS\n", gain, db(gain));

    std::vector<float> all;
    for (size_t i = 0; i < renders.size(); ++i) {
        const Render &r = renders[i];
        const std::string path = outDir + "/" + used[i]->name + ".wav";
        if (!writeWav(path, r.pcm, gain)) { std::printf("can't write %s\n", path.c_str()); return 1; }
        double pk = 0; size_t clipped = 0;
        for (float s : r.pcm) { pk = std::max(pk, double(std::abs(s)) * gain); if (std::abs(s) * gain >= 0.999) ++clipped; }
        const double total = rms(r.pcm, 0, r.pcm.size(), gain);
        std::printf("\n%s  %.2f s  peak %.1f dBFS  rms %.1f dBFS  clipped samples %zu\n", path.c_str(),
                    r.pcm.size() / kFs, db(pk), db(total), clipped);
        for (const Segment &s : r.segs) {
            const double sr = rms(r.pcm, size_t(s.t0 * kFs), size_t(s.t1 * kFs), gain);
            double hit = 0, med = 0;
            if (s.t1 - s.t0 > 1.0) pitchCheck(r, s, hit, med);
            double map = 0; int nm = 0;
            for (size_t b = size_t(s.t0 / kBlock); b < size_t(s.t1 / kBlock) && b < r.mapTrack.size(); ++b, ++nm) map += r.mapTrack[b];
            std::printf("  %-9s %5.2f-%5.2f s  rpm %4.0f->%4.0f  MAP %5.1f kPa  rms %6.1f dBFS", s.name, s.t0, s.t1,
                        s.rpm0, s.rpm1, map / std::max(1, nm), db(sr));
            if (s.t1 - s.t0 > 1.0)
                std::printf("  firing-freq match %3.0f%% (median f/f_fire %.2f)", 100 * hit, med);
            std::printf("\n");
        }
        all.insert(all.end(), r.pcm.begin(), r.pcm.end());
        all.insert(all.end(), static_cast<size_t>(0.75 * kFs), 0.0f);
    }
    if (renders.size() > 1) {
        const std::string path = outDir + "/compare_all.wav";
        writeWav(path, all, gain);
        std::printf("\n%s: all variants back to back (0.75 s gaps)\n", path.c_str());
    }
    return 0;
}
