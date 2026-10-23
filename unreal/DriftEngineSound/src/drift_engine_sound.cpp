// drift_engine_sound: engine-sim (MIT, Copyright 2022 AngeTheGreat / Ange Yaghi,
// https://github.com/ange-yaghi/engine-sim) as a synchronous, headless sound library.
//
// What is reused from engine-sim unchanged: the script compiler (Piranha .mr), the engine
// simulation (PistonEngineSimulator: rigid bodies + gas system), its dynamometer and starter
// constraints, and the synthesizer's filters (jitter, DC, derivative, air noise, impulse-response
// convolution, anti-aliasing, optional leveler).
//
// What is different from the engine-sim app:
//  * no audio thread: each advance simulates N physics steps and renders the produced input
//    block immediately (the per-sample render mirrors Synthesizer::renderAudio(int)), output is
//    float without the automatic leveler (so relative loudness survives) unless enabled;
//  * the simulator subclass replicates PistonEngineSimulator::writeToSynthesizer() and adds one
//    extra input channel: intake plenum pressure / flow (for the optional induction layer);
//  * impulse-response WAVs are read with a tiny RIFF reader instead of delta-studio's.
#include "../include/drift_engine_sound.h"

#include "piston_engine_simulator.h"
#include "engine.h"
#include "vehicle.h"
#include "transmission.h"
#include "exhaust_system.h"
#include "impulse_response.h"
#include "ignition_module.h"
#include "intake.h"
#include "units.h"
#include "../scripting/include/compiler.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <fstream>
#include <mutex>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace {

std::mutex g_compileMutex;

// ------------------------------------------------------------------ small DSP helpers
struct Biquad {
    double b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0, z1 = 0, z2 = 0;
    void lowpass(double fc, double q, double fs) {
        const double w = 2 * M_PI * std::min(fc, 0.45 * fs) / fs, c = std::cos(w), al = std::sin(w) / (2 * q);
        const double a0 = 1 + al;
        b0 = (1 - c) / 2 / a0; b1 = (1 - c) / a0; b2 = b0; a1 = -2 * c / a0; a2 = (1 - al) / a0;
    }
    void bandpass(double fc, double q, double fs) {  // constant 0 dB peak gain
        const double w = 2 * M_PI * std::min(fc, 0.45 * fs) / fs, c = std::cos(w), al = std::sin(w) / (2 * q);
        const double a0 = 1 + al;
        b0 = al / a0; b1 = 0; b2 = -al / a0; a1 = -2 * c / a0; a2 = (1 - al) / a0;
    }
    void highpass(double fc, double q, double fs) {
        const double w = 2 * M_PI * fc / fs, c = std::cos(w), al = std::sin(w) / (2 * q);
        const double a0 = 1 + al;
        b0 = (1 + c) / 2 / a0; b1 = -(1 + c) / a0; b2 = b0; a1 = -2 * c / a0; a2 = (1 - al) / a0;
    }
    inline double f(double x) {
        const double y = b0 * x + z1;
        z1 = b1 * x - a1 * y + z2;
        z2 = b2 * x - a2 * y;
        return y;
    }
};

bool readWav16(const std::string &path, std::vector<int16_t> &out) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    char riff[12];
    if (!f.read(riff, 12) || std::memcmp(riff, "RIFF", 4) != 0 || std::memcmp(riff + 8, "WAVE", 4) != 0) return false;
    int channels = 1, bits = 16;
    while (f) {
        char id[4]; uint32_t size = 0;
        if (!f.read(id, 4) || !f.read(reinterpret_cast<char *>(&size), 4)) break;
        if (std::memcmp(id, "fmt ", 4) == 0) {
            std::vector<char> fmt(size);
            f.read(fmt.data(), size);
            if (size >= 16) {
                uint16_t ch, bps;
                std::memcpy(&ch, fmt.data() + 2, 2);
                std::memcpy(&bps, fmt.data() + 14, 2);
                channels = ch; bits = bps;
            }
        } else if (std::memcmp(id, "data", 4) == 0) {
            if (bits != 16 || channels < 1) return false;
            std::vector<int16_t> raw(size / 2);
            f.read(reinterpret_cast<char *>(raw.data()), raw.size() * 2);
            out.clear();
            for (size_t i = 0; i + channels <= raw.size(); i += channels) out.push_back(raw[i]);  // first channel
            return true;
        } else {
            f.seekg(size + (size & 1), std::ios::cur);
        }
    }
    return false;
}

// ------------------------------------------------------------------ simulator subclass
class DesSimulator : public PistonEngineSimulator {
public:
    // Re-create the synthesizer with one extra (intake) input channel.
    void initializeSynth() {
        synthesizer().destroy();
        Synthesizer::Parameters p;
        p.audioBufferSize = 44100;
        p.audioSampleRate = 44100;
        p.inputBufferSize = 44100;
        p.inputChannelCount = m_engine->getExhaustSystemCount() + 1;
        p.inputSampleRate = static_cast<float>(getSimulationFrequency());
        synthesizer().initialize(p);
        synthesizer().m_latency = 0;
        m_staging.assign(p.inputChannelCount, 0.0);
    }


protected:
    // Same as PistonEngineSimulator::writeToSynthesizer (exhaust pulses delayed by the pipe
    // length, scaled by 1/length^2), plus the intake channel.
    void writeToSynthesizer() override {
        const int exhaustCount = m_engine->getExhaustSystemCount();
        for (double &v : m_staging) v = 0;

        const double attenuation = std::min(std::abs(filteredEngineSpeed()), 40.0) / 40.0;
        const double attenuation_3 = attenuation * attenuation * attenuation;

        const int cylinderCount = m_engine->getCylinderCount();
        for (int i = 0; i < cylinderCount; ++i) {
            Piston *piston = m_engine->getPiston(i);
            CylinderBank *bank = piston->getCylinderBank();
            CylinderHead *head = m_engine->getHead(bank->getIndex());
            ExhaustSystem *exhaust = head->getExhaustSystem(piston->getCylinderIndex());
            CombustionChamber *chamber = m_engine->getChamber(i);

            const double exhaustLength =
                head->getHeaderPrimaryLength(piston->getCylinderIndex()) + exhaust->getLength();

            const double exhaustFlow =
                attenuation_3 * 1600 * (
                    1.0 * (chamber->m_exhaustRunnerAndPrimary.pressure() - units::pressure(1.0, units::atm))
                    + 0.1 * chamber->m_exhaustRunnerAndPrimary.dynamicPressure(1.0, 0.0)
                    + 0.1 * chamber->m_exhaustRunnerAndPrimary.dynamicPressure(-1.0, 0.0));

            const double delayedExhaustPulse = m_delayFilters[i].fast_f(exhaustFlow);

            m_staging[exhaust->getIndex()] +=
                head->getSoundAttenuation(piston->getCylinderIndex())
                * (exhaust->getAudioVolume() * delayedExhaustPulse / cylinderCount)
                * (1 / (exhaustLength * exhaustLength));
        }

        // Intake channel: air inflow rate through the intake mouth (mol/s, pulsates with each
        // intake stroke). The induction sound radiated from the open end follows its derivative.
        double inflow = 0.0;
        for (int i = 0; i < m_engine->getIntakeCount(); ++i) inflow += m_engine->getIntake(i)->m_flow;
        m_staging[exhaustCount] = attenuation_3 * inflow * getFluidSimulationSteps() / getTimestep();

        synthesizer().writeInput(m_staging.data());
    }

private:
    std::vector<double> m_staging;
};

} // namespace

// ------------------------------------------------------------------ handle
struct des_engine {
    Engine *engine = nullptr;
    Vehicle *vehicle = nullptr;
    Transmission *transmission = nullptr;
    DesSimulator *sim = nullptr;

    int exhaustCount = 0;
    double simHz = 10000;
    double stepFrac = 0.0;

    // controls (targets) and the values applied at the last step (ramped per step)
    double throttle = 0.0, throttleApplied = 0.0;
    double targetRpm = 800.0, rpmApplied = -1.0;
    int mode = DES_MODE_FREE;
    bool ignition = true, starter = false, autoStarter = true, leveler = false;
    double gain = 1.0;

    // intake layer
    des_intake_sound intake{0.0f, 160.0f, 2.0f, 900.0f, 0.3f};
    Biquad inHp, inRes, inLp1, inLp2, hissBp;
    LowPassFilter flowEnv;
    double inPrev = 0.0;
    static constexpr double kRefFlow = 4.0;      // mol/s: ~2.5 L flat-4 at WOT near 5500 rpm
    static constexpr double kPulseScale = 100.0;   // calibrated in des_render (gain 1 ~ stock exhaust level at WOT)
    static constexpr double kHissScale = 2.0e5;
    static constexpr double kPulseLimit = 3000.0;  // in d(inflow)/dt units, ~3x the WOT rms
    std::minstd_rand rng{12345};

    std::vector<float> fifo;
    size_t fifoRead = 0;

    double simTime = 0.0;
    double cpuTime = 0.0;

    void setupIntakeFilters() {
        const double fs = 44100.0;
        inHp.highpass(25.0, 0.7, fs);
        inRes.bandpass(std::max(20.0f, intake.resonance_hz), std::max(0.3f, intake.resonance_q), fs);
        inLp1.lowpass(std::max(50.0f, intake.lowpass_hz), 0.707, fs);
        inLp2.lowpass(std::max(50.0f, intake.lowpass_hz), 0.707, fs);
        hissBp.bandpass(std::max(200.0f, intake.lowpass_hz * 0.6f), 0.8, fs);
    }

    void destroyAll() {
        if (sim != nullptr) {
            sim->synthesizer().destroy();
            sim->releaseSimulation();
            delete sim;
            sim = nullptr;
        }
        if (engine != nullptr) { engine->destroy(); delete engine; engine = nullptr; }
        delete vehicle; vehicle = nullptr;
        delete transmission; transmission = nullptr;
    }

    // Render all input the synthesizer holds into the FIFO (mirrors Synthesizer::renderAudio).
    void renderPending() {
        Synthesizer &s = sim->synthesizer();
        const int n = static_cast<int>(s.m_inputChannels[0].data.size());
        if (n <= 0) return;
        for (int c = 0; c < s.m_inputChannelCount; ++c) s.m_inputChannels[c].data.read(n, s.m_inputChannels[c].transferBuffer);
        s.m_inputSamplesRead = n;

        const Synthesizer::AudioParameters &ap = s.m_audioParameters;
        const float airNoise = ap.airNoise, mix = ap.dF_F_mix, conv = ap.convolution;
        const float *inTf = s.m_inputChannels[exhaustCount].transferBuffer;

        fifo.reserve(fifo.size() + n);
        for (int i = 0; i < n; ++i) {
            float signal = 0.0f;
            for (int c = 0; c < exhaustCount; ++c) {
                Synthesizer::ProcessingFilters &fl = s.m_filters[c];
                const float jittered = fl.jitterFilter.fast_f(s.m_inputChannels[c].transferBuffer[i]);
                const float f_dc = fl.inputDcFilter.fast_f(jittered);
                const float fv = jittered - f_dc;
                const float f_p = fl.derivative.f(jittered);
                const float noise = 2.0f * (static_cast<float>(rand()) / RAND_MAX) - 1.0f;
                const float r = fl.airNoiseLowPass.fast_f(noise);
                const float r_mixed = airNoise * r + (1 - airNoise);
                float v_in = f_p * mix + fv * r_mixed * (1 - mix);
                if (std::fpclassify(v_in) == FP_SUBNORMAL) v_in = 0;
                const float ex = conv * fl.convolution.f(v_in) + (1 - conv) * v_in;
                signal += ex;
            }

            if (intake.gain > 0.0f) {
                // d(inflow)/dt -> resonance (box/snorkel) -> filter-element low-pass
                const double q = inTf[i];
                const double dq = inHp.f((q - inPrev) * 44100.0);
                inPrev = q;
                const double res = inRes.f(dq);
                // soft limit: tip-in steps in the inflow otherwise give a click ~10 dB over WOT level
                const double pulse = kPulseLimit * std::tanh(inLp2.f(inLp1.f(0.6 * res + 0.4 * dq)) / kPulseLimit);
                // turbulent flow hiss ~ (mean flow / reference flow)^2
                const double env = flowEnv.fast_f(static_cast<float>(std::abs(q))) / kRefFlow;
                const double wn = (static_cast<double>(rng()) / rng.max()) * 2.0 - 1.0;
                const double hiss = hissBp.f(wn) * env * env;
                signal += static_cast<float>(intake.gain * (kPulseScale * pulse + intake.hiss * kHissScale * hiss));
            }

            signal = s.m_antialiasing.fast_f(signal);
            float v = leveler ? s.m_levelingFilter.f(signal) * ap.volume : signal * ap.volume;
            fifo.push_back(static_cast<float>(v / 32768.0 * gain));
        }
    }

    // Simulate `steps` physics steps, ramping throttle and dyno speed across them.
    void simulate(int steps) {
        while (steps > 0) {
            const int k = std::min(steps, 1000);
            steps -= k;

            Synthesizer &s = sim->synthesizer();
            s.m_latency = 0;  // keep startFrame() from stretching/shrinking the step count
            sim->startFrame(k / simHz);

            const double thr0 = throttleApplied, thr1 = throttle;
            const double rpm0 = rpmApplied < 0 ? targetRpm : rpmApplied, rpm1 = targetRpm;
            IgnitionModule *im = engine->getIgnitionModule();
            im->m_enabled = ignition;
            sim->m_dyno.m_enabled = (mode != DES_MODE_FREE);
            sim->m_dyno.m_hold = (mode == DES_MODE_HOLD);

            int i = 0;
            for (;;) {
                const double a = (i + 1) / static_cast<double>(k);
                engine->setSpeedControl(thr0 + (thr1 - thr0) * a);
                sim->m_dyno.m_rotationSpeed = units::rpm(std::max(0.0, rpm0 + (rpm1 - rpm0) * a));
                const double rpm = engine->getRpm();
                bool st = starter;
                if (autoStarter && mode == DES_MODE_FREE && ignition) st = st || rpm < 300.0;
                if (st && rpm > units::toRpm(engine->getStarterSpeed()) * 0.95 && !starter) st = false;
                sim->m_starterMotor.m_enabled = st;
                if (!sim->simulateStep()) break;
                ++i;
            }
            throttleApplied = thr1;
            rpmApplied = rpm1;
            simTime += k / simHz;

            renderPending();
            sim->endFrame();  // drops the consumed input, resets the frame's intake flow average
        }
    }

    int advance(double dt) {
        const size_t before = fifo.size() - fifoRead;
        const auto t0 = std::chrono::steady_clock::now();
        stepFrac += dt * simHz;
        const int steps = static_cast<int>(std::floor(stepFrac));
        stepFrac -= steps;
        simulate(steps);
        cpuTime += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        return static_cast<int>(fifo.size() - fifoRead - before);
    }

    int read(float *out, int maxSamples) {
        const int n = std::min<int>(maxSamples, static_cast<int>(fifo.size() - fifoRead));
        if (n > 0 && out != nullptr) std::memcpy(out, fifo.data() + fifoRead, n * sizeof(float));
        fifoRead += n;
        if (fifoRead > 65536 || fifoRead == fifo.size()) {
            fifo.erase(fifo.begin(), fifo.begin() + fifoRead);
            fifoRead = 0;
        }
        return n;
    }
};

namespace {

void setErr(char *err, int len, const std::string &msg) {
    if (err == nullptr || len <= 0) return;
    std::snprintf(err, len, "%s", msg.c_str());
}

std::string formatErrors(const piranha::ErrorList *errors) {
    std::ostringstream os;
    for (int i = 0; i < errors->getErrorCount(); ++i) {
        const piranha::CompilationError *e = errors->getCompilationError(i);
        const piranha::ErrorCode_struct &code = e->getErrorCode();
        os << e->getCompilationUnit()->getPath().getStem() << "(" << e->getErrorLocation()->lineStart
           << "): error " << code.stage << code.code << ": " << code.info << "\n";
    }
    return os.str();
}

bool compileScript(const std::string &script, const std::string &lib, des_engine *h, std::string &err) {
    std::lock_guard<std::mutex> lock(g_compileMutex);

    es_script::Compiler::Output *out = es_script::Compiler::output();
    out->engine = nullptr;
    out->vehicle = nullptr;
    out->transmission = nullptr;

    es_script::LanguageRules rules;
    piranha::Compiler *compiler = new piranha::Compiler(&rules);
    compiler->setFileExtension(".mr");
    std::string base = lib;
    if (!base.empty() && base.back() != '/' && base.back() != '\\') base += "/";
    compiler->addSearchPath(base + "es/");
    compiler->addSearchPath(base);
    compiler->addSearchPath(base + "assets/");
    rules.initialize();

    bool ok = false;
    piranha::NodeProgram program;
    piranha::IrCompilationUnit *unit = compiler->compile(script);
    if (unit == nullptr) {
        err = "can't open script: " + script;
    } else if (compiler->getErrorList()->getErrorCount() != 0) {
        err = formatErrors(compiler->getErrorList());
    } else {
        unit->build(&program);
        program.initialize();
        if (!program.execute()) err = "script runtime error";
        ok = out->engine != nullptr;
        if (ok) {
            h->engine = out->engine;
            h->vehicle = out->vehicle;
            h->transmission = out->transmission;
        } else if (err.empty()) {
            err = "script did not set an engine (call main() / set_engine())";
        }
    }
    program.free();
    compiler->free();
    delete compiler;
    out->engine = nullptr;
    out->vehicle = nullptr;
    out->transmission = nullptr;
    return ok;
}

} // namespace

extern "C" {

DES_API des_engine *des_create(const char *script_path, const char *library_dir, char *err, int err_len) {
    if (script_path == nullptr) { setErr(err, err_len, "null script path"); return nullptr; }
    des_engine *h = nullptr;
    try {
        h = new des_engine;
        std::string msg;
        if (!compileScript(script_path, library_dir ? library_dir : "", h, msg)) {
            setErr(err, err_len, msg);
            h->destroyAll();
            delete h;
            return nullptr;
        }

        if (h->vehicle == nullptr) {
            Vehicle::Parameters vp;
            vp.mass = units::mass(1500, units::kg);
            vp.diffRatio = 3.9;
            vp.tireRadius = units::distance(10, units::inch);
            vp.dragCoefficient = 0.3;
            vp.crossSectionArea = units::distance(6.0, units::foot) * units::distance(6.0, units::foot);
            vp.rollingResistance = 200.0;
            h->vehicle = new Vehicle;
            h->vehicle->initialize(vp);
        }
        if (h->transmission == nullptr) {
            static const double ratios[] = {3.5, 2.0, 1.4, 1.0, 0.8};
            Transmission::Parameters tp;
            tp.GearCount = 5;
            tp.GearRatios = ratios;
            tp.MaxClutchTorque = units::torque(300.0, units::ft_lb);
            h->transmission = new Transmission;
            h->transmission->initialize(tp);
        }

        // Engine::createSimulator, with our subclass.
        DesSimulator *sim = new DesSimulator;
        Simulator::Parameters sp;
        sp.systemType = Simulator::SystemType::NsvOptimized;
        sim->initialize(sp);
        sim->loadSimulation(h->engine, h->vehicle, h->transmission);
        sim->setFluidSimulationSteps(8);
        h->sim = sim;

        h->engine->calculateDisplacement();
        h->simHz = h->engine->getSimulationFrequency();
        sim->setSimulationFrequency(static_cast<int>(h->simHz));
        sim->setTargetSynthesizerLatency(0.0);
        sim->initializeSynth();
        h->exhaustCount = h->engine->getExhaustSystemCount();

        Synthesizer::AudioParameters ap = sim->synthesizer().getAudioParameters();
        ap.inputSampleNoise = static_cast<float>(h->engine->getInitialJitter());
        ap.airNoise = static_cast<float>(h->engine->getInitialNoise());
        ap.dF_F_mix = static_cast<float>(h->engine->getInitialHighFrequencyGain());
        sim->synthesizer().setAudioParameters(ap);
        for (int c = 0; c < sim->synthesizer().m_inputChannelCount; ++c) {
            sim->synthesizer().m_filters[c].jitterFilter.setJitterScale(ap.inputSampleNoise);
            sim->synthesizer().m_filters[c].airNoiseLowPass.setCutoffFrequency(ap.airNoiseFrequencyCutoff, 44100.0f);
        }

        for (int i = 0; i < h->exhaustCount; ++i) {
            ImpulseResponse *ir = h->engine->getExhaustSystem(i)->getImpulseResponse();
            std::vector<int16_t> samples;
            if (ir == nullptr || !readWav16(ir->getFilename(), samples)) {
                setErr(err, err_len, "can't read impulse response: " + (ir ? ir->getFilename() : std::string("(none)")));
                h->destroyAll();
                delete h;
                return nullptr;
            }
            sim->synthesizer().initializeImpulseResponse(samples.data(), static_cast<unsigned>(samples.size()),
                                                         static_cast<float>(ir->getVolume()), i);
        }
        // Intake channel: pass-through convolution (1 tap) so the shared filter struct is valid.
        {
            const int16_t one = INT16_MAX;
            sim->synthesizer().initializeImpulseResponse(&one, 1, 1.0f, h->exhaustCount);
        }
        h->flowEnv.setCutoffFrequency(20.0);
        h->flowEnv.m_dt = 1.0 / 44100.0;
        h->setupIntakeFilters();
        h->engine->getIgnitionModule()->m_enabled = true;
        return h;
    } catch (...) {
        setErr(err, err_len, "exception while loading engine");
        if (h != nullptr) {
            try { h->destroyAll(); } catch (...) {}
            delete h;
        }
        return nullptr;
    }
}

DES_API void des_destroy(des_engine *e) {
    if (e == nullptr) return;
    try { e->destroyAll(); } catch (...) {}
    delete e;
}

DES_API void des_get_info(const des_engine *e, des_info *out) {
    if (e == nullptr || out == nullptr) return;
    std::memset(out, 0, sizeof(*out));
    out->cylinders = e->engine->getCylinderCount();
    out->exhaust_systems = e->exhaustCount;
    out->displacement_l = e->engine->getDisplacement() / units::volume(1.0, units::L);
    out->redline_rpm = units::toRpm(e->engine->getRedline());
    out->simulation_hz = e->simHz;
    std::snprintf(out->name, sizeof(out->name), "%s", e->engine->getName().c_str());
}

DES_API void des_set_throttle(des_engine *e, double t) { if (e) e->throttle = std::clamp(t, 0.0, 1.0); }
DES_API void des_set_mode(des_engine *e, int mode) { if (e && mode >= 0 && mode <= 2) e->mode = mode; }
DES_API void des_set_target_rpm(des_engine *e, double rpm) {
    if (e) e->targetRpm = std::clamp(rpm, 0.0, 30000.0);
}
DES_API void des_set_ignition(des_engine *e, int on) { if (e) e->ignition = on != 0; }
DES_API void des_set_starter(des_engine *e, int on) { if (e) e->starter = on != 0; }
DES_API void des_set_auto_starter(des_engine *e, int on) { if (e) e->autoStarter = on != 0; }
DES_API void des_set_gain(des_engine *e, double g) { if (e) e->gain = g; }
DES_API void des_set_leveler(des_engine *e, int on) { if (e) e->leveler = on != 0; }
DES_API void des_set_intake_sound(des_engine *e, const des_intake_sound *p) {
    if (e == nullptr || p == nullptr) return;
    e->intake = *p;
    e->setupIntakeFilters();
}

DES_API int des_advance(des_engine *e, double dt) {
    if (e == nullptr || !(dt >= 0.0)) return -1;
    try { return e->advance(std::min(dt, 1.0)); } catch (...) { return -1; }
}

DES_API int des_available(const des_engine *e) {
    return e ? static_cast<int>(e->fifo.size() - e->fifoRead) : 0;
}

DES_API int des_read(des_engine *e, float *out, int max_samples) {
    if (e == nullptr || max_samples <= 0) return 0;
    return e->read(out, max_samples);
}

DES_API int des_render(des_engine *e, float *out, int n) {
    if (e == nullptr || n < 0) return -1;
    try {
        int guard = 0;
        while (des_available(e) < n && guard++ < 1000) {
            const int need = n - des_available(e);
            e->advance((need + 8) / 44100.0);
        }
        const int got = e->read(out, n);
        if (got < n && out != nullptr) std::memset(out + got, 0, (n - got) * sizeof(float));
        return n;
    } catch (...) {
        return -1;
    }
}

DES_API double des_get_rpm(const des_engine *e) { return e ? e->engine->getRpm() : 0.0; }
DES_API double des_get_dyno_torque(const des_engine *e) {
    if (e == nullptr || !e->sim->m_dyno.m_enabled) return 0.0;
    return e->sim->getFilteredDynoTorque();  // averaged over the last 720 deg of crank angle
}
DES_API double des_get_manifold_pressure(const des_engine *e) { return e ? e->engine->getManifoldPressure() : 0.0; }
DES_API double des_get_sim_time(const des_engine *e) { return e ? e->simTime : 0.0; }
DES_API double des_get_cpu_time(const des_engine *e) { return e ? e->cpuTime : 0.0; }

} // extern "C"
