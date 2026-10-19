// tyre_test: single-tyre checks of the DriftLab-patched Chrono Pac02 tyre (see chrono-10.0.0-pac02-falloff.patch).
//
//   tyre_test <ChronoCars root> <car id> [AC car data dir]
//
// Part A evaluates the steady-state Magic Formula of the car's Tire_FRONT / Tire_REAR (.tir via the Pac02 JSON):
// Fy vs slip angle 0-60 deg at 2/3/4 kN, Fx vs slip ratio, combined slip (alpha 30 deg, kappa 0..1), camber -5/0/+5.
// Part B drives the real tyre on Chrono's single-wheel test rig (rigid ground, 10 m/s) to check the physical signs
// (camber thrust towards the side the wheel leans to, restoring cornering force) and that the transient-slip
// path settles on the steady-state curve. Every check prints PASS/FAIL with its target; exit code = failures.
//
// Sign conventions printed here: ISO, x forward, y left. Slip angle alpha > 0 means the wheel slides to the
// right, so the restoring Fy is > 0 (left). gamma_iso > 0 = wheel top leaning to the right.
//
// Targets (sources):
//  - peak mu_y at 3 kN within 4% of AC's DY0 + DY1*Fz (the car's AC data; AC is the cross-check), falling with
//    load (load sensitivity: every published car tyre, Pacejka TVD 3rd ed. ch. 4).
//  - peak slip angle 5-12 deg (street / semi-slick car tyres: Milliken RCVD ch. 2; AC FRICTION_LIMIT_ANGLE 12).
//  - cornering stiffness 0.15-0.45 /deg of Fz (passenger to performance radials, Pacejka TVD table 4.x, Gillespie).
//  - sliding level: Fy(30)/peak 0.80-0.97, Fy(60)/peak 0.70-0.92, no rise after the peak (dry asphalt sliding /
//    peak friction 0.7-0.9: locked-wheel vs peak braking data; AC FALLOFF_LEVEL 0.85-0.92 on road tyres).
//  - Fx: peak slip ratio 0.06-0.15, Kx/Fz 15-35, Fx(kappa=1)/peak 0.70-0.92 (Pacejka TVD; ABS literature).
//  - combined (alpha 30 deg): Fy falls monotonically with kappa, |F| never above 1.05 x the larger pure-slip peak
//    (friction circle), and for kappa >= 0.1 the force is within 12 deg of opposing the sliding velocity
//    (Pacejka TVD 4.3: a fully sliding tread acts against the slip velocity).
//  - camber: thrust towards the lean at zero slip, 0.005-0.03 Fz/deg (radial camber stiffness, TNO MF data);
//    peak grip with the tyre leaning into the force >= leaning out, gain at 5 deg 0-8% (Milliken RCVD 2.7).
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/solver/ChIterativeSolverVI.h"
#include "chrono/functions/ChFunctionConst.h"
#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/tire/Pac02Tire.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChTireTestRig.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace {
constexpr double kDeg = CH_DEG_TO_RAD;
int g_fail = 0;

void Check(bool ok, const char* what, double val, const char* target) {
    std::printf("  [%s] %-58s %8.3f  (target %s)\n", ok ? "PASS" : "FAIL", what, val, target);
    if (!ok) ++g_fail;
}

std::string ReadFile(const std::string& p) {
    std::ifstream f(p);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

std::string JsonStr(const std::string& doc, const std::string& key) {
    auto i = doc.find("\"" + key + "\"");
    i = doc.find('"', doc.find(':', i) + 1);
    return doc.substr(i + 1, doc.find('"', i + 1) - i - 1);
}

// Minimal AC ini reader: section -> key -> value.
std::map<std::string, std::map<std::string, std::string>> ReadIni(const std::string& path) {
    std::map<std::string, std::map<std::string, std::string>> ini;
    std::ifstream f(path);
    std::string line, sec;
    while (std::getline(f, line)) {
        line = line.substr(0, line.find(';'));
        while (!line.empty() && std::isspace((unsigned char)line.back())) line.pop_back();
        size_t s = line.find_first_not_of(" \t");
        if (s == std::string::npos) continue;
        line = line.substr(s);
        if (line[0] == '[') {
            sec = line.substr(1, line.find(']') - 1);
        } else if (line.find('=') != std::string::npos) {
            std::string k = line.substr(0, line.find('=')), v = line.substr(line.find('=') + 1);
            while (!k.empty() && std::isspace((unsigned char)k.back())) k.pop_back();
            ini[sec][k] = v;
        }
    }
    return ini;
}

// Exposes the steady-state MF of a Pac02Tire (protected in Chrono).
class ProbeTire : public Pac02Tire {
  public:
    explicit ProbeTire(const std::string& json) : Pac02Tire(json) { SetMFParams(); }
    // alpha: ISO slip angle (> 0: sliding right, restoring Fy > 0). gamma_iso > 0: top leaning right.
    void Eval(double kappa, double alpha, double Fz, double gamma_iso, double& fx, double& fy) {
        m_states.mu_scale = 1.0;
        m_states.Fz0_prime = m_par.FNOMIN * m_par.LFZO;
        m_states.dfz0 = (Fz - m_states.Fz0_prime) / m_states.Fz0_prime;
        m_states.Pi0_prime = m_par.IP_NOM * m_par.LIP;
        m_states.dpi = (m_par.IP - m_states.Pi0_prime) / m_states.Pi0_prime;
        m_states.gamma = -gamma_iso;   // internal convention, see ChPac02Tire::Synchronize
        double Fx, Fy, Mz;
        CalcFxyMz(Fx, Fy, Mz, kappa, alpha, Fz, -gamma_iso);
        fx = Fx;
        fy = -Fy;   // as ChPac02Tire::Advance reports it (ISO)
    }
    double SigmaA(double Fz) { return CalcSigmaA(Fz); }
    double SigmaK(double Fz) {
        m_states.Fz0_prime = m_par.FNOMIN * m_par.LFZO;
        m_states.dfz0 = (Fz - m_states.Fz0_prime) / m_states.Fz0_prime;
        return CalcSigmaK(Fz);
    }
};

struct Ac {
    bool ok = false;
    double dy0 = 0, dy1 = 0, dx0 = 0, dx1 = 0;
};

void PartA(ProbeTire& t, const char* name, const Ac& ac) {
    std::printf("\n=== %s: steady-state curves ===\n", name);
    const double loads[3] = {2000, 3000, 4000};
    // Fy vs alpha.
    std::printf("Fy/Fz vs slip angle (gamma 0, kappa 0)\n  alpha:");
    const int angles[] = {0, 2, 4, 6, 8, 10, 12, 15, 20, 25, 30, 40, 50, 60};
    for (int a : angles) std::printf(" %5d", a);
    std::printf("\n");
    double peak_mu[3], peak_a[3], f30[3], f60[3];
    bool mono[3];
    for (int l = 0; l < 3; ++l) {
        const double Fz = loads[l];
        std::printf("  %2.0fkN:", Fz / 1000);
        for (int a : angles) {
            double fx, fy;
            t.Eval(0, a * kDeg, Fz, 0, fx, fy);
            std::printf(" %5.3f", fy / Fz);
        }
        std::printf("\n");
        peak_mu[l] = 0;
        peak_a[l] = 0;
        mono[l] = true;
        double prev = 0;
        bool past = false;
        for (double a = 0.1; a <= 60.0; a += 0.1) {
            double fx, fy;
            t.Eval(0, a * kDeg, Fz, 0, fx, fy);
            if (fy > peak_mu[l] * Fz) { peak_mu[l] = fy / Fz; peak_a[l] = a; }
            if (past && fy > prev + 1e-6) mono[l] = false;
            if (a > peak_a[l] + 0.5) past = true;
            prev = fy;
        }
        double fx;
        t.Eval(0, 30 * kDeg, Fz, 0, fx, f30[l]);
        t.Eval(0, 60 * kDeg, Fz, 0, fx, f60[l]);
        f30[l] /= peak_mu[l] * Fz;
        f60[l] /= peak_mu[l] * Fz;
        std::printf("        peak mu_y %.3f at %.1f deg, Fy(30)/peak %.3f, Fy(60)/peak %.3f\n", peak_mu[l], peak_a[l],
                    f30[l], f60[l]);
    }
    double fx, fy1;
    t.Eval(0, 0.5 * kDeg, 3000, 0, fx, fy1);
    const double cs = fy1 / 3000 / 0.5;   // per deg
    if (ac.ok) {
        const double ac3 = ac.dy0 + ac.dy1 * 3.0;
        Check(std::fabs(peak_mu[1] / ac3 - 1) < 0.04, "peak mu_y @3kN / AC (DY0+DY1*Fz)", peak_mu[1] / ac3, "0.96-1.04");
    }
    Check(peak_mu[0] > peak_mu[1] && peak_mu[1] > peak_mu[2], "load sensitivity mu(2kN)-mu(4kN)", peak_mu[0] - peak_mu[2],
          "> 0");
    Check(peak_a[1] >= 5 && peak_a[1] <= 12, "peak slip angle @3kN [deg]", peak_a[1], "5-12");
    Check(cs >= 0.15 && cs <= 0.45, "cornering stiffness @3kN [Fz/deg]", cs, "0.15-0.45");
    Check(f30[1] >= 0.80 && f30[1] <= 0.97, "Fy(30 deg)/peak @3kN", f30[1], "0.80-0.97");
    Check(f60[1] >= 0.70 && f60[1] <= 0.92, "Fy(60 deg)/peak @3kN", f60[1], "0.70-0.92");
    Check(mono[0] && mono[1] && mono[2], "Fy falls monotonically past the peak (2/3/4 kN)", 1.0 * (mono[0] + mono[1] + mono[2]),
          "3");

    // Fx vs kappa.
    std::printf("Fx/Fz vs slip ratio (3 kN)\n  kappa:");
    const double kap[] = {0, 0.02, 0.05, 0.08, 0.1, 0.12, 0.15, 0.2, 0.3, 0.5, 0.7, 1.0};
    for (double k : kap) std::printf(" %5.2f", k);
    std::printf("\n   Fx/Fz:");
    for (double k : kap) {
        double fx, fy;
        t.Eval(k, 0, 3000, 0, fx, fy);
        std::printf(" %5.3f", fx / 3000);
    }
    std::printf("\n");
    double pk = 0, pkk = 0;
    for (double k = 0.001; k <= 1.0; k += 0.001) {
        double fx, fy;
        t.Eval(k, 0, 3000, 0, fx, fy);
        if (fx > pk) { pk = fx; pkk = k; }
    }
    double fxs, fxl, fy;
    t.Eval(0.002, 0, 3000, 0, fxs, fy);
    t.Eval(1.0, 0, 3000, 0, fxl, fy);
    if (ac.ok) {
        const double ac3 = ac.dx0 + ac.dx1 * 3.0;
        Check(std::fabs(pk / 3000 / ac3 - 1) < 0.05, "peak mu_x @3kN / AC (DX0+DX1*Fz)", pk / 3000 / ac3, "0.95-1.05");
    }
    Check(pkk >= 0.06 && pkk <= 0.15, "peak slip ratio", pkk, "0.06-0.15");
    Check(fxs / 3000 / 0.002 >= 15 && fxs / 3000 / 0.002 <= 35, "Kx/Fz [1/unit slip]", fxs / 3000 / 0.002, "15-35");
    Check(fxl / pk >= 0.70 && fxl / pk <= 0.92, "Fx(kappa=1)/peak", fxl / pk, "0.70-0.92");

    // Combined slip.
    std::printf("Combined: alpha 30 deg, 3 kN, kappa 0..1 (drive)\n");
    std::printf("  kappa   Fx/Fz   Fy/Fz  |F|/Fz  force dir  slip dir (deg from +x, opposing)\n");
    bool fy_mono = true, circle = true, aligned = true;
    double prev_fy = 1e9, fy0 = 0, fy1k = 0, worst_dir = 0;
    double mu_max = std::max(peak_mu[1], pk / 3000);
    for (double k = 0; k <= 1.0001; k += 0.05) {
        double fx, fy;
        t.Eval(k, 30 * kDeg, 3000, 0, fx, fy);
        const double fd = std::atan2(fy, fx) / kDeg;
        const double sd = std::atan2(std::tan(30 * kDeg), k) / kDeg;   // = -(sliding velocity) direction
        const double F = std::hypot(fx, fy) / 3000;
        if (std::fmod(k + 1e-9, 0.1) < 0.01)
            std::printf("  %4.2f  %+6.3f  %+6.3f  %6.3f   %6.1f     %6.1f\n", k, fx / 3000, fy / 3000, F, fd, sd);
        if (fy > prev_fy + 1e-6) fy_mono = false;
        if (F > 1.05 * mu_max) circle = false;
        if (k >= 0.1 - 1e-9) worst_dir = std::max(worst_dir, std::fabs(fd - sd));
        if (k == 0) fy0 = fy;
        fy1k = fy;
        prev_fy = fy;
    }
    aligned = worst_dir <= 12;
    Check(fy_mono, "Fy(alpha 30) falls monotonically with kappa", fy_mono, "1");
    Check(fy1k / fy0 < 0.6, "Fy(alpha 30, kappa 1) / Fy(alpha 30, kappa 0)", fy1k / fy0, "< 0.6");
    Check(circle, "|F| <= 1.05 x max pure-slip peak", circle, "1");
    Check(aligned, "max angle force vs sliding direction, kappa>=0.1 [deg]", worst_dir, "<= 12");

    // Camber.
    std::printf("Camber (3 kN): Fy/Fz vs alpha for gamma_iso -5 / 0 / +5 deg (+ = top leaning right)\n");
    const int ca[] = {-10, -5, -2, 0, 2, 5, 10, 30};
    std::printf("  alpha:");
    for (int a : ca) std::printf(" %6d", a);
    std::printf("\n");
    for (int g : {-5, 0, 5}) {
        std::printf("  g %+2d: ", g);
        for (int a : ca) {
            double fx, fy;
            t.Eval(0, a * kDeg, 3000, g * kDeg, fx, fy);
            std::printf(" %+6.3f", fy / 3000);
        }
        std::printf("\n");
    }
    // Leaning right (gamma_iso > 0) helps force to the right (alpha < 0).
    auto peak_right = [&](double g) {
        double m = 0;
        for (double a = 0.1; a <= 30; a += 0.1) {
            double fx, fy;
            t.Eval(0, -a * kDeg, 3000, g * kDeg, fx, fy);
            m = std::max(m, -fy / 3000);
        }
        return m;
    };
    double fx0, thrust;
    t.Eval(0, 0, 3000, 1 * kDeg, fx0, thrust);   // 1 deg lean right
    const double p0 = peak_right(0), pin = peak_right(5), pout = peak_right(-5);
    std::printf("  peak mu (force to the right): lean 0 %.3f, lean into force 5 deg %.3f, lean away 5 deg %.3f\n", p0,
                pin, pout);
    Check(thrust < 0, "camber thrust points towards the lean (Fy at alpha 0, lean right)", thrust / 3000, "< 0");
    Check(-thrust / 3000 >= 0.005 && -thrust / 3000 <= 0.03, "camber thrust [Fz/deg]", -thrust / 3000, "0.005-0.03");
    Check(pin >= pout, "peak: leaning into the force >= leaning away", pin - pout, ">= 0");
    Check(pin / p0 - 1 >= 0 && pin / p0 - 1 <= 0.08, "peak gain leaning 5 deg into the force", pin / p0 - 1, "0-0.08");
    std::printf("Relaxation lengths at 2/3/4 kN (zero slip): sigma_alpha %.2f/%.2f/%.2f m, sigma_kappa %.2f/%.2f/%.2f m\n",
                t.SigmaA(2000), t.SigmaA(3000), t.SigmaA(4000), t.SigmaK(2000), t.SigmaK(3000), t.SigmaK(4000));
    Check(t.SigmaA(3000) >= 0.3 && t.SigmaA(3000) <= 0.6, "sigma_alpha @3kN [m] (car tyres 0.3-0.6)", t.SigmaA(3000),
          "0.3-0.6");
}

struct RigResult {
    double fy = 0, fx = 0, fz = 0, lean_z = 0, vlat = 0;
};

// Runs the tyre on Chrono's test rig at 10 m/s with a fixed camber (rig convention) and slip angle.
RigResult RunRig(const std::string& tire_json, const std::string& wheel_json, double camber, double slip) {
    ChSystemNSC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    std::static_pointer_cast<ChIterativeSolverVI>(sys.GetSolver())->SetMaxIterations(100);
    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    auto wheel = ReadWheelJSON(GetVehicleDataFile(wheel_json));
    auto tire = ReadTireJSON(GetVehicleDataFile(tire_json));
    const double step = 5e-4;
    tire->SetStepsize(step);
    ChTireTestRig rig(wheel, tire, &sys);
    rig.SetGravitationalAcceleration(9.81);
    rig.SetNormalLoad(3000);
    rig.SetCamberAngle(camber);
    rig.SetTireStepsize(step);
    rig.SetTireCollisionType(ChTire::CollisionType::SINGLE_POINT);
    rig.SetTireVisualizationType(VisualizationType::NONE);
    ChTireTestRig::TerrainPatchSize size;
    size.length = 80;
    size.width = 4;
    size.depth = 0.1;
    ChTireTestRig::TerrainParamsRigid params;
    params.friction = 0.8f;
    params.restitution = 0;
    params.Young_modulus = 2e7f;
    rig.SetTerrainRigid(size, params);
    rig.SetConstantLongitudinalSlip(0.0, 10.0);
    rig.SetSlipAngleFunction(chrono_types::make_shared<ChFunctionConst>(slip));
    rig.SetTimeDelay(0.2);
    rig.Initialize(ChTireTestRig::Mode::TEST, 0.5);
    RigResult r;
    int n = 0;
    while (sys.GetChTime() < 3.0) {
        rig.Advance(step);
        if (sys.GetChTime() > 2.5 && rig.OutputEnabled()) {
            auto f = rig.ReportTireForce().force;
            r.fx += f.x();
            r.fy += f.y();
            r.fz += f.z();
            auto st = wheel->GetState();
            r.lean_z = st.rot.GetAxisY().z();
            r.vlat = Vdot(st.lin_vel, st.rot.GetAxisY());
            ++n;
        }
    }
    if (n) { r.fx /= n; r.fy /= n; r.fz /= n; }
    return r;
}

void PartB(const std::string& tire_json, const std::string& wheel_json, ProbeTire& probe) {
    std::printf("\n=== %s on the test rig (10 m/s, 3 kN) ===\n", tire_json.c_str());
    for (double cam : {-5.0, 5.0}) {
        RigResult r = RunRig(tire_json, wheel_json, cam * kDeg, 0.0);
        const double gamma_iso = std::asin(r.lean_z) / kDeg;
        std::printf("  rig camber %+.0f deg: gamma_iso %+.2f deg (%s), Fy %+7.1f N, Fz %6.1f N\n", cam, gamma_iso,
                    gamma_iso > 0 ? "top leaning right" : "top leaning left", r.fy, r.fz);
        Check(r.fy * gamma_iso < 0, "rig: camber thrust towards the lean", r.fy / std::max(r.fz, 1.0), "sign");
    }
    RigResult r = RunRig(tire_json, wheel_json, 0.0, 5 * kDeg);
    std::printf("  rig slip 5 deg: Fy %+7.1f N, Fz %6.1f N, wheel lateral velocity %+.2f m/s\n", r.fy, r.fz, r.vlat);
    Check(r.fy * r.vlat < 0, "rig: cornering force opposes the lateral sliding", r.fy * r.vlat, "< 0");
    double fx, fy;
    probe.Eval(0, 5 * kDeg, r.fz, 0, fx, fy);
    Check(std::fabs(std::fabs(r.fy) / std::fabs(fy) - 1) < 0.05, "rig |Fy| / steady-state MF |Fy| at 5 deg",
          std::fabs(r.fy) / std::fabs(fy), "0.95-1.05");
}
}  // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::printf("usage: tyre_test <ChronoCars root> <car id> [AC car data dir] [--no-rig]\n");
        return 2;
    }
    const std::string root = argv[1], id = argv[2];
    bool rig = true;
    std::string ac_dir;
    for (int i = 3; i < argc; ++i) {
        if (std::string(argv[i]) == "--no-rig") rig = false;
        else ac_dir = argv[i];
    }
    SetVehicleDataPath(root + "/");
    const std::string car = ReadFile(root + "/" + id + "/car.json");
    std::map<std::string, std::map<std::string, std::string>> tyres;
    std::string idx;
    if (!ac_dir.empty()) {
        tyres = ReadIni(ac_dir + "/tyres.ini");
        idx = tyres["COMPOUND_DEFAULT"]["INDEX"];
    }
    for (const char* axle : {"FRONT", "REAR"}) {
        const std::string tj = JsonStr(car, std::string(axle) == "FRONT" ? "tire_front" : "tire_rear");
        ProbeTire probe(GetVehicleDataFile(tj));
        Ac ac;
        if (!tyres.empty()) {
            std::string sec = (idx.empty() || std::atoi(idx.c_str()) == 0) ? axle : std::string(axle) + "_" + idx;
            auto& s = tyres[sec];
            ac.ok = s.count("DY0") > 0;
            ac.dy0 = std::atof(s["DY0"].c_str());
            ac.dy1 = std::atof(s["DY1"].c_str());
            ac.dx0 = std::atof(s["DX0"].c_str());
            ac.dx1 = std::atof(s["DX1"].c_str());
        }
        PartA(probe, axle, ac);
        if (rig) PartB(tj, id + "/Wheel_" + axle + ".json", probe);
    }
    std::printf("\n%d check(s) failed\n", g_fail);
    return g_fail;
}
