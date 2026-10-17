// ChronoLab: tries Project Chrono's vehicle model (multibody double wishbones,
// Pacejka 2002 tyres) on cars converted from Assetto Corsa by
// drift-sim/tools/ac_to_chrono.py, and scores it the same way as DriftCore.
//   chrono_lab <car root> <car id> perf
//   chrono_lab <car root> <car id> skidpad [speed_kmh=60]
//   chrono_lab <car root> <car id> replay <ac csv> [window_s=0.25]
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "chrono/physics/ChShaftBodyConstraint.h"
#include "chrono/physics/ChShaftsGearbox.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/solver/ChIterativeSolverVI.h"
#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace {
constexpr double kStep = 1e-3;
constexpr double kDeg = 57.29578;

struct Car {
    std::unique_ptr<WheeledVehicle> veh;
    std::unique_ptr<RigidTerrain> terrain;
    double lock = 0.9;          // road-wheel angle at full steering input [rad]
    double motor_inertia = 0;   // to find the engine shaft among the system's shafts
    std::shared_ptr<ChShaft> motorshaft;
    std::shared_ptr<ChSolver> step_solver;   // Barzilai-Borwein: handles clutch/brake friction limits
    std::shared_ptr<ChSolver> exact_solver;  // SPARSE_LU: exact joint assembly when resetting state
};

std::string ReadFile(const std::string& p) {
    std::ifstream f(p);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

double JsonNum(const std::string& doc, const std::string& key, double def) {
    auto i = doc.find("\"" + key + "\"");
    if (i == std::string::npos) return def;
    i = doc.find(':', i);
    return std::atof(doc.c_str() + i + 1);
}

std::string JsonStr(const std::string& doc, const std::string& key) {
    auto i = doc.find("\"" + key + "\"");
    i = doc.find('"', doc.find(':', i) + 1);
    return doc.substr(i + 1, doc.find('"', i + 1) - i - 1);
}

void Build(Car& c, const std::string& root, const std::string& id) {
    SetVehicleDataPath(root + "/");
    const std::string car = ReadFile(root + "/" + id + "/car.json");
    c.veh = std::make_unique<WheeledVehicle>(GetVehicleDataFile(JsonStr(car, "vehicle")), ChContactMethod::NSC);
    c.veh->Initialize(ChCoordsys<>(ChVector3d(0, 0, 0.05), QUNIT));
    c.veh->SetChassisVisualizationType(VisualizationType::NONE);
    auto engine = ReadEngineJSON(GetVehicleDataFile(JsonStr(car, "engine")));
    auto trans = ReadTransmissionJSON(GetVehicleDataFile(JsonStr(car, "transmission")));
    c.veh->InitializePowertrain(chrono_types::make_shared<ChPowertrainAssembly>(engine, trans));
    for (auto& axle : c.veh->GetAxles())
        for (auto& wheel : axle->GetWheels()) {
            const bool front = axle == c.veh->GetAxle(0);
            auto tire = ReadTireJSON(GetVehicleDataFile(JsonStr(car, front ? "tire_front" : "tire_rear")));
            c.veh->InitializeTire(tire, wheel, VisualizationType::NONE);
        }
    c.lock = JsonNum(car, "steer_lock", 0.9);
    c.veh->GetChassis()->SetAerodynamicDrag(JsonNum(car, "cda", 0.7), 1.0, 1.2);   // Cd x A given as CdA
    c.motor_inertia = JsonNum(ReadFile(GetVehicleDataFile(JsonStr(car, "transmission"))), "Motorshaft Inertia", 0.15);
    for (auto& s : c.veh->GetSystem()->GetShafts())
        if (std::fabs(s->GetInertia() - c.motor_inertia) < 1e-9) c.motorshaft = s;

    ChSystem* sys = c.veh->GetSystem();
    sys->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    // Clutches and brakes are friction-limited ("boxed") constraints: they need a VI solver. A direct
    // linear solver (SPARSE_LU) silently ignores the limits and locks them. Barzilai-Borwein is what
    // Chrono recommends for NSC vehicles.
    sys->SetSolverType(ChSolver::Type::SPARSE_LU);
    c.exact_solver = sys->GetSolver();
    sys->SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    {
        auto solver = std::static_pointer_cast<ChIterativeSolverVI>(sys->GetSolver());
        solver->SetMaxIterations(std::getenv("DL_ITERS") ? std::atoi(std::getenv("DL_ITERS")) : 100);
        solver->SetOmega(0.8);
        solver->SetSharpnessLambda(1.0);
    }
    c.step_solver = sys->GetSolver();
    sys->SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    c.terrain = std::make_unique<RigidTerrain>(sys);
    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
    mat->SetFriction(0.8f);
    c.terrain->AddPatch(mat, ChCoordsys<>(ChVector3d(0, 0, 0), QUNIT), 4000, 4000, 1, false, 1, false);
    c.terrain->Initialize();
}

void Step(Car& c, const DriverInputs& in, double t) {
    c.veh->Synchronize(t, in, *c.terrain);
    c.terrain->Synchronize(t);
    c.veh->Advance(kStep);
    c.terrain->Advance(kStep);
}

ChVector3d LocalVel(const Car& c) {
    auto b = c.veh->GetChassisBody();
    return b->GetRot().RotateBack(b->GetPosDt());
}

double YawRate(const Car& c) {
    auto b = c.veh->GetChassisBody();
    return b->GetRot().RotateBack(b->GetAngVelParent()).z();
}

double FrontWheelAngle(const Car& c) {
    // Average road-wheel steer of the front pair, relative to the chassis.
    double a = 0;
    for (auto side : {LEFT, RIGHT}) {
        auto st = c.veh->GetWheel(0, side)->GetState();
        // The spin axis (local Y) is unaffected by wheel rotation: Y = (-sin d, cos d, .) in the chassis frame.
        ChVector3d ax = c.veh->GetChassisBody()->GetRot().RotateBack(st.rot.GetAxisY());
        a += std::atan2(-ax.x(), ax.y()) / 2;
    }
    return a;
}

double Settle(Car& c) {
    // Measure the real road-wheel angle at full lock (rack geometry), then centre and settle.
    double t = 0;
    for (int i = 0; i < 800; ++i, t += kStep) Step(c, DriverInputs{1, 0, 1, 0}, t);
    const double left = FrontWheelAngle(c);
    for (int i = 0; i < 800; ++i, t += kStep) Step(c, DriverInputs{-1, 0, 1, 0}, t);
    const double right = -FrontWheelAngle(c);
    std::printf("full lock: %.1f deg left, %.1f deg right (AC lock %.1f)\n", left * kDeg, right * kDeg, c.lock * kDeg);
    c.lock = (left + right) / 2;
    for (int i = 0; i < 1500; ++i, t += kStep) Step(c, DriverInputs{0, 0, 1, 0}, t);
    return t;
}

int Diag(Car& c) {
    double t = Settle(c);
    auto ch = c.veh->GetChassisBody();
    std::printf("settled: chassis z %.3f\n", ch->GetPos().z());
    c.veh->GetTransmission()->SetGear(1);
    for (int i = 0; i < 6000; ++i, t += kStep) {
        DriverInputs in{0, i < 4000 ? 0.5 : 0.0, i < 4000 ? 0.0 : 0.6, 0};
        Step(c, in, t);
        if (i % 250 == 0) {
            auto e = c.veh->GetEngine();
            std::printf("t %4.2f v %+6.2f km/h gear %d motor %6.0f rpm  torque %6.1f  wRL %+6.2f wFL %+6.2f  z %.3f pos (%.2f %.2f)\n", t,
                        c.veh->GetSpeed() * 3.6, c.veh->GetTransmission()->GetCurrentGear(), e->GetMotorSpeed() * 60 / CH_2PI,
                        e->GetOutputMotorshaftTorque(), c.veh->GetSpindleOmega(1, LEFT), c.veh->GetSpindleOmega(0, LEFT),
                        ch->GetPos().z(), ch->GetPos().x(), ch->GetPos().y());
        }
    }
    return 0;
}

int ResetTest(Car& c) {
    double t = Settle(c);
    ChSystem* sys = c.veh->GetSystem();
    ChState x0(sys->GetNumCoordsPosLevel(), sys);
    ChStateDelta v0(sys->GetNumCoordsVelLevel(), sys);
    double T0 = 0;
    sys->StateGather(x0, v0, T0);
    for (int pass = 0; pass < 3; ++pass) {
        if (pass) sys->StateScatter(x0, v0, sys->GetChTime(), UpdateFlags::UPDATE_ALL_NO_VISUAL);
        if (pass == 2) c.veh->GetChassisBody()->SetPosDt(ChVector3d(0, 0, 0));
        std::printf("pass %d:", pass);
        for (int i = 0; i < 1000; ++i, t += kStep) {
            Step(c, DriverInputs{0, 0, 1, 0}, t);
            if (i % 200 == 0) std::printf(" z %.3f", c.veh->GetChassisBody()->GetPos().z());
        }
        std::printf("\n");
    }
    return 0;
}

int Perf(Car& c) {
    double t = Settle(c);
    c.veh->GetTransmission()->SetGear(1);
    const auto t0 = std::chrono::steady_clock::now();
    const int n = 20000;
    for (int i = 0; i < n; ++i, t += kStep) {
        DriverInputs in{0.3 * std::sin(t * 0.8), 0.6, 0, 0};
        Step(c, in, t);
    }
    const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf("simulated %.1f s at %.0f Hz in %.2f s wall -> %.1fx real time (%.1f us/step), speed %.0f km/h\n",
                n * kStep, 1 / kStep, wall, n * kStep / wall, wall / n * 1e6, c.veh->GetSpeed() * 3.6);
    return 0;
}

// Steady-state understeer: hold speed, ramp steering slowly, fit (steer - L/R) vs lateral g.
int Skidpad(Car& c, double kmh) {
    double t = Settle(c);
    auto tr = c.veh->GetTransmission();
    tr->SetGear(2);
    const double L = c.veh->GetWheelbase();
    const double v_set = kmh / 3.6;
    double integ = 0;
    std::vector<std::pair<double, double>> pts;
    std::printf("  t     speed  steer(deg)  lat g   drift(deg)  gear\n");
    for (int i = 0; i < 60000; ++i, t += kStep) {
        const double v = LocalVel(c).x();
        const double err = v_set - v;
        integ = std::clamp(integ + err * kStep, -5.0, 5.0);
        double thr = std::clamp(0.3 * err + 0.08 * integ + 0.1, 0.0, 1.0);
        double brk = err < -1.0 ? std::clamp(-0.1 * err, 0.0, 0.5) : 0.0;
        const double ramp = t < 8 ? 0 : (t - 8) / 50.0;   // full lock after 50 s
        DriverInputs in{std::min(ramp, 1.0) * 0.25, thr, brk, 0};
        Step(c, in, t);
        if (i % 50 == 0 && t > 8 && v > v_set * 0.9) {
            const double r = YawRate(c);
            const double ay = v * r / 9.81;
            pts.emplace_back(ay, (FrontWheelAngle(c) - L * r / v) * kDeg);
            const double drift = std::atan2(LocalVel(c).y(), v) * kDeg;
            if (i % 2000 == 0)
                std::printf("%5.1f  %5.1f   %6.2f    %5.3f   %+5.2f      %d\n", t, v * 3.6, FrontWheelAngle(c) * kDeg, ay,
                            drift, tr->GetCurrentGear());
            if (ay > 0.95 || std::fabs(drift) > 8) break;
        }
    }
    // least squares on 0.1 .. 0.6 g
    double n = 0, sx = 0, sy = 0, sxx = 0, sxy = 0, maxg = 0;
    for (auto [x, y] : pts) {
        maxg = std::max(maxg, x);
        if (x < 0.1 || x > 0.6) continue;
        n++, sx += x, sy += y, sxx += x * x, sxy += x * y;
    }
    if (n > 3)
        std::printf("understeer gradient %.2f deg/g (0.1-0.6 g, %d pts), max steady lateral %.2f g\n",
                    (n * sxy - sx * sy) / (n * sxx - sx * sx), (int)n, maxg);
    return 0;
}

struct Row {
    std::map<std::string, double> v;
    double operator[](const std::string& k) const {
        auto it = v.find(k);
        return it == v.end() ? 0.0 : it->second;
    }
};

std::vector<Row> LoadCsv(const char* path) {
    std::ifstream f(path);
    std::string line;
    std::vector<std::string> cols;
    std::vector<Row> rows;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::stringstream ss(line);
        std::string cell;
        if (cols.empty()) {
            while (std::getline(ss, cell, ',')) cols.push_back(cell);
            continue;
        }
        Row r;
        size_t i = 0;
        while (std::getline(ss, cell, ',') && i < cols.size()) r.v[cols[i++]] = std::atof(cell.c_str());
        rows.push_back(r);
    }
    return rows;
}

double AcDrift(const Row& r) { return r["speed_kmh"] > 3 ? std::atan2(r["vel_x"], r["vel_z"]) * kDeg : 0.0; }

// Puts the settled car into AC's recorded motion: every body moves rigidly with the chassis,
// wheels spin at AC's rates, the engine at AC's rpm; driveline shafts follow by velocity assembly.
void SetMotion(Car& c, const Row& a, double steer) {
    ChSystem* sys = c.veh->GetSystem();
    sys->SetSolver(c.exact_solver);
    struct Restore {
        Car& c;
        ~Restore() { c.veh->GetSystem()->SetSolver(c.step_solver); }
    } restore{c};
    // Point the steering actuator at this window's input and move the rack/uprights there first:
    // otherwise the velocity assembly tries to close the gap to the old target in one step.
    c.veh->Synchronize(sys->GetChTime(), DriverInputs{steer, 0, 0, 0}, *c.terrain);
    sys->DoAssembly(AssemblyAnalysis::Level::POSITION);
    auto ch = c.veh->GetChassisBody();
    const ChQuaternion<> q = ch->GetRot();
    const ChVector3d v = q.Rotate(ChVector3d(a["vel_z"], a["vel_x"], a["vel_y"]));
    const ChVector3d w = q.Rotate(ChVector3d(0, 0, a["yaw_rate"]));
    const ChVector3d p0 = ch->GetPos();
    for (auto& b : sys->GetBodies()) {
        if (b->IsFixed()) continue;
        b->SetPosDt(v + w.Cross(b->GetPos() - p0));
        b->SetAngVelParent(w);
    }
    static const char* omegas[2][2] = {{"omega_fl", "omega_fr"}, {"omega_rl", "omega_rr"}};
    std::vector<std::shared_ptr<ChShaftBodyRotation>> couplings;
    for (auto& item : sys->GetOtherPhysicsItems())
        if (auto sb = std::dynamic_pointer_cast<ChShaftBodyRotation>(item)) couplings.push_back(sb);
    // The velocity projection is a mass-weighted least-squares fit, so wheels and the shafts glued to
    // them must already agree or the wheels get dragged towards the shafts' speed. Set wheel spins and
    // their shafts, project, repeat: the free shafts (driveshaft, diff) converge to consistent speeds.
    for (int pass = 0; pass < 8; ++pass) {
        for (int ax = 0; ax < 2; ++ax)
            for (int s = 0; s < 2; ++s) {
                auto sp = c.veh->GetAxle(ax)->m_suspension->GetSpindle(s == 0 ? LEFT : RIGHT);
                // Forward rolling is +spin about the vehicle's left (+Y) axis; spindle frames may point either way.
                ChVector3d axle = sp->GetRot().GetAxisY();
                if (axle.Dot(q.GetAxisY()) < 0) axle = -axle;
                sp->SetAngVelParent(w + axle * std::fabs(a[omegas[ax][s]]));
            }
        for (auto& sb : couplings)
            sb->GetShaft()->SetPosDt(sb->GetBody()->GetAngVelLocal().Dot(sb->GetShaftDirection()));
        if (c.motorshaft) c.motorshaft->SetPosDt(a["rpm"] * CH_2PI / 60.0);
        sys->DoAssembly(AssemblyAnalysis::Level::VELOCITY);
        // The gearbox's own driveshaft is driven kinematically from the driveline at Synchronize.
        c.veh->Synchronize(sys->GetChTime(), DriverInputs{steer, 0, 0, 0}, *c.terrain);
    }
    if (c.motorshaft) c.motorshaft->SetPosDt(a["rpm"] * CH_2PI / 60.0);
    // Gearbox input (clutch side) = output / ratio; the projection leaves it behind otherwise.
    for (auto& item : sys->GetOtherPhysicsItems())
        if (auto gb = std::dynamic_pointer_cast<ChShaftsGearbox>(item))
            if (std::fabs(gb->GetTransmissionRatio()) > 1e-6)
                gb->GetShaft1()->SetPosDt(gb->GetShaft2()->GetPosDt() / gb->GetTransmissionRatio());
}

// Coast-down from 80 km/h, clutch pressed, straight: decel should only be rolling resistance (no aero here).
int Coast(Car& c) {
    double t = Settle(c);
    Row a;
    a.v = {{"vel_z", 80 / 3.6}, {"rpm", 3000}, {"omega_fl", 80 / 3.6 / 0.275}, {"omega_fr", 80 / 3.6 / 0.275},
           {"omega_rl", 80 / 3.6 / 0.275}, {"omega_rr", 80 / 3.6 / 0.275}};
    c.veh->GetTransmission()->SetGear(3);
    SetMotion(c, a, 0);
    t = c.veh->GetSystem()->GetChTime();
    for (auto& s : c.veh->GetSystem()->GetShafts())
        std::printf("  shaft '%s' I %.3f  w %.1f\n", s->GetName().c_str(), s->GetInertia(), s->GetPosDt());
    for (int i = 0; i <= 3000; ++i, t += kStep) {
        if (i <= 30 && i % 3 == 0) {
            std::printf("  %3d ms shafts:", i);
            for (auto& s : c.veh->GetSystem()->GetShafts()) std::printf(" %.1f", s->GetPosDt());
            std::printf("\n");
        }
        if (i <= 100 && i % 10 == 0) {
            std::printf("  %3d ms v %.2f |", i, c.veh->GetSpeed() * 3.6);
            for (auto& ax : c.veh->GetAxles())
                for (auto& wh : ax->GetWheels()) {
                    auto f = wh->GetTire()->ReportTireForce(c.terrain.get());
                    std::printf(" Fx %+5.0f Fz %5.0f w %+6.1f |", f.force.x(), f.force.z(),
                                wh->GetSpindle()->GetAngVelLocal().y());
                }
            std::printf("\n");
        }
        if (i % 250 == 0) {
            double fx = 0, my = 0;
            for (auto& ax : c.veh->GetAxles())
                for (auto& wh : ax->GetWheels()) {
                    auto f = wh->GetTire()->ReportTireForce(c.terrain.get());
                    fx += f.force.x(), my += f.moment.y();
                }
            std::printf("t %.2f  %.2f km/h  tyre Fx sum %+.0f N  My sum %+.0f Nm  wFL %.2f (free roll %.2f)\n", i * kStep,
                        c.veh->GetSpeed() * 3.6, fx, my, c.veh->GetSpindleOmega(0, LEFT), c.veh->GetSpeed() / 0.275);
        }
        Step(c, DriverInputs{0, 0, 0, 1}, t);
    }
    return 0;
}

int Replay(Car& c, const char* csv, double window, double steer_ratio) {
    const std::vector<Row> rows = LoadCsv(csv);
    double row_dt = 0.01;
    {
        std::vector<double> est;
        for (size_t i = 10; i < rows.size(); i += 10) {
            const double v = (rows[i - 10]["speed_kmh"] + rows[i]["speed_kmh"]) / 7.2;
            if (v < 8.0) continue;
            est.push_back(std::hypot(rows[i]["pos_x"] - rows[i - 10]["pos_x"], rows[i]["pos_z"] - rows[i - 10]["pos_z"]) / v / 10.0);
        }
        if (!est.empty()) {
            std::sort(est.begin(), est.end());
            row_dt = est[est.size() / 2];
        }
    }
    const int n = static_cast<int>(window / row_dt + 0.5);
    // AC steer: -1..1 of 450 deg wheel lock, + = right. Chrono: -1..1 of its lock, + = left.
    const double steer_scale = -450.0 / steer_ratio / (c.lock * kDeg);

    double t = Settle(c);
    ChSystem* sys = c.veh->GetSystem();
    ChState x0(sys->GetNumCoordsPosLevel(), sys);
    ChStateDelta v0(sys->GetNumCoordsVelLevel(), sys);
    double T0 = 0;
    sys->StateGather(x0, v0, T0);

    struct Bin { double n = 0, e = 0, e2 = 0, ey = 0, es = 0; };
    std::map<int, Bin> bins;
    double speed_by[4][2] = {};   // coast, full gas, braking, part gas
    int windows = 0;
    double base2 = 0, sxy = 0, sxx = 0, syy = 0;
    const auto w0 = std::chrono::steady_clock::now();
    for (size_t i = 0; i + n < rows.size(); i += n) {
        const Row& a = rows[i];
        const Row& b = rows[i + n];
        if (a["speed_kmh"] < 25 || a["gear"] < 1 || a["vel_z"] < 3) continue;
        bool same_gear = true;
        for (int k = 0; k < n; ++k) same_gear &= rows[i + k]["gear"] == a["gear"];
        if (!same_gear) continue;
        // Skip AC glitches: car reset/teleported or the recording paused inside the window.
        bool glitch = b["speed_kmh"] < 3;
        for (int k = 1; k <= n && !glitch; ++k) {
            const Row& p = rows[i + k - 1];
            const Row& q = rows[i + k];
            glitch = std::hypot(q["pos_x"] - p["pos_x"], q["pos_z"] - p["pos_z"]) > 3 * row_dt * p["speed_kmh"] / 3.6 + 0.5;
        }
        if (glitch) continue;
        if (std::getenv("DL_FROM") && i < static_cast<size_t>(std::atoi(std::getenv("DL_FROM")))) continue;
        sys->StateScatter(x0, v0, sys->GetChTime(), UpdateFlags::UPDATE_ALL_NO_VISUAL);  // never rewind time (tyres integrate on it)
        c.veh->GetTransmission()->SetGear(static_cast<int>(a["gear"]));
        SetMotion(c, a, std::clamp(a["steer"] * steer_scale, -1.0, 1.0));
        t = sys->GetChTime();
        const bool dbg = windows < 2;
        if (dbg) {
            double hp = 0, hv = 0, hs = 0;
            for (auto& bd : sys->GetBodies()) hp += bd->GetPos().Length(), hv += bd->GetPosDt().Length() + bd->GetAngVelParent().Length();
            for (auto& sh : sys->GetShafts()) hs += std::fabs(sh->GetPosDt());
            std::printf("  state hash: pos %.6f vel %.6f shafts %.6f\n", hp, hv, hs);
            for (auto& bd : sys->GetBodies())
                if (bd->GetPosDt().Length() + bd->GetAngVelParent().Length() > 60)
                    std::printf("  fast body '%s' v %.1f w %.1f\n", bd->GetName().c_str(), bd->GetPosDt().Length(),
                                bd->GetAngVelParent().Length());
            for (auto& ax : c.veh->GetAxles())
                for (auto& wh : ax->GetWheels()) {
                    auto f = wh->GetTire()->ReportTireForce(c.terrain.get());
                    std::printf("  tire force %.0f %.0f %.0f\n", f.force.x(), f.force.y(), f.force.z());
                }
        }
        if (dbg)
            std::printf("window %zu: AC %.1f km/h gear %.0f rpm %.0f -> ours %.1f km/h motor %.0f rpm\n", i, a["speed_kmh"],
                        a["gear"], a["rpm"], c.veh->GetSpeed() * 3.6, c.veh->GetEngine()->GetMotorSpeed() * 60 / CH_2PI);
        for (int k = 0; k < n; ++k) {
            const Row& r = rows[i + k];
            DriverInputs in{std::clamp(r["steer"] * steer_scale, -1.0, 1.0), r["gas"], r["brake"], 1.0 - r["clutch"]};  // AC: 1 = engaged; Chrono: pedal
            const int sub = static_cast<int>(row_dt / kStep + 0.5);
            for (int s = 0; s < sub; ++s, t += kStep) Step(c, in, t);
            if (dbg)
                std::printf("  row %d: ours %.1f km/h z %.3f yaw %.3f | AC %.1f km/h yaw %.3f\n", k, c.veh->GetSpeed() * 3.6,
                            c.veh->GetChassisBody()->GetPos().z(), YawRate(c), rows[i + k + 1]["speed_kmh"],
                            rows[i + k + 1]["yaw_rate"]);
        }
        const ChVector3d lv = LocalVel(c);
        const double ours = std::hypot(lv.x(), lv.y()) > 1 ? std::atan2(lv.y(), lv.x()) * kDeg : 0;
        const double ac0 = AcDrift(a), ac1 = AcDrift(b);
        const double sgn = ac0 >= 0 ? 1.0 : -1.0;
        const int band = std::min(60, static_cast<int>(std::fabs(ac0)) / 10 * 10);
        Bin& bin = bins[band];
        bin.n += 1;
        bin.e += (ours - ac1) * sgn;
        bin.e2 += (ours - ac1) * (ours - ac1);
        if (std::fabs(ours - ac1) > 15)
            std::printf("outlier row %zu: AC drift %.1f -> %.1f, ours %.1f, %.0f km/h, steer %.2f gas %.2f\n", i, ac0, ac1,
                        ours, a["speed_kmh"], a["steer"], a["gas"]);
        bin.ey += (YawRate(c) - b["yaw_rate"]) * (b["yaw_rate"] >= 0 ? 1.0 : -1.0);
        bin.es += std::hypot(lv.x(), lv.y()) * 3.6 - b["speed_kmh"];   // AC speed is total, not forward
        {
            const int g = a["gas"] < 0.05 ? (a["brake"] > 0.05 ? 2 : 0) : (a["gas"] > 0.9 ? 1 : 3);
            speed_by[g][0] += 1;
            speed_by[g][1] += std::hypot(lv.x(), lv.y()) * 3.6 - b["speed_kmh"];
        }
        const double dp = ours - ac0, da = ac1 - ac0;
        base2 += da * da, sxy += dp * da, sxx += dp * dp, syy += da * da;
        ++windows;
    }
    const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - w0).count();
    std::printf("%d windows of %.2f s (%.1f s wall)\n|AC drift|  n    drift err (+ = we rotate more)  RMS    yaw err      speed err\n",
                windows, window, wall);
    double tn = 0, td = 0, td2 = 0;
    for (auto& [band, bin] : bins) {
        std::printf("%3d-%-3d  %5.0f   %+6.2f deg                    %5.2f   %+6.3f rad/s  %+5.2f km/h\n", band, band + 10,
                    bin.n, bin.e / bin.n, std::sqrt(bin.e2 / bin.n), bin.ey / bin.n, bin.es / bin.n);
        tn += bin.n, td += bin.e, td2 += bin.e2;
    }
    static const char* pedal[4] = {"coast", "full gas", "braking", "part gas"};
    for (int g = 0; g < 4; ++g)
        std::printf("speed error %-9s n %4.0f  %+5.2f km/h\n", pedal[g], speed_by[g][0], speed_by[g][0] > 0 ? speed_by[g][1] / speed_by[g][0] : 0.0);
    std::printf("ALL      %5.0f   %+6.2f deg                    %5.2f\n", tn, td / tn, std::sqrt(td2 / tn));
    std::printf("baseline (assume no change) RMS %.2f | corr %.2f | our change / AC change %.2f\n",
                std::sqrt(base2 / windows), sxy / std::sqrt(sxx * syy), sxy / syy);
    return 0;
}
}  // namespace

int main(int argc, char** argv) {
    if (argc < 4) {
        std::printf("usage: chrono_lab <car root> <car id> perf | skidpad [kmh] | replay <csv> [window_s] [steer_ratio]\n");
        return 2;
    }
    Car c;
    Build(c, argv[1], argv[2]);
    const std::string mode = argv[3];
    std::printf("car %s: lock %.1f deg, motorshaft %s, %zu bodies, %zu shafts\n", argv[2], c.lock * kDeg,
                c.motorshaft ? "found" : "MISSING", c.veh->GetSystem()->GetBodies().size(),
                c.veh->GetSystem()->GetShafts().size());
    if (mode == "lock") return Settle(c) > 0 ? 0 : 1;   // prints the measured full lock
    if (mode == "perf") return Perf(c);
    if (mode == "coast") return Coast(c);
    if (mode == "diag") return Diag(c);
    if (mode == "reset") return ResetTest(c);
    if (mode == "skidpad") return Skidpad(c, argc > 4 ? std::atof(argv[4]) : 60.0);
    if (mode == "replay" && argc > 4)
        return Replay(c, argv[4], argc > 5 ? std::atof(argv[5]) : 0.25, argc > 6 ? std::atof(argv[6]) : 8.5);
    std::printf("unknown mode\n");
    return 2;
}
