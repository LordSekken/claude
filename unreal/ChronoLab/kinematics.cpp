// Suspension kinematics & compliance of a Chrono::Vehicle car converted by ac_to_chrono.py:
//   kinematics <car root> <car id> [steering_wheel_lock_deg=450]
// The chassis is fixed and each spindle centre is driven vertically by a constraint; the linkage
// is solved by position assembly (pure kinematics: springs/ARBs do not move anything), and the
// spring / damper / anti-roll-bar rates at the wheel come from virtual work on the solved poses.
// Then the car is dropped on flat ground with tyres to check where it really sits.
// Frames: vehicle ISO (X forward, Y left, Z up). Signs: camber + = top out, toe + = toe-out,
// travel + = bump (wheel up relative to the body), caster + = upper ball joint behind,
// KPI + = upper ball joint inboard, trail + = kingpin axis meets the ground ahead of the contact,
// scrub + = kingpin axis meets the ground inboard of the contact.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/solver/ChIterativeSolverVI.h"
#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ChDoubleWishbone.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace {
constexpr double kDeg = 57.29578;

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

struct Rig {
    std::unique_ptr<WheeledVehicle> veh;
    std::shared_ptr<ChBody> target[2][2];   // fixed bodies carrying each spindle's height constraint
    double z0[2][2];                        // design spindle heights
    double radius[2];
    double arb_k[2];
    std::shared_ptr<ChBody> arm[2][2];      // anti-roll bar arms [axle][side]
    ChSystem* sys = nullptr;
};

std::shared_ptr<ChDoubleWishbone> Dwb(Rig& r, int axle) {
    return std::dynamic_pointer_cast<ChDoubleWishbone>(r.veh->GetAxle(axle)->m_suspension);
}

void BuildRig(Rig& r, const std::string& root, const std::string& id) {
    SetVehicleDataPath(root + "/");
    const std::string car = ReadFile(root + "/" + id + "/car.json");
    const std::string vfile = GetVehicleDataFile(JsonStr(car, "vehicle"));
    r.veh = std::make_unique<WheeledVehicle>(vfile, ChContactMethod::NSC);
    r.veh->Initialize(ChCoordsys<>(ChVector3d(0, 0, 0), QUNIT));
    r.veh->GetChassis()->SetFixed(true);
    r.sys = r.veh->GetSystem();
    r.sys->SetSolverType(ChSolver::Type::SPARSE_LU);
    const std::string vdoc = ReadFile(vfile);
    for (int a = 0; a < 2; ++a) {
        auto susp = r.veh->GetAxle(a)->m_suspension;
        r.radius[a] = susp->GetSpindle(LEFT)->GetPos().z();
        for (int s = 0; s < 2; ++s) {
            auto sp = susp->GetSpindle(s == 0 ? LEFT : RIGHT);
            r.z0[a][s] = sp->GetPos().z();
            auto t = chrono_types::make_shared<ChBody>();
            t->SetFixed(true);
            t->SetPos(sp->GetPos());
            r.sys->AddBody(t);
            r.target[a][s] = t;
            auto mate = chrono_types::make_shared<ChLinkMateGeneric>(false, false, true, false, false, false);
            mate->Initialize(sp, t, false, ChFrame<>(sp->GetPos()), ChFrame<>(sp->GetPos()));
            r.sys->AddLink(mate);
        }
        r.arb_k[a] = 0;
        for (auto& b : r.sys->GetBodies()) {
            const std::string n = b->GetName();
            for (int s = 0; s < 2; ++s)
                if (n.find(s == 0 ? "_arm_left" : "_arm_right") != std::string::npos &&
                    std::fabs(b->GetPos().x() - susp->GetSpindle(LEFT)->GetPos().x()) < 1.2 && !r.arm[a][s])
                    r.arm[a][s] = b;
        }
    }
    // ARB rotational stiffness from the ARB JSONs (axle order as in Vehicle.json).
    size_t pos = 0;
    for (int a = 0; a < 2; ++a) {
        pos = vdoc.find("\"Antirollbar Input File\"", pos);
        if (pos == std::string::npos) break;
        const std::string f = JsonStr(vdoc.substr(pos), "Antirollbar Input File");
        r.arb_k[a] = JsonNum(ReadFile(GetVehicleDataFile(f)), "Spring Coefficient", 0);
        ++pos;
    }
}

void SetTravel(Rig& r, double zl0, double zr0, double zl1, double zr1, double steer = 0.0) {
    const double z[2][2] = {{zl0, zr0}, {zl1, zr1}};
    for (int a = 0; a < 2; ++a)
        for (int s = 0; s < 2; ++s) {
            auto p = r.target[a][s]->GetPos();
            r.target[a][s]->SetPos(ChVector3d(p.x(), p.y(), r.z0[a][s] + z[a][s]));
        }
    DriverInputs in{steer, 0, 0, 0};
    r.veh->GetSteering(0)->Synchronize(0, in);
    r.sys->DoAssembly(AssemblyAnalysis::Level::POSITION, 50, 1e-12, 1e-10, 1e-10);
    r.sys->Update(UpdateFlags::UPDATE_ALL_NO_VISUAL);
}

struct Corner {
    double camber, toe_out, steer;   // deg (steer + = left)
    ChVector3d wc, contact, lbj, ubj;
    double spring_len, shock_len, spring_force;
};

Corner Measure(Rig& r, int a, int s) {
    Corner c{};
    auto side = s == 0 ? LEFT : RIGHT;
    auto dwb = Dwb(r, a);
    auto sp = dwb->GetSpindle(side);
    const ChVector3d y = sp->GetRot().GetAxisY();   // spin axis (points left for both sides)
    c.steer = std::atan2(-y.x(), y.y()) * kDeg;
    c.camber = (s == 0 ? -1 : 1) * std::asin(y.z()) * kDeg;
    c.toe_out = (s == 0 ? 1 : -1) * c.steer;
    c.wc = sp->GetPos();
    ChVector3d down = ChVector3d(0, 0, -1) - y * (-y.z());
    down.Normalize();
    c.contact = c.wc + down * r.radius[a];
    c.lbj = dwb->GetBallJointPosLCA(side);
    c.ubj = dwb->GetBallJointPosUCA(side);
    c.spring_len = dwb->GetSpringLength(side);
    c.shock_len = dwb->GetShockLength(side);
    c.spring_force = dwb->GetSpringForce(side);
    return c;
}

double ArbAngle(Rig& r, int a) {
    if (!r.arm[a][0] || !r.arm[a][1]) return 0;
    const ChQuaternion<> q = r.arm[a][0]->GetRot().GetConjugate() * r.arm[a][1]->GetRot();
    return 2 * std::atan2(q.e2(), q.e0());   // relative rotation about the bar (lateral) axis
}

// Wheel-centre vertical force (N, + = pushes the wheel down / body up) from the spring at travel z,
// by virtual work: F_w = F_spring * (-dL/dz).
double WheelForce(Rig& r, int a, double z, double* mr_spring = nullptr, double* mr_shock = nullptr) {
    const double h = 1e-4;
    auto set = [&](double zz) {
        if (a == 0) SetTravel(r, zz, zz, 0, 0); else SetTravel(r, 0, 0, zz, zz);
    };
    set(z - h);
    const Corner m = Measure(r, a, 0);
    set(z + h);
    const Corner p = Measure(r, a, 0);
    set(z);
    const Corner c = Measure(r, a, 0);
    const double mr = -(p.spring_len - m.spring_len) / (2 * h);
    if (mr_spring) *mr_spring = mr;
    if (mr_shock) *mr_shock = -(p.shock_len - m.shock_len) / (2 * h);
    return c.spring_force * mr;
}

void Kinematics(Rig& r, double sw_lock) {
    const char* names[2] = {"FRONT", "REAR"};
    for (int a = 0; a < 2; ++a) {
        auto set = [&](double z) { if (a == 0) SetTravel(r, z, z, 0, 0); else SetTravel(r, 0, 0, z, z); };
        set(0);
        const Corner c0 = Measure(r, a, 0), c0r = Measure(r, a, 1);
        const double half = c0.wc.y();
        // Steering axis at design.
        ChVector3d k = (c0.ubj - c0.lbj).GetNormalized();
        const double caster = std::atan2(-k.x(), k.z()) * kDeg;
        const double kpi = std::atan2(-k.y(), k.z()) * kDeg;   // left side: inboard = -Y
        const double sgr = (c0.contact.z() - c0.lbj.z()) / k.z();
        const ChVector3d g = c0.lbj + k * sgr;
        const double trail = g.x() - c0.contact.x();
        const double scrub = g.y() - c0.contact.y();   // left side: + = intercept inboard? (inboard is -Y)
        std::printf("== %s (left corner; right: camber %+.3f toe-out %+.3f)\n", names[a], c0r.camber, c0r.toe_out);
        std::printf("  static: camber %+.2f deg  toe-out %+.3f deg  caster %.2f deg  KPI %.2f deg  trail %.1f mm  scrub %.1f mm\n",
                    c0.camber, c0.toe_out, caster, kpi, trail * 1000, -scrub * 1000);
        std::printf("  wheel centre y %.4f  contact y %.4f  LBJ (%.3f %.3f %.3f) UBJ (%.3f %.3f %.3f)\n", c0.wc.y(),
                    c0.contact.y(), c0.lbj.x(), c0.lbj.y(), c0.lbj.z(), c0.ubj.x(), c0.ubj.y(), c0.ubj.z());
        // Travel sweep.
        std::printf("  travel  camber   toe-out  contact-dy  spring-L  MR(spring) MR(shock)  F_wheel   rate(N/m)\n");
        double prevF = 0;
        for (int cm = -6; cm <= 6; ++cm) {
            const double z = cm / 100.0;
            double mrs = 0, mrd = 0;
            const double Fp = WheelForce(r, a, z + 0.002);
            const double Fm = WheelForce(r, a, z - 0.002);
            const double F = WheelForce(r, a, z, &mrs, &mrd);
            const Corner c = Measure(r, a, 0);
            std::printf("  %+3d cm  %+6.2f   %+7.3f   %+7.2f mm  %.4f    %.3f      %.3f    %7.0f   %7.0f\n", cm, c.camber,
                        c.toe_out, (c.contact.y() - c0.contact.y()) * 1000, c.spring_len, mrs, mrd, F,
                        (Fp - Fm) / 0.004);
            prevF = F;
        }
        (void)prevF;
        // Gradients at design (+-1 cm).
        set(-0.01);
        const Corner cm1 = Measure(r, a, 0);
        set(0.01);
        const Corner cp1 = Measure(r, a, 0);
        set(0);
        const double gain = (cp1.camber - cm1.camber) / 2;            // deg/cm
        const double bump_steer = (cp1.toe_out - cm1.toe_out) / 2;    // deg/cm
        const double dy = (cp1.contact.y() - cm1.contact.y()) / 0.02;  // outward (+Y) contact motion per bump
        const double dg = (cp1.camber - cm1.camber) / kDeg / 0.02;     // rad/m, + = top out
        const double fvsa = std::fabs(dg) > 1e-9 ? -1.0 / dg : 1e9;
        const double rc = c0.contact.y() * dy;   // contact y = distance from the centreline
        std::printf("  camber gain %+.3f deg/cm  bump steer %+.4f deg/cm (toe-out)  FVSA %.2f m  IC height %.0f mm  RC %.0f mm\n",
                    gain, bump_steer, fvsa, fvsa * dy * 1000, rc * 1000);
        double mrs = 0, mrd = 0;
        const double Fs = WheelForce(r, a, 0, &mrs, &mrd);
        const double rate = (WheelForce(r, a, 0.005) - WheelForce(r, a, -0.005)) / 0.01;
        auto dwb = Dwb(r, a);
        auto shock = dwb->GetShock(LEFT);
        auto damp = [&](double vw) {   // wheel damping force for wheel speed vw (+ = bump)
            const double vs = -mrd * vw;
            return shock->GetForceFunctor()->evaluate(0, shock->GetRestLength(), shock->GetLength(), vs, *shock) * mrd;
        };
        std::printf("  spring MR %.3f  shock MR %.3f  wheel rate %.0f N/m  static wheel force %.0f N\n", mrs, mrd, rate, Fs);
        std::printf("  wheel damping (N s/m): bump 0.05 %.0f  0.3 %.0f | rebound 0.05 %.0f  0.3 %.0f\n", damp(0.05) / 0.05,
                    damp(0.3) / 0.3, damp(-0.05) / -0.05, damp(-0.3) / -0.3);
        // ARB: pure roll (left up, right down) and single wheel.
        for (int mode = 0; mode < 2; ++mode) {
            const double z = 0.02;
            const double zr = mode == 0 ? -z : 0.0;
            if (a == 0) SetTravel(r, z, zr, 0, 0); else SetTravel(r, 0, 0, z, zr);
            const double th = ArbAngle(r, a);
            const double e = 0.5 * r.arb_k[a] * th * th;
            std::printf("  ARB %s: twist %.3f deg  -> AC-equivalent rate %.0f N/m (k_rot %.0f N m/rad)\n",
                        mode == 0 ? "roll  " : "1-wheel", th * kDeg, 2 * e / ((z - zr) * (z - zr)), r.arb_k[a]);
        }
        set(0);
    }
    // Steering.
    std::printf("== STEERING (steering wheel = input x %.0f deg)\n", sw_lock);
    SetTravel(r, 0, 0, 0, 0, 0);
    const double base_l = Measure(r, 0, 0).steer, base_r = Measure(r, 0, 1).steer;
    const double L = Measure(r, 0, 0).wc.x() - Measure(r, 1, 0).wc.x();
    const double T = 2 * Measure(r, 0, 0).wc.y();
    for (double sw : {5.0, 45.0, 90.0, 180.0, 270.0, 360.0, 450.0}) {
        const double in = sw / sw_lock;
        SetTravel(r, 0, 0, 0, 0, in);   // left turn
        const double inner = Measure(r, 0, 0).steer - base_l, outer = Measure(r, 0, 1).steer - base_r;
        const double avg = (inner + outer) / 2;
        const double ideal = std::atan(1.0 / (1.0 / std::tan(outer / kDeg) - T / L)) * kDeg;
        std::printf("  SW %3.0f deg: inner %6.2f  outer %6.2f  avg %6.2f  ratio %5.2f  Ackermann %4.0f%%\n", sw, inner, outer,
                    avg, sw / avg, (inner - outer) / (ideal - outer) * 100);
    }
    SetTravel(r, 0, 0, 0, 0, 0);
}

// Drop the car (tyres on flat ground) and report where each corner settles relative to design.
void Settle(const std::string& root, const std::string& id) {
    SetVehicleDataPath(root + "/");
    const std::string car = ReadFile(root + "/" + id + "/car.json");
    WheeledVehicle veh(GetVehicleDataFile(JsonStr(car, "vehicle")), ChContactMethod::NSC);
    veh.Initialize(ChCoordsys<>(ChVector3d(0, 0, 0.0), QUNIT));
    auto engine = ReadEngineJSON(GetVehicleDataFile(JsonStr(car, "engine")));
    auto trans = ReadTransmissionJSON(GetVehicleDataFile(JsonStr(car, "transmission")));
    veh.InitializePowertrain(chrono_types::make_shared<ChPowertrainAssembly>(engine, trans));
    for (auto& axle : veh.GetAxles())
        for (auto& wheel : axle->GetWheels()) {
            const bool front = axle == veh.GetAxle(0);
            veh.InitializeTire(ReadTireJSON(GetVehicleDataFile(JsonStr(car, front ? "tire_front" : "tire_rear"))), wheel,
                               VisualizationType::NONE);
        }
    ChSystem* sys = veh.GetSystem();
    sys->SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    std::static_pointer_cast<ChIterativeSolverVI>(sys->GetSolver())->SetMaxIterations(100);
    sys->SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    RigidTerrain terrain(sys);
    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
    terrain.AddPatch(mat, ChCoordsys<>(ChVector3d(0, 0, 0), QUNIT), 200, 200);
    terrain.Initialize();
    // Spindle heights relative to the chassis at design.
    auto rel = [&](int a, VehicleSide s) {
        return veh.GetChassisBody()->GetFrameRefToAbs().TransformPointParentToLocal(veh.GetSpindlePos(a, s)).z();
    };
    double d0[2] = {rel(0, LEFT), rel(1, LEFT)};
    const double cg0 = veh.GetChassisBody()->GetPos().z();
    double t = 0;
    for (int i = 0; i < 4000; ++i, t += 1e-3) {
        DriverInputs in{0, 0, 1, 0};
        veh.Synchronize(t, in, terrain);
        terrain.Synchronize(t);
        veh.Advance(1e-3);
        terrain.Advance(1e-3);
    }
    std::printf("== SETTLED on tyres (4 s): chassis CoM height %.4f m (initial %.4f)\n", veh.GetChassisBody()->GetPos().z(), cg0);
    for (int a = 0; a < 2; ++a) {
        auto dwb = std::dynamic_pointer_cast<ChDoubleWishbone>(veh.GetAxle(a)->m_suspension);
        const ChVector3d y = veh.GetChassisBody()->GetRot().RotateBack(veh.GetSpindleRot(a, LEFT).GetAxisY());
        std::printf("  %s: travel from design %+.1f mm  camber %+.2f deg  toe-out %+.3f deg  tyre load %.0f N  spindle z %.4f\n",
                    a == 0 ? "FRONT" : "REAR ", (rel(a, LEFT) - d0[a]) * 1000, -std::asin(y.z()) * kDeg,
                    std::atan2(-y.x(), y.y()) * kDeg, veh.GetWheel(a, LEFT)->GetTire()->ReportTireForce(&terrain).force.z(),
                    veh.GetSpindlePos(a, LEFT).z());
        (void)dwb;
    }
}
}  // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::printf("usage: kinematics <car root> <car id> [steering_wheel_lock_deg=450]\n");
        return 1;
    }
    const double sw_lock = argc > 3 ? std::atof(argv[3]) : 450.0;
    {
        Rig r;
        BuildRig(r, argv[1], argv[2]);
        Kinematics(r, sw_lock);
    }
    Settle(argv[1], argv[2]);
    return 0;
}
