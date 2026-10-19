// DriftChrono implementation: a Chrono::Vehicle wheeled vehicle built from the
// JSON set written by drift-sim/tools/ac_to_chrono.py, with the host's ground
// ray casts as terrain and the host's body contacts as penalty springs.
// Chrono works in ISO (X forward, Y left, Z up); the API is DriftCore's frame.
#include "drift_chrono.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChShaftBodyConstraint.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/solver/ChIterativeSolverVI.h"
#include "chrono_vehicle/ChTerrain.h"
#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace {
constexpr double kStep = 1e-3;

// DriftCore (x right, y up, z back) <-> ISO (X forward, Y left, Z up). A proper rotation.
ChVector3d Iso(const double d[3]) { return ChVector3d(-d[2], -d[0], d[1]); }
void Dc(const ChVector3d& v, double out[3]) {
    out[0] = -v.y();
    out[1] = v.z();
    out[2] = -v.x();
}

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

std::vector<double> JsonVec(const std::string& doc, const std::string& key) {
    std::vector<double> v;
    auto i = doc.find("\"" + key + "\"");
    if (i == std::string::npos) return v;
    i = doc.find('[', i);
    const auto end = doc.find(']', i);
    std::stringstream ss(doc.substr(i + 1, end - i - 1));
    std::string cell;
    while (std::getline(ss, cell, ',')) v.push_back(std::atof(cell.c_str()));
    return v;
}

std::string JsonStr(const std::string& doc, const std::string& key) {
    auto i = doc.find("\"" + key + "\"");
    if (i == std::string::npos) return "";
    i = doc.find('"', doc.find(':', i) + 1);
    return doc.substr(i + 1, doc.find('"', i + 1) - i - 1);
}

// The host's level as Chrono terrain: one ray per tyre query.
class HostTerrain : public ChTerrain {
  public:
    dc_raycast_fn fn = nullptr;
    void* ctx = nullptr;

    void GetProperties(const ChVector3d& loc, ChVector3d& point, double& height, ChVector3d& normal,
                       float& friction) const override {
        double from[3], to[3], p[3], n[3], grip = 1.0;
        Dc(loc, from);
        Dc(loc - ChVector3d(0, 0, 4.0), to);
        if (fn && fn(ctx, from, to, p, n, &grip)) {
            point = Iso(p);
            normal = Iso(n);
            height = point.z();
            // Pac02 scales grip by mu_road / 0.8 (its "Coefficient of Friction") and clamps mu to 0.1..1.
            friction = static_cast<float>(std::clamp(0.8 * grip, 0.1, 1.0));
        } else {
            point = ChVector3d(loc.x(), loc.y(), -1e6);
            normal = ChVector3d(0, 0, 1);
            height = -1e6;
            friction = 0.8f;
        }
    }
    double GetHeight(const ChVector3d& loc) const override {
        ChVector3d p, n;
        double h;
        float mu;
        GetProperties(loc, p, h, n, mu);
        return h;
    }
    ChVector3d GetPoint(const ChVector3d& loc) const override {
        ChVector3d p, n;
        double h;
        float mu;
        GetProperties(loc, p, h, n, mu);
        return p;
    }
    ChVector3d GetNormal(const ChVector3d& loc) const override {
        ChVector3d p, n;
        double h;
        float mu;
        GetProperties(loc, p, h, n, mu);
        return n;
    }
    float GetCoefficientFriction(const ChVector3d& loc) const override {
        ChVector3d p, n;
        double h;
        float mu;
        GetProperties(loc, p, h, n, mu);
        return mu;
    }
};

struct Contact {
    ChVector3d local;   // chassis-ref-frame point that was touching
    ChVector3d p0, n;   // world (ISO) at query time
    double depth;
    ChVector3d other_vel;
    double other_mass;
};
}  // namespace

struct dc_car {
    std::unique_ptr<WheeledVehicle> veh;
    HostTerrain terrain;
    ChVector3d dc_origin;   // DriftCore origin in the chassis reference frame (ISO)
    double lock = 0.9;
    double mass = 1200;
    int forward_gears = 5;
    bool neutral = false;
    double accumulator = 0;
    double spin[4] = {0, 0, 0, 0};
    unsigned int contact_acc = 0;
    std::vector<Contact> contacts;
    std::shared_ptr<ChLinkLockLinActuator> rack;
    double last_us = 0;
};

namespace {
std::shared_ptr<ChWheel> WheelAt(const dc_car* c, int i) {
    return c->veh->GetWheel(i / 2, (i % 2) == 0 ? LEFT : RIGHT);
}

ChFrame<> ChassisRef(const dc_car* c) { return c->veh->GetChassisBody()->GetFrameRefToAbs(); }

void ApplyContacts(dc_car* c) {
    auto body = c->veh->GetChassisBody();
    body->EmptyAccumulator(c->contact_acc);
    if (c->contacts.empty()) return;
    const ChFrame<> ref = ChassisRef(c);
    // ~6 Hz on the car's mass, near-critical damping, shared between simultaneous contacts.
    const double k = c->mass * std::pow(CH_2PI * 6.0, 2) / c->contacts.size();
    const double damp = 2.0 * std::sqrt(k * c->mass / c->contacts.size()) * 0.7;
    for (const Contact& ct : c->contacts) {
        const ChVector3d p = ref.TransformPointLocalToParent(ct.local);
        const double pen = ct.depth - Vdot(p - ct.p0, ct.n);
        if (pen <= 0) continue;
        const ChVector3d v = body->GetPosDt() + Vcross(body->GetAngVelParent(), p - body->GetPos()) - ct.other_vel;
        const double vn = Vdot(v, ct.n);
        const double share = ct.other_mass > 0 ? ct.other_mass / (ct.other_mass + c->mass) : 1.0;
        const double fn = std::max(0.0, share * (k * pen - damp * vn));
        ChVector3d f = ct.n * fn;
        ChVector3d vt = v - ct.n * vn;
        const double vts = vt.Length();
        if (vts > 1e-3) f -= vt / vts * std::min(0.4 * fn, damp * vts);
        body->AccumulateForce(c->contact_acc, f, p, false);
    }
}
}  // namespace

extern "C" {

DC_API int dc_version(void) { return DC_VERSION; }

DC_API dc_car* dc_create(const char* car_root, const char* car_id, char* err, int err_len) {
    auto fail = [&](const std::string& m) -> dc_car* {
        if (err && err_len > 0) std::snprintf(err, static_cast<size_t>(err_len), "%s", m.c_str());
        return nullptr;
    };
    const std::string root = car_root;
    const std::string car = ReadFile(root + "/" + car_id + "/car.json");
    if (car.empty()) return fail("missing " + root + "/" + car_id + "/car.json (run drift-sim/tools/ac_to_chrono.py)");
    auto c = std::make_unique<dc_car>();
    try {
        SetVehicleDataPath(root + "/");
        c->veh = std::make_unique<WheeledVehicle>(GetVehicleDataFile(JsonStr(car, "vehicle")), ChContactMethod::NSC);
        c->veh->Initialize(ChCoordsys<>(ChVector3d(0, 0, 0), QUNIT));
        c->veh->SetChassisVisualizationType(VisualizationType::NONE);
        auto engine = ReadEngineJSON(GetVehicleDataFile(JsonStr(car, "engine")));
        auto trans = ReadTransmissionJSON(GetVehicleDataFile(JsonStr(car, "transmission")));
        if (!engine || !trans) return fail("bad engine/transmission JSON");
        c->veh->InitializePowertrain(chrono_types::make_shared<ChPowertrainAssembly>(engine, trans));
        for (auto& axle : c->veh->GetAxles())
            for (auto& wheel : axle->GetWheels()) {
                const bool front = axle == c->veh->GetAxle(0);
                auto tire = ReadTireJSON(GetVehicleDataFile(JsonStr(car, front ? "tire_front" : "tire_rear")));
                if (!tire) return fail("bad tyre JSON");
                c->veh->InitializeTire(tire, wheel, VisualizationType::NONE);
            }
    } catch (const std::exception& e) {
        return fail(std::string("Chrono: ") + e.what());
    }
    c->lock = JsonNum(car, "steer_lock", 0.9);
    c->forward_gears =
        static_cast<int>(JsonVec(ReadFile(GetVehicleDataFile(JsonStr(car, "transmission"))), "Forward Gear Ratios").size());
    c->veh->GetChassis()->SetAerodynamicDrag(JsonNum(car, "cda", 0.7), 1.0, 1.2);
    const auto o = JsonVec(car, "dc_origin");
    c->dc_origin = o.size() == 3 ? ChVector3d(o[0], o[1], o[2]) : ChVector3d(0, 0, 0.3);
    c->mass = c->veh->GetMass();

    ChSystem* sys = c->veh->GetSystem();
    sys->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    // Clutches/brakes are friction-limited constraints: they need a VI solver.
    sys->SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    auto solver = std::static_pointer_cast<ChIterativeSolverVI>(sys->GetSolver());
    solver->SetMaxIterations(100);
    solver->SetOmega(0.8);
    solver->SetSharpnessLambda(1.0);
    sys->SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    c->contact_acc = c->veh->GetChassisBody()->AddAccumulator();
    for (auto& l : sys->GetLinks())
        if (auto a = std::dynamic_pointer_cast<ChLinkLockLinActuator>(l)) {
            c->rack = a;
            if (std::getenv("DC_VERBOSE")) std::printf("rack actuator: %s\n", a->GetName().c_str());
        }
    return c.release();
}

DC_API void dc_destroy(dc_car* car) { delete car; }

DC_API void dc_set_ground(dc_car* car, dc_raycast_fn fn, void* ctx) {
    car->terrain.fn = fn;
    car->terrain.ctx = ctx;
}

DC_API void dc_reset(dc_car* car, const double origin[3], const double rot[9]) {
    // Target chassis reference frame: body axes from the DriftCore columns (right, up, back).
    const ChVector3d x = -Iso(rot + 6), y = -Iso(rot + 0), z = Iso(rot + 3);
    ChMatrix33<> m;
    m.SetFromDirectionAxes(x, y, z);
    const ChQuaternion<> q = m.GetQuaternion();
    const ChVector3d ref_pos = Iso(origin) - q.Rotate(car->dc_origin);
    const ChFrame<> target(ref_pos, q);
    const ChFrame<> delta = target * ChassisRef(car).GetInverse();
    ChSystem* sys = car->veh->GetSystem();
    for (auto& b : sys->GetBodies()) {
        if (b->IsFixed()) continue;
        const ChFrame<> f = delta * ChFrame<>(b->GetPos(), b->GetRot());
        b->SetPos(f.GetPos());
        b->SetRot(f.GetRot());
        b->SetPosDt(VNULL);
        b->SetAngVelParent(VNULL);
    }
    for (auto& s : sys->GetShafts()) s->SetPosDt(0);
    sys->Update(UpdateFlags::UPDATE_ALL_NO_VISUAL);
    car->accumulator = 0;
    car->contacts.clear();
}

DC_API int dc_step(dc_car* car, double dt, const dc_controls* ctl, const dc_contact* contacts, int count) {
    const auto t0 = std::chrono::steady_clock::now();
    ChSystem* sys = car->veh->GetSystem();
    car->contacts.clear();
    const ChFrame<> ref = ChassisRef(car);
    for (int i = 0; i < count; ++i) {
        Contact ct;
        ct.p0 = Iso(contacts[i].point);
        ct.n = Iso(contacts[i].normal);
        ct.local = ref.TransformPointParentToLocal(ct.p0);
        ct.depth = contacts[i].depth;
        ct.other_vel = Iso(contacts[i].other_vel);
        ct.other_mass = contacts[i].other_mass;
        car->contacts.push_back(ct);
    }
    auto tr = car->veh->GetTransmission();
    // Chrono's neutral is a 1e20 gear ratio that blows up the gearbox constraint: keep a real
    // gear engaged and hold the clutch open instead.
    const int gear = std::clamp(ctl->gear, -1, car->forward_gears);
    const bool neutral = gear == 0;
    if (!neutral && gear != tr->GetCurrentGear()) tr->SetGear(gear);
    if (tr->GetCurrentGear() == 0) tr->SetGear(1);
    car->neutral = neutral;
    DriverInputs in;
    in.m_steering = std::clamp(-ctl->steer_angle / car->lock, -1.0, 1.0);
    in.m_throttle = std::clamp(ctl->throttle, 0.0, 1.0);
    in.m_braking = std::clamp(ctl->brake, 0.0, 1.0);
    in.m_clutch = neutral ? 1.0 : std::clamp(1.0 - ctl->clutch_engage, 0.0, 1.0);
    const double rear_brake = std::max(in.m_braking, std::clamp(ctl->handbrake, 0.0, 1.0));

    // At most 30 ms of physics per frame (~9 ms of work): after a slow frame (loading, a hitch)
    // the game briefly runs slow instead of catching up, which would make the next frame slower
    // still - at a 0.25 s cap one hitch locked the game at ~8 fps.
    car->accumulator = std::min(car->accumulator + dt, 0.03);
    int steps = 0;
    while (car->accumulator >= kStep) {
        const double t = sys->GetChTime();
        car->veh->Synchronize(t, in, car->terrain);
        if (rear_brake > in.m_braking) {
            auto axle = car->veh->GetAxle(1);
            if (axle->m_brake_left) axle->m_brake_left->Synchronize(t, rear_brake);
            if (axle->m_brake_right) axle->m_brake_right->Synchronize(t, rear_brake);
        }
        ApplyContacts(car);
        car->veh->Advance(kStep);
        for (int i = 0; i < 4; ++i) {
            auto sp = WheelAt(car, i)->GetSpindle();
            const ChVector3d left = car->veh->GetChassisBody()->GetRot().GetAxisY();
            car->spin[i] += Vdot(sp->GetAngVelParent() - car->veh->GetChassisBody()->GetAngVelParent(), left) * kStep;
        }
        car->accumulator -= kStep;
        ++steps;
    }
    car->last_us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
    return steps;
}

DC_API void dc_get_state(const dc_car* car, dc_state* s) {
    std::memset(s, 0, sizeof(*s));
    auto body = car->veh->GetChassisBody();
    const ChFrame<> ref = ChassisRef(car);
    const ChQuaternion<> q = ref.GetRot();
    Dc(ref.TransformPointLocalToParent(car->dc_origin), s->origin);
    Dc(-q.GetAxisY(), s->rot + 0);
    Dc(q.GetAxisZ(), s->rot + 3);
    Dc(-q.GetAxisX(), s->rot + 6);
    Dc(body->GetPosDt(), s->vel);
    Dc(body->GetAngVelParent(), s->ang_vel);
    const ChVector3d left = q.GetAxisY();
    for (int i = 0; i < 4; ++i) {
        auto wh = WheelAt(car, i);
        auto sp = wh->GetSpindle();
        dc_wheel& w = s->wheel[i];
        Dc(ref.TransformPointParentToLocal(sp->GetPos()) - car->dc_origin, w.center_local);
        const ChVector3d axle = q.RotateBack(sp->GetRot().GetAxisY());
        const double a = std::atan2(-axle.x(), axle.y());   // + = left in ISO
        w.steer = -a;
        w.omega = Vdot(sp->GetAngVelParent() - body->GetAngVelParent(), left);
        w.spin = car->spin[i];
        auto tire = wh->GetTire();
        if (tire) {
            const TerrainForce f = tire->ReportTireForce(const_cast<HostTerrain*>(&car->terrain));
            const ChVector3d up(0, 0, 1);
            ChVector3d fwd = Vcross(sp->GetRot().GetAxisY(), up);
            if (Vdot(fwd, q.GetAxisX()) < 0) fwd = -fwd;
            fwd.Normalize();
            const ChVector3d lat = Vcross(up, fwd);   // left
            w.fz = Vdot(f.force, up);
            w.fx = Vdot(f.force, fwd);
            w.fy = -Vdot(f.force, lat);
            w.contact = w.fz > 1.0;
            Dc(f.point, w.point);
            w.slip_ratio = tire->GetLongitudinalSlip();
            w.slip_angle = -tire->GetSlipAngle();
        }
    }
    s->rpm = car->veh->GetEngine()->GetMotorSpeed() * 60.0 / CH_2PI;
    s->gear = car->neutral ? 0 : car->veh->GetTransmission()->GetCurrentGear();
    // The actuator locks X only; its reaction is what holds the rack, so the road load is minus that
    // (checked by dc_smoke: steering right, the tyres push the rack back to the left).
    s->rack_force = car->rack ? -car->rack->GetReaction2().force.x() : 0.0;
    s->steer_lock = car->lock;
    s->sim_time = car->veh->GetSystem()->GetChTime();
}

DC_API double dc_last_step_us(const dc_car* car) { return car->last_us; }

}  // extern "C"
