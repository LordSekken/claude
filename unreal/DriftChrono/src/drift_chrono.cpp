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
#include <unordered_map>
#include <vector>

#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChShaftBodyConstraint.h"
#include "chrono/physics/ChShaftsClutch.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/solver/ChIterativeSolverVI.h"
#include "chrono/solver/ChDirectSolverLS.h"
#include "chrono/solver/ChSolverADMM.h"
#include "chrono_vehicle/ChTerrain.h"
#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ChDoubleWishbone.h"
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

// The host's level as Chrono terrain.
//  - Cached: the ground is ray cast once per 5 cm grid cell (per 3 m height band, so bridges
//    work) and remembered - the track is static, and rays are the expensive part.
//  - Tyre envelope: Chrono's single-point contact sees a kerb edge as the ground jumping up
//    under the wheel in one step (a 23 kN kick that rolled the car). Instead the height under a
//    wheel is the highest ground point the round tyre would touch first, among samples up to
//    0.8 r away: h_i - (r - sqrt(r^2 - d_i^2)). The contact normal then points from that point
//    towards the wheel centre, so a kerb pushes the wheel back as well as up.
class HostTerrain : public ChTerrain {
  public:
    dc_raycast_fn fn = nullptr;
    void* ctx = nullptr;
    double radius = 0.3;
    static constexpr double kGrid = 0.05;
    static constexpr double kBand = 3.0;

    struct Cell {
        bool hit = false;
        double h = 0;         // ISO height
        ChVector3d n{0, 0, 1};
        double grip = 1.0;
    };

    void Clear() { cache_.clear(); }

    void GetProperties(const ChVector3d& loc, ChVector3d& point, double& height, ChVector3d& normal,
                       float& friction) const override {
        // Chrono queries at the tyre's lowest point + 2r, so the wheel centre is r below loc.
        // Pick the ground sample closest to the wheel centre; Chrono then measures the
        // penetration against the plane through that point facing the centre, which is
        // exactly r - distance (a plane through a point under the centre, tilted towards an
        // edge, would sit far too close and gave 45 kN spikes on a kerb).
        const double r = radius;
        const ChVector3d C(loc.x(), loc.y(), loc.z() - r);
        const int band = static_cast<int>(std::floor(loc.z() / kBand));
        double best = 1e9, grip = 1.0, best_h = -1e6;
        ChVector3d best_n(0, 0, 1), best_off(0, 0, 0);
        bool any = false;
        static const double kDirs[8][2] = {{1, 0}, {0.7071, 0.7071}, {0, 1}, {-0.7071, 0.7071},
                                           {-1, 0}, {-0.7071, -0.7071}, {0, -1}, {0.7071, -0.7071}};
        for (int ring = 0; ring <= 6; ++ring) {
            const double d = r * 0.8 * ring / 6.0;
            for (int k = 0; k < (ring == 0 ? 1 : 8); ++k) {
                const double ox = kDirs[k][0] * d, oy = kDirs[k][1] * d;
                Cell c = Bilinear(loc.x() + ox, loc.y() + oy, band);
                if (!c.hit) continue;
                const double below = std::max(0.0, C.z() - c.h);   // ground above the axle: touching side-on
                const double dist = std::sqrt(d * d + below * below);
                if (!any || dist < best - 1e-5) {
                    any = true;
                    best = dist;
                    best_h = c.h;
                    best_off = ChVector3d(ox, oy, 0);
                    best_n = c.n;
                    grip = c.grip;
                }
            }
        }
        if (!any) {
            point = ChVector3d(loc.x(), loc.y(), -1e6);
            normal = ChVector3d(0, 0, 1);
            height = -1e6;
            friction = 0.8f;
            return;
        }
        point = ChVector3d(loc.x() + best_off.x(), loc.y() + best_off.y(), best_h);
        if (best_off.Length2() > 1e-8 && best > 1e-6) {
            normal = (C - point) / best;   // from the touching point towards the wheel centre
        } else {
            normal = best_n;
        }
        height = best_h;
        // Pac02 scales grip by mu_road / 0.8 (its "Coefficient of Friction") and clamps mu to 0.1..1.
        friction = static_cast<float>(std::clamp(0.8 * grip, 0.1, 1.0));
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

    size_t CachedCells() const { return cache_.size(); }

  private:
    mutable std::unordered_map<uint64_t, Cell> cache_;

    // By value: a full cache is cleared, which would leave a caller's earlier reference dangling
    // (crashed after a few km of driving in one session).
    Cell CellAt(int ix, int iy, int band) const {
        const uint64_t key = (static_cast<uint64_t>(static_cast<uint32_t>(ix)) << 36) ^
                             (static_cast<uint64_t>(static_cast<uint32_t>(iy) & 0xFFFFFFFu) << 8) ^
                             static_cast<uint64_t>(static_cast<uint8_t>(band));
        auto it = cache_.find(key);
        if (it != cache_.end()) return it->second;
        if (cache_.size() > 2000000) cache_.clear();
        Cell c;
        if (fn) {
            const double top = (band + 1) * kBand + 1.0;
            double from[3], to[3], p[3], n[3], grip = 1.0;
            Dc(ChVector3d(ix * kGrid, iy * kGrid, top), from);
            Dc(ChVector3d(ix * kGrid, iy * kGrid, band * kBand - 4.0), to);
            if (fn(ctx, from, to, p, n, &grip)) {
                c.hit = true;
                c.h = Iso(p).z();
                c.n = Iso(n);
                c.grip = grip;
            }
        }
        return cache_.emplace(key, c).first->second;
    }

    // Heights blended between the four surrounding cells; normal/grip from the nearest.
    Cell Bilinear(double x, double y, int band) const {
        const double fx = x / kGrid, fy = y / kGrid;
        const int ix = static_cast<int>(std::floor(fx)), iy = static_cast<int>(std::floor(fy));
        const double tx = fx - ix, ty = fy - iy;
        const Cell cells[4] = {CellAt(ix, iy, band), CellAt(ix + 1, iy, band), CellAt(ix, iy + 1, band),
                               CellAt(ix + 1, iy + 1, band)};
        const Cell* c[4] = {&cells[0], &cells[1], &cells[2], &cells[3]};
        const double w[4] = {(1 - tx) * (1 - ty), tx * (1 - ty), (1 - tx) * ty, tx * ty};
        Cell out;
        double wsum = 0, best_w = -1;
        for (int k = 0; k < 4; ++k) {
            if (!c[k]->hit) continue;
            out.h += c[k]->h * w[k];
            wsum += w[k];
            if (w[k] > best_w) {
                best_w = w[k];
                out.n = c[k]->n;
                out.grip = c[k]->grip;
            }
        }
        if (wsum <= 1e-9) return out;
        out.h /= wsum;
        out.hit = true;
        return out;
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
    // Every body's design pose relative to the chassis reference frame: a reset restores the
    // suspension layout exactly (a crash could leave it bent, and the car wobbled after a reset).
    std::vector<std::pair<std::shared_ptr<ChBody>, ChFrame<>>> design;
    ChVector3d dc_origin;   // DriftCore origin in the chassis reference frame (ISO)
    double lock = 0.9;
    double mass = 1200;
    int forward_gears = 5;
    bool neutral = false;
    DriverInputs last_in{0, 0, 0, 0};
    bool have_last_in = false;
    double rack_avg = 0;
    double torque_avg = 0;
    double steer_ratio = 12.0;   // steering-wheel angle per road-wheel angle (nominal, on centre)
    double steer_input = 0;      // current Chrono steering input (-1..1, + left)
    std::vector<double> steer_gain;   // d(road wheel)/d(rim) vs input, 41 samples over -1..1

    double SteerGain(double u) const {
        if (steer_gain.size() < 2) return 1.0 / steer_ratio;
        const double x = std::clamp((u + 1.0) * 0.5, 0.0, 1.0) * (steer_gain.size() - 1);
        const size_t i = std::min(static_cast<size_t>(x), steer_gain.size() - 2);
        return steer_gain[i] + (steer_gain[i + 1] - steer_gain[i]) * (x - i);
    }
    double limiter_rpm = 1e9, limiter_hz = 0, cut_timer = 0, idle_rpm = 900;   // fuel-cut limiter, run per 1 ms step
    double rear_brake_nm = 0, handbrake_nm = 0;                // rear brakes double as the handbrake
    // Rear diff lock: Chrono's ShaftsDriveline2WD clutch between the axle shafts, created with modulation
    // 0 (= open diff) and a fixed limit. We set its limit per step like AC's LSD: preload + power/coast
    // fraction of the diff input torque; welded = a limit far above any tyre torque.
    std::shared_ptr<ChShaftsClutch> diff_lock;
    bool diff_welded = false;
    double diff_power = 0, diff_coast = 0, diff_preload = 0, final_drive = 4.0;
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

// Steering-column torque (N m, + = turns the rim right). Each front tyre's force and moment (at the
// wheel centre, normal load included) taken about that wheel's kingpin axis (lower -> upper ball
// joint), summed and divided by the steering ratio. Mechanical trail x Fy keeps it restoring past the
// tyres' peak, which is what winds the rim into countersteer.
double ColumnTorque(dc_car* c) {
    auto dwb = std::dynamic_pointer_cast<ChDoubleWishbone>(c->veh->GetAxle(0)->m_suspension);
    if (!dwb) return 0.0;
    double about_kingpins = 0.0;   // + = steers the wheels left (ISO, Z up)
    for (auto side : {LEFT, RIGHT}) {
        auto tire = c->veh->GetWheel(0, side)->GetTire();
        if (!tire) continue;
        const ChVector3d lbj = dwb->GetBallJointPosLCA(side);
        const ChVector3d k = (dwb->GetBallJointPosUCA(side) - lbj).GetNormalized();
        const TerrainForce f = tire->ReportTireForce(&c->terrain);   // force + moment at the wheel centre, global
        about_kingpins += Vdot(k, Vcross(f.point - lbj, f.force) + f.moment);
    }
    // d(road-wheel angle)/d(rim angle) at the current steering input: the rack geometry is non-linear
    // (BDC Miata: 8.4:1 on centre, 6.8:1 near lock), measured at load time.
    return -about_kingpins * c->SteerGain(c->steer_input);
}

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
    c->limiter_rpm = JsonNum(car, "limiter_rpm", 1e9);
    c->steer_ratio = std::max(1.0, JsonNum(car, "steer_ratio", 12.0));
    c->limiter_hz = JsonNum(car, "limiter_hz", 0.0);
    c->idle_rpm = JsonNum(car, "idle_rpm", 900.0);
    c->rear_brake_nm = JsonNum(car, "rear_brake_nm", 0.0);
    c->handbrake_nm = JsonNum(car, "handbrake_nm", 0.0);
    {
        const std::string type = car.find("\"diff_type\": \"welded\"") != std::string::npos ? "welded" : "";
        c->diff_welded = !type.empty();
        c->diff_power = JsonNum(car, "diff_power", 0.0);
        c->diff_coast = JsonNum(car, "diff_coast", 0.0);
        c->diff_preload = JsonNum(car, "diff_preload_nm", 0.0);
        c->final_drive = JsonNum(car, "final_drive", 4.0);
        ChShaft* left = c->veh->GetAxle(1)->m_suspension->GetAxle(LEFT).get();
        for (auto& item : c->veh->GetSystem()->GetOtherPhysicsItems())
            if (auto cl = std::dynamic_pointer_cast<ChShaftsClutch>(item))
                if (cl->GetShaft1() == left || cl->GetShaft2() == left) c->diff_lock = cl;
        if (c->diff_lock) c->diff_lock->SetModulation(1.0);
    }
    c->forward_gears =
        static_cast<int>(JsonVec(ReadFile(GetVehicleDataFile(JsonStr(car, "transmission"))), "Forward Gear Ratios").size());
    c->veh->GetChassis()->SetAerodynamicDrag(JsonNum(car, "cda", 0.7), 1.0, 1.2);
    const auto o = JsonVec(car, "dc_origin");
    c->dc_origin = o.size() == 3 ? ChVector3d(o[0], o[1], o[2]) : ChVector3d(0, 0, 0.3);
    c->mass = c->veh->GetMass();
    c->terrain.radius = WheelAt(c.get(), 0)->GetTire() ? WheelAt(c.get(), 0)->GetTire()->GetRadius() : 0.3;
    {
        const ChFrame<> ref_inv = ChassisRef(c.get()).GetInverse();
        for (auto& b : c->veh->GetSystem()->GetBodies())
            if (!b->IsFixed()) c->design.emplace_back(b, ref_inv * ChFrame<>(b->GetPos(), b->GetRot()));
    }

    ChSystem* sys = c->veh->GetSystem();
    sys->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    // Clutches/brakes are friction-limited constraints: they need a VI solver.
    const char* which = std::getenv("DC_SOLVER");
    const std::string solver_name = which ? which : "bb";
    if (solver_name == "admm") {
        auto admm = chrono_types::make_shared<ChSolverADMM>(chrono_types::make_shared<ChSolverSparseLU>());
        admm->SetMaxIterations(std::getenv("DC_ITERS") ? std::atoi(std::getenv("DC_ITERS")) : 30);
        sys->SetSolver(admm);
    } else if (solver_name == "lu") {
        sys->SetSolverType(ChSolver::Type::SPARSE_LU);
    } else {
        sys->SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
        auto solver = std::static_pointer_cast<ChIterativeSolverVI>(sys->GetSolver());
        solver->SetMaxIterations(std::getenv("DC_ITERS") ? std::atoi(std::getenv("DC_ITERS")) : 100);
        solver->SetOmega(0.8);
        solver->SetSharpnessLambda(1.0);
    }
    sys->SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    c->contact_acc = c->veh->GetChassisBody()->AddAccumulator();
    // Road-wheel angle vs steering input (exact position assembly at the design pose), for the
    // column torque's d(road wheel)/d(rim). Rim angle = input x lock x ratio (AC: linear rack).
    // Road-wheel angle vs steering input, for the column torque's d(road wheel)/d(rim). Measured on a
    // throwaway copy of the car (exact position assembly): assembling the real one at load time left
    // its state NaN. Rim angle = input x lock x ratio (AC: the rack moves linearly with the rim).
    if (!std::getenv("DC_NO_STEER_TABLE")) {
        WheeledVehicle probe(GetVehicleDataFile(JsonStr(car, "vehicle")), ChContactMethod::NSC);
        probe.Initialize(ChCoordsys<>(ChVector3d(0, 0, 0), QUNIT));
        ChSystem* ps = probe.GetSystem();
        ps->SetSolverType(ChSolver::Type::SPARSE_LU);
        ChTerrain flat;
        const int n = 41;
        std::vector<double> ang(n);
        for (int i = 0; i < n; ++i) {
            const double u = -1.0 + 2.0 * i / (n - 1);
            probe.Synchronize(0, DriverInputs{u, 0, 0, 0}, flat);
            ps->DoAssembly(AssemblyAnalysis::Level::POSITION);
            const ChQuaternion<> q = probe.GetChassisBody()->GetFrameRefToAbs().GetRot();
            double a = 0;
            for (auto side : {LEFT, RIGHT}) {
                const ChVector3d axle = q.RotateBack(probe.GetWheel(0, side)->GetSpindle()->GetRot().GetAxisY());
                a += 0.5 * std::atan2(-axle.x(), axle.y());
            }
            ang[i] = a;
        }
        const double rim_per_input = c->lock * c->steer_ratio;   // rad of rim per unit input
        c->steer_gain.assign(n, 1.0 / c->steer_ratio);
        for (int i = 0; i < n; ++i) {
            const int i0 = std::max(0, i - 1), i1 = std::min(n - 1, i + 1);
            const double du = 2.0 * (i1 - i0) / (n - 1);
            const double g = std::fabs((ang[i1] - ang[i0]) / du) / rim_per_input;
            if (std::isfinite(g) && g > 0) c->steer_gain[i] = g;
        }
        if (std::getenv("DC_VERBOSE"))
            std::printf("steering: lock %.1f deg, ratio on centre %.2f, at lock %.2f\n", ang[n - 1] * 57.2958,
                        1.0 / c->steer_gain[n / 2], 1.0 / c->steer_gain[n - 1]);
    }
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
    ChSystem* sys = car->veh->GetSystem();
    for (auto& [b, rel] : car->design) {
        const ChFrame<> f = target * rel;
        b->SetPos(f.GetPos());
        b->SetRot(f.GetRot());
        b->SetPosDt(VNULL);
        b->SetAngVelParent(VNULL);
    }
    for (auto& s : sys->GetShafts()) s->SetPosDt(0);
    sys->Update(UpdateFlags::UPDATE_ALL_NO_VISUAL);
    car->accumulator = 0;
    car->contacts.clear();
    car->have_last_in = false;
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
    car->steer_input = in.m_steering;
    in.m_throttle = std::clamp(ctl->throttle, 0.0, 1.0);
    in.m_braking = std::clamp(ctl->brake, 0.0, 1.0);
    in.m_clutch = neutral ? 1.0 : std::clamp(1.0 - ctl->clutch_engage, 0.0, 1.0);
    const double handbrake = std::clamp(ctl->handbrake, 0.0, 1.0);
    // The rear brakes are sized for the handbrake (max of service share and AC's HANDBRAKE_TORQUE);
    // the pedal only ever uses the service share of them.
    const double rear_max = std::max(car->rear_brake_nm, car->handbrake_nm);
    const double service_share = rear_max > 0 ? car->rear_brake_nm / rear_max : 1.0;

    // At most 30 ms of physics per frame (~9 ms of work): after a slow frame (loading, a hitch)
    // the game briefly runs slow instead of catching up, which would make the next frame slower
    // still - at a 0.25 s cap one hitch locked the game at ~8 fps.
    car->accumulator = std::min(car->accumulator + dt, 0.03);
    // The host's inputs change once per frame; ramp them across this frame's physics steps.
    // A steering step held by the position-driven rack is an impulse every frame (~470 N of
    // rack-force buzz in the owner's wheel at a steady 56 km/h).
    const DriverInputs target = in;
    const DriverInputs from = car->have_last_in ? car->last_in : target;
    const int frame_steps = std::max(1, static_cast<int>(car->accumulator / kStep));
    int steps = 0;
    double rack_sum = 0, torque_sum = 0;
    while (car->accumulator >= kStep) {
        const double t = sys->GetChTime();
        const double k = std::min(1.0, (steps + 1.0) / frame_steps);
        in.m_steering = from.m_steering + (target.m_steering - from.m_steering) * k;
        in.m_throttle = from.m_throttle + (target.m_throttle - from.m_throttle) * k;
        // Rev limiter at physics rate: AC-style fuel-cut pulses of 1/LIMITER_HZ (33 Hz on the BDC
        // Miata), never smeared across a frame. Chrono's own max speed is only a map clamp.
        const double rpm = car->veh->GetEngine()->GetMotorSpeed() * 60.0 / CH_2PI;
        if (rpm > car->limiter_rpm && car->cut_timer <= 0) car->cut_timer = car->limiter_hz > 1 ? 1.0 / car->limiter_hz : 0.06;
        if (car->cut_timer > 0) {
            in.m_throttle = 0;
            car->cut_timer -= kStep;
        }
        // Idle governor at physics rate (like VDrift / BeamNG): a throttle floor that rises as the engine
        // drops below idle, so it holds ~idle and picks itself back up instead of sitting at 0 rpm.
        if (car->cut_timer <= 0 && rpm < car->idle_rpm)
            in.m_throttle = std::max(in.m_throttle, std::clamp((car->idle_rpm - rpm) / 250.0, 0.0, 0.6));
        in.m_braking = from.m_braking + (target.m_braking - from.m_braking) * k;
        car->veh->Synchronize(t, in, car->terrain);
        if (car->diff_lock) {
            const double t_in = car->veh->GetPowertrainAssembly()->GetOutputTorque() * car->final_drive;
            const double f = t_in >= 0 ? car->diff_power : car->diff_coast;
            car->diff_lock->SetTorqueLimit(car->diff_welded ? 1e5 : car->diff_preload + f * std::fabs(t_in));
        }
        {
            const double rear = std::max(in.m_braking * service_share, handbrake);
            auto axle = car->veh->GetAxle(1);
            if (axle->m_brake_left) axle->m_brake_left->Synchronize(t, rear);
            if (axle->m_brake_right) axle->m_brake_right->Synchronize(t, rear);
        }
        ApplyContacts(car);
        car->veh->Advance(kStep);
        if (car->rack) rack_sum += -car->rack->GetReaction2().force.x();
        torque_sum += ColumnTorque(car);
        for (int i = 0; i < 4; ++i) {
            auto sp = WheelAt(car, i)->GetSpindle();
            const ChVector3d left = car->veh->GetChassisBody()->GetRot().GetAxisY();
            car->spin[i] += Vdot(sp->GetAngVelParent() - car->veh->GetChassisBody()->GetAngVelParent(), left) * kStep;
        }
        car->accumulator -= kStep;
        ++steps;
    }
    if (steps) car->rack_avg = rack_sum / steps;   // the frame's mean: what a wheel can render
    if (steps) car->torque_avg = torque_sum / steps;
    car->last_in = target;
    car->have_last_in = true;
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
    s->rack_force = car->rack_avg;
    s->steer_torque = car->torque_avg;
    s->steer_lock = car->lock;
    s->sim_time = car->veh->GetSystem()->GetChTime();
}

DC_API double dc_last_step_us(const dc_car* car) { return car->last_us; }

}  // extern "C"
