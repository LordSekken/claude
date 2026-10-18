"""Builds a Chrono::Vehicle car (JSON subsystems + Pacejka 2002 .tir tyres) from
an Assetto Corsa car's unpacked data/ folder and its Drift Lab spec JSON.

    python ac_to_chrono.py <ac car dir> <drift spec .json> <out root>

Writes <out root>/<id>/*.json. The output is derived from the car author's data,
so keep it out of git (like the AC car itself).

Frames: Chrono vehicle ISO (X forward, Y left, Z up). AC suspension hardpoints are
relative to the wheel pivot: x towards the car centre, y up, z forward.
"""
import json
import math
import os
import sys

TEMPLATE_TIR = os.path.join(os.path.dirname(__file__), "chrono_pac02_template.tir")
G = 9.81


class Ini(dict):
    def has_section(self, name):
        return name in self


def read_ini(path):
    """AC-style INI: [SECTION] / KEY=VALUE, ';' comments; stray lines are ignored."""
    ini = Ini()
    sec = None
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.split(";")[0].strip()
            if line.startswith("[") and "]" in line:
                sec = ini.setdefault(line[1:line.index("]")].strip(), {})
            elif sec is not None and "=" in line:
                k, v = line.split("=", 1)
                sec[k.strip()] = v.strip()
    return ini


def num(sec, key, default=0.0):
    try:
        return float(str(sec.get(key, default)).split()[0])
    except (ValueError, IndexError):
        return default


def vec(sec, key):
    return [float(v) for v in sec[key].split(",")]


def tyre_section(tyres, axle):
    idx = int(num(tyres["COMPOUND_DEFAULT"], "INDEX", 0)) if tyres.has_section("COMPOUND_DEFAULT") else 0
    name = axle if idx == 0 else "%s_%d" % (axle, idx)
    return tyres[name] if tyres.has_section(name) else tyres[axle]


def write(path, obj):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(obj, f, indent=2)


def wishbone(susp, side, half_track, radius, wheel_rate, bump, rebound, hub_mass, wheel_mass, tyre_mass):
    """Chrono DoubleWishbone (left side, relative to the axle centre at wheel-centre height)."""
    s = susp[side]

    def p(key):
        x, y, z = vec(s, key)
        return [z, half_track - x, y]

    lca_out = p("WBTYRE_BOTTOM")
    lca_in = [(a + b) / 2 for a, b in zip(p("WBCAR_BOTTOM_FRONT"), p("WBCAR_BOTTOM_REAR"))]
    # Spring/damper straight above the lower ball joint: motion ratio = lever ratio of the lower arm.
    mr = max(0.3, min(1.0, (lca_out[1] - lca_in[1]) / max(0.05, half_track - lca_in[1])))
    spring_arm = list(lca_out)
    spring_chassis = [lca_out[0], lca_out[1] - 0.02, lca_out[2] + 0.35]
    k = wheel_rate / (mr * mr)
    upright_mass = 8.0
    arm_mass = 1.5
    spindle_mass = max(2.0, hub_mass - wheel_mass - tyre_mass - upright_mass - 2 * arm_mass)
    return {
        "Name": "AC %s DoubleWishbone" % side, "Type": "Suspension", "Template": "DoubleWishbone",
        "Camber Angle (deg)": num(s, "STATIC_CAMBER"),
        "Toe Angle (deg)": 0.0,
        "Spindle": {"Mass": spindle_mass, "COM": [0, half_track, 0], "Inertia": [0.05, 0.08, 0.05],
                    "Radius": 0.1, "Width": 0.02},
        "Upright": {"Mass": upright_mass, "COM": [0, half_track - 0.08, 0],
                    "Moments of Inertia": [0.05, 0.05, 0.02], "Products of Inertia": [0, 0, 0], "Radius": 0.025},
        "Upper Control Arm": {"Mass": arm_mass, "COM": p("WBTYRE_TOP"),
                              "Moments of Inertia": [0.01, 0.005, 0.01], "Products of Inertia": [0, 0, 0],
                              "Radius": 0.02, "Location Chassis Front": p("WBCAR_TOP_FRONT"),
                              "Location Chassis Back": p("WBCAR_TOP_REAR"), "Location Upright": p("WBTYRE_TOP")},
        "Lower Control Arm": {"Mass": arm_mass, "COM": lca_out,
                              "Moments of Inertia": [0.01, 0.005, 0.01], "Products of Inertia": [0, 0, 0],
                              "Radius": 0.03, "Location Chassis Front": p("WBCAR_BOTTOM_FRONT"),
                              "Location Chassis Back": p("WBCAR_BOTTOM_REAR"), "Location Upright": lca_out},
        "Tierod": {"Location Chassis": p("WBCAR_STEER"), "Location Upright": p("WBTYRE_STEER")},
        # Free length is set by build_car once the corner load is known.
        "Spring": {"Location Chassis": spring_chassis, "Location Arm": spring_arm,
                   "Free Length": 0.35, "Spring Coefficient": k},
        "Shock": {"Location Chassis": spring_chassis, "Location Arm": spring_arm,
                  "Damping Coefficient": (bump + rebound) / 2 / (mr * mr)},
        "Axle": {"Inertia": 0.4},
        "_motion_ratio": mr,
    }


def tir(path, t, spec_t, radius, width, ac_t):
    """Pacejka 2002 tyre scaled to AC's grip: mu(Fz) = DY0 + DY1*Fz[kN] at FNOMIN = 3.5 kN,
    peak slip angle / ratio from the Drift Lab spec, falloff from the spec tail."""
    fnom = 3500.0
    mu_y = spec_t["mu_lat"]
    mu_x = spec_t["mu_long"]
    ac_mu_y = num(ac_t, "DY0", 1.2) + num(ac_t, "DY1", 0.0) * 3.5
    ac_mu_x = num(ac_t, "DX0", 1.2) + num(ac_t, "DX1", 0.0) * 3.5
    pdy2 = num(ac_t, "DY1", 0.0) * 3.5 * mu_y / ac_mu_y
    pdx2 = num(ac_t, "DX1", 0.0) * 3.5 * mu_x / ac_mu_x
    tail = max(0.5, min(0.98, spec_t.get("tail", 0.8)))
    cy = 2.0 - (2.0 / math.pi) * math.asin(tail)          # sin(C*pi/2) = tail at large slip
    by = math.tan(math.pi / (2 * cy)) / math.radians(spec_t["peak_slip_angle_deg"])
    pky2 = 2.0
    pky1 = -by * cy * mu_y / math.sin(2 * math.atan(1.0 / pky2))   # Chrono: Ky = PKY1*FNOMIN*sin(..)*LMUY
    cx = 1.65
    bx = math.tan(math.pi / (2 * cx)) / spec_t["peak_slip_ratio"]
    vals = {
        "UNLOADED_RADIUS": radius, "WIDTH": width, "ASPECT_RATIO": 0.45, "RIM_RADIUS": radius - 0.09,
        "RIM_WIDTH": width - 0.02, "VERTICAL_STIFFNESS": num(ac_t, "RATE", 250000),
        "VERTICAL_DAMPING": num(ac_t, "DAMP", 500), "FNOMIN": fnom, "LFZO": 1.0,
        "PCY1": cy, "PDY1": mu_y, "PDY2": pdy2, "PDY3": 0.0, "PEY1": 0.0, "PEY2": 0.0, "PEY3": 0.0,
        "PEY4": 0.0, "PKY1": pky1, "PKY2": pky2, "PKY3": 0.0, "PHY1": 0.0, "PHY2": 0.0,
        "PHY3": num(ac_t, "CAMBER_GAIN", 0.0) * 0.1, "PVY1": 0.0, "PVY2": 0.0, "PVY3": 0.0, "PVY4": 0.0,
        "PCX1": cx, "PDX1": mu_x, "PDX2": pdx2, "PEX1": 0.0, "PEX2": 0.0, "PEX3": 0.0, "PEX4": 0.0,
        "PKX1": bx * cx * mu_x, "PKX2": 0.0, "PKX3": 0.0, "PHX1": 0.0, "PHX2": 0.0, "PVX1": 0.0, "PVX2": 0.0,
        "LMUX": 1.0, "LMUY": 1.0, "LKX": 1.0, "LKY": 1.0,
    }
    out = []
    with open(TEMPLATE_TIR, encoding="utf-8") as f:
        for line in f:
            key = line.split("=")[0].strip()
            if "=" in line and key in vals and not line.startswith(("$", "!")):
                line = "%-25s= %.6g\n" % (key, vals[key])
            out.append(line)
    with open(path, "w", encoding="utf-8") as f:
        f.writelines(out)
    return {"PKY1": pky1, "PCY1": cy, "PKX1": vals["PKX1"]}


def build_car(ac_dir, spec_path, out_root):
    data = os.path.join(ac_dir, "data")
    susp = read_ini(os.path.join(data, "suspensions.ini"))
    tyres = read_ini(os.path.join(data, "tyres.ini"))
    with open(spec_path, encoding="utf-8") as f:
        spec_doc = json.load(f)
    sp = spec_doc["spec"]
    cid = spec_doc["id"]
    d = os.path.join(out_root, cid)
    os.makedirs(d, exist_ok=True)

    L = sp["wheelbase"]
    a = sp["com_from_front"]               # CG behind the front axle
    b = L - a
    rf, rr = sp["radius_f"], sp["radius_r"]
    tf, tr = tyre_section(tyres, "FRONT"), tyre_section(tyres, "REAR")
    hub_f, hub_r = num(susp["FRONT"], "HUB_MASS", 40), num(susp["REAR"], "HUB_MASS", 40)
    wheel_mass, tyre_mass = 8.0, 9.0
    total = sp["mass"]
    chassis_mass = total - 2 * hub_f - 2 * hub_r
    m, w, h, l = total, sp["width"], sp["height"], sp["length"]
    inertia = [chassis_mass * (h * h + w * w) / 12, chassis_mass * (h * h + l * l) / 12,
               chassis_mass * (w * w + l * l) / 12 * sp.get("yaw_inertia_scale", 1.0)]

    # Reference frame: origin on the ground under the CG.
    write(os.path.join(d, "Chassis.json"), {
        "Name": "AC chassis", "Type": "Chassis", "Template": "RigidChassis",
        "Components": [{"Centroidal Frame": {"Location": [0, 0, sp["cg_height"]], "Orientation": [1, 0, 0, 0]},
                        "Mass": chassis_mass, "Moments of Inertia": inertia, "Products of Inertia": [0, 0, 0],
                        "Void": False}],
        "Driver Position": {"Location": [sp["eye"][2] * -1.0 if "eye" in sp else 0.0, 0.35, 1.0],
                            "Orientation": [1, 0, 0, 0]},
        "Rear Connector Location": [-b - 1.0, 0, 0.2],
    })

    axles = []
    notes = {}
    corner_load = {"FRONT": total * G * b / L / 2, "REAR": total * G * a / L / 2}
    for side, half, radius, hub, pre, tsec, st in (
            ("FRONT", sp["track_f"] / 2, rf, hub_f, "f", tf, sp["tire_f"]),
            ("REAR", sp["track_r"] / 2, rr, hub_r, "r", tr, sp["tire_r"])):
        s = wishbone(susp, side, half, radius, sp["spring_" + pre], sp["bump_" + pre], sp["rebound_" + pre],
                     hub, wheel_mass, tyre_mass)
        mr = s.pop("_motion_ratio")
        sprung = corner_load[side] - hub * G
        length0 = math.dist(s["Spring"]["Location Chassis"], s["Spring"]["Location Arm"])
        s["Spring"]["Free Length"] = length0 + sprung / mr / s["Spring"]["Spring Coefficient"]
        write(os.path.join(d, "Susp_%s.json" % side), s)
        arb = sp["arb_" + pre]
        arm = 0.25
        write(os.path.join(d, "ARB_%s.json" % side), {
            "Name": "AC ARB", "Type": "Antirollbar", "Template": "AntirollBarRSD",
            "Arm": {"Mass": 1.0, "Inertia": [0.05, 0.05, 0.05], "Length": arm, "Width": half - 0.25, "Radius": 0.015},
            "Droplink": {"Height": -0.04},
            # AC ARB is a wheel rate (N/m of left-right travel difference).
            "RSD": {"Spring Coefficient": arb * arm * arm / (mr * mr), "Damping Coefficient": 50},
        })
        width = st.get("width_mm", 205) / 1000.0
        notes[side] = tir(os.path.join(d, "Tire_%s.tir" % side), tsec, st, radius, width, tsec)
        notes[side]["motion_ratio"] = mr
        inertia_total = num(tsec, "ANGULAR_INERTIA", 1.2)
        write(os.path.join(d, "Tire_%s.json" % side), {
            "Name": "AC Pac02 %s" % side, "Type": "Tire", "Template": "Pac02Tire", "Mass": tyre_mass,
            "Inertia": [0.4, inertia_total * 0.6, 0.4], "TIR Specification File": "%s/Tire_%s.tir" % (cid, side),
            "Coefficient of Friction": 0.8})
        write(os.path.join(d, "Wheel_%s.json" % side), {
            "Name": "AC wheel", "Type": "Wheel", "Template": "Wheel", "Mass": wheel_mass,
            "Inertia": [0.3, inertia_total * 0.4, 0.3]})
        brake = sp["brake_torque"] * (sp["brake_bias"] if side == "FRONT" else 1 - sp["brake_bias"]) * 0.5
        write(os.path.join(d, "Brake_%s.json" % side), {
            "Name": "AC brake", "Type": "Brake", "Template": "BrakeSimple", "Maximum Torque": brake})
        axles.append({
            "Suspension Input File": "%s/Susp_%s.json" % (cid, side),
            "Suspension Location": [a if side == "FRONT" else -b, 0, radius],
            "Left Wheel Input File": "%s/Wheel_%s.json" % (cid, side),
            "Right Wheel Input File": "%s/Wheel_%s.json" % (cid, side),
            "Left Brake Input File": "%s/Brake_%s.json" % (cid, side),
            "Right Brake Input File": "%s/Brake_%s.json" % (cid, side),
            "Antirollbar Input File": "%s/ARB_%s.json" % (cid, side),
            "Antirollbar Location": [(a if side == "FRONT" else -b) - 0.25, 0, radius - 0.05],
        })
    axles[0]["Steering Index"] = 0

    lock_deg = math.degrees(sp["steer_lock"])
    sf = susp["FRONT"]
    steer_arm = math.hypot(vec(sf, "WBTYRE_STEER")[2], vec(sf, "WBTYRE_STEER")[0] - vec(sf, "WBTYRE_BOTTOM")[0])
    write(os.path.join(d, "Steering.json"), {
        "Name": "AC rack", "Type": "Steering", "Template": "RackPinion",
        "Steering Link": {"Mass": 2.0, "COM": 0, "Inertia": [0.1, 0.0001, 0.1], "Radius": 0.02, "Length": 0.6},
        # Rack travel = steering arm * sin(lock); the prototype reports the real road-wheel angle.
        "Pinion": {"Radius": 0.1, "Maximum Angle (deg)": math.degrees(steer_arm * math.sin(sp["steer_lock"]) / 0.1)},
    })
    write(os.path.join(d, "Driveline.json"), {
        "Name": "AC RWD", "Type": "Driveline", "Template": "ShaftsDriveline2WD",
        "Shaft Direction": {"Motor Block": [1, 0, 0], "Axle": [0, 1, 0]},
        "Shaft Inertia": {"Driveshaft": 0.05, "Differential Box": 0.1},
        "Gear Ratio": {"Conical Gear": 1.0 / sp["final"]},
        "Axle Differential Locking Limit": 1.0e5 if sp.get("diff_type") == "welded" else 500.0,
    })
    rpm = sp["curve_rpm"]
    nm = sp["curve_nm"]
    write(os.path.join(d, "Engine.json"), {
        "Name": "AC engine", "Type": "Engine", "Template": "EngineSimpleMap",
        "Maximal Engine Speed RPM": sp["limiter_rpm"],
        "Map Full Throttle": [[-100, 0]] + [[r, t] for r, t in zip(rpm, nm)],
        "Map Zero Throttle": [[-100, 0], [0, 0], [sp["idle_rpm"], 0],
                              [sp["limiter_rpm"], -sp["coast_nm"]], [sp["limiter_rpm"] * 1.2, -sp["coast_nm"] * 1.5]],
    })
    write(os.path.join(d, "Transmission.json"), {
        "Name": "AC gearbox", "Type": "Transmission", "Template": "ManualTransmissionShafts",
        "Transmission Block Inertia": 0.1, "Input Shaft Inertia": 0.02,
        "Motorshaft Inertia": sp.get("engine_inertia", 0.15), "Driveshaft Inertia": 0.02,
        "Clutch Torque Limit": sp.get("clutch_nm", 400),
        "Gear Box": {"Reverse Gear Ratio": -1.0 / sp["reverse"],
                     "Forward Gear Ratios": [1.0 / g for g in sp["gears"]]},
    })
    write(os.path.join(d, "Vehicle.json"), {
        "Name": sp["name"], "Type": "Vehicle", "Template": "WheeledVehicle",
        "Chassis": {"Input File": "%s/Chassis.json" % cid},
        "Axles": axles,
        "Steering Subsystems": [{"Input File": "%s/Steering.json" % cid,
                                 "Location": [a - 0.08, 0, rf - 0.11], "Orientation": [1, 0, 0, 0]}],
        "Wheelbase": L, "Minimum Turning Radius": 5.0,
        "Maximum Steering Angle (deg)": lock_deg,
        "Driveline": {"Input File": "%s/Driveline.json" % cid, "Suspension Indexes": [1]},
    })
    write(os.path.join(d, "car.json"), {"id": cid, "engine": "%s/Engine.json" % cid,
                                        "transmission": "%s/Transmission.json" % cid,
                                        "vehicle": "%s/Vehicle.json" % cid,
                                        "tire_front": "%s/Tire_FRONT.json" % cid,
                                        "tire_rear": "%s/Tire_REAR.json" % cid,
                                        "steer_ratio": sp.get("model", {}).get("steer_ratio", 12.0),
                                        "steer_lock": sp["steer_lock"], "cda": sp.get("cda", 0.7),
                                        # DriftCore's body origin (mid-axles, wheel-centre height) in the
                                        # chassis reference frame, so models/cameras line up.
                                        "dc_origin": [(a - b) / 2, 0.0, (rf + rr) / 2],
                                        "notes": notes})
    return d, notes


def calibrate_lock(out_dir, chrono_lab, target_deg):
    """Rack geometry is non-linear: secant-search the pinion angle until chrono_lab measures AC's lock."""
    import re
    import subprocess
    path = os.path.join(out_dir, "Steering.json")
    root, cid = os.path.split(out_dir)

    def measure(angle):
        with open(path, encoding="utf-8") as f:
            d = json.load(f)
        d["Pinion"]["Maximum Angle (deg)"] = angle
        write(path, d)
        out = subprocess.run([chrono_lab, root, cid, "lock"], capture_output=True, text=True).stdout
        return float(re.search(r"full lock: ([\d.]+) deg left", out).group(1))

    with open(path, encoding="utf-8") as f:
        a0 = json.load(f)["Pinion"]["Maximum Angle (deg)"]
    m0 = measure(a0)
    a1 = a0 * target_deg / m0
    for _ in range(6):
        m1 = measure(a1)
        if abs(m1 - target_deg) < 0.2 or m1 == m0:
            break
        a0, m0, a1 = a1, m1, a1 + (target_deg - m1) * (a1 - a0) / (m1 - m0)
    print("steering lock %.1f deg (target %.1f) with pinion %.2f deg" % (m1, target_deg, a1))


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--calibrate=")]
    lab = [a.split("=", 1)[1] for a in sys.argv[1:] if a.startswith("--calibrate=")]
    if len(args) != 3:
        sys.exit(__doc__ + "\n    optional: --calibrate=<chrono_lab.exe> to match AC's steering lock")
    out, notes = build_car(*args)
    print("wrote", out)
    print(json.dumps(notes, indent=1))
    if lab:
        with open(args[1], encoding="utf-8") as f:
            calibrate_lock(out, lab[0], math.degrees(json.load(f)["spec"]["steer_lock"]))
