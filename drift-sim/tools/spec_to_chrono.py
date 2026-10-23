"""Builds a Project Chrono vehicle (DriftChrono.dll) from a Drift Lab car spec made from published data -
no Assetto Corsa files. First car: the owner's 1998 Subaru Legacy Outback (BG9).

    python spec_to_chrono.py <Content/DriftData/Cars/<id>.json> <ChronoCars root>

Where the spec has no measured number, the geometry follows typical values for the suspension type
(MacPherson strut: KPI 13-15 deg, caster 3-4 deg, small positive scrub, ~15 mm trail - Gillespie,
Fundamentals of Vehicle Dynamics ch. 7; Reimpell, The Automotive Chassis). Every estimate is listed in
car.json "notes" so it can be replaced by a measurement later. Validate with dc_maneuvers against the
car's published road tests.
"""
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ac_to_chrono import G, tir, write   # noqa: E402  (shared tyre + writer)


def strut(half, radius, kpi_deg, caster_deg, camber_deg, wheel_rate, damp_wheel, sprung_load, hub_mass, steered):
    """MacPherson strut (Chrono template), left side, relative to the wheel centre (x fwd, y left, z up)."""
    kpi, cas = math.radians(kpi_deg), math.radians(caster_deg)
    lbj = [0.0, half - 0.075, -0.14]             # lower ball joint: 75 mm inboard, 140 mm below the hub

    def on_axis(z):   # point on the kingpin (strut) axis at height z
        dz = z - lbj[2]
        return [lbj[0] - dz * math.tan(cas), lbj[1] - dz * math.tan(kpi), z]

    top = on_axis(0.60)                           # strut top mount ~0.6 m above the hub
    low = on_axis(0.10)                           # strut lower mount on the upright
    mr = math.cos(kpi) * math.cos(cas)            # spring travel per wheel travel
    k = wheel_rate / (mr * mr)
    length = math.dist(top, low)
    free = length + sprung_load / mr / k          # preloaded to hold the car at the design height
    arm = 0.14 if steered else 0.12
    tierod_up = [-arm, half - 0.10, -0.04]
    tierod_ch = [-arm - 0.02, half - 0.40, -0.04]
    return {
        "Name": "Strut", "Type": "Suspension", "Template": "MacPhersonStrut",
        "Camber Angle (deg)": camber_deg, "Toe Angle (deg)": 0.0,
        "Spindle": {"Mass": 1.5, "COM": [0, half, 0], "Inertia": [0.003, 0.004, 0.003], "Radius": 0.1, "Width": 0.05},
        "Upright": {"Mass": max(hub_mass - 12.0, 5.0), "COM": [0, half - 0.04, 0], "Inertia": [0.05, 0.05, 0.03],
                    "Radius": 0.03},
        "Control Arm": {"Mass": 4.0, "COM": [0.02, half - 0.24, -0.145], "Inertia": [0.02, 0.05, 0.03], "Radius": 0.02,
                        "Location Chassis Front": [0.25, half - 0.42, -0.15],
                        "Location Chassis Back": [-0.12, half - 0.40, -0.14],
                        "Location Upright": lbj},
        "Strut": {"Mass": 3.0, "Inertia": [0.02, 0.02, 0.005], "Radius": 0.025},
        "Tierod": {"Location Chassis": tierod_ch, "Location Upright": tierod_up},
        "Spring": {"Location Chassis": top, "Location Upright": low, "Spring Coefficient": k, "Free Length": free},
        "Shock": {"Location Chassis": top, "Location Upright": low, "Damping Coefficient": damp_wheel / (mr * mr)},
        "Axle": {"Inertia": 0.01},
        "_kingpin": {"lbj": lbj, "top": top, "mr": mr},
    }


def build(spec_path, out_root):
    with open(spec_path, encoding="utf-8") as f:
        doc = json.load(f)
    sp, cid = doc["spec"], doc["id"]
    d = os.path.join(out_root, cid)
    os.makedirs(d, exist_ok=True)
    notes = {"estimates": []}
    est = notes["estimates"].append

    L, a = sp["wheelbase"], sp["com_from_front"]
    b = L - a
    rf, rr = sp["radius_f"], sp["radius_r"]
    total = sp["mass"]
    hub = {"FRONT": 35.0, "REAR": 30.0}   # knuckle + hub + brake + half strut/arm
    wheel_mass, tyre_mass = 8.0, 10.0
    est("unsprung corner 35/30 kg + 8 kg wheel + 10 kg tyre (typical 15-inch strut car)")
    chassis_mass = total - 2 * (hub["FRONT"] + hub["REAR"] + wheel_mass + tyre_mass)
    w, h = sp["width"], sp["height"]
    # Yaw inertia: m a b k with dynamic index k ~0.95 (Milliken RCVD 5.2); pitch scales with it; roll from the box.
    izz_total = total * a * b * sp.get("yaw_dynamic_index", 0.95)
    corners = sum(2 * (hub[s] + wheel_mass + tyre_mass) * (x * x + y * y)
                  for s, x, y in (("FRONT", a, sp["track_f"] / 2), ("REAR", b, sp["track_r"] / 2)))
    izz = max(izz_total - corners, chassis_mass * (w * w + L * L) / 12 * 0.5)
    iyy = izz * (h * h + L * L) / (w * w + L * L)
    inertia = [chassis_mass * (h * h + w * w) / 12 * 0.8, iyy, izz]
    est("yaw inertia from m*a*b*0.95 = %.0f kg m^2 (whole car)" % izz_total)
    write(os.path.join(d, "Chassis.json"), {
        "Name": sp["name"], "Type": "Chassis", "Template": "RigidChassis",
        "Components": [{"Centroidal Frame": {"Location": [0, 0, sp["cg_height"]], "Orientation": [1, 0, 0, 0]},
                        "Mass": chassis_mass, "Moments of Inertia": inertia, "Products of Inertia": [0, 0, 0],
                        "Void": False}],
        "Driver Position": {"Location": [0.0, 0.35, 1.0], "Orientation": [1, 0, 0, 0]},
        "Rear Connector Location": [-b - 1.0, 0, 0.2],
    })

    corner_load = {"FRONT": total * G * b / L / 2, "REAR": total * G * a / L / 2}
    geo = {"FRONT": dict(kpi=14.0, caster=3.5, camber=-0.5), "REAR": dict(kpi=8.0, caster=0.0, camber=-1.0)}
    est("strut geometry: front KPI 14, caster 3.5, camber -0.5; rear KPI 8, camber -1 (typical; measure)")
    axles, kingpin = [], None
    for side, pre, radius in (("FRONT", "f", rf), ("REAR", "r", rr)):
        half = sp["track_" + pre] / 2
        sprung = corner_load[side] - (hub[side] + wheel_mass + tyre_mass) * G
        damp = 0.5 * (sp["bump_" + pre] + sp["rebound_" + pre])
        s = strut(half, radius, geo[side]["kpi"], geo[side]["caster"], geo[side]["camber"], sp["spring_" + pre],
                  damp, sprung, hub[side], side == "FRONT")
        kp = s.pop("_kingpin")
        if side == "FRONT":
            kingpin = kp
        write(os.path.join(d, "Susp_%s.json" % side), s)
        # ARB: droplink on the control arm 0.25 m inboard of the ball joint; spec rate = N/m of wheel travel diff.
        drop_y = half - 0.25
        lever = (drop_y - (half - 0.41)) / ((half - 0.075) - (half - 0.41))   # droplink travel per wheel travel
        arm_len, drop = 0.2, 0.1
        write(os.path.join(d, "ARB_%s.json" % side), {
            "Name": "ARB", "Type": "Antirollbar", "Template": "AntirollBarRSD",
            "Arm": {"Mass": 1.0, "Inertia": [0.05, 0.05, 0.05], "Length": drop_y, "Width": arm_len, "Radius": 0.012},
            "Droplink": {"Height": drop},
            "RSD": {"Spring Coefficient": sp["arb_" + pre] * arm_len ** 2 / lever ** 2, "Damping Coefficient": 50},
        })
        st = sp["tire_" + pre]
        width = st.get("width_mm", 205) / 1000.0
        # All-season passenger tyre: load sensitivity ~-0.05 mu/kN, vertical rate ~220 kN/m (70-series).
        tsec = {"DY1": -0.05, "DX1": -0.05, "RATE": 220000, "DAMP": 500}
        notes[side] = tir(os.path.join(d, "Tire_%s.tir" % side), tsec, st, radius, width, tsec)
        wi = sp.get("wheel_inertia_" + pre, 1.2)
        write(os.path.join(d, "Tire_%s.json" % side), {
            "Name": "Pac02 %s" % side, "Type": "Tire", "Template": "Pac02Tire", "Mass": tyre_mass,
            "Inertia": [0.4, wi * 0.65, 0.4], "TIR Specification File": "%s/Tire_%s.tir" % (cid, side),
            "Coefficient of Friction": 0.8})
        write(os.path.join(d, "Wheel_%s.json" % side), {
            "Name": "Wheel", "Type": "Wheel", "Template": "Wheel", "Mass": wheel_mass, "Inertia": [0.3, wi * 0.35, 0.3]})
        brake = sp["brake_torque"] * (sp["brake_bias"] if side == "FRONT" else 1 - sp["brake_bias"]) * 0.5
        if side == "REAR":
            brake = max(brake, sp.get("handbrake_torque", 0.0) / 2)
        write(os.path.join(d, "Brake_%s.json" % side), {
            "Name": "Brake", "Type": "Brake", "Template": "BrakeSimple", "Maximum Torque": brake})
        x0 = a if side == "FRONT" else -b
        axles.append({
            "Suspension Input File": "%s/Susp_%s.json" % (cid, side), "Suspension Location": [x0, 0, radius],
            "Left Wheel Input File": "%s/Wheel_%s.json" % (cid, side),
            "Right Wheel Input File": "%s/Wheel_%s.json" % (cid, side),
            "Left Brake Input File": "%s/Brake_%s.json" % (cid, side),
            "Right Brake Input File": "%s/Brake_%s.json" % (cid, side),
            "Antirollbar Input File": "%s/ARB_%s.json" % (cid, side),
            "Antirollbar Location": [x0 - arm_len, 0, radius - 0.145 - drop],
        })
    axles[0]["Steering Index"] = 0

    lock = sp["steer_lock"]
    rack = 0.14 * math.sin(lock)
    write(os.path.join(d, "Steering.json"), {
        "Name": "Rack", "Type": "Steering", "Template": "RackPinion",
        "Steering Link": {"Mass": 2.0, "COM": 0, "Inertia": [0.1, 0.0001, 0.1], "Radius": 0.02,
                          "Length": 2 * (sp["track_f"] / 2 - 0.40)},
        "Pinion": {"Radius": 0.1, "Maximum Angle (deg)": math.degrees(rack / 0.1)},
    })

    awd = sp.get("drivetrain", "RWD") == "AWD"
    if awd:
        write(os.path.join(d, "Driveline.json"), {
            "Name": "AWD", "Type": "Driveline", "Template": "ShaftsDriveline4WD",
            "Shaft Direction": {"Motor Block": [1, 0, 0], "Axle": [0, 1, 0]},
            "Shaft Inertia": {"Driveshaft": 0.05, "Front Driveshaft": 0.02, "Rear Driveshaft": 0.03,
                              "Central Differential Box": 0.05, "Front Differential Box": 0.05,
                              "Rear Differential Box": 0.05},
            "Gear Ratio": {"Front Conical Gear": 1.0 / sp["final"], "Rear Conical Gear": 1.0 / sp["final"],
                           "Central Differential": -1.0, "Front Differential": -1.0, "Rear Differential": -1.0},
            "Axle Differential Locking Limit": 500.0, "Central Differential Locking Limit": 0.0,
        })
    else:
        write(os.path.join(d, "Driveline.json"), {
            "Name": "RWD", "Type": "Driveline", "Template": "ShaftsDriveline2WD",
            "Shaft Direction": {"Motor Block": [1, 0, 0], "Axle": [0, 1, 0]},
            "Shaft Inertia": {"Driveshaft": 0.05, "Differential Box": 0.1},
            "Gear Ratio": {"Conical Gear": 1.0 / sp["final"]},
            "Axle Differential Locking Limit": 500.0,
        })
    rpm, nm, lim = sp["curve_rpm"], sp["curve_nm"], sp["limiter_rpm"]
    write(os.path.join(d, "Engine.json"), {
        "Name": sp.get("engine_name", "engine"), "Type": "Engine", "Template": "EngineSimpleMap",
        "Maximal Engine Speed RPM": lim + 1500,
        "Map Full Throttle": [[-100, 0]] + [[r, t] for r, t in zip(rpm, nm)] + [[lim + 1500, nm[-1] * 0.5]],
        "Map Zero Throttle": [[-100, 0], [0, 0], [lim, -sp["coast_nm"]], [lim + 1500, -sp["coast_nm"] * (lim + 1500) / lim]],
    })
    write(os.path.join(d, "Transmission.json"), {
        "Name": sp.get("transmission_name", "gearbox"), "Type": "Transmission", "Template": "ManualTransmissionShafts",
        "Transmission Block Inertia": 0.1, "Input Shaft Inertia": 0.02,
        "Motorshaft Inertia": sp.get("engine_inertia", 0.15), "Driveshaft Inertia": 0.02,
        "Clutch Torque Limit": sp.get("clutch_nm", 400),
        "Gear Box": {"Reverse Gear Ratio": -1.0 / sp["reverse"], "Forward Gear Ratios": [1.0 / g for g in sp["gears"]]},
    })
    write(os.path.join(d, "Vehicle.json"), {
        "Name": sp["name"], "Type": "Vehicle", "Template": "WheeledVehicle",
        "Chassis": {"Input File": "%s/Chassis.json" % cid},
        "Axles": axles,
        "Steering Subsystems": [{"Input File": "%s/Steering.json" % cid,
                                 "Location": [a - 0.16, 0, rf - 0.04], "Orientation": [1, 0, 0, 0]}],
        "Wheelbase": L, "Minimum Turning Radius": 5.0, "Maximum Steering Angle (deg)": math.degrees(lock),
        "Driveline": {"Input File": "%s/Driveline.json" % cid, "Suspension Indexes": [0, 1] if awd else [1]},
    })
    open_rear = sp.get("diff_type", "open") == "open"
    steer_ratio = sp.get("steer_ratio", 16.5)
    est("steering ratio %.1f:1 (period Legacy figure)" % steer_ratio)
    write(os.path.join(d, "car.json"), {
        "id": cid, "engine": "%s/Engine.json" % cid, "transmission": "%s/Transmission.json" % cid,
        "vehicle": "%s/Vehicle.json" % cid, "tire_front": "%s/Tire_FRONT.json" % cid,
        "tire_rear": "%s/Tire_REAR.json" % cid,
        "steer_ratio": steer_ratio, "steer_lock": lock, "cda": sp.get("cda", 0.7),
        "limiter_rpm": lim, "limiter_hz": sp.get("limiter_hz", 0.0), "idle_rpm": sp["idle_rpm"],
        "handbrake_nm": sp.get("handbrake_torque", 0.0) / 2,
        "rear_brake_nm": sp["brake_torque"] * (1 - sp["brake_bias"]) * 0.5,
        "diff_type": sp.get("diff_type", "open"),
        "diff_power": 0.0 if open_rear else sp.get("diff_power", 0.0),
        "diff_coast": 0.0 if open_rear else sp.get("diff_coast", 0.0),
        "diff_preload_nm": 0.0 if open_rear else sp.get("diff_preload", 0.0),
        "final_drive": sp["final"],
        "drivetrain": "AWD" if awd else "RWD",
        # Viscous centre coupling: torque ~ c x driveshaft speed difference, capped (silicone fluid
        # couplings: ~2 N m per rpm of slip -> 19 N m s/rad; the spec's centre_lock_nm is the cap scale).
        "center_visc_nms": sp.get("center_visc_nms", 19.0),
        "center_cap_nm": sp.get("center_lock_nm", 180.0) * 3.0,
        # Kingpin (strut axis) points for the steering-torque FFB, left wheel, relative to the front wheel centre.
        "kingpin_lbj": kingpin["lbj"], "kingpin_top": kingpin["top"],
        "dc_origin": [(a - b) / 2, 0.0, (rf + rr) / 2],
        "notes": notes,
    })
    return d, notes


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    out, notes = build(sys.argv[1], sys.argv[2])
    print("wrote", out)
    print(json.dumps(notes["estimates"], indent=1))
