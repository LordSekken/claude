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


def wishbone(susp, side, half_track, radius, wheel_rate, bump, rebound, hub_mass, wheel_mass, tyre_mass,
             sprung_load=None):
    """Chrono DoubleWishbone (left side, relative to the axle centre at wheel-centre height).

    AC's springs are wheel rates with zero force at the hardpoint ("design") position, so AC's car rests
    sprung_load / SPRING_RATE into bump (its telemetry suspensionTravel = BUMPSTOP_DN + that). Chrono's
    JSON pose is the pose the car is built and reset in, so the AC linkage is solved at that rest travel and
    written in the rest pose (camber/toe/roll centre/bump steer at rest then match AC). The spring and
    damper sit on the lower ball joint and get force curves through the solved motion ratio, so the wheel
    sees AC's wheel rate, bump stops and slow/fast bump/rebound damping. Kinematics checked by
    unreal/ChronoLab/kinematics against an independent solve (C:/Dev/kin_analysis/ac_dwb.py).
    """
    s = susp[side]

    def p(key):
        x, y, z = vec(s, key)
        return [z, half_track - x, y]

    # --- AC linkage solve: upright = rigid body through both ball joints, steering-rod point, wheel centre.
    def sub3(u, v): return [u[0] - v[0], u[1] - v[1], u[2] - v[2]]
    def cross3(u, v): return [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]

    def rot(r, q):   # Rodrigues: rotate q by rotation vector r
        th = math.sqrt(sum(c * c for c in r))
        if th < 1e-14:
            return [q[i] + cross3(r, q)[i] for i in range(3)]
        k = [c / th for c in r]
        kq, kd = cross3(k, q), sum(k[i] * q[i] for i in range(3))
        return [q[i] * math.cos(th) + kq[i] * math.sin(th) + k[i] * kd * (1 - math.cos(th)) for i in range(3)]

    wc0 = [0.0, half_track, 0.0]
    lca_f, lca_b, uca_f, uca_b = p("WBCAR_BOTTOM_FRONT"), p("WBCAR_BOTTOM_REAR"), p("WBCAR_TOP_FRONT"), p("WBCAR_TOP_REAR")
    lbj0, ubj0, tru0, trc = p("WBTYRE_BOTTOM"), p("WBTYRE_TOP"), p("WBTYRE_STEER"), p("WBCAR_STEER")
    links = [(lbj0, lca_f, math.dist(lbj0, lca_f)), (lbj0, lca_b, math.dist(lbj0, lca_b)),
             (ubj0, uca_f, math.dist(ubj0, uca_f)), (ubj0, uca_b, math.dist(ubj0, uca_b)),
             # AC TOE_OUT changes the steering-rod length (m).
             (tru0, trc, math.dist(tru0, trc) + num(s, "TOE_OUT"))]
    state = [0.0] * 6   # wheel-centre translation, upright rotation vector (about the design wheel centre)

    def place(x, q):
        return [a + b + c for a, b, c in zip(rot(x[3:], sub3(q, wc0)), wc0, x[:3])]

    def solve(z):
        """Upright pose with the wheel centre z above design (bump +)."""
        def f(x):
            return [math.dist(place(x, q), c) - l for q, c, l in links] + [x[2] - z]
        x = list(state)
        for _ in range(40):
            f0 = f(x)
            if max(abs(v) for v in f0) < 1e-12:
                break
            cols = []
            for j in range(6):
                xp = list(x)
                xp[j] += 1e-7
                cols.append([(u - v) / 1e-7 for u, v in zip(f(xp), f0)])
            m = [[cols[j][i] for j in range(6)] + [-f0[i]] for i in range(6)]
            for c in range(6):   # Gauss-Jordan with partial pivoting
                piv = max(range(c, 6), key=lambda r: abs(m[r][c]))
                m[c], m[piv] = m[piv], m[c]
                for r in range(6):
                    if r != c:
                        g = m[r][c] / m[c][c]
                        m[r] = [u - g * v for u, v in zip(m[r], m[c])]
            x = [x[i] + m[i][6] / m[i][i] for i in range(6)]
        state[:] = x
        return x

    k_w = wheel_rate
    sag = (sprung_load / k_w) if sprung_load else 0.0
    rest = solve(sag)
    off = [rest[0], rest[1], sag]     # rest wheel centre - design wheel centre: re-centre on it
    sh = lambda q: sub3(q, off)                      # chassis-fixed point in the rest-centred frame
    up = lambda x, q: sub3(place(x, q), off)         # upright point in the rest-centred frame
    cam0 = math.radians(num(s, "STATIC_CAMBER"))
    spin = rot(rest[3:], [0.0, math.cos(cam0), -math.sin(cam0)])   # spin axis (outboard), static camber on the hub
    camber = -math.degrees(math.asin(spin[2]))                       # + = top out
    toe_in = -math.degrees(math.atan2(-spin[0], spin[1]))            # Chrono: + = toe-in
    lbj, ubj, tru = up(rest, lbj0), up(rest, ubj0), up(rest, tru0)

    # Spring and damper straight above the lower ball joint (on the LCA). Motion ratio and spring length
    # vs wheel travel from the linkage, so the wheel rate is AC's along the whole travel.
    spring_arm = list(lbj)
    spring_chassis = [lbj[0], lbj[1] - 0.02, lbj[2] + 0.35]
    up_stop, dn_stop = num(s, "BUMPSTOP_UP", 1.0), num(s, "BUMPSTOP_DN", 1.0)
    k_stop, k_prog = num(s, "BUMP_STOP_RATE", 0.0), num(s, "PROGRESSIVE_SPRING_RATE", 0.0)

    def wheel_force(zd):   # AC wheel force at travel zd from design (+ = pushes the wheel down)
        f = k_w * zd + k_prog * zd * abs(zd)
        if zd > up_stop:
            f += k_stop * (zd - up_stop)
        if zd < -dn_stop:
            f += k_stop * (zd + dn_stop)
        return f

    state[:] = rest
    samples = []
    zs = sorted(set([round(i * 0.01, 4) for i in range(-15, 16)] + [round(up_stop - sag, 4), round(-dn_stop - sag, 4)]))
    for zw in [z for z in zs if -0.15 <= z <= 0.15]:   # travel from rest
        h = 1e-4
        lengths = []
        for dz in (-h, 0.0, h):
            state[:] = rest
            for k in range(1, 6):   # walk out so the solver stays on the same branch
                x = solve(sag + (zw + dz) * k / 5)
            lengths.append(math.dist(spring_chassis, up(x, lbj0)))
        mr_z = -(lengths[2] - lengths[0]) / (2 * h)
        samples.append((lengths[1], wheel_force(zw + sag) / mr_z, mr_z))
    state[:] = rest
    l_rest = math.dist(spring_chassis, spring_arm)
    mr = [m for l, f, m in samples if abs(l - l_rest) < 1e-9][0]
    spring_curve = sorted([round(l - l_rest, 6), -f] for l, f, m in samples)   # Chrono: F = -curve(L - L0)

    # AC dampers: wheel N s/m, slow below / fast above the threshold (m/s), bump vs rebound.
    def wheel_damp(v):   # v = wheel speed, + = bump; returns the resisting force magnitude with sign of v
        c_s, c_f, thr = ((bump, num(s, "DAMP_FAST_BUMP", bump), num(s, "DAMP_FAST_BUMPTHRESHOLD", 1e3)) if v > 0 else
                         (rebound, num(s, "DAMP_FAST_REBOUND", rebound), num(s, "DAMP_FAST_REBOUNDTHRESHOLD", 1e3)))
        a = abs(v)
        return math.copysign(c_s * min(a, thr) + c_f * max(0.0, a - thr), v)
    vws = sorted(set([0.0] + [sgn * x for sgn in (-1, 1) for x in (0.02, 0.05, num(s, "DAMP_FAST_BUMPTHRESHOLD", 0.1),
                                                                 num(s, "DAMP_FAST_REBOUNDTHRESHOLD", 0.1), 0.3, 1.0, 3.0)]))
    # Shock speed vs = -mr * vw; Chrono damper force = -curve(vs), + = pushes apart (= wheel down).
    damp_curve = sorted([round(-mr * v, 6), -wheel_damp(v) / mr] for v in vws)

    # Anti-roll bar droplink point: 60% out along the LCA from its inner axis (moves 0.6 x the ball joint).
    lca_in = [(u + v) / 2 for u, v in zip(sh(lca_f), sh(lca_b))]
    arb_point = [a + 0.6 * (b - a) for a, b in zip(lca_in, lbj)]
    state[:] = rest
    za, zb = solve(sag - 0.002), solve(sag + 0.002)
    lbj_mr = (place(zb, lbj0)[2] - place(za, lbj0)[2]) / 0.004   # ball-joint vertical / wheel vertical
    state[:] = rest

    upright_mass = 8.0
    arm_mass = 1.5
    spindle_mass = max(2.0, hub_mass - wheel_mass - tyre_mass - upright_mass - 2 * arm_mass)
    return {
        "Name": "AC %s DoubleWishbone" % side, "Type": "Suspension", "Template": "DoubleWishbone",
        # Wheel camber/toe at AC's rest position (static camber on the hub + linkage at the rest travel).
        "Camber Angle (deg)": camber,
        "Toe Angle (deg)": toe_in,
        "Spindle": {"Mass": spindle_mass, "COM": [0, half_track, 0], "Inertia": [0.05, 0.01, 0.05],   # spin inertia is the wheel+tyre's (AC ANGULAR_INERTIA)
                    "Radius": 0.1, "Width": 0.02},
        "Upright": {"Mass": upright_mass, "COM": [0, half_track - 0.08, 0],
                    "Moments of Inertia": [0.05, 0.05, 0.02], "Products of Inertia": [0, 0, 0], "Radius": 0.025},
        "Upper Control Arm": {"Mass": arm_mass, "COM": ubj,
                              "Moments of Inertia": [0.01, 0.005, 0.01], "Products of Inertia": [0, 0, 0],
                              "Radius": 0.02, "Location Chassis Front": sh(uca_f),
                              "Location Chassis Back": sh(uca_b), "Location Upright": ubj},
        "Lower Control Arm": {"Mass": arm_mass, "COM": lbj,
                              "Moments of Inertia": [0.01, 0.005, 0.01], "Products of Inertia": [0, 0, 0],
                              "Radius": 0.03, "Location Chassis Front": sh(lca_f),
                              "Location Chassis Back": sh(lca_b), "Location Upright": lbj},
        "Tierod": {"Location Chassis": sh(trc), "Location Upright": tru},
        # Preload is in the curve (force at L0 = rest wheel load / motion ratio).
        "Spring": {"Location Chassis": spring_chassis, "Location Arm": spring_arm,
                   "Free Length": l_rest, "Spring Curve Data": spring_curve},
        "Shock": {"Location Chassis": spring_chassis, "Location Arm": spring_arm,
                  "Damping Curve Data": damp_curve},
        "Axle": {"Inertia": 0.01},   # was 0.4: +48% rear spin inertia over AC (sluggish wheelspin / clutch kicks)
        "_motion_ratio": mr,
        "_arb_point": arb_point, "_arb_mr": 0.6 * lbj_mr,
        "_rest": {"sag_m": sag, "camber_deg": camber, "toe_in_deg": toe_in},
    }


def tir(path, t, spec_t, radius, width, ac_t):
    """Pacejka 2002 (MF 5.2) tyre from AC's tyres.ini + published tyre data.

    Peak grip: mu(Fz) = DY0 + DY1*Fz[kN] (AC, linear load sensitivity) at FNOMIN = 3.5 kN, via the Drift Lab
    spec's mu (= AC at 3.5 kN). Peak slip angle / ratio from the spec (0.75 x FRICTION_LIMIT_ANGLE, calibrated
    against AC). Falloff to the sliding level "tail" (AC FALLOFF_LEVEL if the compound has one, else the spec's
    0.8): dry-asphalt sliding/peak friction is 0.7-0.9, and full sliding is isotropic, so Fx(kappa=1) and
    Fy(alpha~60 deg) share it (both C factors from the tail; Pacejka, Tire and Vehicle Dynamics 3rd ed. 4.3).
    Camber (Chrono's Pac02 computes the inclination since the DriftLab patch; TYDEX signs):
      - peak: AC's D *= 1 + DCAMBER_0*g + DCAMBER_1*g^2 (g = lean towards the force, rad) as MF terms:
        linear part = vertical shift PVY3 (adds on the side the tyre leans to, subtracts on the other),
        quadratic part = PDY3 (Chrono form 1 + PDY3 g^2). Limited to the published range for road /
        semi-slick tyres (linear gain <= 0.8/rad, optimum 3-4 deg -> +2-3% at the optimum, ~-11% at 5 deg
        leaning away: Milliken RCVD 2.7, TTC/TNO MF data): this car's front 2.3/-0.48 would be +18%/-19%
        at its -4.6 deg static camber.
      - camber thrust at zero slip = k * Ky * gamma with k = AC CAMBER_GAIN ("slip angle multiplier")
        limited to 0.02-0.045 (radial tyres: camber stiffness ~ 4-5% of cornering stiffness, e.g. the TNO
        MF 5.2 245/40R18 template: 0.018 Fz/deg); PHY3 is set so Ky*Shy + Svy gives that.
    Relaxation lengths from tyre literature (car tyres 0.3-0.6 m at nominal load, rising with load): AC's
    RELAXATION_LENGTH (0.05 m) floored at 0.1 m set up a front-axle oscillation at ~18 km/h. The Chrono patch
    shrinks them as the tyre saturates (secant stiffness), which gives the sliding response AC's short length
    was standing in for."""
    fnom = 3500.0
    mu_y = spec_t["mu_lat"]
    mu_x = spec_t["mu_long"]
    ac_mu_y = num(ac_t, "DY0", 1.2) + num(ac_t, "DY1", 0.0) * 3.5
    ac_mu_x = num(ac_t, "DX0", 1.2) + num(ac_t, "DX1", 0.0) * 3.5
    pdy2 = num(ac_t, "DY1", 0.0) * 3.5 * mu_y / ac_mu_y
    pdx2 = num(ac_t, "DX1", 0.0) * 3.5 * mu_x / ac_mu_x
    tail = num(ac_t, "FALLOFF_LEVEL", spec_t.get("tail", 0.8))
    tail = max(0.6, min(0.95, tail))
    c = 2.0 - (2.0 / math.pi) * math.asin(tail)          # sin(C*pi/2) = tail at large slip
    # Lateral: E = 0, B from the peak slip angle -> Ky/Fz ~ 0.4 /deg (performance tyre range 0.25-0.45).
    cy = c
    by = math.tan(math.pi / (2 * cy)) / math.radians(spec_t["peak_slip_angle_deg"])
    pky2 = 2.0
    ky_fz = by * cy * mu_y                                 # Ky / Fz at FNOMIN [1/rad]
    pky1 = -ky_fz / math.sin(2 * math.atan(1.0 / pky2))   # Chrono: Ky = PKY1*FNOMIN*sin(..)*LMUY (TYDEX: < 0)
    # Longitudinal: slip stiffness Kx/Fz = 23*mu (~28 /unit slip, MF data 20-30); E puts the peak at the spec's
    # slip ratio (x - E(x - atan x) = tan(pi/2C) at x = B*kappa_peak).
    cx = c
    kx_fz = 23.0 * mu_x
    bx = kx_fz / (cx * mu_x)
    xm = bx * spec_t["peak_slip_ratio"]
    ex = (xm - math.tan(math.pi / (2 * cx))) / (xm - math.atan(xm))
    ex = max(-5.0, min(0.9, ex))
    # Camber.
    d0 = max(0.0, min(num(ac_t, "DCAMBER_0", 1.2), 0.8))   # <= ~1.4 %/deg leaning into the force
    d1 = num(ac_t, "DCAMBER_1", -13.0)                       # optimum d0 / -2 d1 kept within 3-4 deg
    d1 = max(-d0 / (2 * math.radians(3.0)), min(d1, -d0 / (2 * math.radians(4.0))))
    k_thrust = max(0.02, min(num(ac_t, "CAMBER_GAIN", 0.04), 0.045))
    pvy3 = -mu_y * d0                                     # TYDEX sign: lean right (gamma > 0) -> Fy < 0
    phy3 = k_thrust - mu_y * d0 / ky_fz                    # Ky*PHY3*g + Svy = k*Ky*g at alpha = 0
    sig_a, sig_k = 0.50, 0.50                              # relaxation lengths at FNOMIN [m]
    vals = {
        "UNLOADED_RADIUS": radius, "WIDTH": width, "ASPECT_RATIO": 0.45, "RIM_RADIUS": radius - 0.09,
        "RIM_WIDTH": width - 0.02, "VERTICAL_STIFFNESS": num(ac_t, "RATE", 250000),
        "VERTICAL_DAMPING": num(ac_t, "DAMP", 500), "FNOMIN": fnom, "LFZO": 1.0,
        "PCY1": cy, "PDY1": mu_y, "PDY2": pdy2, "PDY3": d1, "PEY1": 0.0, "PEY2": 0.0, "PEY3": 0.0,
        "PEY4": 0.0, "PKY1": pky1, "PKY2": pky2, "PKY3": 0.0, "PHY1": 0.0, "PHY2": 0.0,
        "PHY3": phy3, "PVY1": 0.0, "PVY2": 0.0, "PVY3": pvy3, "PVY4": 0.0,
        "PCX1": cx, "PDX1": mu_x, "PDX2": pdx2, "PEX1": ex, "PEX2": 0.0, "PEX3": 0.0, "PEX4": 0.0,
        "PKX1": kx_fz, "PKX2": 0.0, "PKX3": 0.0, "PHX1": 0.0, "PHX2": 0.0, "PVX1": 0.0, "PVX2": 0.0,
        # sigma_alpha = PTY1 sin(2 atan(Fz/(PTY2 FNOMIN))) R0 (0.43 m at a 2.8 kN Miata front corner);
        # sigma_kappa = Fz/FNOMIN PTX1 R0.
        "PTY1": sig_a / (math.sin(2 * math.atan(0.5)) * radius), "PTY2": 2.0,
        "PTX1": sig_k / radius, "PTX2": 0.0, "PTX3": 0.0,
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
    return {"PKY1": pky1, "PCY1": cy, "PKX1": vals["PKX1"], "PEX1": ex, "PVY3": pvy3, "PHY3": phy3, "PDY3": d1}


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
        # Wheel load the spring carries at rest: everything but the unsprung corner (the chassis carries
        # one arm's worth of the two control arms).
        sprung = corner_load[side] - (hub - 1.5) * G
        s = wishbone(susp, side, half, radius, sp["spring_" + pre], sp["bump_" + pre], sp["rebound_" + pre],
                     hub, wheel_mass, tyre_mass, sprung)
        mr = s.pop("_motion_ratio")
        arb_point, arb_mr = s.pop("_arb_point"), s.pop("_arb_mr")
        rest = s.pop("_rest")
        write(os.path.join(d, "Susp_%s.json" % side), s)
        # AC ARB = N/m of left-right wheel-travel difference (F = k (zL - zR) on each wheel). Chrono
        # AntirollBarRSD: "Length" = half the bar's width (droplink lateral position), "Width" = arm length
        # (along +X from the bar), droplink "Height" from the arm tip up to the LCA point. Bar twist =
        # arb_mr (zL - zR) / arm, so k_rot = k_AC arm^2 / arb_mr^2.
        arb = sp["arb_" + pre]
        arm, drop = 0.2, 0.1
        write(os.path.join(d, "ARB_%s.json" % side), {
            "Name": "AC ARB", "Type": "Antirollbar", "Template": "AntirollBarRSD",
            "Arm": {"Mass": 1.0, "Inertia": [0.05, 0.05, 0.05], "Length": arb_point[1], "Width": arm, "Radius": 0.015},
            "Droplink": {"Height": drop},
            "RSD": {"Spring Coefficient": arb * arm * arm / (arb_mr * arb_mr), "Damping Coefficient": 50},
        })
        width = st.get("width_mm", 205) / 1000.0
        notes[side] = tir(os.path.join(d, "Tire_%s.tir" % side), tsec, st, radius, width, tsec)
        notes[side]["motion_ratio"] = mr
        notes[side]["arb_motion_ratio"] = arb_mr
        notes[side]["rest"] = rest
        inertia_total = num(tsec, "ANGULAR_INERTIA", 1.2)
        write(os.path.join(d, "Tire_%s.json" % side), {
            "Name": "AC Pac02 %s" % side, "Type": "Tire", "Template": "Pac02Tire", "Mass": tyre_mass,
            "Inertia": [0.4, inertia_total * 0.6, 0.4], "TIR Specification File": "%s/Tire_%s.tir" % (cid, side),
            "Coefficient of Friction": 0.8})
        write(os.path.join(d, "Wheel_%s.json" % side), {
            "Name": "AC wheel", "Type": "Wheel", "Template": "Wheel", "Mass": wheel_mass,
            "Inertia": [0.3, inertia_total * 0.4, 0.3]})
        brake = sp["brake_torque"] * (sp["brake_bias"] if side == "FRONT" else 1 - sp["brake_bias"]) * 0.5
        if side == "REAR":   # the rear brakes also do the handbrake's job (DLL scales the service share)
            brake = max(brake, sp.get("handbrake_torque", 0.0) / 2)
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
            # Bar centre: arm tips (bar + "Width" along +X) right under the droplink point on the LCA.
            "Antirollbar Location": [(a if side == "FRONT" else -b) + arb_point[0] - arm, 0, radius + arb_point[2] - drop],
        })
    axles[0]["Steering Index"] = 0

    lock_deg = math.degrees(sp["steer_lock"])
    sf = susp["FRONT"]
    steer_arm = math.hypot(vec(sf, "WBTYRE_STEER")[2], vec(sf, "WBTYRE_STEER")[0] - vec(sf, "WBTYRE_BOTTOM")[0])
    # Rack travel = steering arm * sin(lock) (a guess) unless AC gives LINEAR_STEER_ROD_RATIO: AC moves the
    # inner steering-rod point by that many metres per degree of (steering-wheel angle / STEER_RATIO), i.e.
    # per degree of the lock the DLL maps the input against, so the on-centre ratio and the real lock come
    # out of the geometry exactly as in AC (BDC Miata: 8.4:1 on centre, ~66 deg at 450 deg; not linear).
    car_ini = read_ini(os.path.join(data, "car.ini")) if os.path.exists(os.path.join(data, "car.ini")) else Ini()
    lsr = num(car_ini.get("CONTROLS", {}), "LINEAR_STEER_ROD_RATIO", 0.0)
    rack = lsr * lock_deg if lsr > 0 else steer_arm * math.sin(sp["steer_lock"])
    write(os.path.join(d, "Steering.json"), {
        "Name": "AC rack", "Type": "Steering", "Template": "RackPinion",
        "Steering Link": {"Mass": 2.0, "COM": 0, "Inertia": [0.1, 0.0001, 0.1], "Radius": 0.02, "Length": 0.6},
        "Pinion": {"Radius": 0.1, "Maximum Angle (deg)": math.degrees(rack / 0.1)},
        "AC Rack Travel": rack if lsr > 0 else 0.0,   # set: --calibrate only reports the lock
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
        # The DLL's limiter cuts fuel at limiter_rpm; Chrono's own max speed is only a clamp on the map
        # lookup (at the limiter it kept giving full torque - no limiter at all), so put it well above.
        "Maximal Engine Speed RPM": sp["limiter_rpm"] + 1500,
        "Map Full Throttle": [[-100, 0]] + [[r, t] for r, t in zip(rpm, nm)] + [[sp["limiter_rpm"] + 1500, nm[-1] * 0.5]],
        # Engine braking linear in rpm like AC's COAST_REF (0 at 0 rpm), continuing past the limiter.
        "Map Zero Throttle": [[-100, 0], [0, 0], [sp["limiter_rpm"], -sp["coast_nm"]],
                              [sp["limiter_rpm"] + 1500, -sp["coast_nm"] * (sp["limiter_rpm"] + 1500) / sp["limiter_rpm"]]],
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
                                        "limiter_rpm": sp["limiter_rpm"], "limiter_hz": sp.get("limiter_hz", 0.0),
                                        "idle_rpm": sp["idle_rpm"],
                                        "handbrake_nm": sp.get("handbrake_torque", 0.0) / 2,   # per rear wheel
                                        "rear_brake_nm": sp["brake_torque"] * (1 - sp["brake_bias"]) * 0.5,
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
        st = json.load(f)
    a0 = st["Pinion"]["Maximum Angle (deg)"]
    if st.get("AC Rack Travel", 0.0) > 0:   # AC's own rack ratio: keep it, the lock is what the geometry gives
        print("steering lock %.1f deg with AC's rack travel %.1f mm (spec lock %.1f)" % (
            measure(a0), st["AC Rack Travel"] * 1000, target_deg))
        return
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
