"""Compares Assetto Corsa and DriftGame telemetry (same CSV layout, AC signs).

    python compare_telemetry.py [file.csv ...]

No arguments: the newest ac_*.csv and dg_*.csv in Documents/DriftLab/telemetry.
Prints, per drift-angle band, how hard the wheel pulls toward countersteer, how
much countersteer the driver holds, yaw rate and speed - plus grip-cornering
weight and the parked feel - side by side for each file.

Signs (AC): drift = atan2(vel_x, vel_z), countersteer is steer opposite to drift,
final_ff > 0 pushes the rim left, so "pull into countersteer" = final_ff * sign(drift).
"""
import csv
import glob
import math
import os
import statistics as st
import sys

BANDS = [(0, 3), (3, 8), (8, 15), (15, 25), (25, 35), (35, 45), (45, 60), (60, 90)]


def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(line for line in f if not line.startswith("#")))
    out = []
    for r in rows:
        d = {k: float(v) for k, v in r.items() if v not in ("", None)}
        d["drift"] = math.degrees(math.atan2(d["vel_x"], d["vel_z"])) if d["speed_kmh"] > 3 else 0.0
        out.append(d)
    return out


def summarize(rows):
    moving = [r for r in rows if r["speed_kmh"] > 25 and abs(r["drift"]) < 90]
    s = {"samples": len(rows), "seconds": len(rows) / 100.0, "bands": []}
    for lo, hi in BANDS:
        b = [r for r in moving if lo <= abs(r["drift"]) < hi]
        if len(b) < 30:
            s["bands"].append(None)
            continue
        sg = lambda r: 1.0 if r["drift"] > 0 else -1.0
        s["bands"].append({
            "n": len(b),
            "pull": st.mean(r["final_ff"] * sg(r) for r in b),
            "counter": -st.mean(r["steer"] * sg(r) for r in b) * 450.0,   # wheel degrees of countersteer
            "yaw": st.mean(abs(r["yaw_rate"]) for r in b),
            "speed": st.mean(r["speed_kmh"] for r in b),
        })
    grip = [r for r in moving if abs(r["drift"]) < 3 and abs(r["steer"]) > 0.03]
    s["grip_slope"] = (sum(r["final_ff"] * r["steer"] for r in grip) / sum(r["steer"] ** 2 for r in grip)) if grip else None
    park = [r for r in rows if r["speed_kmh"] < 2]
    s["parked_ff"] = st.mean(abs(r["final_ff"]) for r in park) if park else None
    s["sideways_share"] = (sum(abs(r["drift"]) > 15 for r in moving) / len(moving)) if moving else 0.0
    return s


def main():
    files = sys.argv[1:]
    if not files:
        d = os.path.join(os.path.expanduser("~"), "Documents", "DriftLab", "telemetry")
        for prefix in ("ac_", "dg_"):
            found = sorted(glob.glob(os.path.join(d, prefix + "*.csv")), key=os.path.getmtime)
            if found:
                files.append(found[-1])
    if not files:
        print("No telemetry files found.")
        return
    sums = [(os.path.basename(f), summarize(load(f))) for f in files]
    w = 26
    print("".ljust(18) + "".join(n[:w - 2].ljust(w) for n, _ in sums))
    print("seconds".ljust(18) + "".join(("%.0f s, %.0f%% >15deg" % (s["seconds"], 100 * s["sideways_share"])).ljust(w) for _, s in sums))
    print("\nDrift band (deg)  wheel pull into countersteer | countersteer held (wheel deg) | yaw rate (rad/s) | km/h")
    for i, (lo, hi) in enumerate(BANDS):
        line = ("%d-%d" % (lo, hi)).ljust(18)
        for _, s in sums:
            b = s["bands"][i]
            line += ("%+.2f | %4.0f | %.2f | %3.0f" % (b["pull"], b["counter"], b["yaw"], b["speed"]) if b else "-").ljust(w)
        print(line)
    print("\ngrip: force per unit steer".ljust(18) + "".join(("%.2f" % s["grip_slope"] if s["grip_slope"] is not None else "-").ljust(w) for _, s in sums))
    print("parked |force|".ljust(18) + "".join(("%.2f" % s["parked_ff"] if s["parked_ff"] is not None else "-").ljust(w) for _, s in sums))


if __name__ == "__main__":
    main()
