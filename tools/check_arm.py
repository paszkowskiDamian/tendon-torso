"""Checks for the 7-DOF arm.

    .venv/bin/python tools/check_arm.py           # all checks
    .venv/bin/python tools/check_arm.py ranges    # only joint ranges
"""
import itertools
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import cadquery as cq  # noqa: E402

from arm import assembly as A  # noqa: E402
from arm import crank as C  # noqa: E402
from arm import params as P  # noqa: E402

EL = [n for n, _, _ in A.CHAIN]


def vol(a, b):
    try:
        i = a.intersect(b)
        return i.val().Volume() if i.vals() else 0.0
    except Exception:
        return -1.0


def bb_overlap(a, b, tol=0.5):
    p, q = a.val().BoundingBox(), b.val().BoundingBox()
    return not (p.xmax < q.xmin - tol or q.xmax < p.xmin - tol or p.ymax < q.ymin - tol or
                q.ymax < p.ymin - tol or p.zmax < q.zmin - tol or q.zmax < p.zmin - tol)


def owner(part):
    """Which chain element a part belongs to."""
    for n, spec, _ in A.CHAIN:
        k = n.lower()
        if part == k or part.startswith(k + "_"):
            return n
    return None


def collisions(q, joint_idx=None, min_vol=1.0):
    """Pairs of parts from DIFFERENT, non-adjacent... (any) elements that overlap > min_vol."""
    solids = A.arm_solids(q)
    names = list(solids)
    hits = []
    for a, b in itertools.combinations(names, 2):
        ea, eb = owner(a), owner(b)
        if ea == eb and not (a.endswith("_rotor") or b.endswith("_rotor") or "rotor_" in a or "rotor_" in b):
            continue
        if not bb_overlap(solids[a], solids[b]):
            continue
        v = vol(solids[a], solids[b])
        if v > min_vol or v < 0:
            hits.append((a, b, round(v, 1)))
    return hits


def check_ranges():
    print("== joint ranges (each joint at its limits, others straight) ==")
    ok = True
    for j in A.JOINTS:
        s = A.joint_spec(j)
        for a in (s.lo, s.hi):
            h = collisions({j: a})
            print(f"  {j:3s} {a:+5.0f}: {'clear' if not h else h}")
            ok &= not h
    return ok


def check_combos():
    print("== combined poses ==")
    poses = [
        {"E": 115, "F": 85, "W1": 60},
        {"E": 115, "F": -85, "W1": -60, "W2": 30},
        {"S2": 100, "S3": 80, "E": 90},
        {"S2": 100, "S3": -80, "E": 115},
        {"S1": 120, "S2": 60, "E": 90, "W1": 45},
        {"S1": -30, "E": 115},
        {"S3": 80, "F": 85, "W1": 60, "W2": -30},
    ]
    ok = True
    for q in poses:
        h = collisions(q)
        print(f"  {q}: {'clear' if not h else h}")
        ok &= not h
    return ok


def check_cranks():
    print("== crank modules ==")
    parts = A.crank_parts()
    frames = A.crank_frames()
    js = list(frames)
    ok = True
    # neighbouring modules with the arm in a few positions
    for ang in (0, 45, 90, 135, 180, 270):
        posed = {}
        for j in js:
            m = frames[j]
            for k, w in parts.items():
                if k in ("crank_arm", "crank_knob"):
                    w = w.rotate((0, 0, 0), (1, 0, 0), ang)
                posed[(j, k)] = A.place(w, m)
        hits = []
        for (j1, k1), (j2, k2) in itertools.combinations(posed, 2):
            if j1 == j2:
                continue
            if bb_overlap(posed[(j1, k1)], posed[(j2, k2)]) and vol(posed[(j1, k1)], posed[(j2, k2)]) > 0.5:
                hits.append((j1, k1, j2, k2))
        print(f"  crank arm at {ang:3d}: {'clear' if not hits else hits}")
        ok &= not hits
    low = A.SHAFT_Z - (C.ARM_R + 7)
    print(f"  lowest point of the crank knob above the table: {low:.1f} mm (rack top at 5)")
    ok &= low > 6
    return ok


def check_tubes():
    print("== Bowden tube passages ==")
    ok = True
    for j in A.JOINTS:
        s = A.joint_spec(j)
        n = s.tubes_through
        if hasattr(s, "cav"):
            w, h = 2 * (s.cav - 1), 2 * s.slot_y
            cap = int(w // (P.TUBE_OD + 0.3)) * int(h // (P.TUBE_OD + 0.3))
            kind = f"slot {w:.0f} x {h:.0f} mm"
        else:
            d = 2 * s.r_in
            cap = {12: 7, 13: 8, 15: 10, 16: 12, 17: 12, 18: 14, 19: 16, 20: 19}.get(int(d), int((d / (P.TUBE_OD + 0.3)) ** 2 * 0.78))
            kind = f"bore {d:.0f} mm"
        print(f"  {j:3s}: {n:2d} tubes through, {kind}, fits ~{cap}  {'ok' if cap >= n else 'TOO SMALL'}")
        ok &= cap >= n
    return ok


def check_tensions():
    print("== tendon tension / crank effort (static, arm 2 kg, 0.5 kg payload) ==")
    # rough static torques with the arm horizontal
    g = 9.81
    torques = {"S1": 2.0 * g * 0.35 + 0.5 * g * 0.75, "S2": 2.0 * g * 0.30 + 0.5 * g * 0.70,
               "S3": 1.5, "E": 0.9 * g * 0.18 + 0.5 * g * 0.40, "F": 0.4, "W1": 0.3 * g * 0.08 + 0.5 * g * 0.15,
               "W2": 0.5 * g * 0.12}
    for j, t in torques.items():
        s = A.joint_spec(j)
        tension = t / (s.R / 1000)
        crank = tension * C.R_SPOOL / 1000
        print(f"  {j:3s}: joint {t:5.2f} N*m -> tendon {tension:5.0f} N, crank torque {crank:4.2f} N*m"
              f" ({crank / (C.ARM_R / 1000):4.1f} N on the knob), ratio {A.crank_ratio(j):.2f}:1")
    return True


if __name__ == "__main__":
    which = sys.argv[1:] or ["tubes", "tensions", "ranges", "combos", "cranks"]
    res = {}
    for w in which:
        res[w] = {"ranges": check_ranges, "combos": check_combos, "cranks": check_cranks,
                  "tubes": check_tubes, "tensions": check_tensions}[w]()
    print("\nRESULT:", {k: ("PASS" if v else "FAIL") for k, v in res.items()})
