"""Checks for the 7-DOF arm.

    .venv/bin/python tools/check_arm.py           # all checks
    .venv/bin/python tools/check_arm.py ranges    # only joint ranges
    .venv/bin/python tools/check_arm.py bends     # only the tube bends through the hinges (slow)
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
    # roll joints: the Bowden sockets open under the stator plate, inside the link below it;
    # the link bore there must take the tube (+ 0.5 mm)
    from arm import link as L
    from arm.roll import RollSpec
    names = [c[0] for c in A.CHAIN]
    for j in A.JOINTS:
        s = A.joint_spec(j)
        if not isinstance(s, RollSpec):
            continue
        prev = A.CHAIN[names.index(j) - 1]
        bore = P.FLANGES[prev[1][1]]["tube_r"] - L.WALL
        need = s.socket_r_max() + P.TUBE_OD / 2 + 0.5
        good = bore >= need
        print(f"  {j:3s}: socket exits at r {s.socket_r_max():.1f}, {prev[0]} bore r {bore:.1f}"
              f" (needs {need:.1f})  {'ok' if good else 'TUBE BLOCKED'}")
        ok &= good
    return ok


def check_bends():
    """Tubes passing through each hinge bend by the joint angle: bend radius >= the PTFE minimum."""
    sys.path.insert(0, str(ROOT / "tools"))
    import check_tube_bends as TB
    return all(TB.check_all().values())


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


def _balance_hits(q, statics, min_vol=0.5):
    """Balance parts (posed for q) against the arm (posed), the torso + cranks and each other."""
    arm = A.arm_solids(q)
    bal = A.balance_parts(q)
    hits = []
    for bn, bw in bal.items():
        cord = "cord" in bn
        for an, aw in arm.items():
            if cord and an == A.BALANCE_YOKE_ON:      # the cord starts on the yoke pin
                continue
            if bb_overlap(bw, aw) and (v := vol(bw, aw)) > min_vol:
                hits.append((bn, an, round(v, 1)))
        for sn, sw in statics.items():
            if bb_overlap(bw, sw) and (v := vol(bw, sw)) > min_vol:
                hits.append((bn, sn, round(v, 1)))
    skip = {("sheave", "cord"), ("cordback", "spring")}
    for (a, aw), (b, bw) in itertools.combinations(bal.items(), 2):
        ka, kb = a.split("_")[1], b.split("_")[1]
        if a[-1] != b[-1] or (ka, kb) in skip or (kb, ka) in skip:
            continue
        if bb_overlap(aw, bw) and (v := vol(aw, bw)) > min_vol:
            hits.append((a, b, round(v, 1)))
    return hits


def check_balance():
    """Shoulder balance hardware: yoke (on link_s1_s2), masts, sheaves, cords, springs,
    anchor posts vs the arm at the joint limits + combined poses, the beam, S1 base,
    crank racks and crank modules. Plus the yoke's clearance at full abduction."""
    from arm import balance as B
    from arm import torso as T
    print("== gravity balance hardware ==")
    statics = {"torso_beam": T.shoulder_beam(A.joint_spec("S1")), **A.rack_solids()}
    s1 = [(n, b) for n, _, b, _ in A.chain_frames({}) if n == "S1"][0][1]
    from arm import hinge as H
    statics["s1_base"] = A.place(H.base(A.joint_spec("S1")), s1)
    poses = [{}] + [{j: a} for j in ("S1", "S2") for a in (A.joint_spec(j).lo, A.joint_spec(j).hi)]
    poses += [{"S1": 60}, {"S1": 120, "S2": 100}, {"S1": -30, "S2": 100}, {"S1": 90, "S2": 100, "E": 115},
              {"S2": 100, "S3": 80, "E": 90}, {"S2": 100, "S3": -80, "E": 90}, {"S2": 85, "S3": -80, "E": 115},
              {"S1": 120, "S2": 60, "E": 90, "W1": 45}, {"S1": -30, "E": 115}]
    # documented reach limit: hand folded back over the shoulder (S2 > ~90, S3 < ~-70, E > ~105
    # together) touches the -X mast (the plain arm clears the beam there by ~37 mm)
    limits = [{"S2": 100, "S3": -80, "E": 115}]
    ok = True
    for q in poses + limits:
        h = _balance_hits(q, statics)
        if q in limits:
            print(f"  {q}: {'clear' if not h else 'KNOWN REACH LIMIT (hand vs mast, documented): ' + str(h)}")
            continue
        print(f"  {q}: {'clear' if not h else h}")
        ok &= not h
    # how close does the abducted upper arm come to the yoke tips?
    for q in ({"S2": 100}, {"S2": 100, "S3": 80, "E": 115}, {"S2": 100, "S3": -80, "E": 115}):
        arm = A.arm_solids(q)
        yoke = arm.pop(A.BALANCE_YOKE_ON).val()
        d = min(yoke.distance(w.val()) for n, w in arm.items()
                if A.CHAIN[[c[0] for c in A.CHAIN].index(owner(n))][0] not in ("S1", "link_s1_s2")
                and not n.startswith("s2_base"))
        print(f"  yoke clearance to the abducted arm at {q}: {d:.1f} mm")
        ok &= d > 2.0
    return ok


def check_gripper():
    """Gripper: finger/pad/palm interference over the whole opening, the cord and the
    opening spring run free, the open stops work, tool point = grasp centre, forces."""
    from arm import gripper as G
    from arm import kinematics as K
    from arm.cad import rod
    print("== gripper ==")
    ok = True
    good = abs(K.TOOL_OFFSET - G.GRASP_Z) < 1e-6
    print(f"  tool point {K.TOOL_OFFSET:.1f} mm past the W2 plate = grasp centre {G.GRASP_Z:.1f}  {'ok' if good else 'MISMATCH'}")
    ok &= good
    own = ({"gripper_finger_a", "gripper_pad_a"}, {"gripper_finger_b", "gripper_pad_b"})
    cord = rod((0, 0, G.BOSS_TOP + 0.3), (0, 0, G.ZP), P.CORD_D)
    yc = (G.SPRING[0] + G.FY) / 2            # spring centre plane
    for th in [0, 5, 10, 15, 20, 25, 30, 35, G.THETA_MAX]:
        p = G.parts(th)
        hits = [(a, b, round(v, 2)) for a, b in itertools.combinations(p, 2)
                if {a, b} not in own and (v := vol(p[a], p[b])) > 0.01]
        # the cord runs up the axis onto finger A's drum (touches it only at the tangent point)
        hits += [("cord", k, round(v, 2)) for k, w in p.items() if (v := vol(cord, w)) > 0.05]
        # the opening spring between the two posts (body without the hooks)
        L, F, _ = G.spring_state(th)
        x = L / 2 - G.POST_R - 2.5
        ang = math.radians(th)
        z = G.ZP + G.TAIL[0] * math.sin(ang) + G.TAIL[1] * math.cos(ang)
        spring = rod((-x, yc, z), (x, yc, z), G.SPRING_OD)
        hits += [("spring", k, round(v, 2)) for k, w in p.items() if (v := vol(spring, w)) > 0.01]
        print(f"  opening {th:4.1f} deg ({G.opening(th):4.1f} mm): {'clear' if not hits else hits}")
        ok &= not hits
    p = G.parts(G.THETA_MAX + 2)
    stop = vol(p["gripper_palm"], p["gripper_finger_a"]) > 0.05 and vol(p["gripper_palm"], p["gripper_finger_b"]) > 0.05
    print(f"  open stops on the palm end the travel at {G.THETA_MAX:.0f} deg: {'ok' if stop else 'NO STOP'}")
    ok &= stop
    w = G.opening(G.THETA_MAX)
    T10 = max(G.cord_for_force(10.0, th) for th in range(0, int(G.THETA_MAX) + 1))
    pull = min(2 * G.spring_state(th)[2] / G.R_C for th in range(0, int(G.THETA_MAX) + 1))
    print(f"  opening 0..{w:.0f} mm (need >= 60); cord for 10 N per pad <= {T10:.0f} N (need < 60);"
          f" spring pull-back on the cord >= {pull:.1f} N (need >= 1)")
    ok &= w >= 60 and T10 < 60 and pull >= 1.0
    return ok


if __name__ == "__main__":
    which = sys.argv[1:] or ["tubes", "bends", "tensions", "gripper", "ranges", "combos", "cranks", "balance"]
    res = {}
    for w in which:
        res[w] = {"ranges": check_ranges, "combos": check_combos, "cranks": check_cranks,
                  "tubes": check_tubes, "bends": check_bends, "tensions": check_tensions, "balance": check_balance,
                  "gripper": check_gripper}[w]()
    print("\nRESULT:", {k: ("PASS" if v else "FAIL") for k, v in res.items()})
